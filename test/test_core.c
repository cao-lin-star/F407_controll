#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "crc16.h"
#include "kinematics.h"
#include "pid.h"
#include "protocol.h"

static int nearly_equal(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

static void test_crc_known_vector(void)
{
    static const uint8_t text[] = "123456789";
    assert(crc16_ccitt_false(text, sizeof(text) - 1U) == 0x29B1U);
}

static void test_protocol_fragment_and_sticky_frames(void)
{
    protocol_parser_t parser;
    protocol_frame_t parsed;
    uint8_t payload[8];
    uint8_t first[PROTOCOL_MAX_FRAME_SIZE];
    uint8_t second[PROTOCOL_MAX_FRAME_SIZE];
    size_t first_len;
    size_t second_len;
    size_t i;
    unsigned frames = 0U;

    protocol_write_f32_le(&payload[0], 0.25f);
    protocol_write_f32_le(&payload[4], -0.5f);
    first_len = protocol_build_frame(MSG_CMD_VEL, 0x1234U,
                                     payload, sizeof(payload),
                                     first, sizeof(first));
    second_len = protocol_build_frame(MSG_STOP, 0x1235U,
                                      NULL, 0U, second, sizeof(second));
    assert(first_len == 18U);
    assert(second_len == 10U);

    protocol_parser_init(&parser);
    /* 前置噪声 + 拆包：解析器必须忽略噪声并跨多次 feed 保持状态。 */
    assert(!protocol_parser_feed(&parser, 0x00U, &parsed));
    assert(!protocol_parser_feed(&parser, 0xAAU, &parsed));
    assert(!protocol_parser_feed(&parser, 0x7EU, &parsed));
    for (i = 0U; i < first_len; ++i) {
        if (protocol_parser_feed(&parser, first[i], &parsed)) {
            ++frames;
            assert(parsed.msg_type == MSG_CMD_VEL);
            assert(parsed.seq == 0x1234U);
            assert(parsed.payload_len == 8U);
            assert(nearly_equal(protocol_read_f32_le(&parsed.payload[0]),
                                0.25f, 1e-6f));
            assert(nearly_equal(protocol_read_f32_le(&parsed.payload[4]),
                                -0.5f, 1e-6f));
        }
    }
    /* 紧接第二帧，覆盖“粘包”连续字节场景。 */
    for (i = 0U; i < second_len; ++i) {
        if (protocol_parser_feed(&parser, second[i], &parsed)) {
            ++frames;
            assert(parsed.msg_type == MSG_STOP);
            assert(parsed.payload_len == 0U);
        }
    }
    assert(frames == 2U);
    assert(parser.stats.valid_frames == 2U);
}

static void test_protocol_crc_length_and_resync(void)
{
    protocol_parser_t parser;
    protocol_frame_t parsed;
    uint8_t good[PROTOCOL_MAX_FRAME_SIZE];
    uint8_t bad[PROTOCOL_MAX_FRAME_SIZE];
    uint8_t bad_length[] = {0xAAU, 0x55U, 0x01U, 0x01U,
                            0x00U, 0x00U, 0xFFU, 0x7FU};
    size_t length;
    size_t i;
    unsigned valid = 0U;

    length = protocol_build_frame(MSG_STOP, 7U, NULL, 0U,
                                  good, sizeof(good));
    memcpy(bad, good, length);
    bad[length - 1U] ^= 0x5AU;
    protocol_parser_init(&parser);

    for (i = 0U; i < length; ++i) {
        (void)protocol_parser_feed(&parser, bad[i], &parsed);
    }
    for (i = 0U; i < sizeof(bad_length); ++i) {
        (void)protocol_parser_feed(&parser, bad_length[i], &parsed);
    }
    for (i = 0U; i < length; ++i) {
        if (protocol_parser_feed(&parser, good[i], &parsed)) {
            ++valid;
        }
    }
    assert(valid == 1U);
    assert(parser.stats.crc_errors == 1U);
    assert(parser.stats.length_errors == 1U);
}

static void test_kinematics_and_odometry(void)
{
    float left;
    float right;
    float linear;
    float angular;
    odometry_t odom = {0};

    kinematics_inverse(1.0f, 1.0f, 0.330f, &left, &right);
    assert(nearly_equal(left, 0.835f, 1e-6f));
    assert(nearly_equal(right, 1.165f, 1e-6f));
    kinematics_forward(left, right, 0.330f, &linear, &angular);
    assert(nearly_equal(linear, 1.0f, 1e-6f));
    assert(nearly_equal(angular, 1.0f, 1e-5f));

    kinematics_integrate(&odom, 0.1f, 0.1f, 0.1f, 0.330f);
    assert(nearly_equal(odom.x_m, 0.1f, 1e-6f));
    assert(nearly_equal(odom.y_m, 0.0f, 1e-6f));
    assert(nearly_equal(odom.linear_mps, 1.0f, 1e-6f));
}

static void test_pid_limit_anti_windup_and_reset(void)
{
    pid_controller_t pid;
    float output;
    unsigned i;

    pid_init(&pid, 0.625f, 0.125f, 0.0f, 2500.0f, -100.0f, 100.0f);
    pid_set_target(&pid, 100.0f);
    output = pid_update(&pid, 0.0f);
    assert(nearly_equal(output, 75.0f, 1e-6f));
    for (i = 0U; i < 100U; ++i) {
        output = pid_update(&pid, 0.0f);
        assert(output <= 100.0f);
    }
    assert(pid.integral < pid.integral_limit);
    (void)pid_update(&pid, 200.0f);
    assert(pid.integral < 300.0f);
    pid_reset(&pid);
    assert(pid.target == 0.0f);
    assert(pid.integral == 0.0f);
    assert(pid.previous_error == 0.0f);
}

int main(void)
{
    test_crc_known_vector();
    test_protocol_fragment_and_sticky_frames();
    test_protocol_crc_length_and_resync();
    test_kinematics_and_odometry();
    test_pid_limit_anti_windup_and_reset();
    puts("All C core tests passed.");
    return 0;
}
