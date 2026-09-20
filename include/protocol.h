#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PROTOCOL_SOF1                 0xAAU
#define PROTOCOL_SOF2                 0x55U
#define PROTOCOL_VERSION              0x01U
#define PROTOCOL_HEADER_SIZE              6U
#define PROTOCOL_MAX_PAYLOAD             64U
#define PROTOCOL_MAX_FRAME_SIZE \
    (2U + PROTOCOL_HEADER_SIZE + PROTOCOL_MAX_PAYLOAD + 2U)

typedef enum {
    MSG_CMD_VEL      = 0x01,
    MSG_ODOM         = 0x02,
    MSG_HEARTBEAT    = 0x03,
    MSG_STOP         = 0x04,
    MSG_RANGE_STATUS = 0x05,
    MSG_IMU_RAW      = 0x06,
    MSG_CONTROL_SOURCE = 0x07, /* one byte: NONE/RK/DEBUG/PS2 */
    MSG_DEBUG_STATUS = 0x7F
} protocol_msg_type_t;

typedef struct {
    uint8_t msg_type;
    uint16_t seq;
    uint16_t payload_len;
    uint8_t payload[PROTOCOL_MAX_PAYLOAD];
} protocol_frame_t;

typedef struct {
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t version_errors;
    uint32_t discarded_bytes;
} protocol_parser_stats_t;

typedef struct {
    uint8_t buffer[PROTOCOL_MAX_FRAME_SIZE];
    uint16_t count;
    uint16_t expected_size;
    protocol_parser_stats_t stats;
} protocol_parser_t;

void protocol_parser_init(protocol_parser_t *parser);
bool protocol_parser_feed(protocol_parser_t *parser,
                          uint8_t byte,
                          protocol_frame_t *out_frame);
size_t protocol_build_frame(uint8_t msg_type,
                            uint16_t seq,
                            const uint8_t *payload,
                            uint16_t payload_len,
                            uint8_t *out,
                            size_t out_capacity);

uint16_t protocol_read_u16_le(const uint8_t *p);
uint32_t protocol_read_u32_le(const uint8_t *p);
int32_t protocol_read_i32_le(const uint8_t *p);
float protocol_read_f32_le(const uint8_t *p);
void protocol_write_u16_le(uint8_t *p, uint16_t value);
void protocol_write_u32_le(uint8_t *p, uint32_t value);
void protocol_write_i32_le(uint8_t *p, int32_t value);
void protocol_write_f32_le(uint8_t *p, float value);

#endif /* PROTOCOL_H */
