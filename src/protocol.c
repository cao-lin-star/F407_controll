#include "protocol.h"

#include <string.h>

#include "crc16.h"

typedef char protocol_float_must_be_32_bits[(sizeof(float) == 4U) ? 1 : -1];

uint16_t protocol_read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8U);
}

uint32_t protocol_read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8U)
         | ((uint32_t)p[2] << 16U)
         | ((uint32_t)p[3] << 24U);
}

int32_t protocol_read_i32_le(const uint8_t *p)
{
    return (int32_t)protocol_read_u32_le(p);
}

float protocol_read_f32_le(const uint8_t *p)
{
    const uint32_t bits = protocol_read_u32_le(p);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

void protocol_write_u16_le(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

void protocol_write_u32_le(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8U) & 0xFFU);
    p[2] = (uint8_t)((value >> 16U) & 0xFFU);
    p[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

void protocol_write_i32_le(uint8_t *p, int32_t value)
{
    protocol_write_u32_le(p, (uint32_t)value);
}

void protocol_write_f32_le(uint8_t *p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    protocol_write_u32_le(p, bits);
}

void protocol_parser_init(protocol_parser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
}

/*
 * 淇濈暀缂撳瓨涓渶鏃╃殑涓嬩竴缁?AA 55锛涜嫢娌℃湁锛屽垯鍙繚鐣欐湯灏惧崟鐙殑 AA銆? * 杩欐牱閿欒甯х殑 payload 鍐呮伆濂藉嚭鐜颁笅涓€甯?SOF 鏃朵篃鑳藉敖蹇仮澶嶃€? */
static void parser_resync(protocol_parser_t *parser)
{
    uint16_t i;

    for (i = 1U; (uint16_t)(i + 1U) < parser->count; ++i) {
        if (parser->buffer[i] == PROTOCOL_SOF1
            && parser->buffer[i + 1U] == PROTOCOL_SOF2) {
            const uint16_t remaining = (uint16_t)(parser->count - i);
            memmove(parser->buffer, &parser->buffer[i], remaining);
            parser->stats.discarded_bytes += i;
            parser->count = remaining;
            parser->expected_size = 0U;
            return;
        }
    }

    if (parser->count > 0U
        && parser->buffer[parser->count - 1U] == PROTOCOL_SOF1) {
        parser->stats.discarded_bytes += (uint32_t)(parser->count - 1U);
        parser->buffer[0] = PROTOCOL_SOF1;
        parser->count = 1U;
    } else {
        parser->stats.discarded_bytes += parser->count;
        parser->count = 0U;
    }
    parser->expected_size = 0U;
}

bool protocol_parser_feed(protocol_parser_t *parser,
                          uint8_t byte,
                          protocol_frame_t *out_frame)
{
    if (parser->count == 0U) {
        if (byte != PROTOCOL_SOF1) {
            ++parser->stats.discarded_bytes;
            return false;
        }
        parser->buffer[parser->count++] = byte;
        return false;
    }

    if (parser->count == 1U) {
        if (byte == PROTOCOL_SOF2) {
            parser->buffer[parser->count++] = byte;
        } else if (byte == PROTOCOL_SOF1) {
            /* AA AA 55锛氱浜屼釜 AA 鍙兘鎵嶆槸鐪熸甯уご銆?*/
            ++parser->stats.discarded_bytes;
        } else {
            parser->stats.discarded_bytes += 2U;
            parser->count = 0U;
        }
        return false;
    }

    if (parser->count >= PROTOCOL_MAX_FRAME_SIZE) {
        ++parser->stats.length_errors;
        parser_resync(parser);
    }
    if (parser->count >= PROTOCOL_MAX_FRAME_SIZE) {
        return false;
    }
    parser->buffer[parser->count++] = byte;

    for (;;) {
        uint16_t payload_len;
        uint16_t received_crc;
        uint16_t calculated_crc;

        if (parser->count < (2U + PROTOCOL_HEADER_SIZE)) {
            return false;
        }

        if (parser->expected_size == 0U) {
            if (parser->buffer[2] != PROTOCOL_VERSION) {
                ++parser->stats.version_errors;
                parser_resync(parser);
                if (parser->count < (2U + PROTOCOL_HEADER_SIZE)) {
                    return false;
                }
                continue;
            }

            payload_len = protocol_read_u16_le(&parser->buffer[6]);
            if (payload_len > PROTOCOL_MAX_PAYLOAD) {
                ++parser->stats.length_errors;
                parser_resync(parser);
                if (parser->count < (2U + PROTOCOL_HEADER_SIZE)) {
                    return false;
                }
                continue;
            }
            parser->expected_size = (uint16_t)(2U + PROTOCOL_HEADER_SIZE
                                               + payload_len + 2U);
        }

        if (parser->count < parser->expected_size) {
            return false;
        }

        payload_len = protocol_read_u16_le(&parser->buffer[6]);
        received_crc = protocol_read_u16_le(
            &parser->buffer[2U + PROTOCOL_HEADER_SIZE + payload_len]);
        calculated_crc = crc16_ccitt_false(&parser->buffer[2],
                                            PROTOCOL_HEADER_SIZE + payload_len);
        if (received_crc != calculated_crc) {
            ++parser->stats.crc_errors;
            parser_resync(parser);
            if (parser->count < (2U + PROTOCOL_HEADER_SIZE)) {
                return false;
            }
            continue;
        }

        out_frame->msg_type = parser->buffer[3];
        out_frame->seq = protocol_read_u16_le(&parser->buffer[4]);
        out_frame->payload_len = payload_len;
        if (payload_len > 0U) {
            memcpy(out_frame->payload, &parser->buffer[8], payload_len);
        }
        ++parser->stats.valid_frames;
        parser->count = 0U;
        parser->expected_size = 0U;
        return true;
    }
}

size_t protocol_build_frame(uint8_t msg_type,
                            uint16_t seq,
                            const uint8_t *payload,
                            uint16_t payload_len,
                            uint8_t *out,
                            size_t out_capacity)
{
    const size_t total = 2U + PROTOCOL_HEADER_SIZE + payload_len + 2U;
    uint16_t crc;

    if (out == NULL || payload_len > PROTOCOL_MAX_PAYLOAD
        || total > out_capacity || (payload_len > 0U && payload == NULL)) {
        return 0U;
    }

    out[0] = PROTOCOL_SOF1;
    out[1] = PROTOCOL_SOF2;
    out[2] = PROTOCOL_VERSION;
    out[3] = msg_type;
    protocol_write_u16_le(&out[4], seq);
    protocol_write_u16_le(&out[6], payload_len);
    if (payload_len > 0U) {
        memcpy(&out[8], payload, payload_len);
    }
    crc = crc16_ccitt_false(&out[2], PROTOCOL_HEADER_SIZE + payload_len);
    protocol_write_u16_le(&out[8U + payload_len], crc);
    return total;
}


