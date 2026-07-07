/**
 * @file protocol.h
 * @brief CAN frame encode/decode for Piper V2 protocol
 */
#ifndef CPIPER_PROTOCOL_H
#define CPIPER_PROTOCOL_H

#include "messages.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Decode a raw CAN frame into a cpiper_message_t.
 * @param can_id   Arbitration ID of the received frame
 * @param data     8-byte CAN data payload
 * @param msg      Output message (type_ is set based on can_id)
 * @param timestamp  Frame timestamp in seconds
 * @return true if can_id was recognized, false otherwise
 */
bool cpiper_decode(uint32_t can_id, const uint8_t data[8],
                   cpiper_message_t *msg, double timestamp);

/**
 * Encode a cpiper_message_t into a CAN frame payload.
 * @param msg    Input message (type_ determines which fields to use)
 * @param can_id Output: arbitration ID to send on
 * @param data   Output: 8-byte CAN data payload
 * @return true if msg type was recognized and encoded
 */
bool cpiper_encode(const cpiper_message_t *msg, uint32_t *can_id,
                   uint8_t data[8]);

/* ─── Byte helper functions (inline for speed) ───────────────── */

static inline uint8_t cpiper_get_u8(const uint8_t *data, int idx) {
    return data[idx];
}

static inline int8_t cpiper_get_i8(const uint8_t *data, int idx) {
    return (int8_t)data[idx];
}

static inline uint16_t cpiper_get_u16_be(const uint8_t *data, int idx) {
    return (uint16_t)((uint16_t)data[idx] << 8 | (uint16_t)data[idx + 1]);
}

static inline int16_t cpiper_get_i16_be(const uint8_t *data, int idx) {
    uint16_t raw = cpiper_get_u16_be(data, idx);
    return (int16_t)raw;
}

static inline uint32_t cpiper_get_u32_be(const uint8_t *data, int idx) {
    return (uint32_t)((uint32_t)data[idx] << 24 |
                      (uint32_t)data[idx + 1] << 16 |
                      (uint32_t)data[idx + 2] << 8 |
                      (uint32_t)data[idx + 3]);
}

static inline int32_t cpiper_get_i32_be(const uint8_t *data, int idx) {
    uint32_t raw = cpiper_get_u32_be(data, idx);
    return (int32_t)raw;
}

static inline void cpiper_put_u8(uint8_t *data, int idx, uint8_t val) {
    data[idx] = val;
}

static inline void cpiper_put_i8(uint8_t *data, int idx, int8_t val) {
    data[idx] = (uint8_t)val;
}

static inline void cpiper_put_u16_be(uint8_t *data, int idx, uint16_t val) {
    data[idx]     = (uint8_t)(val >> 8);
    data[idx + 1] = (uint8_t)(val & 0xFF);
}

static inline void cpiper_put_i16_be(uint8_t *data, int idx, int16_t val) {
    cpiper_put_u16_be(data, idx, (uint16_t)val);
}

static inline void cpiper_put_u32_be(uint8_t *data, int idx, uint32_t val) {
    data[idx]     = (uint8_t)((val >> 24) & 0xFF);
    data[idx + 1] = (uint8_t)((val >> 16) & 0xFF);
    data[idx + 2] = (uint8_t)((val >> 8) & 0xFF);
    data[idx + 3] = (uint8_t)(val & 0xFF);
}

static inline void cpiper_put_i32_be(uint8_t *data, int idx, int32_t val) {
    cpiper_put_u32_be(data, idx, (uint32_t)val);
}

/**
 * MIT mode: Convert float to unsigned integer for CAN transmission.
 */
uint32_t cpiper_float_to_uint(float x_float, float x_min, float x_max, int bits);

#ifdef __cplusplus
}
#endif

#endif /* CPIPER_PROTOCOL_H */
