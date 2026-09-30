#ifndef SIDE_ULTRASONIC_GUARD_H
#define SIDE_ULTRASONIC_GUARD_H
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "board_config.h"

/* Probe-to-obstacle distance, NOT clearance from the body. No swept cone.
 * Invalid/stale sensing remains a fault; valid echoes > 12 cm never block.
 * Straight reverse escape retains the existing rear-safety policy. */
static inline bool side_ultrasonic_blocked(float distance, uint8_t status,
    uint32_t age_ms, bool left, float left_mps, float right_mps)
{
    (void)left;
    const float v = (left_mps + right_mps) * 0.5f;
    const float w = (right_mps - left_mps) / WHEEL_TRACK_M;
    if (fabsf(v) <= 0.001f && fabsf(w) <= 0.01f) return false;
    if (v < 0.0f && fabsf(w) < 0.05f) return false;
    if (age_ms > SIDE_ULTRASONIC_TIMEOUT_MS || status == 0U || status == 3U)
        return true;
    if (status == 2U) return false;
    if (status != 1U || !isfinite(distance) || distance < 0.02f) return true;
    return distance <= SIDE_ULTRASONIC_STOP_DISTANCE_M;
}
#endif
