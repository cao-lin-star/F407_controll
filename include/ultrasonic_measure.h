#ifndef ULTRASONIC_MEASURE_H
#define ULTRASONIC_MEASURE_H
#include <stdint.h>
/* TIM4 wraps at 65536us; CS100A no-target ECHO is about 66ms. */
static inline float ultrasonic_pulse_distance(uint16_t pulse_us, uint32_t elapsed_ms)
{
    return elapsed_ms >= 60U ? 5.0f : (float)pulse_us * 0.0001715f;
}
#endif
