#include <assert.h>
#include <math.h>
#include "ultrasonic_measure.h"
int main(void) {
    assert(fabsf(ultrasonic_pulse_distance(2915,3)-0.4999225f)<0.00001f);
    assert(ultrasonic_pulse_distance((uint16_t)66000U,66)>4.0f);
    assert(ultrasonic_pulse_distance(0,0)<0.02f);
    assert(ultrasonic_pulse_distance(24000,24)>4.0f);
    assert(ultrasonic_pulse_distance((uint16_t)(2000U-65000U),3)>0.4f);
    return 0;
}
