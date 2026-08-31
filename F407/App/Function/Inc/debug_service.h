#ifndef DEBUG_SERVICE_H
#define DEBUG_SERVICE_H

#include <stdint.h>

#include "control.h"

void debug_service_init(uint32_t now_ms);
void debug_service_process(chassis_control_t *chassis, uint32_t now_ms);

#endif /* DEBUG_SERVICE_H */
