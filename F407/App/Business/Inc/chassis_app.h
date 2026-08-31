#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

#include <stdint.h>

void chassis_app_init(void);
void chassis_app_control_process(uint32_t now_ms);
void chassis_app_sensor_process(uint32_t now_ms);
void chassis_app_telemetry_process(uint32_t now_ms);
void chassis_app_housekeeping_process(uint32_t now_ms);

#endif /* CHASSIS_APP_H */
