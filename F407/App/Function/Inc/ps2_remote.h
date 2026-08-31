#ifndef PS2_REMOTE_H
#define PS2_REMOTE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PS2_REMOTE_ACTION_NONE = 0,
    PS2_REMOTE_ACTION_STOP,
    PS2_REMOTE_ACTION_VELOCITY,
    PS2_REMOTE_ACTION_RELEASE
} ps2_remote_action_t;

typedef struct {
    uint32_t valid_frames;
    uint32_t invalid_frames;
    uint32_t last_valid_ms;
    float linear_mps;
    float angular_rps;
    uint8_t mode_id;
    uint8_t btn1_raw;
    uint8_t btn2_raw;
    uint8_t right_x;
    uint8_t right_y;
    uint8_t left_x;
    uint8_t left_y;
    uint8_t connected;
    uint8_t manual_mode;
    uint8_t deadman_held;
    uint8_t stop_latched;
} ps2_remote_status_t;

void ps2_remote_init(uint32_t now_ms);
ps2_remote_action_t ps2_remote_process(uint32_t now_ms,
                                       float *linear_mps,
                                       float *angular_rps);
bool ps2_remote_manual_active(void);
void ps2_remote_force_stop(void);
void ps2_remote_get_status(ps2_remote_status_t *out);

#endif /* PS2_REMOTE_H */
