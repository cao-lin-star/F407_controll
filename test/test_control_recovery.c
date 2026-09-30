#include <assert.h>
#include <math.h>
#include "control.h"
void board_motor_stop(void) {}
void board_motor_set_percent(float l,float r) {(void)l;(void)r;}
int main(void) {
 chassis_control_t c;
 control_init(&c,100);
 control_raise_stop_fault(&c,FAULT_OBSTACLE);
 assert(!control_accept_cmd_vel(&c,.1f,0,101));
 assert(c.motion_recovery_required);
 assert(c.fault_flags==FAULT_OBSTACLE);
 control_clear_fault(&c,FAULT_OBSTACLE);
 assert(!control_accept_cmd_vel(&c,.1f,0,102));
 assert(!(c.fault_flags & FAULT_INVALID_COMMAND));
 assert(control_accept_cmd_vel(&c,0,0,103));
 control_update(&c,0,0,110);
 assert(!c.motion_recovery_required);
 assert(control_accept_cmd_vel(&c,.1f,0,111));
 assert(!control_accept_cmd_vel(&c,NAN,0,112));
 assert(c.fault_flags & FAULT_INVALID_COMMAND);
 control_init(&c,200);
 control_raise_stop_fault(&c,FAULT_ESTOP);
 assert(!control_accept_debug_pwm(&c,1,1,201));
 assert(c.fault_flags==FAULT_ESTOP);
 assert(control_accept_cmd_vel(&c,0,0,202));
 control_update(&c,0,0,210);
 assert(c.motion_recovery_required); // A zero cannot bypass the actual stop cause.
 control_init(&c,1000);
 control_raise_stop_fault(&c,FAULT_LEFT_ENCODER);
 assert(control_accept_cmd_vel(&c,0,0,1001));
 for(unsigned t=1010;t<=3010;t+=10) {
   control_accept_cmd_vel(&c,0,0,t);
   control_update(&c,0,0,t);
 }
 assert(!(c.fault_flags & FAULT_LEFT_ENCODER));
 assert(!c.motion_recovery_required);
 return 0;
}
