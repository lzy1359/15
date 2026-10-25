#ifndef __MOTOR_CONTROL_CLIENT_H__
#define __MOTOR_CONTROL_CLIENT_H__

#include <stdbool.h>

#include "motor_control_protocol.h"

int motor_control_get_state(struct motor_control_state *state);
int motor_control_reset_all(struct motor_control_state *state);
int motor_control_sync_all(struct motor_control_state *state);
int motor_control_set_position(int axis, int angle,
                               struct motor_control_state *state);
int motor_control_move_relative(int axis, int angle,
                                struct motor_control_state *state);
int motor_control_run_test(struct motor_control_state *state);
int motor_control_start_demo(bool forward, struct motor_control_state *state);
int motor_control_stop_motion(struct motor_control_state *state);

#endif
