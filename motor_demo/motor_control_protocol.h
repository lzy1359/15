#ifndef __MOTOR_CONTROL_PROTOCOL_H__
#define __MOTOR_CONTROL_PROTOCOL_H__

#include <stdint.h>

#define MOTOR_CONTROL_SOCKET_PATH "/run/motor-stack/motor_control.sock"
#define MOTOR_CONTROL_AXIS_COUNT  1
#define MOTOR_CONTROL_CYCLIC_FREQUENCY_HZ 1000
#define MOTOR_CONTROL_ANGLE_FULL_TURN_DEG 360
/*
 * DS2-E 17-bit magnetic encoder configuration from the user manual:
 *   - encoder resolution: 131072 counts per motor revolution
 *   - 0x6091 electronic gear: 131072 / 10000
 *
 * The gear maps one mechanical motor revolution to 10000 position pulses,
 * which are the units used by 0x6064 feedback and 0x607A target position.
 */
#define MOTOR_CONTROL_ENCODER_COUNTS_PER_MOTOR_REV 131072
#define MOTOR_CONTROL_ELECTRONIC_GEAR_NUMERATOR    131072
#define MOTOR_CONTROL_ELECTRONIC_GEAR_DENOMINATOR  10000
#define MOTOR_CONTROL_COUNTS_PER_REV \
    ((MOTOR_CONTROL_ENCODER_COUNTS_PER_MOTOR_REV * \
      MOTOR_CONTROL_ELECTRONIC_GEAR_DENOMINATOR) / \
     MOTOR_CONTROL_ELECTRONIC_GEAR_NUMERATOR)

enum motor_control_command_id
{
    MOTOR_CONTROL_CMD_GET_STATE = 1,
    MOTOR_CONTROL_CMD_RESET_ALL,
    MOTOR_CONTROL_CMD_SYNC_ALL,
    MOTOR_CONTROL_CMD_SET_POSITION,
    MOTOR_CONTROL_CMD_MOVE_RELATIVE,
    MOTOR_CONTROL_CMD_RUN_TEST,
    MOTOR_CONTROL_CMD_START_DEMO_FORWARD,
    MOTOR_CONTROL_CMD_START_DEMO_REVERSE,
    MOTOR_CONTROL_CMD_STOP_MOTION,
};

enum motor_control_motion_mode
{
    MOTOR_CONTROL_MOTION_IDLE = 0,
    MOTOR_CONTROL_MOTION_TEST_TO_ZERO,
    MOTOR_CONTROL_MOTION_TEST_TO_TEN,
    MOTOR_CONTROL_MOTION_TEST_RETURN_ZERO,
    MOTOR_CONTROL_MOTION_DEMO_FORWARD,
    MOTOR_CONTROL_MOTION_DEMO_REVERSE,
};

enum motor_control_status_code
{
    MOTOR_CONTROL_STATUS_OK = 0,
    MOTOR_CONTROL_STATUS_ERROR = -1,
    MOTOR_CONTROL_STATUS_UNAVAILABLE = -2,
    MOTOR_CONTROL_STATUS_INVALID = -3,
    MOTOR_CONTROL_STATUS_BUSY = -4,
};

struct motor_control_request
{
    int32_t command;
    int32_t axis;
    int32_t value;
};

struct motor_control_state
{
    int32_t ready;
    int32_t motion_mode;
    int32_t motion_result;
    int32_t axis_online[MOTOR_CONTROL_AXIS_COUNT];
    int32_t axis_position[MOTOR_CONTROL_AXIS_COUNT];
    int32_t axis_target[MOTOR_CONTROL_AXIS_COUNT];
    uint32_t axis_status_word[MOTOR_CONTROL_AXIS_COUNT];
    int32_t axis_opmode[MOTOR_CONTROL_AXIS_COUNT];
    int32_t axis_al_state[MOTOR_CONTROL_AXIS_COUNT];
    uint32_t axis_working_counter[MOTOR_CONTROL_AXIS_COUNT];
    int32_t axis_wc_state[MOTOR_CONTROL_AXIS_COUNT];
    int32_t axis_servo_on[MOTOR_CONTROL_AXIS_COUNT];
    int32_t axis_in_position[MOTOR_CONTROL_AXIS_COUNT];
    int32_t axis_fault[MOTOR_CONTROL_AXIS_COUNT];
    uint32_t jitter_us;
};

struct motor_control_response
{
    int32_t status;
    struct motor_control_state state;
};

#endif
