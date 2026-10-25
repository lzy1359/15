#include "motor_control_client.h"

#include <sys/time.h>
#include <stddef.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define MOTOR_CONTROL_SOCKET_TIMEOUT_MS  80

static void motor_control_zero_state(struct motor_control_state *state)
{
    if (!state)
        return;

    memset(state, 0, sizeof(*state));
}

static ssize_t read_full(int fd, void *buf, size_t size)
{
    size_t done = 0;

    while (done < size)
    {
        ssize_t ret = read(fd, (char *)buf + done, size - done);
        if (ret <= 0)
            return ret;
        done += ret;
    }

    return (ssize_t)done;
}

static ssize_t write_full(int fd, const void *buf, size_t size)
{
    size_t done = 0;

    while (done < size)
    {
        ssize_t ret = write(fd, (const char *)buf + done, size - done);
        if (ret <= 0)
            return ret;
        done += ret;
    }

    return (ssize_t)done;
}

static int motor_control_exchange(const struct motor_control_request *request,
                                  struct motor_control_state *state)
{
    int fd;
    int status;
    struct sockaddr_un addr;
    struct motor_control_response response;
    struct timeval timeout;

    motor_control_zero_state(state);
    memset(&addr, 0, sizeof(addr));
    memset(&response, 0, sizeof(response));
    timeout.tv_sec = 0;
    timeout.tv_usec = MOTOR_CONTROL_SOCKET_TIMEOUT_MS * 1000;

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0)
        return MOTOR_CONTROL_STATUS_UNAVAILABLE;

    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, MOTOR_CONTROL_SOCKET_PATH,
            sizeof(addr.sun_path) - 1);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(fd);
        return MOTOR_CONTROL_STATUS_UNAVAILABLE;
    }

    if (write_full(fd, request, sizeof(*request)) != sizeof(*request))
    {
        close(fd);
        return MOTOR_CONTROL_STATUS_UNAVAILABLE;
    }

    if (read_full(fd, &response, sizeof(response)) != sizeof(response))
    {
        close(fd);
        return MOTOR_CONTROL_STATUS_UNAVAILABLE;
    }

    close(fd);

    if (state)
        *state = response.state;

    status = response.status;
    return status;
}

int motor_control_get_state(struct motor_control_state *state)
{
    const struct motor_control_request request = {
        .command = MOTOR_CONTROL_CMD_GET_STATE,
        .axis = -1,
        .value = 0,
    };

    return motor_control_exchange(&request, state);
}

int motor_control_reset_all(struct motor_control_state *state)
{
    const struct motor_control_request request = {
        .command = MOTOR_CONTROL_CMD_RESET_ALL,
        .axis = -1,
        .value = 0,
    };

    return motor_control_exchange(&request, state);
}

int motor_control_sync_all(struct motor_control_state *state)
{
    const struct motor_control_request request = {
        .command = MOTOR_CONTROL_CMD_SYNC_ALL,
        .axis = -1,
        .value = 0,
    };

    return motor_control_exchange(&request, state);
}

int motor_control_set_position(int axis, int angle,
                               struct motor_control_state *state)
{
    const struct motor_control_request request = {
        .command = MOTOR_CONTROL_CMD_SET_POSITION,
        .axis = axis,
        .value = angle,
    };

    return motor_control_exchange(&request, state);
}

int motor_control_move_relative(int axis, int angle,
                                struct motor_control_state *state)
{
    const struct motor_control_request request = {
        .command = MOTOR_CONTROL_CMD_MOVE_RELATIVE,
        .axis = axis,
        .value = angle,
    };

    return motor_control_exchange(&request, state);
}

int motor_control_run_test(struct motor_control_state *state)
{
    const struct motor_control_request request = {
        .command = MOTOR_CONTROL_CMD_RUN_TEST,
        .axis = 0,
        .value = 0,
    };

    return motor_control_exchange(&request, state);
}

int motor_control_start_demo(bool forward, struct motor_control_state *state)
{
    const struct motor_control_request request = {
        .command = forward ? MOTOR_CONTROL_CMD_START_DEMO_FORWARD :
                             MOTOR_CONTROL_CMD_START_DEMO_REVERSE,
        .axis = 0,
        .value = 0,
    };

    return motor_control_exchange(&request, state);
}

int motor_control_stop_motion(struct motor_control_state *state)
{
    const struct motor_control_request request = {
        .command = MOTOR_CONTROL_CMD_STOP_MOTION,
        .axis = 0,
        .value = 0,
    };

    return motor_control_exchange(&request, state);
}
