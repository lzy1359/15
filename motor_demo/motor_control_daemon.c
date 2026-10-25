#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <poll.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "ethercat_master_ioctl.h"
#include "Rockchip_MADHT1505BA1.h"
#include "motor_control_protocol.h"

#define MOTOR_READY_RETRY_COUNT         50
#define MOTOR_STARTUP_READY_RETRY_COUNT 200
#define MOTOR_READY_RETRY_US            50000
#define MOTOR_SYNC_TARGET_DEG   0
#define MOTOR_TARGET_MAX_DEG    360
#define MOTOR_EAGER_RETRY_US    20000
#define MOTOR_MASTER_PHASE_RETRY_US 5000
#define MOTOR_MASTER_IDLE_STABLE_POLLS 2
#define MOTOR_MASTER_RETRY_US   20000
#define MOTOR_DEFAULT_RT_PRIO   20
/*
 * The EtherCAT cyclic task runs at 1 kHz in Rockchip_MADHT1505BA1.c.
 * Slew the commanded target so position moves are visibly smooth.
 */
#define MOTOR_MOVE_SPEED_DEG_PER_SEC 45
#define MOTOR_CYCLIC_FREQUENCY_HZ    MOTOR_CONTROL_CYCLIC_FREQUENCY_HZ
#define MOTOR_MOVE_STEP_NUMERATOR \
    (MOTOR_CONTROL_COUNTS_PER_REV * MOTOR_MOVE_SPEED_DEG_PER_SEC)
#define MOTOR_MOVE_STEP_DENOMINATOR \
    (MOTOR_CONTROL_ANGLE_FULL_TURN_DEG * MOTOR_CYCLIC_FREQUENCY_HZ)
#define MOTOR_MOTION_TICK_MS          20
#define MOTOR_DEMO_RPM                 6
#define MOTOR_TEST_STEP_TIMEOUT_MS     3000
#define MOTOR_TEST_POSITION_TOLERANCE_COUNTS 30

static volatile sig_atomic_t daemon_quit;
static bool motor_initialized;
static bool motor_ready;
static bool motor_has_been_ready;
static bool motor_reference_valid;
static bool motor_waiting_for_master;
static int32_t motor_reference_raw;
static int listen_fd = -1;
static MADHT1505BA1_object slave;
static enum motor_control_motion_mode motor_motion_mode;
static int32_t motor_motion_result;
static int32_t motor_test_target_raw;
static uint64_t motor_test_deadline_ms;
static uint64_t motor_demo_last_ms;
static int64_t motor_demo_count_remainder;

enum motor_master_phase
{
    MOTOR_MASTER_PHASE_WAITING = 0,
    MOTOR_MASTER_PHASE_IDLE = 1,
    MOTOR_MASTER_PHASE_OPERATION = 2,
};

enum motor_master_wait_reason
{
    MOTOR_MASTER_WAIT_NONE = 0,
    MOTOR_MASTER_WAIT_DEVICE,
    MOTOR_MASTER_WAIT_IDLE_PHASE,
    MOTOR_MASTER_WAIT_SCAN,
    MOTOR_MASTER_WAIT_ACTIVE,
    MOTOR_MASTER_WAIT_IDLE_STABLE,
};

static enum motor_master_wait_reason motor_master_wait_reason;

static uint64_t monotonic_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static int thread_bind_cpu(int cpu)
{
    cpu_set_t mask;

    CPU_ZERO(&mask);
    CPU_SET(cpu, &mask);

    return sched_setaffinity(0, sizeof(mask), &mask);
}

static bool env_get_int(const char *name, int *value)
{
    char *end = NULL;
    const char *text = getenv(name);
    long parsed;

    if (text == NULL || *text == '\0')
        return false;

    errno = 0;
    parsed = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0')
        return false;

    *value = (int)parsed;
    return true;
}

static void configure_process_runtime(void)
{
    struct sched_param param;
    int cpu;
    int policy;
    int rt_prio;

    /* Tighten nanosleep wakeups for the cyclic control thread. */
#ifdef PR_SET_TIMERSLACK
    if (prctl(PR_SET_TIMERSLACK, 1UL, 0, 0, 0) != 0)
        perror("motor_control_daemon: prctl(PR_SET_TIMERSLACK) failed");
#endif

    if (env_get_int("MOTOR_CONTROL_CPU", &cpu) && thread_bind_cpu(cpu) != 0)
        perror("motor_control_daemon: bind cpu failed");

    policy = sched_getscheduler(0);
    if (policy < 0)
    {
        perror("motor_control_daemon: sched_getscheduler failed");
        return;
    }

    if (!env_get_int("MOTOR_CONTROL_RT_PRIO", &rt_prio))
    {
        if (policy == SCHED_FIFO || policy == SCHED_RR)
            return;

        rt_prio = MOTOR_DEFAULT_RT_PRIO;
    }

    if (rt_prio <= 0)
        return;

    memset(&param, 0, sizeof(param));
    param.sched_priority = rt_prio;
    if (sched_setscheduler(getpid(), SCHED_FIFO, &param) != 0)
        perror("motor_control_daemon: sched_setscheduler failed");
}

static void daemon_request_shutdown(void)
{
    daemon_quit = 1;
    if (listen_fd >= 0)
    {
        close(listen_fd);
        listen_fd = -1;
    }
}

static int32_t motor_floor_div(int32_t dividend, int32_t divisor)
{
    int32_t quotient;
    int32_t remainder;

    quotient = dividend / divisor;
    remainder = dividend % divisor;
    if ((remainder != 0) && (dividend < 0))
        quotient--;

    return quotient;
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

static void handle_signal(int sig)
{
    printf("motor_control_daemon: received signal %d\n", sig);
    daemon_request_shutdown();
}

static int install_signal_handler(int sig, void (*handler)(int))
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handler;
    sigemptyset(&sa.sa_mask);
    return sigaction(sig, &sa, NULL);
}

static void motor_state_reset(struct motor_control_state *state)
{
    memset(state, 0, sizeof(*state));
}

static bool motor_refresh_ready_state(void)
{
    if (!motor_initialized)
    {
        motor_ready = false;
        return false;
    }

    if (MADHT1505BA1_check_motor(&slave) == 0)
    {
        motor_ready = true;
        return true;
    }

    if (motor_ready)
    {
        printf("motor_control_daemon: motor lost ready status=0x%04x mode=%d al=0x%02x wc=%u state=%u\n",
               slave.status, slave.opmode, slave.sc_state.al_state,
               slave.domain_state.working_counter, slave.domain_state.wc_state);
    }
    motor_ready = false;
    return false;
}

static int32_t motor_get_raw_position(void)
{
    return MADHT1505BA1_run_position_acquisition(&slave);
}

static void motor_capture_reference(int32_t raw_position)
{
    motor_reference_raw = raw_position;
    motor_reference_valid = true;
}

static int32_t motor_counts_to_display_degrees(int32_t raw_position)
{
    int32_t relative_counts;
    int32_t normalized_counts;
    int32_t rounded_degrees;

    if (!motor_reference_valid)
        return 0;

    relative_counts = raw_position - motor_reference_raw;
    normalized_counts = relative_counts % MOTOR_CONTROL_COUNTS_PER_REV;
    if (normalized_counts < 0)
        normalized_counts += MOTOR_CONTROL_COUNTS_PER_REV;

    rounded_degrees =
        (int32_t)(((int64_t)normalized_counts *
                   MOTOR_CONTROL_ANGLE_FULL_TURN_DEG +
                   (MOTOR_CONTROL_COUNTS_PER_REV / 2)) /
                  MOTOR_CONTROL_COUNTS_PER_REV);
    if (rounded_degrees >= MOTOR_CONTROL_ANGLE_FULL_TURN_DEG)
        rounded_degrees = 0;

    return rounded_degrees;
}

static int32_t motor_degrees_to_target_counts(int32_t degrees)
{
    int32_t current_raw;
    int32_t relative_counts;
    int32_t current_turn;
    int32_t best_target;
    int64_t best_delta;

    if (!motor_reference_valid)
        motor_capture_reference(motor_get_raw_position());

    current_raw = motor_get_raw_position();
    relative_counts = current_raw - motor_reference_raw;
    current_turn = motor_floor_div(relative_counts, MOTOR_CONTROL_COUNTS_PER_REV);

    best_target = motor_reference_raw +
                  current_turn * MOTOR_CONTROL_COUNTS_PER_REV +
                  (int32_t)(((int64_t)degrees *
                             MOTOR_CONTROL_COUNTS_PER_REV +
                             (MOTOR_CONTROL_ANGLE_FULL_TURN_DEG / 2)) /
                            MOTOR_CONTROL_ANGLE_FULL_TURN_DEG);
    best_delta = llabs((long long)best_target - current_raw);

    for (int offset = -1; offset <= 1; offset++)
    {
        int32_t candidate = motor_reference_raw +
                            (current_turn + offset) * MOTOR_CONTROL_COUNTS_PER_REV +
                            (int32_t)(((int64_t)degrees *
                                       MOTOR_CONTROL_COUNTS_PER_REV +
                                       (MOTOR_CONTROL_ANGLE_FULL_TURN_DEG / 2)) /
                                      MOTOR_CONTROL_ANGLE_FULL_TURN_DEG);
        int64_t delta = llabs((long long)candidate - current_raw);

        if (delta < best_delta)
        {
            best_target = candidate;
            best_delta = delta;
        }
    }

    return best_target;
}

static int32_t motor_degrees_to_counts(int32_t degrees)
{
    return (int32_t)(((int64_t)degrees * MOTOR_CONTROL_COUNTS_PER_REV +
                      (degrees >= 0 ? MOTOR_CONTROL_ANGLE_FULL_TURN_DEG / 2 :
                                      -(MOTOR_CONTROL_ANGLE_FULL_TURN_DEG / 2))) /
                     MOTOR_CONTROL_ANGLE_FULL_TURN_DEG);
}

static bool motor_motion_active(void)
{
    return motor_motion_mode != MOTOR_CONTROL_MOTION_IDLE;
}

static void motor_set_target_raw(int32_t target_raw)
{
    MADHT1505BA1_motor_set_position_run(target_raw, &slave);
}

static void motor_stop_motion_internal(bool failed)
{
    if (motor_initialized)
        motor_set_target_raw(motor_get_raw_position());

    if (motor_motion_active())
        printf("motor_control_daemon: motion %d stopped%s\n", motor_motion_mode,
               failed ? " due to error" : "");

    motor_motion_mode = MOTOR_CONTROL_MOTION_IDLE;
    motor_test_deadline_ms = 0;
    motor_demo_count_remainder = 0;
    if (failed)
        motor_motion_result = -1;
}

static void motor_start_test_step(enum motor_control_motion_mode mode,
                                  int32_t angle)
{
    motor_motion_mode = mode;
    motor_test_target_raw = motor_degrees_to_target_counts(angle);
    motor_test_deadline_ms = monotonic_ms() + MOTOR_TEST_STEP_TIMEOUT_MS;
    motor_set_target_raw(motor_test_target_raw);
    printf("motor_control_daemon: test step=%d angle=%d raw_target=%d\n",
           mode, angle, motor_test_target_raw);
}

static void motor_start_test(void)
{
    motor_motion_result = 0;
    motor_start_test_step(MOTOR_CONTROL_MOTION_TEST_TO_ZERO, 0);
}

static void motor_start_demo(bool forward)
{
    motor_motion_mode = forward ? MOTOR_CONTROL_MOTION_DEMO_FORWARD :
                                  MOTOR_CONTROL_MOTION_DEMO_REVERSE;
    motor_motion_result = 0;
    motor_demo_last_ms = monotonic_ms();
    motor_demo_count_remainder = 0;
    motor_set_target_raw(motor_get_raw_position());
    printf("motor_control_daemon: %s demo started at %d, speed=%d rpm\n",
           forward ? "forward" : "reverse", motor_get_raw_position(),
           MOTOR_DEMO_RPM);
}

static void motor_motion_tick(void)
{
    uint64_t now_ms;

    if (!motor_motion_active())
        return;

    if (!motor_refresh_ready_state() || (slave.status & 0x0008))
    {
        motor_stop_motion_internal(true);
        return;
    }

    now_ms = monotonic_ms();
    switch (motor_motion_mode)
    {
    case MOTOR_CONTROL_MOTION_TEST_TO_ZERO:
    case MOTOR_CONTROL_MOTION_TEST_TO_TEN:
    case MOTOR_CONTROL_MOTION_TEST_RETURN_ZERO:
        if (now_ms > motor_test_deadline_ms)
        {
            printf("motor_control_daemon: test timed out at step=%d raw=%d target=%d\n",
                   motor_motion_mode, motor_get_raw_position(),
                   motor_test_target_raw);
            motor_stop_motion_internal(true);
            return;
        }

        if (llabs((long long)motor_get_raw_position() - motor_test_target_raw) >
                MOTOR_TEST_POSITION_TOLERANCE_COUNTS ||
            !(slave.status & 0x0400))
        {
            return;
        }

        if (motor_motion_mode == MOTOR_CONTROL_MOTION_TEST_TO_ZERO)
            motor_start_test_step(MOTOR_CONTROL_MOTION_TEST_TO_TEN, 10);
        else if (motor_motion_mode == MOTOR_CONTROL_MOTION_TEST_TO_TEN)
            motor_start_test_step(MOTOR_CONTROL_MOTION_TEST_RETURN_ZERO, 0);
        else
        {
            printf("motor_control_daemon: one-key test passed\n");
            motor_motion_mode = MOTOR_CONTROL_MOTION_IDLE;
            motor_motion_result = 1;
        }
        break;

    case MOTOR_CONTROL_MOTION_DEMO_FORWARD:
    case MOTOR_CONTROL_MOTION_DEMO_REVERSE:
    {
        uint64_t elapsed_ms = now_ms - motor_demo_last_ms;
        int64_t counts;

        /* Avoid a delayed userspace scheduling slot creating a large jump. */
        if (elapsed_ms > 200)
            elapsed_ms = 200;
        motor_demo_last_ms = now_ms;
        motor_demo_count_remainder += (int64_t)elapsed_ms *
                                      MOTOR_CONTROL_COUNTS_PER_REV *
                                      MOTOR_DEMO_RPM;
        counts = motor_demo_count_remainder / 60000;
        motor_demo_count_remainder %= 60000;
        if (counts > 0)
        {
            if (motor_motion_mode == MOTOR_CONTROL_MOTION_DEMO_REVERSE)
                counts = -counts;
            motor_set_target_raw(slave.user_target_pos + (int32_t)counts);
        }
        break;
    }

    case MOTOR_CONTROL_MOTION_IDLE:
    default:
        break;
    }
}

static uint32_t motor_get_jitter_us(void)
{
    uint32_t max;

    max = MADHT1505BA1_time_statistics_latency_max_ns();
    /* Round up so a displayed 30 us never masks a 30.x us latency. */
    return (max + 999U) / 1000U;
}

static void motor_fill_state(struct motor_control_state *state)
{
    motor_state_reset(state);
    motor_refresh_ready_state();
    state->ready = motor_ready ? 1 : 0;
    state->motion_mode = motor_motion_mode;
    state->motion_result = motor_motion_result;

    if (!motor_initialized)
        return;

    state->axis_online[0] = slave.sc_state.online ? 1 : 0;
    state->axis_position[0] =
        motor_counts_to_display_degrees(motor_get_raw_position());
    state->axis_target[0] =
        motor_counts_to_display_degrees(slave.user_target_pos);
    state->axis_status_word[0] = slave.status;
    state->axis_opmode[0] = slave.opmode;
    state->axis_al_state[0] = slave.sc_state.al_state;
    state->axis_working_counter[0] = slave.domain_state.working_counter;
    state->axis_wc_state[0] = slave.domain_state.wc_state;
    state->axis_servo_on[0] = ((slave.status & 0x006f) == 0x0027) ? 1 : 0;
    state->axis_in_position[0] = (slave.status & 0x0400) ? 1 : 0;
    state->axis_fault[0] = (slave.status & 0x0008) ? 1 : 0;
    state->jitter_us = motor_get_jitter_us();
}

static void motor_deinit(void)
{
    if (!motor_initialized)
        return;

    motor_stop_motion_internal(true);
    MADHT1505BA1_master_deinit();
    motor_initialized = false;
    motor_ready = false;
    motor_reference_valid = false;
    motor_master_wait_reason = MOTOR_MASTER_WAIT_NONE;
}

static const char *motor_master_phase_name(uint8_t phase)
{
    switch (phase)
    {
    case MOTOR_MASTER_PHASE_WAITING:
        return "waiting";
    case MOTOR_MASTER_PHASE_IDLE:
        return "idle";
    case MOTOR_MASTER_PHASE_OPERATION:
        return "operation";
    default:
        return "unknown";
    }
}

static bool motor_query_master_info(ec_ioctl_master_t *master_info)
{
    int fd;
    int ret;

    if (!master_info)
        return false;

    fd = open("/dev/EtherCAT0", O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        return false;

    memset(master_info, 0, sizeof(*master_info));
    ret = ioctl(fd, EC_IOCTL_MASTER, master_info);
    close(fd);
    return ret == 0;
}

static int motor_wait_for_master_idle(void)
{
    ec_ioctl_master_t master_info;
    unsigned int idle_stable_polls = 0;

    while (!daemon_quit)
    {
        if (!motor_query_master_info(&master_info))
        {
            idle_stable_polls = 0;
            if (motor_master_wait_reason != MOTOR_MASTER_WAIT_DEVICE)
            {
                printf("motor_control_daemon: waiting for EtherCAT control device\n");
                motor_master_wait_reason = MOTOR_MASTER_WAIT_DEVICE;
            }
            usleep(MOTOR_MASTER_PHASE_RETRY_US);
            continue;
        }

        if (master_info.active)
        {
            idle_stable_polls = 0;
            if (motor_master_wait_reason != MOTOR_MASTER_WAIT_ACTIVE)
            {
                printf("motor_control_daemon: waiting for EtherCAT master release "
                       "(phase=%s active=%u scan_busy=%u)\n",
                       motor_master_phase_name(master_info.phase),
                       master_info.active, master_info.scan_busy);
                motor_master_wait_reason = MOTOR_MASTER_WAIT_ACTIVE;
            }
            usleep(MOTOR_MASTER_PHASE_RETRY_US);
            continue;
        }

        if (master_info.phase != MOTOR_MASTER_PHASE_IDLE)
        {
            idle_stable_polls = 0;
            if (motor_master_wait_reason != MOTOR_MASTER_WAIT_IDLE_PHASE)
            {
                printf("motor_control_daemon: waiting for EtherCAT master idle phase "
                       "(phase=%s active=%u scan_busy=%u)\n",
                       motor_master_phase_name(master_info.phase),
                       master_info.active, master_info.scan_busy);
                motor_master_wait_reason = MOTOR_MASTER_WAIT_IDLE_PHASE;
            }
            usleep(MOTOR_MASTER_PHASE_RETRY_US);
            continue;
        }

        if (master_info.scan_busy)
        {
            idle_stable_polls = 0;
            if (motor_master_wait_reason != MOTOR_MASTER_WAIT_SCAN)
            {
                printf("motor_control_daemon: waiting for EtherCAT bus scan to finish "
                       "(phase=%s active=%u scan_busy=%u)\n",
                       motor_master_phase_name(master_info.phase),
                       master_info.active, master_info.scan_busy);
                motor_master_wait_reason = MOTOR_MASTER_WAIT_SCAN;
            }
            usleep(MOTOR_MASTER_PHASE_RETRY_US);
            continue;
        }

        idle_stable_polls++;
        if (idle_stable_polls < MOTOR_MASTER_IDLE_STABLE_POLLS)
        {
            if (motor_master_wait_reason != MOTOR_MASTER_WAIT_IDLE_STABLE)
            {
                printf("motor_control_daemon: waiting for EtherCAT idle phase to stabilize "
                       "(phase=%s active=%u scan_busy=%u)\n",
                       motor_master_phase_name(master_info.phase),
                       master_info.active, master_info.scan_busy);
                motor_master_wait_reason = MOTOR_MASTER_WAIT_IDLE_STABLE;
            }
            usleep(MOTOR_MASTER_PHASE_RETRY_US);
            continue;
        }

        if (motor_master_wait_reason != MOTOR_MASTER_WAIT_NONE)
        {
            printf("motor_control_daemon: EtherCAT master idle phase detected and stable\n");
            motor_master_wait_reason = MOTOR_MASTER_WAIT_NONE;
        }
        return MOTOR_CONTROL_STATUS_OK;
    }

    return MOTOR_CONTROL_STATUS_ERROR;
}

static int motor_ready_retry_limit(void)
{
    return motor_has_been_ready ?
        MOTOR_READY_RETRY_COUNT : MOTOR_STARTUP_READY_RETRY_COUNT;
}

static useconds_t motor_retry_delay_us(void)
{
    if (!motor_initialized && motor_waiting_for_master)
        return MOTOR_MASTER_RETRY_US;

    return MOTOR_EAGER_RETRY_US;
}

static int motor_init_once(void)
{
    int ret;

    slave.alias = 0;
    slave.position = 0;
    slave.step_numerator = MOTOR_MOVE_STEP_NUMERATOR;
    slave.step_denominator = MOTOR_MOVE_STEP_DENOMINATOR;
    slave.step_remainder = 0;
    motor_master_wait_reason = MOTOR_MASTER_WAIT_NONE;

    ret = motor_wait_for_master_idle();
    if (ret != MOTOR_CONTROL_STATUS_OK)
        return MOTOR_CONTROL_STATUS_ERROR;

    ret = MADHT1505BA1_master_init(2);
    if (ret == -1)
    {
        if (!motor_waiting_for_master)
        {
            printf("motor_control_daemon: waiting for EtherCAT master availability\n");
            motor_waiting_for_master = true;
        }
        return MOTOR_CONTROL_STATUS_ERROR;
    }
    if (motor_waiting_for_master)
    {
        printf("motor_control_daemon: EtherCAT master is now available\n");
        motor_waiting_for_master = false;
    }
    motor_initialized = true;

    ret = MADHT1505BA1_slaves_init(&slave);
    if (ret == -1)
        goto error;

    ret = MADHT1505BA1_master_activate();
    if (ret == -1)
        goto error;

    ret = MADHT1505BA1_slaves_activate(&slave);
    if (ret == -1)
        goto error;

    ret = MADHT1505BA1_slave_start(1, &slave);
    if (ret == -1)
        goto error;

    return MOTOR_CONTROL_STATUS_OK;

error:
    printf("motor_control_daemon: motor init sequence failed\n");
    motor_deinit();
    return MOTOR_CONTROL_STATUS_ERROR;
}

static int motor_ensure_ready(void)
{
    int retry;
    int retry_limit;

    if (motor_refresh_ready_state())
        return MOTOR_CONTROL_STATUS_OK;

    retry_limit = motor_ready_retry_limit();

    if (motor_initialized)
    {
        for (retry = 0; retry < retry_limit && !daemon_quit; retry++)
        {
            if (motor_refresh_ready_state())
            {
                motor_capture_reference(motor_get_raw_position());
                motor_has_been_ready = true;
                printf("motor_control_daemon: motor recovered raw=%d reference=%d\n",
                       motor_get_raw_position(), motor_reference_raw);
                return MOTOR_CONTROL_STATUS_OK;
            }
            usleep(MOTOR_READY_RETRY_US);
        }

        printf("motor_control_daemon: in-place ready wait timeout, reinitializing status=0x%04x mode=%d al=0x%02x wc=%u state=%u\n",
               slave.status, slave.opmode, slave.sc_state.al_state,
               slave.domain_state.working_counter, slave.domain_state.wc_state);
        motor_deinit();
    }

    if (motor_init_once() != MOTOR_CONTROL_STATUS_OK)
        return MOTOR_CONTROL_STATUS_ERROR;

    retry_limit = motor_ready_retry_limit();
    for (retry = 0; retry < retry_limit && !daemon_quit; retry++)
    {
        if (motor_refresh_ready_state())
        {
            motor_capture_reference(motor_get_raw_position());
            motor_has_been_ready = true;
            printf("motor_control_daemon: motor is ready raw=%d reference=%d\n",
                   motor_get_raw_position(), motor_reference_raw);
            return MOTOR_CONTROL_STATUS_OK;
        }
        usleep(MOTOR_READY_RETRY_US);
    }

    printf("motor_control_daemon: motor ready timeout status=0x%04x mode=%d pos=%d al=0x%02x wc=%u state=%u\n",
           slave.status, slave.opmode, slave.curpos, slave.sc_state.al_state,
           slave.domain_state.working_counter, slave.domain_state.wc_state);
    motor_deinit();
    return MOTOR_CONTROL_STATUS_ERROR;
}

static int handle_request(const struct motor_control_request *request,
                          struct motor_control_response *response)
{
    int status = MOTOR_CONTROL_STATUS_OK;

    switch (request->command)
    {
    case MOTOR_CONTROL_CMD_GET_STATE:
        break;
    case MOTOR_CONTROL_CMD_RESET_ALL:
        if (motor_motion_active())
        {
            status = MOTOR_CONTROL_STATUS_BUSY;
            break;
        }
        status = motor_ensure_ready();
        if (status == MOTOR_CONTROL_STATUS_OK)
        {
            if (!(slave.status & 0x0400))
            {
                status = MOTOR_CONTROL_STATUS_BUSY;
                break;
            }
            motor_capture_reference(motor_get_raw_position());
            motor_set_target_raw(motor_get_raw_position());
            motor_motion_result = 0;
            printf("motor_control_daemon: zero reference calibrated raw=%d\n",
                   motor_reference_raw);
        }
        break;
    case MOTOR_CONTROL_CMD_SYNC_ALL:
        if (motor_motion_active())
        {
            status = MOTOR_CONTROL_STATUS_BUSY;
            break;
        }
        status = motor_ensure_ready();
        if (status == MOTOR_CONTROL_STATUS_OK)
            motor_set_target_raw(
                motor_degrees_to_target_counts(MOTOR_SYNC_TARGET_DEG));
        break;
    case MOTOR_CONTROL_CMD_SET_POSITION:
        if ((request->axis < 0) || (request->axis >= MOTOR_CONTROL_AXIS_COUNT) ||
            (request->value < 0) || (request->value > MOTOR_TARGET_MAX_DEG))
        {
            status = MOTOR_CONTROL_STATUS_INVALID;
            break;
        }
        if (motor_motion_active())
        {
            status = MOTOR_CONTROL_STATUS_BUSY;
            break;
        }

        status = motor_ensure_ready();
        if (status == MOTOR_CONTROL_STATUS_OK)
        {
            int32_t target_counts = motor_degrees_to_target_counts(request->value);

            printf("motor_control_daemon: set axis=%d angle=%d raw_cur=%d raw_target=%d\n",
                   request->axis, request->value, motor_get_raw_position(),
                   target_counts);
            motor_set_target_raw(target_counts);
        }
        break;
    case MOTOR_CONTROL_CMD_MOVE_RELATIVE:
        if ((request->axis < 0) || (request->axis >= MOTOR_CONTROL_AXIS_COUNT) ||
            (request->value < -MOTOR_TARGET_MAX_DEG) ||
            (request->value > MOTOR_TARGET_MAX_DEG))
        {
            status = MOTOR_CONTROL_STATUS_INVALID;
            break;
        }
        if (motor_motion_active())
        {
            status = MOTOR_CONTROL_STATUS_BUSY;
            break;
        }

        status = motor_ensure_ready();
        if (status == MOTOR_CONTROL_STATUS_OK)
        {
            int32_t target_counts = motor_get_raw_position() +
                                    motor_degrees_to_counts(request->value);

            printf("motor_control_daemon: relative axis=%d delta=%d raw_cur=%d raw_target=%d\n",
                   request->axis, request->value, motor_get_raw_position(),
                   target_counts);
            motor_set_target_raw(target_counts);
        }
        break;
    case MOTOR_CONTROL_CMD_RUN_TEST:
        if (motor_motion_active())
        {
            status = MOTOR_CONTROL_STATUS_BUSY;
            break;
        }
        status = motor_ensure_ready();
        if (status == MOTOR_CONTROL_STATUS_OK)
            motor_start_test();
        break;
    case MOTOR_CONTROL_CMD_START_DEMO_FORWARD:
    case MOTOR_CONTROL_CMD_START_DEMO_REVERSE:
        if (motor_motion_active())
        {
            status = MOTOR_CONTROL_STATUS_BUSY;
            break;
        }
        status = motor_ensure_ready();
        if (status == MOTOR_CONTROL_STATUS_OK)
            motor_start_demo(request->command == MOTOR_CONTROL_CMD_START_DEMO_FORWARD);
        break;
    case MOTOR_CONTROL_CMD_STOP_MOTION:
        status = motor_ensure_ready();
        if (status == MOTOR_CONTROL_STATUS_OK)
        {
            motor_stop_motion_internal(false);
            motor_motion_result = 0;
        }
        break;
    default:
        status = MOTOR_CONTROL_STATUS_INVALID;
        break;
    }

    response->status = status;
    motor_fill_state(&response->state);
    return status;
}

int main(void)
{
    int fd;
    int client_fd;
    int ret;
    struct sockaddr_un addr;
    struct pollfd poll_fd;

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    configure_process_runtime();

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        perror("motor_control_daemon: mlockall failed");

    if (install_signal_handler(SIGINT, handle_signal) != 0)
        perror("motor_control_daemon: sigaction(SIGINT)");
    if (install_signal_handler(SIGTERM, handle_signal) != 0)
        perror("motor_control_daemon: sigaction(SIGTERM)");
    if (install_signal_handler(SIGHUP, SIG_IGN) != 0)
        perror("motor_control_daemon: sigaction(SIGHUP)");
    if (install_signal_handler(SIGPIPE, SIG_IGN) != 0)
        perror("motor_control_daemon: sigaction(SIGPIPE)");

    unlink(MOTOR_CONTROL_SOCKET_PATH);
    listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd < 0)
    {
        perror("motor_control_daemon: socket");
        return 1;
    }
    fd = listen_fd;

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, MOTOR_CONTROL_SOCKET_PATH,
            sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("motor_control_daemon: bind");
        close(listen_fd);
        listen_fd = -1;
        unlink(MOTOR_CONTROL_SOCKET_PATH);
        return 1;
    }

    if (listen(fd, 4) < 0)
    {
        perror("motor_control_daemon: listen");
        close(listen_fd);
        listen_fd = -1;
        unlink(MOTOR_CONTROL_SOCKET_PATH);
        return 1;
    }

    printf("motor_control_daemon: listening on %s\n",
           MOTOR_CONTROL_SOCKET_PATH);

    while (!daemon_quit && !motor_ready)
    {
        if (motor_ensure_ready() == MOTOR_CONTROL_STATUS_OK)
            break;
        usleep(motor_retry_delay_us());
    }

    while (!daemon_quit)
    {
        struct motor_control_request request;
        struct motor_control_response response;

        poll_fd.fd = listen_fd;
        poll_fd.events = POLLIN;
        poll_fd.revents = 0;
        ret = poll(&poll_fd, 1, MOTOR_MOTION_TICK_MS);
        motor_motion_tick();
        if (ret == 0)
            continue;
        if (ret < 0)
        {
            if (daemon_quit)
                break;
            if (errno == EINTR)
                continue;
            perror("motor_control_daemon: poll");
            usleep(100000);
            continue;
        }

        if (!(poll_fd.revents & POLLIN))
            continue;

        client_fd = accept(listen_fd, NULL, NULL);
        if (client_fd < 0)
        {
            if (daemon_quit)
                break;
            if (errno == EINTR)
                continue;
            perror("motor_control_daemon: accept");
            continue;
        }

        memset(&request, 0, sizeof(request));
        memset(&response, 0, sizeof(response));

        if (read_full(client_fd, &request, sizeof(request)) == sizeof(request))
            handle_request(&request, &response);
        else
        {
            perror("motor_control_daemon: read request");
            response.status = MOTOR_CONTROL_STATUS_UNAVAILABLE;
        }

        if (write_full(client_fd, &response, sizeof(response)) != sizeof(response))
            perror("motor_control_daemon: write response");
        close(client_fd);
    }

    printf("motor_control_daemon: exiting\n");
    motor_deinit();
    if (listen_fd >= 0)
        close(listen_fd);
    listen_fd = -1;
    unlink(MOTOR_CONTROL_SOCKET_PATH);
    return 0;
}
