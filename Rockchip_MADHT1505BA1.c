#define _GNU_SOURCE
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>
#include <getopt.h>
#include <sched.h>
#include <pthread.h>
#include <stdarg.h>
#include "Rockchip_MADHT1505BA1.h"

#define POSITION_MAX 2000000000UL   
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING
#define TARGET_VELOCITY        1124000 /*target velocity*/ 
/*
 * Run the bounded cyclic exchange above the dedicated Ethernet IRQ thread.
 * It only owns CPU2 around each 1 ms deadline; the IRQ thread has the rest
 * of the period to process EtherCAT frames without delaying the next wakeup.
 */
#define MOTOR_RT_PRIORITY      70
#define STATS_WINDOW_CYCLES    FREQUENCY
#define MOTOR_OPMODE_CSP       8
#define MOTOR_CW_ENABLE        0x000F
#define MOTOR_6091_NUMERATOR   131072U
#define MOTOR_6091_DENOMINATOR 10000U
#define MOTOR_SEND_INTERVAL_US (PERIOD_NS / 1000)
#define MOTOR_TAKEOVER_RESET_CYCLES ((FREQUENCY + 19) / 20)
#define MOTOR_READY_STABLE_CYCLES   ((FREQUENCY + 19) / 20)
#define MOTOR_SHUTDOWN_TIMEOUT_CYCLES FREQUENCY
#define MOTOR_STARTUP_PRIME_MIN_CYCLES  (FREQUENCY / 8)
#define MOTOR_STARTUP_PRIME_MAX_CYCLES  ((FREQUENCY * 5) / 4)
#define MOTOR_STARTUP_DC_STABLE_CYCLES  3
#define MOTOR_SYNC_MONITOR_INTERVAL_CYCLES 4
#define MOTOR_DC_SYNC_TARGET_NS   25000U
#define MOTOR_DC_SYNC_INVALID_NS  0xffffffffU
/*
 * Spend the last 300 us in a tight loop so the 1 ms EtherCAT cycle is less
 * exposed to scheduler wakeup latency spikes on the isolated CPU.
 */
#define MOTOR_WAKEUP_SPIN_NS      300000ULL
#define MOTOR_STACK_PREFAULT_BYTES (16 * 1024)
#define MOTOR_WATCHDOG_DIVIDER    2500U
#define MOTOR_WATCHDOG_INTERVALS  20000U

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)                                                   
#define SHIFT_NS  (NSEC_PER_SEC / FREQUENCY /4)
#define STATUS_SERVO_ENABLE_BIT  (0x04)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
        (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
#define SLAVES_NUM_MAX 10

// Time statistics
uint32_t  latency_min_ns = 0, latency_max_ns = 0,
          period_min_ns = 0, period_max_ns = 0,
          exec_min_ns = 0, exec_max_ns = 0;
static int clean_cycle = 0;
// Time statistics

static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static bool master_runtime_logged;

int debug_mode = 0;
bool run = true;
bool shutdown_requested = false;
bool shutdown_started = false;
const struct timespec cycletime = {0, PERIOD_NS};
int slaves_cnt;
pthread_t thread;
bool thread_started = false;
int cpu_core;
MADHT1505BA1_object* slaves_group[SLAVES_NUM_MAX];

/*
 * DS2-EC identity observed on target:
 *   Vendor Id    0x00000a79
 *   Product code 0x00005000
 */
const int MADHT1505BA1_vendor = 0x00000a79;
const int MADHT1505BA1_product_code = 0x00005000;

static ec_pdo_entry_info_t slave_0_rx_pdo_1600_entries[] = {
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8},
};

static ec_pdo_entry_info_t slave_0_rx_pdo_1601_entries[] = {
    {0x607a, 0x00, 32},
};

static ec_pdo_entry_info_t slave_0_tx_pdo_1a00_entries[] = {
    {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},
};

static ec_pdo_entry_info_t slave_0_tx_pdo_1a01_entries[] = {
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 2, slave_0_rx_pdo_1600_entries},
    {0x1601, 1, slave_0_rx_pdo_1601_entries},
    {0x1a00, 2, slave_0_tx_pdo_1a00_entries},
    {0x1a01, 2, slave_0_tx_pdo_1a01_entries},
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave_0_pdos + 2, EC_WD_DISABLE},
    {0xff},
};

static void check_master_state(void);
static void check_slave_config_states(MADHT1505BA1_object *object);
static void check_domain_state(MADHT1505BA1_object *object);

static void printf_debug(const char* fmt, ...){
	if (debug_mode == 1) {
	  va_list args;
	  va_start(args, fmt); 
	  vprintf(fmt, args); 
	  va_end(args);
	}
}

static int thread_bind_cpu(int target_cpu)
{
    cpu_set_t mask;
    int cpu_num = sysconf(_SC_NPROCESSORS_CONF);
    int i;

    if (target_cpu >= cpu_num)
        return -1;

    CPU_ZERO(&mask);
    CPU_SET(target_cpu, &mask);

    if (pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask) < 0)
        perror("pthread_setaffinity_np");

    if (pthread_getaffinity_np(pthread_self(), sizeof(mask), &mask) < 0)
        perror("pthread_getaffinity_np");

    printf("Thread(%ld) bound to cpu:", gettid());
    for (i = 0; i < CPU_SETSIZE; i++) {
        if (CPU_ISSET(i, &mask)) {
            printf(" %d", i);
            break;    
        }
    }
    printf("\n");

    return i >= cpu_num ? -1 : i;
}

static struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

static void prefault_rt_stack(void)
{
    volatile char stack[MOTOR_STACK_PREFAULT_BYTES];

    memset((void *)stack, 0, sizeof(stack));
}

static void wait_until_target_time(const struct timespec *target_time)
{
    struct timespec now;
    struct timespec sleep_until;
    uint64_t target_ns;
    uint64_t sleep_until_ns;

    target_ns = TIMESPEC2NS(*target_time);

    if (MOTOR_WAKEUP_SPIN_NS > 0 && target_ns > MOTOR_WAKEUP_SPIN_NS) {
        sleep_until_ns = target_ns - MOTOR_WAKEUP_SPIN_NS;
        sleep_until.tv_sec = sleep_until_ns / NSEC_PER_SEC;
        sleep_until.tv_nsec = sleep_until_ns % NSEC_PER_SEC;
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &sleep_until, NULL);
    }

    for (;;) {
        clock_gettime(CLOCK_TO_USE, &now);
        if (TIMESPEC2NS(now) >= target_ns) {
            break;
        }
    }
}

static int domain_regs_fill_in(MADHT1505BA1_object *object) {

	// for (int i = 0; i < ARRAY_SIZE(code); i++) {
	// 	object.domain_regs[i] = 
	// 		(ec_pdo_entry_reg_t){object.alias, object.position, MADHT1505BA1_vendor, MADHT1505BA1_product_code, code[i], 0, (uint8_t *)object + sizeof(unsigned int) * i};
	// }

	object->domain_regs[0] = (ec_pdo_entry_reg_t){object->alias, object->position, MADHT1505BA1_vendor, MADHT1505BA1_product_code, 0x6040, 0, &(object->control_word)};
	object->domain_regs[1] = (ec_pdo_entry_reg_t){object->alias, object->position, MADHT1505BA1_vendor, MADHT1505BA1_product_code, 0x6060, 0, &(object->modes_of_operation)};
	object->domain_regs[2] = (ec_pdo_entry_reg_t){object->alias, object->position, MADHT1505BA1_vendor, MADHT1505BA1_product_code, 0x607a, 0, &(object->target_position)};
	object->domain_regs[3] = (ec_pdo_entry_reg_t){object->alias, object->position, MADHT1505BA1_vendor, MADHT1505BA1_product_code, 0x6041, 0, &(object->status_word)};
	object->domain_regs[4] = (ec_pdo_entry_reg_t){object->alias, object->position, MADHT1505BA1_vendor, MADHT1505BA1_product_code, 0x6061, 0, &(object->modes_of_operation_display)};
	object->domain_regs[5] = (ec_pdo_entry_reg_t){object->alias, object->position, MADHT1505BA1_vendor, MADHT1505BA1_product_code, 0x6064, 0, &(object->position_actual_value)};
	object->domain_regs[6] = (ec_pdo_entry_reg_t){object->alias, object->position, MADHT1505BA1_vendor, MADHT1505BA1_product_code, 0x606c, 0, &(object->current_velocity)};
	object->domain_regs[7] = (ec_pdo_entry_reg_t){};

	object->domain_pd = NULL;
    object->change_pos = false;
    object->target_pos_valid = false;
    object->drive_ready = false;
    object->takeover_required = true;
    object->takeover_reset_cycles = MOTOR_TAKEOVER_RESET_CYCLES;
    object->ready_stable_cycles = 0;
    object->user_set_pos = 0;
    object->user_target_pos = 0;
    object->max_step_per_cycle = 0;
	object->step_numerator = 0;
	object->step_denominator = 0;
	object->step_remainder = 0;
	
    return 0;
}

static int step_towards_target(MADHT1505BA1_object *object)
{
    uint64_t accumulated_step;
    int delta;
    int max_step;

    if (!object || object->step_numerator == 0 ||
        object->step_denominator == 0)
        return object ? object->user_target_pos : 0;

    delta = object->user_target_pos - object->user_set_pos;
    if (delta == 0) {
        object->step_remainder = 0;
        return object->user_target_pos;
    }

    accumulated_step = (uint64_t)object->step_remainder +
                       object->step_numerator;
    max_step = accumulated_step / object->step_denominator;
    object->step_remainder = accumulated_step % object->step_denominator;

    if (max_step <= 0)
        return object->user_set_pos;

    if (delta > max_step)
        return object->user_set_pos + max_step;
    if (delta < -max_step)
        return object->user_set_pos - max_step;

    object->step_remainder = 0;
    return object->user_target_pos;
}

static void slave_mark_not_ready(MADHT1505BA1_object *object)
{
    if (!object)
        return;

    object->drive_ready = false;
    object->ready_stable_cycles = 0;
}

static void slave_request_takeover(MADHT1505BA1_object *object)
{
    if (!object)
        return;

    slave_mark_not_ready(object);
    object->takeover_required = true;
    object->takeover_reset_cycles = MOTOR_TAKEOVER_RESET_CYCLES;
    object->target_pos_valid = false;
}

static bool slave_process_data_ready(const MADHT1505BA1_object *object)
{
    if (!object)
        return false;

    return object->sc_state.online &&
           object->sc_state.operational &&
           object->domain_state.wc_state == EC_WC_COMPLETE;
}

static bool slave_status_is_operation_enabled(uint16_t status)
{
    return (status & 0x006f) == 0x0027;
}

static bool slave_can_skip_takeover_reset(const MADHT1505BA1_object *object)
{
    if (!object) {
        return false;
    }

    return slave_status_is_operation_enabled(object->status) &&
           object->opmode == MOTOR_OPMODE_CSP;
}

static bool slave_needs_fast_state_poll(const MADHT1505BA1_object *object)
{
    if (!object)
        return false;

    return !object->sc_state.online ||
           !object->sc_state.operational ||
           object->takeover_required ||
           !object->drive_ready ||
           object->domain_state.wc_state != EC_WC_COMPLETE;
}

static void slave_seed_target_from_current(MADHT1505BA1_object *object)
{
    if (!object)
        return;

    object->user_set_pos = object->curpos;
    object->step_remainder = 0;
    if (!object->change_pos) {
        object->user_target_pos = object->curpos;
    }
    object->target_pos_valid = true;
}

static void slave_hold_current_position(MADHT1505BA1_object *object)
{
    if (!object || !object->domain_pd)
        return;

    slave_seed_target_from_current(object);
    EC_WRITE_S32(object->domain_pd + object->target_position,
                 object->user_set_pos);
}

static void slave_prepare_startup_output(MADHT1505BA1_object *object)
{
    if (!object || !object->domain_pd)
        return;

    object->status =
        EC_READ_U16(object->domain_pd + object->status_word);
    object->opmode =
        EC_READ_U8(object->domain_pd + object->modes_of_operation_display);
    object->curpos =
        EC_READ_S32(object->domain_pd + object->position_actual_value);

    EC_WRITE_U8(object->domain_pd + object->modes_of_operation,
                MOTOR_OPMODE_CSP);

    if (!object->target_pos_valid) {
        slave_seed_target_from_current(object);
    }

    if (object->status & 0x0008) {
        EC_WRITE_U16(object->domain_pd + object->control_word, 0x0080);
        slave_request_takeover(object);
        return;
    }

    if ((object->status & 0x004f) == 0x0040) {
        EC_WRITE_U16(object->domain_pd + object->control_word, 0x0006);
        return;
    }

    if ((object->status & 0x006f) == 0x0021) {
        EC_WRITE_U16(object->domain_pd + object->control_word, 0x0007);
        return;
    }

    if (slave_can_skip_takeover_reset(object)) {
        object->takeover_reset_cycles = 0;
    }

    slave_hold_current_position(object);
    EC_WRITE_U16(object->domain_pd + object->control_word, MOTOR_CW_ENABLE);
}

static bool master_startup_sync_stable(uint32_t sync_diff_ns,
                                       unsigned int *stable_cycles)
{
    if (sync_diff_ns == MOTOR_DC_SYNC_INVALID_NS ||
        sync_diff_ns > MOTOR_DC_SYNC_TARGET_NS) {
        *stable_cycles = 0;
        return false;
    }

    if (*stable_cycles < MOTOR_STARTUP_DC_STABLE_CYCLES) {
        (*stable_cycles)++;
    }

    return *stable_cycles >= MOTOR_STARTUP_DC_STABLE_CYCLES;
}

static void master_prime_process_data(void)
{
    struct timespec wakeupTime;
    unsigned int cycle;
    unsigned int sync_monitor_counter = 0;
    unsigned int sync_stable_cycles = 0;
    uint32_t sync_diff_ns = MOTOR_DC_SYNC_INVALID_NS;
    bool sync_monitor_armed = false;
    bool dc_sync_ready = false;
    int i;

    if (!master || slaves_cnt <= 0) {
        return;
    }

    printf("Priming EtherCAT process data for %u-%u cycles before RT handoff\n",
           MOTOR_STARTUP_PRIME_MIN_CYCLES,
           MOTOR_STARTUP_PRIME_MAX_CYCLES);

    clock_gettime(CLOCK_TO_USE, &wakeupTime);
    for (cycle = 0; cycle < MOTOR_STARTUP_PRIME_MAX_CYCLES; cycle++) {
        wakeupTime = timespec_add(wakeupTime, cycletime);
        wait_until_target_time(&wakeupTime);

        ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));
        ecrt_master_receive(master);
        if (sync_monitor_armed) {
            uint32_t diff = ecrt_master_sync_monitor_process(master);

            if (diff != MOTOR_DC_SYNC_INVALID_NS) {
                sync_diff_ns = diff;
            }
        }

        for (i = 0; i < slaves_cnt; i++) {
            if (!slaves_group[i] || !slaves_group[i]->domain ||
                !slaves_group[i]->domain_pd) {
                continue;
            }

            ecrt_domain_process(slaves_group[i]->domain);
            check_domain_state(slaves_group[i]);
            if (cycle == 0 || cycle + 1 == MOTOR_STARTUP_PRIME_MAX_CYCLES) {
                check_slave_config_states(slaves_group[i]);
            }
            slave_prepare_startup_output(slaves_group[i]);
        }

        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);
        if (++sync_monitor_counter >= MOTOR_SYNC_MONITOR_INTERVAL_CYCLES) {
            ecrt_master_sync_monitor_queue(master);
            sync_monitor_armed = true;
            sync_monitor_counter = 0;
        }

        for (i = 0; i < slaves_cnt; i++) {
            if (!slaves_group[i] || !slaves_group[i]->domain) {
                continue;
            }
            ecrt_domain_queue(slaves_group[i]->domain);
        }
        ecrt_master_send(master);

        if (cycle + 1 >= MOTOR_STARTUP_PRIME_MIN_CYCLES &&
            master_startup_sync_stable(sync_diff_ns, &sync_stable_cycles)) {
            dc_sync_ready = true;
            break;
        }
    }

    if (dc_sync_ready) {
        printf("EtherCAT startup prime settled after %u cycles, DC diff=%u ns\n",
               cycle + 1, sync_diff_ns);
    } else if (sync_diff_ns != MOTOR_DC_SYNC_INVALID_NS) {
        printf("EtherCAT startup prime reached %u cycles, DC diff=%u ns; hand off to RT thread\n",
               cycle, sync_diff_ns);
    } else {
        printf("EtherCAT startup prime reached %u cycles without valid DC sync sample; hand off to RT thread\n",
               cycle);
    }
}

static int slave_configure_position_units(MADHT1505BA1_object *object)
{
    int ret;

    if (!object || !object->sc)
        return -1;

    /*
     * DS2-E user manual, section 4.16, defines 0x6091 as the electronic
     * gear ratio and gives 131072 / 10000 as the default. 131072 is the
     * 17-bit magnetic encoder resolution, so one motor revolution is 10000
     * position pulses in 0x6064 and 0x607A units.
     */
    ret = ecrt_slave_config_sdo32(object->sc, 0x6091, 0x01,
                                  MOTOR_6091_NUMERATOR);
    if (ret < 0) {
        printf("Failed to configure 0x6091:01 motor revolutions, ret=%d.\n",
               ret);
        return -1;
    }

    ret = ecrt_slave_config_sdo32(object->sc, 0x6091, 0x02,
                                  MOTOR_6091_DENOMINATOR);
    if (ret < 0) {
        printf("Failed to configure 0x6091:02 shaft revolutions, ret=%d.\n",
               ret);
        return -1;
    }

    printf("Configured 0x6091 electronic gear: numerator=%u denominator=%u\n",
           MOTOR_6091_NUMERATOR, MOTOR_6091_DENOMINATOR);
    return 0;
}

static void check_master_state(void)
{

    ec_master_state_t ms;
 
    ecrt_master_state(master, &ms);
 
    if (ms.slaves_responding != master_state.slaves_responding) {
        printf_debug("found %u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf_debug("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf_debug("Link is %s.\n", ms.link_up ? "up" : "down");
    }
 
    master_state = ms;
}

static void check_slave_config_states(MADHT1505BA1_object *object)
{

    ec_slave_config_state_t s;
 
    ecrt_slave_config_state(object->sc, &s);
 
    if (s.al_state != object->sc_state.al_state) {
        printf_debug("slaveDrive %d: State 0x%02X.\n", object->alias, s.al_state);
    }
    if (s.online != object->sc_state.online) {
        printf_debug("slaveDrive %d: %s.\n", object->alias, s.online ? "online" : "offline");
    }
    printf_debug("slaveDrive %d: %s  operational.\n", object->alias, s.operational ? "yes" : "Not ");
 
    object->sc_state = s;
}

static void check_domain_state(MADHT1505BA1_object *object)
{

    ec_domain_state_t ds;
 
    ecrt_domain_state(object->domain, &ds);
 
    if (ds.working_counter != object->domain_state.working_counter) {
        printf_debug("Domain %d: WC %u.\n", object->alias, ds.working_counter);
    }
    if (ds.wc_state != object->domain_state.wc_state) {
        printf_debug("Domain %d: State %u.\n", object->alias, ds.wc_state);
    }
 
    object->domain_state = ds;
}

int MADHT1505BA1_master_init(int bind_core) {
	char* tmp = NULL;

	if (!master_runtime_logged) {
		printf("rockchip MADHT1505BA1 Motor drive program\n");

		tmp = getenv("RKOCKCHIP_MADHT1505BA1_DEBUG");
	    if (tmp == NULL) {
	        printf("env RKOCKCHIP_MADHT1505BA1_DEBUG not set\n");
	        debug_mode = 0;
	    } else if (!strcmp("1", tmp)) {
			debug_mode = 1;
		}
		master_runtime_logged = true;
	}
    cpu_core = bind_core;
    shutdown_requested = false;
    shutdown_started = false;
	master = ecrt_request_master(0);
    if (!master) {
		return -1;
	}
	printf("request_master sucess, check slaves responding %d\n", master_state.slaves_responding);
	check_master_state();

    if (ecrt_master_set_send_interval(master, MOTOR_SEND_INTERVAL_US) < 0) {
        printf("Failed to set send interval to %u us.\n",
               (unsigned int)MOTOR_SEND_INTERVAL_US);
        return -1;
    }

	return 0;
}

int MADHT1505BA1_slaves_init(MADHT1505BA1_object *object) {
	domain_regs_fill_in(object);
	object->sc = ecrt_master_slave_config(
								master,
								object->alias,
								object->position,
								MADHT1505BA1_vendor,
								MADHT1505BA1_product_code);
    if(!object->sc) {
        printf("Failed to get slave configuration.\n");
        return -1;
    }
	check_slave_config_states(object);
	
    object->domain = ecrt_master_create_domain(master);
    if (!object->domain) {
        printf("ecrt_master_create_domain is fail\n");
        return -1;  
    }
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(object->sc, EC_END, slave_0_syncs)) {
        printf("Failed to configure PDOs.\n");
        return -1;
    }
    /*
     * The drive raises AL 0x001B ("Sync manager watchdog") if PDO exchange
     * pauses for too long during the short hand-off window between the kernel
     * master entering IDLE and the userspace cyclic loop taking over.
     *
     * 2500 * 20000 * 40 ns = 2000 ms. The July 17, 2026 boot log shows the
     * current startup path can take longer than the previous 400 ms budget
     * before the userspace cyclic loop begins exchanging PDOs. Two seconds
     * covers bus scan, activation and prime cycles while still retaining
     * watchdog protection during runtime.
     */
    ecrt_slave_config_watchdog(object->sc, MOTOR_WATCHDOG_DIVIDER,
                               MOTOR_WATCHDOG_INTERVALS);
    if (slave_configure_position_units(object) < 0) {
        return -1;
    }
    // for(int i = 0; i<15; i++) {
    // 	printf("domain_regs[%d] : alias = %d position = %d index = %x\n",i, object->domain_regs[i].alias, object->domain_regs[i].position, object->domain_regs[i].index);
    // }
    if(ecrt_domain_reg_pdo_entry_list(object->domain, object->domain_regs)) {
        printf("Failed to ecrt_domain_reg_pdo_entry_list.\n");
        return -1;
    };

    ecrt_slave_config_dc(object->sc, 0x300, PERIOD_NS, 0, 0, 0);
	return 0;
}

int MADHT1505BA1_master_activate(void) {
    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        printf("ecrt_master_activate is fail\n");
        return -1;
    }
    return 0;
}

int MADHT1505BA1_slaves_activate(MADHT1505BA1_object *object) {
    printf("activate slaves %d\n", object->alias);
    if (!(object->domain_pd = ecrt_domain_data(object->domain))) {
        printf("ecrt_domain_data is fail\n");
        return -1;
    }else {
        printf("activate slaves %d is success\n", object->alias);
        return 0;
    }
}

int MADHT1505BA1_master_deinit(void) {
    shutdown_requested = true;
    run = false;

    if (thread_started) {
        pthread_join(thread, NULL);
        thread_started = false;
    }

    if (master) {
        ecrt_master_deactivate(master);
        ecrt_release_master(master);
        master = NULL;
    }

    for (int i = 0; i < slaves_cnt; i++) {
        if (slaves_group[i]) {
            slaves_group[i]->domain_pd = NULL;
            slaves_group[i]->change_pos = false;
            slaves_group[i]->target_pos_valid = false;
            slaves_group[i]->drive_ready = false;
            slaves_group[i]->takeover_required = true;
            slaves_group[i]->takeover_reset_cycles = MOTOR_TAKEOVER_RESET_CYCLES;
            slaves_group[i]->ready_stable_cycles = 0;
        }
        slaves_group[i] = NULL;
    }
    shutdown_requested = false;
    shutdown_started = false;
    slaves_cnt = 0;
    printf("MADHT1505BA1_master_deinit\n");
    return 0;
}

// void *slave_velocity_mode_pthread(void *arg) {
//     struct timespec wakeupTime, time;
// 	//MADHT1505BA1_object **object = (MADHT1505BA1_object **)arg;
// 	int counter = 0;
// 	struct sched_param param;
//     int maxpri, count, i;
//     int curpos = 0;

//     printf("slave_pthread bind_cpu\n");
//     if(thread_bind_cpu(cpu_core) == -1) {
//         printf("bind cpu core fail\n");
//     }

//     // The scheduling priority is the highest
//     maxpri = sched_get_priority_max(SCHED_FIFO);
//     if(maxpri == -1) { 
//         printf("sched_get_priority_max() failed");
//     }

//     param.sched_priority = maxpri;
//     if (sched_setscheduler(getpid(), SCHED_FIFO, &param) == -1) { 
//         perror("sched_setscheduler() failed");
//     }
//     printf("end thread set\n");

//     // Time statistics
//     struct timespec startTime, endTime, lastStartTime = {};
//     uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0;
             
//     period_max_ns = 0;
//     period_min_ns = 0xffffffff;
//     latency_max_ns = 0;
//     latency_min_ns = 0xffffffff;
//     clock_gettime(CLOCK_TO_USE, &lastStartTime);
//     // Time statistics

//     clock_gettime(CLOCK_TO_USE, &wakeupTime);
// 	while(run) {
		
//         wakeupTime = timespec_add(wakeupTime, cycletime);
// 		clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

//         // Write application time to master
//         //
//         // It is a good idea to use the target time (not the measured time) as
//         // application time, because it is more stable.
//         //
//         ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));
        
//         /*Receive process data*/
//         ecrt_master_receive(master);
//         for (i = 0; i < slaves_cnt; i++) {
//             ecrt_domain_process(slaves_group[i]->domain);
//             // check process data state (optional)
//             check_domain_state(slaves_group[i]);
//         }
//        	if(counter) {
//        		counter--;
//        	}else {
//        		counter = FREQUENCY;
//        		check_master_state();
//        		for (i = 0; i < slaves_cnt; i++) {
//                 check_slave_config_states(slaves_group[i]);
//                 EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word, 0x80);
//                 slaves_group[i]->status = EC_READ_U16(slaves_group[i]->domain_pd + slaves_group[i]->status_word);
//                 slaves_group[i]->opmode = EC_READ_U8(slaves_group[i]->domain_pd + slaves_group[i]->modes_of_operation_display);
//                 slaves_group[i]->cur_velocity = EC_READ_S32(slaves_group[i]->domain_pd + slaves_group[i]->current_velocity);
                
//                 curpos = EC_READ_S32(slaves_group[i]->domain_pd + slaves_group[i]->position_actual_value);
//                 // if(curpos < 0) {
//                 //     curpos = POSITION_MAX - abs(curpos);
//                 // }
//                 slaves_group[i]->curpos = curpos;
                
//                 printf_debug("slave %d madht:  act velocity = %d ,act position = %d,  status = 0x%x, opmode = 0x%x\n", 
//                 slaves_group[i]->alias, slaves_group[i]->cur_velocity, slaves_group[i]->curpos, slaves_group[i]->status, slaves_group[i]->opmode);
                
//                 if( (slaves_group[i]->status & 0x004f) == 0x0040) {
//                     printf_debug("0x06\n");
//                     EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word, 0x0006);
//                     EC_WRITE_U8(slaves_group[i]->domain_pd + slaves_group[i]->modes_of_operation, 9);
//                 }
//                 else if( (slaves_group[i]->status & 0x006f) == 0x0021) {
//                     printf_debug("0x07\n");
//                     EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word, 0x0007);
//                 }
//                 else if( (slaves_group[i]->status & 0x006f) == 0x0023) {
//                     printf_debug("0x0f\n");
//                     EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word, 0x000f);
//                     EC_WRITE_S32(slaves_group[i]->domain_pd + slaves_group[i]->target_velocity, slaves_group[i]->user_velocity);
//                 }
//                 //operation enabled
//                 else if( (slaves_group[i]->status & 0x006f) == 0x0027) {
//                     printf_debug("0x1f\n");
//                     EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word, 0x001f);
//                 }
//                 if(slaves_group[i]->change_velocity) {
//                     printf_debug("change velocity\n");
//                     EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word, 0x0007); // stop slaves
//                     slaves_group[i]->change_velocity = false;
//                 }
//             }        	
//        	}

//         clock_gettime(CLOCK_TO_USE, &time);
//         ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));

//         ecrt_master_sync_slave_clocks(master);
//         // send process data
//         for (i = 0; i < slaves_cnt; i++) {
//             ecrt_domain_queue(slaves_group[i]->domain);
//         }
//         ecrt_master_send(master);
        
//         // Time statistics
//         clock_gettime(CLOCK_TO_USE, &startTime);
//         latency_ns = DIFF_NS(wakeupTime, startTime);
//         period_ns = DIFF_NS(lastStartTime, startTime);
//         if (clean_cycle >= (12 * 60 * 60 * 1000)) { // 12 hour clean
//             clean_cycle = 0;
//             period_max_ns = 0;
//             period_min_ns = 0xffffffff;
//             latency_max_ns = 0;
//             latency_min_ns = 0xffffffff;
//         }
//         if (latency_ns > latency_max_ns) {
//             latency_max_ns = latency_ns;
//         }
//         if (latency_ns < latency_min_ns) {
//             latency_min_ns = latency_ns;
//         }
//         if (period_ns > period_max_ns) {
//             period_max_ns = period_ns;
//         }
//         if (period_ns < period_min_ns) {
//             period_min_ns = period_ns;
//         }
//         clean_cycle++;
//         lastStartTime = startTime;
//         // Time statistics
// 	}
// }

void *slave_position_mode_pthread(void *arg) {
    struct timespec wakeupTime;
    //MADHT1505BA1_object **object = (MADHT1505BA1_object **)arg;
    int counter = 0;
    struct sched_param param;
    int maxpri, i;
    int curpos = 0;
    uint32_t stats_window_cycles = 0;
    unsigned int shutdown_wait_cycles = 0;
    bool refresh_master_state = false;

    printf("slave_pthread bind_cpu\n");
    if(thread_bind_cpu(cpu_core) == -1) {
        printf("bind cpu core fail\n");
    }

    prefault_rt_stack();

    // The scheduling priority is the highest
    maxpri = sched_get_priority_max(SCHED_FIFO);
    if(maxpri == -1) { 
        printf("sched_get_priority_max() failed");
    }

    param.sched_priority = MOTOR_RT_PRIORITY;
    if (param.sched_priority > maxpri) {
        param.sched_priority = maxpri;
    }
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        perror("pthread_setschedparam() failed");
    }
    printf("end thread set priority=%d\n", param.sched_priority);

    // Time statistics
    struct timespec startTime, lastStartTime = {};
    uint32_t period_ns = 0, latency_ns = 0;
             
    period_max_ns = 0;
    period_min_ns = 0xffffffff;
    latency_max_ns = 0;
    latency_min_ns = 0xffffffff;
    clock_gettime(CLOCK_TO_USE, &lastStartTime);
    // Time statistics

    clock_gettime(CLOCK_TO_USE, &wakeupTime);
    while(run || shutdown_requested) {
        
        wakeupTime = timespec_add(wakeupTime, cycletime);
        wait_until_target_time(&wakeupTime);

        // Time statistics: measure wakeup latency immediately after sleep.
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        if (stats_window_cycles == 0) {
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
        }
        if (clean_cycle >= (5 * 60 * 1000)) { // 5 min clean
            clean_cycle = 0;
        }
        if (latency_ns > latency_max_ns) {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns) {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns) {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns) {
            period_min_ns = period_ns;
        }
        clean_cycle++;
        stats_window_cycles++;
        if (stats_window_cycles >= STATS_WINDOW_CYCLES) {
            stats_window_cycles = 0;
        }
        lastStartTime = startTime;

        // Write application time to master
        //
        // It is a good idea to use the target time (not the measured time) as
        // application time, because it is more stable.
        //
        ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));
        
        /*Receive process data*/
        ecrt_master_receive(master);
        for (i = 0; i < slaves_cnt; i++) {
            if (!slaves_group[i] || !slaves_group[i]->domain || !slaves_group[i]->domain_pd) {
                continue;
            }
            ecrt_domain_process(slaves_group[i]->domain);
            // check process data state (optional)
            check_domain_state(slaves_group[i]);
            slaves_group[i]->status =
                EC_READ_U16(slaves_group[i]->domain_pd + slaves_group[i]->status_word);
            slaves_group[i]->opmode =
                EC_READ_U8(slaves_group[i]->domain_pd + slaves_group[i]->modes_of_operation_display);
            curpos =
                EC_READ_S32(slaves_group[i]->domain_pd + slaves_group[i]->position_actual_value);
            slaves_group[i]->curpos = curpos;
        }

        refresh_master_state = false;
        if (counter == 0) {
            counter = FREQUENCY;
            refresh_master_state = true;
            check_master_state();
        } else {
            counter--;
        }

        for (i = 0; i < slaves_cnt; i++) {
            if (!slaves_group[i] || !slaves_group[i]->domain_pd) {
                continue;
            }
            if (refresh_master_state ||
                slave_needs_fast_state_poll(slaves_group[i])) {
                check_slave_config_states(slaves_group[i]);
            }
        }

        if (shutdown_requested) {
            bool all_preop = true;

            if (!shutdown_started) {
                printf("slave thread: shutdown sequence begin\n");
                for (i = 0; i < slaves_cnt; i++) {
                    if (!slaves_group[i] || !slaves_group[i]->domain_pd) {
                        continue;
                    }
                    slave_hold_current_position(slaves_group[i]);
                    EC_WRITE_U16(slaves_group[i]->domain_pd +
                                 slaves_group[i]->control_word, 0x0006);
                }
                ecrt_master_deactivate_slaves(master);
                shutdown_started = true;
                shutdown_wait_cycles = MOTOR_SHUTDOWN_TIMEOUT_CYCLES;
            }

            for (i = 0; i < slaves_cnt; i++) {
                if (!slaves_group[i]) {
                    continue;
                }
                check_slave_config_states(slaves_group[i]);
                if (slaves_group[i]->sc_state.operational ||
                    slaves_group[i]->sc_state.al_state == EC_AL_STATE_OP ||
                    slaves_group[i]->sc_state.al_state == EC_AL_STATE_SAFEOP) {
                    all_preop = false;
                }
            }

            if (all_preop) {
                printf("slave thread: all slaves left OP, shutdown sequence complete\n");
                break;
            }

            if (shutdown_wait_cycles == 0) {
                printf("slave thread: shutdown wait timeout, forcing master deactivate\n");
                break;
            }

            shutdown_wait_cycles--;
        } else {
        for (i = 0; i < slaves_cnt; i++) {
            if (!slaves_group[i] || !slaves_group[i]->domain_pd) {
                continue;
            }

            EC_WRITE_U8(slaves_group[i]->domain_pd + slaves_group[i]->modes_of_operation,
                        MOTOR_OPMODE_CSP);

            printf_debug("slave %d madht: act position = %d, status = 0x%x, opmode = 0x%x, target = %d\n",
                         slaves_group[i]->alias, slaves_group[i]->curpos,
                         slaves_group[i]->status, slaves_group[i]->opmode,
                         slaves_group[i]->user_set_pos);

            if (!slave_process_data_ready(slaves_group[i])) {
                slave_request_takeover(slaves_group[i]);
                continue;
            }

            if (slaves_group[i]->status & 0x0008) {
                EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word, 0x0080);
                slave_request_takeover(slaves_group[i]);
                continue;
            }

            if (slaves_group[i]->takeover_required &&
                slaves_group[i]->takeover_reset_cycles > 0 &&
                !slave_can_skip_takeover_reset(slaves_group[i])) {
                slave_hold_current_position(slaves_group[i]);
                EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word,
                             0x0006);
                slaves_group[i]->takeover_reset_cycles--;
                continue;
            }

            if (slaves_group[i]->takeover_required &&
                slave_can_skip_takeover_reset(slaves_group[i])) {
                slaves_group[i]->takeover_reset_cycles = 0;
            }

            if ((slaves_group[i]->status & 0x004f) == 0x0040) {
                slave_mark_not_ready(slaves_group[i]);
                EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word, 0x0006);
                continue;
            }

            if ((slaves_group[i]->status & 0x006f) == 0x0021) {
                slave_mark_not_ready(slaves_group[i]);
                EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word, 0x0007);
                continue;
            }

            if ((slaves_group[i]->status & 0x006f) == 0x0007) {
                slave_mark_not_ready(slaves_group[i]);
                slave_hold_current_position(slaves_group[i]);
                EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word,
                             MOTOR_CW_ENABLE);
                continue;
            }

            if ((slaves_group[i]->status & 0x006f) == 0x0023) {
                if (!slaves_group[i]->target_pos_valid) {
                    slave_seed_target_from_current(slaves_group[i]);
                }
                slave_mark_not_ready(slaves_group[i]);
                if (slaves_group[i]->takeover_required) {
                    slave_hold_current_position(slaves_group[i]);
                } else {
                    slaves_group[i]->user_set_pos =
                        step_towards_target(slaves_group[i]);
                    EC_WRITE_S32(slaves_group[i]->domain_pd + slaves_group[i]->target_position,
                                 slaves_group[i]->user_set_pos);
                }
                EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word,
                             MOTOR_CW_ENABLE);
                continue;
            }

            if (slave_status_is_operation_enabled(slaves_group[i]->status)) {
                if (!slaves_group[i]->target_pos_valid) {
                    slave_seed_target_from_current(slaves_group[i]);
                }
                if (slaves_group[i]->takeover_required ||
                    slaves_group[i]->opmode != MOTOR_OPMODE_CSP) {
                    slaves_group[i]->drive_ready = false;
                    slave_hold_current_position(slaves_group[i]);
                    EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word,
                                 MOTOR_CW_ENABLE);
                    if (slaves_group[i]->opmode == MOTOR_OPMODE_CSP) {
                        if (slaves_group[i]->ready_stable_cycles <
                            MOTOR_READY_STABLE_CYCLES) {
                            slaves_group[i]->ready_stable_cycles++;
                        }
                        if (slaves_group[i]->ready_stable_cycles >=
                            MOTOR_READY_STABLE_CYCLES) {
                            slaves_group[i]->takeover_required = false;
                            slaves_group[i]->drive_ready = true;
                            printf("slave %d: takeover complete status=0x%04x opmode=%d pos=%d\n",
                                   slaves_group[i]->alias, slaves_group[i]->status,
                                   slaves_group[i]->opmode, slaves_group[i]->curpos);
                        }
                    } else {
                        slaves_group[i]->ready_stable_cycles = 0;
                    }
                    continue;
                }

                slaves_group[i]->drive_ready = true;
                if (slaves_group[i]->change_pos) {
                    slaves_group[i]->change_pos = false;
                    printf("slave %d: new target position %d\n",
                           slaves_group[i]->alias, slaves_group[i]->user_target_pos);
                }
                slaves_group[i]->user_set_pos =
                    step_towards_target(slaves_group[i]);
                EC_WRITE_S32(slaves_group[i]->domain_pd + slaves_group[i]->target_position,
                             slaves_group[i]->user_set_pos);
                EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word,
                             MOTOR_CW_ENABLE);
                continue;
            }

            slave_request_takeover(slaves_group[i]);
            slave_hold_current_position(slaves_group[i]);
            EC_WRITE_U16(slaves_group[i]->domain_pd + slaves_group[i]->control_word, 0x0006);
        }
        }

        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);
        // send process data
        for (i = 0; i < slaves_cnt; i++) {
            if (!slaves_group[i] || !slaves_group[i]->domain) {
                continue;
            }
            ecrt_domain_queue(slaves_group[i]->domain);
        }
        ecrt_master_send(master);
    }

    return NULL;
}

int MADHT1505BA1_slave_start(int cnt, ...) {
	int err;
    int maxpri;
    int i;
    struct sched_param param;
    pthread_attr_t attr;
    bool attr_ready = false;
    __builtin_va_list vaptr;
    __builtin_va_start(vaptr, cnt);
    run = true;
    if (cnt > SLAVES_NUM_MAX) {
        printf("The number of slave stations set by the user exceeds the supported maximum (%d)\n",
               SLAVES_NUM_MAX);
        return -1;
    }
    if (cnt > master_state.slaves_responding) {
        printf("Configured %d slave(s), but only %u responding yet; continue and wait for bus scan\n",
               cnt, master_state.slaves_responding);
    }
    
    slaves_cnt = cnt;

    for (i = 0; i < cnt; i++) {
        slaves_group[i] = __builtin_va_arg(vaptr, MADHT1505BA1_object *);
    }
    __builtin_va_end(vaptr);

    master_prime_process_data();

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall() failed");
    }

    memset(&param, 0, sizeof(param));
    maxpri = sched_get_priority_max(SCHED_FIFO);
    if (maxpri < 0) {
        printf("sched_get_priority_max() failed\n");
    } else {
        param.sched_priority = MOTOR_RT_PRIORITY;
        if (param.sched_priority > maxpri) {
            param.sched_priority = maxpri;
        }
        if (pthread_attr_init(&attr) == 0) {
            if (pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE) == 0 &&
                pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) == 0 &&
                pthread_attr_setschedpolicy(&attr, SCHED_FIFO) == 0 &&
                pthread_attr_setschedparam(&attr, &param) == 0) {
                attr_ready = true;
            } else {
                pthread_attr_destroy(&attr);
            }
        }
    }

	err = pthread_create(&thread, attr_ready ? &attr : NULL,
                         slave_position_mode_pthread, NULL);
    if (attr_ready) {
        pthread_attr_destroy(&attr);
    }
	if(err != 0) {
		printf("MADHT1505BA1_drive_slave: can't create thread\n");
		return -1;
	}else {
        thread_started = true;
		return 0;
	}
}

// int MADHT1505BA1_motor_start(MADHT1505BA1_object *object, int velocity) {
//     uint16_t    status;
//     status = EC_READ_U16(object->domain_pd + object->status_word);
//     if(status == 0x1237) {
//         object->change_velocity = true;
//         object->user_velocity = velocity;
//     }else {
//         printf("slave %d not start\n", object->alias);
//     }
//     return 0;
// }

// int MADHT1505BA1_motor_stop(MADHT1505BA1_object *object) {
//     uint16_t    status;
//     status = EC_READ_U16(object->domain_pd + object->status_word);
//     if(status == 0x1237) {
//         object->change_velocity = true;
//         object->user_velocity = 0;
//     }else {
//         printf("slave %d not start\n", object->alias);
//     }
//     return 0;
// }

int MADHT1505BA1_check_motor(MADHT1505BA1_object *object) {
    uint16_t    status;
    uint8_t opmode;
    if (!object || !object->domain_pd) {
        return -1;
    }
    if (!object->drive_ready) {
        return -1;
    }
    if (!slave_process_data_ready(object)) {
        return -1;
    }
    status = EC_READ_U16(object->domain_pd + object->status_word);
    opmode = EC_READ_U8(object->domain_pd + object->modes_of_operation_display);
    if ((status & 0x006f) != 0x0027) {
        return -1;
    }
    if (opmode != MOTOR_OPMODE_CSP) {
        return -1;
    }
    return 0;
}

uint32_t MADHT1505BA1_time_statistics_latency_min_ns(void) {
    return latency_min_ns;
}

uint32_t MADHT1505BA1_time_statistics_latency_max_ns(void) {
    return latency_max_ns;
}

uint32_t MADHT1505BA1_time_statistics_period_min_ns(void) {
    return period_min_ns;
}

uint32_t MADHT1505BA1_time_statistics_period_max_ns(void) {
    return period_max_ns;
}

int MADHT1505BA1_run_position_acquisition(MADHT1505BA1_object *object) {
    if (!object) {
        return 0;
    }
    return object->curpos;
}

void MADHT1505BA1_motor_set_position_run(int user_position, MADHT1505BA1_object *object) {
    if (!object) {
        return;
    }
    object->change_pos = true;
    object->user_target_pos = user_position;
    if (object->step_numerator == 0 || object->step_denominator == 0) {
        object->user_set_pos = user_position;
    }
}

void MADHT1505BA1_position_reset(MADHT1505BA1_object *object) {
    if (!object) {
        return;
    }
    object->change_pos = true;
    object->user_target_pos = 0;
    if (object->step_numerator == 0 || object->step_denominator == 0) {
        object->user_set_pos = 0;
    }
}
