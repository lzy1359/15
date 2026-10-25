/*
 * Copyright (c) 2024 Rockchip, Inc. All Rights Reserved.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
#define _GNU_SOURCE
#include <lvgl/lvgl.h>
#include <lvgl/lv_conf.h>
#include <lvgl/src/core/lv_refr.h>
#include <lvgl/src/draw/lv_img_decoder.h>
#include <lvgl/src/misc/lv_mem.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sched.h>
#include <pthread.h>

#include <im2d.h>

#include "main.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "ml_label.h"
#include "motor_control_client.h"
#include "soc.h"
#include "ui_scaler.h"

#define START_ANGLE         90
#define MAX_SPEED           ((METER_TICKS - 1) * 1000)
#define ANGLE_TO_SPEED(x)   ((x) * (METER_TICKS - 1) * 1000 / max_angle)
#define SPEED_TO_ANGLE(x)   ((x) * max_angle / (METER_TICKS - 1) / 1000)
#define SCALER_RESIZE(x)    ui_scaler_calc(ui_scaler, x)
#define UI_INPUT_BUILD_DELAY_MS   20
#define UI_INPUT_RETRY_DELAY_MS   50
#define ENABLE_NEEDLE_RGA_PRE_SCALE 0

static float max_angle = 360.0;
static struct motor_control_state motor_state;

struct axis_ui
{
    lv_obj_t *cont;
    lv_obj_t *light;
    lv_obj_t *label;
    lv_obj_t *pos;
    lv_obj_t *pos_val;
    lv_obj_t *target;
    lv_obj_t *target_val;
    int axis_id;
    int txt_idx;
};

lv_ft_info_t ttf_main;
lv_ft_info_t ttf_main_l;
lv_ft_info_t ttf_main_s;
lv_ft_info_t ttf_main_m;
lv_ft_info_t ttf_main_xs;
static lv_obj_t *scr;
static lv_obj_t *label_title;
static lv_obj_t *btn_overview;
static lv_obj_t *btn_reset;
static lv_obj_t *btn_axissync;
static lv_obj_t *btn_alart;
static lv_obj_t *label_overview;
static lv_obj_t *label_autolocate;
static lv_obj_t *label_axissync;
static lv_obj_t *label_alart;
static lv_obj_t *position_cont;
static lv_obj_t *label_pos_ctrl_target;
static lv_obj_t *page_overview;
static lv_obj_t *language_switch;
static lv_obj_t *label_soc;
static struct meter *pos_meter;
static lv_obj_t *panel_header;
static lv_obj_t *panel_body;
static lv_obj_t *panel_main_wrap;
static lv_obj_t *panel_boot_card;
static lv_obj_t *panel_meter;
static lv_obj_t *panel_status;
static lv_obj_t *panel_footer;
static lv_obj_t *panel_meter_placeholder;
static lv_obj_t *meter_wrap;
static lv_obj_t *label_state_pill;
static lv_obj_t *label_state_pill_value;
static lv_obj_t *label_target_hint;
static lv_obj_t *label_status_cur_pos;
static lv_obj_t *label_status_target_pos;
static lv_obj_t *label_status_run_state;
static lv_obj_t *label_status_bus_state;
static lv_obj_t *label_status_ctrl_mode;
static lv_obj_t *label_status_status_word;
static lv_obj_t *label_status_servo;
static lv_obj_t *label_status_inpos;
static lv_obj_t *label_status_fault;
static lv_obj_t *label_status_comm;
static lv_obj_t *label_status_jitter;
static lv_obj_t *label_status_cyclic_frequency;

/* 0 for Axis-X, 1 for Axis-Y */
static struct axis_ui axiss[MOTOR_CONTROL_AXIS_COUNT];
static struct axis_ui *position_ctrl_target = NULL;

static int quit = 0;
static int ui_target_angle = 0;
static char *compatible_name;
static char *kernel_version;
static char *soc_name;
static char *sys_version;

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

static void *ui_scaler;
static bool ui_ready;
static bool ui_fonts_ready;
static bool ui_detail_ready;
static bool ui_visuals_ready;
static bool ui_input_ready;
static uint64_t ui_input_retry_log_deadline_ms;
static lv_img_dsc_t g_needle_rga_dsc;
static uint8_t *g_needle_rga_buf;
static uint16_t g_needle_rga_zoom;

static void ui_build_input_timer_cb(lv_timer_t *timer);
static void reset_cb(lv_event_t *e);
static void sync_cb(lv_event_t *e);
static void target_minus_cb(lv_event_t *e);
static void target_plus_cb(lv_event_t *e);
static void target_preset_cb(lv_event_t *e);
static void one_key_test_cb(lv_event_t *e);
static void demo_forward_cb(lv_event_t *e);
static void demo_reverse_cb(lv_event_t *e);
static void stop_motion_cb(lv_event_t *e);
static struct meter *meter_create(lv_obj_t *parent);
static void meter_set_needle_angle(struct meter *meter, int16_t angle);
static lv_obj_t *panel_create(lv_obj_t *parent, lv_coord_t w, lv_coord_t h);
static lv_obj_t *button_area_create(lv_obj_t *parent, lv_coord_t w,
                                    lv_coord_t h);
static lv_obj_t *status_row_create(lv_obj_t *parent);
static lv_obj_t *status_card_create(lv_obj_t *parent, int title_idx,
                                    lv_obj_t **value_label);
static lv_obj_t *action_button_create(lv_obj_t *parent, const char *text,
                                      lv_event_cb_t cb, void *user_data,
                                      lv_color_t color);
static void display_motor_information(void);

static uint64_t monotonic_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static const char *safe_string(const char *value, const char *fallback)
{
    return value ? value : fallback;
}

static const lv_font_t *safe_font(const lv_ft_info_t *info)
{
    return info->font ? info->font : LV_FONT_DEFAULT;
}

static bool meter_prepare_needle_image(struct meter *meter, const char *src_path,
                                       uint16_t zoom)
{
    lv_img_decoder_dsc_t decoder;
    uint8_t *scaled = NULL;
    int src_w;
    int src_h;
    int dst_w;
    int dst_h;
    int src_size;
    int dst_size;
    IM_STATUS status;
    im_handle_param_t src_param;
    im_handle_param_t dst_param;
    rga_buffer_handle_t src_handle = 0;
    rga_buffer_handle_t dst_handle = 0;
    rga_buffer_t src;
    rga_buffer_t dst;

    if (!meter || !src_path || zoom == 0 || zoom == LV_IMG_ZOOM_NONE)
        return false;

    if (g_needle_rga_buf && g_needle_rga_zoom == zoom)
    {
        meter->needle_dsc = g_needle_rga_dsc;
        meter->needle_buf = g_needle_rga_buf;
        meter->needle_rga_scaled = 1;
        return true;
    }

#if LV_COLOR_DEPTH != 32
    printf("motor_demo: RGA needle scaling requires LV_COLOR_DEPTH=32\n");
    return false;
#endif

    memset(&decoder, 0, sizeof(decoder));
    if (lv_img_decoder_open(&decoder, src_path, lv_color_white(), 0) != LV_RES_OK)
    {
        printf("motor_demo: failed to decode %s for RGA scaling\n", src_path);
        return false;
    }

    if (!decoder.img_data || decoder.header.cf != LV_IMG_CF_TRUE_COLOR_ALPHA)
    {
        printf("motor_demo: unsupported needle format cf=%u\n",
               decoder.header.cf);
        lv_img_decoder_close(&decoder);
        return false;
    }

    src_w = decoder.header.w;
    src_h = decoder.header.h;
    dst_w = (src_w * zoom + LV_IMG_ZOOM_NONE / 2) / LV_IMG_ZOOM_NONE;
    dst_h = (src_h * zoom + LV_IMG_ZOOM_NONE / 2) / LV_IMG_ZOOM_NONE;
    if (dst_w < 1)
        dst_w = 1;
    if (dst_h < 1)
        dst_h = 1;

    if (dst_w == src_w && dst_h == src_h)
    {
        lv_img_decoder_close(&decoder);
        return false;
    }

    src_size = LV_IMG_BUF_SIZE_TRUE_COLOR_ALPHA(src_w, src_h);
    dst_size = LV_IMG_BUF_SIZE_TRUE_COLOR_ALPHA(dst_w, dst_h);

    scaled = lv_mem_alloc(dst_size);
    if (!scaled)
    {
        printf("motor_demo: needle RGA buffer allocation failed\n");
        lv_img_decoder_close(&decoder);
        return false;
    }

    memset(&src_param, 0, sizeof(src_param));
    memset(&dst_param, 0, sizeof(dst_param));
    src_param.width = src_w;
    src_param.height = src_h;
    src_param.format = RK_FORMAT_ARGB_8888;
    dst_param.width = dst_w;
    dst_param.height = dst_h;
    dst_param.format = RK_FORMAT_ARGB_8888;

    src_handle = importbuffer_virtualaddr((void *)decoder.img_data, &src_param);
    dst_handle = importbuffer_virtualaddr(scaled, &dst_param);
    if (!src_handle || !dst_handle)
    {
        printf("motor_demo: importbuffer_virtualaddr failed src=%d dst=%d err=%s\n",
               src_handle, dst_handle, imStrError());
        if (src_handle)
            releasebuffer_handle(src_handle);
        if (dst_handle)
            releasebuffer_handle(dst_handle);
        lv_mem_free(scaled);
        lv_img_decoder_close(&decoder);
        return false;
    }

    src = wrapbuffer_handle(src_handle, src_w, src_h, RK_FORMAT_ARGB_8888);
    dst = wrapbuffer_handle(dst_handle, dst_w, dst_h, RK_FORMAT_ARGB_8888);
    status = imcheck(src, dst, (im_rect){0}, (im_rect){0});
    if (status != IM_STATUS_NOERROR)
    {
        printf("motor_demo: RGA needle imcheck failed: %s\n", imStrError(status));
        releasebuffer_handle(src_handle);
        releasebuffer_handle(dst_handle);
        lv_mem_free(scaled);
        lv_img_decoder_close(&decoder);
        return false;
    }

    status = imresize(src, dst, 0, 0, INTER_LINEAR, 1);
    releasebuffer_handle(src_handle);
    releasebuffer_handle(dst_handle);
    if (status < 0)
    {
        printf("motor_demo: RGA needle resize failed: %s\n",
               imStrError(status));
        lv_mem_free(scaled);
        lv_img_decoder_close(&decoder);
        return false;
    }

    memset(&g_needle_rga_dsc, 0, sizeof(g_needle_rga_dsc));
    g_needle_rga_dsc.header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA;
    g_needle_rga_dsc.header.always_zero = 0;
    g_needle_rga_dsc.header.reserved = 0;
    g_needle_rga_dsc.header.w = dst_w;
    g_needle_rga_dsc.header.h = dst_h;
    g_needle_rga_dsc.data_size = dst_size;
    g_needle_rga_dsc.data = scaled;
    g_needle_rga_buf = scaled;
    g_needle_rga_zoom = zoom;
    meter->needle_dsc = g_needle_rga_dsc;
    meter->needle_buf = g_needle_rga_buf;
    meter->needle_rga_scaled = 1;

    printf("motor_demo: RGA pre-scaled needle %dx%d -> %dx%d (zoom=%u)\n",
           src_w, src_h, dst_w, dst_h, zoom);

    lv_img_decoder_close(&decoder);
    return true;
}

static void meter_set_needle_angle(struct meter *meter, int16_t angle)
{
    if (!meter || !meter->needle)
        return;

    while (angle >= 3600)
        angle -= 3600;
    while (angle < 0)
        angle += 3600;

    meter->needle_angle = angle;
    lv_img_set_angle(meter->needle, angle);
}

static const char *format_kernel_summary(const char *version)
{
    static char summary[128];
    const char *value = safe_string(version, "unknown");

    if (strstr(value, "-rt"))
        snprintf(summary, sizeof(summary), "PREEMPT_RT %s", value);
    else
        snprintf(summary, sizeof(summary), "Kernel %s", value);

    return summary;
}

static void check_scr(void)
{
    printf("motor_demo: screen %dx%d\n", LV_HOR_RES, LV_VER_RES);
}

static bool motor_available(void)
{
#if ENABLE_MOTOR_CONTROL
    return motor_state.ready;
#else
    return false;
#endif
}

static void motor_state_reset(void)
{
    memset(&motor_state, 0, sizeof(motor_state));
}

static int clamp_target_angle(int angle)
{
    if (angle < 0)
        return 0;
    if (angle > 360)
        return 360;
    return angle;
}

static void set_target_widgets(int angle)
{
    angle = clamp_target_angle(angle);
    ui_target_angle = angle;
}

static int motor_get_state(void)
{
#if ENABLE_MOTOR_CONTROL
    int status = motor_control_get_state(&motor_state);

    if (status != MOTOR_CONTROL_STATUS_OK)
        motor_state_reset();

    return status;
#else
    motor_state_reset();
    return MOTOR_CONTROL_STATUS_UNAVAILABLE;
#endif
}

static bool motor_command_ok(int status, const char *action)
{
    if (status == MOTOR_CONTROL_STATUS_OK)
        return true;

    printf("motor_demo: %s failed (%d)\n", action, status);
    return false;
}

static void set_axis_indicator(struct axis_ui *axis, bool online)
{
    if (!axis || !axis->light)
        return;

    if (online)
    {
        lv_led_on(axis->light);
        lv_obj_set_style_bg_color(axis->light, lv_color_hex(0x24d957),
                                  LV_PART_MAIN);
    }
    else
    {
        lv_led_off(axis->light);
        lv_obj_set_style_bg_color(axis->light, lv_color_hex(0x242424),
                                  LV_PART_MAIN);
    }
}

static void apply_target_angle(int angle, const char *action)
{
#if ENABLE_MOTOR_CONTROL
    angle = clamp_target_angle(angle);
    if (!motor_command_ok(
            motor_control_set_position(0, angle, &motor_state), action))
        return;
    set_target_widgets(motor_state.axis_target[0]);
#else
    angle = clamp_target_angle(angle);
    set_target_widgets(angle);
#endif
}

static void apply_target_delta(int delta)
{
#if ENABLE_MOTOR_CONTROL
    if (!motor_command_ok(
            motor_control_move_relative(0, delta, &motor_state),
            delta > 0 ? "target+" : "target-"))
        return;
    set_target_widgets(motor_state.axis_target[0]);
#else
    set_target_widgets(ui_target_angle + delta);
#endif
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
    for (i = 0; i < CPU_SETSIZE; i++)
    {
        if (CPU_ISSET(i, &mask))
        {
            printf(" %d", i);
            break;
        }
    }
    printf("\n");

    return i >= cpu_num ? -1 : i;
}

static void configure_process_runtime(void)
{
    struct sched_param param;
    int cpu;
    int policy;
    int rt_prio;

    if (env_get_int("MOTOR_UI_CPU", &cpu) && thread_bind_cpu(cpu) == -1)
        printf("motor_demo: bind ui thread to cpu%d failed\n", cpu);

    policy = sched_getscheduler(0);
    if (policy == -1)
    {
        perror("motor_demo: sched_getscheduler failed");
        return;
    }

    /*
     * The stack launcher already decides CPU affinity and whether the UI should
     * run under RT scheduling. Preserve externally applied FIFO/RR settings,
     * but do not silently promote the UI to FIFO on housekeeping CPUs.
     */
    if (!env_get_int("MOTOR_UI_RT_PRIO", &rt_prio))
    {
        if (policy == SCHED_FIFO || policy == SCHED_RR)
            return;
        return;
    }

    if (rt_prio <= 0)
        return;

    memset(&param, 0, sizeof(param));
    param.sched_priority = rt_prio;
    if (sched_setscheduler(getpid(), SCHED_FIFO, &param) == -1)
        perror("motor_demo: sched_setscheduler failed");
}

static void sigterm_handler(int sig)
{
    fprintf(stderr, "signal %d\n", sig);
    quit = 1;
}

static void font_init(void)
{
    uint64_t start_ms = monotonic_ms();

    if (!lv_freetype_init(64, 1, 0))
        printf("motor_demo: lv_freetype_init failed, using fallback font\n");

    ttf_main.weight = SCALER_RESIZE(68);
    ttf_main.name = SRC_FONT("SmileySans-Oblique.ttf");
    ttf_main.style = FT_FONT_STYLE_NORMAL;
    if (!lv_ft_font_init(&ttf_main))
    {
        ttf_main.font = (lv_font_t *)LV_FONT_DEFAULT;
        printf("motor_demo: failed to load %s\n", ttf_main.name);
    }

    ttf_main_l.weight = SCALER_RESIZE(92);
    ttf_main_l.name = SRC_FONT("SmileySans-Oblique.ttf");
    ttf_main_l.style = FT_FONT_STYLE_NORMAL;
    if (!lv_ft_font_init(&ttf_main_l))
    {
        ttf_main_l.font = (lv_font_t *)LV_FONT_DEFAULT;
        printf("motor_demo: failed to load %s\n", ttf_main_l.name);
    }

    ttf_main_s.weight = SCALER_RESIZE(48);
    ttf_main_s.name = SRC_FONT("SmileySans-Oblique.ttf");
    ttf_main_s.style = FT_FONT_STYLE_NORMAL;
    if (!lv_ft_font_init(&ttf_main_s))
    {
        ttf_main_s.font = (lv_font_t *)LV_FONT_DEFAULT;
        printf("motor_demo: failed to load %s\n", ttf_main_s.name);
    }

    ttf_main_m.weight = SCALER_RESIZE(38);
    ttf_main_m.name = SRC_FONT("SmileySans-Oblique.ttf");
    ttf_main_m.style = FT_FONT_STYLE_NORMAL;
    if (!lv_ft_font_init(&ttf_main_m))
    {
        ttf_main_m.font = (lv_font_t *)LV_FONT_DEFAULT;
        printf("motor_demo: failed to load %s\n", ttf_main_m.name);
    }

    ttf_main_xs.weight = SCALER_RESIZE(32);
    ttf_main_xs.name = SRC_FONT("SmileySans-Oblique.ttf");
    ttf_main_xs.style = FT_FONT_STYLE_NORMAL;
    if (!lv_ft_font_init(&ttf_main_xs))
    {
        ttf_main_xs.font = (lv_font_t *)LV_FONT_DEFAULT;
        printf("motor_demo: failed to load %s\n", ttf_main_xs.name);
    }

    printf("motor_demo: font_init finished in %" PRIu64 " ms\n",
           monotonic_ms() - start_ms);
}

static void lvgl_init_display(void)
{
    uint64_t start_ms = monotonic_ms();

    lv_init();
    lv_port_disp_init(0, 0, 0);
    if (!lv_disp_get_default())
    {
        fprintf(stderr, "motor_demo: display init failed, no default display\n");
        exit(EXIT_FAILURE);
    }
    check_scr();
    printf("motor_demo: display init finished in %" PRIu64 " ms\n",
           monotonic_ms() - start_ms);

    ui_scaler = ui_scaler_new(1920, 1080);
    ui_scaler_set_refer_size(ui_scaler, LV_HOR_RES, LV_VER_RES);
}

static int lvgl_init_input(void)
{
    uint64_t start_ms = monotonic_ms();
    int ret;

    if (ui_input_ready)
        return 0;

    ret = lv_port_indev_init(0);
    if (ret != 0)
    {
        if (start_ms >= ui_input_retry_log_deadline_ms)
        {
            printf("motor_demo: input init deferred, retrying\n");
            ui_input_retry_log_deadline_ms = start_ms + 500;
        }
        return -1;
    }

    ui_input_ready = true;
    ui_input_retry_log_deadline_ms = 0;
    printf("motor_demo: input init finished in %" PRIu64 " ms\n",
           monotonic_ms() - start_ms);
    return 0;
}

static void ui_prepare_runtime_info(void)
{
    if (compatible_name)
        return;

    compatible_name = get_compatible_name();
    soc_name = get_soc_name(compatible_name);
    kernel_version = get_kernel_version(NULL);
    sys_version = get_system_version();
}

static void ui_reset_objects(void)
{
    label_title = NULL;
    btn_overview = NULL;
    btn_reset = NULL;
    btn_axissync = NULL;
    btn_alart = NULL;
    label_overview = NULL;
    label_autolocate = NULL;
    label_axissync = NULL;
    label_alart = NULL;
    position_cont = NULL;
    label_pos_ctrl_target = NULL;
    page_overview = NULL;
    language_switch = NULL;
    label_soc = NULL;
    pos_meter = NULL;
    panel_header = NULL;
    panel_body = NULL;
    panel_main_wrap = NULL;
    panel_boot_card = NULL;
    panel_meter = NULL;
    panel_status = NULL;
    panel_footer = NULL;
    panel_meter_placeholder = NULL;
    meter_wrap = NULL;
    label_state_pill = NULL;
    label_state_pill_value = NULL;
    label_target_hint = NULL;
    label_status_cur_pos = NULL;
    label_status_target_pos = NULL;
    label_status_run_state = NULL;
    label_status_bus_state = NULL;
    label_status_ctrl_mode = NULL;
    label_status_status_word = NULL;
    label_status_servo = NULL;
    label_status_inpos = NULL;
    label_status_fault = NULL;
    label_status_comm = NULL;
    label_status_jitter = NULL;
    label_status_cyclic_frequency = NULL;
    ui_ready = false;
    ui_fonts_ready = false;
    ui_detail_ready = false;
    ui_visuals_ready = false;
    ui_input_ready = false;
    ui_input_retry_log_deadline_ms = 0;
}

static void ui_apply_shell_fonts(void)
{
    if (!ui_fonts_ready)
        return;

    if (label_title)
        lv_obj_set_style_text_font(label_title, safe_font(&ttf_main),
                                   LV_PART_MAIN);
    if (label_soc)
        lv_obj_set_style_text_font(label_soc, safe_font(&ttf_main_s),
                                   LV_PART_MAIN);
    if (label_state_pill_value)
        lv_obj_set_style_text_font(label_state_pill_value,
                                   safe_font(&ttf_main_s), LV_PART_MAIN);
    if (label_target_hint)
        lv_obj_set_style_text_font(label_target_hint, safe_font(&ttf_main_s),
                                   LV_PART_MAIN);
}

static void ui_build_full_header(void)
{
    lv_obj_t *header_left;
    lv_obj_t *header_right;

    ui_prepare_runtime_info();

    panel_header = panel_create(panel_body, lv_pct(100), SCALER_RESIZE(236));
    lv_obj_set_layout(panel_header, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel_header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(panel_header, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(panel_header, SCALER_RESIZE(18), LV_PART_MAIN);

    header_left = lv_obj_create(panel_header);
    lv_obj_remove_style_all(header_left);
    lv_obj_clear_flag(header_left, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(header_left, lv_pct(58), lv_pct(100));
    lv_obj_set_layout(header_left, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(header_left, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(header_left, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_gap(header_left, SCALER_RESIZE(8), LV_PART_MAIN);

    label_title = ml_label_create(header_left, TXT_TITLE);
    lv_obj_set_width(label_title, lv_pct(100));
    lv_obj_set_style_text_color(label_title, lv_color_hex(0x1d2733),
                                LV_PART_MAIN);

    label_soc = lv_label_create(header_left);
    lv_obj_set_width(label_soc, lv_pct(100));
    lv_label_set_long_mode(label_soc, LV_LABEL_LONG_CLIP);
    lv_label_set_text_fmt(label_soc, "%s / %s",
                          safe_string(soc_name, "RK3506"),
                          format_kernel_summary(kernel_version));
    lv_obj_set_style_text_color(label_soc, lv_color_hex(0x7c6d5b),
                                LV_PART_MAIN);
    lv_obj_set_style_pad_top(label_soc, SCALER_RESIZE(4), LV_PART_MAIN);

    header_right = lv_obj_create(panel_header);
    lv_obj_remove_style_all(header_right);
    lv_obj_clear_flag(header_right, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(header_right, lv_pct(38), lv_pct(100));
    lv_obj_set_layout(header_right, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(header_right, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header_right, LV_FLEX_ALIGN_END,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(header_right, SCALER_RESIZE(12), LV_PART_MAIN);

    label_state_pill = lv_obj_create(header_right);
    lv_obj_clear_flag(label_state_pill, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(label_state_pill, lv_pct(100), SCALER_RESIZE(122));
    lv_obj_set_style_radius(label_state_pill, SCALER_RESIZE(20), LV_PART_MAIN);
    lv_obj_set_style_border_width(label_state_pill, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(label_state_pill, lv_color_hex(0x8c8c8c),
                              LV_PART_MAIN);

    label_state_pill_value = lv_label_create(label_state_pill);
    lv_label_set_text(label_state_pill_value, "NOT READY");
    lv_obj_set_style_text_color(label_state_pill_value, lv_color_white(),
                                LV_PART_MAIN);
    lv_obj_center(label_state_pill_value);
}

static void ui_force_present(void)
{
    lv_obj_invalidate(lv_scr_act());
    lv_refr_now(NULL);
}

static void ui_init_light(void)
{
    uint64_t start_ms = monotonic_ms();
    scr = lv_scr_act();
    lv_obj_clean(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xe9e1d2), LV_PART_MAIN);
    ui_reset_objects();

    panel_body = lv_obj_create(scr);
    lv_obj_remove_style_all(panel_body);
    lv_obj_clear_flag(panel_body, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(panel_body, lv_pct(100), lv_pct(100));
    lv_obj_center(panel_body);
    lv_obj_set_layout(panel_body, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel_body, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel_body, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(panel_body, SCALER_RESIZE(18), LV_PART_MAIN);
    lv_obj_set_style_pad_gap(panel_body, SCALER_RESIZE(18), LV_PART_MAIN);

    panel_boot_card = panel_create(panel_body, lv_pct(100), 0);
    lv_obj_set_width(panel_boot_card, lv_pct(100));
    lv_obj_set_height(panel_boot_card, 0);
    lv_obj_set_flex_grow(panel_boot_card, 1);
    lv_obj_set_layout(panel_boot_card, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel_boot_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel_boot_card, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(panel_boot_card, SCALER_RESIZE(18), LV_PART_MAIN);

    label_state_pill = lv_obj_create(panel_boot_card);
    lv_obj_clear_flag(label_state_pill, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(label_state_pill, lv_pct(46), SCALER_RESIZE(118));
    lv_obj_set_style_radius(label_state_pill, SCALER_RESIZE(20), LV_PART_MAIN);
    lv_obj_set_style_border_width(label_state_pill, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(label_state_pill, lv_color_hex(0x8c8c8c),
                              LV_PART_MAIN);

    label_state_pill_value = lv_label_create(label_state_pill);
    lv_label_set_text(label_state_pill_value, "BOOTING");
    lv_obj_set_style_text_color(label_state_pill_value, lv_color_white(),
                                LV_PART_MAIN);
    lv_obj_center(label_state_pill_value);

    label_target_hint = lv_label_create(panel_boot_card);
    lv_obj_set_width(label_target_hint, lv_pct(100));
    lv_label_set_long_mode(label_target_hint, LV_LABEL_LONG_WRAP);
    lv_label_set_text(label_target_hint,
                      "Starting display and motor stack...");
    lv_obj_set_style_text_align(label_target_hint, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_color(label_target_hint, lv_color_hex(0x7c6d5b),
                                LV_PART_MAIN);

    ui_ready = true;
    printf("motor_demo: light ui ready in %" PRIu64 " ms\n",
           monotonic_ms() - start_ms);
}

static void ui_init_detail(void)
{
    uint64_t start_ms = monotonic_ms();
    lv_obj_t *status_grid;
    lv_obj_t *status_row;
    lv_obj_t *control_group;
    lv_obj_t *meter_head;

    if (ui_detail_ready || !panel_body)
        return;

    if (!ui_fonts_ready)
    {
        font_init();
        ui_fonts_ready = true;
    }

    lv_obj_clean(panel_body);
    panel_header = NULL;
    panel_boot_card = NULL;
    panel_main_wrap = NULL;
    panel_meter = NULL;
    panel_status = NULL;
    panel_footer = NULL;
    panel_meter_placeholder = NULL;
    meter_wrap = NULL;
    label_title = NULL;
    label_soc = NULL;
    label_state_pill = NULL;
    label_state_pill_value = NULL;
    label_target_hint = NULL;

    lv_obj_set_size(panel_body, lv_pct(96), lv_pct(98));
    lv_obj_align(panel_body, LV_ALIGN_TOP_MID, 0, SCALER_RESIZE(8));
    lv_obj_set_layout(panel_body, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel_body, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel_body, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(panel_body, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_gap(panel_body, SCALER_RESIZE(10), LV_PART_MAIN);

    ui_build_full_header();
    ui_apply_shell_fonts();

    panel_main_wrap = panel_create(panel_body, lv_pct(100), 0);
    lv_obj_set_width(panel_main_wrap, lv_pct(100));
    lv_obj_set_height(panel_main_wrap, 0);
    lv_obj_set_flex_grow(panel_main_wrap, 1);
    lv_obj_set_layout(panel_main_wrap, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel_main_wrap, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(panel_main_wrap, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(panel_main_wrap, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_gap(panel_main_wrap, 0, LV_PART_MAIN);

    panel_meter = lv_obj_create(panel_main_wrap);
    lv_obj_remove_style_all(panel_meter);
    lv_obj_clear_flag(panel_meter, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(panel_meter, lv_pct(54), lv_pct(100));
    lv_obj_set_layout(panel_meter, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel_meter, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel_meter, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_top(panel_meter, SCALER_RESIZE(6), LV_PART_MAIN);
    lv_obj_set_style_pad_gap(panel_meter, SCALER_RESIZE(10), LV_PART_MAIN);

    meter_head = lv_obj_create(panel_meter);
    lv_obj_remove_style_all(meter_head);
    lv_obj_clear_flag(meter_head, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(meter_head, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_layout(meter_head, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(meter_head, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(meter_head, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(meter_head, SCALER_RESIZE(10), LV_PART_MAIN);

    label_pos_ctrl_target = ml_label_create(meter_head, TXT_AXIS_X);
    lv_obj_set_style_text_color(label_pos_ctrl_target, lv_color_hex(0x7c6d5b),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(label_pos_ctrl_target, safe_font(&ttf_main_s),
                               LV_PART_MAIN);

    meter_wrap = lv_obj_create(panel_meter);
    lv_obj_remove_style_all(meter_wrap);
    lv_obj_clear_flag(meter_wrap, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_width(meter_wrap, lv_pct(100));
    lv_obj_set_height(meter_wrap, 0);
    lv_obj_set_flex_grow(meter_wrap, 1);

    panel_meter_placeholder = lv_obj_create(meter_wrap);
    lv_obj_remove_style_all(panel_meter_placeholder);
    lv_obj_clear_flag(panel_meter_placeholder, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(panel_meter_placeholder, lv_pct(100), lv_pct(100));
    lv_obj_set_layout(panel_meter_placeholder, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel_meter_placeholder, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel_meter_placeholder, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    label_target_hint = lv_label_create(panel_meter_placeholder);
    lv_obj_set_width(label_target_hint, lv_pct(100));
    lv_label_set_long_mode(label_target_hint, LV_LABEL_LONG_WRAP);
    lv_label_set_text(label_target_hint, "Loading gauge...");
    lv_obj_set_style_text_align(label_target_hint, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_text_color(label_target_hint, lv_color_hex(0x7c6d5b),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(label_target_hint, safe_font(&ttf_main_s),
                               LV_PART_MAIN);

    panel_status = lv_obj_create(panel_main_wrap);
    lv_obj_remove_style_all(panel_status);
    lv_obj_clear_flag(panel_status, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(panel_status, lv_pct(46), lv_pct(100));
    lv_obj_set_layout(panel_status, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel_status, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel_status, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_top(panel_status, SCALER_RESIZE(4), LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(panel_status, SCALER_RESIZE(4), LV_PART_MAIN);
    lv_obj_set_style_pad_left(panel_status, SCALER_RESIZE(14), LV_PART_MAIN);
    lv_obj_set_style_pad_right(panel_status, SCALER_RESIZE(8), LV_PART_MAIN);
    lv_obj_set_style_pad_gap(panel_status, SCALER_RESIZE(6), LV_PART_MAIN);

    status_grid = lv_obj_create(panel_status);
    lv_obj_remove_style_all(status_grid);
    lv_obj_clear_flag(status_grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_width(status_grid, lv_pct(100));
    lv_obj_set_height(status_grid, 0);
    lv_obj_set_flex_grow(status_grid, 1);
    lv_obj_set_layout(status_grid, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(status_grid, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(status_grid, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_gap(status_grid, SCALER_RESIZE(4), LV_PART_MAIN);

    status_row = status_row_create(status_grid);
    status_card_create(status_row, TXT_CUR_POS, &label_status_cur_pos);
    status_card_create(status_row, TXT_TAR_POS, &label_status_target_pos);
    lv_obj_set_style_text_font(label_status_cur_pos, safe_font(&ttf_main_s),
                               LV_PART_MAIN);
    lv_obj_set_style_text_font(label_status_target_pos, safe_font(&ttf_main_s),
                               LV_PART_MAIN);

    status_row = status_row_create(status_grid);
    status_card_create(status_row, TXT_RUN_STATE, &label_status_run_state);
    status_card_create(status_row, TXT_BUS_STATE, &label_status_bus_state);
    status_row = status_row_create(status_grid);
    status_card_create(status_row, TXT_CTRL_MODE, &label_status_ctrl_mode);
    status_card_create(status_row, TXT_STATUS_WORD, &label_status_status_word);
    status_row = status_row_create(status_grid);
    status_card_create(status_row, TXT_SERVO_STATE, &label_status_servo);
    status_card_create(status_row, TXT_INPOS_STATE, &label_status_inpos);
    status_row = status_row_create(status_grid);
    status_card_create(status_row, TXT_FAULT_STATE, &label_status_fault);
    status_card_create(status_row, TXT_COMM_STATE, &label_status_comm);
    status_row = status_row_create(status_grid);
    status_card_create(status_row, TXT_JITTER, &label_status_jitter);
    status_card_create(status_row, TXT_CYCLIC_FREQUENCY,
                       &label_status_cyclic_frequency);

    panel_footer = panel_create(panel_body, lv_pct(100), SCALER_RESIZE(360));
    lv_obj_set_layout(panel_footer, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel_footer, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel_footer, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(panel_footer, SCALER_RESIZE(12), LV_PART_MAIN);

    control_group = button_area_create(panel_footer, lv_pct(100), lv_pct(47));
    btn_reset = action_button_create(control_group, "零点校准", reset_cb, NULL,
                                     lv_color_hex(0x457b9d));
    lv_obj_set_size(btn_reset, lv_pct(24), lv_pct(100));
    btn_axissync = action_button_create(control_group, "回零", sync_cb, NULL,
                                        lv_color_hex(0x2a9d8f));
    lv_obj_set_size(btn_axissync, lv_pct(24), lv_pct(100));
    btn_overview = action_button_create(control_group, "-10\xC2\xB0",
                                        target_minus_cb, NULL,
                                        lv_color_hex(0xb56576));
    lv_obj_set_size(btn_overview, lv_pct(24), lv_pct(100));
    btn_alart = action_button_create(control_group, "+10\xC2\xB0",
                                     target_plus_cb, NULL,
                                     lv_color_hex(0xe76f51));
    lv_obj_set_size(btn_alart, lv_pct(24), lv_pct(100));

    control_group = button_area_create(panel_footer, lv_pct(100), lv_pct(47));
    btn_overview = action_button_create(control_group, "一键测试",
                                        one_key_test_cb, NULL,
                                        lv_color_hex(0x6d597a));
    lv_obj_set_size(btn_overview, lv_pct(24), lv_pct(100));
    btn_alart = action_button_create(control_group, "正转展示",
                                     demo_forward_cb, NULL,
                                     lv_color_hex(0x355070));
    lv_obj_set_size(btn_alart, lv_pct(24), lv_pct(100));
    btn_overview = action_button_create(control_group, "反转展示",
                                        demo_reverse_cb, NULL,
                                        lv_color_hex(0x588157));
    lv_obj_set_size(btn_overview, lv_pct(24), lv_pct(100));
    btn_alart = action_button_create(control_group, "停止",
                                     stop_motion_cb, NULL,
                                     lv_color_hex(0xbc4b36));
    lv_obj_set_size(btn_alart, lv_pct(24), lv_pct(100));

    set_target_widgets(0);
    ui_detail_ready = true;
    display_motor_information();
    printf("motor_demo: detail ui ready in %" PRIu64 " ms\n",
           monotonic_ms() - start_ms);
}

static void ui_init_visuals(void)
{
    uint64_t start_ms = monotonic_ms();

    if (ui_visuals_ready || !panel_meter)
        return;

    if (panel_meter_placeholder)
    {
        lv_obj_del(panel_meter_placeholder);
        panel_meter_placeholder = NULL;
        label_target_hint = NULL;
    }

    pos_meter = meter_create(meter_wrap);
    ui_visuals_ready = true;
    display_motor_information();
    printf("motor_demo: heavy visuals ready in %" PRIu64 " ms\n",
           monotonic_ms() - start_ms);
}

static void overview_cb(lv_event_t *e)
{
    if (position_cont)
        lv_obj_add_flag(position_cont, LV_OBJ_FLAG_HIDDEN);
    if (!page_overview)
        return;

    if (lv_obj_has_flag(page_overview, LV_OBJ_FLAG_HIDDEN))
        lv_obj_clear_flag(page_overview, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(page_overview, LV_OBJ_FLAG_HIDDEN);
}

static void scr_cb(lv_event_t *e)
{
    if (page_overview)
        lv_obj_add_flag(page_overview, LV_OBJ_FLAG_HIDDEN);
    if (position_cont)
        lv_obj_add_flag(position_cont, LV_OBJ_FLAG_HIDDEN);
}

static void reset_cb(lv_event_t *e)
{
    (void)e;

#if ENABLE_MOTOR_CONTROL
    if (!motor_command_ok(motor_control_reset_all(&motor_state), "calibrate zero"))
        return;
#endif
    set_target_widgets(0);
}

static void close_cb(lv_event_t *e)
{
    if (position_cont)
        lv_obj_add_flag(position_cont, LV_OBJ_FLAG_HIDDEN);
}

static void sync_cb(lv_event_t *e)
{
    (void)e;

#if ENABLE_MOTOR_CONTROL
    if (!motor_command_ok(motor_control_sync_all(&motor_state), "return zero"))
        return;
#endif
    set_target_widgets(0);
}

static void target_minus_cb(lv_event_t *e)
{
    (void)e;
    apply_target_delta(-10);
}

static void target_plus_cb(lv_event_t *e)
{
    (void)e;
    apply_target_delta(10);
}

static void target_step_cb(lv_event_t *e)
{
    int delta = (int)(intptr_t)lv_event_get_user_data(e);

    apply_target_delta(delta);
}

static void target_preset_cb(lv_event_t *e)
{
    int angle = (int)(intptr_t)lv_event_get_user_data(e);

    apply_target_angle(angle, "preset");
}

static void one_key_test_cb(lv_event_t *e)
{
    (void)e;
#if ENABLE_MOTOR_CONTROL
    motor_command_ok(motor_control_run_test(&motor_state), "one-key test");
#endif
}

static void demo_forward_cb(lv_event_t *e)
{
    (void)e;
#if ENABLE_MOTOR_CONTROL
    motor_command_ok(motor_control_start_demo(true, &motor_state), "forward demo");
#endif
}

static void demo_reverse_cb(lv_event_t *e)
{
    (void)e;
#if ENABLE_MOTOR_CONTROL
    motor_command_ok(motor_control_start_demo(false, &motor_state), "reverse demo");
#endif
}

static void stop_motion_cb(lv_event_t *e)
{
    (void)e;
#if ENABLE_MOTOR_CONTROL
    motor_command_ok(motor_control_stop_motion(&motor_state), "stop motion");
#endif
}

static void axis_cb(lv_event_t *e)
{
    struct axis_ui *axis = lv_event_get_user_data(e);

    //printf("%s %d\n", __func__, __LINE__);
    if (page_overview)
        lv_obj_add_flag(page_overview, LV_OBJ_FLAG_HIDDEN);
    if (!position_cont)
        return;

    if (lv_obj_has_flag(position_cont, LV_OBJ_FLAG_HIDDEN))
        lv_obj_clear_flag(position_cont, LV_OBJ_FLAG_HIDDEN);
    else if (position_ctrl_target == axis)
        lv_obj_add_flag(position_cont, LV_OBJ_FLAG_HIDDEN);
    position_ctrl_target = axis;
    ml_label_set_text(label_pos_ctrl_target, axis->txt_idx);
}

static void axis_ui_init(lv_obj_t *parent,
                         struct axis_ui *axis)
{
    lv_obj_t *subcont;

    axis->cont = lv_obj_create(parent);
    lv_obj_set_flex_flow(axis->cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(axis->cont, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(axis->cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(axis->cont, SCALER_RESIZE(720), SCALER_RESIZE(280));
    lv_obj_set_style_bg_color(axis->cont, lv_color_hex(0x2566a8),
                              LV_PART_MAIN);
    lv_obj_set_style_pad_gap(axis->cont, 5, LV_PART_MAIN);
    lv_obj_add_event_cb(axis->cont, axis_cb, LV_EVENT_CLICKED, axis);

    subcont = lv_obj_create(axis->cont);
    lv_obj_remove_style_all(subcont);
    lv_obj_add_flag(subcont, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_size(subcont, lv_pct(100), SCALER_RESIZE(50));

    axis->light = lv_led_create(subcont);
    lv_obj_set_size(axis->light, SCALER_RESIZE(30), SCALER_RESIZE(30));
    lv_obj_align(axis->light, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_set_style_bg_color(axis->light, lv_color_hex(0x242424),
                              LV_PART_MAIN);
    lv_led_set_color(axis->light, lv_color_hex(0x24d957));
    lv_led_off(axis->light);
    lv_obj_add_flag(axis->light, LV_OBJ_FLAG_EVENT_BUBBLE);

    axis->label = ml_label_create(subcont, axis->txt_idx);
    lv_obj_set_style_text_color(axis->label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(axis->label, safe_font(&ttf_main_s), LV_PART_MAIN);
    lv_obj_align_to(axis->label, axis->light, LV_ALIGN_OUT_RIGHT_MID,
                    SCALER_RESIZE(20), 0);
    lv_obj_add_flag(axis->label, LV_OBJ_FLAG_EVENT_BUBBLE);

    subcont = lv_obj_create(axis->cont);
    lv_obj_remove_style_all(subcont);
    lv_obj_add_flag(subcont, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_size(subcont, lv_pct(100), SCALER_RESIZE(85));

    axis->pos = ml_label_create(subcont, TXT_CUR_POS);
    lv_obj_set_size(axis->pos, lv_pct(60), LV_SIZE_CONTENT);
    lv_obj_set_style_text_color(axis->pos, lv_color_white(),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(axis->pos, safe_font(&ttf_main_s), LV_PART_MAIN);
    lv_obj_align(axis->pos, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_flag(axis->pos, LV_OBJ_FLAG_EVENT_BUBBLE);

    axis->pos_val = lv_label_create(subcont);
    lv_label_set_text(axis->pos_val, "000.00");
    lv_obj_set_size(axis->pos_val, lv_pct(40), LV_SIZE_CONTENT);
    lv_obj_set_style_radius(axis->pos_val, 5, LV_PART_MAIN);
    lv_obj_set_style_text_color(axis->pos_val, lv_color_black(),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(axis->pos_val, safe_font(&ttf_main_s), LV_PART_MAIN);
    lv_obj_set_style_text_align(axis->pos_val, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_bg_color(axis->pos_val, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(axis->pos_val, LV_OPA_100, LV_PART_MAIN);
    lv_obj_align(axis->pos_val, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_flag(axis->pos_val, LV_OBJ_FLAG_EVENT_BUBBLE);

    subcont = lv_obj_create(axis->cont);
    lv_obj_remove_style_all(subcont);
    lv_obj_set_size(subcont, lv_pct(100), SCALER_RESIZE(85));
    lv_obj_add_flag(subcont, LV_OBJ_FLAG_EVENT_BUBBLE);

    axis->target = ml_label_create(subcont, TXT_TAR_POS);
    lv_obj_set_size(axis->target, lv_pct(60), LV_SIZE_CONTENT);
    lv_obj_set_style_text_color(axis->target, lv_color_white(),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(axis->target, safe_font(&ttf_main_s), LV_PART_MAIN);
    lv_obj_align(axis->target, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_flag(axis->target, LV_OBJ_FLAG_EVENT_BUBBLE);

    axis->target_val = lv_label_create(subcont);
    lv_label_set_text(axis->target_val, "  0.00");
    lv_obj_set_size(axis->target_val, lv_pct(40), LV_SIZE_CONTENT);
    lv_obj_set_style_radius(axis->target_val, 5, LV_PART_MAIN);
    lv_obj_set_style_text_color(axis->target_val, lv_color_black(),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(axis->target_val, safe_font(&ttf_main_s), LV_PART_MAIN);
    lv_obj_set_style_text_align(axis->target_val, LV_TEXT_ALIGN_CENTER,
                                LV_PART_MAIN);
    lv_obj_set_style_bg_color(axis->target_val, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(axis->target_val, LV_OPA_100, LV_PART_MAIN);
    lv_obj_align(axis->target_val, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_flag(axis->target_val, LV_OBJ_FLAG_EVENT_BUBBLE);
}

static void switch_language(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);

    if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
        ml_label_set_language(1);
    else
        ml_label_set_language(0);
}

void update_tick_points(lv_point_t *p, int angle)
{
    float degree = angle * 2 * PI / 360.0;
    p[0].x = SCALER_RESIZE(240) * cos(degree) + SCALER_RESIZE(290);
    p[0].y = SCALER_RESIZE(240) * sin(degree) + SCALER_RESIZE(290);
    p[1].x = SCALER_RESIZE(230) * cos(degree) + SCALER_RESIZE(290);
    p[1].y = SCALER_RESIZE(230) * sin(degree) + SCALER_RESIZE(290);
    p[2].x = SCALER_RESIZE(200) * cos(degree) + SCALER_RESIZE(290);
    p[2].y = SCALER_RESIZE(200) * sin(degree) + SCALER_RESIZE(290);
}

void update_needle_points(lv_point_t *p, int angle)
{
    float degree = angle * 2 * PI / 360.0;
    p[0].x = SCALER_RESIZE(290) * cos(degree) + SCALER_RESIZE(290);
    p[0].y = SCALER_RESIZE(290) * sin(degree) + SCALER_RESIZE(290);
    p[1].x = SCALER_RESIZE(280) * cos(degree) + SCALER_RESIZE(290);
    p[1].y = SCALER_RESIZE(280) * sin(degree) + SCALER_RESIZE(290);
}

static void needle_cb(lv_event_t *e)
{
    struct meter *meter;
    lv_indev_t *indev;
    lv_area_t a;
    lv_point_t p;
    lv_point_t c;
    lv_coord_t w, h;
    lv_coord_t dx, dy;
    lv_coord_t angle;
    lv_coord_t final_angle;

    meter = lv_event_get_user_data(e);
    if ((e->code == LV_EVENT_PRESSING) ||
            (e->code == LV_EVENT_RELEASED))
    {
        indev = lv_indev_get_act();
        lv_indev_get_point(indev, &p);
        lv_obj_get_coords(lv_event_get_target(e), &a);
        w = a.x2 - a.x1;
        h = a.y2 - a.y1;
        c.x = a.x1 + w / 2;
        c.y = a.y1 + h / 2;
        dx = p.x - c.x;
        dy = p.y - c.y;
        angle = lv_atan2(dy, dx);
        final_angle = angle - START_ANGLE;
        if (final_angle < 0)
            final_angle += 360;
        lv_label_set_text_fmt(meter->label_val,
                              "%d\xC2\xB0", final_angle);
        meter_set_needle_angle(meter, angle * 10);
        if (e->code == LV_EVENT_RELEASED)
            apply_target_angle(final_angle, "meter");
    }
}

static struct meter *meter_create(lv_obj_t *parent)
{
    struct meter *meter;
    int max_ticks;

    meter = malloc(sizeof(struct meter));
    if (!meter)
    {
        printf("motor_demo: meter allocation failed\n");
        return NULL;
    }
    memset(meter, 0, sizeof(*meter));

    meter->cont = lv_obj_create(parent);
    lv_obj_remove_style_all(meter->cont);
    lv_obj_clear_flag(meter->cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(meter->cont, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(meter->cont, SCALER_RESIZE(600), SCALER_RESIZE(600));
    lv_obj_add_event_cb(meter->cont, needle_cb, LV_EVENT_ALL, meter);
    lv_obj_center(meter->cont);

    meter->main = lv_arc_create(meter->cont);
    lv_obj_clear_flag(meter->main, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(meter->main, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(meter->main, lv_pct(100), lv_pct(100));
    lv_arc_set_rotation(meter->main, START_ANGLE);
    lv_arc_set_bg_angles(meter->main, 0, max_angle);
    lv_obj_set_style_arc_width(meter->main, 20,
                               LV_PART_MAIN);
    lv_obj_set_style_arc_width(meter->main, 20,
                               LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(meter->main, LV_OPA_0,
                            LV_PART_MAIN);
    lv_obj_set_style_bg_opa(meter->main, LV_OPA_0,
                            LV_PART_KNOB);
    lv_arc_set_value(meter->main, 0);
    lv_arc_set_range(meter->main, 0, MAX_SPEED);
    lv_obj_center(meter->main);

    meter->adorn = lv_arc_create(meter->cont);
    lv_obj_clear_flag(meter->adorn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(meter->adorn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(meter->adorn, SCALER_RESIZE(520), SCALER_RESIZE(520));
    lv_arc_set_rotation(meter->adorn, START_ANGLE);
    lv_arc_set_bg_angles(meter->adorn, 0, max_angle);
    lv_obj_set_style_arc_width(meter->adorn, 2,
                               LV_PART_MAIN);
    lv_obj_set_style_arc_width(meter->adorn, 2,
                               LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(meter->adorn, LV_OPA_0,
                            LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(meter->adorn, LV_OPA_0,
                            LV_PART_KNOB);
    lv_arc_set_value(meter->adorn, 0);
    lv_arc_set_range(meter->adorn, 0, MAX_SPEED);
    lv_obj_center(meter->adorn);

    meter->needle = lv_img_create(meter->cont);
    meter->needle_zoom = SCALER_RESIZE(256);
    if (meter->needle_zoom == 0)
        meter->needle_zoom = LV_IMG_ZOOM_NONE;
    if (ENABLE_NEEDLE_RGA_PRE_SCALE &&
        meter_prepare_needle_image(meter, SRC_PNG(needle), meter->needle_zoom))
        lv_img_set_src(meter->needle, &meter->needle_dsc);
    else
    {
        lv_img_set_src(meter->needle, SRC_PNG(needle));
        lv_img_set_zoom(meter->needle, meter->needle_zoom);
    }
    meter_set_needle_angle(meter, START_ANGLE * 10);
    lv_obj_center(meter->needle);
    lv_obj_add_flag(meter->needle, LV_OBJ_FLAG_EVENT_BUBBLE);

    max_ticks = METER_TICKS;
    if (max_angle >= 360.0)
        max_ticks -= 1;
    for (int i = 0; i < max_ticks; i++)
    {
        int w, h;
        meter->ticks[i].obj = lv_line_create(meter->cont);
        update_tick_points(meter->ticks[i].p,
                           START_ANGLE + SPEED_TO_ANGLE(i * 1000));
        lv_obj_set_style_line_color(meter->ticks[i].obj,
                                    lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_line_width(meter->ticks[i].obj,
                                    4, LV_PART_MAIN);
        lv_line_set_points(meter->ticks[i].obj,
                           meter->ticks[i].p, 2);

        meter->ticks[i].label = lv_label_create(meter->cont);
        lv_label_set_text_fmt(meter->ticks[i].label,
                              "%d\xC2\xB0", i * 30);
        lv_obj_refr_size(meter->ticks[i].label);
        w = lv_obj_get_width(meter->ticks[i].label);
        h = lv_obj_get_height(meter->ticks[i].label);
        lv_obj_set_pos(meter->ticks[i].label,
                       meter->ticks[i].p[2].x - w / 2,
                       meter->ticks[i].p[2].y - h / 2);
        lv_obj_set_style_text_color(meter->ticks[i].label,
                                    lv_color_black(), LV_PART_MAIN);
    }

    meter->label_val = lv_label_create(meter->cont);
    lv_label_set_text(meter->label_val, "0\xC2\xB0");
    lv_obj_center(meter->label_val);
    lv_obj_set_style_text_font(meter->label_val,
                               safe_font(&ttf_main_l), LV_PART_MAIN);
    lv_obj_set_style_text_color(meter->label_val,
                                lv_color_black(), LV_PART_MAIN);

    return meter;
}

static void meter_set_value(struct meter *meter, int current_angle,
                            int target_angle, lv_color_t accent)
{
    if (!meter)
        return;

    current_angle = clamp_target_angle(current_angle);
    target_angle = clamp_target_angle(target_angle);

    lv_arc_set_value(meter->main, ANGLE_TO_SPEED(current_angle));
    meter_set_needle_angle(meter, (START_ANGLE + current_angle) * 10);
    lv_label_set_text_fmt(meter->label_val, "%d\xC2\xB0", current_angle);
    lv_obj_set_style_arc_color(meter->main, accent, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(meter->adorn, accent, LV_PART_MAIN);
    set_target_widgets(target_angle);
}

static lv_obj_t *panel_create(lv_obj_t *parent, lv_coord_t w, lv_coord_t h)
{
    lv_obj_t *obj = lv_obj_create(parent);

    lv_obj_set_size(obj, w, h);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(obj, SCALER_RESIZE(28), LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0xf5f1e8), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(obj, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_pad_all(obj, SCALER_RESIZE(18), LV_PART_MAIN);
    return obj;
}

static lv_obj_t *button_area_create(lv_obj_t *parent, lv_coord_t w,
                                    lv_coord_t h)
{
    lv_obj_t *obj = lv_obj_create(parent);

    lv_obj_remove_style_all(obj);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_layout(obj, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(obj, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(obj, SCALER_RESIZE(12), LV_PART_MAIN);
    return obj;
}

static lv_obj_t *status_row_create(lv_obj_t *parent)
{
    lv_obj_t *row = lv_obj_create(parent);

    lv_obj_remove_style_all(row);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, 0);
    lv_obj_set_flex_grow(row, 1);
    lv_obj_set_layout(row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_gap(row, 0, LV_PART_MAIN);
    return row;
}

static lv_obj_t *status_card_create(lv_obj_t *parent, int title_idx, lv_obj_t **value_label)
{
    lv_obj_t *card;
    lv_obj_t *title;
    lv_obj_t *value;

    card = lv_obj_create(parent);
    lv_obj_remove_style_all(card);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(card, lv_pct(50), lv_pct(100));
    lv_obj_set_layout(card, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_ver(card, SCALER_RESIZE(8), LV_PART_MAIN);
    lv_obj_set_style_pad_hor(card, SCALER_RESIZE(12), LV_PART_MAIN);
    lv_obj_set_style_pad_gap(card, SCALER_RESIZE(2), LV_PART_MAIN);

    title = ml_label_create(card, title_idx);
    lv_obj_set_width(title, lv_pct(100));
    lv_label_set_long_mode(title, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_style_text_color(title, lv_color_hex(0x7c6d5b), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, safe_font(&ttf_main_m), LV_PART_MAIN);

    value = lv_label_create(card);
    lv_obj_set_width(value, lv_pct(100));
    lv_label_set_long_mode(value, LV_LABEL_LONG_CLIP);
    lv_label_set_text(value, "--");
    lv_obj_set_style_text_align(value, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_style_text_color(value, lv_color_hex(0x1d2733), LV_PART_MAIN);
    lv_obj_set_style_text_font(value, safe_font(&ttf_main_s), LV_PART_MAIN);
    lv_obj_set_style_pad_top(value, SCALER_RESIZE(1), LV_PART_MAIN);

    if (value_label)
        *value_label = value;

    return card;
}

static lv_obj_t *action_button_create(lv_obj_t *parent, const char *text,
                                      lv_event_cb_t cb, void *user_data,
                                      lv_color_t color)
{
    lv_obj_t *btn;
    lv_obj_t *label;

    btn = lv_btn_create(parent);
    lv_obj_set_height(btn, lv_pct(100));
    lv_obj_set_style_radius(btn, SCALER_RESIZE(24), LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, color, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);

    label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, safe_font(&ttf_main_s), LV_PART_MAIN);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_width(label, lv_pct(100));
    lv_obj_center(label);

    return btn;
}

static const char *al_state_to_string(int al_state)
{
    switch (al_state & 0x0f)
    {
    case 0x01:
        return "INIT";
    case 0x02:
        return "PREOP";
    case 0x04:
        return "SAFEOP";
    case 0x08:
        return "OP";
    default:
        return "--";
    }
}

static const char *opmode_to_string(int opmode)
{
    switch (opmode)
    {
    case 8:
        return "CSP";
    case 0:
        return "Init";
    default:
        return "Other";
    }
}

static const char *run_state_to_string(void)
{
    if (!motor_state.ready)
        return "NOT READY";
    if (motor_state.axis_fault[0])
        return "FAULT";
    if (!motor_state.axis_online[0])
        return "OFFLINE";
    switch (motor_state.motion_mode)
    {
    case MOTOR_CONTROL_MOTION_TEST_TO_ZERO:
        return "TEST: ZERO";
    case MOTOR_CONTROL_MOTION_TEST_TO_TEN:
        return "TEST: +10 DEG";
    case MOTOR_CONTROL_MOTION_TEST_RETURN_ZERO:
        return "TEST: RETURN";
    case MOTOR_CONTROL_MOTION_DEMO_FORWARD:
        return "DEMO: FORWARD";
    case MOTOR_CONTROL_MOTION_DEMO_REVERSE:
        return "DEMO: REVERSE";
    case MOTOR_CONTROL_MOTION_IDLE:
    default:
        break;
    }
    if (motor_state.motion_result < 0)
        return "TEST FAILED";
    if (motor_state.motion_result > 0)
        return "TEST PASSED";
    if (motor_state.axis_servo_on[0])
        return "RUNNING";
    return "PREPARING";
}

static lv_color_t run_state_color(void)
{
    if (!motor_state.ready)
        return lv_color_hex(0x8c8c8c);
    if (motor_state.axis_fault[0])
        return lv_color_hex(0xc44536);
    if (motor_state.motion_result < 0)
        return lv_color_hex(0xc44536);
    if (motor_state.motion_mode != MOTOR_CONTROL_MOTION_IDLE)
        return lv_color_hex(0x2e9d5b);
    if (motor_state.axis_servo_on[0])
        return lv_color_hex(0x2e9d5b);
    return lv_color_hex(0xd88c2d);
}

static const char *comm_state_to_string(void)
{
    switch (motor_state.axis_wc_state[0])
    {
    case 2:
        return "OK";
    case 1:
        return "PART";
    case 0:
    default:
        return "LOST";
    }
}

static void meter_cb(lv_event_t *e)
{
    lv_coord_t angle;
    lv_coord_t final_angle;

    if (!pos_meter || !position_ctrl_target || !position_ctrl_target->target_val)
        return;

    angle = pos_meter->needle_angle;
    final_angle = angle / 10 - START_ANGLE;
    if (final_angle < 0)
        final_angle += 360;
    lv_label_set_text_fmt(position_ctrl_target->target_val, "%3.2f",
                          final_angle / 1.0);
#if ENABLE_MOTOR_CONTROL
    if (!motor_command_ok(
            motor_control_set_position(position_ctrl_target->axis_id,
                                       final_angle, &motor_state),
            "set position"))
        return;
#endif
}

static void position_ctrl_create(void)
{
    lv_obj_t *btn_cont;
    lv_obj_t *btn;
    lv_obj_t *label;

    position_cont = lv_obj_create(scr);
    lv_obj_set_size(position_cont, lv_pct(40), lv_pct(100));
    lv_obj_align(position_cont, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_flag(position_cont, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(position_cont, LV_OBJ_FLAG_SCROLLABLE);

    label_pos_ctrl_target = ml_label_create(position_cont, TXT_AXIS_X);
    if (!label_pos_ctrl_target)
    {
        printf("motor_demo: failed to create position control target label\n");
        return;
    }
    lv_obj_align(label_pos_ctrl_target, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_color(label_pos_ctrl_target, lv_color_black(),
                                LV_PART_MAIN);
    lv_obj_set_style_text_font(label_pos_ctrl_target, safe_font(&ttf_main_s),
                               LV_PART_MAIN);

    pos_meter = meter_create(position_cont);
    if (!pos_meter)
        return;

    btn_cont = lv_obj_create(position_cont);
    lv_obj_remove_style_all(btn_cont);
    lv_obj_set_size(btn_cont, lv_pct(80), SCALER_RESIZE(80));
    lv_obj_align(btn_cont, LV_ALIGN_BOTTOM_MID, 0, 0);

    btn = lv_btn_create(btn_cont);
    lv_obj_set_size(btn, lv_pct(45), lv_pct(100));
    lv_obj_align(btn, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_event_cb(btn, meter_cb, LV_EVENT_CLICKED, NULL);

    label = ml_label_create(btn, TXT_CONFIRM);
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, safe_font(&ttf_main_s),
                               LV_PART_MAIN);
    lv_obj_center(label);

    btn = lv_btn_create(btn_cont);
    lv_obj_set_size(btn, lv_pct(45), lv_pct(100));
    lv_obj_align(btn, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(btn, close_cb, LV_EVENT_CLICKED, NULL);

    label = ml_label_create(btn, TXT_CLOSE);
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, safe_font(&ttf_main_s),
                               LV_PART_MAIN);
    lv_obj_center(label);
}

static void overview_page_create(void)
{
    lv_obj_t *cont;
    lv_obj_t *subcont;
    lv_obj_t *obj;
    lv_obj_t *label;

    cont = page_overview = lv_obj_create(scr);
    lv_obj_set_size(cont, lv_pct(40), lv_pct(100));
    lv_obj_align(cont, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);

    subcont = lv_obj_create(cont);
    lv_obj_set_size(subcont, lv_pct(100), 200);

    label = ml_label_create(subcont, TXT_LANGUAGE);
    if (!label)
        return;
    lv_obj_set_style_text_color(label, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, safe_font(&ttf_main_s),
                               LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);

    label = lv_label_create(subcont);
    lv_label_set_text(label, "English");
    lv_obj_set_style_text_color(label, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, safe_font(&ttf_main_s),
                               LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_RIGHT_MID, 0, 0);

    obj = lv_switch_create(subcont);
    lv_obj_set_size(obj, 80, 48);
    lv_obj_add_event_cb(obj, switch_language, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_align_to(obj, label, LV_ALIGN_OUT_LEFT_MID, 0, 0);

    label = lv_label_create(subcont);
    lv_label_set_text(label, "中文");
    lv_obj_set_style_text_color(label, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, safe_font(&ttf_main_s),
                               LV_PART_MAIN);
    lv_obj_align_to(label, obj, LV_ALIGN_OUT_LEFT_MID, 0, 0);

    subcont = lv_obj_create(cont);
    lv_obj_set_size(subcont, lv_pct(100), LV_SIZE_CONTENT);

    label = ml_label_create(subcont, TXT_HW_VER);
    if (!label)
        return;
    lv_obj_set_size(label, lv_pct(33), LV_SIZE_CONTENT);
    lv_obj_set_style_text_color(label, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, safe_font(&ttf_main_s),
                               LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);

    label = lv_label_create(subcont);
    lv_obj_set_size(label, lv_pct(66), LV_SIZE_CONTENT);
    lv_label_set_text(label, safe_string(compatible_name, "unknown"));
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(label, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, safe_font(&ttf_main_s),
                               LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_RIGHT_MID, 0, 0);

    subcont = lv_obj_create(cont);
    lv_obj_set_size(subcont, lv_pct(100), LV_SIZE_CONTENT);

    label = ml_label_create(subcont, TXT_SYS_VER);
    if (!label)
        return;
    lv_obj_set_size(label, lv_pct(33), LV_SIZE_CONTENT);
    lv_obj_set_style_text_color(label, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, safe_font(&ttf_main_s),
                               LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);

    label = lv_label_create(subcont);
    lv_obj_set_size(label, lv_pct(66), LV_SIZE_CONTENT);
    lv_label_set_text(label, safe_string(sys_version, "unknown"));
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(label, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, safe_font(&ttf_main_s),
                               LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_add_flag(page_overview, LV_OBJ_FLAG_HIDDEN);
}

static void ui_build_input_timer_cb(lv_timer_t *timer)
{
    if (lvgl_init_input() != 0)
    {
        if (timer)
            lv_timer_set_period(timer, UI_INPUT_RETRY_DELAY_MS);
        return;
    }

    if (timer)
        lv_timer_del(timer);
}

static void display_motor_information(void)
{
    lv_color_t accent = lv_color_hex(0x8c8c8c);
    const char *state_text = "NOT READY";
    const char *bus_state = "--";
    const char *ctrl_mode = "--";
    const char *servo_state = "OFF";
    const char *inpos_state = "NO";
    const char *fault_state = "NO";
    const char *comm_state = "LOST";
    const char *hint_text = "Preparing EtherCAT, daemon and gauge...";
    int current_angle = 0;
    int target_angle = 0;
    unsigned int jitter_us = 0;
    bool available = false;

    if (!ui_ready || !label_state_pill || !label_state_pill_value)
        return;

#if ENABLE_MOTOR_CONTROL
    if ((motor_get_state() == MOTOR_CONTROL_STATUS_OK) && motor_available())
    {
        available = true;
        accent = run_state_color();
        state_text = run_state_to_string();
        bus_state = al_state_to_string(motor_state.axis_al_state[0]);
        ctrl_mode = opmode_to_string(motor_state.axis_opmode[0]);
        servo_state = motor_state.axis_servo_on[0] ? "ON" : "OFF";
        inpos_state = motor_state.axis_in_position[0] ? "YES" : "NO";
        fault_state = motor_state.axis_fault[0] ? "FAULT" : "OK";
        comm_state = comm_state_to_string();
        current_angle = motor_state.axis_position[0];
        target_angle = motor_state.axis_target[0];
        jitter_us = motor_state.jitter_us;
        hint_text = "Motor stack ready";
    }
#endif

    lv_label_set_text(label_state_pill_value, state_text);
    lv_obj_set_style_bg_color(label_state_pill, accent, LV_PART_MAIN);

    if (label_target_hint)
    {
        if (available)
        {
            lv_label_set_text_fmt(label_target_hint,
                                  "Axis 0 %d deg / Target %d deg / Jitter %u us",
                                  current_angle, target_angle, jitter_us);
        }
        else
        {
            lv_label_set_text(label_target_hint, hint_text);
        }
    }

    if (label_status_cur_pos)
        lv_label_set_text_fmt(label_status_cur_pos, "%d deg", current_angle);
    if (label_status_target_pos)
        lv_label_set_text_fmt(label_status_target_pos, "%d deg", target_angle);
    if (label_status_run_state)
        lv_label_set_text(label_status_run_state, state_text);
    if (label_status_bus_state)
        lv_label_set_text(label_status_bus_state, bus_state);
    if (label_status_ctrl_mode)
        lv_label_set_text(label_status_ctrl_mode, ctrl_mode);
    if (label_status_status_word)
        lv_label_set_text_fmt(label_status_status_word, "0x%04x",
                              available ? motor_state.axis_status_word[0] : 0);
    if (label_status_servo)
        lv_label_set_text(label_status_servo, servo_state);
    if (label_status_inpos)
        lv_label_set_text(label_status_inpos, inpos_state);
    if (label_status_fault)
        lv_label_set_text(label_status_fault, fault_state);
    if (label_status_comm)
    {
        if (available)
        {
            lv_label_set_text_fmt(label_status_comm, "%s / %u",
                                  comm_state,
                                  motor_state.axis_working_counter[0]);
        }
        else
        {
            lv_label_set_text(label_status_comm, comm_state);
        }
    }
    if (label_status_jitter)
        lv_label_set_text_fmt(label_status_jitter, "%u us", jitter_us);
    if (label_status_cyclic_frequency)
        lv_label_set_text_fmt(label_status_cyclic_frequency, "%u Hz",
                              MOTOR_CONTROL_CYCLIC_FREQUENCY_HZ);

    if (pos_meter)
        meter_set_value(pos_meter, current_angle, target_angle, accent);
}

int main(int argc, char **argv)
{
    int fre = 500;
    uint64_t start_ms = monotonic_ms();

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    printf("motor_demo: starting\n");
    configure_process_runtime();

    signal(SIGINT, sigterm_handler);
    lvgl_init_display();
    printf("motor_demo: lvgl display initialized\n");
    /* Do not present the temporary boot card before the full UI is ready. */
    ui_init_light();
    ui_init_detail();
    ui_init_visuals();
    display_motor_information();
    ui_force_present();
    printf("motor_demo: first frame presented in %" PRIu64 " ms\n",
           monotonic_ms() - start_ms);
    printf("motor_demo: full ui initialized\n");
    lv_task_handler();
    lv_timer_create(ui_build_input_timer_cb, UI_INPUT_BUILD_DELAY_MS, NULL);
    motor_state_reset();

    while (!quit)
    {
        if (fre == 0)
        {
            display_motor_information();
            fre = 500;
        }
        fre--;
        lv_task_handler();
        usleep(1000);
    }

    return 0;
}
