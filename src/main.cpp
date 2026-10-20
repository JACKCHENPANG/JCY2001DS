#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <TAMC_GT911.h>
#include <esp_heap_caps.h>
#include <freertos/semphr.h>
#include <lvgl.h>

#include "battery_soc.h"
#include "jcy_audio.h"
#include "jcy_rgb_bus.h"
#include "jcy_rgb_display.h"
#include "jcy_scpi_link.h"
#include "jcy_wireless.h"
#include "jcy_ota.h"
#include "jcy_panel_profile.h"
#include "jcy_secondary_uart.h"
#include "jcy_cloud_ota.h"
#include "jcy_lower_mcu_ota.h"
#if defined(JCY_HY200C_UI) && JCY_HY200C_UI
#include "hy200_protocol.h"
#include "hy200_ui.h"
#endif
#include <WiFi.h>

// TLS handshake needs multiple large, concurrently-live record buffers.  The
// ESP-IDF default mbedTLS allocator only requests internal DRAM, which can
// fail after RGB/LVGL buffers are initialized even when PSRAM has ample free
// space.  Route sufficiently large TLS allocations to PSRAM first, then fall
// back to internal DRAM.  This must run before any HTTPS request.
extern "C" int mbedtls_platform_set_calloc_free(void *(*calloc_func)(size_t, size_t),
                                                void (*free_func)(void *));

static volatile uint32_t jcy_mbed_psram_hits = 0;
static volatile uint32_t jcy_mbed_intram_hits = 0;
static volatile uint32_t jcy_mbed_fail_hits = 0;

constexpr size_t JCY_MBED_PSRAM_THRESHOLD = 512;

extern "C" void *jcy_mbedtls_calloc(size_t n, size_t size) {
    void *p = nullptr;
    const size_t total = n * size;
    if (total >= JCY_MBED_PSRAM_THRESHOLD) {
        p = heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (p != nullptr) {
            ++jcy_mbed_psram_hits;
            return p;
        }
    }
    p = heap_caps_calloc(n, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p != nullptr) {
        ++jcy_mbed_intram_hits;
        return p;
    }
    p = heap_caps_calloc(n, size, MALLOC_CAP_8BIT);
    if (p == nullptr) ++jcy_mbed_fail_hits;
    return p;
}

extern "C" void jcy_mbedtls_free(void *ptr) {
    if (ptr != nullptr) heap_caps_free(ptr);
}

static void jcy_install_mbedtls_psram_allocator() {
    const int rc =
        mbedtls_platform_set_calloc_free(jcy_mbedtls_calloc, jcy_mbedtls_free);
    Serial.printf("[MBEDTLS-FIX] set_calloc_free rc=%d psram_free=%u internal_max_blk=%u\\n",
                  rc,
                  static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
                  static_cast<unsigned>(
                      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)));
}

#define JCY1000_FIRMWARE_VERSION "V0.8.8-A95"
#define JCY1000_BUILD_DATE "2026-08-24"
#define JCY1000_BUILD_TAG \
    "2026-08-24 V0.8.8-A95 sh051-native-scale-fix"
#define JCY_PRODUCT_NAME "JCY2001D"

LV_FONT_DECLARE(lv_font_jcy16);
LV_FONT_DECLARE(lv_font_jcy18);
LV_FONT_DECLARE(lv_font_jcy20);
LV_IMG_DECLARE(lv_img_jcy_logo);
LV_IMG_DECLARE(lv_img_jcy_splash);
// 启动鲸鱼动画 30 帧、480x288 SJPG 数组，从完整 10 秒源视频均匀抽帧。
extern const lv_img_dsc_t * const jcy_boot_animation_frames[];
extern const uint32_t jcy_boot_animation_frame_count;
LV_IMG_DECLARE(lv_img_jcy_main_skin);
LV_IMG_DECLARE(lv_img_jcy_power_tab_source);
LV_IMG_DECLARE(lv_img_jcy_power_tab_load);
LV_IMG_DECLARE(lv_img_jcy_power_tab_cycle);
LV_IMG_DECLARE(lv_img_jcy_power_dual_power);
LV_IMG_DECLARE(lv_img_jcy_power_power_output);
LV_IMG_DECLARE(lv_img_jcy_power_power_switch);
LV_IMG_DECLARE(lv_img_jcy_power_metric_v);
LV_IMG_DECLARE(lv_img_jcy_power_metric_a);
LV_IMG_DECLARE(lv_img_jcy_power_metric_w);
LV_IMG_DECLARE(lv_img_jcy_power_metric_t);
LV_IMG_DECLARE(lv_img_jcy_power_info);
LV_IMG_DECLARE(lv_img_jcy_power_fine_tune);

namespace {
constexpr int SCREEN_WIDTH = jcy_panel::kUiWidth;
constexpr int SCREEN_HEIGHT = jcy_panel::kUiHeight;
constexpr int PANEL_WIDTH = jcy_panel::kPanelWidth;
constexpr int PANEL_HEIGHT = jcy_panel::kPanelHeight;
constexpr int32_t RGB_PCLK_HZ = jcy_panel::kPclkHz;
constexpr int SPLASH_ANIM_WIDTH = 480;
constexpr int SPLASH_ANIM_HEIGHT = 288;
constexpr uint32_t SPLASH_ANIM_FRAME_MS = 100;
constexpr uint32_t SPLASH_FINAL_HOLD_MS = 1250;
constexpr int BACKLIGHT_PIN = 2;
// OTA 只使用静态深色提醒页，避免升级过程中回到主界面或触发大面积动画。
// 背光全程保持打开，使用户可明确看到下载、校验和自动重启的状态。
// EVT RGB40 routes native USB on GPIO19/20.  The separate CTP bus is on
// U0TXD/U0RXD (GPIO43/44); CTP reset is GPIO38 and INT is not connected.
#if JCY_PANEL_SH051_1024X600 || JCY_PANEL_B043_480X272
constexpr int TOUCH_SDA = 43;
constexpr int TOUCH_SCL = 44;
constexpr int TOUCH_INT = -1;
constexpr int TOUCH_RST = 38;
#else
constexpr int TOUCH_SDA = 19;
constexpr int TOUCH_SCL = 20;
constexpr int TOUCH_INT = -1;
constexpr int TOUCH_RST = 38;
#endif

// HY200-C TTL1 production wiring: TX=GPIO17, RX=GPIO18. This display-only
// product does not initialize the NS4168 I2S path that otherwise shares these
// pins. Its front-panel keys use GPIO11/12/13 independently.
constexpr bool TTL_ENABLED = true;
#if defined(JCY_HY200C_UI) && JCY_HY200C_UI
constexpr int TTL_TX_PIN = 17;
constexpr int TTL_RX_PIN = 18;
#else
constexpr int TTL_TX_PIN = 13;
constexpr int TTL_RX_PIN = 12;
#endif
constexpr uint32_t TTL_BAUD = 115200;
constexpr uint8_t TTL_ADDRESS = 1;

constexpr int HEADER_HEIGHT = 70;
constexpr int MODE_WIDTH = 100;
constexpr int METER_WIDTH = 210;
constexpr int SETTINGS_X = MODE_WIDTH + METER_WIDTH;
constexpr uint32_t UI_BG_TOP = 0x061A33;
constexpr uint32_t UI_BG_BOTTOM = 0x020A16;
constexpr uint32_t UI_PANEL_TOP = 0x092641;
constexpr uint32_t UI_PANEL_BOTTOM = 0x041224;
constexpr uint32_t UI_CARD_TOP = 0x0C3152;
constexpr uint32_t UI_CARD_BOTTOM = 0x06182D;
constexpr uint32_t UI_BORDER = 0x0C5577;
constexpr uint32_t UI_BORDER_DIM = 0x123D58;
constexpr uint32_t UI_CYAN = 0x00E9FF;
constexpr uint32_t UI_BLUE = 0x168DFF;
constexpr uint32_t UI_GREEN = 0x38D79A;
constexpr uint32_t UI_TEXT = 0xE6F1FF;
constexpr uint32_t UI_MUTED = 0x8AA8C6;
constexpr bool REFERENCE_SKIN_UI = true;
constexpr bool DISPLAY_DIAGNOSTICS = false;
constexpr bool DISPLAY_VSYNC_GUARD = true;
constexpr int SKIN_ORIGINAL_BODY_Y = 74;
constexpr int SKIN_COMPACT_HEADER_HEIGHT = 58;
constexpr int SKIN_ORIGINAL_BODY_HEIGHT = SCREEN_HEIGHT - SKIN_ORIGINAL_BODY_Y;
constexpr int SKIN_COMPACT_BODY_HEIGHT = SCREEN_HEIGHT - SKIN_COMPACT_HEADER_HEIGHT;
constexpr int SKIN_HEADER_CROP_TOP = 10;

constexpr int skin_body_y(int original_y) {
    return SKIN_COMPACT_HEADER_HEIGHT +
        ((original_y - SKIN_ORIGINAL_BODY_Y) * SKIN_COMPACT_BODY_HEIGHT +
         SKIN_ORIGINAL_BODY_HEIGHT / 2) /
            SKIN_ORIGINAL_BODY_HEIGHT;
}

constexpr int skin_body_height(int original_y, int original_height) {
    return skin_body_y(original_y + original_height) -
           skin_body_y(original_y);
}

constexpr int skin_header_y(int original_y) {
    return original_y - SKIN_HEADER_CROP_TOP;
}
constexpr uint32_t DISPLAY_VSYNC_TIMEOUT_MS = 40;
constexpr uint32_t LCD_REFRESH_HZ =
    static_cast<uint32_t>(RGB_PCLK_HZ) /
    ((PANEL_WIDTH + jcy_panel::kHsyncFrontPorch +
      jcy_panel::kHsyncPulseWidth + jcy_panel::kHsyncBackPorch) *
     (PANEL_HEIGHT + jcy_panel::kVsyncFrontPorch +
      jcy_panel::kVsyncPulseWidth + jcy_panel::kVsyncBackPorch));

JcyEsp32RgbPanel rgb_bus(
    GFX_NOT_DEFINED, GFX_NOT_DEFINED, GFX_NOT_DEFINED,
    40, 41, 39, 42,
    45, 48, 47, 21, 14,
    5, 6, 7, 15, 16, 4,
    8, 3, 46, 9, 1);

JcyRpiDpiRgbPanel gfx(
    &rgb_bus,
    PANEL_WIDTH, jcy_panel::kHsyncPolarity,
    jcy_panel::kHsyncFrontPorch, jcy_panel::kHsyncPulseWidth,
    jcy_panel::kHsyncBackPorch,
    PANEL_HEIGHT, jcy_panel::kVsyncPolarity,
    jcy_panel::kVsyncFrontPorch, jcy_panel::kVsyncPulseWidth,
    jcy_panel::kVsyncBackPorch,
    jcy_panel::kPclkActiveNeg, RGB_PCLK_HZ, true);

TAMC_GT911 touch(
    TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST,
    PANEL_WIDTH, PANEL_HEIGHT);

JcyScpiLink scpi_link(2);
JcySecondaryUart secondary_uart;
#if defined(JCY_HY200C_UI) && JCY_HY200C_UI
hy200::Protocol hy200_link(2);
#endif

lv_disp_draw_buf_t draw_buffer;
lv_color_t *draw_pixels = nullptr;
lv_disp_drv_t display_driver;
lv_indev_drv_t touch_driver;
lv_obj_t *debug_perf_panel = nullptr;
lv_obj_t *debug_perf_label = nullptr;
uint32_t debug_completed_frames = 0;
uint32_t debug_last_frame_counter = 0;
uint32_t debug_flush_calls = 0;
uint64_t debug_flush_total_us = 0;
uint32_t debug_flush_max_us = 0;
uint32_t debug_slow_flush_count = 0;
uint32_t debug_last_update_ms = 0;
uint32_t debug_vsync_timeout_count = 0;
SemaphoreHandle_t display_vsync_semaphore = nullptr;
bool display_frame_needs_vsync = true;

struct DisplayDirtyRect {
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
};

constexpr uint8_t DISPLAY_DIRTY_RECT_LIMIT = 16;
DisplayDirtyRect display_current_dirty[DISPLAY_DIRTY_RECT_LIMIT] = {};
DisplayDirtyRect display_pending_dirty[DISPLAY_DIRTY_RECT_LIMIT] = {};
uint8_t display_current_dirty_count = 0;
uint8_t display_pending_dirty_count = 0;

constexpr int scale_ceil(int value, int numerator, int denominator) {
    return (value * numerator + denominator - 1) / denominator;
}

DisplayDirtyRect panel_rect_for_ui_area(const lv_area_t *area) {
    const int ui_x1 = constrain(area->x1, 0, SCREEN_WIDTH - 1);
    const int ui_y1 = constrain(area->y1, 0, SCREEN_HEIGHT - 1);
    const int ui_x2 = constrain(area->x2, 0, SCREEN_WIDTH - 1);
    const int ui_y2 = constrain(area->y2, 0, SCREEN_HEIGHT - 1);
    return {
        static_cast<int16_t>(
            jcy_panel::kContentX +
            scale_ceil(ui_x1, jcy_panel::kContentWidth, SCREEN_WIDTH)),
        static_cast<int16_t>(
            jcy_panel::kContentY +
            scale_ceil(ui_y1, jcy_panel::kContentHeight, SCREEN_HEIGHT)),
        static_cast<int16_t>(
            jcy_panel::kContentX +
            scale_ceil(ui_x2 + 1, jcy_panel::kContentWidth,
                       SCREEN_WIDTH) - 1),
        static_cast<int16_t>(
            jcy_panel::kContentY +
            scale_ceil(ui_y2 + 1, jcy_panel::kContentHeight,
                       SCREEN_HEIGHT) - 1),
    };
}

int ui_x_for_panel_x(int panel_x) {
    const int content_x = constrain(
        panel_x - jcy_panel::kContentX, 0,
        jcy_panel::kContentWidth - 1);
    return constrain(
        content_x * SCREEN_WIDTH / jcy_panel::kContentWidth,
        0, SCREEN_WIDTH - 1);
}

int ui_y_for_panel_y(int panel_y) {
    const int content_y = constrain(
        panel_y - jcy_panel::kContentY, 0,
        jcy_panel::kContentHeight - 1);
    return constrain(
        content_y * SCREEN_HEIGHT / jcy_panel::kContentHeight,
        0, SCREEN_HEIGHT - 1);
}

bool IRAM_ATTR rgb_frame_trans_done_callback(
    esp_lcd_panel_handle_t,
    const esp_lcd_rgb_panel_event_data_t *,
    void *user_ctx) {
    SemaphoreHandle_t semaphore =
        static_cast<SemaphoreHandle_t>(user_ctx);
    if (semaphore == nullptr) {
        return false;
    }
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(semaphore, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

bool dirty_rects_touch(const DisplayDirtyRect &a,
                       const DisplayDirtyRect &b) {
    return a.x1 <= b.x2 + 1 && a.x2 + 1 >= b.x1 &&
           a.y1 <= b.y2 + 1 && a.y2 + 1 >= b.y1;
}

DisplayDirtyRect dirty_rect_union(const DisplayDirtyRect &a,
                                  const DisplayDirtyRect &b) {
    return {
        static_cast<int16_t>(min(a.x1, b.x1)),
        static_cast<int16_t>(min(a.y1, b.y1)),
        static_cast<int16_t>(max(a.x2, b.x2)),
        static_cast<int16_t>(max(a.y2, b.y2)),
    };
}

void add_display_dirty_rect(DisplayDirtyRect *rects, uint8_t &count,
                            const DisplayDirtyRect &dirty) {
    DisplayDirtyRect merged = dirty;
    for (uint8_t i = 0; i < count;) {
        if (!dirty_rects_touch(merged, rects[i])) {
            ++i;
            continue;
        }
        merged = dirty_rect_union(merged, rects[i]);
        rects[i] = rects[count - 1];
        --count;
        i = 0;
    }
    if (count < DISPLAY_DIRTY_RECT_LIMIT) {
        rects[count++] = merged;
        return;
    }
    for (uint8_t i = 0; i < count; ++i) {
        merged = dirty_rect_union(merged, rects[i]);
    }
    rects[0] = merged;
    count = 1;
}

void set_pending_full_frame() {
    display_pending_dirty[0] = {0, 0, PANEL_WIDTH - 1, PANEL_HEIGHT - 1};
    display_pending_dirty_count = 1;
}

void sync_pending_dirty_to_backbuffer() {
    if (display_pending_dirty_count == 0) {
        return;
    }
    const uint16_t *front = gfx.getFramebuffer();
    uint16_t *back = gfx.getBackFramebuffer();
    if (front == nullptr || back == nullptr || front == back) {
        display_pending_dirty_count = 0;
        return;
    }
    for (uint8_t i = 0; i < display_pending_dirty_count; ++i) {
        const DisplayDirtyRect &rect = display_pending_dirty[i];
        const uint32_t width = rect.x2 - rect.x1 + 1;
        const uint32_t height = rect.y2 - rect.y1 + 1;
        const uint16_t *source =
            front + rect.y1 * PANEL_WIDTH + rect.x1;
        uint16_t *destination =
            back + rect.y1 * PANEL_WIDTH + rect.x1;
        for (uint32_t row = 0; row < height; ++row) {
            memcpy(destination + row * PANEL_WIDTH,
                   source + row * PANEL_WIDTH,
                   width * sizeof(uint16_t));
        }
        Cache_WriteBack_Addr(
            reinterpret_cast<uint32_t>(back + rect.y1 * PANEL_WIDTH),
            PANEL_WIDTH * height * sizeof(uint16_t));
    }
    display_pending_dirty_count = 0;
}

bool virtual_touch_active = false;
bool virtual_touch_press_reported = false;
int virtual_touch_x = 0;
int virtual_touch_y = 0;
uint32_t virtual_touch_release_at = 0;
bool physical_touch_pressed = false;
uint8_t physical_touch_press_samples = 0;
uint8_t physical_touch_release_samples = 0;
int physical_touch_x = 0;
int physical_touch_y = 0;
// GT911 本身已完成触摸判定。保留释放确认以抑制偶发掉点，但首触必须
// 立即上报，且坐标不再做 1/2 均值滤波，避免按键和拖动出现可感知滞后。
constexpr uint8_t TOUCH_PRESS_CONFIRM_SAMPLES = 1;
constexpr uint8_t TOUCH_RELEASE_CONFIRM_SAMPLES = 2;
constexpr int TOUCH_COORD_DEADBAND = 2;
char serial_command[64] = {};
size_t serial_command_length = 0;
constexpr char PASSTHROUGH_ESCAPE_SEQUENCE[] = "+++JCYEXIT+++";
uint8_t passthrough_escape_buffer[
    sizeof(PASSTHROUGH_ESCAPE_SEQUENCE) - 1] = {};
size_t passthrough_escape_length = 0;
uint32_t passthrough_escape_started_ms = 0;

enum class JcyAlarmKind : uint8_t {
    None = 0,
    Overcurrent,
    ShortCircuit,
    ReversePolarity,
    OverTemperature,
    Generic,
};

bool contains_ignore_case(const char *text, const char *needle) {
    if (text == nullptr || needle == nullptr || *needle == '\0') {
        return false;
    }
    for (const char *start = text; *start != '\0'; ++start) {
        const char *a = start;
        const char *b = needle;
        while (*a != '\0' && *b != '\0' &&
               tolower(static_cast<unsigned char>(*a)) ==
                   tolower(static_cast<unsigned char>(*b))) {
            ++a;
            ++b;
        }
        if (*b == '\0') {
            return true;
        }
    }
    return false;
}

JcyAlarmKind classify_alarm(int code, const char *status) {
    if (code <= 0) {
        return JcyAlarmKind::None;
    }
    if (contains_ignore_case(status, "overcurrent") ||
        contains_ignore_case(status, "over current")) {
        return JcyAlarmKind::Overcurrent;
    }
    if (contains_ignore_case(status, "short")) {
        return JcyAlarmKind::ShortCircuit;
    }
    if (contains_ignore_case(status, "reverse") ||
        contains_ignore_case(status, "polarity")) {
        return JcyAlarmKind::ReversePolarity;
    }
    if (contains_ignore_case(status, "temperature") ||
        contains_ignore_case(status, "overheat")) {
        return JcyAlarmKind::OverTemperature;
    }
    return JcyAlarmKind::Generic;
}

const char *alarm_short_label(JcyAlarmKind kind) {
    switch (kind) {
        case JcyAlarmKind::Overcurrent:     return "过流保护";
        case JcyAlarmKind::ShortCircuit:    return "短路保护";
        case JcyAlarmKind::ReversePolarity: return "反接保护";
        case JcyAlarmKind::OverTemperature: return "过温保护";
        case JcyAlarmKind::Generic:         return "设备保护";
        default:                            return "设备正常";
    }
}

const char *alarm_long_text(JcyAlarmKind kind) {
    switch (kind) {
        case JcyAlarmKind::Overcurrent:
            return "输出电流超过过流阈值。\n请检查负载和过流保护参数。";
        case JcyAlarmKind::ShortCircuit:
            return "检测到输出短路。\n请断开负载并检查接线。";
        case JcyAlarmKind::ReversePolarity:
            return "检测到电池反接。\n请立即检查接线极性。";
        case JcyAlarmKind::OverTemperature:
            return "内部温度过高。\n请降低功率或改善散热。";
        case JcyAlarmKind::Generic:
            return "设备进入保护状态。\n具体类型需按底层协议错误码表补充。";
        default:
            return "当前未检测到设备保护报警。";
    }
}

enum class WorkMode : uint8_t {
    BatterySimulator,
    BidirectionalPower,
};

enum class CommunicationMode : uint8_t {
    Controller,
    Passthrough,
};

enum class PageKind : uint8_t {
    None,
    BatterySettings,
    CutoffTest,
    Help,
    SettingsHub,
    Power,
    Wifi,
    Bluetooth,
    Ota,
    Communication,
};

enum class CutoffRunPhase : uint8_t {
    Idle = 0,
    EnsureBatteryMode,
    ConfigureRamp,
    TriggerRamp,
    Running,
    Completed,
    Failed,
    Stopped,
};

enum class PowerTab : uint8_t {
    Source,
    Load,
    Cycle,
};

enum class NumericField : uint8_t {
    None = 0,
    SetVoltage,
    BatteryCells,
    CellMinVoltage,
    CellMaxVoltage,
    Overcurrent,
    ProtectionDelay,
    StaticThreshold,
    FilterDepth,
    BatteryResistance,
    ChargeStart,
    ChargeEnd,
    ChargeDuration,
    ChargeCutoffCurrent,
    DischargeStart,
    DischargeEnd,
    DischargeDuration,
    DischargeCutoffCurrent,
    SourceVoltage,
    SourceCurrent,
    LoadCurrent,
    LoadVoltage,
    Count,
};

struct BatterySettings {
    BatteryChemistry chemistry = BatteryChemistry::LiIon;
    int cells = 1;
    int cell_min_mv = 3000;
    int cell_max_mv = 4200;
    int overcurrent_ma = 10000;
    int protection_delay_ms = 100;
    int static_threshold_ma = 20;
    int filter_depth_ms = 400;
    int resistance_mohm = 0;
    bool force_amp_range = false;
    bool popup_configuration = true;
};

struct CutoffSettings {
    int start_mv;
    int end_mv;
    int duration_s;
    int cutoff_current_ua;
};

struct PowerSettings {
    int source_voltage_mv = 5000;
    int source_current_ma = 1000;
    int load_current_ma = 1000;
    int load_voltage_mv = 5000;
    bool load_cc = true;
    bool cycle_running = false;
};

struct NumericSpec {
    NumericField field;
    const char *title;
    int *value;
    int minimum;
    int maximum;
    int scale;
    uint8_t decimals;

    NumericSpec()
        : field(NumericField::None), title(""), value(nullptr),
          minimum(0), maximum(0), scale(1), decimals(0) {}

    NumericSpec(NumericField field_value, const char *title_value,
                int *value_pointer, int minimum_value, int maximum_value,
                int scale_value, uint8_t decimals_value)
        : field(field_value), title(title_value), value(value_pointer),
          minimum(minimum_value), maximum(maximum_value),
          scale(scale_value), decimals(decimals_value) {}
};

struct PowerDigitCommand {
    NumericField field;
    uint8_t digit;
};

struct PowerStepCommand {
    NumericField field;
    int direction;
};

constexpr WorkMode MODE_OPTIONS[] = {
    WorkMode::BatterySimulator,
    WorkMode::BidirectionalPower,
};
constexpr BatteryChemistry BATTERY_CHEMISTRY_OPTIONS[] = {
    BatteryChemistry::LiIon,
    BatteryChemistry::LiFePO4,
    BatteryChemistry::LiPoHv,
    BatteryChemistry::LeadAcid,
};
constexpr int VOLTAGE_DIGIT_STEPS_MV[] = {10000, 1000, 100, 10, 1};
constexpr int VOLTAGE_STEP_DIRECTIONS[] = {-1, 1};
constexpr uint8_t VOLTAGE_DIGIT_INDICES[] = {0, 1, 2, 3, 4};
constexpr bool CUTOFF_TYPE_OPTIONS[] = {true, false};
constexpr int QUICK_PERCENTAGES[] = {10, 20, 40, 60, 80, 100};
constexpr int MAX_BATTERY_PACK_MV = 25000;
// SETup:OUT 的 OK 回应可能早于上一笔 MEASure:ALL? 的状态被界面消费。
// 在输出切换确认后的短窗口内，不能让这笔切换前的旧测量结果把波形页
// 又切回来；等待一笔与确认状态一致的新测量，超时后再以实测为准。
constexpr uint32_t OUTPUT_STATE_MEASUREMENT_FENCE_TIMEOUT_MS = 1250U;
constexpr size_t POWER_DIGIT_FIELD_COUNT = 4;
constexpr NumericField POWER_DIGIT_FIELDS[POWER_DIGIT_FIELD_COUNT] = {
    NumericField::SourceVoltage,
    NumericField::SourceCurrent,
    NumericField::LoadCurrent,
    NumericField::LoadVoltage,
};
constexpr PowerDigitCommand
    POWER_DIGIT_COMMANDS[POWER_DIGIT_FIELD_COUNT][5] = {
        {{NumericField::SourceVoltage, 0},
         {NumericField::SourceVoltage, 1},
         {NumericField::SourceVoltage, 2},
         {NumericField::SourceVoltage, 3},
         {NumericField::SourceVoltage, 4}},
        {{NumericField::SourceCurrent, 0},
         {NumericField::SourceCurrent, 1},
         {NumericField::SourceCurrent, 2},
         {NumericField::SourceCurrent, 3},
         {NumericField::SourceCurrent, 4}},
        {{NumericField::LoadCurrent, 0},
         {NumericField::LoadCurrent, 1},
         {NumericField::LoadCurrent, 2},
         {NumericField::LoadCurrent, 3},
         {NumericField::LoadCurrent, 4}},
        {{NumericField::LoadVoltage, 0},
         {NumericField::LoadVoltage, 1},
         {NumericField::LoadVoltage, 2},
         {NumericField::LoadVoltage, 3},
         {NumericField::LoadVoltage, 4}},
};
constexpr PowerStepCommand
    POWER_STEP_COMMANDS[POWER_DIGIT_FIELD_COUNT][2] = {
        {{NumericField::SourceVoltage, -1},
         {NumericField::SourceVoltage, 1}},
        {{NumericField::SourceCurrent, -1},
         {NumericField::SourceCurrent, 1}},
        {{NumericField::LoadCurrent, -1},
         {NumericField::LoadCurrent, 1}},
        {{NumericField::LoadVoltage, -1},
         {NumericField::LoadVoltage, 1}},
};

WorkMode active_mode = WorkMode::BatterySimulator;
CommunicationMode communication_mode = CommunicationMode::Controller;
enum class DeviceModeTransitionPhase : uint8_t {
    Idle = 0,
    WaitOutputOff,
    WaitModeAck,
    WaitModeVerify,
};
DeviceModeTransitionPhase device_mode_transition_phase =
    DeviceModeTransitionPhase::Idle;
JcyScpiMode requested_device_mode = JcyScpiMode::Battery;
JcyScpiMode confirmed_device_mode = JcyScpiMode::Battery;
bool confirmed_device_mode_valid = false;
bool mode_transition_from_ui = false;
uint32_t device_mode_transition_started_ms = 0;
// SETup:MODE must never be sent merely because an OUT OFF command left the
// queue. It may have timed out or been rejected. This latch is set only by
// the lower instrument's successful OUT OFF acknowledgement.
bool mode_transition_output_off_confirmed = false;
bool output_running = false;
bool requested_output_state = false;
bool output_change_pending = false;
struct OutputMeasurementFence {
    bool active = false;
    bool target_output_on = false;
    uint32_t baseline_sequence = 0;
    uint32_t armed_ms = 0;
};
OutputMeasurementFence output_measurement_fence;
bool boot_safe_output_off_active = false;
uint8_t boot_safe_output_off_attempts = 0;
uint32_t boot_safe_output_off_last_request_ms = 0;
int set_voltage_mv = 3800;
uint8_t voltage_edit_digit = 2;
BatterySettings battery_settings;
BatterySettings battery_settings_snapshot;
CutoffSettings charge_settings = {3600, 4250, 10, 20000};
CutoffSettings discharge_settings = {4250, 3000, 10, 20000};
PowerSettings power_settings;
PageKind current_page = PageKind::None;
PowerTab active_power_tab = PowerTab::Source;
bool cutoff_running = false;
bool cutoff_is_charge = true;
CutoffRunPhase cutoff_phase = CutoffRunPhase::Idle;
uint32_t cutoff_last_query_ms = 0;
uint32_t cutoff_last_result_sequence = 0;
uint32_t cutoff_started_ms = 0;
bool constant_current_mode = false;
bool requested_constant_current_mode = false;
bool constant_current_change_pending = false;
bool main_waveform_visible = false;
uint32_t main_waveform_last_sequence = 0;
int main_wave_current_range_ma = 1000;
int cutoff_current_range_ma = 100;
bool measurement_long_press_shown = false;
uint32_t measurement_press_started = 0;
uint32_t last_live_ui_refresh_ms = 0;
uint32_t last_measurement_sequence = 0;
// 下位机的每次轮询回应都会增加 sequence，即使电压、电流和状态并没有
// 改变。待机画面若按 sequence 刷新，会在 RGB 双缓冲之间不停交换帧，肉眼
// 会看到静态内容轻微跳动。此缓存只用于判断“已显示的内容”是否真的变化。
struct IdleRenderCache {
    bool valid = false;
    bool fresh = false;
    bool online = false;
    bool output_on = false;
    bool output_change_pending = false;
    bool requested_output_state = false;
    int voltage_mv = 0;
    int current_ua = 0;
    int temperature_c10 = 0;
    int error_code = 0;
    char status[sizeof(JcyScpiMeasurements::status)] = {};
};
IdleRenderCache idle_render_cache;
int last_scpi_command_error = 0;
uint32_t last_scpi_command_error_ms = 0;
uint32_t last_identity_request_ms = 0;
uint32_t last_identity_sequence = 0;

lv_obj_t *mode_buttons[2] = {nullptr, nullptr};
lv_obj_t *mode_labels[2] = {nullptr, nullptr};
lv_obj_t *mode_battery_cells[5] = {};
lv_obj_t *status_led = nullptr;
lv_obj_t *status_badge = nullptr;
lv_obj_t *status_value_label = nullptr;
lv_obj_t *measurement_voltage_label = nullptr;
lv_obj_t *measurement_current_label = nullptr;
lv_obj_t *measurement_current_limit_label = nullptr;
lv_obj_t *measurement_temperature_label = nullptr;
lv_obj_t *measurement_power_label = nullptr;
lv_obj_t *measurement_popup_voltage_label = nullptr;
lv_obj_t *measurement_popup_current_label = nullptr;
lv_obj_t *measurement_popup_power_label = nullptr;
lv_obj_t *measurement_soc_label = nullptr;
lv_obj_t *measurement_soc_caption_label = nullptr;
lv_obj_t *measurement_battery_shell = nullptr;
lv_obj_t *measurement_battery_cells[5] = {};
lv_obj_t *set_voltage_header_label = nullptr;
lv_obj_t *set_voltage_digit_boxes[5] = {};
lv_obj_t *set_voltage_digit_labels[5] = {};
lv_obj_t *power_button = nullptr;
lv_obj_t *power_icon = nullptr;
lv_obj_t *power_icon_stem = nullptr;
lv_obj_t *power_switch_track = nullptr;
lv_obj_t *power_switch_knob = nullptr;
lv_obj_t *power_switch_label = nullptr;
lv_obj_t *alarm_bar = nullptr;
lv_obj_t *alarm_icon = nullptr;
lv_obj_t *alarm_label = nullptr;
JcyAlarmKind current_alarm_kind = JcyAlarmKind::None;
int current_alarm_code = 0;
uint32_t voltage_last_tap_ms = 0;
uint8_t voltage_last_tap_digit = 0xFF;
lv_obj_t *ui_screen = nullptr;
lv_obj_t *splash_screen = nullptr;
lv_timer_t *splash_auto_timer = nullptr;
lv_timer_t *splash_anim_timer = nullptr;
lv_obj_t *splash_anim_image = nullptr;
uint32_t splash_anim_index = 0;
lv_obj_t *page_overlay = nullptr;
lv_obj_t *keypad_overlay = nullptr;
lv_obj_t *info_overlay = nullptr;
lv_obj_t *keypad_value_label = nullptr;
lv_obj_t *keypad_error_label = nullptr;
lv_obj_t *numeric_value_labels[static_cast<size_t>(NumericField::Count)] = {};
lv_obj_t *quick_voltage_labels[6] = {};
lv_obj_t *quick_voltage_buttons[6] = {};
lv_obj_t *minimum_voltage_label = nullptr;
lv_obj_t *maximum_voltage_label = nullptr;
lv_obj_t *constant_current_card = nullptr;
lv_obj_t *constant_current_card_value_label = nullptr;
lv_obj_t *constant_current_title_label = nullptr;
lv_obj_t *constant_current_switch_track = nullptr;
lv_obj_t *constant_current_switch_knob = nullptr;
lv_obj_t *constant_current_overcurrent_label = nullptr;
lv_obj_t *constant_current_constant_label = nullptr;
lv_obj_t *constant_current_active_underline = nullptr;
lv_obj_t *battery_card_value_label = nullptr;
lv_obj_t *cutoff_card_value_label = nullptr;
lv_obj_t *settings_idle_content = nullptr;
lv_obj_t *settings_wave_content = nullptr;
lv_obj_t *main_wave_chart = nullptr;
lv_chart_series_t *main_voltage_series = nullptr;
lv_chart_series_t *main_current_series = nullptr;
lv_obj_t *main_wave_voltage_label = nullptr;
lv_obj_t *main_wave_current_label = nullptr;
lv_obj_t *main_wave_digit_boxes[5] = {};
lv_obj_t *main_wave_digit_labels[5] = {};
lv_obj_t *force_amp_button = nullptr;
lv_obj_t *force_amp_label = nullptr;
lv_obj_t *popup_config_button = nullptr;
lv_obj_t *popup_config_label = nullptr;
lv_obj_t *battery_chemistry_buttons[4] = {};
lv_obj_t *battery_chemistry_labels[4] = {};
lv_obj_t *battery_cell_range_label = nullptr;
lv_obj_t *battery_pack_range_label = nullptr;
lv_obj_t *battery_cells_value_label = nullptr;
lv_obj_t *cutoff_start_button = nullptr;
lv_obj_t *cutoff_start_label = nullptr;
lv_obj_t *cutoff_status_label = nullptr;
lv_obj_t *cutoff_realtime_voltage_label = nullptr;
lv_obj_t *cutoff_realtime_current_label = nullptr;
lv_obj_t *cutoff_result_label = nullptr;
lv_obj_t *cutoff_elapsed_label = nullptr;
lv_obj_t *cutoff_charge_button = nullptr;
lv_obj_t *cutoff_discharge_button = nullptr;
lv_obj_t *cutoff_chart = nullptr;
lv_chart_series_t *cutoff_voltage_series = nullptr;
lv_chart_series_t *cutoff_current_series = nullptr;
lv_obj_t *power_content = nullptr;
lv_obj_t *power_tab_buttons[3] = {};
lv_obj_t *power_output_label = nullptr;
lv_obj_t *power_page_output_button = nullptr;
lv_obj_t *power_page_output_icon = nullptr;
lv_obj_t *power_page_output_stem = nullptr;
lv_obj_t *cycle_button_label = nullptr;
lv_obj_t *power_live_voltage_label = nullptr;
lv_obj_t *power_live_current_label = nullptr;
lv_obj_t *power_live_power_label = nullptr;
lv_obj_t *power_live_temperature_label = nullptr;
lv_obj_t *power_rating_label = nullptr;
lv_obj_t *power_page_status_bar = nullptr;
lv_obj_t *power_page_status_icon = nullptr;
lv_obj_t *power_page_status_label = nullptr;
lv_obj_t *power_digit_boxes[POWER_DIGIT_FIELD_COUNT][5] = {};
lv_obj_t *power_digit_labels[POWER_DIGIT_FIELD_COUNT][5] = {};
uint8_t power_edit_digits[POWER_DIGIT_FIELD_COUNT] = {2, 2, 2, 2};
uint8_t power_last_tap_digits[POWER_DIGIT_FIELD_COUNT] = {
    0xFF, 0xFF, 0xFF, 0xFF};
uint32_t power_last_tap_ms[POWER_DIGIT_FIELD_COUNT] = {};
lv_obj_t *wifi_status_label = nullptr;
lv_obj_t *wifi_info_label = nullptr;
lv_obj_t *wifi_ssid_textarea = nullptr;
lv_obj_t *wifi_password_textarea = nullptr;
lv_obj_t *wifi_scan_dropdown = nullptr;
lv_obj_t *wifi_keyboard = nullptr;
lv_obj_t *wifi_keyboard_target = nullptr;
lv_obj_t *wifi_connect_button_label = nullptr;
lv_obj_t *ble_status_label = nullptr;
lv_obj_t *ble_info_label = nullptr;
lv_obj_t *ble_toggle_button_label = nullptr;
lv_obj_t *ota_ip_label = nullptr;
lv_obj_t *ota_status_label = nullptr;
lv_obj_t *ota_info_label = nullptr;
lv_obj_t *ota_progress_bar = nullptr;
lv_obj_t *cloud_ota_remote_label = nullptr;
lv_obj_t *cloud_ota_version_label = nullptr;
lv_obj_t *cloud_ota_notes_label = nullptr;
lv_obj_t *cloud_ota_progress_bar = nullptr;
lv_obj_t *cloud_ota_check_button = nullptr;
lv_obj_t *cloud_ota_download_button = nullptr;
lv_obj_t *lower_ota_status_label = nullptr;
lv_obj_t *lower_ota_version_label = nullptr;
lv_obj_t *lower_ota_notes_label = nullptr;
lv_obj_t *lower_ota_progress_bar = nullptr;
lv_obj_t *lower_ota_check_button = nullptr;
lv_obj_t *lower_ota_upgrade_button = nullptr;
lv_obj_t *ota_transfer_guard = nullptr;
lv_obj_t *ota_transfer_stage_label = nullptr;
lv_obj_t *ota_transfer_detail_label = nullptr;
lv_obj_t *ota_transfer_progress_label = nullptr;
lv_obj_t *ota_transfer_progress_bar = nullptr;
lv_obj_t *ota_transfer_return_button = nullptr;
JcyCloudOtaState ota_transfer_last_state = JcyCloudOtaState::Idle;
uint8_t ota_transfer_last_percent = 0xFF;
bool cloud_ota_quiet_active = false;
bool lower_ota_guard_active = false;
lv_obj_t *communication_controller_button = nullptr;
lv_obj_t *communication_passthrough_button = nullptr;
lv_obj_t *communication_controller_label = nullptr;
lv_obj_t *communication_passthrough_label = nullptr;
lv_obj_t *communication_status_label = nullptr;
lv_obj_t *communication_detail_label = nullptr;
int16_t wifi_scan_option_indices[12] = {};
uint8_t wifi_scan_option_count = 0;
bool wifi_scan_options_loaded = false;
uint32_t last_wireless_ui_refresh_ms = 0;

enum class WifiKeyboardLayout : uint8_t {
    Lowercase,
    Uppercase,
    Symbols,
};

enum class WifiKeyboardAction : uint8_t {
    Text,
    Lowercase,
    Uppercase,
    Symbols,
    Space,
    Backspace,
    Clear,
    Confirm,
    CursorLeft,
    CursorRight,
};

struct WifiKeyboardKey {
    const char *label;
    const char *text;
    WifiKeyboardAction action;
};

WifiKeyboardLayout wifi_keyboard_layout = WifiKeyboardLayout::Lowercase;
NumericSpec active_numeric_spec;
char numeric_input[24] = {};
bool numeric_input_replace_on_next_key = false;
int quick_voltages_mv[6] = {};

lv_color_t color(uint32_t rgb) {
    return lv_color_hex(rgb);
}

void open_page(PageKind page);
void close_page();
void open_numeric_field(NumericField field);
void refresh_main_values();
void rebuild_power_content();
void refresh_power_page_output_state();
void refresh_power_identity_ui();
void refresh_power_page_status();
void refresh_power_digit_editor(NumericField field);
void update_power_digit_selection(NumericField field);
void refresh_debug_overlay(uint32_t now_ms);
void show_info_dialog(const char *title, const char *body);
void refresh_mode_buttons();
void refresh_power_state();
void refresh_alarm_bar(int code, const char *status);
void refresh_set_voltage();
void update_voltage_digit_selection();
void refresh_battery_chemistry_label();
void refresh_battery_settings_page();
void refresh_live_measurements();
void sync_battery_settings_to_device();
bool request_device_mode(JcyScpiMode mode, bool from_ui);
bool device_mode_ready(JcyScpiMode mode);
bool current_control_mode_ready();
void apply_confirmed_mode_parameters(JcyScpiMode mode);
void continue_device_mode_transition();
void apply_output_change(bool enabled, bool from_ui);
bool request_boot_safe_output_off(uint32_t now_ms);
void process_boot_output_safety(uint32_t now_ms);
void update_battery_soc_display(int voltage_mv);
void format_current_text(char *buffer, size_t length, int current_ua);
void send_numeric_field_to_device(NumericField field);
void process_scpi_events();
void process_cutoff_state(uint32_t now_ms);
void create_cutoff_page();
void refresh_cutoff_page();
void refresh_constant_current_card();
void refresh_main_waveform_visibility();
void append_main_waveform_sample(const JcyScpiMeasurements &measurements);
void append_cutoff_waveform_sample(int voltage_mv, int current_ua);
void restore_default_poll_interval();
void normalize_battery_settings();
void refresh_wireless_header();
void refresh_wireless_ui();
void invalidate_idle_render_cache();
void set_label_text_if_changed(lv_obj_t *label, const char *text);
void set_button_disabled_if_changed(lv_obj_t *button, bool disabled);
void refresh_ota_ui();
void refresh_cloud_ota_ui();
void refresh_lower_ota_ui();
void show_ota_transfer_guard();
void hide_ota_transfer_guard();
void refresh_ota_transfer_guard();
void ota_transfer_return_event(lv_event_t *event);
void cloud_ota_check_event(lv_event_t *event);
void cloud_ota_download_event(lv_event_t *event);
void lower_ota_check_event(lv_event_t *event);
void lower_ota_upgrade_event(lv_event_t *event);
void open_wifi_page();
void create_wifi_keyboard_layout();
void open_ble_page();
void open_ota_page();
void create_communication_page();
void refresh_communication_mode_page();
bool set_communication_mode(CommunicationMode mode, bool from_ui);
void poll_passthrough_bridge(uint32_t now_ms);
void make_adjust_icon(lv_obj_t *parent, bool plus);

void splash_confirm_event(lv_event_t *event);

size_t numeric_index(NumericField field) {
    return static_cast<size_t>(field);
}

int power_digit_field_index(NumericField field) {
    for (size_t i = 0; i < POWER_DIGIT_FIELD_COUNT; ++i) {
        if (POWER_DIGIT_FIELDS[i] == field) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int power_max_voltage_mv() {
    const JcyScpiIdentity &identity = scpi_link.identity();
    return identity.valid && identity.primary_voltage_mv > 0
               ? identity.primary_voltage_mv
               : 25000;
}

int power_max_current_ma() {
    const JcyScpiIdentity &identity = scpi_link.identity();
    return identity.valid && identity.primary_current_ma > 0
               ? identity.primary_current_ma
               : 10000;
}

bool output_is_confirmed_off() {
    return !boot_safe_output_off_active && !output_running &&
           !requested_output_state && !output_change_pending;
}

bool require_output_off_for_action(const char *action) {
    if (output_is_confirmed_off()) {
        return true;
    }

    char body[192] = {};
    snprintf(
        body, sizeof(body),
        "当前输出仍在开启或正在切换。请先关闭输出，并等待状态确认已关闭后再%s。",
        action == nullptr || action[0] == '\0' ? "继续操作" : action);
    show_info_dialog("请先关闭输出", body);
    Serial.printf(
        "OUTPUT NAV GATE blocked action=%s running=%d requested=%d pending=%d boot=%d\n",
        action == nullptr ? "unknown" : action,
        output_running, requested_output_state, output_change_pending,
        boot_safe_output_off_active);
    return false;
}

bool page_requires_output_off(PageKind page) {
    switch (page) {
        case PageKind::BatterySettings:
        case PageKind::CutoffTest:
        case PageKind::SettingsHub:
        case PageKind::Power:
        case PageKind::Wifi:
        case PageKind::Bluetooth:
        case PageKind::Ota:
        case PageKind::Communication:
            return true;
        default:
            return false;
    }
}

const char *page_output_gate_action(PageKind page) {
    switch (page) {
        case PageKind::BatterySettings: return "进入电池设置";
        case PageKind::CutoffTest: return "进入截止电压测试";
        case PageKind::SettingsHub: return "进入系统设置";
        case PageKind::Power: return "切换到双向电源";
        case PageKind::Wifi: return "进入 Wi-Fi 设置";
        case PageKind::Bluetooth: return "进入蓝牙设置";
        case PageKind::Ota: return "进入系统升级";
        case PageKind::Communication: return "进入通讯模式";
        default: return "继续操作";
    }
}

bool cloud_ota_should_be_quiet() {
    const JcyCloudOtaState state = jcy_cloud_ota::status().state;
    return state == JcyCloudOtaState::Downloading ||
           state == JcyCloudOtaState::Verifying ||
           state == JcyCloudOtaState::Success;
}

bool lower_ota_should_be_quiet() {
    const JcyLowerMcuOtaState state = jcy_lower_mcu_ota::status().state;
    return state == JcyLowerMcuOtaState::Downloading ||
           state == JcyLowerMcuOtaState::EnteringBootloader ||
           state == JcyLowerMcuOtaState::Transferring ||
           state == JcyLowerMcuOtaState::Verifying ||
           state == JcyLowerMcuOtaState::Reconnecting;
}

void update_cloud_ota_quiet_mode() {
    const bool quiet = cloud_ota_should_be_quiet() ||
                       lower_ota_should_be_quiet();
    if (cloud_ota_quiet_active == quiet) {
        return;
    }
    cloud_ota_quiet_active = quiet;
    if (TTL_ENABLED && !scpi_link.iap_transport_active()) {
        scpi_link.set_polling_suspended(quiet);
    }
    if (!quiet) {
        invalidate_idle_render_cache();
    }
    Serial.printf("Cloud OTA display quiet mode %s\n",
                  quiet ? "ON" : "OFF");
}

int battery_max_cells() {
    const int cell_max_mv = max(1, battery_settings.cell_max_mv);
    return max(1, min(20, power_max_voltage_mv() / cell_max_mv));
}

int pack_minimum_mv() {
    return battery_settings.cell_min_mv * battery_settings.cells;
}

int pack_maximum_mv() {
    return battery_settings.cell_max_mv * battery_settings.cells;
}

void format_current_text(char *buffer, size_t length, int current_ua) {
    if (buffer == nullptr || length == 0) {
        return;
    }
    const long absolute = labs(static_cast<long>(current_ua));
    const char *prefix = current_ua < 0 ? "-" :
                         (current_ua > 0 ? "+" : "");
    if (absolute >= 1000000L) {
        const long whole_a = absolute / 1000000L;
        if (whole_a >= 10) {
            snprintf(buffer, length, "%s%ld.%02ldA", prefix,
                     whole_a, (absolute % 1000000L) / 10000L);
        } else {
            snprintf(buffer, length, "%s%ld.%03ldA", prefix,
                     whole_a, (absolute % 1000000L) / 1000L);
        }
    } else if (absolute >= 1000L) {
        const long whole_ma = absolute / 1000L;
        const long fraction = absolute % 1000L;
        if (whole_ma >= 100) {
            snprintf(buffer, length, "%s%ld.%01ldmA", prefix,
                     whole_ma, fraction / 100L);
        } else if (whole_ma >= 10) {
            snprintf(buffer, length, "%s%ld.%02ldmA", prefix,
                     whole_ma, fraction / 10L);
        } else {
            snprintf(buffer, length, "%s%ld.%03ldmA", prefix,
                     whole_ma, fraction);
        }
    } else {
        snprintf(buffer, length, "%s%lduA", prefix, absolute);
    }
}

// 下位机在输出关闭后仍持续采样，用于保护、日志和下一次启动前的安全检查。
// ADC 零点附近的小偏移只在显示层归零，原始测量值仍完整保留在
// scpi_link.measurements() 中，禁止影响控制、保护、SOC、波形判定和日志。
constexpr int IDLE_DISPLAY_VOLTAGE_DEADZONE_MV = 100;
constexpr int IDLE_DISPLAY_CURRENT_DEADZONE_UA = 10000;

struct DisplayMeasurementValues {
    int voltage_mv;
    int current_ua;
    bool current_zeroed;
};

DisplayMeasurementValues measurement_values_for_display(
    const JcyScpiMeasurements &measurements) {
    DisplayMeasurementValues values = {
        measurements.voltage_mv,
        measurements.current_ua,
        false,
    };
    if (!output_running) {
        if (abs(values.voltage_mv) < IDLE_DISPLAY_VOLTAGE_DEADZONE_MV) {
            values.voltage_mv = 0;
        }
        if (labs(static_cast<long>(values.current_ua)) <
            IDLE_DISPLAY_CURRENT_DEADZONE_UA) {
            values.current_ua = 0;
            values.current_zeroed = true;
        }
    }
    return values;
}

struct AdaptiveFontCandidate {
    uint8_t pixel_size;
    const lv_font_t *font;
};

const lv_font_t *fit_montserrat_numeric_font(const char *text,
                                              lv_coord_t max_width,
                                              uint8_t max_pixel_size) {
    static constexpr AdaptiveFontCandidate CANDIDATES[] = {
        {48, &lv_font_montserrat_48},
        {36, &lv_font_montserrat_36},
        {30, &lv_font_montserrat_30},
        {24, &lv_font_montserrat_24},
        {20, &lv_font_montserrat_20},
        {18, &lv_font_montserrat_18},
        {16, &lv_font_montserrat_16},
        {14, &lv_font_montserrat_14},
        {12, &lv_font_montserrat_12},
    };
    if (text == nullptr || max_width <= 0) {
        return &lv_font_montserrat_12;
    }
    const uint32_t text_length = static_cast<uint32_t>(strlen(text));
    for (const AdaptiveFontCandidate &candidate : CANDIDATES) {
        if (candidate.pixel_size > max_pixel_size) {
            continue;
        }
        const lv_coord_t rendered_width = lv_txt_get_width(
            text, text_length, candidate.font, 0, LV_TEXT_FLAG_NONE);
        if (rendered_width <= max_width) {
            return candidate.font;
        }
    }
    return &lv_font_montserrat_12;
}

void set_adaptive_numeric_label(lv_obj_t *label, const char *text,
                                uint8_t max_pixel_size) {
    if (label == nullptr || text == nullptr) {
        return;
    }
    // Never let a long live value wrap into the next measurement row.
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_label_set_text(label, text);
    lv_coord_t available_width = lv_obj_get_width(label) - 2;
    if (available_width < 1) {
        available_width = 1;
    }
    lv_obj_set_style_text_font(
        label,
        fit_montserrat_numeric_font(text, available_width,
                                    max_pixel_size),
        0);
}

const char *scpi_mode_name(JcyScpiMode mode) {
    switch (mode) {
        case JcyScpiMode::Battery: return "BATTERY";
        case JcyScpiMode::BatteryConstantCurrent: return "BATTER_CC";
        case JcyScpiMode::Source: return "SOURCE";
        case JcyScpiMode::Load: return "LOAD";
        case JcyScpiMode::ConstantResistance: return "CONST_R";
        default: return "UNKNOWN";
    }
}

void sync_battery_settings_to_device() {
    normalize_battery_settings();
    request_device_mode(constant_current_mode
                           ? JcyScpiMode::BatteryConstantCurrent
                           : JcyScpiMode::Battery,
                        false);
}

bool device_mode_ready(JcyScpiMode mode) {
    return device_mode_transition_phase ==
               DeviceModeTransitionPhase::Idle &&
           confirmed_device_mode_valid &&
           confirmed_device_mode == mode;
}

bool current_control_mode_ready() {
    if (active_mode == WorkMode::BatterySimulator) {
        return device_mode_ready(
            constant_current_mode
                ? JcyScpiMode::BatteryConstantCurrent
                : JcyScpiMode::Battery);
    }
    if (active_power_tab == PowerTab::Source) {
        return device_mode_ready(JcyScpiMode::Source);
    }
    if (active_power_tab == PowerTab::Load) {
        return device_mode_ready(JcyScpiMode::Load);
    }
    return false;
}

void apply_confirmed_mode_parameters(JcyScpiMode mode) {
    if (mode == JcyScpiMode::Source) {
        scpi_link.set_voltage_mv(power_settings.source_voltage_mv);
        scpi_link.set_current_ma(power_settings.source_current_ma);
        return;
    }
    if (mode == JcyScpiMode::Load) {
        if (power_settings.load_cc) {
            scpi_link.set_current_ma(power_settings.load_current_ma);
        } else {
            scpi_link.set_voltage_mv(power_settings.load_voltage_mv);
        }
        return;
    }

    normalize_battery_settings();
    scpi_link.set_voltage_mv(set_voltage_mv);
    scpi_link.set_current_ma(battery_settings.overcurrent_ma);
    scpi_link.set_protection_delay_ms(
        battery_settings.protection_delay_ms);
    scpi_link.set_current_mode(battery_settings.force_amp_range);
}

bool request_device_mode(JcyScpiMode mode, bool from_ui) {
    requested_device_mode = mode;
    mode_transition_from_ui = from_ui;

    if (device_mode_ready(mode)) {
        apply_confirmed_mode_parameters(mode);
        Serial.printf("MODE READY resync=%d\n", static_cast<int>(mode));
        return true;
    }

    // 切换模式前始终先关闭输出。各模式的电压/电流值保存在各自的
    // battery_settings / power_settings 中，不在切换时恢复默认值。
    mode_transition_output_off_confirmed = false;
    if (!scpi_link.set_output(false)) {
        if (from_ui) {
            show_info_dialog("模式切换失败",
                             "底层命令队列忙，请稍后重试。");
        }
        return false;
    }
    requested_output_state = false;
    output_change_pending = true;
    device_mode_transition_phase =
        DeviceModeTransitionPhase::WaitOutputOff;
    device_mode_transition_started_ms = millis();
    refresh_power_state();
    refresh_main_waveform_visibility();
    Serial.printf("MODE TRANSITION target=%d wait=OUT_OFF\n",
                  static_cast<int>(mode));
    return true;
}

void continue_device_mode_transition() {
    if (device_mode_transition_phase !=
        DeviceModeTransitionPhase::WaitOutputOff) {
        return;
    }
    // Wait for the actual OUT OFF acknowledgement, not merely for the
    // command queue to become empty. This prevents a failed OUT OFF command
    // from being followed by a mode switch while the instrument may still be
    // driving its output.
    if (!mode_transition_output_off_confirmed &&
        millis() - device_mode_transition_started_ms < 900U) {
        return;
    }

    if (!mode_transition_output_off_confirmed) {
        device_mode_transition_phase = DeviceModeTransitionPhase::Idle;
        if (constant_current_change_pending) {
            requested_constant_current_mode = constant_current_mode;
            constant_current_change_pending = false;
            refresh_constant_current_card();
        }
        if (mode_transition_from_ui) {
            show_info_dialog("模式切换失败",
                             "底层未确认输出已关闭，未切换工作模式。");
        }
        Serial.printf("MODE TRANSITION aborted target=%s OUT_OFF not confirmed\n",
                      scpi_mode_name(requested_device_mode));
        return;
    }

    output_running = false;
    requested_output_state = false;
    output_change_pending = false;
    if (!scpi_link.set_mode(requested_device_mode)) {
        device_mode_transition_phase =
            DeviceModeTransitionPhase::Idle;
        mode_transition_output_off_confirmed = false;
        confirmed_device_mode_valid = false;
        if (mode_transition_from_ui) {
            show_info_dialog("模式切换失败",
                             "SETup:MODE 命令未能排队。");
        }
        return;
    }
    device_mode_transition_phase =
        DeviceModeTransitionPhase::WaitModeAck;
    device_mode_transition_started_ms = millis();
    Serial.printf("MODE TRANSITION target=%d wait=MODE_ACK\n",
                  static_cast<int>(requested_device_mode));
}

void restore_default_poll_interval() {
    scpi_link.set_poll_interval(output_running ? 100 : 350);
}

void arm_output_measurement_fence(bool target_output_on) {
    output_measurement_fence.active = true;
    output_measurement_fence.target_output_on = target_output_on;
    output_measurement_fence.baseline_sequence =
        scpi_link.measurements().sequence;
    output_measurement_fence.armed_ms = millis();
    Serial.printf("OUTPUT UI fence target=%d seq=%lu\n", target_output_on,
                  static_cast<unsigned long>(
                      output_measurement_fence.baseline_sequence));
}

bool accept_measured_output_state(const JcyScpiMeasurements &measurements,
                                  uint32_t now_ms) {
    if (!output_measurement_fence.active) {
        return true;
    }

    // 该回应在 OUT 命令确认前就已收到，状态一定不能覆盖确认后的 UI。
    if (measurements.sequence == output_measurement_fence.baseline_sequence) {
        return false;
    }

    if (measurements.output_on == output_measurement_fence.target_output_on) {
        Serial.printf("OUTPUT UI fence release target=%d seq=%lu\n",
                      output_measurement_fence.target_output_on,
                      static_cast<unsigned long>(measurements.sequence));
        output_measurement_fence.active = false;
        return true;
    }

    // 正常情况下下一笔轮询会与 OUT ACK 一致。若底层实际没有切换，
    // 不能无限期以界面意图覆盖实测状态；超时后回退到实测状态。
    if (now_ms - output_measurement_fence.armed_ms <
        OUTPUT_STATE_MEASUREMENT_FENCE_TIMEOUT_MS) {
        return false;
    }
    Serial.printf("OUTPUT UI fence timeout target=%d measured=%d seq=%lu\n",
                  output_measurement_fence.target_output_on,
                  measurements.output_on,
                  static_cast<unsigned long>(measurements.sequence));
    output_measurement_fence.active = false;
    return true;
}

bool displayed_output_state(const JcyScpiMeasurements &measurements) {
    if (output_change_pending) {
        return requested_output_state;
    }
    if (output_measurement_fence.active) {
        return output_running;
    }
    return measurements.output_on;
}

void send_numeric_field_to_device(NumericField field) {
    JcyScpiMode required_mode = JcyScpiMode::Battery;
    bool requires_mode = true;
    switch (field) {
        case NumericField::SetVoltage:
            required_mode = constant_current_mode
                                ? JcyScpiMode::BatteryConstantCurrent
                                : JcyScpiMode::Battery;
            break;
        case NumericField::SourceVoltage:
        case NumericField::SourceCurrent:
            required_mode = JcyScpiMode::Source;
            break;
        case NumericField::LoadCurrent:
        case NumericField::LoadVoltage:
            required_mode = JcyScpiMode::Load;
            break;
        default:
            requires_mode = false;
            break;
    }
    if (requires_mode && !device_mode_ready(required_mode)) {
        // 只更新保存在 RAM 中的目标值；模式 ACK 后会由
        // apply_confirmed_mode_parameters 下发最新值。
        Serial.printf("MODE GATE hold field=%u target=%d\n",
                      static_cast<unsigned>(field),
                      static_cast<int>(required_mode));
        return;
    }
    switch (field) {
        case NumericField::SetVoltage:
            scpi_link.set_voltage_mv(set_voltage_mv);
            break;
        case NumericField::SourceVoltage:
            scpi_link.set_voltage_mv(power_settings.source_voltage_mv);
            break;
        case NumericField::SourceCurrent:
            scpi_link.set_current_ma(power_settings.source_current_ma);
            break;
        case NumericField::LoadCurrent:
            scpi_link.set_current_ma(power_settings.load_current_ma);
            break;
        case NumericField::LoadVoltage:
            scpi_link.set_voltage_mv(power_settings.load_voltage_mv);
            break;
        default:
            break;
    }
}

void apply_output_change(bool enabled, bool from_ui) {
    if (enabled && !current_control_mode_ready()) {
        requested_output_state = false;
        if (from_ui) {
            show_info_dialog(
                "模式同步中",
                active_power_tab == PowerTab::Cycle
                    ? "充放电模式请从工步设置中启动，不能直接开启输出。"
                    : "请等待底层工作模式确认完成后再开启输出。");
        }
        Serial.println("MODE GATE blocked OUT ON");
        return;
    }
    if (enabled && boot_safe_output_off_active) {
        requested_output_state = false;
        if (from_ui) {
            show_info_dialog(
                "安全关断确认中",
                "开机后必须先确认下位机输出为OFF，确认完成前禁止开启输出。");
        }
        Serial.println("BOOT SAFE: blocked OUT ON before OFF verification");
        return;
    }

    // Do not discard a second tap while the previous output command is still
    // waiting for ACK. JcyScpiLink coalesces queued output commands, so this
    // always records the user's newest intent; OFF is additionally prioritized.
    if (!scpi_link.set_output(enabled)) {
        if (from_ui) {
            show_info_dialog("命令未发送",
                             "TTL命令队列忙，请稍后重试。关闭输出会保持最高优先级。");
        }
        Serial.printf("OUTPUT intent queue failed value=%d\n", enabled);
        return;
    }
    requested_output_state = enabled;
    output_change_pending =
        scpi_link.command_pending(JcyScpiCommandKind::SetOutput);
    // 新的 OUT 意图取代上一轮确认后的测量栅栏；尤其是 ON->OFF 时，
    // 要在命令应答到达前立即收起波形页，不能等下一次350ms轮询。
    output_measurement_fence.active = false;
    refresh_main_waveform_visibility();
    refresh_power_state();
    refresh_power_page_output_state();
    if (power_output_label != nullptr && output_change_pending) {
        lv_label_set_text(power_output_label,
                          enabled ? "输出开启发送中" : "输出关闭发送中");
    }
    Serial.printf("OUTPUT intent value=%d pending=%d\n",
                  enabled, output_change_pending);
}

bool request_boot_safe_output_off(uint32_t now_ms) {
    if (!TTL_ENABLED ||
        scpi_link.command_pending(JcyScpiCommandKind::SetOutput)) {
        return false;
    }
    if (!scpi_link.set_output(false)) {
        return false;
    }
    requested_output_state = false;
    output_change_pending = false;
    boot_safe_output_off_last_request_ms = now_ms;
    if (boot_safe_output_off_attempts < 255) {
        ++boot_safe_output_off_attempts;
    }
    Serial.printf("BOOT SAFE: queued OUT OFF attempt=%u\n",
                  static_cast<unsigned>(boot_safe_output_off_attempts));
    return true;
}

void process_boot_output_safety(uint32_t now_ms) {
    if (!boot_safe_output_off_active || !TTL_ENABLED) {
        return;
    }

    const JcyScpiMeasurements &measurements = scpi_link.measurements();
    if (scpi_link.measurements_fresh(now_ms, 1200) &&
        !measurements.output_on) {
        boot_safe_output_off_active = false;
        output_running = false;
        requested_output_state = false;
        output_change_pending = false;
        restore_default_poll_interval();
        refresh_power_state();
        refresh_main_waveform_visibility();
        Serial.printf("BOOT SAFE: OUT OFF verified attempts=%u\n",
                      static_cast<unsigned>(boot_safe_output_off_attempts));
        return;
    }

    const uint32_t retry_interval_ms =
        boot_safe_output_off_attempts < 5 ? 250U : 2000U;
    if (!scpi_link.command_pending(JcyScpiCommandKind::SetOutput) &&
        now_ms - boot_safe_output_off_last_request_ms >=
            retry_interval_ms) {
        request_boot_safe_output_off(now_ms);
    }
}

void fail_cutoff_test(const char *message, int error_code) {
    cutoff_running = false;
    cutoff_phase = CutoffRunPhase::Failed;
    last_scpi_command_error = error_code;
    last_scpi_command_error_ms = millis();
    scpi_link.set_output(false);
    restore_default_poll_interval();
    if (cutoff_status_label != nullptr) {
        lv_label_set_text_fmt(cutoff_status_label, "%s E%03d",
                              message == nullptr ? "通信错误" : message,
                              max(error_code, 0));
    }
    refresh_cutoff_page();
}

bool enqueue_active_cutoff_ramp() {
    const CutoffSettings &settings = cutoff_is_charge
                                         ? charge_settings
                                         : discharge_settings;
    return scpi_link.set_ramp(
        settings.start_mv, settings.end_mv,
        settings.duration_s * 1000,
        battery_settings.overcurrent_ma * 1000,
        settings.cutoff_current_ua);
}

void process_scpi_events() {
    JcyScpiEvent event;
    while (scpi_link.take_event(event)) {
        if (event.success) {
            last_scpi_command_error = 0;

            if (event.kind == JcyScpiCommandKind::QueryIdentity) {
                const JcyScpiIdentity &identity = scpi_link.identity();
                if (identity.valid &&
                    identity.sequence != last_identity_sequence) {
                    last_identity_sequence = identity.sequence;
                    power_settings.source_voltage_mv = constrain(
                        power_settings.source_voltage_mv, 0,
                        power_max_voltage_mv());
                    power_settings.load_voltage_mv = constrain(
                        power_settings.load_voltage_mv, 0,
                        power_max_voltage_mv());
                    power_settings.source_current_ma = constrain(
                        power_settings.source_current_ma, 0,
                        power_max_current_ma());
                    power_settings.load_current_ma = constrain(
                        power_settings.load_current_ma, 0,
                        power_max_current_ma());
                    if (current_page == PageKind::Power) {
                        rebuild_power_content();
                    }
                    refresh_power_identity_ui();
                    Serial.printf(
                        "TTL IDN model=%s spec=%s primary=%s fw=%s\n",
                        identity.model, identity.specification,
                        identity.primary_rating, identity.firmware);
                }
            } else if (event.kind == JcyScpiCommandKind::SetOutput &&
                       device_mode_transition_phase ==
                           DeviceModeTransitionPhase::WaitOutputOff &&
                       event.requested_value == 0) {
                // This is the only path that allows the transition to send
                // SETup:MODE. A queue drain or a timeout is not proof that
                // the physical output is actually off.
                mode_transition_output_off_confirmed = true;
                Serial.printf("MODE TRANSITION OUT_OFF acknowledged target=%s\n",
                              scpi_mode_name(requested_device_mode));
            } else if (event.kind == JcyScpiCommandKind::SetMode) {
                const JcyScpiMode acknowledged_mode =
                    static_cast<JcyScpiMode>(event.requested_value);
                if (device_mode_transition_phase ==
                        DeviceModeTransitionPhase::WaitModeAck &&
                    acknowledged_mode == requested_device_mode) {
                    // SETup:MODE returns OK when the command is accepted, but
                    // the on-screen mode must wait for SETup:MODE? to report
                    // the lower instrument's actual state.  This keeps the
                    // overcurrent/constant-current switch from becoming a
                    // UI-only indicator.
                    if (!scpi_link.query_mode()) {
                        device_mode_transition_phase =
                            DeviceModeTransitionPhase::Idle;
                        confirmed_device_mode_valid = false;
                        if (constant_current_change_pending) {
                            requested_constant_current_mode =
                                constant_current_mode;
                            constant_current_change_pending = false;
                            refresh_constant_current_card();
                        }
                        if (mode_transition_from_ui) {
                            show_info_dialog(
                                "模式切换失败",
                                "SETup:MODE? 校验命令未能排队。");
                        }
                        Serial.printf(
                            "MODE VERIFY queue failed expected=%s\n",
                            scpi_mode_name(requested_device_mode));
                    } else {
                        device_mode_transition_phase =
                            DeviceModeTransitionPhase::WaitModeVerify;
                        device_mode_transition_started_ms = millis();
                        Serial.printf(
                            "MODE TRANSITION expected=%s wait=MODE_QUERY\n",
                            scpi_mode_name(requested_device_mode));
                    }
                } else {
                    // Direct mode writes used by the cutoff-test state
                    // machine retain their existing ACK behavior. All normal
                    // page and overcurrent/constant-current transitions go
                    // through request_device_mode() and are verified above.
                    confirmed_device_mode = acknowledged_mode;
                    confirmed_device_mode_valid = true;
                }
                if (cutoff_phase == CutoffRunPhase::EnsureBatteryMode &&
                    event.requested_value ==
                        static_cast<int>(JcyScpiMode::Battery)) {
                    constant_current_mode = false;
                    requested_constant_current_mode = false;
                    constant_current_change_pending = false;
                    refresh_constant_current_card();
                    if (enqueue_active_cutoff_ramp()) {
                        cutoff_phase = CutoffRunPhase::ConfigureRamp;
                    } else {
                        fail_cutoff_test("命令队列满", -3);
                    }
                    refresh_cutoff_page();
                }
            } else if (event.kind == JcyScpiCommandKind::QueryMode) {
                const JcyScpiMode reported_mode =
                    static_cast<JcyScpiMode>(event.requested_value);
                const bool verifies_requested_mode =
                    device_mode_transition_phase ==
                    DeviceModeTransitionPhase::WaitModeVerify;
                confirmed_device_mode = reported_mode;
                confirmed_device_mode_valid = true;

                if (verifies_requested_mode) {
                    const bool matches_requested =
                        reported_mode == requested_device_mode;
                    device_mode_transition_phase =
                        DeviceModeTransitionPhase::Idle;

                    // The battery-current selector follows the mode actually
                    // read back from the lower instrument, never the user's
                    // tap intent. A non-battery mismatch falls back to the
                    // safer overcurrent appearance and leaves output gated by
                    // current_control_mode_ready().
                    if (constant_current_change_pending) {
                        constant_current_mode =
                            reported_mode ==
                            JcyScpiMode::BatteryConstantCurrent;
                        requested_constant_current_mode =
                            constant_current_mode;
                        constant_current_change_pending = false;
                        refresh_constant_current_card();
                    }

                    if (matches_requested) {
                        apply_confirmed_mode_parameters(reported_mode);
                        Serial.printf(
                            "MODE VERIFY pass expected=%s actual=%s\n",
                            scpi_mode_name(requested_device_mode),
                            scpi_mode_name(reported_mode));
                    } else {
                        if (mode_transition_from_ui) {
                            char body[160] = {};
                            snprintf(
                                body, sizeof(body),
                                "底层返回 %s，未进入请求的 %s。输出保持关闭。",
                                scpi_mode_name(reported_mode),
                                scpi_mode_name(requested_device_mode));
                            show_info_dialog("模式未确认", body);
                        }
                        Serial.printf(
                            "MODE VERIFY mismatch expected=%s actual=%s\n",
                            scpi_mode_name(requested_device_mode),
                            scpi_mode_name(reported_mode));
                    }
                }
            } else if (event.kind == JcyScpiCommandKind::SetRamp &&
                       cutoff_phase == CutoffRunPhase::ConfigureRamp) {
                if (scpi_link.trigger_ramp()) {
                    cutoff_phase = CutoffRunPhase::TriggerRamp;
                } else {
                    fail_cutoff_test("触发排队失败", -3);
                }
                refresh_cutoff_page();
            } else if (event.kind == JcyScpiCommandKind::TriggerRamp &&
                       cutoff_phase == CutoffRunPhase::TriggerRamp) {
                cutoff_phase = CutoffRunPhase::Running;
                cutoff_running = true;
                cutoff_started_ms = millis();
                cutoff_last_query_ms = 0;
                scpi_link.set_poll_interval(100);
                refresh_cutoff_page();
            }

            if (event.kind == JcyScpiCommandKind::SetOutput &&
                output_change_pending) {
                const bool confirmed_state = event.requested_value != 0;
                output_running = confirmed_state;

                bool command_still_pending =
                    scpi_link.command_pending(
                        JcyScpiCommandKind::SetOutput);
                if (confirmed_state != requested_output_state &&
                    !command_still_pending) {
                    command_still_pending =
                        scpi_link.set_output(requested_output_state);
                }

                const bool settled =
                    confirmed_state == requested_output_state &&
                    !command_still_pending;
                output_change_pending = !settled;
                if (settled) {
                    arm_output_measurement_fence(confirmed_state);
                    restore_default_poll_interval();
                }
                refresh_power_state();
                refresh_main_waveform_visibility();
                refresh_power_page_output_state();
                if (power_output_label != nullptr && settled) {
                    lv_label_set_text(power_output_label,
                                      confirmed_state
                                          ? "输出已开启"
                                          : "输出已关闭");
                }
                Serial.printf(
                    "OUTPUT ACK confirmed=%d desired=%d pending=%d\n",
                    confirmed_state, requested_output_state,
                    output_change_pending);
            }
            Serial.printf("TTL OK kind=%u value=%d\n",
                          static_cast<unsigned>(event.kind),
                          event.requested_value);
        } else {
            last_scpi_command_error = event.error_code;
            last_scpi_command_error_ms = millis();

            const bool cutoff_command =
                (cutoff_phase == CutoffRunPhase::EnsureBatteryMode &&
                 event.kind == JcyScpiCommandKind::SetMode) ||
                (cutoff_phase == CutoffRunPhase::ConfigureRamp &&
                 event.kind == JcyScpiCommandKind::SetRamp) ||
                (cutoff_phase == CutoffRunPhase::TriggerRamp &&
                 event.kind == JcyScpiCommandKind::TriggerRamp) ||
                (cutoff_phase == CutoffRunPhase::Running &&
                 event.kind == JcyScpiCommandKind::QueryRampState);
            if (cutoff_command) {
                fail_cutoff_test("测试失败", event.error_code);
            } else if (event.kind == JcyScpiCommandKind::SetMode &&
                       constant_current_change_pending) {
                requested_constant_current_mode = constant_current_mode;
                constant_current_change_pending = false;
                refresh_constant_current_card();
            }

            if (event.kind == JcyScpiCommandKind::SetMode &&
                device_mode_transition_phase ==
                    DeviceModeTransitionPhase::WaitModeAck &&
                event.requested_value ==
                    static_cast<int>(requested_device_mode)) {
                device_mode_transition_phase =
                    DeviceModeTransitionPhase::Idle;
                confirmed_device_mode_valid = false;
                if (mode_transition_from_ui) {
                    show_info_dialog("模式切换失败",
                                     "底层未确认工作模式，请检查通讯。");
                }
            }

            if (event.kind == JcyScpiCommandKind::QueryMode &&
                device_mode_transition_phase ==
                    DeviceModeTransitionPhase::WaitModeVerify) {
                device_mode_transition_phase =
                    DeviceModeTransitionPhase::Idle;
                confirmed_device_mode_valid = false;
                if (constant_current_change_pending) {
                    requested_constant_current_mode = constant_current_mode;
                    constant_current_change_pending = false;
                    refresh_constant_current_card();
                }
                if (mode_transition_from_ui) {
                    show_info_dialog("模式未确认",
                                     "底层未返回 SETup:MODE?，输出保持关闭。");
                }
                Serial.printf("MODE VERIFY failed expected=%s code=%d\n",
                              scpi_mode_name(requested_device_mode),
                              event.error_code);
            }

            if (event.kind == JcyScpiCommandKind::SetOutput &&
                output_change_pending) {
                const bool failed_state = event.requested_value != 0;
                bool command_still_pending =
                    scpi_link.command_pending(
                        JcyScpiCommandKind::SetOutput);
                if (failed_state != requested_output_state &&
                    !command_still_pending) {
                    command_still_pending =
                        scpi_link.set_output(requested_output_state);
                }

                if (command_still_pending) {
                    output_change_pending = true;
                } else {
                    output_change_pending = false;
                    requested_output_state = output_running;
                    restore_default_poll_interval();
                }
                refresh_power_state();
                refresh_main_waveform_visibility();
                refresh_power_page_output_state();
                if (power_output_label != nullptr &&
                    !output_change_pending) {
                    lv_label_set_text(power_output_label,
                                      output_running
                                          ? "输出已开启"
                                          : "输出已关闭");
                }
                Serial.printf(
                    "OUTPUT ERR failed=%d desired=%d pending=%d code=%d\n",
                    failed_state, requested_output_state,
                    output_change_pending, event.error_code);
            }
            Serial.printf("TTL ERR kind=%u code=%d\n",
                          static_cast<unsigned>(event.kind),
                          event.error_code);
        }
    }
}

void normalize_battery_settings() {
    // 电池类型只提供默认值；用户可以为特殊电芯修改单节范围。
    const int maximum_cell_mv = max(500, min(5000, power_max_voltage_mv()));
    battery_settings.cell_min_mv = constrain(
        battery_settings.cell_min_mv, 500, maximum_cell_mv);
    battery_settings.cell_max_mv = constrain(
        battery_settings.cell_max_mv, 500, maximum_cell_mv);
    if (battery_settings.cell_min_mv > battery_settings.cell_max_mv) {
        battery_settings.cell_min_mv = battery_settings.cell_max_mv;
    }
    const int maximum_cells = battery_max_cells();
    battery_settings.cells = constrain(
        battery_settings.cells, 1, maximum_cells);
    battery_settings.overcurrent_ma = constrain(
        battery_settings.overcurrent_ma, 10, power_max_current_ma());
    battery_settings.protection_delay_ms = constrain(
        battery_settings.protection_delay_ms, 5, 500);
    battery_settings.static_threshold_ma = constrain(
        battery_settings.static_threshold_ma, 1, 1000);
    battery_settings.filter_depth_ms = constrain(
        battery_settings.filter_depth_ms, 100, 2000);
    battery_settings.resistance_mohm = constrain(
        battery_settings.resistance_mohm, 0, 100000);
    set_voltage_mv = constrain(
        set_voltage_mv, pack_minimum_mv(), pack_maximum_mv());
}

void compute_quick_voltages() {
    normalize_battery_settings();
    const int minimum = pack_minimum_mv();
    const int span = pack_maximum_mv() - minimum;
    for (size_t i = 0; i < 6; ++i) {
        quick_voltages_mv[i] = minimum +
            static_cast<int>((static_cast<int64_t>(span) *
                              QUICK_PERCENTAGES[i] + 50) / 100);
    }
}

void format_decimal_value(char *buffer, size_t length, int value,
                          int scale, uint8_t decimals) {
    if (decimals == 0 || scale <= 1) {
        snprintf(buffer, length, "%d", value);
        return;
    }
    const int absolute = abs(value);
    if (decimals == 1) {
        snprintf(buffer, length, "%s%d.%01d", value < 0 ? "-" : "",
                 absolute / scale, absolute % scale);
    } else if (decimals == 2) {
        snprintf(buffer, length, "%s%d.%02d", value < 0 ? "-" : "",
                 absolute / scale, absolute % scale);
    } else {
        snprintf(buffer, length, "%s%d.%03d", value < 0 ? "-" : "",
                 absolute / scale, absolute % scale);
    }
}

void format_field_value(NumericField field, char *buffer, size_t length,
                        bool include_unit = true) {
    int value = 0;
    int scale = 1;
    uint8_t decimals = 0;
    const char *unit = "";

    switch (field) {
        case NumericField::SetVoltage:
            value = set_voltage_mv; scale = 1000; decimals = 3; unit = "V"; break;
        case NumericField::BatteryCells:
            value = battery_settings.cells; unit = "节"; break;
        case NumericField::CellMinVoltage:
            value = battery_settings.cell_min_mv; scale = 1000; decimals = 3; unit = "V"; break;
        case NumericField::CellMaxVoltage:
            value = battery_settings.cell_max_mv; scale = 1000; decimals = 3; unit = "V"; break;
        case NumericField::Overcurrent:
            value = battery_settings.overcurrent_ma; scale = 1000; decimals = 3; unit = "A"; break;
        case NumericField::ProtectionDelay:
            value = battery_settings.protection_delay_ms; unit = "ms"; break;
        case NumericField::StaticThreshold:
            value = battery_settings.static_threshold_ma; unit = "mA"; break;
        case NumericField::FilterDepth:
            value = battery_settings.filter_depth_ms; unit = "ms"; break;
        case NumericField::BatteryResistance:
            value = battery_settings.resistance_mohm; scale = 1000; decimals = 3; unit = "Ω"; break;
        case NumericField::ChargeStart:
            value = charge_settings.start_mv; scale = 1000; decimals = 3; unit = "V"; break;
        case NumericField::ChargeEnd:
            value = charge_settings.end_mv; scale = 1000; decimals = 3; unit = "V"; break;
        case NumericField::ChargeDuration:
            value = charge_settings.duration_s; unit = "s"; break;
        case NumericField::ChargeCutoffCurrent:
            value = charge_settings.cutoff_current_ua; scale = 1000; decimals = 3; unit = "mA"; break;
        case NumericField::DischargeStart:
            value = discharge_settings.start_mv; scale = 1000; decimals = 3; unit = "V"; break;
        case NumericField::DischargeEnd:
            value = discharge_settings.end_mv; scale = 1000; decimals = 3; unit = "V"; break;
        case NumericField::DischargeDuration:
            value = discharge_settings.duration_s; unit = "s"; break;
        case NumericField::DischargeCutoffCurrent:
            value = discharge_settings.cutoff_current_ua; scale = 1000; decimals = 3; unit = "mA"; break;
        case NumericField::SourceVoltage:
            value = power_settings.source_voltage_mv; scale = 1000; decimals = 3; unit = "V"; break;
        case NumericField::SourceCurrent:
            value = power_settings.source_current_ma; scale = 1000; decimals = 3; unit = "A"; break;
        case NumericField::LoadCurrent:
            value = power_settings.load_current_ma; scale = 1000; decimals = 3; unit = "A"; break;
        case NumericField::LoadVoltage:
            value = power_settings.load_voltage_mv; scale = 1000; decimals = 3; unit = "V"; break;
        default:
            snprintf(buffer, length, "--");
            return;
    }

    char number[24] = {};
    format_decimal_value(number, sizeof(number), value, scale, decimals);
    snprintf(buffer, length, "%s%s", number, include_unit ? unit : "");
}

NumericSpec numeric_spec_for(NumericField field) {
    switch (field) {
        case NumericField::SetVoltage:
            return {field, "设置输出电压", &set_voltage_mv,
                    pack_minimum_mv(), pack_maximum_mv(), 1000, 3};
        case NumericField::BatteryCells:
            return {field, "电池节数", &battery_settings.cells, 1,
                    battery_max_cells(), 1, 0};
        case NumericField::CellMinVoltage:
            return {field, "单节最低电压", &battery_settings.cell_min_mv,
                    500, battery_settings.cell_max_mv, 1000, 3};
        case NumericField::CellMaxVoltage:
            return {field, "单节最高电压", &battery_settings.cell_max_mv,
                    battery_settings.cell_min_mv,
                    max(500, min(5000, power_max_voltage_mv())),
                    1000, 3};
        case NumericField::Overcurrent:
            return {field, "过流保护值", &battery_settings.overcurrent_ma,
                    10, power_max_current_ma(), 1000, 3};
        case NumericField::ProtectionDelay:
            return {field, "过流保护延时", &battery_settings.protection_delay_ms,
                    5, 500, 1, 0};
        case NumericField::StaticThreshold:
            return {field, "静态阈值", &battery_settings.static_threshold_ma,
                    1, 1000, 1, 0};
        case NumericField::FilterDepth:
            return {field, "电流滤波深度", &battery_settings.filter_depth_ms,
                    100, 2000, 1, 0};
        case NumericField::BatteryResistance:
            return {field, "电池内阻", &battery_settings.resistance_mohm,
                    0, 100000, 1000, 3};
        case NumericField::ChargeStart:
            return {field, "充电开始电压", &charge_settings.start_mv,
                    0, MAX_BATTERY_PACK_MV, 1000, 3};
        case NumericField::ChargeEnd:
            return {field, "充电终止电压", &charge_settings.end_mv,
                    0, MAX_BATTERY_PACK_MV, 1000, 3};
        case NumericField::ChargeDuration:
            return {field, "充电模拟时长", &charge_settings.duration_s,
                    5, 600, 1, 0};
        case NumericField::ChargeCutoffCurrent:
            return {field, "充电截止电流", &charge_settings.cutoff_current_ua,
                    1, 1000000, 1000, 3};
        case NumericField::DischargeStart:
            return {field, "放电开始电压", &discharge_settings.start_mv,
                    0, MAX_BATTERY_PACK_MV, 1000, 3};
        case NumericField::DischargeEnd:
            return {field, "放电终止电压", &discharge_settings.end_mv,
                    0, MAX_BATTERY_PACK_MV, 1000, 3};
        case NumericField::DischargeDuration:
            return {field, "放电模拟时长", &discharge_settings.duration_s,
                    5, 600, 1, 0};
        case NumericField::DischargeCutoffCurrent:
            return {field, "放电截止电流", &discharge_settings.cutoff_current_ua,
                    1, 1000000, 1000, 3};
        case NumericField::SourceVoltage:
            return {field, "电源输出电压", &power_settings.source_voltage_mv,
                    0, power_max_voltage_mv(), 1000, 3};
        case NumericField::SourceCurrent:
            return {field, "电源限流值", &power_settings.source_current_ma,
                    0, power_max_current_ma(), 1000, 3};
        case NumericField::LoadCurrent:
            return {field, "负载恒流值", &power_settings.load_current_ma,
                    0, power_max_current_ma(), 1000, 3};
        case NumericField::LoadVoltage:
            return {field, "负载恒压值", &power_settings.load_voltage_mv,
                    0, power_max_voltage_mv(), 1000, 3};
        default:
            return {};
    }
}

void refresh_numeric_label(NumericField field) {
    refresh_power_digit_editor(field);
    lv_obj_t *label = numeric_value_labels[numeric_index(field)];
    if (label == nullptr) {
        return;
    }
    char text[32] = {};
    format_field_value(field, text, sizeof(text));
    lv_label_set_text(label, text);
}

void update_power_digit_selection(NumericField field) {
    const int field_index = power_digit_field_index(field);
    if (field_index < 0) {
        return;
    }
    for (size_t digit = 0; digit < 5; ++digit) {
        lv_obj_t *box = power_digit_boxes[field_index][digit];
        lv_obj_t *label = power_digit_labels[field_index][digit];
        if (box == nullptr) {
            continue;
        }
        const bool selected = digit == power_edit_digits[field_index];
        lv_obj_set_style_border_width(box, 1, 0);
        lv_obj_set_style_border_color(box, color(UI_BORDER_DIM), 0);
        lv_obj_set_style_bg_color(
            box, color(selected ? 0x0C4164 : 0x071A31), 0);
        lv_obj_set_style_bg_grad_color(
            box, color(selected ? 0x08243D : 0x030D1C), 0);
        if (label != nullptr) {
            lv_obj_set_style_text_color(
                label, color(selected ? UI_CYAN : 0xD8EEFF), 0);
        }
    }
}

void refresh_power_digit_editor(NumericField field) {
    const int field_index = power_digit_field_index(field);
    if (field_index < 0) {
        return;
    }
    const NumericSpec spec = numeric_spec_for(field);
    if (spec.value == nullptr) {
        return;
    }
    static constexpr int divisors[5] = {10000, 1000, 100, 10, 1};
    const int value = abs(*spec.value);
    for (size_t digit = 0; digit < 5; ++digit) {
        if (power_digit_labels[field_index][digit] != nullptr) {
            lv_label_set_text_fmt(
                power_digit_labels[field_index][digit], "%d",
                (value / divisors[digit]) % 10);
        }
    }
    update_power_digit_selection(field);
}

void copy_ui_area_to_panel_framebuffer(
    uint16_t *framebuffer, const lv_area_t *area,
    const uint16_t *source) {
    const uint32_t source_width = area->x2 - area->x1 + 1;
    const uint32_t source_height = area->y2 - area->y1 + 1;
    if (!jcy_panel::kScaledUi) {
        uint16_t *destination =
            framebuffer + area->y1 * PANEL_WIDTH + area->x1;
        if (source_width == static_cast<uint32_t>(PANEL_WIDTH)) {
            memcpy(destination, source,
                   source_width * source_height * sizeof(uint16_t));
        } else {
            for (uint32_t row = 0; row < source_height; ++row) {
                memcpy(destination + row * PANEL_WIDTH,
                       source + row * source_width,
                       source_width * sizeof(uint16_t));
            }
        }
        return;
    }

    const DisplayDirtyRect panel_rect = panel_rect_for_ui_area(area);
    for (int panel_y = panel_rect.y1; panel_y <= panel_rect.y2; ++panel_y) {
        const int ui_y = ui_y_for_panel_y(panel_y);
        const uint16_t *source_row =
            source + static_cast<size_t>(ui_y - area->y1) * source_width;
        uint16_t *destination =
            framebuffer + static_cast<size_t>(panel_y) * PANEL_WIDTH +
            panel_rect.x1;
        for (int panel_x = panel_rect.x1;
             panel_x <= panel_rect.x2; ++panel_x) {
            const int ui_x = ui_x_for_panel_x(panel_x);
            *destination++ = source_row[ui_x - area->x1];
        }
    }
}

void display_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *pixels) {
    const uint32_t flush_started_us = micros();

    // 等上一次 submitFrame() 对应的扫描完成后，先把上一帧修改过的区域
    // 同步到已释放的 back buffer，再写本帧脏区。完成信号不能在这里先
    // 清空，否则屏幕空闲后首次点击会被迫再等一整帧（约 33ms）。
    if (DISPLAY_VSYNC_GUARD && display_frame_needs_vsync &&
        display_vsync_semaphore != nullptr) {
        if (xSemaphoreTake(
                display_vsync_semaphore,
                pdMS_TO_TICKS(DISPLAY_VSYNC_TIMEOUT_MS)) != pdTRUE) {
            ++debug_vsync_timeout_count;
        }
        sync_pending_dirty_to_backbuffer();
        display_frame_needs_vsync = false;
    }

    const DisplayDirtyRect panel_dirty = panel_rect_for_ui_area(area);
    add_display_dirty_rect(
        display_current_dirty, display_current_dirty_count, panel_dirty);
    uint16_t *framebuffer = gfx.getBackFramebuffer();
    const uint16_t *source =
        reinterpret_cast<const uint16_t *>(&pixels->full);
    if (framebuffer != nullptr) {
        copy_ui_area_to_panel_framebuffer(framebuffer, area, source);
        // RGB DMA 直接读取 PSRAM；写回触及的完整行，保证缓存数据及时可见。
        Cache_WriteBack_Addr(
            reinterpret_cast<uint32_t>(
                framebuffer + panel_dirty.y1 * PANEL_WIDTH),
            PANEL_WIDTH *
                (panel_dirty.y2 - panel_dirty.y1 + 1) * sizeof(uint16_t));
    } else {
        // esp_lcd is configured with two framebuffers.  Reaching this branch
        // means panel initialization failed; do not draw an unscaled 800x480
        // block into the 1024x600 profile and conceal the fault.
        Serial.println("RGB back framebuffer unavailable");
    }
    const uint32_t elapsed_us = micros() - flush_started_us;
    ++debug_flush_calls;
    debug_flush_total_us += elapsed_us;
    debug_flush_max_us = max(debug_flush_max_us, elapsed_us);
    if (elapsed_us >= 10000) {
        ++debug_slow_flush_count;
    }
    if (lv_disp_flush_is_last(disp)) {
        // submit 前清掉更早的周期性完成信号。这样下一次 flush 等到/取得
        // 的就是本次提交之后的完成信号，同时保留空闲时已到达的有效信号，
        // 避免点击后额外等待一个 LCD 帧周期。
        if (DISPLAY_VSYNC_GUARD && display_vsync_semaphore != nullptr) {
            while (xSemaphoreTake(display_vsync_semaphore, 0) == pdTRUE) {
            }
        }
        if (gfx.submitFrame()) {
            display_pending_dirty_count = display_current_dirty_count;
            memcpy(display_pending_dirty, display_current_dirty,
                   display_current_dirty_count * sizeof(DisplayDirtyRect));
            display_frame_needs_vsync =
                DISPLAY_VSYNC_GUARD && display_vsync_semaphore != nullptr;
        } else {
            Serial.println("RGB double-buffer submit failed");
            display_pending_dirty_count = 0;
            display_frame_needs_vsync = false;
        }
        display_current_dirty_count = 0;
        ++debug_completed_frames;
    }
    lv_disp_flush_ready(disp);
}

void refresh_debug_overlay(uint32_t now_ms) {
    if (!DISPLAY_DIAGNOSTICS || debug_perf_label == nullptr ||
        now_ms - debug_last_update_ms < 2000) {
        return;
    }

    const uint32_t frames =
        debug_completed_frames - debug_last_frame_counter;
    debug_last_frame_counter = debug_completed_frames;
    const uint32_t flush_calls = debug_flush_calls;
    const uint32_t average_flush_us = flush_calls == 0
                                          ? 0
                                          : static_cast<uint32_t>(
                                                debug_flush_total_us /
                                                flush_calls);
    const uint32_t maximum_flush_us = debug_flush_max_us;
    debug_flush_calls = 0;
    debug_flush_total_us = 0;
    debug_flush_max_us = 0;
    debug_last_update_ms = now_ms;

    const uint32_t heap_free = ESP.getFreeHeap();
    const uint32_t heap_min = ESP.getMinFreeHeap();
    const uint32_t psram_free = ESP.getFreePsram();
    const uint32_t stack_free =
        uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);

    lv_label_set_text_fmt(
        debug_perf_label,
        JCY1000_FIRMWARE_VERSION " FPS:%lu/%lu\n"
        "FL:%lu/%luus H:%lu/%luK\n"
        "P:%luK ST:%luK VT:%lu",
        static_cast<unsigned long>(frames),
        static_cast<unsigned long>(LCD_REFRESH_HZ),
        static_cast<unsigned long>(average_flush_us),
        static_cast<unsigned long>(maximum_flush_us),
        static_cast<unsigned long>(heap_free / 1024),
        static_cast<unsigned long>(heap_min / 1024),
        static_cast<unsigned long>(psram_free / 1024),
        static_cast<unsigned long>(stack_free / 1024),
        static_cast<unsigned long>(debug_vsync_timeout_count));
}

void touch_read(lv_indev_drv_t *, lv_indev_data_t *data) {
    if (virtual_touch_active) {
        data->point.x = virtual_touch_x;
        data->point.y = virtual_touch_y;
        if (static_cast<int32_t>(millis() - virtual_touch_release_at) < 0) {
            data->state = LV_INDEV_STATE_PR;
            if (!virtual_touch_press_reported) {
                Serial.printf("VTOUCH press raw %d %d\n", virtual_touch_x,
                              virtual_touch_y);
                virtual_touch_press_reported = true;
            }
        } else {
            data->state = LV_INDEV_STATE_REL;
            virtual_touch_active = false;
            Serial.println("VTOUCH release");
        }
        return;
    }

    touch.read();
    if (touch.isTouched) {
        const int panel_x = constrain(
            map(touch.points[0].x, PANEL_WIDTH, 0, 0, PANEL_WIDTH - 1),
            0, PANEL_WIDTH - 1);
        const int panel_y = constrain(
            map(touch.points[0].y, PANEL_HEIGHT, 0, 0, PANEL_HEIGHT - 1),
            0, PANEL_HEIGHT - 1);
        const int raw_x = ui_x_for_panel_x(panel_x);
        const int raw_y = ui_y_for_panel_y(panel_y);

        physical_touch_release_samples = 0;
        if (!physical_touch_pressed) {
            if (++physical_touch_press_samples >=
                TOUCH_PRESS_CONFIRM_SAMPLES) {
                physical_touch_pressed = true;
                physical_touch_press_samples = 0;
                physical_touch_x = raw_x;
                physical_touch_y = raw_y;
            }
        } else {
            physical_touch_press_samples = 0;
            if (abs(raw_x - physical_touch_x) > TOUCH_COORD_DEADBAND) {
                physical_touch_x = raw_x;
            }
            if (abs(raw_y - physical_touch_y) > TOUCH_COORD_DEADBAND) {
                physical_touch_y = raw_y;
            }
        }
    } else {
        physical_touch_press_samples = 0;
        if (physical_touch_pressed) {
            if (++physical_touch_release_samples >=
                TOUCH_RELEASE_CONFIRM_SAMPLES) {
                physical_touch_pressed = false;
                physical_touch_release_samples = 0;
            }
        } else {
            physical_touch_release_samples = 0;
        }
    }

    data->state = physical_touch_pressed ? LV_INDEV_STATE_PR
                                         : LV_INDEV_STATE_REL;
    if (physical_touch_pressed) {
        data->point.x = physical_touch_x;
        data->point.y = physical_touch_y;
    }
}

void queue_virtual_tap(int display_x, int display_y) {
    display_x = constrain(display_x, 0, SCREEN_WIDTH - 1);
    display_y = constrain(display_y, 0, SCREEN_HEIGHT - 1);

    // LVGL will rotate pointer coordinates together with the display driver.
    // Convert visible coordinates back to the panel's native orientation first.
    virtual_touch_x = SCREEN_WIDTH - 1 - display_x;
    virtual_touch_y = SCREEN_HEIGHT - 1 - display_y;
    virtual_touch_release_at = millis() + 140;
    virtual_touch_active = true;
    virtual_touch_press_reported = false;
    Serial.printf("OK tap %d %d\n", display_x, display_y);
}

bool point_inside(lv_obj_t *obj, int x, int y) {
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    return x >= coords.x1 && x <= coords.x2 && y >= coords.y1 && y <= coords.y2;
}

lv_obj_t *find_clickable_at(lv_obj_t *parent, int x, int y) {
    const uint32_t child_count = lv_obj_get_child_cnt(parent);
    for (int32_t i = static_cast<int32_t>(child_count) - 1; i >= 0; --i) {
        lv_obj_t *child = lv_obj_get_child(parent, i);
        if (lv_obj_has_flag(child, LV_OBJ_FLAG_HIDDEN) ||
            !point_inside(child, x, y)) {
            continue;
        }

        lv_obj_t *nested = find_clickable_at(child, x, y);
        if (nested != nullptr) {
            return nested;
        }
        if (lv_obj_has_flag(child, LV_OBJ_FLAG_CLICKABLE)) {
            return child;
        }
    }
    return nullptr;
}

void dispatch_debug_tap(int display_x, int display_y) {
    display_x = constrain(display_x, 0, SCREEN_WIDTH - 1);
    display_y = constrain(display_y, 0, SCREEN_HEIGHT - 1);
    // A serial port open can reset the board and deliver the first command
    // before LVGL's first refresh pass. Force the final object coordinates now.
    lv_obj_t *active_screen = lv_scr_act();
    if (active_screen == nullptr) {
        Serial.println("ERR no active screen");
        return;
    }
    lv_obj_update_layout(active_screen);
    lv_obj_t *target = find_clickable_at(
        active_screen, display_x, display_y);
    if (target == nullptr) {
        Serial.printf("ERR no clickable object at %d %d\n", display_x, display_y);
        return;
    }
    lv_event_send(target, LV_EVENT_CLICKED, nullptr);
    Serial.printf("OK click %d %d\n", display_x, display_y);
}

uint32_t screenshot_crc32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            const uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

void send_screenshot_packet(const uint8_t *data, size_t length) {
    while (length > 0) {
        const size_t written = Serial.write(data, length);
        if (written == 0) {
            delay(1);
            continue;
        }
        data += written;
        length -= written;
    }
}

// PackBits-style RGB565 stream:
//   control bit7=1: repeat one 16-bit pixel (count=low7+1)
//   control bit7=0: literal pixels follow      (count=low7+1)
// The RGB panel framebuffer stores each RGB565 pixel in big-endian byte order.
void send_usb_screenshot() {
    // 截图只能读取当前帧，禁止借截图命令跳过强制开机动画。
    const uint16_t *framebuffer = gfx.getFramebuffer();
    if (framebuffer == nullptr) {
        Serial.println("ERR screenshot framebuffer unavailable");
        return;
    }

    constexpr size_t PIXEL_COUNT = PANEL_WIDTH * PANEL_HEIGHT;
    constexpr size_t RAW_BYTES = PIXEL_COUNT * sizeof(uint16_t);
    const uint8_t *raw = reinterpret_cast<const uint8_t *>(framebuffer);
    const uint32_t crc = screenshot_crc32(raw, RAW_BYTES);

    Serial.flush();
    Serial.printf("JCYSHOT1 %d %d RGB565BE PACKBITS %lu %08lX\n",
                  PANEL_WIDTH, PANEL_HEIGHT,
                  static_cast<unsigned long>(RAW_BYTES),
                  static_cast<unsigned long>(crc));

    size_t pixel = 0;
    uint32_t packet_count = 0;
    while (pixel < PIXEL_COUNT) {
        size_t run = 1;
        while (run < 128 && pixel + run < PIXEL_COUNT &&
               framebuffer[pixel + run] == framebuffer[pixel]) {
            ++run;
        }

        if (run >= 3) {
            const uint8_t control =
                static_cast<uint8_t>(0x80U | static_cast<uint8_t>(run - 1));
            send_screenshot_packet(&control, 1);
            send_screenshot_packet(raw + pixel * sizeof(uint16_t),
                                   sizeof(uint16_t));
            pixel += run;
        } else {
            const size_t literal_start = pixel;
            size_t literal_count = 0;
            while (pixel < PIXEL_COUNT && literal_count < 128) {
                run = 1;
                while (run < 128 && pixel + run < PIXEL_COUNT &&
                       framebuffer[pixel + run] == framebuffer[pixel]) {
                    ++run;
                }
                if (run >= 3) {
                    break;
                }
                ++pixel;
                ++literal_count;
            }

            const uint8_t control =
                static_cast<uint8_t>(literal_count - 1);
            send_screenshot_packet(&control, 1);
            send_screenshot_packet(
                raw + literal_start * sizeof(uint16_t),
                literal_count * sizeof(uint16_t));
        }

        if ((++packet_count & 0x1FU) == 0) {
            delay(1);
        }
    }

    Serial.printf("\nJCYSHOT1 END %08lX\n",
                  static_cast<unsigned long>(crc));
    Serial.flush();
}

bool communication_mode_can_enter_passthrough(bool from_ui) {
    const bool cutoff_active =
        cutoff_running ||
        cutoff_phase == CutoffRunPhase::EnsureBatteryMode ||
        cutoff_phase == CutoffRunPhase::ConfigureRamp ||
        cutoff_phase == CutoffRunPhase::TriggerRamp ||
        cutoff_phase == CutoffRunPhase::Running;
    const char *reason = nullptr;
    if (!TTL_ENABLED) {
        reason = "底层TTL串口未启用，无法进入透传模式。";
    } else if (boot_safe_output_off_active) {
        reason = "正在确认底层输出已关闭，请稍后再试。";
    } else if (output_running || requested_output_state ||
               output_change_pending) {
        reason = "进入透传前必须先关闭输出。";
    } else if (cutoff_active) {
        reason = "截止电压测试运行中，不能切换通讯模式。";
    }

    if (reason == nullptr) {
        return true;
    }
    if (from_ui) {
        show_info_dialog("无法进入透传", reason);
    }
    Serial.printf("ERR passthrough: %s\n", reason);
    return false;
}

bool set_communication_mode(CommunicationMode mode, bool from_ui) {
    if (communication_mode == mode) {
        refresh_communication_mode_page();
        return true;
    }

    if (mode == CommunicationMode::Passthrough) {
        if (!communication_mode_can_enter_passthrough(from_ui)) {
            return false;
        }
        // IO10=TX、IO11=RX 作为外部透传口；IO12/IO13 仍由
        // scpi_link 连接底层硬件。TF 卡接口在透传期间不可使用。
        pinMode(10, OUTPUT);
        digitalWrite(10, HIGH);
        secondary_uart.begin(TTL_BAUD);
        serial_command_length = 0;
        passthrough_escape_length = 0;
        passthrough_escape_started_ms = 0;
        scpi_link.set_passthrough(true);
        // 丢弃切换瞬间可能迟到的最后一帧内部测量响应，确保READY之后
        // USB端看到的只有用户主动发送命令的返回数据。
        delay(80);
        while (scpi_link.passthrough_available() > 0) {
            scpi_link.passthrough_read();
        }
        while (secondary_uart.available() > 0) {
            secondary_uart.read();
        }
        communication_mode = CommunicationMode::Passthrough;
        refresh_communication_mode_page();
        Serial.println(
            "\r\nJCY PASSTHROUGH READY IO10=TX IO11=RX 115200 8N1; "
            "escape=+++JCYEXIT+++");
        Serial.flush();
        return true;
    }

    scpi_link.set_passthrough(false);
    secondary_uart.end();
    communication_mode = CommunicationMode::Controller;
    serial_command_length = 0;
    passthrough_escape_length = 0;
    passthrough_escape_started_ms = 0;

    // 透传期间用户可能手动打开底层输出。返回本机控制时先强制执行
    // 一次OUT OFF确认，再恢复界面控制和自动轮询。
    output_running = false;
    requested_output_state = false;
    output_change_pending = false;
    boot_safe_output_off_active = TTL_ENABLED;
    boot_safe_output_off_attempts = 0;
    boot_safe_output_off_last_request_ms = millis() - 250;
    if (TTL_ENABLED) {
        request_boot_safe_output_off(millis());
    }
    refresh_power_state();
    refresh_main_waveform_visibility();
    refresh_communication_mode_page();
    Serial.println(
        "\r\nJCY PASSTHROUGH CLOSED; OUT OFF verification started");
    return true;
}

void flush_passthrough_escape_buffer() {
    if (passthrough_escape_length == 0) {
        return;
    }
    scpi_link.passthrough_write(
        passthrough_escape_buffer, passthrough_escape_length);
    passthrough_escape_length = 0;
    passthrough_escape_started_ms = 0;
}

void poll_passthrough_bridge(uint32_t now_ms) {
    if (communication_mode != CommunicationMode::Passthrough ||
        !scpi_link.passthrough_active()) {
        return;
    }

    if (passthrough_escape_length > 0 &&
        now_ms - passthrough_escape_started_ms > 500) {
        flush_passthrough_escape_buffer();
    }

    constexpr size_t BRIDGE_BUDGET = 256;
    size_t host_bytes = 0;
    while (secondary_uart.available() > 0 &&
           host_bytes++ < BRIDGE_BUDGET) {
        const int value = secondary_uart.read();
        if (value < 0) {
            break;
        }
        const uint8_t byte = static_cast<uint8_t>(value);
        if (byte == static_cast<uint8_t>(
                        PASSTHROUGH_ESCAPE_SEQUENCE[
                            passthrough_escape_length])) {
            if (passthrough_escape_length == 0) {
                passthrough_escape_started_ms = now_ms;
            }
            passthrough_escape_buffer[passthrough_escape_length++] = byte;
            if (passthrough_escape_length ==
                sizeof(PASSTHROUGH_ESCAPE_SEQUENCE) - 1) {
                passthrough_escape_length = 0;
                passthrough_escape_started_ms = 0;
                set_communication_mode(CommunicationMode::Controller, false);
                return;
            }
            continue;
        }

        flush_passthrough_escape_buffer();
        if (byte == static_cast<uint8_t>(PASSTHROUGH_ESCAPE_SEQUENCE[0])) {
            passthrough_escape_buffer[0] = byte;
            passthrough_escape_length = 1;
            passthrough_escape_started_ms = now_ms;
        } else {
            scpi_link.passthrough_write(&byte, 1);
        }
    }

    size_t device_bytes = 0;
    while (scpi_link.passthrough_available() > 0 &&
           secondary_uart.available_for_write() > 0 &&
           device_bytes++ < BRIDGE_BUDGET) {
        const int value = scpi_link.passthrough_read();
        if (value < 0) {
            break;
        }
        const uint8_t byte = static_cast<uint8_t>(value);
        secondary_uart.write(&byte, 1);
    }
}

void process_serial_command(const char *command) {
    int x = 0;
    int y = 0;
    if (sscanf(command, "tap %d %d", &x, &y) == 2) {
        dispatch_debug_tap(x, y);
        return;
    }
    if (sscanf(command, "touch %d %d", &x, &y) == 2) {
        queue_virtual_tap(x, y);
        return;
    }
    if (strcmp(command, "screenshot") == 0) {
        send_usb_screenshot();
        return;
    }

    if (strcmp(command, "passthrough on") == 0) {
        if (current_page != PageKind::Communication) {
            open_page(PageKind::Communication);
        }
        set_communication_mode(CommunicationMode::Passthrough, false);
        return;
    }
    if (strcmp(command, "passthrough status") == 0) {
        Serial.printf("OK passthrough %s\n",
                      communication_mode == CommunicationMode::Passthrough
                          ? "ON" : "OFF");
        return;
    }

    int voltage_mv = 0;
    if (sscanf(command, "voltage %d", &voltage_mv) == 1) {
        set_voltage_mv = constrain(voltage_mv,
                                   pack_minimum_mv(),
                                   pack_maximum_mv());
        refresh_set_voltage();
        scpi_link.set_voltage_mv(set_voltage_mv);
        Serial.printf("OK voltage %d mV\n", set_voltage_mv);
        return;
    }
    if (strcmp(command, "output on") == 0) {
        apply_output_change(true, true);
        Serial.println("OK output on requested");
        return;
    }
    if (strcmp(command, "output off") == 0) {
        apply_output_change(false, true);
        Serial.println("OK output off requested");
        return;
    }

    if (strcmp(command, "wifi on") == 0) {
        jcy_wireless::wifi_enable();
        Serial.println("OK wifi on");
        return;
    }
    if (strcmp(command, "wifi off") == 0) {
        jcy_wireless::wifi_disable();
        Serial.println("OK wifi off");
        return;
    }
    if (strcmp(command, "wifi scan") == 0) {
        jcy_wireless::wifi_start_scan();
        Serial.println("OK wifi scan started");
        return;
    }
    if (strcmp(command, "wifi forget") == 0) {
        jcy_wireless::wifi_forget_credentials();
        jcy_wireless::wifi_disable();
        Serial.println("OK wifi credentials forgotten");
        return;
    }
    if (strcmp(command, "wifi status") == 0) {
        const WifiStatus &status = jcy_wireless::wifi_status();
        const char *state = "OFF";
        switch (status.state) {
            case JcyLinkState::Off: state = "OFF"; break;
            case JcyLinkState::Idle: state = "IDLE"; break;
            case JcyLinkState::Busy: state = "BUSY"; break;
            case JcyLinkState::Connected: state = "CONNECTED"; break;
            case JcyLinkState::Error: state = "ERROR"; break;
        }
        Serial.printf("OK wifi %s scan=%d rssi=%d ssid=%s ip=%s tx_qdbm=%d sleep=%s\n",
                      state, status.scan_count, status.rssi,
                      status.ssid, status.ip,
                      static_cast<int>(WiFi.getTxPower()),
                      WiFi.getSleep() ? "ON" : "OFF");
        const int16_t count = status.scan_count;
        const int16_t limit = count < 8 ? count : 8;
        for (int16_t i = 0; i < limit; ++i) {
            const char *ssid = jcy_wireless::wifi_ssid_for_index(i);
            if (ssid == nullptr || ssid[0] == '\0') continue;
            Serial.printf("    [%d] %s (%ddBm)\n", i, ssid,
                          jcy_wireless::wifi_rssi_for_index(i));
        }
        return;
    }

    if (strcmp(command, "lower ota test on") == 0) {
        if (!output_is_confirmed_off() || cutoff_running) {
            Serial.println("ERR lower ota test requires confirmed OUT OFF");
            return;
        }
        jcy_lower_mcu_ota::set_same_version_test(true);
        Serial.println("OK lower ota same-version test enabled once");
        return;
    }
    if (strcmp(command, "lower ota test off") == 0) {
        jcy_lower_mcu_ota::set_same_version_test(false);
        Serial.println("OK lower ota same-version test disabled");
        return;
    }
    if (strcmp(command, "lower ota status") == 0) {
        const JcyLowerMcuOtaStatus &status = jcy_lower_mcu_ota::status();
        Serial.printf(
            "OK lower ota state=%u current=%s latest=%s progress=%u "
            "allowed=%u update=%u same_test=%u message=\"%s\"\n",
            static_cast<unsigned>(status.state), status.current_version,
            status.latest_version, static_cast<unsigned>(status.percent),
            status.update_allowed ? 1U : 0U, status.has_update ? 1U : 0U,
            status.same_version_test ? 1U : 0U, status.message);
        return;
    }
    if (strcmp(command, "lower ota check") == 0) {
        if (!jcy_lower_mcu_ota::start_check()) {
            Serial.println("ERR lower ota task busy");
            return;
        }
        Serial.println("OK lower ota check started");
        return;
    }
    if (strcmp(command, "lower ota upgrade") == 0) {
        if (!output_is_confirmed_off() || cutoff_running) {
            Serial.println("ERR lower ota upgrade requires confirmed OUT OFF");
            return;
        }
        if (!jcy_lower_mcu_ota::start_upgrade()) {
            Serial.println("ERR lower ota upgrade not ready");
            return;
        }
        Serial.println("OK lower ota upgrade started");
        return;
    }

    if (strcmp(command, "display diag") == 0) {
        const size_t internal_free =
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        const size_t internal_largest = heap_caps_get_largest_free_block(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        const uint32_t average_flush_us = debug_flush_calls == 0
            ? 0U
            : static_cast<uint32_t>(debug_flush_total_us / debug_flush_calls);
        Serial.printf(
            "OK display panel=%s physical=%dx%d ui=%dx%d pclk=%ld nominal_hz=%lu frames=%lu flush=%lu avg_us=%lu max_us=%lu slow=%lu vsync_timeout=%lu internal=%lu largest=%lu psram=%lu\n",
            jcy_panel::kModel, PANEL_WIDTH, PANEL_HEIGHT,
            SCREEN_WIDTH, SCREEN_HEIGHT,
            static_cast<long>(RGB_PCLK_HZ),
            static_cast<unsigned long>(LCD_REFRESH_HZ),
            static_cast<unsigned long>(debug_completed_frames),
            static_cast<unsigned long>(debug_flush_calls),
            static_cast<unsigned long>(average_flush_us),
            static_cast<unsigned long>(debug_flush_max_us),
            static_cast<unsigned long>(debug_slow_flush_count),
            static_cast<unsigned long>(debug_vsync_timeout_count),
            static_cast<unsigned long>(internal_free),
            static_cast<unsigned long>(internal_largest),
            static_cast<unsigned long>(ESP.getFreePsram()));
        return;
    }
    if (strncmp(command, "wifi connect ", 13) == 0) {
        char ssid[33] = {};
        char password[65] = {};
        const char *arguments = command + 13;
        const char *separator = strrchr(arguments, ' ');
        if (separator != nullptr && separator > arguments) {
            const size_t ssid_length = min(
                static_cast<size_t>(separator - arguments),
                sizeof(ssid) - 1);
            memcpy(ssid, arguments, ssid_length);
            ssid[ssid_length] = '\0';
            snprintf(password, sizeof(password), "%s", separator + 1);
        } else {
            snprintf(ssid, sizeof(ssid), "%s", arguments);
        }
        if (ssid[0] == '\0') {
            Serial.println("ERR wifi connect: missing ssid");
            return;
        }
        jcy_wireless::wifi_save_credentials(ssid, password);
        jcy_wireless::wifi_connect_to_ssid(ssid, password);
        Serial.printf("OK wifi connecting to %s\n", ssid);
        return;
    }

    if (strcmp(command, "ble on") == 0) {
        jcy_wireless::ble_enable();
        Serial.println("OK ble on");
        return;
    }
    if (strcmp(command, "ble off") == 0) {
        jcy_wireless::ble_disable();
        Serial.println("OK ble off");
        return;
    }
    if (strcmp(command, "ble status") == 0) {
        const BleStatus &status = jcy_wireless::ble_status();
        const char *state = "OFF";
        switch (status.state) {
            case JcyLinkState::Off: state = "OFF"; break;
            case JcyLinkState::Idle: state = "IDLE"; break;
            case JcyLinkState::Busy: state = "BUSY"; break;
            case JcyLinkState::Connected: state = "CONNECTED"; break;
            case JcyLinkState::Error: state = "ERROR"; break;
        }
        Serial.printf("OK ble %s name=%s conn=%d tx=%u rx=%u\n",
                      state, status.device_name, status.connections,
                      status.bytes_sent, status.bytes_received);
        return;
    }

#if defined(JCY_HY200C_UI) && JCY_HY200C_UI
    if (strcmp(command, "key left") == 0) {
        hy200_ui::debug_key_left();
        Serial.println("OK key left");
        return;
    }
    if (strcmp(command, "key right") == 0) {
        hy200_ui::debug_key_right();
        Serial.println("OK key right");
        return;
    }
    if (strcmp(command, "key confirm") == 0) {
        hy200_ui::debug_key_confirm();
        Serial.println("OK key confirm");
        return;
    }
#endif

    if (strcmp(command, "help") == 0) {
        Serial.println("Commands: screenshot, tap <x> <y>, touch <x> <y>, "
                       "voltage <mV>, output on|off, "
                       "wifi on/off/scan/status, display diag, ble on/off/status, "
                       "passthrough on/status, ttl diag/reset, "
                       "lower ota test on|off, lower ota status/check/upgrade");
        return;
    }

    int poll_ms = 0;
    if (sscanf(command, "poll %d", &poll_ms) == 1) {
        if (poll_ms < 10) poll_ms = 10;
        if (poll_ms > 5000) poll_ms = 5000;
        scpi_link.set_poll_interval(static_cast<uint32_t>(poll_ms));
        Serial.printf("OK poll_interval=%d ms\n", poll_ms);
        return;
    }

    if (strcmp(command, "ttl diag reset") == 0) {
        scpi_link.reset_diagnostics();
        Serial.println("OK ttl diag reset");
        return;
    }

    if (strcmp(command, "ttl diag") == 0) {
        const uint32_t now = millis();
        const JcyScpiDiagnostics diag = scpi_link.diagnostics();
        const uint32_t byte_age = diag.last_byte_ms == 0
                                      ? 0xFFFFFFFFu
                                      : now - diag.last_byte_ms;
        const uint32_t line_age = diag.last_line_ms == 0
                                      ? 0xFFFFFFFFu
                                      : now - diag.last_line_ms;
        const uint32_t accepted_age = diag.last_accepted_ms == 0
                                          ? 0xFFFFFFFFu
                                          : now - diag.last_accepted_ms;
        Serial.printf(
            "OK ttl diag rxpin=%d level=%d buffered=%d raw=%lu lines=%lu "
            "accepted=%lu parse_fail=%lu unsolicited=%lu timeout=%lu "
            "overflow=%lu byte_age=%lu line_age=%lu accepted_age=%lu "
            "active=%u kind=%u queued=%u last=\"%s\"\n",
            TTL_RX_PIN, digitalRead(TTL_RX_PIN),
            scpi_link.pending_rx_bytes(),
            static_cast<unsigned long>(diag.raw_rx_bytes),
            static_cast<unsigned long>(diag.complete_lines),
            static_cast<unsigned long>(diag.accepted_lines),
            static_cast<unsigned long>(diag.parse_failures),
            static_cast<unsigned long>(diag.unsolicited_lines),
            static_cast<unsigned long>(diag.timeouts),
            static_cast<unsigned long>(diag.line_overflows),
            static_cast<unsigned long>(byte_age),
            static_cast<unsigned long>(line_age),
            static_cast<unsigned long>(accepted_age),
            diag.command_active ? 1u : 0u,
            static_cast<unsigned>(diag.active_kind),
            static_cast<unsigned>(diag.queued_commands),
            diag.last_line);
        return;
    }

    Serial.printf("ERR command: %s\n", command);
}

void poll_serial_commands() {
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            if (serial_command_length > 0) {
                serial_command[serial_command_length] = '\0';
                process_serial_command(serial_command);
                serial_command_length = 0;
            }
            continue;
        }

        if (serial_command_length < sizeof(serial_command) - 1) {
            serial_command[serial_command_length++] = ch;
        } else {
            serial_command_length = 0;
            Serial.println("ERR command too long");
        }
    }
}

void style_clean(lv_obj_t *obj) {
    lv_obj_set_style_pad_all(obj, 0, 0);
    lv_obj_set_style_shadow_width(obj, 0, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

void button_sound_event(lv_event_t *event) {
    const auto cue = static_cast<jcy_audio::Cue>(
        reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    jcy_audio::play(cue);
}

void attach_button_sound(lv_obj_t *button,
                         jcy_audio::Cue cue = jcy_audio::Cue::Click) {
    if (button == nullptr || cue == jcy_audio::Cue::None) {
        return;
    }
    lv_obj_add_event_cb(
        button, button_sound_event, LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(cue)));
}

lv_obj_t *make_gradient_box(lv_obj_t *parent, int x, int y, int width, int height,
                            uint32_t top_color, uint32_t bottom_color,
                            int radius = 0, uint32_t border_color = 0x000000,
                            int border_width = 0) {
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_set_pos(box, x, y);
    lv_obj_set_size(box, width, height);
    style_clean(box);
    lv_obj_set_style_radius(box, radius, 0);
    lv_obj_set_style_border_width(box, border_width, 0);
    lv_obj_set_style_border_color(box, color(border_color), 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(box, color(top_color), 0);
    lv_obj_set_style_bg_grad_color(box, color(bottom_color), 0);
    lv_obj_set_style_bg_grad_dir(box, LV_GRAD_DIR_VER, 0);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_CLICKABLE);
    return box;
}

lv_obj_t *make_label(lv_obj_t *parent, const char *text, int x, int y,
                     const lv_font_t *font, uint32_t text_color) {
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color(text_color), 0);
    return label;
}

lv_obj_t *make_center_label(lv_obj_t *parent, const char *text,
                            const lv_font_t *font, uint32_t text_color) {
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color(text_color), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(label);
    return label;
}

lv_obj_t *make_gradient_button(lv_obj_t *parent, int x, int y, int width, int height,
                               uint32_t top_color, uint32_t bottom_color,
                               int radius = 4, uint32_t border_color = 0x2777C8,
                               int border_width = 1,
                               jcy_audio::Cue sound = jcy_audio::Cue::Click) {
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, width, height);
    style_clean(button);
    lv_obj_set_style_radius(button, radius, 0);
    lv_obj_set_style_border_width(button, border_width, 0);
    lv_obj_set_style_border_color(button, color(border_color), 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(button, color(top_color), 0);
    lv_obj_set_style_bg_grad_color(button, color(bottom_color), 0);
    lv_obj_set_style_bg_grad_dir(button, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_color(button, color(0x1268C6), LV_STATE_PRESSED);
    lv_obj_set_style_bg_grad_color(button, color(0x174DA4), LV_STATE_PRESSED);
    attach_button_sound(button, sound);
    return button;
}

void reset_page_references() {
    memset(numeric_value_labels, 0, sizeof(numeric_value_labels));
    force_amp_button = nullptr;
    force_amp_label = nullptr;
    popup_config_button = nullptr;
    popup_config_label = nullptr;
    memset(battery_chemistry_buttons, 0,
           sizeof(battery_chemistry_buttons));
    memset(battery_chemistry_labels, 0,
           sizeof(battery_chemistry_labels));
    battery_cell_range_label = nullptr;
    battery_pack_range_label = nullptr;
    battery_cells_value_label = nullptr;
    cutoff_start_button = nullptr;
    cutoff_start_label = nullptr;
    cutoff_status_label = nullptr;
    cutoff_realtime_voltage_label = nullptr;
    cutoff_realtime_current_label = nullptr;
    cutoff_result_label = nullptr;
    cutoff_elapsed_label = nullptr;
    cutoff_charge_button = nullptr;
    cutoff_discharge_button = nullptr;
    cutoff_chart = nullptr;
    cutoff_voltage_series = nullptr;
    cutoff_current_series = nullptr;
    power_content = nullptr;
    memset(power_tab_buttons, 0, sizeof(power_tab_buttons));
    power_output_label = nullptr;
    power_page_output_button = nullptr;
    power_page_output_icon = nullptr;
    power_page_output_stem = nullptr;
    cycle_button_label = nullptr;
    power_live_voltage_label = nullptr;
    power_live_current_label = nullptr;
    power_live_power_label = nullptr;
    power_live_temperature_label = nullptr;
    power_rating_label = nullptr;
    power_page_status_bar = nullptr;
    power_page_status_icon = nullptr;
    power_page_status_label = nullptr;
    memset(power_digit_boxes, 0, sizeof(power_digit_boxes));
    memset(power_digit_labels, 0, sizeof(power_digit_labels));
    wifi_status_label = nullptr;
    wifi_info_label = nullptr;
    wifi_ssid_textarea = nullptr;
    wifi_password_textarea = nullptr;
    wifi_scan_dropdown = nullptr;
    wifi_keyboard = nullptr;
    wifi_keyboard_target = nullptr;
    wifi_connect_button_label = nullptr;
    ble_status_label = nullptr;
    ble_info_label = nullptr;
    ble_toggle_button_label = nullptr;
    ota_ip_label = nullptr;
    ota_status_label = nullptr;
    ota_info_label = nullptr;
    ota_progress_bar = nullptr;
    cloud_ota_remote_label = nullptr;
    cloud_ota_version_label = nullptr;
    cloud_ota_notes_label = nullptr;
    cloud_ota_progress_bar = nullptr;
    cloud_ota_check_button = nullptr;
    cloud_ota_download_button = nullptr;
    lower_ota_status_label = nullptr;
    lower_ota_version_label = nullptr;
    lower_ota_notes_label = nullptr;
    lower_ota_progress_bar = nullptr;
    lower_ota_check_button = nullptr;
    lower_ota_upgrade_button = nullptr;
    communication_controller_button = nullptr;
    communication_passthrough_button = nullptr;
    communication_controller_label = nullptr;
    communication_passthrough_label = nullptr;
    communication_status_label = nullptr;
    communication_detail_label = nullptr;
    wifi_scan_option_count = 0;
    wifi_scan_options_loaded = false;
    wifi_keyboard_layout = WifiKeyboardLayout::Lowercase;
}

void close_keypad() {
    if (keypad_overlay != nullptr) {
        lv_obj_del(keypad_overlay);
        keypad_overlay = nullptr;
    }
    keypad_value_label = nullptr;
    keypad_error_label = nullptr;
    numeric_input_replace_on_next_key = false;
    active_numeric_spec = {};
}

void close_info_dialog() {
    if (info_overlay != nullptr) {
        lv_obj_del(info_overlay);
        info_overlay = nullptr;
    }
    measurement_popup_voltage_label = nullptr;
    measurement_popup_current_label = nullptr;
    measurement_popup_power_label = nullptr;
}

void close_keypad_event(lv_event_t *) {
    close_keypad();
}

void close_info_event(lv_event_t *) {
    close_info_dialog();
}

void close_page_event(lv_event_t *) {
    close_page();
}

lv_obj_t *create_page_shell(const char *title) {
    close_info_dialog();
    close_keypad();
    if (page_overlay != nullptr) {
        lv_obj_del(page_overlay);
        page_overlay = nullptr;
    }
    reset_page_references();

    page_overlay = lv_obj_create(ui_screen);
    lv_obj_set_pos(page_overlay, 0, 0);
    lv_obj_set_size(page_overlay, SCREEN_WIDTH, SCREEN_HEIGHT);
    style_clean(page_overlay);
    lv_obj_set_style_radius(page_overlay, 0, 0);
    lv_obj_set_style_border_width(page_overlay, 0, 0);
    lv_obj_set_style_bg_color(page_overlay, color(UI_BG_TOP), 0);
    lv_obj_set_style_bg_grad_color(page_overlay, color(UI_BG_BOTTOM), 0);
    lv_obj_set_style_bg_grad_dir(page_overlay, LV_GRAD_DIR_VER, 0);

    lv_obj_t *header = make_gradient_box(
        page_overlay, 0, 0, SCREEN_WIDTH, 62,
        0x0A2440, 0x030D1C, 0, UI_BORDER_DIM, 0);
    // Logo is 232 px wide at x=15, so page titles start at x=270.
    // This keeps a 23 px gap and reserves x=510 onward for the version.
    lv_obj_t *logo = lv_img_create(header);
    lv_img_set_src(logo, &lv_img_jcy_logo);
    lv_obj_set_pos(logo, 15, 3);
    lv_obj_set_style_img_recolor(logo, color(0xFFFFFF), 0);
    lv_obj_set_style_img_recolor_opa(logo, LV_OPA_COVER, 0);
    lv_obj_t *title_label = make_label(
        header, title, 270, 20, &lv_font_jcy20, 0xFFFFFF);
    lv_obj_set_width(title_label, 220);
    lv_label_set_long_mode(title_label, LV_LABEL_LONG_DOT);
    make_label(header, JCY1000_FIRMWARE_VERSION,
               510, 22, &lv_font_montserrat_14, UI_MUTED);

    lv_obj_t *close_button = make_gradient_button(
        header, 742, 9, 48, 44, 0x5A1830, 0x260B1B,
        LV_RADIUS_CIRCLE, 0xEF4A67, 1, jcy_audio::Cue::Back);
    lv_obj_add_event_cb(close_button, close_page_event,
                        LV_EVENT_CLICKED, nullptr);
    make_center_label(close_button, LV_SYMBOL_CLOSE,
                      &lv_font_montserrat_20, 0xFFFFFF);
    return page_overlay;
}

void show_info_dialog(const char *title, const char *body) {
    close_info_dialog();
    info_overlay = lv_obj_create(ui_screen);
    lv_obj_set_pos(info_overlay, 0, 0);
    lv_obj_set_size(info_overlay, SCREEN_WIDTH, SCREEN_HEIGHT);
    style_clean(info_overlay);
    lv_obj_set_style_bg_color(info_overlay, color(0x06162A), 0);
    lv_obj_set_style_bg_opa(info_overlay, LV_OPA_70, 0);
    lv_obj_set_style_border_width(info_overlay, 0, 0);
    lv_obj_add_event_cb(info_overlay, close_info_event,
                        LV_EVENT_CLICKED, nullptr);

    lv_obj_t *dialog = make_gradient_box(
        info_overlay, 120, 78, 560, 325,
        0x092641, 0x041224, 12, UI_CYAN, 2);
    lv_obj_add_flag(dialog, LV_OBJ_FLAG_CLICKABLE);
    make_label(dialog, title, 28, 20, &lv_font_jcy20, 0xFFFFFF);

    lv_obj_t *body_label = make_label(
        dialog, body, 28, 67, &lv_font_jcy18, 0xF1FAFF);
    lv_obj_set_width(body_label, 504);
    lv_label_set_long_mode(body_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_line_space(body_label, 9, 0);

    lv_obj_t *ok = make_gradient_button(
        dialog, 205, 260, 150, 48,
        0x0D806F, 0x063C39, 6, 0x50FFD2, 1,
        jcy_audio::Cue::Confirm);
    lv_obj_add_event_cb(ok, close_info_event, LV_EVENT_CLICKED, nullptr);
    make_center_label(ok, "确定", &lv_font_jcy18, 0xFFFFFF);
}

void refresh_keypad_display() {
    if (keypad_value_label != nullptr) {
        lv_label_set_text(keypad_value_label,
                          numeric_input[0] == '\0' ? "0" : numeric_input);
    }
}

void keypad_key_event(lv_event_t *event) {
    const char *key = static_cast<const char *>(lv_event_get_user_data(event));
    if (key == nullptr) {
        return;
    }

    // 弹窗打开时保留当前值供用户确认；第一次按数字、点或删除键时，
    // 自动清空旧值，避免把新输入追加到原来的设定值后面。
    if (numeric_input_replace_on_next_key) {
        numeric_input[0] = '\0';
        numeric_input_replace_on_next_key = false;
    }

    if (strcmp(key, "DEL") == 0) {
        const size_t length = strlen(numeric_input);
        if (length > 0) {
            numeric_input[length - 1] = '\0';
        }
    } else if (strcmp(key, ".") == 0) {
        if (strchr(numeric_input, '.') == nullptr &&
            active_numeric_spec.decimals > 0) {
            const size_t length = strlen(numeric_input);
            if (length < sizeof(numeric_input) - 1) {
                if (length == 0) {
                    strlcpy(numeric_input, "0.", sizeof(numeric_input));
                } else {
                    numeric_input[length] = '.';
                    numeric_input[length + 1] = '\0';
                }
            }
        }
    } else {
        const size_t length = strlen(numeric_input);
        if (length < sizeof(numeric_input) - 1) {
            if (length == 1 && numeric_input[0] == '0' &&
                active_numeric_spec.decimals == 0) {
                numeric_input[0] = key[0];
            } else {
                numeric_input[length] = key[0];
                numeric_input[length + 1] = '\0';
            }
        }
    }
    if (keypad_error_label != nullptr) {
        lv_label_set_text(keypad_error_label, "");
    }
    refresh_keypad_display();
}

void normalize_after_numeric_input(NumericField field) {
    switch (field) {
        case NumericField::BatteryCells:
        case NumericField::CellMinVoltage:
        case NumericField::CellMaxVoltage:
        case NumericField::Overcurrent:
        case NumericField::ProtectionDelay:
        case NumericField::StaticThreshold:
        case NumericField::FilterDepth:
        case NumericField::BatteryResistance:
            normalize_battery_settings();
            break;
        case NumericField::ChargeStart:
            if (charge_settings.start_mv > charge_settings.end_mv) {
                charge_settings.end_mv = charge_settings.start_mv;
            }
            break;
        case NumericField::ChargeEnd:
            if (charge_settings.end_mv < charge_settings.start_mv) {
                charge_settings.start_mv = charge_settings.end_mv;
            }
            break;
        case NumericField::DischargeStart:
            if (discharge_settings.start_mv < discharge_settings.end_mv) {
                discharge_settings.end_mv = discharge_settings.start_mv;
            }
            break;
        case NumericField::DischargeEnd:
            if (discharge_settings.end_mv > discharge_settings.start_mv) {
                discharge_settings.start_mv = discharge_settings.end_mv;
            }
            break;
        default:
            break;
    }
}

void refresh_current_page_values() {
    for (size_t i = 1; i < numeric_index(NumericField::Count); ++i) {
        refresh_numeric_label(static_cast<NumericField>(i));
    }
    refresh_main_values();
    if (current_page == PageKind::Power) {
        rebuild_power_content();
    } else if (current_page == PageKind::BatterySettings) {
        refresh_battery_settings_page();
    }
}

void confirm_keypad_event(lv_event_t *) {
    if (active_numeric_spec.value == nullptr || numeric_input[0] == '\0') {
        return;
    }
    const double entered = strtod(numeric_input, nullptr);
    const int converted = static_cast<int>(
        lround(entered * active_numeric_spec.scale));
    if (converted < active_numeric_spec.minimum ||
        converted > active_numeric_spec.maximum) {
        if (keypad_error_label != nullptr) {
            lv_label_set_text(keypad_error_label, "输入值超出允许范围");
        }
        return;
    }

    *active_numeric_spec.value = converted;
    const NumericField field = active_numeric_spec.field;
    normalize_after_numeric_input(field);
    Serial.printf("EVENT numeric field=%u value=%d UI_ONLY\n",
                  static_cast<unsigned>(field), converted);
    close_keypad();
    refresh_current_page_values();
    send_numeric_field_to_device(field);
}

void open_numeric_field(NumericField field) {
    active_numeric_spec = numeric_spec_for(field);
    if (active_numeric_spec.value == nullptr) {
        return;
    }
    close_keypad();
    active_numeric_spec = numeric_spec_for(field);
    format_field_value(field, numeric_input, sizeof(numeric_input), false);
    numeric_input_replace_on_next_key = true;

    keypad_overlay = lv_obj_create(ui_screen);
    lv_obj_set_pos(keypad_overlay, 0, 0);
    lv_obj_set_size(keypad_overlay, SCREEN_WIDTH, SCREEN_HEIGHT);
    style_clean(keypad_overlay);
    lv_obj_set_style_bg_color(keypad_overlay, color(0x06162A), 0);
    lv_obj_set_style_bg_opa(keypad_overlay, LV_OPA_70, 0);
    lv_obj_set_style_border_width(keypad_overlay, 0, 0);

    lv_obj_t *dialog = make_gradient_box(
        keypad_overlay, 170, 18, 460, 444,
        0x092641, 0x041224, 12, UI_CYAN, 2);
    make_label(dialog, active_numeric_spec.title, 22, 14,
               &lv_font_jcy20, 0xFFFFFF);

    lv_obj_t *display = make_gradient_box(
        dialog, 20, 52, 420, 53,
        0x071A31, 0x030D1B, 7, UI_BORDER, 1);
    keypad_value_label = make_center_label(
        display, numeric_input, &lv_font_montserrat_30, 0xFFFFFF);

    char min_text[24] = {};
    char max_text[24] = {};
    format_decimal_value(min_text, sizeof(min_text), active_numeric_spec.minimum,
                         active_numeric_spec.scale,
                         active_numeric_spec.decimals);
    format_decimal_value(max_text, sizeof(max_text), active_numeric_spec.maximum,
                         active_numeric_spec.scale,
                         active_numeric_spec.decimals);
    char range_text[64] = {};
    snprintf(range_text, sizeof(range_text), "范围 %s - %s", min_text, max_text);
    make_label(dialog, range_text, 22, 109, &lv_font_jcy16, 0xD9F4FF);

    static const char *KEYS[] = {
        "1", "2", "3", "4", "5", "6",
        "7", "8", "9", ".", "0", "DEL",
    };
    for (size_t i = 0; i < 12; ++i) {
        const int col = static_cast<int>(i % 3);
        const int row = static_cast<int>(i / 3);
        lv_obj_t *key = make_gradient_button(
            dialog, 20 + col * 92, 137 + row * 61,
            84, 53, 0x0B4161, 0x051A2D, 6, UI_CYAN, 1);
        lv_obj_add_event_cb(key, keypad_key_event, LV_EVENT_CLICKED,
                            const_cast<char *>(KEYS[i]));
        make_center_label(key, KEYS[i],
                          strcmp(KEYS[i], "DEL") == 0
                              ? &lv_font_montserrat_16
                              : &lv_font_montserrat_20,
                          0xFFFFFF);
    }

    lv_obj_t *cancel = make_gradient_button(
        dialog, 306, 137, 134, 83,
        0x25384B, 0x111D2A, 6, 0x59748A, 1,
        jcy_audio::Cue::Back);
    lv_obj_add_event_cb(cancel, close_keypad_event, LV_EVENT_CLICKED, nullptr);
    make_center_label(cancel, "取消", &lv_font_jcy18, 0xFFFFFF);

    lv_obj_t *confirm = make_gradient_button(
        dialog, 306, 231, 134, 150,
        0x0D806F, 0x063C39, 6, 0x50FFD2, 1,
        jcy_audio::Cue::Confirm);
    lv_obj_add_event_cb(confirm, confirm_keypad_event,
                        LV_EVENT_CLICKED, nullptr);
    make_center_label(confirm, "确定", &lv_font_jcy20, 0xFFFFFF);

    keypad_error_label = make_label(
        dialog, "", 20, 389, &lv_font_jcy16, 0xFFF067);
}

void numeric_field_event(lv_event_t *event) {
    const auto raw = reinterpret_cast<intptr_t>(lv_event_get_user_data(event));
    open_numeric_field(static_cast<NumericField>(raw));
}

lv_obj_t *create_numeric_row(lv_obj_t *parent, int x, int y, int width,
                             const char *name, NumericField field) {
    lv_obj_t *row = make_gradient_box(
        parent, x, y, width, 57,
        0x0B2D4B, 0x051426, 6, UI_BORDER_DIM, 1);
    make_label(row, name, 14, 17, &lv_font_jcy16, 0xEAF8FF);

    lv_obj_t *value_button = make_gradient_button(
        row, width - 145, 7, 135, 43,
        0x092B52, 0x041327, 5, UI_CYAN, 1);
    lv_obj_add_event_cb(
        value_button, numeric_field_event, LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(field)));
    char value[32] = {};
    format_field_value(field, value, sizeof(value));
    numeric_value_labels[numeric_index(field)] = make_center_label(
        value_button, value, &lv_font_jcy16, 0xFFFFFF);
    return row;
}

void make_battery_icon(lv_obj_t *parent) {
    make_gradient_box(parent, 41, 24, 18, 6,
                      0x8EFAFF, 0x27BCE8, 2, 0xB6FAFF, 0);
    lv_obj_t *shell = make_gradient_box(
        parent, 24, 29, 52, 86, 0x0A4067, 0x041528,
        8, UI_CYAN, 3);
    constexpr int CELL_COUNT = 5;
    for (int i = 0; i < CELL_COUNT; ++i) {
        lv_obj_t *cell = make_gradient_box(
            shell, 6, 65 - i * 14, 36, 10,
            0x0B3150, 0x061A30, 3, UI_BORDER_DIM, 1);
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_CLICKABLE);
        mode_battery_cells[i] = cell;
    }
}

void make_power_supply_icon(lv_obj_t *parent) {
    // 充电插头: 矩形插头主体 + V 字插孔 + 圆头插脚 + 下垂软电缆
    lv_obj_t *body = make_gradient_box(
        parent, 18, 18, 64, 64, 0x0C3F61, 0x04172A,
        12, UI_CYAN, 3);
    // 插头底部的小凸起 (与线缆衔接)
    lv_obj_t *neck = make_gradient_box(
        parent, 37, 78, 26, 9, 0x0B3552, 0x06192D,
        4, UI_BORDER, 1);
    lv_obj_clear_flag(neck, LV_OBJ_FLAG_CLICKABLE);
    // 插头上的 V 字标识 (代替电源符号)
    make_label(body, "V", 22, 8,
               &lv_font_montserrat_24, UI_CYAN);
    // 圆头插脚 (左侧 = L / 火线)
    lv_obj_t *pin_left = make_gradient_box(
        body, 10, 39, 12, 12, 0xB6F4FF, 0x69CDE9,
        LV_RADIUS_CIRCLE, 0xFFFFFF, 1);
    lv_obj_clear_flag(pin_left, LV_OBJ_FLAG_CLICKABLE);
    // 圆头插脚 (右侧 = N / 零线)
    lv_obj_t *pin_right = make_gradient_box(
        body, 42, 39, 12, 12, 0xB6F4FF, 0x69CDE9,
        LV_RADIUS_CIRCLE, 0xFFFFFF, 1);
    lv_obj_clear_flag(pin_right, LV_OBJ_FLAG_CLICKABLE);
    // 下垂电缆: 直线段 + 弧形段 (用矩形 + 圆头模拟弯曲)
    lv_obj_t *cable = make_gradient_box(
        parent, 44, 87, 12, 27, 0x0A3552, 0x041629,
        0, UI_BORDER, 0);
    lv_obj_clear_flag(cable, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t *cable_curve = make_gradient_box(
        parent, 55, 108, 26, 11, 0x0A3552, 0x041629,
        LV_RADIUS_CIRCLE, UI_BORDER, 0);
    lv_obj_clear_flag(cable_curve, LV_OBJ_FLAG_CLICKABLE);
}

lv_obj_t *make_measurement_badge(lv_obj_t *parent, const char *text,
                                  int x, int y, uint32_t top,
                                  uint32_t bottom) {
    lv_obj_t *badge = make_gradient_box(
        parent, x, y, 28, 28, top, bottom,
        LV_RADIUS_CIRCLE, 0xB7F7FF, 1);
    make_center_label(badge, text, &lv_font_montserrat_14, 0xFFFFFF);
    return badge;
}

lv_obj_t *header_wifi_button = nullptr;
lv_obj_t *header_bt_button = nullptr;
lv_obj_t *header_wifi_icon = nullptr;
lv_obj_t *header_wifi_bars[3] = {};
lv_obj_t *header_bt_status_dot = nullptr;
lv_obj_t *header_wifi_status_badge = nullptr;
lv_obj_t *header_wifi_status_label = nullptr;
lv_obj_t *header_bt_status_badge = nullptr;
lv_obj_t *header_bt_status_label = nullptr;
lv_obj_t *settings_hub_bt_button = nullptr;

void refresh_toggle_button(lv_obj_t *button, lv_obj_t *label, bool enabled) {
    if (button == nullptr || label == nullptr) {
        return;
    }
    lv_obj_set_style_bg_color(
        button, color(enabled ? 0x0D806F : 0x09243E), 0);
    lv_obj_set_style_bg_grad_color(
        button, color(enabled ? 0x063C39 : 0x041326), 0);
    lv_obj_set_style_border_color(
        button, color(enabled ? 0x50FFD2 : UI_BORDER_DIM), 0);
    lv_label_set_text(label, enabled ? "开启" : "关闭");
}

void refresh_battery_chemistry_label() {
    for (size_t i = 0; i < 4; ++i) {
        lv_obj_t *button = battery_chemistry_buttons[i];
        lv_obj_t *label = battery_chemistry_labels[i];
        if (button == nullptr || label == nullptr) {
            continue;
        }
        const bool selected =
            battery_settings.chemistry == BATTERY_CHEMISTRY_OPTIONS[i];
        lv_obj_set_style_bg_color(
            button, color(selected ? 0x0D806F : 0x09243E), 0);
        lv_obj_set_style_bg_grad_color(
            button, color(selected ? 0x063C39 : 0x041326), 0);
        lv_obj_set_style_border_color(
            button, color(selected ? 0x50FFD2 : UI_BORDER_DIM), 0);
        lv_obj_set_style_border_width(button, selected ? 2 : 1, 0);
        lv_obj_set_style_text_color(
            label, color(selected ? 0xFFFFFF : 0xA6C3D7), 0);
    }
}

void refresh_battery_range_labels() {
    const int maximum_cells = battery_max_cells();
    if (battery_cell_range_label != nullptr) {
        lv_label_set_text_fmt(
            battery_cell_range_label, "%d.%03dV - %d.%03dV",
            battery_settings.cell_min_mv / 1000,
            abs(battery_settings.cell_min_mv % 1000),
            battery_settings.cell_max_mv / 1000,
            abs(battery_settings.cell_max_mv % 1000));
    }
    if (battery_pack_range_label != nullptr) {
        lv_label_set_text_fmt(
            battery_pack_range_label, "%d串总范围  %d.%03dV - %d.%03dV",
            battery_settings.cells,
            pack_minimum_mv() / 1000,
            abs(pack_minimum_mv() % 1000),
            pack_maximum_mv() / 1000,
            abs(pack_maximum_mv() % 1000));
    }
    if (battery_cells_value_label != nullptr) {
        lv_label_set_text_fmt(battery_cells_value_label, "%d / %d串",
                              battery_settings.cells, maximum_cells);
    }
}

void refresh_battery_settings_page() {
    normalize_battery_settings();
    refresh_toggle_button(force_amp_button, force_amp_label,
                          battery_settings.force_amp_range);
    refresh_toggle_button(popup_config_button, popup_config_label,
                          battery_settings.popup_configuration);
    refresh_battery_chemistry_label();
    refresh_battery_range_labels();
    const NumericField fields[] = {
        NumericField::CellMinVoltage,
        NumericField::CellMaxVoltage,
        NumericField::Overcurrent,
        NumericField::ProtectionDelay,
        NumericField::StaticThreshold,
        NumericField::FilterDepth,
        NumericField::BatteryResistance,
    };
    for (NumericField field : fields) {
        refresh_numeric_label(field);
    }
}

void toggle_force_amp_event(lv_event_t *) {
    battery_settings.force_amp_range = !battery_settings.force_amp_range;
    refresh_battery_settings_page();
    Serial.printf("EVENT battery force_amp=%d UI_ONLY\n",
                  battery_settings.force_amp_range);
}

void toggle_popup_config_event(lv_event_t *) {
    battery_settings.popup_configuration =
        !battery_settings.popup_configuration;
    refresh_battery_settings_page();
    Serial.printf("EVENT battery popup_config=%d UI_ONLY\n",
                  battery_settings.popup_configuration);
}

void select_battery_chemistry_event(lv_event_t *event) {
    const auto *chemistry = static_cast<const BatteryChemistry *>(
        lv_event_get_user_data(event));
    if (chemistry == nullptr ||
        *chemistry >= BatteryChemistry::Count) {
        return;
    }
    battery_settings.chemistry = *chemistry;
    battery_settings.cell_min_mv = battery_chemistry_min_mv(*chemistry);
    battery_settings.cell_max_mv = battery_chemistry_max_mv(*chemistry);
    normalize_battery_settings();
    refresh_battery_settings_page();
    refresh_main_values();
    update_battery_soc_display(scpi_link.measurements().voltage_mv);
    Serial.printf("EVENT battery chemistry=%d cells_max=%d UI_ONLY\n",
                  static_cast<int>(*chemistry), battery_max_cells());
}

void battery_cells_step_event(lv_event_t *event) {
    const auto *direction = static_cast<const int *>(
        lv_event_get_user_data(event));
    if (direction == nullptr || (*direction != -1 && *direction != 1)) {
        return;
    }
    battery_settings.cells = constrain(
        battery_settings.cells + *direction, 1, battery_max_cells());
    normalize_battery_settings();
    refresh_battery_settings_page();
    refresh_main_values();
    update_battery_soc_display(scpi_link.measurements().voltage_mv);
    Serial.printf("EVENT battery cells=%d max=%d UI_ONLY\n",
                  battery_settings.cells, battery_max_cells());
}

void save_battery_settings_event(lv_event_t *) {
    normalize_battery_settings();
    Serial.printf(
        "EVENT battery_settings save chemistry=%d cells=%d "
        "range=%d-%d ocp=%d TTL\n",
        static_cast<int>(battery_settings.chemistry),
        battery_settings.cells, battery_settings.cell_min_mv,
        battery_settings.cell_max_mv, battery_settings.overcurrent_ma);
    // BATTERY模式下SETup:CURRent即过流保护值；只在用户保存后统一下发。
    sync_battery_settings_to_device();
    close_page();
}

void cancel_battery_settings_event(lv_event_t *) {
    battery_settings = battery_settings_snapshot;
    normalize_battery_settings();
    Serial.println("EVENT battery_settings cancel");
    close_page();
}

void create_battery_settings_page() {
    battery_settings_snapshot = battery_settings;
    normalize_battery_settings();
    lv_obj_t *page = create_page_shell("模拟电池参数设置");
    make_label(page, "1. 先选择电池类型，自动带出默认范围；最低/最高可修改",
               24, 68, &lv_font_jcy16, 0xFFF06B);

    static constexpr int CHEMISTRY_X[4] = {24, 212, 400, 588};
    static constexpr int CHEMISTRY_W[4] = {176, 176, 176, 188};
    for (size_t i = 0; i < 4; ++i) {
        battery_chemistry_buttons[i] = make_gradient_button(
            page, CHEMISTRY_X[i], 91, CHEMISTRY_W[i], 43,
            0x09243E, 0x041326, 7, UI_BORDER_DIM, 1,
            jcy_audio::Cue::Adjust);
        lv_obj_add_event_cb(
            battery_chemistry_buttons[i],
            select_battery_chemistry_event, LV_EVENT_CLICKED,
            const_cast<BatteryChemistry *>(&BATTERY_CHEMISTRY_OPTIONS[i]));
        battery_chemistry_labels[i] = make_center_label(
            battery_chemistry_buttons[i],
            battery_chemistry_name(BATTERY_CHEMISTRY_OPTIONS[i]),
            &lv_font_jcy16, 0xFFFFFF);
    }

    lv_obj_t *range_card = make_gradient_box(
        page, 24, 145, 366, 58,
        0x0B2D4B, 0x051426, 7, UI_BORDER_DIM, 1);
    make_label(range_card, "最低", 10, 19,
               &lv_font_jcy16, 0xEAF8FF);
    lv_obj_t *cell_min_button = make_gradient_button(
        range_card, 50, 7, 112, 44,
        0x092B52, 0x041327, 5, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(
        cell_min_button, numeric_field_event, LV_EVENT_CLICKED,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(NumericField::CellMinVoltage)));
    char cell_min_text[24] = {};
    format_field_value(NumericField::CellMinVoltage,
                       cell_min_text, sizeof(cell_min_text));
    numeric_value_labels[numeric_index(NumericField::CellMinVoltage)] =
        make_center_label(cell_min_button, cell_min_text,
                          &lv_font_montserrat_16, 0xFFFFFF);

    make_label(range_card, "最高", 174, 19,
               &lv_font_jcy16, 0xEAF8FF);
    lv_obj_t *cell_max_button = make_gradient_button(
        range_card, 214, 7, 142, 44,
        0x092B52, 0x041327, 5, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(
        cell_max_button, numeric_field_event, LV_EVENT_CLICKED,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(NumericField::CellMaxVoltage)));
    char cell_max_text[24] = {};
    format_field_value(NumericField::CellMaxVoltage,
                       cell_max_text, sizeof(cell_max_text));
    numeric_value_labels[numeric_index(NumericField::CellMaxVoltage)] =
        make_center_label(cell_max_button, cell_max_text,
                          &lv_font_montserrat_16, 0xFFFFFF);

    lv_obj_t *cells_card = make_gradient_box(
        page, 410, 145, 366, 58,
        0x0B2D4B, 0x051426, 7, UI_BORDER_DIM, 1);
    make_label(cells_card, "电池串数", 14, 19,
               &lv_font_jcy16, 0xEAF8FF);
    lv_obj_t *cells_minus = make_gradient_button(
        cells_card, 142, 7, 52, 44,
        0x0B4161, 0x051A2D, 6, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(
        cells_minus, battery_cells_step_event, LV_EVENT_CLICKED,
        const_cast<int *>(&VOLTAGE_STEP_DIRECTIONS[0]));
    make_adjust_icon(cells_minus, false);
    lv_obj_t *cells_value = make_gradient_box(
        cells_card, 202, 7, 96, 44,
        0x092B52, 0x041327, 6, UI_CYAN, 1);
    battery_cells_value_label = make_center_label(
        cells_value, "1 / 5串", &lv_font_jcy16, 0xFFFFFF);
    lv_obj_t *cells_plus = make_gradient_button(
        cells_card, 306, 7, 50, 44,
        0x0B4161, 0x051A2D, 6, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(
        cells_plus, battery_cells_step_event, LV_EVENT_CLICKED,
        const_cast<int *>(&VOLTAGE_STEP_DIRECTIONS[1]));
    make_adjust_icon(cells_plus, true);

    create_numeric_row(page, 24, 214, 366,
                       "过流保护值", NumericField::Overcurrent);
    create_numeric_row(page, 410, 214, 366,
                       "过流保护延时", NumericField::ProtectionDelay);
    create_numeric_row(page, 24, 276, 366,
                       "静态阈值", NumericField::StaticThreshold);
    create_numeric_row(page, 410, 276, 366,
                       "电流滤波深度", NumericField::FilterDepth);
    create_numeric_row(page, 24, 338, 366,
                       "电池内阻", NumericField::BatteryResistance);

    lv_obj_t *advanced = make_gradient_box(
        page, 410, 338, 366, 57,
        0x0B2D4B, 0x051426, 6, UI_BORDER_DIM, 1);
    make_label(advanced, "强制安培档", 14, 18,
               &lv_font_jcy16, 0xEAF8FF);
    force_amp_button = make_gradient_button(
        advanced, 111, 7, 72, 43,
        0x09243E, 0x041326, 5, UI_BORDER_DIM, 1);
    lv_obj_add_event_cb(force_amp_button, toggle_force_amp_event,
                        LV_EVENT_CLICKED, nullptr);
    force_amp_label = make_center_label(
        force_amp_button, "关闭", &lv_font_jcy16, 0xFFFFFF);

    make_label(advanced, "切换配置", 198, 18,
               &lv_font_jcy16, 0xEAF8FF);
    popup_config_button = make_gradient_button(
        advanced, 282, 7, 74, 43,
        0x0D806F, 0x063C39, 5, 0x50FFD2, 1);
    lv_obj_add_event_cb(popup_config_button, toggle_popup_config_event,
                        LV_EVENT_CLICKED, nullptr);
    popup_config_label = make_center_label(
        popup_config_button, "开启", &lv_font_jcy16, 0xFFFFFF);

    battery_pack_range_label = make_label(
        page, "--", 24, 399, &lv_font_jcy16, UI_CYAN);
    lv_obj_set_width(battery_pack_range_label, 520);

    make_label(page, "保存后通过TTL下发过流保护等SCPI参数",
               24, 429, &lv_font_jcy16, 0xCBE8F8);
    lv_obj_t *cancel = make_gradient_button(
        page, 558, 414, 100, 46,
        0x25384B, 0x111D2A, 5, 0x59748A, 1,
        jcy_audio::Cue::Back);
    lv_obj_add_event_cb(cancel, cancel_battery_settings_event,
                        LV_EVENT_CLICKED, nullptr);
    make_center_label(cancel, "取消", &lv_font_jcy16, 0xFFFFFF);

    lv_obj_t *save = make_gradient_button(
        page, 670, 414, 106, 46,
        0x0D806F, 0x063C39, 5, 0x50FFD2, 1,
        jcy_audio::Cue::Confirm);
    lv_obj_add_event_cb(save, save_battery_settings_event,
                        LV_EVENT_CLICKED, nullptr);
    make_center_label(save, "保存并下发", &lv_font_jcy16, 0xFFFFFF);

    refresh_battery_settings_page();
}

const char *ramp_state_label(JcyScpiRampState state) {
    switch (state) {
        case JcyScpiRampState::Processing: return "测试中";
        case JcyScpiRampState::Pass: return "PASS";
        case JcyScpiRampState::Up: return "达到电流上限";
        case JcyScpiRampState::Down: return "检测到截止点";
        case JcyScpiRampState::Stop: return "已停止";
        case JcyScpiRampState::ErrorOcp: return "过流保护";
        case JcyScpiRampState::ErrorOvp: return "过压保护";
        case JcyScpiRampState::Error: return "设备错误";
        default: return "等待数据";
    }
}

void refresh_cutoff_type_buttons() {
    lv_obj_t *buttons[2] = {cutoff_charge_button, cutoff_discharge_button};
    for (size_t i = 0; i < 2; ++i) {
        if (buttons[i] == nullptr) {
            continue;
        }
        const bool active = cutoff_is_charge == CUTOFF_TYPE_OPTIONS[i];
        lv_obj_set_style_bg_color(
            buttons[i], color(active ? 0x0D806F : 0x09243E), 0);
        lv_obj_set_style_bg_grad_color(
            buttons[i], color(active ? 0x063C39 : 0x041326), 0);
        lv_obj_set_style_border_color(
            buttons[i], color(active ? 0x50FFD2 : UI_BORDER_DIM), 0);
    }
}

void refresh_cutoff_page() {
    const bool active = cutoff_phase == CutoffRunPhase::EnsureBatteryMode ||
                        cutoff_phase == CutoffRunPhase::ConfigureRamp ||
                        cutoff_phase == CutoffRunPhase::TriggerRamp ||
                        cutoff_phase == CutoffRunPhase::Running;
    if (cutoff_start_label != nullptr) {
        lv_label_set_text(cutoff_start_label, active ? "停止" : "启动测试");
    }
    if (cutoff_start_button != nullptr) {
        lv_obj_set_style_bg_color(
            cutoff_start_button,
            color(active ? 0xFF675A : 0x0D806F), 0);
        lv_obj_set_style_bg_grad_color(
            cutoff_start_button,
            color(active ? 0xD82D48 : 0x063C39), 0);
        lv_obj_set_style_border_color(
            cutoff_start_button,
            color(active ? 0xFFB4A8 : 0x50FFD2), 0);
    }
    if (cutoff_status_label != nullptr) {
        const char *text = "待机";
        switch (cutoff_phase) {
            case CutoffRunPhase::EnsureBatteryMode:
                text = "切换普通电池模式";
                break;
            case CutoffRunPhase::ConfigureRamp:
                text = "下发爬升/下降参数";
                break;
            case CutoffRunPhase::TriggerRamp:
                text = "正在触发测试";
                break;
            case CutoffRunPhase::Running:
                text = "测试中";
                break;
            case CutoffRunPhase::Completed:
                text = "测试完成";
                break;
            case CutoffRunPhase::Failed:
                text = "测试失败";
                break;
            case CutoffRunPhase::Stopped:
                text = "已停止";
                break;
            default:
                break;
        }
        lv_label_set_text(cutoff_status_label, text);
        lv_obj_set_style_text_color(
            cutoff_status_label,
            color(cutoff_phase == CutoffRunPhase::Failed
                      ? 0xFFF06B
                      : (cutoff_phase == CutoffRunPhase::Completed
                             ? 0x73FFD8
                             : 0xFFFFFF)), 0);
    }
    refresh_cutoff_type_buttons();
}

void append_cutoff_waveform_sample(int voltage_mv, int current_ua) {
    if (cutoff_chart == nullptr || cutoff_voltage_series == nullptr ||
        cutoff_current_series == nullptr) {
        return;
    }
    const int current_ma = current_ua / 1000;
    if (abs(current_ma) * 10 >= cutoff_current_range_ma * 8) {
        cutoff_current_range_ma = min(
            max(cutoff_current_range_ma * 2, abs(current_ma) * 2),
            max(100, battery_settings.overcurrent_ma));
        lv_chart_set_range(cutoff_chart, LV_CHART_AXIS_SECONDARY_Y,
                           -cutoff_current_range_ma,
                           cutoff_current_range_ma);
    }
    lv_chart_set_next_value(cutoff_chart, cutoff_voltage_series, voltage_mv);
    lv_chart_set_next_value(cutoff_chart, cutoff_current_series,
                            current_ma);
}

void stop_cutoff_test(const char *reason) {
    const bool was_active = cutoff_phase == CutoffRunPhase::EnsureBatteryMode ||
                            cutoff_phase == CutoffRunPhase::ConfigureRamp ||
                            cutoff_phase == CutoffRunPhase::TriggerRamp ||
                            cutoff_phase == CutoffRunPhase::Running;
    cutoff_running = false;
    cutoff_phase = CutoffRunPhase::Stopped;
    scpi_link.set_output(false);
    restore_default_poll_interval();
    if (cutoff_status_label != nullptr) {
        lv_label_set_text(cutoff_status_label,
                          reason == nullptr ? "已停止" : reason);
    }
    refresh_cutoff_page();
    if (was_active) {
        Serial.println("EVENT cutoff emergency stop -> OUT OFF");
    }
}

void cutoff_start_event(lv_event_t *) {
    const bool active = cutoff_phase == CutoffRunPhase::EnsureBatteryMode ||
                        cutoff_phase == CutoffRunPhase::ConfigureRamp ||
                        cutoff_phase == CutoffRunPhase::TriggerRamp ||
                        cutoff_phase == CutoffRunPhase::Running;
    if (active) {
        stop_cutoff_test("用户停止");
        return;
    }
    if (output_running || requested_output_state || output_change_pending) {
        show_info_dialog("请先关闭输出",
                         "截止电压测试必须从安全的输出关闭状态启动。请先关闭主界面输出，再进入测试。");
        return;
    }

    CutoffSettings &settings = cutoff_is_charge
                                   ? charge_settings
                                   : discharge_settings;
    if ((cutoff_is_charge && settings.start_mv >= settings.end_mv) ||
        (!cutoff_is_charge && settings.start_mv <= settings.end_mv)) {
        show_info_dialog("参数方向错误",
                         cutoff_is_charge
                             ? "过充测试要求开始电压小于终止电压。"
                             : "过放测试要求开始电压大于终止电压。");
        return;
    }

    scpi_link.clear_ramp_result();
    cutoff_last_result_sequence = 0;
    cutoff_started_ms = 0;
    if (cutoff_chart != nullptr) {
        lv_chart_set_all_value(cutoff_chart, cutoff_voltage_series,
                               LV_CHART_POINT_NONE);
        lv_chart_set_all_value(cutoff_chart, cutoff_current_series,
                               LV_CHART_POINT_NONE);
        lv_chart_refresh(cutoff_chart);
    }
    if (cutoff_result_label != nullptr) {
        lv_label_set_text(cutoff_result_label, "--.---V / --");
    }
    if (cutoff_elapsed_label != nullptr) {
        lv_label_set_text(cutoff_elapsed_label, "0ms");
    }

    // RAMP 只能在普通 BATTERY 模式触发。无论主界面是否处于恒流电池
    // 模式，都先等待模式切换成功，再下发 RAMP 参数。
    requested_constant_current_mode = false;
    constant_current_change_pending = false;
    cutoff_running = false;
    cutoff_phase = CutoffRunPhase::EnsureBatteryMode;
    if (!scpi_link.set_mode(JcyScpiMode::Battery)) {
        fail_cutoff_test("模式命令失败", -3);
        return;
    }
    refresh_cutoff_page();
    Serial.printf("EVENT cutoff %s start=%dmV end=%dmV time=%dms TTL\n",
                  cutoff_is_charge ? "overcharge" : "overdischarge",
                  settings.start_mv, settings.end_mv,
                  settings.duration_s * 1000);
}

void cutoff_type_event(lv_event_t *event) {
    if (cutoff_running ||
        cutoff_phase == CutoffRunPhase::EnsureBatteryMode ||
        cutoff_phase == CutoffRunPhase::ConfigureRamp ||
        cutoff_phase == CutoffRunPhase::TriggerRamp) {
        show_info_dialog("测试进行中", "请先停止当前测试，再切换过充/过放类型。");
        return;
    }
    const bool *charge = static_cast<const bool *>(
        lv_event_get_user_data(event));
    if (charge == nullptr || cutoff_is_charge == *charge) {
        return;
    }
    cutoff_is_charge = *charge;
    cutoff_phase = CutoffRunPhase::Idle;
    create_cutoff_page();
}

void save_cutoff_result_event(lv_event_t *) {
    const JcyScpiRampResult &result = scpi_link.ramp_result();
    if (!result.valid) {
        show_info_dialog("保存测试结果", "当前还没有可保存的截止测试结果。");
        return;
    }
    char current_text[24] = {};
    format_current_text(current_text, sizeof(current_text), result.current_ua);
    char body[220] = {};
    snprintf(body, sizeof(body),
             "类型：%s\n截止电压：%d.%03dV\n截止电流：%s\n耗时：%dms\n结果：%s\n\n当前版本先保留在本次运行内，SD卡文件导出后续接入。",
             cutoff_is_charge ? "过充测试" : "过放测试",
             result.voltage_mv / 1000, abs(result.voltage_mv % 1000),
             current_text, result.elapsed_ms,
             ramp_state_label(result.state));
    show_info_dialog("截止测试结果", body);
}

void create_cutoff_page() {
    lv_obj_t *page = create_page_shell("截止电压测试");
    make_label(page, "RAMP自动升降压抓取保护点；启动前必须关闭普通输出",
               24, 68, &lv_font_jcy16, 0xFFF06B);

    cutoff_charge_button = make_gradient_button(
        page, 24, 94, 174, 42,
        0x0D806F, 0x063C39, 6, 0x50FFD2, 1);
    lv_obj_add_event_cb(cutoff_charge_button, cutoff_type_event,
                        LV_EVENT_CLICKED,
                        const_cast<bool *>(&CUTOFF_TYPE_OPTIONS[0]));
    make_center_label(cutoff_charge_button, "过充测试",
                      &lv_font_jcy18, 0xFFFFFF);
    cutoff_discharge_button = make_gradient_button(
        page, 210, 94, 174, 42,
        0x09243E, 0x041326, 6, UI_BORDER_DIM, 1);
    lv_obj_add_event_cb(cutoff_discharge_button, cutoff_type_event,
                        LV_EVENT_CLICKED,
                        const_cast<bool *>(&CUTOFF_TYPE_OPTIONS[1]));
    make_center_label(cutoff_discharge_button, "过放测试",
                      &lv_font_jcy18, 0xFFFFFF);

    const NumericField start_field = cutoff_is_charge
        ? NumericField::ChargeStart : NumericField::DischargeStart;
    const NumericField end_field = cutoff_is_charge
        ? NumericField::ChargeEnd : NumericField::DischargeEnd;
    const NumericField duration_field = cutoff_is_charge
        ? NumericField::ChargeDuration : NumericField::DischargeDuration;
    const NumericField current_field = cutoff_is_charge
        ? NumericField::ChargeCutoffCurrent
        : NumericField::DischargeCutoffCurrent;

    create_numeric_row(page, 24, 145, 360, "开始电压", start_field);
    create_numeric_row(page, 24, 207, 360, "终止电压", end_field);
    create_numeric_row(page, 24, 269, 360, "爬升/下降时长", duration_field);
    create_numeric_row(page, 24, 331, 360,
                       cutoff_is_charge ? "充电截止电流" : "放电截止电流",
                       current_field);

    lv_obj_t *chart_panel = make_gradient_box(
        page, 405, 94, 371, 208,
        0x081F38, 0x030D1B, 7, UI_BORDER, 1);
    make_label(chart_panel, "V", 10, 7,
               &lv_font_montserrat_16, 0x65F2DB);
    make_label(chart_panel, "I", 337, 7,
               &lv_font_montserrat_16, 0xFFB46A);
    cutoff_chart = lv_chart_create(chart_panel);
    lv_obj_set_pos(cutoff_chart, 10, 27);
    lv_obj_set_size(cutoff_chart, 351, 170);
    lv_chart_set_type(cutoff_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_update_mode(cutoff_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_point_count(cutoff_chart, 120);
    lv_chart_set_range(cutoff_chart, LV_CHART_AXIS_PRIMARY_Y,
                       max(0, pack_minimum_mv() - 300),
                       min(MAX_BATTERY_PACK_MV, pack_maximum_mv() + 300));
    const CutoffSettings &active_settings = cutoff_is_charge
                                                ? charge_settings
                                                : discharge_settings;
    cutoff_current_range_ma = min(
        max(100, active_settings.cutoff_current_ua / 200),
        max(100, battery_settings.overcurrent_ma));
    lv_chart_set_range(cutoff_chart, LV_CHART_AXIS_SECONDARY_Y,
                       -cutoff_current_range_ma,
                       cutoff_current_range_ma);
    lv_chart_set_div_line_count(cutoff_chart, 5, 9);
    lv_obj_set_style_bg_color(cutoff_chart, color(0x0C2853), 0);
    lv_obj_set_style_border_color(cutoff_chart, color(0x3D83B8), 0);
    lv_obj_set_style_line_color(cutoff_chart, color(0x285683), LV_PART_MAIN);
    lv_obj_set_style_line_width(cutoff_chart, 1, LV_PART_MAIN);
    lv_obj_set_style_size(cutoff_chart, 0, LV_PART_INDICATOR);
    cutoff_voltage_series = lv_chart_add_series(
        cutoff_chart, color(0x65F2DB), LV_CHART_AXIS_PRIMARY_Y);
    cutoff_current_series = lv_chart_add_series(
        cutoff_chart, color(0xFFB46A), LV_CHART_AXIS_SECONDARY_Y);

    lv_obj_t *status_panel = make_gradient_box(
        page, 405, 311, 371, 80,
        0x0B2D4B, 0x051426, 7, UI_BORDER_DIM, 1);
    cutoff_realtime_voltage_label = make_label(
        status_panel, "V --.---", 12, 8,
        &lv_font_montserrat_16, 0x65F2DB);
    lv_obj_set_width(cutoff_realtime_voltage_label, 170);
    lv_label_set_long_mode(cutoff_realtime_voltage_label,
                           LV_LABEL_LONG_CLIP);
    cutoff_realtime_current_label = make_label(
        status_panel, "I --", 196, 8,
        &lv_font_montserrat_16, 0xFFCB88);
    lv_obj_set_width(cutoff_realtime_current_label, 151);
    lv_obj_set_style_text_align(cutoff_realtime_current_label,
                                LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_long_mode(cutoff_realtime_current_label,
                           LV_LABEL_LONG_CLIP);
    cutoff_result_label = make_label(
        status_panel, "--.---V / --", 12, 36,
        &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_set_width(cutoff_result_label, 235);
    cutoff_elapsed_label = make_label(
        status_panel, "0ms", 259, 36,
        &lv_font_montserrat_16, 0xCDEBFA);
    cutoff_status_label = make_label(
        status_panel, "待机", 255, 8,
        &lv_font_jcy16, 0xFFFFFF);
    lv_obj_set_width(cutoff_status_label, 104);
    lv_obj_set_style_text_align(cutoff_status_label, LV_TEXT_ALIGN_RIGHT, 0);

    cutoff_start_button = make_gradient_button(
        page, 405, 399, 224, 61,
        0x0D806F, 0x063C39, 8, 0x50FFD2, 1,
        jcy_audio::Cue::Toggle);
    lv_obj_add_event_cb(cutoff_start_button, cutoff_start_event,
                        LV_EVENT_CLICKED, nullptr);
    cutoff_start_label = make_center_label(
        cutoff_start_button, "启动测试", &lv_font_jcy20, 0xFFFFFF);

    lv_obj_t *save_button = make_gradient_button(
        page, 642, 399, 134, 61,
        0x0B4161, 0x051A2D, 8, UI_CYAN, 1,
        jcy_audio::Cue::Navigate);
    lv_obj_add_event_cb(save_button, save_cutoff_result_event,
                        LV_EVENT_CLICKED, nullptr);
    make_center_label(save_button, "查看结果", &lv_font_jcy16, 0xFFFFFF);

    refresh_cutoff_page();
}

void process_cutoff_state(uint32_t now_ms) {
    if (cutoff_phase != CutoffRunPhase::Running) {
        return;
    }

    if (now_ms - cutoff_last_query_ms >= 250) {
        scpi_link.query_ramp_state();
        cutoff_last_query_ms = now_ms;
    }

    const JcyScpiRampResult &result = scpi_link.ramp_result();
    if (result.valid && result.sequence != cutoff_last_result_sequence) {
        cutoff_last_result_sequence = result.sequence;
        append_cutoff_waveform_sample(result.voltage_mv, result.current_ua);
        char current_text[24] = {};
        format_current_text(current_text, sizeof(current_text),
                            result.current_ua);
        if (cutoff_realtime_voltage_label != nullptr) {
            lv_label_set_text_fmt(cutoff_realtime_voltage_label,
                                  "V %d.%03d",
                                  result.voltage_mv / 1000,
                                  abs(result.voltage_mv % 1000));
        }
        if (cutoff_realtime_current_label != nullptr) {
            lv_label_set_text_fmt(cutoff_realtime_current_label,
                                  "I %s", current_text);
        }
        if (cutoff_result_label != nullptr) {
            lv_label_set_text_fmt(cutoff_result_label,
                                  "%d.%03dV / %s",
                                  result.voltage_mv / 1000,
                                  abs(result.voltage_mv % 1000),
                                  ramp_state_label(result.state));
        }
        if (cutoff_elapsed_label != nullptr) {
            lv_label_set_text_fmt(cutoff_elapsed_label, "%dms",
                                  result.elapsed_ms);
        }
        if (cutoff_status_label != nullptr) {
            lv_label_set_text(cutoff_status_label,
                              ramp_state_label(result.state));
        }

        if (result.terminal) {
            cutoff_running = false;
            cutoff_phase = (result.state == JcyScpiRampState::Pass ||
                            result.state == JcyScpiRampState::Up ||
                            result.state == JcyScpiRampState::Down)
                               ? CutoffRunPhase::Completed
                               : (result.state == JcyScpiRampState::Stop
                                      ? CutoffRunPhase::Stopped
                                      : CutoffRunPhase::Failed);
            scpi_link.set_output(false);
            restore_default_poll_interval();
            refresh_cutoff_page();
            Serial.printf("EVENT cutoff complete state=%s V=%d I=%d t=%d\n",
                          result.state_text, result.voltage_mv,
                          result.current_ua, result.elapsed_ms);
        }
    }

    if (cutoff_started_ms != 0) {
        const CutoffSettings &settings = cutoff_is_charge
                                             ? charge_settings
                                             : discharge_settings;
        const uint32_t timeout_ms =
            static_cast<uint32_t>(settings.duration_s) * 1000U + 5000U;
        if (now_ms - cutoff_started_ms > timeout_ms) {
            fail_cutoff_test("测试超时", -4);
        }
    }
}

void refresh_power_page_output_state() {
    if (power_page_output_button == nullptr) {
        return;
    }
    const bool visual_on = output_change_pending
                               ? requested_output_state
                               : output_running;
    uint32_t top = 0x0A3150;
    uint32_t bottom = 0x041426;
    uint32_t border = UI_CYAN;
    uint32_t icon = UI_CYAN;
    const char *text = "输出已关闭";
    if (output_change_pending) {
        top = 0xD88935;
        bottom = 0x8F431E;
        border = 0xFFD28B;
        icon = 0xFFF4D2;
        text = visual_on ? "正在开启输出" : "正在关闭输出";
    } else if (visual_on) {
        top = 0xFF675A;
        bottom = 0xD82D48;
        border = 0xFFB4A8;
        icon = 0xFFFFFF;
        text = "输出已开启";
    }
    lv_obj_set_style_bg_color(power_page_output_button, color(top), 0);
    lv_obj_set_style_bg_grad_color(power_page_output_button,
                                   color(bottom), 0);
    lv_obj_set_style_border_color(power_page_output_button,
                                  color(border), 0);
    if (power_page_output_icon != nullptr) {
        lv_obj_set_style_img_recolor(power_page_output_icon,
                                     color(icon), 0);
        lv_obj_set_style_img_recolor_opa(
            power_page_output_icon,
            output_change_pending || visual_on ? LV_OPA_20 : LV_OPA_TRANSP,
            0);
    }
    if (power_page_output_stem != nullptr) {
        lv_obj_set_style_bg_color(power_page_output_stem,
                                  color(icon), 0);
        lv_obj_set_style_bg_grad_color(power_page_output_stem,
                                       color(icon), 0);
    }
    if (power_output_label != nullptr) {
        lv_label_set_text(power_output_label, text);
    }
}

void refresh_power_identity_ui() {
    if (power_rating_label == nullptr) {
        return;
    }
    const JcyScpiIdentity &identity = scpi_link.identity();
    if (identity.valid) {
        lv_label_set_text_fmt(power_rating_label, "%s  参数设置",
                              identity.primary_rating);
    } else {
        lv_label_set_text(power_rating_label, "规格读取中");
    }
}

void refresh_power_page_status() {
    if (power_page_status_bar == nullptr ||
        power_page_status_icon == nullptr ||
        power_page_status_label == nullptr) {
        return;
    }

    const uint32_t now = millis();
    const JcyScpiMeasurements &measurements = scpi_link.measurements();
    const bool fresh = scpi_link.measurements_fresh(now);
    uint32_t top = 0x0A3A45;
    uint32_t bottom = 0x06232F;
    uint32_t border = 0x167B72;
    uint32_t icon_color = 0x83FFD3;
    const char *icon = "OK";
    const char *text = "设备正常";

    if (!fresh) {
        top = 0x27465F;
        bottom = 0x173044;
        border = 0x537C98;
        icon_color = 0xB7D8EC;
        icon = "--";
        text = "等待实时数据";
    } else {
        const bool idle_stop = !measurements.output_on &&
                               measurements.error_code == 5;
        int alarm_code = idle_stop ? 0 : measurements.error_code;
        const char *alarm_status = idle_stop ? nullptr : measurements.status;
        if (alarm_code == 0 && measurements.temperature_c10 > 600) {
            alarm_code = 60;
            alarm_status = "overtemperature";
        }
        const JcyAlarmKind alarm = classify_alarm(alarm_code, alarm_status);
        if (alarm != JcyAlarmKind::None) {
            icon = "!";
            text = alarm_short_label(alarm);
            icon_color = 0xFFFFFF;
            if (alarm == JcyAlarmKind::ShortCircuit ||
                alarm == JcyAlarmKind::ReversePolarity) {
                top = 0xFF596A;
                bottom = 0xD92E49;
                border = 0xFF9BAD;
            } else {
                top = 0xFFAD4D;
                bottom = 0xE36B34;
                border = 0xFFD18A;
            }
        }
    }

    lv_obj_set_style_bg_color(power_page_status_bar, color(top), 0);
    lv_obj_set_style_bg_grad_color(power_page_status_bar, color(bottom), 0);
    lv_obj_set_style_border_color(power_page_status_bar, color(border), 0);
    lv_obj_set_style_text_color(power_page_status_icon,
                                color(icon_color), 0);
    lv_label_set_text(power_page_status_icon, icon);
    lv_label_set_text(power_page_status_label, text);
}

void power_output_event(lv_event_t *) {
    apply_output_change(!requested_output_state, true);
    refresh_power_page_output_state();
    Serial.printf("EVENT power output=%d TTL\n", requested_output_state);
}

void power_digit_select_event(lv_event_t *event) {
    const auto *command = static_cast<const PowerDigitCommand *>(
        lv_event_get_user_data(event));
    if (command == nullptr || command->digit >= 5) {
        return;
    }
    const int field_index = power_digit_field_index(command->field);
    if (field_index < 0) {
        return;
    }

    const uint32_t now = millis();
    const bool double_tap =
        power_last_tap_digits[field_index] == command->digit &&
        static_cast<uint32_t>(now - power_last_tap_ms[field_index]) <= 350;
    power_edit_digits[field_index] = command->digit;
    update_power_digit_selection(command->field);
    power_last_tap_digits[field_index] = command->digit;
    power_last_tap_ms[field_index] = now;

    if (double_tap) {
        power_last_tap_digits[field_index] = 0xFF;
        power_last_tap_ms[field_index] = 0;
        open_numeric_field(command->field);
        Serial.printf("EVENT power direct input field=%u\n",
                      static_cast<unsigned>(command->field));
        return;
    }

    Serial.printf("EVENT power digit field=%u digit=%u step=%d\n",
                  static_cast<unsigned>(command->field),
                  static_cast<unsigned>(command->digit),
                  VOLTAGE_DIGIT_STEPS_MV[command->digit]);
}

void power_digit_step_event(lv_event_t *event) {
    const auto *command = static_cast<const PowerStepCommand *>(
        lv_event_get_user_data(event));
    if (command == nullptr ||
        (command->direction != -1 && command->direction != 1)) {
        return;
    }
    const int field_index = power_digit_field_index(command->field);
    NumericSpec spec = numeric_spec_for(command->field);
    if (field_index < 0 || spec.value == nullptr) {
        return;
    }

    const int step = VOLTAGE_DIGIT_STEPS_MV[
        power_edit_digits[field_index]];
    *spec.value = constrain(*spec.value + command->direction * step,
                            spec.minimum, spec.maximum);
    refresh_power_digit_editor(command->field);
    send_numeric_field_to_device(command->field);
    Serial.printf("EVENT power_step field=%u value=%d step=%d TTL\n",
                  static_cast<unsigned>(command->field), *spec.value,
                  command->direction * step);
}

void power_tab_event(lv_event_t *event) {
    const PowerTab requested_tab = static_cast<PowerTab>(
        reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    if (requested_tab == active_power_tab) {
        return;
    }
    if (!require_output_off_for_action("切换双向电源工作模式")) {
        return;
    }
    active_power_tab = requested_tab;
    if (active_power_tab == PowerTab::Source) {
        request_device_mode(JcyScpiMode::Source, true);
    } else if (active_power_tab == PowerTab::Load) {
        request_device_mode(JcyScpiMode::Load, true);
    } else {
        // 充放电是工步配置入口，不直接对应单一底层模式。进入时先关断
        // 输出；真正运行工步时再按每个步骤切 SOURCE/LOAD。
        apply_output_change(false, false);
    }
    rebuild_power_content();
}

void load_mode_event(lv_event_t *event) {
    const bool requested_load_cc =
        reinterpret_cast<intptr_t>(lv_event_get_user_data(event)) != 0;
    if (requested_load_cc == power_settings.load_cc) {
        return;
    }
    if (!require_output_off_for_action("切换负载 CC/CV 模式")) {
        return;
    }
    power_settings.load_cc = requested_load_cc;
    if (device_mode_ready(JcyScpiMode::Load)) {
        apply_confirmed_mode_parameters(JcyScpiMode::Load);
    } else {
        request_device_mode(JcyScpiMode::Load, true);
    }
    rebuild_power_content();
    Serial.printf("EVENT load_mode=%s TTL\n",
                  power_settings.load_cc ? "CC" : "CV");
}

void cycle_toggle_event(lv_event_t *) {
    power_settings.cycle_running = !power_settings.cycle_running;
    if (cycle_button_label != nullptr) {
        lv_label_set_text(cycle_button_label,
                          power_settings.cycle_running
                              ? "停止循环" : "启动循环");
    }
    Serial.printf("EVENT cycle running=%d UI_ONLY\n",
                  power_settings.cycle_running);
}

void cycle_settings_event(lv_event_t *) {
    show_info_dialog(
        "充放电工步",
        "对应充电、静置、放电和循环工步配置。当前已完成页面入口与启停状态，下一步接入工步表、真实电压电流采样和记录存储。");
}

void create_power_digit_adjuster(lv_obj_t *parent, int y, int height,
                                 const char *name, NumericField field) {
    const int field_index = power_digit_field_index(field);
    if (field_index < 0) {
        return;
    }

    const int parent_width = lv_obj_get_width(parent);
    const int card_width = max(360, parent_width - 24);
    lv_obj_t *card = make_gradient_box(
        parent, 12, y, card_width, height,
        0x0B2D4B, 0x051426, 8, UI_BORDER_DIM, 1);
    make_label(card, name, 12, 8, &lv_font_jcy16, 0xDFF6FF);
    lv_obj_t *hint = make_label(
        card, "单击选位  双击数字直接输入", 150, 9,
        &lv_font_jcy16, UI_MUTED);
    lv_obj_set_width(hint, max(180, card_width - 162));
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_RIGHT, 0);

    const int control_height = height - 43;
    constexpr int OUTER_MARGIN = 8;
    constexpr int CONTROL_GAP = 8;
    constexpr int MINUS_WIDTH = 50;
    constexpr int PLUS_WIDTH = 54;
    const int value_width = card_width - OUTER_MARGIN * 2 -
                            CONTROL_GAP * 2 - MINUS_WIDTH - PLUS_WIDTH;
    const int value_x = OUTER_MARGIN + MINUS_WIDTH + CONTROL_GAP;
    const int plus_x = value_x + value_width + CONTROL_GAP;
    lv_obj_t *minus = make_gradient_button(
        card, OUTER_MARGIN, 35, MINUS_WIDTH, control_height,
        0x0B4161, 0x051A2D, 7, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(
        minus, power_digit_step_event, LV_EVENT_CLICKED,
        const_cast<PowerStepCommand *>(&POWER_STEP_COMMANDS[field_index][0]));
    make_adjust_icon(minus, false);

    lv_obj_t *value_box = make_gradient_box(
        card, value_x, 35, value_width, control_height,
        0x071B35, 0x020913, 7, UI_BORDER, 1);
    static constexpr int DIGIT_X[5] = {6, 44, 94, 132, 170};
    const int digit_height = control_height - 16;
    for (size_t digit = 0; digit < 5; ++digit) {
        power_digit_boxes[field_index][digit] = make_gradient_button(
            value_box, DIGIT_X[digit], 8, 36, digit_height,
            0x071A31, 0x030D1C, 4, UI_BORDER_DIM, 1,
            jcy_audio::Cue::Adjust);
        lv_obj_add_event_cb(
            power_digit_boxes[field_index][digit],
            power_digit_select_event, LV_EVENT_CLICKED,
            const_cast<PowerDigitCommand *>(
                &POWER_DIGIT_COMMANDS[field_index][digit]));
        power_digit_labels[field_index][digit] = make_center_label(
            power_digit_boxes[field_index][digit], "0",
            &lv_font_montserrat_20, UI_TEXT);
    }
    make_label(value_box, ".", 82, (control_height - 25) / 2,
               &lv_font_montserrat_20, UI_TEXT);
    const char *unit =
        field == NumericField::SourceCurrent ||
                field == NumericField::LoadCurrent
            ? "A" : "V";
    make_label(value_box, unit, value_width - 24,
               (control_height - 20) / 2,
               &lv_font_montserrat_16, UI_MUTED);

    lv_obj_t *plus = make_gradient_button(
        card, plus_x, 35, PLUS_WIDTH, control_height,
        0x0B4161, 0x051A2D, 7, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(
        plus, power_digit_step_event, LV_EVENT_CLICKED,
        const_cast<PowerStepCommand *>(&POWER_STEP_COMMANDS[field_index][1]));
    make_adjust_icon(plus, true);

    refresh_power_digit_editor(field);
}

lv_obj_t *make_power_asset_image(lv_obj_t *parent,
                                 const lv_img_dsc_t *source,
                                 int x, int y) {
    lv_obj_t *image = lv_img_create(parent);
    lv_img_set_src(image, source);
    lv_obj_set_pos(image, x, y);
    lv_obj_clear_flag(image, LV_OBJ_FLAG_CLICKABLE);
    return image;
}

void create_power_metric_row(lv_obj_t *parent,
                             const char *name, int y,
                             lv_obj_t **value_label,
                             const lv_font_t *value_font =
                                 &lv_font_montserrat_36) {
    const int row_width = max(150, lv_obj_get_width(parent) - 20);
    lv_obj_t *row = make_gradient_box(
        parent, 10, y, row_width, 43,
        0x082844, 0x041426, 6, UI_BORDER_DIM, 1);
    make_label(row, name, 10, 12,
               &lv_font_jcy16, UI_TEXT);
    *value_label = make_label(row, "--", 50, 2,
                              value_font, 0xFFFFFF);
    lv_obj_set_width(*value_label, max(90, row_width - 58));
    lv_obj_set_style_text_align(*value_label, LV_TEXT_ALIGN_RIGHT, 0);
}

void rebuild_power_content() {
    if (power_content == nullptr) {
        return;
    }
    memset(numeric_value_labels, 0, sizeof(numeric_value_labels));
    memset(power_digit_boxes, 0, sizeof(power_digit_boxes));
    memset(power_digit_labels, 0, sizeof(power_digit_labels));
    lv_obj_clean(power_content);
    power_page_output_button = nullptr;
    power_page_output_icon = nullptr;
    power_page_output_stem = nullptr;
    power_output_label = nullptr;
    cycle_button_label = nullptr;
    power_page_status_bar = nullptr;
    power_page_status_icon = nullptr;
    power_page_status_label = nullptr;
    power_live_voltage_label = nullptr;
    power_live_current_label = nullptr;
    power_live_power_label = nullptr;
    power_live_temperature_label = nullptr;

    for (size_t i = 0; i < 3; ++i) {
        if (power_tab_buttons[i] == nullptr) {
            continue;
        }
        const bool active = static_cast<size_t>(active_power_tab) == i;
        lv_obj_set_style_bg_color(
            power_tab_buttons[i],
            color(active ? 0x0D748D : 0x071A31), 0);
        lv_obj_set_style_bg_grad_color(
            power_tab_buttons[i],
            color(active ? 0x073A51 : 0x030D1B), 0);
        lv_obj_set_style_border_color(
            power_tab_buttons[i],
            color(active ? UI_CYAN : UI_BORDER_DIM), 0);
        lv_obj_set_style_border_width(
            power_tab_buttons[i], active ? 2 : 1, 0);
    }

    lv_obj_t *meter = make_gradient_box(
        power_content, 0, 0, 194, 294,
        0x092641, 0x041224, 10, UI_BORDER, 1);
    make_label(meter, "实时数据", 14, 11,
               &lv_font_jcy18, 0xFFFFFF);
    lv_obj_t *mode_text = make_label(
        meter,
        active_power_tab == PowerTab::Source ? "电源模式" :
        active_power_tab == PowerTab::Load ? "负载模式" : "充放电",
        105, 13, &lv_font_jcy16, UI_CYAN);
    lv_obj_set_width(mode_text, 74);
    lv_obj_set_style_text_align(mode_text, LV_TEXT_ALIGN_RIGHT, 0);

    create_power_metric_row(
        meter, "电压", 44, &power_live_voltage_label);
    create_power_metric_row(
        meter, "电流", 90, &power_live_current_label);
    create_power_metric_row(
        meter, "功率", 136, &power_live_power_label,
        &lv_font_montserrat_30);

    power_page_output_button = make_gradient_button(
        meter, 10, 183, 174, 68,
        0x0A3150, 0x041426, 8, UI_CYAN, 1,
        jcy_audio::Cue::Power);
    lv_obj_add_event_cb(power_page_output_button, power_output_event,
                        LV_EVENT_CLICKED, nullptr);
    power_page_output_icon = make_power_asset_image(
        power_page_output_button,
        &lv_img_jcy_power_power_switch, 12, 5);
    power_output_label = make_label(
        power_page_output_button, "输出已关闭", 58, 24,
        &lv_font_jcy18, 0xFFFFFF);
    lv_obj_set_width(power_output_label, 106);
    lv_obj_set_style_text_align(power_output_label,
                                LV_TEXT_ALIGN_CENTER, 0);

    power_page_status_bar = make_gradient_box(
        meter, 10, 257, 174, 27,
        0x0A3A45, 0x06232F, 7, 0x167B72, 1);
    power_page_status_icon = make_label(
        power_page_status_bar, "OK", 10, 5,
        &lv_font_montserrat_14, 0x83FFD3);
    power_page_status_label = make_label(
        power_page_status_bar, "设备正常", 40, 4,
        &lv_font_jcy16, 0xFFFFFF);
    lv_obj_set_width(power_page_status_label, 124);
    lv_obj_set_style_text_align(power_page_status_label,
                                LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *control = make_gradient_box(
        power_content, 206, 0, 442, 294,
        0x081F38, 0x030D1B, 10, UI_BORDER, 1);

    if (active_power_tab == PowerTab::Source) {
        make_label(control, "电源模式", 18, 11,
                   &lv_font_jcy20, 0xFFFFFF);
        make_label(control, "设置输出电压和限流值", 112, 14,
                   &lv_font_jcy16, UI_MUTED);
        create_power_digit_adjuster(
            control, 42, 116, "电压设置 / 微调",
            NumericField::SourceVoltage);
        create_power_digit_adjuster(
            control, 170, 116, "电流设置 / 微调",
            NumericField::SourceCurrent);
    } else if (active_power_tab == PowerTab::Load) {
        make_label(control, "负载模式", 18, 11,
                   &lv_font_jcy20, 0xFFFFFF);
        make_label(control, "选择恒流CC或恒压CV", 108, 14,
                   &lv_font_jcy16, UI_MUTED);
        lv_obj_t *cc = make_gradient_button(
            control, 282, 6, 68, 36,
            power_settings.load_cc ? 0x0D806F : 0x09243E,
            power_settings.load_cc ? 0x063C39 : 0x041326,
            18, power_settings.load_cc ? 0x50FFD2 : UI_BORDER_DIM, 1);
        lv_obj_add_event_cb(cc, load_mode_event, LV_EVENT_CLICKED,
                            reinterpret_cast<void *>(1));
        make_center_label(cc, "CC", &lv_font_montserrat_16, 0xFFFFFF);
        lv_obj_t *cv = make_gradient_button(
            control, 358, 6, 68, 36,
            !power_settings.load_cc ? 0x0D806F : 0x09243E,
            !power_settings.load_cc ? 0x063C39 : 0x041326,
            18, !power_settings.load_cc ? 0x50FFD2 : UI_BORDER_DIM, 1);
        lv_obj_add_event_cb(cv, load_mode_event, LV_EVENT_CLICKED,
                            reinterpret_cast<void *>(0));
        make_center_label(cv, "CV", &lv_font_montserrat_16, 0xFFFFFF);

        if (power_settings.load_cc) {
            create_power_digit_adjuster(
                control, 54, 122, "恒流设置 / 微调",
                NumericField::LoadCurrent);
        } else {
            create_power_digit_adjuster(
                control, 54, 122, "恒压设置 / 微调",
                NumericField::LoadVoltage);
        }
        lv_obj_t *hint = make_gradient_box(
            control, 12, 184, 418, 102,
            0x071A31, 0x041224, 8, UI_BORDER_DIM, 1);
        make_power_asset_image(
            hint, &lv_img_jcy_power_info, 14, 10);
        make_label(hint, "工作说明", 48, 13,
                   &lv_font_jcy16, UI_CYAN);
        make_label(hint,
                   "CC按设定电流吸收能量，CV按设定电压限制负载\n输出开启后左侧实时显示电压、电流和功率",
                   48, 42, &lv_font_jcy16, 0xCDE8F7);
    } else {
        make_label(control, "充放电循环", 18, 11,
                   &lv_font_jcy20, 0xFFFFFF);
        make_label(control, "电压 / 电流趋势", 143, 14,
                   &lv_font_jcy16, UI_MUTED);
        lv_obj_t *chart_box = make_gradient_box(
            control, 12, 43, 418, 179,
            0x071A31, 0x030D1B, 7, 0x1A5A82, 1);
        for (int i = 1; i < 5; ++i) {
            make_gradient_box(chart_box, 0, i * 35, 418, 1,
                              0x1B466B, 0x1B466B);
        }
        for (int i = 1; i < 7; ++i) {
            make_gradient_box(chart_box, i * 59, 0, 1, 179,
                              0x1B466B, 0x1B466B);
        }
        static lv_point_t voltage_points[] = {
            {10, 138}, {70, 115}, {130, 77}, {196, 77},
            {262, 99}, {328, 133}, {402, 133},
        };
        lv_obj_t *voltage_line = lv_line_create(chart_box);
        lv_line_set_points(voltage_line, voltage_points,
                           sizeof(voltage_points) /
                               sizeof(voltage_points[0]));
        lv_obj_set_style_line_width(voltage_line, 3, 0);
        lv_obj_set_style_line_color(voltage_line,
                                    color(0x65F2DB), 0);
        static lv_point_t current_points[] = {
            {10, 105}, {70, 67}, {130, 67}, {196, 145},
            {262, 145}, {328, 72}, {402, 72},
        };
        lv_obj_t *current_line = lv_line_create(chart_box);
        lv_line_set_points(current_line, current_points,
                           sizeof(current_points) /
                               sizeof(current_points[0]));
        lv_obj_set_style_line_width(current_line, 3, 0);
        lv_obj_set_style_line_color(current_line,
                                    color(0xFFB46A), 0);
        make_label(chart_box, "V", 12, 7,
                   &lv_font_montserrat_14, 0x65F2DB);
        make_label(chart_box, "I", 39, 7,
                   &lv_font_montserrat_14, 0xFFB46A);

        lv_obj_t *cycle_button = make_gradient_button(
            control, 12, 232, 204, 52,
            power_settings.cycle_running ? 0xFF675A : 0x0D806F,
            power_settings.cycle_running ? 0xD82D48 : 0x063C39,
            8, power_settings.cycle_running ? 0xFFB4A8 : 0x50FFD2, 1);
        lv_obj_add_event_cb(cycle_button, cycle_toggle_event,
                            LV_EVENT_CLICKED, nullptr);
        cycle_button_label = make_center_label(
            cycle_button,
            power_settings.cycle_running ? "停止循环" : "启动循环",
            &lv_font_jcy18, 0xFFFFFF);
        lv_obj_t *settings = make_gradient_button(
            control, 226, 232, 204, 52,
            0x0B4161, 0x051A2D, 8, UI_CYAN, 1);
        lv_obj_add_event_cb(settings, cycle_settings_event,
                            LV_EVENT_CLICKED, nullptr);
        make_center_label(settings, "工步设置",
                          &lv_font_jcy18, 0xFFFFFF);
    }
    refresh_live_measurements();
    refresh_power_page_output_state();
    refresh_power_page_status();
}

void create_power_page() {
    active_mode = WorkMode::BidirectionalPower;
    // 每次进入双向电源默认显示 SOURCE，但保留 Source/Load 各自上次值。
    active_power_tab = PowerTab::Source;
    request_device_mode(JcyScpiMode::Source, false);

    close_info_dialog();
    close_keypad();
    if (page_overlay != nullptr) {
        lv_obj_del(page_overlay);
        page_overlay = nullptr;
    }
    reset_page_references();

    // 双向电源只替换右侧工作区。主界面顶部状态栏和左侧“模拟电池 /
    // 双向电源”两个固定入口始终保留，用户无需先关闭本页即可看清当前
    // 模式并在两种模式之间切换。
    constexpr int POWER_PANE_X = 132;
    constexpr int POWER_PANE_Y = 58;
    constexpr int POWER_PANE_WIDTH = SCREEN_WIDTH - POWER_PANE_X;
    constexpr int POWER_PANE_HEIGHT = SCREEN_HEIGHT - POWER_PANE_Y;
    page_overlay = lv_obj_create(ui_screen);
    lv_obj_set_pos(page_overlay, POWER_PANE_X, POWER_PANE_Y);
    lv_obj_set_size(page_overlay, POWER_PANE_WIDTH, POWER_PANE_HEIGHT);
    style_clean(page_overlay);
    lv_obj_set_style_radius(page_overlay, 0, 0);
    lv_obj_set_style_border_width(page_overlay, 1, 0);
    lv_obj_set_style_border_color(page_overlay, color(UI_BORDER_DIM), 0);
    lv_obj_set_style_bg_color(page_overlay, color(UI_BG_TOP), 0);
    lv_obj_set_style_bg_grad_color(page_overlay, color(UI_BG_BOTTOM), 0);
    lv_obj_set_style_bg_grad_dir(page_overlay, LV_GRAD_DIR_VER, 0);

    lv_obj_t *header = make_gradient_box(
        page_overlay, 0, 0, POWER_PANE_WIDTH, 48,
        0x0A2440, 0x030D1C, 0, UI_BORDER_DIM, 0);
    make_label(header, "双向电源", 18, 6,
               &lv_font_jcy20, 0xFFFFFF);
    make_label(header, "SOURCE / LOAD / CYCLE", 128, 11,
               &lv_font_montserrat_12, UI_MUTED);
    make_label(header, JCY1000_FIRMWARE_VERSION,
               438, 15, &lv_font_montserrat_14, UI_MUTED);
    lv_obj_t *close_button = make_gradient_button(
        header, 610, 3, 48, 42,
        0x5A1830, 0x260B1B,
        LV_RADIUS_CIRCLE, 0xEF4A67, 1);
    lv_obj_add_event_cb(close_button, close_page_event,
                        LV_EVENT_CLICKED, nullptr);
    make_center_label(close_button, LV_SYMBOL_CLOSE,
                      &lv_font_montserrat_20, 0xFFFFFF);

    const char *tab_names[] = {"电源模式", "负载模式", "充放电"};
    const char *tab_subtitles[] = {"SOURCE", "LOAD", "CYCLE"};
    const lv_img_dsc_t *tab_icons[] = {
        &lv_img_jcy_power_tab_source,
        &lv_img_jcy_power_tab_load,
        &lv_img_jcy_power_tab_cycle,
    };
    static constexpr int TAB_X[] = {10, 152, 294};
    for (size_t i = 0; i < 3; ++i) {
        const bool active = static_cast<size_t>(active_power_tab) == i;
        power_tab_buttons[i] = make_gradient_button(
            page_overlay, TAB_X[i], 54, 136, 60,
            active ? 0x0D748D : 0x071A31,
            active ? 0x073A51 : 0x030D1B,
            9, active ? UI_CYAN : UI_BORDER_DIM,
            active ? 2 : 1);
        lv_obj_add_event_cb(
            power_tab_buttons[i], power_tab_event, LV_EVENT_CLICKED,
            reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        make_power_asset_image(
            power_tab_buttons[i], tab_icons[i], 6, 7);
        make_label(power_tab_buttons[i], tab_names[i], 52, 8,
                   &lv_font_jcy16, 0xFFFFFF);
        make_label(power_tab_buttons[i], tab_subtitles[i], 52, 34,
                   &lv_font_montserrat_12, UI_MUTED);
    }
    lv_obj_t *rating = make_gradient_box(
        page_overlay, 438, 62, 220, 44,
        0x071A31, 0x041224, 8, UI_CYAN, 1);
    power_rating_label = make_center_label(
        rating, "规格读取中",
        &lv_font_jcy18, UI_CYAN);

    power_content = lv_obj_create(page_overlay);
    lv_obj_set_pos(power_content, 10, 122);
    lv_obj_set_size(power_content, 648, 294);
    style_clean(power_content);
    lv_obj_set_style_bg_opa(power_content, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(power_content, 0, 0);
    lv_obj_clear_flag(power_content, LV_OBJ_FLAG_CLICKABLE);
    rebuild_power_content();
    refresh_power_identity_ui();
    refresh_mode_buttons();
}

enum class HubAction : uint8_t {
    ModeSwitch,
    Cutoff,
    FileManager,
    DcImpedance,
    Manual,
    Network,
    Bluetooth,
    About,
    Ota,
    Communication,
};

void hub_action_event(lv_event_t *event) {
    const auto action = static_cast<HubAction>(
        reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    switch (action) {
        case HubAction::ModeSwitch:
            open_page(PageKind::Power);
            break;
        case HubAction::Cutoff:
            open_page(PageKind::CutoffTest);
            break;
        case HubAction::FileManager:
            show_info_dialog(
                "文件管理",
                "对应导出测试记录和升级文件。当前ESP32界面尚未接入SD卡/U盘文件系统，按钮入口已保留。");
            break;
        case HubAction::DcImpedance:
            show_info_dialog(
                "直流阻抗",
                "用于电池直流内阻测试。需要接入真实恒流工步、采样和安全保护后再启用输出。");
            break;
        case HubAction::Manual:
            open_page(PageKind::Help);
            break;
        case HubAction::Network:
            open_page(PageKind::Wifi);
            break;
        case HubAction::Bluetooth:
            open_page(PageKind::Bluetooth);
            break;
        case HubAction::Ota:
            open_page(PageKind::Ota);
            break;
        case HubAction::Communication:
            open_page(PageKind::Communication);
            break;
        case HubAction::About: {
            static char about_text[160];
            snprintf(
                about_text, sizeof(about_text),
                JCY_PRODUCT_NAME " / %s\nESP32-S3 + %dx%d RGB触控界面\n%s",
                jcy_panel::kModel, PANEL_WIDTH, PANEL_HEIGHT,
                JCY1000_FIRMWARE_VERSION);
            show_info_dialog(
                "关于",
                about_text);
            break;
        }
    }
}

void create_hub_button(lv_obj_t *parent, int x, int y,
                       const char *title, const char *subtitle,
                       HubAction action) {
    lv_obj_t *button = make_gradient_button(
        parent, x, y, 235, 78,
        0x0B2D4B, 0x041224, 8, UI_BORDER_DIM, 1,
        jcy_audio::Cue::Navigate);
    lv_obj_add_event_cb(
        button, hub_action_event, LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(action)));
    make_label(button, title, 18, 12, &lv_font_jcy18, 0xFFFFFF);
    lv_obj_t *subtitle_label = make_label(
        button, subtitle, 18, 43, &lv_font_jcy16, 0xD6EFFB);
    lv_obj_set_width(subtitle_label, 204);
    lv_label_set_long_mode(subtitle_label, LV_LABEL_LONG_DOT);
}

void refresh_wireless_status_labels() {
    const WifiStatus &ws = jcy_wireless::wifi_status();
    if (wifi_status_label != nullptr) {
        const char *state = "未启用";
        if (ws.state == JcyLinkState::Idle) state = "待连接";
        else if (ws.state == JcyLinkState::Busy) state = "连接中";
        else if (ws.state == JcyLinkState::Connected) state = "已连接";
        else if (ws.state == JcyLinkState::Error) state = "连接失败";
        char text[32] = {};
        snprintf(text, sizeof(text), "状态: %s", state);
        set_label_text_if_changed(wifi_status_label, text);
    }
    if (wifi_info_label != nullptr) {
        char text[96] = {};
        if (ws.state == JcyLinkState::Connected) {
            snprintf(text, sizeof(text), "SSID: %s   IP: %s   信号: %ddBm",
                     ws.ssid, ws.ip, ws.rssi);
        } else if (jcy_wireless::wifi_scan_in_progress()) {
            snprintf(text, sizeof(text), "%s", "正在扫描附近 Wi-Fi…");
        } else if (ws.scan_count >= 0) {
            snprintf(text, sizeof(text), "扫描到 %d 个网络", ws.scan_count);
        } else {
            snprintf(text, sizeof(text), "%s", "请选择网络并输入密码");
        }
        set_label_text_if_changed(wifi_info_label, text);
    }

    const BleStatus &bs = jcy_wireless::ble_status();
    if (ble_status_label != nullptr) {
        const char *state = "未启用";
        if (bs.state == JcyLinkState::Idle) state = "广播中";
        else if (bs.state == JcyLinkState::Busy) state = "处理中";
        else if (bs.state == JcyLinkState::Connected) state = "已连接";
        else if (bs.state == JcyLinkState::Error) state = "错误";
        char text[56] = {};
        snprintf(text, sizeof(text), "状态: %s   TX:%u  RX:%u",
                 state, bs.bytes_sent, bs.bytes_received);
        set_label_text_if_changed(ble_status_label, text);
    }
    if (ble_info_label != nullptr) {
        char text[96] = {};
        if (bs.state == JcyLinkState::Connected) {
            snprintf(text, sizeof(text), "设备: %s\n连接端: %s",
                     bs.device_name, bs.peer_name);
        } else {
            snprintf(text, sizeof(text), "设备: %s\n手机扫描该名称即可连接",
                     bs.device_name);
        }
        set_label_text_if_changed(ble_info_label, text);
    }
    if (ble_toggle_button_label != nullptr) {
        set_label_text_if_changed(ble_toggle_button_label,
                                  jcy_wireless::ble_is_enabled()
                                      ? "关闭 BLE" : "开启 BLE");
    }
}

void populate_wifi_scan_dropdown() {
    if (wifi_scan_dropdown == nullptr || wifi_scan_options_loaded) {
        return;
    }
    const int16_t count = jcy_wireless::wifi_scan_count();
    if (count < 0 || jcy_wireless::wifi_scan_in_progress()) {
        return;
    }
    char options[768] = {};
    size_t offset = 0;
    wifi_scan_option_count = 0;
    const int16_t limit = count < 12 ? count : 12;
    for (int16_t i = 0; i < limit; ++i) {
        const char *ssid = jcy_wireless::wifi_ssid_for_index(i);
        if (ssid == nullptr || ssid[0] == '\0') {
            continue;
        }
        const int8_t rssi = jcy_wireless::wifi_rssi_for_index(i);
        const int written = snprintf(options + offset,
                                     sizeof(options) - offset,
                                     "%s  (%ddBm)\n", ssid, rssi);
        if (written <= 0 || offset + static_cast<size_t>(written) >= sizeof(options)) {
            break;
        }
        offset += static_cast<size_t>(written);
        wifi_scan_option_indices[wifi_scan_option_count++] = i;
    }
    if (wifi_scan_option_count == 0) {
        lv_dropdown_set_options(wifi_scan_dropdown, "No networks");
    } else {
        options[offset - 1] = '\0';
        lv_dropdown_set_options(wifi_scan_dropdown, options);
    }
    wifi_scan_options_loaded = true;
}

void refresh_wireless_ui() {
    jcy_wireless::wifi_loop();
    jcy_wireless::ble_loop();
    refresh_wireless_header();
    refresh_wireless_status_labels();
    populate_wifi_scan_dropdown();
}

void wifi_textarea_selected(lv_event_t *event) {
    lv_obj_t *textarea = lv_event_get_target(event);
    if (textarea == nullptr) return;
    if (wifi_keyboard_target != nullptr && wifi_keyboard_target != textarea) {
        lv_obj_clear_state(wifi_keyboard_target, LV_STATE_FOCUSED);
    }
    wifi_keyboard_target = textarea;
    lv_obj_add_state(wifi_keyboard_target, LV_STATE_FOCUSED);
}

void focus_wifi_password_input() {
    if (wifi_password_textarea == nullptr) {
        return;
    }
    // 扫描列表选中 SSID 后，下一步一定是输入密码；明确切换自定义键盘
    // 的输入目标，避免用户输入时误改网络名。
    if (wifi_keyboard_target != nullptr &&
        wifi_keyboard_target != wifi_password_textarea) {
        lv_obj_clear_state(wifi_keyboard_target, LV_STATE_FOCUSED);
    }
    wifi_keyboard_target = wifi_password_textarea;
    lv_textarea_set_cursor_pos(wifi_password_textarea,
                               LV_TEXTAREA_CURSOR_LAST);
    lv_obj_add_state(wifi_password_textarea, LV_STATE_FOCUSED);
}

void wifi_dropdown_selected(lv_event_t *) {
    if (wifi_scan_dropdown == nullptr || wifi_ssid_textarea == nullptr ||
        wifi_scan_option_count == 0) {
        return;
    }
    const uint16_t selected = lv_dropdown_get_selected(wifi_scan_dropdown);
    if (selected >= wifi_scan_option_count) return;
    const char *ssid = jcy_wireless::wifi_ssid_for_index(
        wifi_scan_option_indices[selected]);
    if (ssid != nullptr && ssid[0] != '\0') {
        lv_textarea_set_text(wifi_ssid_textarea, ssid);
        // 新网络不能沿用旧网络密码，避免用户误连到错误 AP。
        if (wifi_password_textarea != nullptr) {
            lv_textarea_set_text(wifi_password_textarea, "");
        }
        focus_wifi_password_input();
    }
}

void wifi_toggle_clicked(lv_event_t *) {
    if (jcy_wireless::wifi_is_enabled()) {
        jcy_wireless::wifi_disable();
    } else {
        jcy_wireless::wifi_enable();
    }
    refresh_wireless_ui();
}

void wifi_quick_szd_clicked(lv_event_t *) {
    // Wi-Fi 凭据只保存在设备 NVS，绝不编译进固件或源码仓库。
    char ssid[33] = {};
    char password[65] = {};
    jcy_wireless::wifi_load_credentials(ssid, sizeof(ssid), password,
                                       sizeof(password));
    if (ssid[0] == '\0') {
        if (wifi_connect_button_label != nullptr) {
            lv_label_set_text(wifi_connect_button_label, "请先选择网络");
        }
        return;
    }
    if (wifi_ssid_textarea != nullptr) {
        lv_textarea_set_text(wifi_ssid_textarea, ssid);
    }
    if (wifi_password_textarea != nullptr) {
        lv_textarea_set_text(wifi_password_textarea, password);
    }
    jcy_wireless::wifi_connect_to_ssid(ssid, password);
    if (wifi_connect_button_label != nullptr) {
        lv_label_set_text(wifi_connect_button_label, "连接已保存网络…");
    }
    refresh_wireless_ui();
}

void wifi_scan_clicked(lv_event_t *) {
    jcy_wireless::wifi_start_scan();
    wifi_scan_options_loaded = false;
    wifi_scan_option_count = 0;
    if (wifi_scan_dropdown != nullptr) {
        lv_dropdown_set_options(wifi_scan_dropdown, "Scanning...");
    }
    refresh_wireless_ui();
}

void wifi_connect_clicked(lv_event_t *) {
    if (wifi_ssid_textarea == nullptr || wifi_password_textarea == nullptr) {
        return;
    }
    const char *ssid = lv_textarea_get_text(wifi_ssid_textarea);
    const char *password = lv_textarea_get_text(wifi_password_textarea);
    if (ssid == nullptr || ssid[0] == '\0') {
        if (wifi_connect_button_label != nullptr) {
            lv_label_set_text(wifi_connect_button_label, "请先选择网络");
        }
        return;
    }
    jcy_wireless::wifi_save_credentials(ssid, password);
    jcy_wireless::wifi_connect_to_ssid(ssid, password);
    if (wifi_connect_button_label != nullptr) {
        lv_label_set_text(wifi_connect_button_label, "连接中…");
    }
    refresh_wireless_ui();
}

void wifi_keyboard_key_event(lv_event_t *event) {
    const auto *key = reinterpret_cast<const WifiKeyboardKey *>(
        lv_event_get_user_data(event));
    if (key == nullptr) return;

    switch (key->action) {
        case WifiKeyboardAction::Lowercase:
            wifi_keyboard_layout = WifiKeyboardLayout::Lowercase;
            create_wifi_keyboard_layout();
            return;
        case WifiKeyboardAction::Uppercase:
            wifi_keyboard_layout = WifiKeyboardLayout::Uppercase;
            create_wifi_keyboard_layout();
            return;
        case WifiKeyboardAction::Symbols:
            wifi_keyboard_layout = WifiKeyboardLayout::Symbols;
            create_wifi_keyboard_layout();
            return;
        case WifiKeyboardAction::Confirm:
            wifi_connect_clicked(nullptr);
            return;
        default:
            break;
    }

    lv_obj_t *target = wifi_keyboard_target != nullptr
        ? wifi_keyboard_target : wifi_password_textarea;
    if (target == nullptr) return;

    switch (key->action) {
        case WifiKeyboardAction::Text:
            if (key->text != nullptr) lv_textarea_add_text(target, key->text);
            break;
        case WifiKeyboardAction::Space:
            lv_textarea_add_text(target, " ");
            break;
        case WifiKeyboardAction::Backspace:
            lv_textarea_del_char(target);
            break;
        case WifiKeyboardAction::Clear:
            lv_textarea_set_text(target, "");
            break;
        default:
            break;
    }
    lv_textarea_set_cursor_pos(target, LV_TEXTAREA_CURSOR_LAST);
    lv_obj_add_state(target, LV_STATE_FOCUSED);
}

void add_wifi_keyboard_row(const WifiKeyboardKey *keys, size_t key_count,
                           int y, int height, int key_width, int gap,
                           int x_start = 4, int final_extra_width = 0) {
    if (wifi_keyboard == nullptr || keys == nullptr) return;
    int x = x_start;
    for (size_t i = 0; i < key_count; ++i) {
        const WifiKeyboardKey &key = keys[i];
        const int width = key_width +
            (i + 1 == key_count ? final_extra_width : 0);
        const bool is_confirm = key.action == WifiKeyboardAction::Confirm;
        const bool is_clear = key.action == WifiKeyboardAction::Clear;
        const bool is_mode = key.action == WifiKeyboardAction::Lowercase ||
                             key.action == WifiKeyboardAction::Uppercase ||
                             key.action == WifiKeyboardAction::Symbols;
        const uint32_t top = is_confirm ? 0x0D806F
                           : is_clear ? 0x5A1830
                           : is_mode ? 0x0B4161 : 0x0A2D4C;
        const uint32_t bottom = is_confirm ? 0x063C39
                              : is_clear ? 0x260B1B
                              : is_mode ? 0x051A2D : 0x04162A;
        const uint32_t border = is_confirm ? 0x50FFD2
                              : is_clear ? 0xEF4A67 : UI_BORDER_DIM;
        lv_obj_t *button = make_gradient_button(
            wifi_keyboard, x, y, width, height, top, bottom, 5, border, 1,
            is_confirm ? jcy_audio::Cue::Confirm : jcy_audio::Cue::Click);
        lv_obj_add_event_cb(button, wifi_keyboard_key_event, LV_EVENT_CLICKED,
                            const_cast<WifiKeyboardKey *>(&key));
        const bool chinese = static_cast<uint8_t>(key.label[0]) >= 0x80;
        make_center_label(button, key.label,
                          chinese ? &lv_font_jcy16 : &lv_font_montserrat_16,
                          0xFFFFFF);
        x += width + gap;
    }
}

void create_wifi_keyboard_layout() {
    if (wifi_keyboard == nullptr) return;
    lv_obj_clean(wifi_keyboard);

    static const WifiKeyboardKey lower_row0[] = {
        {"123", nullptr, WifiKeyboardAction::Symbols},
        {"q", "q", WifiKeyboardAction::Text}, {"w", "w", WifiKeyboardAction::Text},
        {"e", "e", WifiKeyboardAction::Text}, {"r", "r", WifiKeyboardAction::Text},
        {"t", "t", WifiKeyboardAction::Text}, {"y", "y", WifiKeyboardAction::Text},
        {"u", "u", WifiKeyboardAction::Text}, {"i", "i", WifiKeyboardAction::Text},
        {"o", "o", WifiKeyboardAction::Text}, {"p", "p", WifiKeyboardAction::Text},
        {"DEL", nullptr, WifiKeyboardAction::Backspace},
    };
    static const WifiKeyboardKey upper_row0[] = {
        {"123", nullptr, WifiKeyboardAction::Symbols},
        {"Q", "Q", WifiKeyboardAction::Text}, {"W", "W", WifiKeyboardAction::Text},
        {"E", "E", WifiKeyboardAction::Text}, {"R", "R", WifiKeyboardAction::Text},
        {"T", "T", WifiKeyboardAction::Text}, {"Y", "Y", WifiKeyboardAction::Text},
        {"U", "U", WifiKeyboardAction::Text}, {"I", "I", WifiKeyboardAction::Text},
        {"O", "O", WifiKeyboardAction::Text}, {"P", "P", WifiKeyboardAction::Text},
        {"DEL", nullptr, WifiKeyboardAction::Backspace},
    };
    static const WifiKeyboardKey symbol_row0[] = {
        {"abc", nullptr, WifiKeyboardAction::Lowercase},
        {"1", "1", WifiKeyboardAction::Text}, {"2", "2", WifiKeyboardAction::Text},
        {"3", "3", WifiKeyboardAction::Text}, {"4", "4", WifiKeyboardAction::Text},
        {"5", "5", WifiKeyboardAction::Text}, {"6", "6", WifiKeyboardAction::Text},
        {"7", "7", WifiKeyboardAction::Text}, {"8", "8", WifiKeyboardAction::Text},
        {"9", "9", WifiKeyboardAction::Text}, {"0", "0", WifiKeyboardAction::Text},
        {"DEL", nullptr, WifiKeyboardAction::Backspace},
    };

    static const WifiKeyboardKey lower_row1[] = {
        {"ABC", nullptr, WifiKeyboardAction::Uppercase},
        {"a", "a", WifiKeyboardAction::Text}, {"s", "s", WifiKeyboardAction::Text},
        {"d", "d", WifiKeyboardAction::Text}, {"f", "f", WifiKeyboardAction::Text},
        {"g", "g", WifiKeyboardAction::Text}, {"h", "h", WifiKeyboardAction::Text},
        {"j", "j", WifiKeyboardAction::Text}, {"k", "k", WifiKeyboardAction::Text},
        {"l", "l", WifiKeyboardAction::Text}, {"确定", nullptr, WifiKeyboardAction::Confirm},
    };
    static const WifiKeyboardKey upper_row1[] = {
        {"abc", nullptr, WifiKeyboardAction::Lowercase},
        {"A", "A", WifiKeyboardAction::Text}, {"S", "S", WifiKeyboardAction::Text},
        {"D", "D", WifiKeyboardAction::Text}, {"F", "F", WifiKeyboardAction::Text},
        {"G", "G", WifiKeyboardAction::Text}, {"H", "H", WifiKeyboardAction::Text},
        {"J", "J", WifiKeyboardAction::Text}, {"K", "K", WifiKeyboardAction::Text},
        {"L", "L", WifiKeyboardAction::Text}, {"确定", nullptr, WifiKeyboardAction::Confirm},
    };
    static const WifiKeyboardKey symbol_row1[] = {
        {"ABC", nullptr, WifiKeyboardAction::Uppercase},
        {"!", "!", WifiKeyboardAction::Text}, {"@", "@", WifiKeyboardAction::Text},
        {"#", "#", WifiKeyboardAction::Text}, {"$", "$", WifiKeyboardAction::Text},
        {"%", "%", WifiKeyboardAction::Text}, {"^", "^", WifiKeyboardAction::Text},
        {"&", "&", WifiKeyboardAction::Text}, {"*", "*", WifiKeyboardAction::Text},
        {"(", "(", WifiKeyboardAction::Text}, {")", ")", WifiKeyboardAction::Text},
    };

    static const WifiKeyboardKey lower_row2[] = {
        {"#+=", nullptr, WifiKeyboardAction::Symbols},
        {"z", "z", WifiKeyboardAction::Text}, {"x", "x", WifiKeyboardAction::Text},
        {"c", "c", WifiKeyboardAction::Text}, {"v", "v", WifiKeyboardAction::Text},
        {"b", "b", WifiKeyboardAction::Text}, {"n", "n", WifiKeyboardAction::Text},
        {"m", "m", WifiKeyboardAction::Text}, {".", ".", WifiKeyboardAction::Text},
        {"-", "-", WifiKeyboardAction::Text}, {"@", "@", WifiKeyboardAction::Text},
    };
    static const WifiKeyboardKey upper_row2[] = {
        {"#+=", nullptr, WifiKeyboardAction::Symbols},
        {"Z", "Z", WifiKeyboardAction::Text}, {"X", "X", WifiKeyboardAction::Text},
        {"C", "C", WifiKeyboardAction::Text}, {"V", "V", WifiKeyboardAction::Text},
        {"B", "B", WifiKeyboardAction::Text}, {"N", "N", WifiKeyboardAction::Text},
        {"M", "M", WifiKeyboardAction::Text}, {".", ".", WifiKeyboardAction::Text},
        {"-", "-", WifiKeyboardAction::Text}, {"@", "@", WifiKeyboardAction::Text},
    };
    static const WifiKeyboardKey symbol_row2[] = {
        {"+", "+", WifiKeyboardAction::Text}, {"-", "-", WifiKeyboardAction::Text},
        {"_", "_", WifiKeyboardAction::Text}, {"=", "=", WifiKeyboardAction::Text},
        {"/", "/", WifiKeyboardAction::Text}, {"\\", "\\", WifiKeyboardAction::Text},
        {":", ":", WifiKeyboardAction::Text}, {";", ";", WifiKeyboardAction::Text},
        {"'", "'", WifiKeyboardAction::Text}, {"\"", "\"", WifiKeyboardAction::Text},
        {"?", "?", WifiKeyboardAction::Text},
    };
    static const WifiKeyboardKey bottom_row[] = {
        {"空格", nullptr, WifiKeyboardAction::Space},
        {"DEL", nullptr, WifiKeyboardAction::Backspace},
        {"清空", nullptr, WifiKeyboardAction::Clear},
        {"确定", nullptr, WifiKeyboardAction::Confirm},
    };

    const WifiKeyboardKey *row0 = wifi_keyboard_layout == WifiKeyboardLayout::Lowercase
        ? lower_row0 : wifi_keyboard_layout == WifiKeyboardLayout::Uppercase
        ? upper_row0 : symbol_row0;
    const WifiKeyboardKey *row1 = wifi_keyboard_layout == WifiKeyboardLayout::Lowercase
        ? lower_row1 : wifi_keyboard_layout == WifiKeyboardLayout::Uppercase
        ? upper_row1 : symbol_row1;
    const WifiKeyboardKey *row2 = wifi_keyboard_layout == WifiKeyboardLayout::Lowercase
        ? lower_row2 : wifi_keyboard_layout == WifiKeyboardLayout::Uppercase
        ? upper_row2 : symbol_row2;
    add_wifi_keyboard_row(row0, 12, 5, 42, 58, 4, 6);
    add_wifi_keyboard_row(row1, 11, 55, 42, 63, 5, 4);
    add_wifi_keyboard_row(row2, 11, 105, 42, 63, 5, 4);

    // 底行保留大面积空格键、退格、清空和确认，便于手指直接操作。
    static const int BOTTOM_X[] = {6, 381, 456, 531};
    static const int BOTTOM_WIDTH[] = {370, 70, 70, 215};
    for (size_t i = 0; i < sizeof(bottom_row) / sizeof(bottom_row[0]); ++i) {
        const WifiKeyboardKey &key = bottom_row[i];
        const bool is_confirm = key.action == WifiKeyboardAction::Confirm;
        const bool is_clear = key.action == WifiKeyboardAction::Clear;
        lv_obj_t *button = make_gradient_button(
            wifi_keyboard, BOTTOM_X[i], 155, BOTTOM_WIDTH[i], 49,
            is_confirm ? 0x0D806F : is_clear ? 0x5A1830 : 0x0A2D4C,
            is_confirm ? 0x063C39 : is_clear ? 0x260B1B : 0x04162A,
            5, is_confirm ? 0x50FFD2 : is_clear ? 0xEF4A67 : UI_BORDER_DIM,
            1, is_confirm ? jcy_audio::Cue::Confirm : jcy_audio::Cue::Click);
        lv_obj_add_event_cb(button, wifi_keyboard_key_event, LV_EVENT_CLICKED,
                            const_cast<WifiKeyboardKey *>(&key));
        make_center_label(button, key.label, &lv_font_jcy16, 0xFFFFFF);
    }
}

void ble_toggle_clicked(lv_event_t *) {
    if (jcy_wireless::ble_is_enabled()) {
        jcy_wireless::ble_disable();
    } else {
        jcy_wireless::ble_enable();
    }
    refresh_wireless_ui();
}

void open_wifi_page() {
    lv_obj_t *page = create_page_shell("Wi-Fi 配置");

    wifi_status_label = make_label(page, "状态: 未启用", 24, 76,
                                   &lv_font_jcy18, 0xFFFFFF);
    wifi_info_label = make_label(page, "请选择网络并输入密码", 24, 108,
                                 &lv_font_jcy16, 0xD7F0FF);
    // 网络名、IP 和 RSSI 会随着路由器名称变长；限制在内容区内，避免
    // 右侧文字直接被屏幕裁掉而看起来像是“缺字”。
    lv_obj_set_width(wifi_info_label, 728);
    lv_label_set_long_mode(wifi_info_label, LV_LABEL_LONG_DOT);

    lv_obj_t *toggle = make_gradient_button(
        page, 24, 140, 128, 44, 0x0B4161, 0x051A2D,
        8, UI_CYAN, 1);
    lv_obj_add_event_cb(toggle, wifi_toggle_clicked,
                        LV_EVENT_CLICKED, nullptr);
    make_center_label(toggle, "Wi-Fi开关", &lv_font_jcy16, 0xFFFFFF);

    lv_obj_t *scan = make_gradient_button(
        page, 164, 140, 128, 44, 0x09243E, 0x041326,
        8, UI_BORDER_DIM, 1);
    lv_obj_add_event_cb(scan, wifi_scan_clicked,
                        LV_EVENT_CLICKED, nullptr);
    make_center_label(scan, "扫描网络", &lv_font_jcy16, 0xFFFFFF);

    lv_obj_t *quick_szd = make_gradient_button(
        page, 304, 140, 220, 44, 0x0D806F, 0x063C39,
        8, 0x50FFD2, 1);
    lv_obj_add_event_cb(quick_szd, wifi_quick_szd_clicked,
                        LV_EVENT_CLICKED, nullptr);
    make_center_label(quick_szd, "连接已保存网络", &lv_font_jcy16, 0xFFFFFF);

    wifi_scan_dropdown = lv_dropdown_create(page);
    lv_dropdown_set_options(wifi_scan_dropdown, "No networks");
    // 精简中文字体未内置 FontAwesome 的下拉箭头符号，使用 ASCII 备用
    // 标识，避免扫描列表入口本身显示为空白。
    lv_dropdown_set_symbol(wifi_scan_dropdown, "v");
    lv_obj_set_pos(wifi_scan_dropdown, 536, 140);
    lv_obj_set_size(wifi_scan_dropdown, 240, 44);
    lv_obj_set_style_text_font(wifi_scan_dropdown, &lv_font_jcy16, 0);
    lv_obj_set_style_bg_color(wifi_scan_dropdown, color(0x071A31), 0);
    lv_obj_set_style_text_color(wifi_scan_dropdown, color(0xFFFFFF), 0);
    lv_obj_set_style_border_color(wifi_scan_dropdown, color(UI_BORDER), 0);
    // 下拉列表不是下拉按钮的子对象，而是独立挂在 screen 上。若只配置
    // 按钮本身，展开后的 SSID/RSSI 会回退到主题默认字体和颜色，造成
    // 深色背景上局部文字不可见或显示为方框。
    lv_obj_t *wifi_scan_list = lv_dropdown_get_list(wifi_scan_dropdown);
    lv_obj_set_style_text_font(wifi_scan_list, &lv_font_jcy16, 0);
    lv_obj_set_style_text_color(wifi_scan_list, color(0xFFFFFF), 0);
    lv_obj_set_style_bg_color(wifi_scan_list, color(0x071A31), 0);
    lv_obj_set_style_bg_grad_color(wifi_scan_list, color(0x041224), 0);
    lv_obj_set_style_bg_grad_dir(wifi_scan_list, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_color(wifi_scan_list, color(UI_CYAN), 0);
    lv_obj_set_style_border_width(wifi_scan_list, 1, 0);
    lv_obj_set_style_text_font(wifi_scan_list, &lv_font_jcy16,
                               LV_PART_SELECTED);
    lv_obj_set_style_text_color(wifi_scan_list, color(0xFFFFFF),
                                LV_PART_SELECTED);
    lv_obj_set_style_bg_color(wifi_scan_list, color(0x0B5D81),
                              LV_PART_SELECTED);
    lv_obj_add_event_cb(wifi_scan_dropdown, wifi_dropdown_selected,
                        LV_EVENT_VALUE_CHANGED, nullptr);

    make_label(page, "SSID", 24, 208, &lv_font_montserrat_16, 0xEAF8FF);
    wifi_ssid_textarea = lv_textarea_create(page);
    lv_textarea_set_one_line(wifi_ssid_textarea, true);
    lv_textarea_set_placeholder_text(wifi_ssid_textarea, "Wi-Fi SSID");
    lv_obj_set_pos(wifi_ssid_textarea, 82, 196);
    lv_obj_set_size(wifi_ssid_textarea, 248, 44);
    lv_obj_set_style_bg_color(wifi_ssid_textarea, color(0x071A31), 0);
    lv_obj_set_style_text_font(wifi_ssid_textarea, &lv_font_jcy16, 0);
    lv_obj_set_style_text_color(wifi_ssid_textarea, color(0xFFFFFF), 0);
    lv_obj_set_style_text_font(wifi_ssid_textarea, &lv_font_jcy16,
                               LV_PART_TEXTAREA_PLACEHOLDER);
    lv_obj_set_style_text_color(wifi_ssid_textarea, color(0x8AA8C6),
                                LV_PART_TEXTAREA_PLACEHOLDER);
    lv_obj_set_style_border_color(wifi_ssid_textarea, color(UI_BORDER), 0);
    lv_obj_add_event_cb(wifi_ssid_textarea, wifi_textarea_selected,
                        LV_EVENT_FOCUSED, nullptr);
    lv_obj_add_event_cb(wifi_ssid_textarea, wifi_textarea_selected,
                        LV_EVENT_CLICKED, nullptr);

    make_label(page, "密码", 350, 208, &lv_font_jcy16, 0xEAF8FF);
    wifi_password_textarea = lv_textarea_create(page);
    lv_textarea_set_one_line(wifi_password_textarea, true);
    lv_textarea_set_password_mode(wifi_password_textarea, true);
    // 默认密码掩码是 U+2022，而精简的中文 UI 字库不包含该符号；密码
    // 实际已输入却显示为空白。改为字库明确包含的 ASCII 星号。
    lv_textarea_set_password_bullet(wifi_password_textarea, "*");
    lv_textarea_set_password_show_time(wifi_password_textarea, 0);
    lv_textarea_set_placeholder_text(wifi_password_textarea,
                                     "选择网络后输入密码");
    lv_obj_set_pos(wifi_password_textarea, 410, 196);
    lv_obj_set_size(wifi_password_textarea, 200, 44);
    lv_obj_set_style_bg_color(wifi_password_textarea, color(0x071A31), 0);
    lv_obj_set_style_text_font(wifi_password_textarea, &lv_font_jcy16, 0);
    lv_obj_set_style_text_color(wifi_password_textarea, color(0xFFFFFF), 0);
    lv_obj_set_style_text_font(wifi_password_textarea, &lv_font_jcy16,
                               LV_PART_TEXTAREA_PLACEHOLDER);
    lv_obj_set_style_text_color(wifi_password_textarea, color(0x8AA8C6),
                                LV_PART_TEXTAREA_PLACEHOLDER);
    lv_obj_set_style_border_color(wifi_password_textarea, color(UI_BORDER), 0);
    lv_obj_add_event_cb(wifi_password_textarea, wifi_textarea_selected,
                        LV_EVENT_FOCUSED, nullptr);
    lv_obj_add_event_cb(wifi_password_textarea, wifi_textarea_selected,
                        LV_EVENT_CLICKED, nullptr);

    lv_obj_t *connect = make_gradient_button(
        page, 622, 196, 154, 44, 0x0D806F, 0x063C39,
        8, 0x50FFD2, 1);
    lv_obj_add_event_cb(connect, wifi_connect_clicked,
                        LV_EVENT_CLICKED, nullptr);
    wifi_connect_button_label = make_center_label(
        connect, "保存并连接", &lv_font_jcy16, 0xFFFFFF);

    // 标准 lv_keyboard 在本工程的无主题配置下虽然创建成功，但按键矩阵
    // 不参与绘制。这里使用普通 LVGL 按钮构成键盘，显示与触控均复用
    // 已验证的仪器 UI 控件，支持字母、数字、空格与常用特殊符号。
    wifi_keyboard = lv_obj_create(page);
    lv_obj_set_pos(wifi_keyboard, 24, 252);
    lv_obj_set_size(wifi_keyboard, 752, 210);
    style_clean(wifi_keyboard);
    lv_obj_set_style_bg_color(wifi_keyboard, color(0x041224), 0);
    lv_obj_set_style_bg_opa(wifi_keyboard, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(wifi_keyboard, color(UI_BORDER_DIM), 0);
    lv_obj_set_style_border_width(wifi_keyboard, 1, 0);
    lv_obj_set_style_radius(wifi_keyboard, 8, 0);
    wifi_keyboard_target = wifi_password_textarea;
    lv_obj_add_state(wifi_keyboard_target, LV_STATE_FOCUSED);
    create_wifi_keyboard_layout();

    char saved_ssid[33] = {};
    char saved_password[65] = {};
    jcy_wireless::wifi_load_credentials(saved_ssid, sizeof(saved_ssid),
                                       saved_password, sizeof(saved_password));
    if (saved_ssid[0] != '\0') {
        lv_textarea_set_text(wifi_ssid_textarea, saved_ssid);
        lv_textarea_set_text(wifi_password_textarea, saved_password);
    }
    refresh_wireless_ui();
}

void open_ble_page() {
    lv_obj_t *page = create_page_shell("蓝牙 BLE");

    lv_obj_t *toggle = make_gradient_button(
        page, 24, 86, 200, 58, 0x0B4161, 0x051A2D,
        8, UI_CYAN, 1);
    lv_obj_add_event_cb(toggle, ble_toggle_clicked,
                        LV_EVENT_CLICKED, nullptr);
    ble_toggle_button_label = make_center_label(
        toggle, "开启 BLE", &lv_font_jcy18, 0xFFFFFF);

    ble_status_label = make_label(
        page, "状态: 未启用   TX:0  RX:0", 24, 166,
        &lv_font_jcy18, 0xFFFFFF);
    ble_info_label = make_label(
        page, "设备: " JCY_PRODUCT_NAME " BLE\n手机扫描该名称即可连接",
        24, 208, &lv_font_jcy18, 0xD8F3FF);
    lv_obj_set_style_text_line_space(ble_info_label, 10, 0);

    lv_obj_t *panel = make_gradient_box(
        page, 24, 284, 752, 168,
        0x0B2D4B, 0x041224, 8, UI_BORDER_DIM, 1);
    lv_obj_t *note = make_label(
        panel,
        "ESP32-S3 使用 BLE，不支持经典蓝牙 SPP。\n"
        "已创建类似无线串口的服务：手机可写入 RX，并通过 TX 通知接收数据。\n"
        "可用 nRF Connect / LightBlue 等 BLE 工具连接测试。",
        18, 16, &lv_font_jcy16, 0xF2FBFF);
    lv_obj_set_width(note, 716);
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_line_space(note, 9, 0);
    refresh_wireless_ui();
}

void cloud_ota_check_event(lv_event_t *) {
    if (WiFi.status() != WL_CONNECTED) {
        show_info_dialog("无法检查", "请先在 Wi-Fi 页面连接网络。");
        return;
    }
    if (!jcy_cloud_ota::start_check()) {
        show_info_dialog("请稍候", "云端升级任务正在运行。");
        return;
    }
    refresh_cloud_ota_ui();
}

void show_ota_transfer_guard() {
    if (ota_transfer_guard != nullptr) {
        return;
    }
    // OTA 写 Flash 时固定在独立、低复杂度页面；不允许用户误返回仪器
    // 主界面，也避免主界面波形和动画在升级期间产生大面积刷新。
    ota_transfer_guard = lv_obj_create(lv_layer_top());
    lv_obj_set_pos(ota_transfer_guard, 0, 0);
    lv_obj_set_size(ota_transfer_guard, SCREEN_WIDTH, SCREEN_HEIGHT);
    style_clean(ota_transfer_guard);
    lv_obj_set_style_bg_color(ota_transfer_guard, color(0x01070F), 0);
    lv_obj_set_style_bg_opa(ota_transfer_guard, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(ota_transfer_guard, 0, 0);
    lv_obj_add_flag(ota_transfer_guard, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *panel = make_gradient_box(
        ota_transfer_guard, 86, 90, 628, 300,
        0x082844, 0x02101F, 14, 0x197FA1, 1);
    lv_obj_t *title = make_label(
        panel, "正在在线升级", 0, 20, &lv_font_jcy20, 0xFFFFFF);
    lv_obj_set_width(title, 628);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);

    ota_transfer_stage_label = make_label(
        panel, "升级准备中，请勿断电", 0, 64,
        &lv_font_jcy20, 0xFFE39A);
    lv_obj_set_width(ota_transfer_stage_label, 628);
    lv_obj_set_style_text_align(ota_transfer_stage_label,
                                LV_TEXT_ALIGN_CENTER, 0);

    ota_transfer_detail_label = make_label(
        panel, "即将下载固件并安全写入系统分区", 0, 106,
        &lv_font_jcy16, 0xBAE8FA);
    lv_obj_set_width(ota_transfer_detail_label, 628);
    lv_obj_set_style_text_align(ota_transfer_detail_label,
                                LV_TEXT_ALIGN_CENTER, 0);

    ota_transfer_progress_bar = lv_bar_create(panel);
    lv_obj_set_pos(ota_transfer_progress_bar, 54, 148);
    lv_obj_set_size(ota_transfer_progress_bar, 520, 24);
    lv_bar_set_range(ota_transfer_progress_bar, 0, 100);
    lv_bar_set_value(ota_transfer_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(ota_transfer_progress_bar, color(0x173F70), 0);
    lv_obj_set_style_border_color(ota_transfer_progress_bar,
                                  color(0x3C7CC4), 0);
    lv_obj_set_style_border_width(ota_transfer_progress_bar, 1, 0);
    lv_obj_set_style_radius(ota_transfer_progress_bar, 6, 0);

    ota_transfer_progress_label = make_label(
        panel, "进度 0%", 0, 186, &lv_font_jcy18, 0x67D7D2);
    lv_obj_set_width(ota_transfer_progress_label, 628);
    lv_obj_set_style_text_align(ota_transfer_progress_label,
                                LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *hint = make_label(
        panel, "升级完成后设备将自动重启，请勿关闭电源", 0, 224,
        &lv_font_jcy16, 0xFFE39A);
    lv_obj_set_width(hint, 628);
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, 0);

    ota_transfer_return_button = make_gradient_button(
        panel, 208, 250, 212, 42, 0xA63D4B, 0x671D30, 8, 0xFFB1B8, 1,
        jcy_audio::Cue::Click);
    make_center_label(ota_transfer_return_button, "返回升级页面",
                      &lv_font_jcy16, 0xFFFFFF);
    lv_obj_add_event_cb(ota_transfer_return_button, ota_transfer_return_event,
                        LV_EVENT_CLICKED, nullptr);
    lv_obj_add_flag(ota_transfer_return_button, LV_OBJ_FLAG_HIDDEN);

    ota_transfer_last_state = JcyCloudOtaState::Idle;
    ota_transfer_last_percent = 0xFF;
    digitalWrite(BACKLIGHT_PIN, HIGH);
}

void hide_ota_transfer_guard() {
    if (ota_transfer_guard != nullptr) {
        lv_obj_del(ota_transfer_guard);
        ota_transfer_guard = nullptr;
    }
    ota_transfer_stage_label = nullptr;
    ota_transfer_detail_label = nullptr;
    ota_transfer_progress_label = nullptr;
    ota_transfer_progress_bar = nullptr;
    ota_transfer_return_button = nullptr;
    ota_transfer_last_state = JcyCloudOtaState::Idle;
    ota_transfer_last_percent = 0xFF;
}

void ota_transfer_return_event(lv_event_t *) {
    hide_ota_transfer_guard();
    lower_ota_guard_active = false;
    refresh_cloud_ota_ui();
    refresh_lower_ota_ui();
}

void refresh_ota_transfer_guard() {
    if (ota_transfer_guard == nullptr) {
        return;
    }
    if (lower_ota_guard_active) {
        const JcyLowerMcuOtaStatus &lower = jcy_lower_mcu_ota::status();
        const bool busy = lower_ota_should_be_quiet();
        const bool success = lower.state == JcyLowerMcuOtaState::Success;
        const bool error = lower.state == JcyLowerMcuOtaState::Error;
        const uint8_t display_percent = lower.percent;
        if (ota_transfer_stage_label != nullptr) {
            set_label_text_if_changed(
                ota_transfer_stage_label,
                success ? "底层控制器升级完成" :
                error ? "底层控制器升级未完成" :
                        "正在升级底层控制器，请勿断电");
            lv_obj_set_style_text_color(
                ota_transfer_stage_label,
                color(success ? 0x77F4C6 : error ? 0xFF9DA5 : 0x67D7D2), 0);
        }
        if (ota_transfer_detail_label != nullptr)
            set_label_text_if_changed(ota_transfer_detail_label,
                                      lower.message);
        if (ota_transfer_progress_label != nullptr) {
            char progress[32] = {};
            snprintf(progress, sizeof(progress), "进度 %u%%",
                     static_cast<unsigned>(display_percent));
            set_label_text_if_changed(ota_transfer_progress_label, progress);
        }
        if (ota_transfer_progress_bar != nullptr &&
            lv_bar_get_value(ota_transfer_progress_bar) != display_percent)
            lv_bar_set_value(ota_transfer_progress_bar, display_percent,
                             LV_ANIM_OFF);
        if (ota_transfer_return_button != nullptr) {
            if (!busy && (success || error))
                lv_obj_clear_flag(ota_transfer_return_button,
                                  LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_add_flag(ota_transfer_return_button,
                                LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }
    const JcyCloudOtaStatus &st = jcy_cloud_ota::status();
    const JcyCloudOtaState state = st.state;
    // ESP32-S3 在写 Flash 时即使只重绘一小块进度条，也可能和 RGB DMA
    // 同时争用 PSRAM/Cache，实机摄像头会抓到短暂的重复扫描画面。下载阶段
    // 因此只在开始前画一次静态状态，不再跟随真实百分比刷新；精确进度仍
    // 保留在串口日志中。写入结束后才更新“校验/完成”阶段。
    uint8_t display_percent = 0;
    if (state == JcyCloudOtaState::Downloading) {
        display_percent = 35;
    } else if (state == JcyCloudOtaState::Verifying) {
        display_percent = 90;
    } else if (state == JcyCloudOtaState::Success) {
        display_percent = 100;
    } else if (state == JcyCloudOtaState::Error &&
               ota_transfer_last_percent != 0xFF) {
        display_percent = ota_transfer_last_percent;
    }
    if (state == ota_transfer_last_state &&
        display_percent == ota_transfer_last_percent) {
        return;
    }
    ota_transfer_last_state = state;
    ota_transfer_last_percent = display_percent;

    const char *stage = "升级准备中，请勿断电";
    const char *detail = "即将下载固件并安全写入系统分区";
    uint32_t stage_color = 0xFFE39A;
    bool show_return = false;
    if (state == JcyCloudOtaState::Downloading) {
        stage = "正在下载并写入固件，请勿断电";
        detail = "下载与写入会持续一段时间，请保持稳定供电";
        stage_color = 0x67D7D2;
    } else if (state == JcyCloudOtaState::Verifying) {
        stage = "正在校验固件完整性";
        detail = "校验通过后将切换到新版本并自动重启";
        stage_color = 0x67D7D2;
    } else if (state == JcyCloudOtaState::Success) {
        stage = "升级完成，设备即将自动重启";
        detail = "新固件已校验通过，请继续保持供电";
        stage_color = 0x77F4C6;
    } else if (state == JcyCloudOtaState::Error) {
        stage = "升级未完成";
        detail = "未切换启动分区，设备仍可继续使用当前固件";
        stage_color = 0xFF9DA5;
        show_return = true;
    }
    if (ota_transfer_stage_label != nullptr) {
        set_label_text_if_changed(ota_transfer_stage_label, stage);
        lv_obj_set_style_text_color(ota_transfer_stage_label,
                                    color(stage_color), 0);
    }
    if (ota_transfer_detail_label != nullptr) {
        set_label_text_if_changed(ota_transfer_detail_label, detail);
    }
    if (ota_transfer_progress_label != nullptr) {
        const char *progress_text = "准备升级";
        if (state == JcyCloudOtaState::Downloading) {
            progress_text = "升级进行中";
        } else if (state == JcyCloudOtaState::Verifying) {
            progress_text = "正在校验";
        } else if (state == JcyCloudOtaState::Success) {
            progress_text = "进度 100%";
        } else if (state == JcyCloudOtaState::Error) {
            progress_text = "升级已停止";
        }
        set_label_text_if_changed(ota_transfer_progress_label, progress_text);
    }
    if (ota_transfer_progress_bar != nullptr &&
        lv_bar_get_value(ota_transfer_progress_bar) != display_percent) {
        lv_bar_set_value(ota_transfer_progress_bar, display_percent,
                         LV_ANIM_OFF);
    }
    if (ota_transfer_return_button != nullptr) {
        if (show_return) {
            lv_obj_clear_flag(ota_transfer_return_button, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(ota_transfer_return_button, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void cloud_ota_download_event(lv_event_t *) {
    if (output_running || requested_output_state || output_change_pending ||
        boot_safe_output_off_active) {
        show_info_dialog("禁止升级", "请先关闭仪器输出并等待状态确认。");
        return;
    }
    // 必须在工作任务开始前建立静态页；云端任务会留出渲染窗口后才真正写Flash。
    show_ota_transfer_guard();
    if (!jcy_cloud_ota::start_download()) {
        hide_ota_transfer_guard();
        show_info_dialog("无法升级", "请先检查云端版本，或等待当前任务完成。");
        return;
    }
    refresh_cloud_ota_ui();
}

void lower_ota_check_event(lv_event_t *) {
    if (WiFi.status() != WL_CONNECTED) {
        show_info_dialog("无法检查", "请先在 Wi-Fi 页面连接网络。");
        return;
    }
    if (!scpi_link.identity().valid) {
        show_info_dialog("无法检查", "尚未读取到底层控制器版本，请先确认 TTL 通讯。");
        return;
    }
    if (!jcy_lower_mcu_ota::start_check()) {
        show_info_dialog("请稍等", "底层控制器升级任务正在运行。");
        return;
    }
    refresh_lower_ota_ui();
}

void lower_ota_upgrade_event(lv_event_t *) {
    if (!output_is_confirmed_off() || cutoff_running) {
        show_info_dialog("禁止升级", "请先关闭输出并停止截止电压测试。");
        return;
    }
    lower_ota_guard_active = true;
    if (!jcy_lower_mcu_ota::start_upgrade()) {
        lower_ota_guard_active = false;
        show_info_dialog("无法升级", "请先检查底层新版本，或等待当前任务完成。");
        return;
    }
    show_ota_transfer_guard();
    if (ota_transfer_stage_label != nullptr)
        set_label_text_if_changed(ota_transfer_stage_label,
                                  "正在升级底层控制器，请勿断电");
    if (ota_transfer_detail_label != nullptr)
        set_label_text_if_changed(ota_transfer_detail_label,
                                  "ESP32 将通过 TTL 安全写入 STM32 固件");
    refresh_lower_ota_ui();
}

void lower_ota_test_mode_event(lv_event_t *) {
    if (!output_is_confirmed_off() || cutoff_running) {
        show_info_dialog("禁止进入测试模式",
                         "请先关闭输出并停止截止电压测试。");
        return;
    }
    const bool enabled = !jcy_lower_mcu_ota::same_version_test_enabled();
    jcy_lower_mcu_ota::set_same_version_test(enabled);
    show_info_dialog(
        enabled ? "底层升级测试模式" : "测试模式已关闭",
        enabled
            ? "已允许一次同版本重刷。仍禁止降级，升级后自动退出测试模式。请重新检查底层版本。"
            : "同版本重刷已取消。");
    refresh_lower_ota_ui();
}

void open_ota_page() {
    lv_obj_t *page = create_page_shell("系统升级 OTA");

    lv_obj_t *cloud_box = make_gradient_box(
        page, 20, 58, 760, 142,
        0x0B3152, 0x041728, 10, 0x27BFE2, 1);
    make_label(cloud_box, "官网在线升级", 18, 12,
               &lv_font_jcy20, 0xFFFFFF);
    make_label(cloud_box, "www.jcytest.com", 175, 16,
               &lv_font_montserrat_16, 0x69DCF2);

    cloud_ota_remote_label = make_label(
        cloud_box, "状态: 等待检查", 18, 43,
        &lv_font_jcy18, 0xFFFFFF);
    cloud_ota_version_label = make_label(
        cloud_box, "当前版本: --    云端版本: --", 18, 72,
        &lv_font_jcy16, 0xD8F3FF);
    cloud_ota_notes_label = make_label(
        cloud_box, "说明: 点击检查云端版本", 18, 101,
        &lv_font_jcy16, 0xA9D5EA);
    lv_obj_set_width(cloud_ota_notes_label, 500);
    lv_label_set_long_mode(cloud_ota_notes_label, LV_LABEL_LONG_DOT);

    cloud_ota_check_button = make_gradient_button(
        cloud_box, 565, 25, 176, 44,
        0x187FD3, 0x0B4A8A, 8, 0x6DDEFF, 1,
        jcy_audio::Cue::Click);
    lv_obj_add_event_cb(cloud_ota_check_button,
                        cloud_ota_check_event, LV_EVENT_CLICKED, nullptr);
    make_center_label(cloud_ota_check_button, "检查更新",
                      &lv_font_jcy18, 0xFFFFFF);

    cloud_ota_download_button = make_gradient_button(
        cloud_box, 565, 79, 176, 44,
        0x169D7C, 0x075947, 8, 0x72F5CF, 1,
        jcy_audio::Cue::Confirm);
    lv_obj_add_event_cb(cloud_ota_download_button,
                        cloud_ota_download_event, LV_EVENT_CLICKED, nullptr);
    make_center_label(cloud_ota_download_button, "下载并升级",
                      &lv_font_jcy18, 0xFFFFFF);

    cloud_ota_progress_bar = lv_bar_create(cloud_box);
    lv_obj_set_pos(cloud_ota_progress_bar, 18, 127);
    lv_obj_set_size(cloud_ota_progress_bar, 723, 10);
    lv_bar_set_value(cloud_ota_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(cloud_ota_progress_bar, color(0x173F70), 0);
    lv_obj_set_style_border_color(cloud_ota_progress_bar,
                                  color(0x3C7CC4), 0);
    lv_obj_set_style_border_width(cloud_ota_progress_bar, 1, 0);
    lv_obj_set_style_radius(cloud_ota_progress_bar, 5, 0);

    lv_obj_t *lower_box = make_gradient_box(
        page, 20, 210, 760, 154,
        0x17304D, 0x061426, 10, 0x38C7A1, 1);
    lv_obj_t *lower_title = make_label(
        lower_box, "底层控制器升级", 18, 10,
        &lv_font_jcy18, 0xFFFFFF);
    lv_obj_add_flag(lower_title, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lower_title, lower_ota_test_mode_event,
                        LV_EVENT_LONG_PRESSED, nullptr);
    make_label(lower_box, "STM32F103 / TTL IAP", 184, 14,
               &lv_font_montserrat_14, 0x6DE3C4);
    lower_ota_status_label = make_label(
        lower_box, "状态: 等待检查", 18, 42,
        &lv_font_jcy16, 0xFFFFFF);
    lower_ota_version_label = make_label(
        lower_box, "当前版本: --    云端版本: --", 18, 69,
        &lv_font_jcy16, 0xD8F3FF);
    lower_ota_notes_label = make_label(
        lower_box, "说明: 长按此处进入同版本升级测试模式", 18, 96,
        &lv_font_jcy16, 0xA9D5EA);
    lv_obj_set_width(lower_ota_notes_label, 520);
    lv_label_set_long_mode(lower_ota_notes_label, LV_LABEL_LONG_DOT);
    lower_ota_check_button = make_gradient_button(
        lower_box, 565, 24, 176, 44,
        0x187FD3, 0x0B4A8A, 8, 0x6DDEFF, 1,
        jcy_audio::Cue::Click);
    lv_obj_add_event_cb(lower_ota_check_button, lower_ota_check_event,
                        LV_EVENT_CLICKED, nullptr);
    make_center_label(lower_ota_check_button, "检查底层版本",
                      &lv_font_jcy16, 0xFFFFFF);
    lower_ota_upgrade_button = make_gradient_button(
        lower_box, 565, 78, 176, 44,
        0x169D7C, 0x075947, 8, 0x72F5CF, 1,
        jcy_audio::Cue::Confirm);
    lv_obj_add_event_cb(lower_ota_upgrade_button, lower_ota_upgrade_event,
                        LV_EVENT_CLICKED, nullptr);
    make_center_label(lower_ota_upgrade_button, "升级底层控制器",
                      &lv_font_jcy16, 0xFFFFFF);
    lower_ota_progress_bar = lv_bar_create(lower_box);
    lv_obj_set_pos(lower_ota_progress_bar, 18, 132);
    lv_obj_set_size(lower_ota_progress_bar, 723, 10);
    lv_bar_set_value(lower_ota_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(lower_ota_progress_bar, color(0x173F70), 0);
    lv_obj_set_style_border_color(lower_ota_progress_bar, color(0x38C7A1), 0);
    lv_obj_set_style_border_width(lower_ota_progress_bar, 1, 0);

    lv_obj_t *local_box = make_gradient_box(
        page, 20, 374, 760, 58,
        0x092641, 0x031224, 10, 0x345E86, 1);
    make_label(local_box, "局域网备用升级", 18, 8,
               &lv_font_jcy16, 0xFFFFFF);
    ota_status_label = make_label(
        local_box, "状态: 等待 Wi-Fi", 190, 8,
        &lv_font_jcy16, 0xCFEAFA);
    ota_ip_label = make_label(
        local_box, "尚未连接 Wi-Fi", 430, 8,
        &lv_font_montserrat_16, 0xFFFFFF);
    ota_info_label = make_label(
        local_box,
        "浏览器访问本机 IP 上传 .bin   用户名 jcy / 密码 jcy1000",
        18, 34, &lv_font_jcy16, 0x8FB4CC);
    lv_obj_set_width(ota_info_label, 720);

    make_label(page,
               "升级前必须关闭输出并保持稳定 5V 供电；完成后设备自动重启。",
               20, 438, &lv_font_jcy16, 0xFFE9A8);

    refresh_ota_ui();
    refresh_cloud_ota_ui();
    refresh_lower_ota_ui();
}

void refresh_cloud_ota_ui() {
    const JcyCloudOtaStatus &st = jcy_cloud_ota::status();
    // 传输保护页位于顶层。升级期间只刷新该页，禁止继续修改被遮挡的
    // OTA 页面对象，否则后台脏区仍会参与绘制并放大 Flash 写入时的闪烁。
    if (ota_transfer_guard != nullptr) {
        refresh_ota_transfer_guard();
        return;
    }
    if (cloud_ota_remote_label != nullptr) {
        char text[104] = {};
        snprintf(text, sizeof(text), "状态: %s", st.message);
        set_label_text_if_changed(cloud_ota_remote_label, text);
    }
    if (cloud_ota_version_label != nullptr) {
        char text[72] = {};
        snprintf(text, sizeof(text), "当前版本: %s    云端版本: %s",
                 st.current_version[0] ? st.current_version : "--",
                 st.latest_version[0] ? st.latest_version : "--");
        set_label_text_if_changed(cloud_ota_version_label, text);
    }
    if (cloud_ota_notes_label != nullptr) {
        char text[112] = {};
        snprintf(text, sizeof(text), "说明: %s",
                 st.notes[0] ? st.notes : "点击检查云端版本");
        set_label_text_if_changed(cloud_ota_notes_label, text);
    }
    if (cloud_ota_progress_bar != nullptr) {
        // OTA 的进度只需要离散显示。连续动画会在 Flash 擦写期间持续
        // 生成大脏区，是物理屏幕抖动的主要软件放大因素之一。
        const uint8_t rounded_percent =
            st.state == JcyCloudOtaState::Downloading
                ? static_cast<uint8_t>((st.percent / 10U) * 10U)
                : st.percent;
        if (lv_bar_get_value(cloud_ota_progress_bar) != rounded_percent) {
            lv_bar_set_value(cloud_ota_progress_bar, rounded_percent,
                             LV_ANIM_OFF);
        }
    }
    if (cloud_ota_check_button != nullptr) {
        const bool busy = st.state == JcyCloudOtaState::Checking ||
                          st.state == JcyCloudOtaState::Downloading ||
                          st.state == JcyCloudOtaState::Verifying;
        set_button_disabled_if_changed(cloud_ota_check_button, busy);
    }
    if (cloud_ota_download_button != nullptr) {
        const bool enabled = st.state == JcyCloudOtaState::UpdateAvailable &&
                             st.update_allowed;
        set_button_disabled_if_changed(cloud_ota_download_button, !enabled);
    }
    refresh_ota_transfer_guard();
}

void refresh_lower_ota_ui() {
    const JcyLowerMcuOtaStatus &st = jcy_lower_mcu_ota::status();
    if (lower_ota_guard_active && ota_transfer_guard != nullptr) {
        refresh_ota_transfer_guard();
        return;
    }
    if (lower_ota_status_label != nullptr) {
        char text[120] = {};
        snprintf(text, sizeof(text), "状态: %s", st.message);
        set_label_text_if_changed(lower_ota_status_label, text);
    }
    if (lower_ota_version_label != nullptr) {
        char text[104] = {};
        snprintf(text, sizeof(text), "当前版本: %s    云端版本: %s",
                 st.current_version[0] ? st.current_version : "--",
                 st.latest_version[0] ? st.latest_version : "--");
        set_label_text_if_changed(lower_ota_version_label, text);
    }
    if (lower_ota_notes_label != nullptr) {
        char text[128] = {};
        snprintf(text, sizeof(text), "说明: %s",
                 st.notes[0] ? st.notes
                             : (st.same_version_test
                                    ? "测试模式已开启：允许同版本重刷一次"
                                    : "长按此处进入同版本升级测试模式"));
        set_label_text_if_changed(lower_ota_notes_label, text);
    }
    if (lower_ota_progress_bar != nullptr &&
        lv_bar_get_value(lower_ota_progress_bar) != st.percent)
        lv_bar_set_value(lower_ota_progress_bar, st.percent, LV_ANIM_OFF);
    const bool busy = st.state == JcyLowerMcuOtaState::Checking ||
                      lower_ota_should_be_quiet();
    set_button_disabled_if_changed(lower_ota_check_button, busy);
    const bool can_upgrade =
        st.state == JcyLowerMcuOtaState::UpdateAvailable &&
        st.has_update && st.update_allowed;
    set_button_disabled_if_changed(lower_ota_upgrade_button, !can_upgrade);
}

void refresh_ota_ui() {
    const JcyOtaStatus &st = jcy_ota::status();
    if (ota_status_label != nullptr) {
        char text[96] = {};
        snprintf(text, sizeof(text), "状态: %s", st.message);
        set_label_text_if_changed(ota_status_label, text);
    }
    if (ota_ip_label != nullptr) {
        char text[48] = {};
        if (st.ip[0] == '\0') {
            snprintf(text, sizeof(text), "%s", "尚未连接 Wi-Fi");
        } else {
            snprintf(text, sizeof(text), "http://%s/", st.ip);
        }
        set_label_text_if_changed(ota_ip_label, text);
    }
}

void refresh_communication_mode_page() {
    const bool passthrough =
        communication_mode == CommunicationMode::Passthrough;
    auto refresh_mode_button = [](lv_obj_t *button, lv_obj_t *label,
                                  bool active) {
        if (button == nullptr || label == nullptr) {
            return;
        }
        lv_obj_set_style_bg_color(
            button, color(active ? 0x0D748D : 0x071A31), 0);
        lv_obj_set_style_bg_grad_color(
            button, color(active ? 0x073A51 : 0x030D1B), 0);
        lv_obj_set_style_border_color(
            button, color(active ? UI_CYAN : UI_BORDER_DIM), 0);
        lv_obj_set_style_border_width(button, active ? 2 : 1, 0);
        lv_obj_set_style_text_color(
            label, color(active ? 0xFFFFFF : UI_MUTED), 0);
    };
    refresh_mode_button(communication_controller_button,
                        communication_controller_label,
                        !passthrough);
    refresh_mode_button(communication_passthrough_button,
                        communication_passthrough_label,
                        passthrough);

    if (communication_status_label != nullptr) {
        lv_label_set_text(
            communication_status_label,
            passthrough ? "当前：串口透传" : "当前：本机控制");
        lv_obj_set_style_text_color(
            communication_status_label,
            color(passthrough ? 0x73FFD8 : UI_CYAN), 0);
    }
    if (communication_detail_label != nullptr) {
        lv_label_set_text(
            communication_detail_label,
            passthrough
                ? "内部轮询已暂停，IO10/IO11数据直接转发到底层IO12/IO13。\n"
                  "发送 +++JCYEXIT+++ 或点击本机控制即可退出。\n"
                  "退出透传时会自动发送 OUT OFF，恢复ESP32控制。"
                : "ESP32正在轮询并控制底层设备。\n"
                  "进入透传前必须关闭输出，且不能运行截止电压测试。\n"
                  "透传口：IO10=TX、IO11=RX，115200 baud / 8N1。");
    }
}

void communication_controller_event(lv_event_t *) {
    set_communication_mode(CommunicationMode::Controller, true);
}

void communication_passthrough_event(lv_event_t *) {
    set_communication_mode(CommunicationMode::Passthrough, true);
}

void create_communication_page() {
    lv_obj_t *page = create_page_shell("通讯模式");

    lv_obj_t *link_card = make_gradient_box(
        page, 24, 78, 752, 86,
        0x0B2D4B, 0x041224, 8, UI_BORDER_DIM, 1);
    make_label(link_card, "透传 TX10 / RX11", 24, 14,
               &lv_font_montserrat_18, UI_CYAN);
    make_label(link_card, "<->  ESP32-S3  <->", 210, 14,
               &lv_font_montserrat_18, 0xFFFFFF);
    make_label(link_card, "TTL TX12 / RX13", 490, 14,
               &lv_font_montserrat_18, UI_CYAN);
    make_label(link_card,
               "两路串口均为115200 8N1；TF卡接口改作透传，GPIO17/18用于喇叭",
               24, 51, &lv_font_jcy16, 0xCBE8F8);

    communication_controller_button = make_gradient_button(
        page, 24, 180, 360, 116,
        0x0D748D, 0x073A51, 10, UI_CYAN, 2);
    lv_obj_add_event_cb(communication_controller_button,
                        communication_controller_event,
                        LV_EVENT_CLICKED, nullptr);
    communication_controller_label = make_label(
        communication_controller_button, "本机控制", 24, 20,
        &lv_font_jcy20, 0xFFFFFF);
    make_label(communication_controller_button,
               "ESP32界面轮询、显示并控制底层",
               24, 66, &lv_font_jcy16, 0xCBE8F8);

    communication_passthrough_button = make_gradient_button(
        page, 416, 180, 360, 116,
        0x071A31, 0x030D1B, 10, UI_BORDER_DIM, 1);
    lv_obj_add_event_cb(communication_passthrough_button,
                        communication_passthrough_event,
                        LV_EVENT_CLICKED, nullptr);
    communication_passthrough_label = make_label(
        communication_passthrough_button, "串口透传", 24, 20,
        &lv_font_jcy20, UI_MUTED);
    make_label(communication_passthrough_button,
               "IO10/IO11数据直接转发到底层TTL",
               24, 66, &lv_font_jcy16, 0xCBE8F8);

    lv_obj_t *status_card = make_gradient_box(
        page, 24, 312, 752, 138,
        0x071D35, 0x030D1B, 8, UI_BORDER_DIM, 1);
    communication_status_label = make_label(
        status_card, "当前：本机控制", 24, 15,
        &lv_font_jcy20, UI_CYAN);
    communication_detail_label = make_label(
        status_card, "", 24, 51, &lv_font_jcy16, 0xEAF8FF);
    lv_obj_set_width(communication_detail_label, 704);
    lv_label_set_long_mode(communication_detail_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_line_space(communication_detail_label, 6, 0);
    refresh_communication_mode_page();
}


void create_settings_hub_page() {
    lv_obj_t *page = create_page_shell("系统设置");
    create_hub_button(page, 24, 72,
                      "模式切换", "模拟电池/双向电源",
                      HubAction::ModeSwitch);
    create_hub_button(page, 282, 72,
                      "截止测试", "充电与放电保护测试",
                      HubAction::Cutoff);
    create_hub_button(page, 540, 72,
                      "文件管理", "导出记录与升级",
                      HubAction::FileManager);
    create_hub_button(page, 24, 158,
                      "直流阻抗", "电芯直流内阻测试",
                      HubAction::DcImpedance);
    create_hub_button(page, 282, 158,
                      "购买与手册", "查看帮助和使用说明",
                      HubAction::Manual);
    create_hub_button(page, 540, 158,
                      "网络连接", "Wi-Fi与云平台",
                      HubAction::Network);
    create_hub_button(page, 24, 244,
                      "系统升级", "Wi-Fi OTA / 局域网更新",
                      HubAction::Ota);
    create_hub_button(page, 282, 244,
                      "BLE 蓝牙", "手机 BLE 串口",
                      HubAction::Bluetooth);
    create_hub_button(page, 540, 244,
                      "通讯模式", "本机控制 / 串口透传",
                      HubAction::Communication);
    create_hub_button(page, 282, 330,
                      "关于", "版本与设备信息",
                      HubAction::About);
}

void create_help_page() {
    lv_obj_t *page = create_page_shell("界面帮助");
    lv_obj_t *body = make_gradient_box(
        page, 24, 82, 752, 360,
        0x0B2D4B, 0x041224, 8, UI_BORDER_DIM, 1);
    const char *help_text =
        "1. 模拟电池/双向电源：切换工作模式。\n"
        "2. 电流模式：在过流保护与恒流电池模式之间切换，需底层确认。\n"
        "3. 电池设置：设置节数、电压范围、过流、滤波和内阻。\n"
        "4. 截止电压测试：选择过充或过放，自动RAMP并抓取保护点。\n"
        "5. 设置电压：点中间数值直接输入，加减键按当前选中位微调。\n"
        "6. 输出开启后：右侧显示电压/电流双曲线，底部仍可选位微调电压。\n"
        "7. 长按电压电流区域2秒：放大显示实时测量值。\n"
        "说明：截止测试启动前必须关闭普通输出；异常、停止或超时会发送OUT OFF。";
    lv_obj_t *label = make_label(
        body, help_text, 28, 24, &lv_font_jcy18, 0xF4FBFF);
    lv_obj_set_width(label, 696);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_line_space(label, 11, 0);
}

void open_page(PageKind page) {
    if (page_requires_output_off(page) &&
        !require_output_off_for_action(page_output_gate_action(page))) {
        return;
    }
    current_page = page;
    // 新页面会新建实时数字对象；强制仅补一次当前测量值，之后继续按实际
    // 数据变化刷新，避免待机时恢复 350ms 的无效帧交换。
    invalidate_idle_render_cache();
    switch (page) {
        case PageKind::BatterySettings:
            create_battery_settings_page();
            break;
        case PageKind::CutoffTest:
            create_cutoff_page();
            break;
        case PageKind::Help:
            create_help_page();
            break;
        case PageKind::SettingsHub:
            create_settings_hub_page();
            break;
        case PageKind::Power:
            create_power_page();
            break;
        case PageKind::Wifi:
            open_wifi_page();
            break;
        case PageKind::Bluetooth:
            open_ble_page();
            break;
        case PageKind::Ota:
            open_ota_page();
            break;
        case PageKind::Communication:
            create_communication_page();
            break;
        default:
            break;
    }
    Serial.printf("EVENT page open=%u\n", static_cast<unsigned>(page));
}

void close_page() {
    if (current_page == PageKind::Power &&
        !require_output_off_for_action("退出双向电源并切回模拟电池")) {
        return;
    }
    close_info_dialog();
    close_keypad();
    if (cutoff_running ||
        cutoff_phase == CutoffRunPhase::EnsureBatteryMode ||
        cutoff_phase == CutoffRunPhase::ConfigureRamp ||
        cutoff_phase == CutoffRunPhase::TriggerRamp) {
        stop_cutoff_test("页面关闭");
    }
    cutoff_running = false;
    if (current_page == PageKind::Communication &&
        communication_mode == CommunicationMode::Passthrough) {
        set_communication_mode(CommunicationMode::Controller, true);
    }
    if (current_page == PageKind::Power) {
        active_mode = WorkMode::BatterySimulator;
        request_device_mode(
            constant_current_mode
                ? JcyScpiMode::BatteryConstantCurrent
                : JcyScpiMode::Battery,
            false);
    }
    if (page_overlay != nullptr) {
        lv_obj_del(page_overlay);
        page_overlay = nullptr;
    }
    current_page = PageKind::None;
    reset_page_references();
    refresh_mode_buttons();
    refresh_main_values();
    Serial.println("EVENT page close");
}

void refresh_mode_buttons() {
    for (size_t i = 0; i < 2; ++i) {
        if (mode_buttons[i] == nullptr) {
            continue;
        }
        const bool active = active_mode == MODE_OPTIONS[i];
        if (REFERENCE_SKIN_UI) {
            // 参考底图上的两个模式图标保持原位置和原尺寸，只叠加
            // 半透明高亮、亮边和光晕，明确显示当前选择。
            lv_obj_set_style_radius(mode_buttons[i], 12, 0);
            lv_obj_set_style_bg_color(
                mode_buttons[i], color(active ? 0x0D8FA5 : 0x071A2E), 0);
            lv_obj_set_style_bg_grad_color(
                mode_buttons[i], color(active ? 0x07536A : 0x030D1B), 0);
            lv_obj_set_style_bg_grad_dir(
                mode_buttons[i], LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_bg_opa(
                mode_buttons[i], active ? LV_OPA_30 : LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_color(
                mode_buttons[i], color(active ? 0x50FFD2 : 0x17425F), 0);
            lv_obj_set_style_border_width(
                mode_buttons[i], active ? 3 : 1, 0);
            lv_obj_set_style_border_opa(
                mode_buttons[i], active ? LV_OPA_COVER : LV_OPA_50, 0);
            lv_obj_set_style_shadow_color(
                mode_buttons[i], color(0x18D7C5), 0);
            lv_obj_set_style_shadow_width(
                mode_buttons[i], active ? 14 : 0, 0);
            lv_obj_set_style_shadow_opa(
                mode_buttons[i], active ? LV_OPA_70 : LV_OPA_TRANSP, 0);
        } else {
            lv_obj_set_style_bg_color(
                mode_buttons[i], color(active ? 0x0D4568 : 0x071A2E), 0);
            lv_obj_set_style_bg_grad_color(
                mode_buttons[i], color(active ? 0x071D35 : 0x030D1B), 0);
            lv_obj_set_style_border_color(
                mode_buttons[i], color(active ? UI_CYAN : UI_BORDER_DIM), 0);
            lv_obj_set_style_border_width(
                mode_buttons[i], active ? 2 : 1, 0);
        }
        if (mode_labels[i] != nullptr) {
            lv_obj_set_style_text_color(
                mode_labels[i], color(active ? UI_TEXT : 0x597C9C), 0);
        }
    }
}

void refresh_power_state() {
    if (status_led != nullptr) {
        lv_obj_set_style_bg_color(
            status_led, color(output_running ? 0x65F3DA : 0xE8F6FF), 0);
    }

    uint32_t bg_top = 0x0A3150;
    uint32_t bg_bottom = 0x041426;
    uint32_t border_color = UI_CYAN;
    uint32_t icon_color = UI_CYAN;
    if (output_change_pending) {
        bg_top = 0xFFB85C;
        bg_bottom = 0xE47A35;
        border_color = 0xFFD49A;
        icon_color = 0xFFF7D6;
    } else if (output_running) {
        bg_top = 0xFF735F;
        bg_bottom = 0xE92F49;
        border_color = 0xFFB2A5;
        icon_color = 0xFFFFFF;
    }

    if (power_button != nullptr) {
        lv_obj_set_style_bg_color(power_button, color(bg_top), 0);
        lv_obj_set_style_bg_grad_color(power_button, color(bg_bottom), 0);
        lv_obj_set_style_border_color(power_button, color(border_color), 0);
    }
    if (power_icon != nullptr) {
        lv_obj_set_style_arc_color(power_icon, color(icon_color),
                                   LV_PART_MAIN);
    }
    if (power_icon_stem != nullptr) {
        lv_obj_set_style_bg_color(power_icon_stem, color(icon_color), 0);
    }
}

void refresh_alarm_bar(int code, const char *status) {
    current_alarm_code = max(code, 0);
    current_alarm_kind = classify_alarm(current_alarm_code, status);
    if (alarm_bar == nullptr || alarm_label == nullptr || alarm_icon == nullptr) {
        return;
    }

    uint32_t top = 0x0A3A45;
    uint32_t bottom = 0x06232F;
    const char *icon = "OK";
    if (current_alarm_kind != JcyAlarmKind::None) {
        icon = "!";
        if (current_alarm_kind == JcyAlarmKind::ShortCircuit ||
            current_alarm_kind == JcyAlarmKind::ReversePolarity) {
            top = 0xFF596A;
            bottom = 0xD92E49;
        } else {
            top = 0xFFAD4D;
            bottom = 0xE36B34;
        }
    }
    lv_obj_set_style_bg_color(alarm_bar, color(top), 0);
    lv_obj_set_style_bg_grad_color(alarm_bar, color(bottom), 0);
    lv_label_set_text(alarm_icon, icon);
    lv_label_set_text(alarm_label, alarm_short_label(current_alarm_kind));
    lv_obj_set_style_text_color(
        alarm_icon,
        color(current_alarm_kind == JcyAlarmKind::None
                  ? 0x83FFD3 : 0xFFFFFF), 0);
}

void alarm_bar_event(lv_event_t *) {
    show_info_dialog(current_alarm_kind == JcyAlarmKind::None
                         ? "设备状态" : "设备报警",
                     alarm_long_text(current_alarm_kind));
}

void set_battery_status_visual(const char *text,
                               uint32_t top_color,
                               uint32_t bottom_color,
                               uint32_t border_color) {
    if (status_value_label != nullptr) {
        lv_label_set_text(status_value_label, text);
        lv_obj_set_style_text_color(status_value_label,
                                    color(0xFFFFFF), 0);
        lv_obj_set_style_bg_color(status_value_label,
                                  color(top_color), 0);
        lv_obj_set_style_bg_grad_color(status_value_label,
                                       color(bottom_color), 0);
        lv_obj_set_style_bg_grad_dir(status_value_label,
                                     LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(status_value_label, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(status_value_label,
                                      color(border_color), 0);
    }
    if (status_badge != nullptr) {
        lv_obj_set_style_bg_color(status_badge, color(top_color), 0);
        lv_obj_set_style_bg_grad_color(status_badge,
                                       color(bottom_color), 0);
        lv_obj_set_style_border_color(status_badge,
                                      color(border_color), 0);
    }
}

void update_voltage_digit_selection() {
    for (size_t i = 0; i < 5; ++i) {
        const bool selected = i == voltage_edit_digit;
        lv_obj_t *boxes[] = {
            set_voltage_digit_boxes[i], main_wave_digit_boxes[i]};
        lv_obj_t *labels[] = {
            set_voltage_digit_labels[i], main_wave_digit_labels[i]};
        for (size_t copy = 0; copy < 2; ++copy) {
            if (boxes[copy] == nullptr) continue;
            lv_obj_set_style_border_width(
                boxes[copy], 1, 0);
            lv_obj_set_style_border_color(
                boxes[copy], color(UI_BORDER_DIM), 0);
            lv_obj_set_style_bg_color(
                boxes[copy],
                color(selected ? 0x0C4164 : 0x071A31), 0);
            lv_obj_set_style_bg_grad_color(
                boxes[copy],
                color(selected ? 0x08243D : 0x030D1C), 0);
            if (labels[copy] != nullptr) {
                lv_obj_set_style_text_color(
                    labels[copy],
                    color(selected ? UI_CYAN : 0xD8EEFF), 0);
            }
        }
    }
}

void refresh_set_voltage() {
    normalize_battery_settings();
    if (set_voltage_header_label != nullptr) {
        lv_label_set_text_fmt(set_voltage_header_label, "%d.%03dV",
                              set_voltage_mv / 1000, set_voltage_mv % 1000);
    }
    static constexpr int divisors[5] = {10000, 1000, 100, 10, 1};
    for (size_t i = 0; i < 5; ++i) {
        const int digit = (set_voltage_mv / divisors[i]) % 10;
        if (set_voltage_digit_labels[i] != nullptr) {
            lv_label_set_text_fmt(set_voltage_digit_labels[i], "%d", digit);
        }
        if (main_wave_digit_labels[i] != nullptr) {
            lv_label_set_text_fmt(main_wave_digit_labels[i], "%d", digit);
        }
    }
    update_voltage_digit_selection();
}

void update_battery_soc_display(int voltage_mv) {
    const uint32_t now = millis();
    const bool measured = scpi_link.measurements_fresh(now);
    const int source_voltage_mv = measured ? voltage_mv : set_voltage_mv;
    const BatterySocEstimate estimate = estimate_battery_soc(
        battery_settings.chemistry, source_voltage_mv,
        battery_settings.cells);

    if (measurement_soc_label != nullptr) {
        lv_label_set_text_fmt(measurement_soc_label, "%d%%", estimate.percent);
    }
    if (measurement_soc_caption_label != nullptr) {
        lv_label_set_text_fmt(
            measurement_soc_caption_label, "%s %d串",
            battery_chemistry_short_name(battery_settings.chemistry),
            battery_settings.cells);
    }
    if (measurement_battery_cells[0] != nullptr) {
        const int active_cells = estimate.percent <= 0
                                     ? 0
                                     : min(5, (estimate.percent + 19) / 20);
        uint32_t top_color = 0x71F7DB;
        uint32_t bottom_color = 0x28C9BE;
        if (estimate.percent <= 15) {
            top_color = 0xFF7B87;
            bottom_color = 0xEE3456;
        } else if (estimate.percent <= 35) {
            top_color = 0xFFD36B;
            bottom_color = 0xF39A2F;
        }
        for (int i = 0; i < 5; ++i) {
            const bool active = i < active_cells;
            lv_obj_set_style_bg_color(
                measurement_battery_cells[i],
                color(active ? top_color : 0x174D7B), 0);
            lv_obj_set_style_bg_grad_color(
                measurement_battery_cells[i],
                color(active ? bottom_color : 0x113C63), 0);
            lv_obj_set_style_border_color(
                measurement_battery_cells[i],
                color(active ? 0xC8FFF6 : 0x2B6B99), 0);
            lv_obj_set_style_bg_opa(measurement_battery_cells[i],
                                    LV_OPA_COVER, 0);
        }
    }
    if (mode_battery_cells[0] != nullptr) {
        const int active_cells = estimate.percent <= 0
                                     ? 0
                                     : min(5, (estimate.percent + 19) / 20);
        uint32_t top_color = 0x71F7DB;
        uint32_t bottom_color = 0x28C9BE;
        if (estimate.percent <= 15) {
            top_color = 0xFF7B87;
            bottom_color = 0xEE3456;
        } else if (estimate.percent <= 35) {
            top_color = 0xFFD36B;
            bottom_color = 0xF39A2F;
        }
        for (int i = 0; i < 5; ++i) {
            const bool active = i < active_cells;
            lv_obj_set_style_bg_color(
                mode_battery_cells[i],
                color(active ? top_color : 0x174D7B), 0);
            lv_obj_set_style_bg_grad_color(
                mode_battery_cells[i],
                color(active ? bottom_color : 0x113C63), 0);
            lv_obj_set_style_border_color(
                mode_battery_cells[i],
                color(active ? 0xC8FFF6 : 0x2B6B99), 0);
        }
    }
}

bool idle_render_state_changed(uint32_t now) {
    const JcyScpiMeasurements &measurements = scpi_link.measurements();
    const DisplayMeasurementValues display_values =
        measurement_values_for_display(measurements);
    const bool fresh = scpi_link.measurements_fresh(now);
    const bool online = scpi_link.online(now, 2000);
    const bool visible_output_on = displayed_output_state(measurements);
    if (!idle_render_cache.valid) {
        return true;
    }
    return idle_render_cache.fresh != fresh ||
           idle_render_cache.online != online ||
           idle_render_cache.output_on != visible_output_on ||
           idle_render_cache.output_change_pending != output_change_pending ||
           idle_render_cache.requested_output_state != requested_output_state ||
           idle_render_cache.voltage_mv != display_values.voltage_mv ||
           idle_render_cache.current_ua != display_values.current_ua ||
           idle_render_cache.temperature_c10 != measurements.temperature_c10 ||
           idle_render_cache.error_code != measurements.error_code ||
           strcmp(idle_render_cache.status, measurements.status) != 0;
}

void cache_idle_render_state(uint32_t now) {
    const JcyScpiMeasurements &measurements = scpi_link.measurements();
    const DisplayMeasurementValues display_values =
        measurement_values_for_display(measurements);
    idle_render_cache.valid = true;
    idle_render_cache.fresh = scpi_link.measurements_fresh(now);
    idle_render_cache.online = scpi_link.online(now, 2000);
    idle_render_cache.output_on = displayed_output_state(measurements);
    idle_render_cache.output_change_pending = output_change_pending;
    idle_render_cache.requested_output_state = requested_output_state;
    idle_render_cache.voltage_mv = display_values.voltage_mv;
    idle_render_cache.current_ua = display_values.current_ua;
    idle_render_cache.temperature_c10 = measurements.temperature_c10;
    idle_render_cache.error_code = measurements.error_code;
    strlcpy(idle_render_cache.status, measurements.status,
            sizeof(idle_render_cache.status));
}

void invalidate_idle_render_cache() {
    idle_render_cache.valid = false;
}

void refresh_live_measurements() {
    const uint32_t now = millis();
    const JcyScpiMeasurements &measurements = scpi_link.measurements();
    const bool fresh = scpi_link.measurements_fresh(now);

    // OUT ACK 到达时 measurements 仍可能是切换前的一笔 ON/OFF 轮询。
    // 只有跨过确认栅栏后的状态才能重新接管 UI，避免 OFF 后
    // “设置页 -> 波形页 -> 设置页”的反复切换。
    const bool accept_output_measurement =
        fresh && !output_change_pending &&
        accept_measured_output_state(measurements, now);
    if (accept_output_measurement) {
        const bool output_changed = output_running != measurements.output_on;
        output_running = measurements.output_on;
        requested_output_state = measurements.output_on;
        if (output_changed) {
            refresh_main_waveform_visibility();
        }
    }
    const bool visible_output_on = displayed_output_state(measurements);
    const DisplayMeasurementValues display_values =
        measurement_values_for_display(measurements);

    char voltage_text[24] = "--.---V";
    char current_text[24] = "--";
    char power_text[24] = "--.---W";
    if (fresh) {
        snprintf(voltage_text, sizeof(voltage_text), "%d.%03dV",
                 display_values.voltage_mv / 1000,
                 abs(display_values.voltage_mv % 1000));
        if (display_values.current_zeroed) {
            strlcpy(current_text, "0.000A", sizeof(current_text));
        } else {
            format_current_text(current_text, sizeof(current_text),
                                display_values.current_ua);
        }
        const double power_w =
            static_cast<double>(display_values.voltage_mv) *
            static_cast<double>(display_values.current_ua) / 1000000000.0;
        snprintf(power_text, sizeof(power_text), "%.3fW", power_w);
    }

    if (measurement_voltage_label != nullptr) {
        set_adaptive_numeric_label(measurement_voltage_label,
                                   voltage_text, 36);
    }
    if (measurement_current_label != nullptr) {
        set_adaptive_numeric_label(
            measurement_current_label, current_text,
            measurement_current_limit_label != nullptr ? 30 : 36);
    }
    if (measurement_power_label != nullptr) {
        set_adaptive_numeric_label(measurement_power_label,
                                   power_text, 30);
    }

    if (measurement_long_press_shown && info_overlay != nullptr) {
        if (measurement_popup_voltage_label != nullptr) {
            set_adaptive_numeric_label(
                measurement_popup_voltage_label,
                voltage_text, 48);
        }
        if (measurement_popup_current_label != nullptr) {
            set_adaptive_numeric_label(
                measurement_popup_current_label,
                current_text, 48);
        }
        if (measurement_popup_power_label != nullptr) {
            set_adaptive_numeric_label(
                measurement_popup_power_label,
                power_text, 48);
        }
    }

    if (power_live_voltage_label != nullptr) {
        set_adaptive_numeric_label(power_live_voltage_label,
                                   voltage_text, 36);
    }
    if (power_live_current_label != nullptr) {
        set_adaptive_numeric_label(power_live_current_label,
                                   current_text, 36);
    }
    if (power_live_power_label != nullptr) {
        set_adaptive_numeric_label(power_live_power_label,
                                   power_text, 30);
    }
    if (cutoff_realtime_voltage_label != nullptr) {
        set_adaptive_numeric_label(cutoff_realtime_voltage_label,
                                   voltage_text, 16);
    }
    if (cutoff_realtime_current_label != nullptr) {
        set_adaptive_numeric_label(cutoff_realtime_current_label,
                                   current_text, 16);
    }

    if (fresh && output_running &&
        (!output_measurement_fence.active || measurements.output_on) &&
        measurements.sequence != main_waveform_last_sequence) {
        append_main_waveform_sample(measurements);
        main_waveform_last_sequence = measurements.sequence;
    }
    if (fresh && cutoff_running && cutoff_chart != nullptr) {
        append_cutoff_waveform_sample(measurements.voltage_mv,
                                      measurements.current_ua);
    }

    if (status_value_label != nullptr && !output_change_pending) {
        const uint32_t online_max_age = 2000;
        const bool online_now = scpi_link.online(now, online_max_age);
        if (!fresh && !online_now) {
            set_battery_status_visual(
                "TTL未连接", 0xD49A2E, 0xA86D17, 0xFFE18A);
        } else if (!fresh) {
            set_battery_status_visual(
                "等待实时数据", 0x587B98, 0x3D5E7C, 0xA9CDE5);
        } else if (!visible_output_on) {
            set_battery_status_visual(
                "静置", 0x4F86B2, 0x356B98, 0x91D5F6);
        } else if (measurements.current_ua <
                   -battery_settings.static_threshold_ma * 1000) {
            set_battery_status_visual(
                "电池充电中", 0x31C890, 0x178D67, 0x84FFD3);
        } else if (measurements.current_ua >
                   battery_settings.static_threshold_ma * 1000) {
            set_battery_status_visual(
                "电池放电中", 0xF39A3D, 0xC66720, 0xFFE08A);
        } else {
            set_battery_status_visual(
                "静置", 0x4F86B2, 0x356B98, 0x91D5F6);
        }
    }

    if (power_output_label != nullptr && !output_change_pending) {
        lv_label_set_text(power_output_label,
                          output_running ? "输出已开启" : "输出已关闭");
    }
    refresh_power_page_output_state();
    if (fresh) {
        // 下位机在输出关闭时会返回 ERROR<005> E STOP，这是待机状态，
        // 不是设备保护。只有输出工作时或其他错误码才显示报警。
        const bool idle_stop = !visible_output_on &&
                               measurements.error_code == 5;
        int alarm_code = idle_stop ? 0 : measurements.error_code;
        const char *alarm_status = idle_stop ? nullptr : measurements.status;
        // 实时温度不占用主界面；超过60°C时转为醒目的过温告警。
        if (alarm_code == 0 && measurements.temperature_c10 > 600) {
            alarm_code = 60;
            alarm_status = "overtemperature";
        }
        refresh_alarm_bar(alarm_code, alarm_status);
    }
    refresh_power_state();
    refresh_main_waveform_visibility();
    refresh_power_page_status();
    update_battery_soc_display(measurements.voltage_mv);
    last_measurement_sequence = measurements.sequence;
    last_live_ui_refresh_ms = now;
}

void refresh_main_values() {
    compute_quick_voltages();
    refresh_set_voltage();
    if (measurement_current_limit_label != nullptr) {
        if (battery_settings.overcurrent_ma >= 1000) {
            lv_label_set_text_fmt(
                measurement_current_limit_label, "MAX %d.%03dA",
                battery_settings.overcurrent_ma / 1000,
                battery_settings.overcurrent_ma % 1000);
        } else {
            lv_label_set_text_fmt(
                measurement_current_limit_label, "MAX %dmA",
                battery_settings.overcurrent_ma);
        }
    }

    if (minimum_voltage_label != nullptr) {
        lv_label_set_text_fmt(minimum_voltage_label, "最小 %d.%03dV",
                              pack_minimum_mv() / 1000,
                              pack_minimum_mv() % 1000);
    }
    if (maximum_voltage_label != nullptr) {
        lv_label_set_text_fmt(maximum_voltage_label, "最大 %d.%03dV",
                              pack_maximum_mv() / 1000,
                              pack_maximum_mv() % 1000);
    }
    if (battery_card_value_label != nullptr) {
        lv_label_set_text_fmt(battery_card_value_label, "%d节电池",
                              battery_settings.cells);
    }
    if (cutoff_card_value_label != nullptr) {
        lv_label_set_text(cutoff_card_value_label, "过充 / 过放");
    }
    refresh_constant_current_card();
    if (main_wave_chart != nullptr) {
        lv_chart_set_range(main_wave_chart, LV_CHART_AXIS_PRIMARY_Y,
                           max(0, pack_minimum_mv() - 300),
                           min(MAX_BATTERY_PACK_MV,
                               pack_maximum_mv() + 300));
        main_wave_current_range_ma = min(
            max(main_wave_current_range_ma, 100),
            max(100, battery_settings.overcurrent_ma));
        lv_chart_set_range(main_wave_chart, LV_CHART_AXIS_SECONDARY_Y,
                           -main_wave_current_range_ma,
                           main_wave_current_range_ma);
    }
    for (size_t i = 0; i < 6; ++i) {
        if (quick_voltage_labels[i] != nullptr) {
            lv_label_set_text_fmt(
                quick_voltage_labels[i], "%d.%03dV\n%d%%",
                quick_voltages_mv[i] / 1000,
                quick_voltages_mv[i] % 1000,
                QUICK_PERCENTAGES[i]);
        }
    }
    update_battery_soc_display(scpi_link.measurements().voltage_mv);
}

void mode_event(lv_event_t *event) {
    const auto *mode = static_cast<const WorkMode *>(lv_event_get_user_data(event));
    if (mode == nullptr) {
        return;
    }

    // 双向电源页只覆盖右侧内容区，左侧两个模式标签始终保留。
    // 点击左侧“模拟电池”直接关闭电源页并恢复电池模式；点击当前
    // “双向电源”只保持选中高亮，不重复重建页面。
    if (current_page == PageKind::Power) {
        if (*mode == WorkMode::BatterySimulator) {
            close_page();
        } else {
            refresh_mode_buttons();
        }
        return;
    }

    if (*mode == active_mode) {
        refresh_mode_buttons();
        return;
    }
    if (*mode != active_mode &&
        !require_output_off_for_action("切换仪器工作模式")) {
        return;
    }
    active_mode = *mode;
    refresh_mode_buttons();
    Serial.printf("EVENT mode=%s TTL\n",
                  active_mode == WorkMode::BatterySimulator
                      ? "battery" : "bidirectional_power");
    if (active_mode == WorkMode::BidirectionalPower) {
        open_page(PageKind::Power);
    } else {
        request_device_mode(
            constant_current_mode
                ? JcyScpiMode::BatteryConstantCurrent
                : JcyScpiMode::Battery,
            true);
    }
}

void constant_current_event(lv_event_t *) {
    if (constant_current_change_pending) {
        return;
    }
    if (cutoff_running ||
        cutoff_phase == CutoffRunPhase::EnsureBatteryMode ||
        cutoff_phase == CutoffRunPhase::ConfigureRamp ||
        cutoff_phase == CutoffRunPhase::TriggerRamp) {
        show_info_dialog("截止测试进行中",
                         "RAMP截止测试期间固定使用普通电池模式，不能切换恒流模式。");
        return;
    }
    if (!require_output_off_for_action("切换过流/恒流模式")) {
        return;
    }
    requested_constant_current_mode = !constant_current_mode;
    constant_current_change_pending = request_device_mode(
        requested_constant_current_mode
            ? JcyScpiMode::BatteryConstantCurrent
            : JcyScpiMode::Battery,
        true);
    if (!constant_current_change_pending) {
        requested_constant_current_mode = constant_current_mode;
        show_info_dialog("命令未发送", "TTL命令队列忙，请稍后重试。");
    }
    refresh_constant_current_card();
    Serial.printf("EVENT battery_cc request=%d TTL\n",
                  requested_constant_current_mode);
}

void power_event(lv_event_t *) {
    apply_output_change(!requested_output_state, true);
    Serial.printf("EVENT battery output=%d TTL\n", requested_output_state);
}

void voltage_digit_select_event(lv_event_t *event) {
    const auto *digit = static_cast<const uint8_t *>(
        lv_event_get_user_data(event));
    if (digit == nullptr || *digit >= 5) {
        return;
    }
    const uint32_t now = millis();
    const bool double_tap =
        voltage_last_tap_digit == *digit &&
        static_cast<uint32_t>(now - voltage_last_tap_ms) <= 350;
    voltage_edit_digit = *digit;
    update_voltage_digit_selection();
    voltage_last_tap_digit = *digit;
    voltage_last_tap_ms = now;
    if (double_tap) {
        voltage_last_tap_digit = 0xFF;
        voltage_last_tap_ms = 0;
        open_numeric_field(NumericField::SetVoltage);
        Serial.println("EVENT voltage direct input");
        return;
    }
    Serial.printf("EVENT voltage digit=%u step=%d mV\n",
                  voltage_edit_digit,
                  VOLTAGE_DIGIT_STEPS_MV[voltage_edit_digit]);
}

void voltage_step_event(lv_event_t *event) {
    const auto *direction = static_cast<const int *>(
        lv_event_get_user_data(event));
    if (direction == nullptr || (*direction != -1 && *direction != 1)) {
        return;
    }
    set_voltage_mv += *direction *
                      VOLTAGE_DIGIT_STEPS_MV[voltage_edit_digit];
    set_voltage_mv = constrain(set_voltage_mv,
                                pack_minimum_mv(),
                                pack_maximum_mv());
    refresh_main_values();
    Serial.printf("EVENT voltage %d mV\n", set_voltage_mv);
    send_numeric_field_to_device(NumericField::SetVoltage);
}

void quick_voltage_event(lv_event_t *event) {
    const auto *voltage = static_cast<const int *>(lv_event_get_user_data(event));
    if (voltage == nullptr) {
        return;
    }
    set_voltage_mv = *voltage;
    refresh_main_values();
    Serial.printf("EVENT quick_voltage=%d TTL\n", set_voltage_mv);
    send_numeric_field_to_device(NumericField::SetVoltage);
}

void open_page_event(lv_event_t *event) {
    const auto page = static_cast<PageKind>(
        reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    open_page(page);
}

void show_measurement_popup() {
    if (measurement_long_press_shown || info_overlay != nullptr ||
        page_overlay != nullptr) {
        return;
    }
    measurement_long_press_shown = true;
    close_info_dialog();
    info_overlay = lv_obj_create(ui_screen);
    lv_obj_set_pos(info_overlay, 0, 0);
    lv_obj_set_size(info_overlay, SCREEN_WIDTH, SCREEN_HEIGHT);
    style_clean(info_overlay);
    lv_obj_set_style_radius(info_overlay, 0, 0);
    lv_obj_set_style_border_width(info_overlay, 0, 0);
    lv_obj_set_style_bg_color(info_overlay, color(0x020B16), 0);
    lv_obj_set_style_bg_grad_color(info_overlay, color(0x071D31), 0);
    lv_obj_set_style_bg_grad_dir(info_overlay, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(info_overlay, LV_OPA_COVER, 0);
    lv_obj_add_flag(info_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(info_overlay, close_info_event,
                        LV_EVENT_CLICKED, nullptr);

    lv_obj_t *header = make_gradient_box(
        info_overlay, 0, 0, SCREEN_WIDTH, 62,
        0x0B2944, 0x041426, 0, 0x123A57, 0);
    make_label(header, "实时测量", 22, 16,
               &lv_font_jcy20, 0xFFFFFF);
    make_label(header, "大字模式", 132, 18,
               &lv_font_jcy16, 0x74DFF2);
    lv_obj_t *version = make_label(
        header, JCY1000_FIRMWARE_VERSION, 602, 22,
        &lv_font_montserrat_16, 0x7FA7C0);
    lv_obj_set_width(version, 108);
    lv_obj_set_style_text_align(version, LV_TEXT_ALIGN_RIGHT, 0);
    make_label(header, "轻触返回", 715, 18,
               &lv_font_jcy16, 0xD8F5FF);

    struct MeasurementCardSpec {
        int y;
        const char *badge;
        const char *title;
        const char *english;
        const char *initial;
        uint32_t top;
        uint32_t bottom;
        uint32_t accent;
        uint32_t badge_bottom;
        lv_obj_t **value_label;
    };
    MeasurementCardSpec cards[] = {
        {72, "V", "实时电压", "VOLTAGE", "--.---V",
         0x0A304C, 0x04182A, 0x69D9FF, 0x3673D7,
         &measurement_popup_voltage_label},
        {196, "A", "实时电流", "CURRENT", "--",
         0x083A3D, 0x031F28, 0x60F1D7, 0x1DA2A8,
         &measurement_popup_current_label},
        {320, "W", "实时功率", "POWER", "--.---W",
         0x29264D, 0x14182E, 0xD69BFF, 0x7056D4,
         &measurement_popup_power_label},
    };

    for (MeasurementCardSpec &card : cards) {
        lv_obj_t *panel = make_gradient_box(
            info_overlay, 18, card.y, 764, 112,
            card.top, card.bottom, 12, card.accent, 1);
        lv_obj_t *accent = make_gradient_box(
            panel, 0, 0, 7, 112,
            card.accent, card.badge_bottom, 12,
            card.accent, 0);
        lv_obj_clear_flag(accent, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t *badge = make_gradient_box(
            panel, 24, 20, 72, 72,
            card.accent, card.badge_bottom,
            LV_RADIUS_CIRCLE, card.accent, 1);
        make_center_label(badge, card.badge,
                          &lv_font_montserrat_36, 0xFFFFFF);

        make_label(panel, card.title, 118, 20,
                   &lv_font_jcy20, 0xFFFFFF);
        make_label(panel, card.english, 119, 57,
                   &lv_font_montserrat_14, card.accent);

        *card.value_label = make_label(
            panel, card.initial, 270, 26,
            &lv_font_montserrat_48, 0xFFFFFF);
        lv_obj_set_width(*card.value_label, 464);
        lv_obj_set_style_text_align(*card.value_label,
                                    LV_TEXT_ALIGN_RIGHT, 0);
        lv_label_set_long_mode(*card.value_label,
                               LV_LABEL_LONG_CLIP);
    }

    make_label(info_overlay, "长按主界面测量区域 2 秒进入",
               18, 449, &lv_font_jcy16, 0x628AA4);
    lv_obj_t *exit_hint = make_label(
        info_overlay, "点击屏幕任意位置退出",
        568, 449, &lv_font_jcy16, 0x8FDDEB);
    lv_obj_set_width(exit_hint, 214);
    lv_obj_set_style_text_align(exit_hint, LV_TEXT_ALIGN_RIGHT, 0);
}

void measurement_press_event(lv_event_t *event) {
    const lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_PRESSED) {
        measurement_press_started = millis();
        measurement_long_press_shown = false;
    } else if (code == LV_EVENT_PRESSING) {
        if (!measurement_long_press_shown &&
            millis() - measurement_press_started >= 2000) {
            show_measurement_popup();
        }
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        measurement_press_started = 0;
    }
}

void apply_wireless_button_state(lv_obj_t *button, JcyLinkState state) {
    if (button == nullptr) {
        return;
    }
    uint32_t top = 0x7793AA;
    uint32_t bottom = 0x526A82;
    uint32_t border = 0x9AB2C6;
    if (state == JcyLinkState::Idle) {
        top = 0x4FA9E8;
        bottom = 0x2774B5;
        border = 0x7EE2FF;
    } else if (state == JcyLinkState::Busy) {
        top = 0xFFC764;
        bottom = 0xE68A31;
        border = 0xFFE6A2;
    } else if (state == JcyLinkState::Connected) {
        top = 0x54E4C8;
        bottom = 0x20A88E;
        border = 0xA3FFF0;
    } else if (state == JcyLinkState::Error) {
        top = 0xFF7C8C;
        bottom = 0xD83F5A;
        border = 0xFFB0BA;
    }
    lv_obj_set_style_bg_color(button, color(top), 0);
    lv_obj_set_style_bg_grad_color(button, color(bottom), 0);
    lv_obj_set_style_border_color(button, color(border), 0);
}

void set_label_text_if_changed(lv_obj_t *label, const char *text) {
    if (label == nullptr || text == nullptr) {
        return;
    }
    // LVGL 的 set_text 即使文本相同也可能触发无效区域刷新。OTA 写 Flash
    // 时避免这种无意义的整页重绘，减少 RGB DMA 与缓存总线的竞争。
    if (strcmp(lv_label_get_text(label), text) != 0) {
        lv_label_set_text(label, text);
    }
}

void set_button_disabled_if_changed(lv_obj_t *button, bool disabled) {
    if (button == nullptr) {
        return;
    }
    const bool current = lv_obj_has_state(button, LV_STATE_DISABLED);
    if (current == disabled) {
        return;
    }
    if (disabled) {
        lv_obj_add_state(button, LV_STATE_DISABLED);
    } else {
        lv_obj_clear_state(button, LV_STATE_DISABLED);
    }
}

struct HeaderWirelessRenderCache {
    lv_obj_t *wifi_button = nullptr;
    lv_obj_t *bt_button = nullptr;
    lv_obj_t *wifi_badge = nullptr;
    lv_obj_t *bt_badge = nullptr;
    JcyLinkState wifi_state = JcyLinkState::Off;
    JcyLinkState bt_state = JcyLinkState::Off;
    int wifi_bars_active = -1;
};

void set_wireless_status_badge(lv_obj_t *badge, lv_obj_t *label,
                               JcyLinkState state, bool wifi) {
    if (badge == nullptr || label == nullptr) {
        return;
    }

    const char *text = "关闭";
    uint32_t top = 0x253849;
    uint32_t bottom = 0x162836;
    uint32_t border = 0x627C91;
    if (state == JcyLinkState::Idle) {
        text = wifi ? "待连" : "广播";
        top = 0x125B9E;
        bottom = 0x0A385F;
        border = 0x62C8FF;
    } else if (state == JcyLinkState::Busy) {
        text = "连接";
        top = 0xC77920;
        bottom = 0x70410F;
        border = 0xFFE1A0;
    } else if (state == JcyLinkState::Connected) {
        text = "已连";
        top = 0x0E856C;
        bottom = 0x075242;
        border = 0x6CFFD7;
    } else if (state == JcyLinkState::Error) {
        text = "错误";
        top = 0x9E3A52;
        bottom = 0x602030;
        border = 0xFF9BAC;
    }
    lv_obj_set_style_bg_color(badge, color(top), 0);
    lv_obj_set_style_bg_grad_color(badge, color(bottom), 0);
    lv_obj_set_style_bg_grad_dir(badge, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_color(badge, color(border), 0);
    set_label_text_if_changed(label, text);
}

void refresh_wireless_header() {
    static HeaderWirelessRenderCache cache;
    const WifiStatus &header_wifi_status = jcy_wireless::wifi_status();
    const BleStatus &header_ble_status = jcy_wireless::ble_status();
    const bool widgets_recreated =
        cache.wifi_button != header_wifi_button ||
        cache.bt_button != header_bt_button ||
        cache.wifi_badge != header_wifi_status_badge ||
        cache.bt_badge != header_bt_status_badge;

    if (widgets_recreated || cache.wifi_state != header_wifi_status.state) {
        apply_wireless_button_state(header_wifi_button,
                                    header_wifi_status.state);
        if (header_wifi_icon != nullptr) {
            lv_obj_set_style_text_color(
                header_wifi_icon,
                color(header_wifi_status.state == JcyLinkState::Busy ||
                              header_wifi_status.state == JcyLinkState::Error
                          ? 0xFFEBC1 : 0xFFFFFF),
                0);
        }
        set_wireless_status_badge(header_wifi_status_badge,
                                  header_wifi_status_label,
                                  header_wifi_status.state, true);
    }
    if (widgets_recreated || cache.bt_state != header_ble_status.state) {
        apply_wireless_button_state(header_bt_button, header_ble_status.state);
        set_wireless_status_badge(header_bt_status_badge,
                                  header_bt_status_label,
                                  header_ble_status.state, false);
    }

    int bars_active = 0;
    if (header_wifi_status.state == JcyLinkState::Connected) {
        bars_active = header_wifi_status.rssi >= -58 ? 3
                    : header_wifi_status.rssi >= -72 ? 2
                    : 1;
    }
    const uint32_t bar_on_top = 0x54E4C8;
    const uint32_t bar_on_bottom = 0x20A88E;
    const uint32_t bar_off_top = 0x33556D;
    const uint32_t bar_off_bottom = 0x1F3447;
    if (widgets_recreated || cache.wifi_bars_active != bars_active) {
        for (int i = 0; i < 3; ++i) {
            if (header_wifi_bars[i] == nullptr) continue;
            const bool on = (i < bars_active);
            lv_obj_set_style_bg_color(
                header_wifi_bars[i],
                color(on ? bar_on_top : bar_off_top), 0);
            lv_obj_set_style_bg_grad_color(
                header_wifi_bars[i],
                color(on ? bar_on_bottom : bar_off_bottom), 0);
            lv_obj_set_style_border_color(
                header_wifi_bars[i],
                color(on ? 0xA3FFF0 : 0x5A7283), 0);
        }
    }
    if (header_bt_status_dot != nullptr &&
        (widgets_recreated || cache.bt_state != header_ble_status.state)) {
        uint32_t dot_color = 0x555E66;
        if (header_ble_status.state == JcyLinkState::Idle) dot_color = 0x67A8DB;
        else if (header_ble_status.state == JcyLinkState::Connected) dot_color = 0x54E4C8;
        else if (header_ble_status.state == JcyLinkState::Busy) dot_color = 0xFFC764;
        else if (header_ble_status.state == JcyLinkState::Error) dot_color = 0xFF7C8C;
        lv_obj_set_style_bg_color(
            header_bt_status_dot, color(dot_color), 0);
    }
    cache.wifi_button = header_wifi_button;
    cache.bt_button = header_bt_button;
    cache.wifi_badge = header_wifi_status_badge;
    cache.bt_badge = header_bt_status_badge;
    cache.wifi_state = header_wifi_status.state;
    cache.bt_state = header_ble_status.state;
    cache.wifi_bars_active = bars_active;
}

void create_header(lv_obj_t *screen) {
    lv_obj_t *header = make_gradient_box(
        screen, 0, 0, SCREEN_WIDTH, HEADER_HEIGHT,
        0x0A2440, 0x030D1C, 0, UI_BORDER_DIM, 0);

    lv_obj_t *logo = lv_img_create(header);
    lv_img_set_src(logo, &lv_img_jcy_logo);
    lv_obj_set_pos(logo, 15, 6);
    lv_obj_set_style_img_recolor(logo, color(0xFFFFFF), 0);
    lv_obj_set_style_img_recolor_opa(logo, LV_OPA_COVER, 0);

    make_label(header, JCY1000_FIRMWARE_VERSION, 440, 24,
               &lv_font_montserrat_16, UI_MUTED);

    header_wifi_button = make_gradient_button(
        header, 558, 17, 36, 36, 0x0B3150, 0x041426,
        LV_RADIUS_CIRCLE, UI_BORDER, 1);
    lv_obj_add_event_cb(
        header_wifi_button, open_page_event, LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(PageKind::Wifi)));
    header_wifi_icon = make_label(
        header_wifi_button, LV_SYMBOL_WIFI, 12, 4,
        &lv_font_montserrat_14, 0xFFFFFF);
    lv_obj_clear_flag(header_wifi_icon, LV_OBJ_FLAG_CLICKABLE);
    constexpr int BAR_W = 4;
    constexpr int BAR_H_LIST[3] = {6, 9, 12};
    for (int i = 0; i < 3; ++i) {
        lv_obj_t *bar = lv_obj_create(header_wifi_button);
        lv_obj_set_pos(bar, 22 + i * (BAR_W + 1),
                       28 - BAR_H_LIST[i]);
        lv_obj_set_size(bar, BAR_W, BAR_H_LIST[i]);
        lv_obj_set_style_radius(bar, 1, 0);
        lv_obj_set_style_border_width(bar, 1, 0);
        lv_obj_set_style_border_color(bar, color(0x5A7283), 0);
        lv_obj_set_style_bg_color(bar, color(0x33556D), 0);
        lv_obj_set_style_bg_grad_color(bar, color(0x1F3447), 0);
        lv_obj_set_style_bg_grad_dir(bar, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_CLICKABLE);
        header_wifi_bars[i] = bar;
    }

    header_bt_button = make_gradient_button(
        header, 604, 17, 36, 36, 0x0B3150, 0x041426,
        LV_RADIUS_CIRCLE, UI_BORDER, 1);
    lv_obj_add_event_cb(
        header_bt_button, open_page_event, LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(PageKind::Bluetooth)));
    make_label(header_bt_button, LV_SYMBOL_BLUETOOTH, 8, 7,
               &lv_font_montserrat_14, 0xFFFFFF);
    header_bt_status_dot = lv_obj_create(header_bt_button);
    lv_obj_set_pos(header_bt_status_dot, 28, 4);
    lv_obj_set_size(header_bt_status_dot, 6, 6);
    lv_obj_set_style_radius(header_bt_status_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(header_bt_status_dot, 0, 0);
    lv_obj_set_style_bg_color(header_bt_status_dot, color(0x555E66), 0);
    lv_obj_set_style_bg_opa(header_bt_status_dot, LV_OPA_COVER, 0);
    lv_obj_clear_flag(header_bt_status_dot, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *help_button = make_gradient_button(
        header, 650, 14, 42, 42, 0x5A1830, 0x260B1B,
        LV_RADIUS_CIRCLE, 0xEF4A67, 1);
    lv_obj_add_event_cb(
        help_button, open_page_event, LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(PageKind::Help)));
    make_center_label(help_button, "?", &lv_font_montserrat_18, 0xFFFFFF);

    lv_obj_t *settings_button = make_gradient_button(
        header, 706, 12, 84, 46, 0x0B3D5A, 0x05182A,
        23, UI_CYAN, 1);
    lv_obj_add_event_cb(
        settings_button, open_page_event, LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(PageKind::SettingsHub)));
    make_label(settings_button, LV_SYMBOL_SETTINGS, 11, 11,
               &lv_font_montserrat_20, UI_CYAN);
    make_label(settings_button, "设置", 41, 12,
               &lv_font_jcy18, 0xFFFFFF);

    if (DISPLAY_DIAGNOSTICS) {
        debug_perf_panel = make_gradient_box(
            header, 220, 2, 210, 66,
            0x0B2741, 0x041325, 8, UI_BORDER, 1);
        lv_obj_set_style_bg_opa(debug_perf_panel, LV_OPA_90, 0);
        debug_perf_label = make_label(
            debug_perf_panel, "FPS:--  HEAP:--  PSRAM:--", 6, 3,
            &lv_font_montserrat_12, 0xE8FAFF);
        lv_obj_set_width(debug_perf_label, 198);
        lv_obj_set_style_text_line_space(debug_perf_label, 1, 0);
    }
    refresh_wireless_header();
}

void create_mode_column(lv_obj_t *screen) {
    lv_obj_t *panel = make_gradient_box(
        screen, 0, HEADER_HEIGHT, MODE_WIDTH,
        SCREEN_HEIGHT - HEADER_HEIGHT,
        UI_BG_TOP, UI_BG_BOTTOM, 0, UI_BORDER_DIM, 0);

    mode_buttons[0] = make_gradient_button(
        panel, 6, 7, MODE_WIDTH - 12, 192,
        0x0D4568, 0x071D35, 12, UI_CYAN, 2,
        jcy_audio::Cue::Navigate);
    lv_obj_add_event_cb(mode_buttons[0], mode_event, LV_EVENT_CLICKED,
                        const_cast<WorkMode *>(&MODE_OPTIONS[0]));
    make_battery_icon(mode_buttons[0]);
    make_label(mode_buttons[0], "25V10A", 1, 127,
               &lv_font_montserrat_20, 0xFFFFFF);
    mode_labels[0] = make_label(mode_buttons[0], "模拟电池", 1, 159,
                                &lv_font_jcy20, 0xFFFFFF);

    mode_buttons[1] = make_gradient_button(
        panel, 6, 211, MODE_WIDTH - 12, 192,
        0x071A2E, 0x030D1B, 12, UI_BORDER_DIM, 1,
        jcy_audio::Cue::Navigate);
    lv_obj_add_event_cb(mode_buttons[1], mode_event, LV_EVENT_CLICKED,
                        const_cast<WorkMode *>(&MODE_OPTIONS[1]));
    make_power_supply_icon(mode_buttons[1]);
    make_label(mode_buttons[1], "25V10A", 1, 127,
               &lv_font_montserrat_20, 0x597C9C);
    mode_labels[1] = make_label(mode_buttons[1], "双向电源", 1, 159,
                                &lv_font_jcy20, 0x597C9C);
}

lv_obj_t *add_measurement_row(lv_obj_t *panel, const char *name, const char *value,
                              int y, bool value_is_chinese = false) {
    make_label(panel, name, 43, y + 4, &lv_font_jcy16, UI_TEXT);
    const int value_y = value_is_chinese ? y + 1 : y - 4;
    lv_obj_t *value_label = make_label(
        panel, value, 0, value_y,
        value_is_chinese ? &lv_font_jcy18 : &lv_font_montserrat_24,
        0xFFFFFF);
    lv_obj_align(value_label, LV_ALIGN_TOP_RIGHT, -8, value_y);
    return value_label;
}

void create_measurement_column(lv_obj_t *screen) {
    lv_obj_t *panel = make_gradient_box(
        screen, MODE_WIDTH, HEADER_HEIGHT, METER_WIDTH,
        SCREEN_HEIGHT - HEADER_HEIGHT,
        UI_PANEL_TOP, UI_PANEL_BOTTOM, 0, UI_BORDER, 1);

    // 电池本体缩小并弱化独立卡片感，给下方电池类型/串数留出完整一行。
    measurement_battery_shell = make_gradient_box(
        panel, 18, 8, 72, 44,
        0x0B3554, 0x041426, 7, UI_CYAN, 2);
    lv_obj_set_style_bg_opa(measurement_battery_shell, LV_OPA_COVER, 0);
    lv_obj_clear_flag(measurement_battery_shell, LV_OBJ_FLAG_CLICKABLE);
    constexpr int BATTERY_CELL_COUNT = 5;
    constexpr int BATTERY_CELL_WIDTH = 10;
    constexpr int BATTERY_CELL_GAP = 3;
    constexpr int BATTERY_INNER_X = 3;
    constexpr int BATTERY_INNER_Y = 4;
    constexpr int BATTERY_INNER_HEIGHT = 36;
    for (int i = 0; i < BATTERY_CELL_COUNT; ++i) {
        lv_obj_t *cell = lv_obj_create(measurement_battery_shell);
        lv_obj_set_pos(cell,
                       BATTERY_INNER_X + i * (BATTERY_CELL_WIDTH + BATTERY_CELL_GAP),
                       BATTERY_INNER_Y);
        lv_obj_set_size(cell, BATTERY_CELL_WIDTH, BATTERY_INNER_HEIGHT);
        style_clean(cell);
        lv_obj_set_style_radius(cell, 3, 0);
        lv_obj_set_style_border_width(cell, 1, 0);
        lv_obj_set_style_border_color(cell, color(UI_BORDER_DIM), 0);
        lv_obj_set_style_bg_color(cell, color(0x0A3150), 0);
        lv_obj_set_style_bg_grad_color(cell, color(0x04172B), 0);
        lv_obj_set_style_bg_grad_dir(cell, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_CLICKABLE);
        measurement_battery_cells[i] = cell;
    }
    lv_obj_t *battery_tip = lv_obj_create(panel);
    lv_obj_set_pos(battery_tip, 90, 22);
    lv_obj_set_size(battery_tip, 5, 16);
    lv_obj_set_style_radius(battery_tip, 2, 0);
    lv_obj_set_style_border_width(battery_tip, 0, 0);
    lv_obj_set_style_bg_color(battery_tip, color(UI_CYAN), 0);
    lv_obj_set_style_bg_opa(battery_tip, LV_OPA_COVER, 0);
    lv_obj_clear_flag(battery_tip, LV_OBJ_FLAG_CLICKABLE);

    measurement_soc_label = make_label(
        panel, "0%", 108, 5,
        &lv_font_montserrat_30, 0xFFFFFF);

    make_measurement_badge(panel, "B", 8, 57,
                           0x6ED9F6, 0x367CCB);
    measurement_soc_caption_label = add_measurement_row(
        panel, "电池", "三元锂 1串", 59, true);

    status_badge = make_measurement_badge(
        panel, LV_SYMBOL_OK, 8, 90, 0x4F86B2, 0x356B98);
    status_value_label = add_measurement_row(
        panel, "状态", "等待连接", 92, true);
    lv_obj_set_size(status_value_label, 128, 30);
    lv_obj_set_style_text_align(status_value_label,
                                LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_radius(status_value_label, 15, 0);
    lv_obj_set_style_border_width(status_value_label, 1, 0);
    lv_obj_set_style_pad_top(status_value_label, 3, 0);
    lv_obj_align(status_value_label, LV_ALIGN_TOP_RIGHT, -6, 90);
    set_battery_status_visual(
        "等待连接", 0x587B98, 0x3D5E7C, 0xA9CDE5);
    make_measurement_badge(panel, "V", 8, 130,
                           0x6CD5FF, 0x3673D7);
    measurement_voltage_label = add_measurement_row(
        panel, "电压", "--.---V", 132);
    lv_obj_set_style_text_font(measurement_voltage_label,
                               &lv_font_montserrat_30, 0);
    lv_obj_set_width(measurement_voltage_label, METER_WIDTH - 72);
    lv_obj_set_style_text_align(measurement_voltage_label,
                                LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(measurement_voltage_label,
                 LV_ALIGN_TOP_RIGHT, -8, 126);
    make_measurement_badge(panel, "A", 8, 185,
                           0x62F1E4, 0x1DA2B6);
    measurement_current_label = add_measurement_row(
        panel, "电流", "--", 187);
    lv_obj_set_style_text_font(measurement_current_label,
                               &lv_font_montserrat_24, 0);
    lv_obj_set_width(measurement_current_label, METER_WIDTH - 72);
    lv_obj_set_style_text_align(measurement_current_label,
                                LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(measurement_current_label,
                 LV_ALIGN_TOP_RIGHT, -8, 181);
    measurement_temperature_label = nullptr;
    make_measurement_badge(panel, "W", 8, 240,
                           0xD593FF, 0x7556D9);
    measurement_power_label = add_measurement_row(
        panel, "功率", "--.---W", 242);
    lv_obj_set_style_text_font(measurement_power_label,
                               &lv_font_montserrat_20, 0);

    lv_obj_t *measurement_touch_area = lv_btn_create(panel);
    lv_obj_set_pos(measurement_touch_area, 0, 88);
    lv_obj_set_size(measurement_touch_area, METER_WIDTH, 194);
    style_clean(measurement_touch_area);
    lv_obj_set_style_bg_opa(measurement_touch_area, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(measurement_touch_area, 0, 0);
    lv_obj_add_event_cb(measurement_touch_area, measurement_press_event,
                        LV_EVENT_ALL, nullptr);

    // 84px 高度，接近旧版主界面的底部大按键，方便手指操作。
    power_button = make_gradient_button(
        panel, 8, 286, METER_WIDTH - 16, 84,
        0x0A3150, 0x041426, 14, UI_CYAN, 1,
        jcy_audio::Cue::Power);
    lv_obj_add_event_cb(power_button, power_event, LV_EVENT_CLICKED, nullptr);
    lv_obj_set_style_bg_grad_dir(power_button, LV_GRAD_DIR_HOR, 0);

    // 参考旧版界面：底部只保留醒目的电源开关图标，不显示“输出开启/关闭”文字。
    power_switch_label = nullptr;
    power_switch_track = nullptr;
    power_switch_knob = nullptr;

    power_icon = lv_arc_create(power_button);
    lv_obj_set_size(power_icon, 62, 62);
    lv_obj_align(power_icon, LV_ALIGN_CENTER, 0, 5);
    // LVGL 的 0° 在右侧；跨 360° 绘制，把缺口留在正上方。
    lv_arc_set_bg_angles(power_icon, 300, 240);
    lv_obj_set_style_arc_width(power_icon, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_color(power_icon, color(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(power_icon, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(power_icon, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(power_icon, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_border_width(power_icon, 0, LV_PART_KNOB);
    lv_obj_clear_flag(power_icon, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(power_icon, LV_OBJ_FLAG_SCROLLABLE);

    power_icon_stem = lv_obj_create(power_button);
    lv_obj_set_size(power_icon_stem, 8, 32);
    lv_obj_align(power_icon_stem, LV_ALIGN_CENTER, 0, -17);
    style_clean(power_icon_stem);
    lv_obj_set_style_radius(power_icon_stem, 4, 0);
    lv_obj_set_style_border_width(power_icon_stem, 0, 0);
    lv_obj_set_style_bg_color(power_icon_stem, color(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(power_icon_stem, LV_OPA_COVER, 0);
    lv_obj_clear_flag(power_icon_stem, LV_OBJ_FLAG_CLICKABLE);

    alarm_bar = make_gradient_button(
        panel, 8, 374, METER_WIDTH - 16, 30,
        0x0A3A45, 0x06232F, 8, 0x167B72, 1);
    lv_obj_add_event_cb(alarm_bar, alarm_bar_event,
                        LV_EVENT_CLICKED, nullptr);
    alarm_icon = make_label(alarm_bar, "OK", 12, 7,
                            &lv_font_montserrat_14, 0xFFFFFF);
    alarm_label = make_label(alarm_bar, "设备正常", 48, 6,
                             &lv_font_jcy16, 0xFFFFFF);
    lv_obj_set_width(alarm_label, METER_WIDTH - 66);
    lv_obj_set_style_text_align(alarm_label, LV_TEXT_ALIGN_CENTER, 0);

    refresh_power_state();
    refresh_alarm_bar(0, nullptr);
}

lv_obj_t *create_top_setting_card(lv_obj_t *parent, int x, int width,
                                  const char *line1, const char *line2,
                                  uint32_t top_color, uint32_t bottom_color,
                                  PageKind page,
                                  bool line2_is_chinese = false) {
    lv_obj_t *card = make_gradient_button(
        parent, x, 0, width, 72, top_color, bottom_color,
        10, UI_BORDER, 1, jcy_audio::Cue::Navigate);
    lv_obj_add_event_cb(
        card, open_page_event, LV_EVENT_CLICKED,
        reinterpret_cast<void *>(static_cast<intptr_t>(page)));
    lv_obj_t *title = make_label(card, line1, 0, 10,
                                 &lv_font_jcy18, 0xFFFFFF);
    lv_obj_set_width(title, width);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *value = make_label(
        card, line2, 0, 38,
        line2_is_chinese ? &lv_font_jcy18 : &lv_font_montserrat_18,
        0xFFFFFF);
    lv_obj_set_width(value, width);
    lv_obj_set_style_text_align(value, LV_TEXT_ALIGN_CENTER, 0);
    if (page == PageKind::BatterySettings) {
        battery_card_value_label = value;
    } else if (page == PageKind::CutoffTest) {
        cutoff_card_value_label = value;
    }
    return card;
}

void refresh_constant_current_card() {
    if (constant_current_card == nullptr ||
        constant_current_switch_track == nullptr ||
        constant_current_switch_knob == nullptr ||
        constant_current_overcurrent_label == nullptr ||
        constant_current_constant_label == nullptr) {
        return;
    }
    // Only the lower instrument's SETup:MODE? response may move this switch.
    // While a request is pending, retain the last verified side and use amber
    // as the explicit "confirming" state.
    const bool visual_on = constant_current_mode;
    const bool visual_active = visual_on || constant_current_change_pending;
    if (REFERENCE_SKIN_UI) {
        lv_obj_set_style_bg_opa(
            constant_current_card,
            visual_active ? LV_OPA_90 : LV_OPA_TRANSP,
            0);
        lv_obj_set_style_border_width(
            constant_current_card, visual_active ? 2 : 0, 0);
    }
    // 按下并亮起表示恒流模式；弹起表示普通过流保护模式。
    // 选中态加深背景、变粗边框、加亮青色光晕并整体下移2px，
    // 模拟实体自锁按键被按下并持续点亮。
    lv_obj_set_style_translate_y(constant_current_card,
                                 visual_on ? 2 : 0, 0);
    if (visual_active) {
        lv_obj_set_style_shadow_width(constant_current_card, 14, 0);
        lv_obj_set_style_shadow_ofs_y(constant_current_card, 0, 0);
        lv_obj_set_style_shadow_color(constant_current_card,
                                      color(0x18D7A2), 0);
        lv_obj_set_style_shadow_opa(constant_current_card,
                                    LV_OPA_90, 0);
    } else {
        lv_obj_set_style_shadow_width(constant_current_card, 6, 0);
        lv_obj_set_style_shadow_ofs_y(constant_current_card, 3, 0);
        lv_obj_set_style_shadow_color(constant_current_card,
                                      color(0x00101D), 0);
        lv_obj_set_style_shadow_opa(constant_current_card,
                                    LV_OPA_60, 0);
    }
    lv_obj_set_style_bg_color(
        constant_current_card,
        color(constant_current_change_pending
                  ? 0xF0A44A
                  : (visual_on ? 0x0BAE83 : UI_CARD_TOP)), 0);
    lv_obj_set_style_bg_grad_color(
        constant_current_card,
        color(constant_current_change_pending
                  ? 0xD46B32
                  : (visual_on ? 0x075849 : UI_CARD_BOTTOM)), 0);
    lv_obj_set_style_border_color(
        constant_current_card,
        color(visual_on ? 0x50FFD2 : UI_BORDER), 0);
    if (constant_current_title_label != nullptr) {
        lv_label_set_text(constant_current_title_label,
                          constant_current_change_pending
                              ? "模式确认中"
                              : "电流模式");
        lv_obj_set_style_text_color(
            constant_current_title_label,
            color(visual_active ? 0xFFFFFF : 0xD8F5FF), 0);
        if (REFERENCE_SKIN_UI) {
            lv_obj_set_style_bg_color(
                constant_current_title_label,
                color(constant_current_change_pending
                          ? 0xF0A44A
                          : (visual_on ? 0x0BAE83 : 0x0A3150)), 0);
        }
    }

    const int track_width = lv_obj_get_width(constant_current_switch_track);
    const int track_height = lv_obj_get_height(constant_current_switch_track);
    const int knob_width = max(1, (track_width - 4) / 2);
    lv_obj_set_size(constant_current_switch_knob,
                    knob_width, max(1, track_height - 4));
    lv_obj_set_pos(constant_current_switch_knob,
                   visual_on ? 2 + knob_width : 2, 2);
    lv_obj_set_style_bg_color(
        constant_current_switch_knob,
        color(constant_current_change_pending
                  ? 0xF29B38
                  : (visual_on ? 0x18D7A2 : 0x2178B9)), 0);
    lv_obj_set_style_bg_grad_color(
        constant_current_switch_knob,
        color(constant_current_change_pending
                  ? 0xC65E27
                  : (visual_on ? 0x078B70 : 0x11446D)), 0);
    lv_obj_set_style_border_color(
        constant_current_switch_track,
        color(constant_current_change_pending
                  ? 0xFFD596
                  : (visual_on ? 0x50FFD2 : 0x3EA9E8)), 0);
    lv_obj_set_style_text_color(
        constant_current_overcurrent_label,
        color(!visual_on ? 0xFFFFFF : 0x6B8AA6), 0);
    lv_obj_set_style_text_color(
        constant_current_constant_label,
        color(visual_on ? 0xFFFFFF : 0x6B8AA6), 0);
    // 选中态把“恒流”放大并加粗，让用户一眼能看出当前模式。
    lv_obj_set_style_text_font(
        constant_current_constant_label,
        visual_on ? &lv_font_jcy18 : &lv_font_jcy16, 0);
    lv_obj_set_style_text_font(
        constant_current_overcurrent_label,
        visual_on ? &lv_font_jcy16 : &lv_font_jcy16, 0);
    // 选中态在“恒流”标签下方加一条亮青色高亮条，强化开关打到右边时的视觉反馈。
    if (constant_current_active_underline == nullptr) {
        constant_current_active_underline = lv_obj_create(
            constant_current_card);
        style_clean(constant_current_active_underline);
        lv_obj_set_style_radius(constant_current_active_underline, 2, 0);
        lv_obj_set_style_border_width(
            constant_current_active_underline, 0, 0);
        lv_obj_set_style_bg_opa(
            constant_current_active_underline, LV_OPA_TRANSP, 0);
        lv_obj_clear_flag(constant_current_active_underline,
                          LV_OBJ_FLAG_CLICKABLE);
    }
    lv_obj_set_size(constant_current_active_underline,
                    visual_on ? 36 : 0, 3);
    lv_obj_align(constant_current_active_underline,
                 LV_ALIGN_BOTTOM_MID, 30, -4);
    lv_obj_set_style_bg_color(
        constant_current_active_underline,
        color(0x50FFD2), 0);
    lv_obj_set_style_bg_opa(
        constant_current_active_underline,
        visual_on ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
}

void create_constant_current_switch(lv_obj_t *parent,
                                    int x, int y,
                                    int width, int height) {
    constant_current_switch_track = lv_obj_create(parent);
    lv_obj_set_pos(constant_current_switch_track, x, y);
    lv_obj_set_size(constant_current_switch_track, width, height);
    style_clean(constant_current_switch_track);
    lv_obj_set_style_radius(constant_current_switch_track, height / 2, 0);
    lv_obj_set_style_border_width(constant_current_switch_track, 1, 0);
    lv_obj_set_style_bg_color(constant_current_switch_track,
                              color(0x071A2D), 0);
    lv_obj_set_style_bg_opa(constant_current_switch_track, LV_OPA_COVER, 0);
    lv_obj_clear_flag(constant_current_switch_track, LV_OBJ_FLAG_CLICKABLE);

    constant_current_switch_knob = lv_obj_create(
        constant_current_switch_track);
    style_clean(constant_current_switch_knob);
    lv_obj_set_style_radius(constant_current_switch_knob,
                            max(1, (height - 4) / 2), 0);
    lv_obj_set_style_border_width(constant_current_switch_knob, 0, 0);
    lv_obj_set_style_bg_opa(constant_current_switch_knob, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_grad_dir(constant_current_switch_knob,
                                 LV_GRAD_DIR_VER, 0);
    lv_obj_clear_flag(constant_current_switch_knob, LV_OBJ_FLAG_CLICKABLE);

    constant_current_overcurrent_label = lv_label_create(
        constant_current_switch_track);
    lv_label_set_text(constant_current_overcurrent_label, "过流");
    lv_obj_set_pos(constant_current_overcurrent_label, 0, 2);
    lv_obj_set_size(constant_current_overcurrent_label, width / 2,
                    max(1, height - 4));
    lv_obj_set_style_text_font(constant_current_overcurrent_label,
                               &lv_font_jcy16, 0);
    lv_obj_set_style_text_align(constant_current_overcurrent_label,
                                LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_clear_flag(constant_current_overcurrent_label,
                      LV_OBJ_FLAG_CLICKABLE);

    constant_current_constant_label = lv_label_create(
        constant_current_switch_track);
    lv_label_set_text(constant_current_constant_label, "恒流");
    lv_obj_set_pos(constant_current_constant_label, width / 2, 2);
    lv_obj_set_size(constant_current_constant_label,
                    width - width / 2, max(1, height - 4));
    lv_obj_set_style_text_font(constant_current_constant_label,
                               &lv_font_jcy16, 0);
    lv_obj_set_style_text_align(constant_current_constant_label,
                                LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_clear_flag(constant_current_constant_label,
                      LV_OBJ_FLAG_CLICKABLE);
}

void create_constant_current_card(lv_obj_t *parent, int x, int width) {
    constant_current_card = make_gradient_button(
        parent, x, 0, width, 72,
        UI_CARD_TOP, UI_CARD_BOTTOM, 10, UI_BORDER, 1,
        jcy_audio::Cue::Toggle);
    lv_obj_add_event_cb(constant_current_card, constant_current_event,
                        LV_EVENT_CLICKED, nullptr);
    constant_current_title_label = make_label(
        constant_current_card, "电流模式", 0, 10,
        &lv_font_jcy18, 0xFFFFFF);
    lv_obj_set_width(constant_current_title_label, width);
    lv_obj_set_style_text_align(constant_current_title_label,
                                LV_TEXT_ALIGN_CENTER, 0);
    constant_current_card_value_label = nullptr;
    create_constant_current_switch(
        constant_current_card, 10, 37, width - 20, 27);
    refresh_constant_current_card();
}

void make_adjust_icon(lv_obj_t *parent, bool plus) {
    lv_obj_t *horizontal = lv_obj_create(parent);
    lv_obj_set_size(horizontal, 30, 5);
    lv_obj_align(horizontal, LV_ALIGN_CENTER, 0, 0);
    style_clean(horizontal);
    lv_obj_set_style_radius(horizontal, 3, 0);
    lv_obj_set_style_bg_color(horizontal, color(UI_CYAN), 0);
    lv_obj_set_style_bg_opa(horizontal, LV_OPA_COVER, 0);
    lv_obj_clear_flag(horizontal, LV_OBJ_FLAG_CLICKABLE);
    if (plus) {
        lv_obj_t *vertical = lv_obj_create(parent);
        lv_obj_set_size(vertical, 5, 30);
        lv_obj_align(vertical, LV_ALIGN_CENTER, 0, 0);
        style_clean(vertical);
        lv_obj_set_style_radius(vertical, 3, 0);
        lv_obj_set_style_bg_color(vertical, color(UI_CYAN), 0);
        lv_obj_set_style_bg_opa(vertical, LV_OPA_COVER, 0);
        lv_obj_clear_flag(vertical, LV_OBJ_FLAG_CLICKABLE);
    }
}

void create_voltage_editor(lv_obj_t *panel) {
    lv_obj_t *editor = make_gradient_box(
        panel, 10, 80, 470, 205,
        UI_PANEL_TOP, UI_PANEL_BOTTOM, 12, UI_BORDER, 1);
    make_label(editor, "设置电压 / 微调", 14, 8,
               &lv_font_jcy18, 0xFFFFFF);
    lv_obj_t *hint = make_label(editor, "单击选位  双击数字直接输入", 188, 10,
                                &lv_font_jcy16, UI_MUTED);
    lv_obj_set_width(hint, 265);
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_RIGHT, 0);

    lv_obj_t *minus = make_gradient_button(
        editor, 12, 42, 68, 96,
        0x0B4161, 0x051A2D, 8, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(minus, voltage_step_event, LV_EVENT_CLICKED,
                        const_cast<int *>(&VOLTAGE_STEP_DIRECTIONS[0]));
    make_adjust_icon(minus, false);

    lv_obj_t *value_box = make_gradient_box(
        editor, 88, 42, 294, 96,
        0x071B35, 0x020913, 8, UI_BORDER, 1);
    constexpr int digit_x[5] = {10, 57, 120, 167, 214};
    for (size_t i = 0; i < 5; ++i) {
        set_voltage_digit_boxes[i] = make_gradient_button(
            value_box, digit_x[i], 4, 44, 88,
            0x071A31, 0x030D1C, 5, UI_BORDER_DIM, 1,
            jcy_audio::Cue::Adjust);
        lv_obj_add_event_cb(
            set_voltage_digit_boxes[i], voltage_digit_select_event,
            LV_EVENT_CLICKED,
            const_cast<uint8_t *>(&VOLTAGE_DIGIT_INDICES[i]));
        set_voltage_digit_labels[i] = make_center_label(
            set_voltage_digit_boxes[i], "0", &lv_font_montserrat_48,
            UI_TEXT);
    }
    make_label(value_box, ".", 101, 20,
               &lv_font_montserrat_48, UI_TEXT);
    make_label(value_box, "V", 270, 34,
               &lv_font_montserrat_24, UI_MUTED);

    lv_obj_t *plus = make_gradient_button(
        editor, 390, 42, 68, 96,
        0x0B4161, 0x051A2D, 8, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(plus, voltage_step_event, LV_EVENT_CLICKED,
                        const_cast<int *>(&VOLTAGE_STEP_DIRECTIONS[1]));
    make_adjust_icon(plus, true);

    minimum_voltage_label = make_label(
        editor, "最小 3.000V", 14, 166,
        &lv_font_jcy16, UI_MUTED);
    lv_obj_set_width(minimum_voltage_label, 190);
    maximum_voltage_label = make_label(
        editor, "最大 4.250V", 266, 166,
        &lv_font_jcy16, UI_MUTED);
    lv_obj_set_width(maximum_voltage_label, 190);
    lv_obj_set_style_text_align(maximum_voltage_label, LV_TEXT_ALIGN_RIGHT, 0);
    set_voltage_header_label = nullptr;
}

void create_quick_voltage_buttons(lv_obj_t *panel) {
    compute_quick_voltages();
    for (size_t i = 0; i < 6; ++i) {
        const int col = static_cast<int>(i % 3);
        const int row = static_cast<int>(i / 3);
        quick_voltage_buttons[i] = make_gradient_button(
            panel, col * 163, 293 + row * 58,
            col == 2 ? 164 : 163, row == 1 ? 59 : 58,
            UI_CARD_TOP, UI_CARD_BOTTOM, 6, UI_BORDER_DIM, 1,
            jcy_audio::Cue::Adjust);
        lv_obj_add_event_cb(quick_voltage_buttons[i], quick_voltage_event,
                            LV_EVENT_CLICKED, &quick_voltages_mv[i]);
        quick_voltage_labels[i] = make_center_label(
            quick_voltage_buttons[i], "", &lv_font_montserrat_16, UI_TEXT);
    }
}

void append_main_waveform_sample(const JcyScpiMeasurements &measurements) {
    if (main_wave_chart == nullptr || main_voltage_series == nullptr ||
        main_current_series == nullptr) {
        return;
    }
    const int current_ma = measurements.current_ua / 1000;
    if (abs(current_ma) * 10 >= main_wave_current_range_ma * 8) {
        main_wave_current_range_ma = min(
            max(main_wave_current_range_ma * 2, abs(current_ma) * 2),
            max(1000, battery_settings.overcurrent_ma));
        lv_chart_set_range(main_wave_chart, LV_CHART_AXIS_SECONDARY_Y,
                           -main_wave_current_range_ma,
                           main_wave_current_range_ma);
    }
    lv_chart_set_next_value(main_wave_chart, main_voltage_series,
                            measurements.voltage_mv);
    lv_chart_set_next_value(main_wave_chart, main_current_series,
                            current_ma);
    if (main_wave_voltage_label != nullptr) {
        char voltage_text[24] = {};
        snprintf(voltage_text, sizeof(voltage_text), "V %d.%03dV",
                 measurements.voltage_mv / 1000,
                 abs(measurements.voltage_mv % 1000));
        set_adaptive_numeric_label(main_wave_voltage_label,
                                   voltage_text, 16);
    }
    if (main_wave_current_label != nullptr) {
        char current_text[24] = {};
        char display_text[28] = {};
        format_current_text(current_text, sizeof(current_text),
                            measurements.current_ua);
        snprintf(display_text, sizeof(display_text), "I %s", current_text);
        set_adaptive_numeric_label(main_wave_current_label,
                                   display_text, 16);
    }
}

void refresh_main_waveform_visibility() {
    if (settings_idle_content == nullptr ||
        settings_wave_content == nullptr) {
        return;
    }
    // During a rapid toggle, follow the user's latest intent instead of a
    // stale intermediate ACK. This avoids briefly flashing the waveform when
    // ON is acknowledged after the user has already requested OFF.
    const bool show_waveform = output_change_pending
                                   ? requested_output_state
                                   : output_running;
    if (show_waveform == main_waveform_visible) {
        return;
    }
    main_waveform_visible = show_waveform;
    if (show_waveform) {
        lv_obj_add_flag(settings_idle_content, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(settings_wave_content, LV_OBJ_FLAG_HIDDEN);
        main_waveform_last_sequence = 0;
        if (main_wave_chart != nullptr) {
            lv_chart_set_range(main_wave_chart, LV_CHART_AXIS_PRIMARY_Y,
                               max(0, pack_minimum_mv() - 300),
                               min(MAX_BATTERY_PACK_MV,
                                   pack_maximum_mv() + 300));
            main_wave_current_range_ma = min(
                1000, max(100, battery_settings.overcurrent_ma));
            lv_chart_set_range(main_wave_chart,
                               LV_CHART_AXIS_SECONDARY_Y,
                               -main_wave_current_range_ma,
                               main_wave_current_range_ma);
            lv_chart_set_all_value(main_wave_chart, main_voltage_series,
                                   LV_CHART_POINT_NONE);
            lv_chart_set_all_value(main_wave_chart, main_current_series,
                                   LV_CHART_POINT_NONE);
            lv_chart_refresh(main_wave_chart);
        }
    } else {
        lv_obj_clear_flag(settings_idle_content, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(settings_wave_content, LV_OBJ_FLAG_HIDDEN);
    }
    restore_default_poll_interval();
}

void create_main_waveform(lv_obj_t *parent) {
    settings_wave_content = lv_obj_create(parent);
    lv_obj_set_pos(settings_wave_content, 0, 0);
    lv_obj_set_size(settings_wave_content,
                    SCREEN_WIDTH - SETTINGS_X,
                    SCREEN_HEIGHT - HEADER_HEIGHT);
    style_clean(settings_wave_content);
    lv_obj_set_style_bg_opa(settings_wave_content, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(settings_wave_content, 0, 0);

    lv_obj_t *wave_panel = make_gradient_box(
        settings_wave_content, 10, 80, 470, 316,
        UI_PANEL_TOP, UI_PANEL_BOTTOM, 10, UI_BORDER, 1);
    make_label(wave_panel, "实时波形", 14, 10,
               &lv_font_jcy18, 0xFFFFFF);
    main_wave_voltage_label = make_label(
        wave_panel, "V --.---V", 128, 11,
        &lv_font_montserrat_16, 0x65F2DB);
    main_wave_current_label = make_label(
        wave_panel, "I --", 318, 11,
        &lv_font_montserrat_16, 0xFFB46A);

    main_wave_chart = lv_chart_create(wave_panel);
    lv_obj_set_pos(main_wave_chart, 12, 42);
    lv_obj_set_size(main_wave_chart, 446, 232);
    lv_chart_set_type(main_wave_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_update_mode(main_wave_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_point_count(main_wave_chart, 240);
    lv_chart_set_range(main_wave_chart, LV_CHART_AXIS_PRIMARY_Y,
                       max(0, pack_minimum_mv() - 300),
                       min(MAX_BATTERY_PACK_MV, pack_maximum_mv() + 300));
    main_wave_current_range_ma = min(
        1000, max(100, battery_settings.overcurrent_ma));
    lv_chart_set_range(main_wave_chart, LV_CHART_AXIS_SECONDARY_Y,
                       -main_wave_current_range_ma,
                       main_wave_current_range_ma);
    lv_chart_set_div_line_count(main_wave_chart, 6, 11);
    lv_obj_set_style_bg_color(main_wave_chart, color(0x0C2853), 0);
    lv_obj_set_style_border_color(main_wave_chart, color(0x3D83B8), 0);
    lv_obj_set_style_line_color(main_wave_chart, color(0x285683), LV_PART_MAIN);
    lv_obj_set_style_line_width(main_wave_chart, 1, LV_PART_MAIN);
    lv_obj_set_style_line_width(main_wave_chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(main_wave_chart, 0, LV_PART_INDICATOR);
    main_voltage_series = lv_chart_add_series(
        main_wave_chart, color(0x65F2DB), LV_CHART_AXIS_PRIMARY_Y);
    main_current_series = lv_chart_add_series(
        main_wave_chart, color(0xFFB46A), LV_CHART_AXIS_SECONDARY_Y);

    make_label(wave_panel, "电压(绿) / 电流(橙)   100ms采样   约24s窗口",
               14, 286, &lv_font_jcy16, 0xCDEBFA);
    lv_obj_add_flag(settings_wave_content, LV_OBJ_FLAG_HIDDEN);
}

void create_settings_column(lv_obj_t *screen) {
    lv_obj_t *panel = make_gradient_box(
        screen, SETTINGS_X, HEADER_HEIGHT,
        SCREEN_WIDTH - SETTINGS_X, SCREEN_HEIGHT - HEADER_HEIGHT,
        UI_BG_TOP, UI_BG_BOTTOM, 0, UI_BORDER_DIM, 1);

    settings_idle_content = lv_obj_create(panel);
    lv_obj_set_pos(settings_idle_content, 0, 0);
    lv_obj_set_size(settings_idle_content,
                    SCREEN_WIDTH - SETTINGS_X,
                    SCREEN_HEIGHT - HEADER_HEIGHT);
    style_clean(settings_idle_content);
    lv_obj_set_style_bg_opa(settings_idle_content, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(settings_idle_content, 0, 0);
    lv_obj_clear_flag(settings_idle_content, LV_OBJ_FLAG_CLICKABLE);
    create_voltage_editor(settings_idle_content);
    create_quick_voltage_buttons(settings_idle_content);
    create_main_waveform(panel);

    create_constant_current_card(panel, 5, 154);
    create_top_setting_card(panel, 165, 158,
                            "电池设置", "1节电池",
                            0x0D748D, 0x073A51,
                            PageKind::BatterySettings, true);
    create_top_setting_card(panel, 329, 156,
                            "截止电压测试", "过充 / 过放",
                            UI_CARD_TOP, UI_CARD_BOTTOM,
                            PageKind::CutoffTest, true);
    refresh_main_waveform_visibility();
}

lv_obj_t *make_skin_hitbox(lv_obj_t *parent, int x, int y,
                           int width, int height,
                           lv_event_cb_t event_callback,
                           void *user_data = nullptr,
                           jcy_audio::Cue sound = jcy_audio::Cue::Click) {
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, width, height);
    style_clean(button);
    lv_obj_set_style_radius(button, 0, 0);
    lv_obj_set_style_border_width(button, 0, 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_10, LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(button, color(UI_CYAN), LV_STATE_PRESSED);
    if (event_callback != nullptr) {
        lv_obj_add_event_cb(button, event_callback, LV_EVENT_CLICKED,
                            user_data);
    }
    attach_button_sound(button, sound);
    return button;
}

lv_obj_t *make_skin_cover_label(lv_obj_t *parent, const char *text,
                                int x, int y, int width, int height,
                                const lv_font_t *font,
                                uint32_t text_color,
                                uint32_t background_color,
                                lv_text_align_t align = LV_TEXT_ALIGN_CENTER,
                                int radius = 0) {
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_size(label, width, height);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color(text_color), 0);
    lv_obj_set_style_text_align(label, align, 0);
    lv_obj_set_style_bg_color(label, color(background_color), 0);
    lv_obj_set_style_bg_opa(label, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(label, radius, 0);
    lv_obj_set_style_pad_top(label, 2, 0);
    lv_obj_clear_flag(label, LV_OBJ_FLAG_CLICKABLE);
    return label;
}

void create_skin_header(lv_obj_t *screen) {
    // 顶部压缩到58px；隐藏调试数据和产品名，只保留固件版本号。
    if (DISPLAY_DIAGNOSTICS) {
        debug_perf_panel = make_gradient_box(
            screen, 185, 2, 174, 52,
            0x061D35, 0x031125, 0, 0x000000, 0);
        debug_perf_label = make_label(
            debug_perf_panel, JCY1000_FIRMWARE_VERSION " FPS:--/--", 6, 2,
            &lv_font_montserrat_12, 0xE8FAFF);
        lv_obj_set_width(debug_perf_label, 164);
        lv_obj_set_style_text_line_space(debug_perf_label, 0, 0);
    }

    lv_obj_t *version = make_label(
        screen, JCY1000_FIRMWARE_VERSION, 350, 19,
        &lv_font_montserrat_16, UI_MUTED);
    lv_obj_set_width(version, 160);
    lv_obj_set_style_text_align(version, LV_TEXT_ALIGN_CENTER, 0);

    header_wifi_button = make_skin_hitbox(
        screen, 532, skin_header_y(14), 49, 48, open_page_event,
        reinterpret_cast<void *>(static_cast<intptr_t>(PageKind::Wifi)));
    header_bt_button = make_skin_hitbox(
        screen, 584, skin_header_y(14), 45, 48, open_page_event,
        reinterpret_cast<void *>(static_cast<intptr_t>(PageKind::Bluetooth)));
    make_skin_hitbox(
        screen, 634, skin_header_y(12), 51, 51, open_page_event,
        reinterpret_cast<void *>(static_cast<intptr_t>(PageKind::Help)));
    make_skin_hitbox(
        screen, 694, skin_header_y(10), 88, 54, open_page_event,
        reinterpret_cast<void *>(static_cast<intptr_t>(PageKind::SettingsHub)));

    // 参考底图的无线图标是静态图片，单靠颜色无法判断是否已经连接。
    // 在不改变图标位置的前提下加状态角标：绿色“已连”、蓝色“待连/广播”、
    // 灰色“关闭”、橙色“连接”、红色“错误”。
    header_wifi_icon = nullptr;
    memset(header_wifi_bars, 0, sizeof(header_wifi_bars));
    header_wifi_status_badge = lv_obj_create(screen);
    lv_obj_set_pos(header_wifi_status_badge, 530, 42);
    lv_obj_set_size(header_wifi_status_badge, 53, 15);
    style_clean(header_wifi_status_badge);
    lv_obj_set_style_radius(header_wifi_status_badge, 5, 0);
    lv_obj_set_style_border_width(header_wifi_status_badge, 1, 0);
    lv_obj_set_style_bg_opa(header_wifi_status_badge, LV_OPA_90, 0);
    lv_obj_clear_flag(header_wifi_status_badge, LV_OBJ_FLAG_CLICKABLE);
    header_wifi_status_label = make_center_label(
        header_wifi_status_badge, "关闭", &lv_font_jcy16, 0xFFFFFF);
    lv_obj_clear_flag(header_wifi_status_label, LV_OBJ_FLAG_CLICKABLE);

    header_bt_status_badge = lv_obj_create(screen);
    lv_obj_set_pos(header_bt_status_badge, 582, 42);
    lv_obj_set_size(header_bt_status_badge, 50, 15);
    style_clean(header_bt_status_badge);
    lv_obj_set_style_radius(header_bt_status_badge, 5, 0);
    lv_obj_set_style_border_width(header_bt_status_badge, 1, 0);
    lv_obj_set_style_bg_opa(header_bt_status_badge, LV_OPA_90, 0);
    lv_obj_clear_flag(header_bt_status_badge, LV_OBJ_FLAG_CLICKABLE);
    header_bt_status_label = make_center_label(
        header_bt_status_badge, "关闭", &lv_font_jcy16, 0xFFFFFF);
    lv_obj_clear_flag(header_bt_status_label, LV_OBJ_FLAG_CLICKABLE);

    header_bt_status_dot = lv_obj_create(header_bt_button);
    lv_obj_set_pos(header_bt_status_dot, 32, 3);
    lv_obj_set_size(header_bt_status_dot, 9, 9);
    style_clean(header_bt_status_dot);
    lv_obj_set_style_radius(header_bt_status_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(header_bt_status_dot, 1, 0);
    lv_obj_set_style_border_color(header_bt_status_dot, color(0xFFFFFF), 0);
    lv_obj_set_style_bg_color(header_bt_status_dot, color(0x555E66), 0);
    lv_obj_set_style_bg_opa(header_bt_status_dot, LV_OPA_COVER, 0);
    refresh_wireless_header();
}

void create_skin_mode_hitboxes(lv_obj_t *screen) {
    memset(mode_buttons, 0, sizeof(mode_buttons));
    memset(mode_labels, 0, sizeof(mode_labels));
    mode_buttons[0] = make_skin_hitbox(
        screen, 20, skin_body_y(88), 106,
        skin_body_height(88, 195), mode_event,
        const_cast<WorkMode *>(&MODE_OPTIONS[0]),
        jcy_audio::Cue::Navigate);
    mode_buttons[1] = make_skin_hitbox(
        screen, 20, skin_body_y(288), 106,
        skin_body_height(288, 174), mode_event,
        const_cast<WorkMode *>(&MODE_OPTIONS[1]),
        jcy_audio::Cue::Navigate);

    // 参考底图中原规格为20V10A；覆盖为当前设备规格25V10A。
    make_skin_cover_label(
        screen, "25V10A", 31, skin_body_y(216), 84,
        skin_body_height(216, 31),
        &lv_font_montserrat_20, 0xFFFFFF, 0x011833,
        LV_TEXT_ALIGN_CENTER);
    make_skin_cover_label(
        screen, "25V10A", 31, skin_body_y(391), 84,
        skin_body_height(391, 31),
        &lv_font_montserrat_20, 0xFFFFFF, 0x00142C,
        LV_TEXT_ALIGN_CENTER);
    refresh_mode_buttons();
}

void create_skin_measurements(lv_obj_t *screen) {
    // SOC电池图标覆盖参考图中的静态空电池，使电量格可实时变化。
    measurement_battery_shell = make_gradient_box(
        screen, 145, skin_body_y(97), 90, skin_body_height(97, 48),
        0x0B3554, 0x041426, 6, UI_CYAN, 1);
    constexpr int CELL_WIDTH = 13;
    for (int i = 0; i < 5; ++i) {
        measurement_battery_cells[i] = make_gradient_box(
            measurement_battery_shell, 5 + i * 16, 5,
            CELL_WIDTH, skin_body_height(98, 38),
            0x174D7B, 0x113C63, 2, 0x2B6B99, 1);
    }
    lv_obj_t *battery_tip = make_gradient_box(
        screen, 235, skin_body_y(112), 5, skin_body_height(112, 17),
        UI_CYAN, UI_CYAN, 2, UI_CYAN, 0);
    lv_obj_clear_flag(battery_tip, LV_OBJ_FLAG_CLICKABLE);

    measurement_soc_label = make_skin_cover_label(
        screen, "0%", 240, skin_body_y(98), 76,
        skin_body_height(98, 38),
        &lv_font_montserrat_30, 0xFFFFFF, 0x05162A,
        LV_TEXT_ALIGN_LEFT);
    measurement_soc_caption_label = make_skin_cover_label(
        screen, "三元锂 1串", 242, skin_body_y(151), 78,
        skin_body_height(151, 27),
        &lv_font_jcy16, 0xFFFFFF, 0x041529,
        LV_TEXT_ALIGN_RIGHT);

    status_badge = nullptr;
    status_value_label = make_skin_cover_label(
        screen, "等待连接", 236, skin_body_y(180), 84,
        skin_body_height(180, 29),
        &lv_font_jcy16, 0xFFFFFF, 0x356B98,
        LV_TEXT_ALIGN_CENTER, 14);
    lv_obj_set_style_border_width(status_value_label, 1, 0);
    set_battery_status_visual(
        "等待连接", 0x587B98, 0x3D5E7C, 0xA9CDE5);

    // 覆盖参考底图中原有的温度行，重新排成更醒目的电压/电流/功率三行。
    lv_obj_t *live_meter = make_gradient_box(
        screen, 143, skin_body_y(211), 183, skin_body_height(211, 143),
        0x061B31, 0x031428, 5, 0x123A57, 1);
    lv_obj_clear_flag(live_meter, LV_OBJ_FLAG_CLICKABLE);
    make_gradient_box(live_meter, 6, 49, 171, 1,
                      0x154563, 0x154563, 0, 0x154563, 0);
    make_gradient_box(live_meter, 6, 100, 171, 1,
                      0x154563, 0x154563, 0, 0x154563, 0);

    make_measurement_badge(live_meter, "V", 7, 9,
                           0x6CD5FF, 0x3673D7);
    measurement_voltage_label = make_label(
        live_meter, "--.---V", 36, 0,
        &lv_font_montserrat_36, 0xFFFFFF);
    lv_obj_set_width(measurement_voltage_label, 141);
    lv_obj_set_style_text_align(measurement_voltage_label,
                                LV_TEXT_ALIGN_RIGHT, 0);

    make_measurement_badge(live_meter, "A", 7, 60,
                           0x62F1E4, 0x1DA2B6);
    measurement_current_limit_label = make_label(
        live_meter, "MAX 10.000A", 38, 51,
        &lv_font_montserrat_12, 0x77BFD5);
    lv_obj_set_width(measurement_current_limit_label, 137);
    lv_obj_set_style_text_align(measurement_current_limit_label,
                                LV_TEXT_ALIGN_LEFT, 0);
    measurement_current_label = make_label(
        live_meter, "--", 36, 64,
        &lv_font_montserrat_30, 0xFFFFFF);
    lv_obj_set_width(measurement_current_label, 141);
    lv_obj_set_style_text_align(measurement_current_label,
                                LV_TEXT_ALIGN_RIGHT, 0);

    measurement_temperature_label = nullptr;
    make_measurement_badge(live_meter, "W", 7, 110,
                           0xD593FF, 0x7556D9);
    measurement_power_label = make_label(
        live_meter, "--.---W", 36, 102,
        &lv_font_montserrat_30, 0xFFFFFF);
    lv_obj_set_width(measurement_power_label, 141);
    lv_obj_set_style_text_align(measurement_power_label,
                                LV_TEXT_ALIGN_RIGHT, 0);

    lv_obj_t *measurement_touch_area = lv_btn_create(screen);
    lv_obj_set_pos(measurement_touch_area, 140, skin_body_y(176));
    lv_obj_set_size(measurement_touch_area, 186,
                    skin_body_height(176, 177));
    style_clean(measurement_touch_area);
    lv_obj_set_style_bg_opa(measurement_touch_area, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(measurement_touch_area, 0, 0);
    lv_obj_add_event_cb(measurement_touch_area, measurement_press_event,
                        LV_EVENT_ALL, nullptr);

    power_button = make_gradient_button(
        screen, 143, skin_body_y(357), 183, skin_body_height(357, 69),
        0x0A3150, 0x041426, 12, UI_CYAN, 1,
        jcy_audio::Cue::Power);
    lv_obj_add_event_cb(power_button, power_event, LV_EVENT_CLICKED, nullptr);
    lv_obj_set_style_bg_grad_dir(power_button, LV_GRAD_DIR_HOR, 0);
    power_switch_label = nullptr;
    power_switch_track = nullptr;
    power_switch_knob = nullptr;

    power_icon = lv_arc_create(power_button);
    lv_obj_set_size(power_icon, 54, 54);
    lv_obj_align(power_icon, LV_ALIGN_CENTER, 0, 4);
    lv_arc_set_bg_angles(power_icon, 300, 240);
    lv_obj_set_style_arc_width(power_icon, 7, LV_PART_MAIN);
    lv_obj_set_style_arc_color(power_icon, color(UI_CYAN), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(power_icon, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(power_icon, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(power_icon, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_border_width(power_icon, 0, LV_PART_KNOB);
    lv_obj_clear_flag(power_icon, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(power_icon, LV_OBJ_FLAG_SCROLLABLE);
    power_icon_stem = make_gradient_box(
        power_button, 88, 5, 7, 27,
        UI_CYAN, UI_CYAN, 4, UI_CYAN, 0);

    alarm_bar = make_gradient_button(
        screen, 143, skin_body_y(429), 183, skin_body_height(429, 30),
        0x0A3A45, 0x06232F, 7, 0x167B72, 1);
    lv_obj_add_event_cb(alarm_bar, alarm_bar_event,
                        LV_EVENT_CLICKED, nullptr);
    alarm_icon = make_label(alarm_bar, "OK", 10, 6,
                            &lv_font_montserrat_14, 0xFFFFFF);
    alarm_label = make_label(alarm_bar, "设备正常", 43, 5,
                             &lv_font_jcy16, 0xFFFFFF);
    lv_obj_set_width(alarm_label, 128);
    lv_obj_set_style_text_align(alarm_label, LV_TEXT_ALIGN_CENTER, 0);
    refresh_power_state();
    refresh_alarm_bar(0, nullptr);
}

void create_skin_top_cards(lv_obj_t *screen) {
    constant_current_card = make_skin_hitbox(
        screen, 340, skin_body_y(87), 147, skin_body_height(87, 63),
        constant_current_event, nullptr);
    constant_current_card_value_label = nullptr;
    // 覆盖参考底图中的旧标题；下方开关明确显示“过流 / 恒流”。
    constant_current_title_label = make_skin_cover_label(
        constant_current_card, "电流模式", 5, 5, 137, 24,
        &lv_font_jcy16, 0xD8F5FF, 0x0A3150,
        LV_TEXT_ALIGN_CENTER, 4);
    create_constant_current_switch(
        constant_current_card, 15, 33, 117, 25);

    lv_obj_t *battery_card = make_skin_hitbox(
        screen, 490, skin_body_y(87), 136, skin_body_height(87, 63),
        open_page_event,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(PageKind::BatterySettings)));
    battery_card_value_label = make_skin_cover_label(
        battery_card, "1节电池", 44, 33, 82, 25,
        &lv_font_jcy16, 0xFFFFFF, 0x07506A,
        LV_TEXT_ALIGN_CENTER, 2);

    lv_obj_t *cutoff_card = make_skin_hitbox(
        screen, 629, skin_body_y(87), 152, skin_body_height(87, 63),
        open_page_event,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(PageKind::CutoffTest)));
    cutoff_card_value_label = make_skin_cover_label(
        cutoff_card, "过充 / 过放", 52, 33, 92, 25,
        &lv_font_jcy16, 0xFFFFFF, 0x071D35,
        LV_TEXT_ALIGN_CENTER, 2);
    refresh_constant_current_card();
}

void create_skin_voltage_editor(lv_obj_t *screen) {
    settings_idle_content = lv_obj_create(screen);
    lv_obj_set_pos(settings_idle_content, 0, 0);
    lv_obj_set_size(settings_idle_content, SCREEN_WIDTH, SCREEN_HEIGHT);
    style_clean(settings_idle_content);
    lv_obj_set_style_bg_opa(settings_idle_content, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(settings_idle_content, 0, 0);
    lv_obj_clear_flag(settings_idle_content, LV_OBJ_FLAG_CLICKABLE);

    // 用完整不透明面板盖住参考底图中原来的两行快捷键和静态选位框。
    // 六个快捷电压压成一行，把纵向空间让给更大的微调触控区。
    lv_obj_t *editor = make_gradient_box(
        settings_idle_content, 340, skin_body_y(158), 441,
        skin_body_height(158, 309),
        0x08213B, 0x030D1B, 10, UI_BORDER, 1);
    make_label(editor, "设置电压 / 微调", 14, 9,
               &lv_font_jcy18, 0xFFFFFF);
    lv_obj_t *hint = make_label(
        editor, "单击选位  双击数字直接输入", 190, 11,
        &lv_font_jcy16, UI_MUTED);
    lv_obj_set_width(hint, 236);
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_RIGHT, 0);

    lv_obj_t *minus = make_gradient_button(
        editor, 10, 46, 66, 139,
        0x0B4161, 0x051A2D, 9, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(minus, voltage_step_event, LV_EVENT_CLICKED,
                        const_cast<int *>(&VOLTAGE_STEP_DIRECTIONS[0]));
    make_adjust_icon(minus, false);

    lv_obj_t *value_box = make_gradient_box(
        editor, 84, 46, 273, 139,
        0x071B35, 0x020913, 8, UI_BORDER, 1);
    static constexpr int DIGIT_X[5] = {6, 52, 111, 157, 203};
    for (size_t i = 0; i < 5; ++i) {
        set_voltage_digit_boxes[i] = make_gradient_button(
            value_box, DIGIT_X[i], 3, 43, 133,
            0x071A31, 0x030D1C, 5, UI_BORDER_DIM, 1,
            jcy_audio::Cue::Adjust);
        lv_obj_add_event_cb(
            set_voltage_digit_boxes[i], voltage_digit_select_event,
            LV_EVENT_CLICKED,
            const_cast<uint8_t *>(&VOLTAGE_DIGIT_INDICES[i]));
        set_voltage_digit_labels[i] = make_center_label(
            set_voltage_digit_boxes[i], "0",
            &lv_font_montserrat_48, UI_TEXT);
    }
    make_label(value_box, ".", 96, 35,
               &lv_font_montserrat_48, UI_TEXT);
    make_label(value_box, "V", 247, 52,
               &lv_font_montserrat_24, UI_MUTED);

    lv_obj_t *plus = make_gradient_button(
        editor, 365, 46, 66, 139,
        0x0B4161, 0x051A2D, 9, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(plus, voltage_step_event, LV_EVENT_CLICKED,
                        const_cast<int *>(&VOLTAGE_STEP_DIRECTIONS[1]));
    make_adjust_icon(plus, true);

    minimum_voltage_label = make_label(
        editor, "最小 3.000V", 12, 195,
        &lv_font_jcy16, UI_MUTED);
    lv_obj_set_width(minimum_voltage_label, 160);
    maximum_voltage_label = make_label(
        editor, "最大 4.250V", 269, 195,
        &lv_font_jcy16, UI_MUTED);
    lv_obj_set_width(maximum_voltage_label, 160);
    lv_obj_set_style_text_align(maximum_voltage_label,
                                LV_TEXT_ALIGN_RIGHT, 0);
    set_voltage_header_label = nullptr;

    compute_quick_voltages();
    static constexpr int QUICK_X[6] = {10, 81, 152, 223, 294, 365};
    static constexpr int QUICK_W[6] = {67, 67, 67, 67, 67, 66};
    for (size_t i = 0; i < 6; ++i) {
        quick_voltage_buttons[i] = make_gradient_button(
            editor, QUICK_X[i], 232, QUICK_W[i], 80,
            0x0A2B4A, 0x061A2D, 6, UI_BORDER_DIM, 1,
            jcy_audio::Cue::Adjust);
        lv_obj_add_event_cb(quick_voltage_buttons[i], quick_voltage_event,
                            LV_EVENT_CLICKED, &quick_voltages_mv[i]);
        quick_voltage_labels[i] = make_center_label(
            quick_voltage_buttons[i], "",
            &lv_font_montserrat_14, UI_TEXT);
        lv_obj_set_width(quick_voltage_labels[i], QUICK_W[i] - 4);
        lv_obj_set_style_text_line_space(quick_voltage_labels[i], -1, 0);
    }
}

void create_skin_main_waveform(lv_obj_t *screen) {
    settings_wave_content = lv_obj_create(screen);
    lv_obj_set_pos(settings_wave_content, 0, 0);
    lv_obj_set_size(settings_wave_content, SCREEN_WIDTH, SCREEN_HEIGHT);
    style_clean(settings_wave_content);
    lv_obj_set_style_bg_opa(settings_wave_content, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(settings_wave_content, 0, 0);
    lv_obj_clear_flag(settings_wave_content, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *wave_panel = make_gradient_box(
        settings_wave_content, 340, skin_body_y(158), 441,
        skin_body_height(158, 309),
        UI_PANEL_TOP, UI_PANEL_BOTTOM, 10, UI_BORDER, 1);
    make_label(wave_panel, "实时波形", 14, 9,
               &lv_font_jcy18, 0xFFFFFF);
    main_wave_voltage_label = make_label(
        wave_panel, "V --.---V", 116, 10,
        &lv_font_montserrat_16, 0x65F2DB);
    lv_obj_set_width(main_wave_voltage_label, 160);
    lv_label_set_long_mode(main_wave_voltage_label, LV_LABEL_LONG_CLIP);
    main_wave_current_label = make_label(
        wave_panel, "I --", 290, 10,
        &lv_font_montserrat_16, 0xFFB46A);
    lv_obj_set_width(main_wave_current_label, 137);
    lv_obj_set_style_text_align(main_wave_current_label,
                                LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_long_mode(main_wave_current_label, LV_LABEL_LONG_CLIP);

    main_wave_chart = lv_chart_create(wave_panel);
    lv_obj_set_pos(main_wave_chart, 12, 40);
    lv_obj_set_size(main_wave_chart, 417, 177);
    lv_chart_set_type(main_wave_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_update_mode(main_wave_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_point_count(main_wave_chart, 240);
    lv_chart_set_range(main_wave_chart, LV_CHART_AXIS_PRIMARY_Y,
                       max(0, pack_minimum_mv() - 300),
                       min(MAX_BATTERY_PACK_MV, pack_maximum_mv() + 300));
    main_wave_current_range_ma = min(
        1000, max(100, battery_settings.overcurrent_ma));
    lv_chart_set_range(main_wave_chart, LV_CHART_AXIS_SECONDARY_Y,
                       -main_wave_current_range_ma,
                       main_wave_current_range_ma);
    lv_chart_set_div_line_count(main_wave_chart, 6, 11);
    lv_obj_set_style_bg_color(main_wave_chart, color(0x071A31), 0);
    lv_obj_set_style_border_color(main_wave_chart, color(0x1A5A82), 0);
    lv_obj_set_style_line_color(main_wave_chart, color(0x1B466B),
                                LV_PART_MAIN);
    lv_obj_set_style_line_width(main_wave_chart, 1, LV_PART_MAIN);
    lv_obj_set_style_line_width(main_wave_chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(main_wave_chart, 0, LV_PART_INDICATOR);
    main_voltage_series = lv_chart_add_series(
        main_wave_chart, color(0x65F2DB), LV_CHART_AXIS_PRIMARY_Y);
    main_current_series = lv_chart_add_series(
        main_wave_chart, color(0xFFB46A), LV_CHART_AXIS_SECONDARY_Y);
    make_label(wave_panel,
               "电压(绿) / 电流(橙)  100ms采样  约24s窗口",
               14, 220, &lv_font_jcy16, 0xCDEBFA);

    // 输出开启后仍保留完整选位微调：单击数字选位、双击直接输入。
    lv_obj_t *adjust_bar = make_gradient_box(
        wave_panel, 10, 242, 421, 68,
        0x0B2D4B, 0x041224, 8, UI_BORDER_DIM, 1);
    lv_obj_t *minus = make_gradient_button(
        adjust_bar, 6, 7, 52, 54,
        0x0B4161, 0x051A2D, 7, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(minus, voltage_step_event, LV_EVENT_CLICKED,
                        const_cast<int *>(&VOLTAGE_STEP_DIRECTIONS[0]));
    make_adjust_icon(minus, false);

    lv_obj_t *value_box = make_gradient_box(
        adjust_bar, 64, 7, 291, 54,
        0x071A31, 0x030D1B, 7, UI_BORDER, 1);
    static constexpr int WAVE_DIGIT_X[5] = {5, 50, 111, 156, 201};
    for (size_t i = 0; i < 5; ++i) {
        main_wave_digit_boxes[i] = make_gradient_button(
            value_box, WAVE_DIGIT_X[i], 4, 41, 46,
            0x071A31, 0x030D1C, 4, UI_BORDER_DIM, 1,
            jcy_audio::Cue::Adjust);
        lv_obj_add_event_cb(
            main_wave_digit_boxes[i], voltage_digit_select_event,
            LV_EVENT_CLICKED,
            const_cast<uint8_t *>(&VOLTAGE_DIGIT_INDICES[i]));
        main_wave_digit_labels[i] = make_center_label(
            main_wave_digit_boxes[i], "0",
            &lv_font_montserrat_30, UI_TEXT);
    }
    make_label(value_box, ".", 94, 9,
               &lv_font_montserrat_30, UI_TEXT);
    make_label(value_box, "V", 258, 16,
               &lv_font_montserrat_20, UI_MUTED);

    lv_obj_t *plus = make_gradient_button(
        adjust_bar, 361, 7, 54, 54,
        0x0B4161, 0x051A2D, 7, UI_CYAN, 1,
        jcy_audio::Cue::Adjust);
    lv_obj_add_event_cb(plus, voltage_step_event, LV_EVENT_CLICKED,
                        const_cast<int *>(&VOLTAGE_STEP_DIRECTIONS[1]));
    make_adjust_icon(plus, true);
    refresh_set_voltage();
    lv_obj_add_flag(settings_wave_content, LV_OBJ_FLAG_HIDDEN);
}

void create_reference_skin_ui(lv_obj_t *screen) {
    lv_obj_t *background = lv_img_create(screen);
    lv_img_set_src(background, &lv_img_jcy_main_skin);
    lv_obj_set_pos(background, 0, 0);
    lv_obj_set_size(background, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_clear_flag(background, LV_OBJ_FLAG_CLICKABLE);

    create_skin_header(screen);
    create_skin_mode_hitboxes(screen);
    create_skin_measurements(screen);
    create_skin_top_cards(screen);
    create_skin_voltage_editor(screen);
    create_skin_main_waveform(screen);
    refresh_main_waveform_visibility();
}

void create_ui() {
    // 这两组对象属于常驻主界面，不能在打开/关闭设置页时清空。
    // 仅在重新创建整套主界面时重置，避免微调后找不到数字标签。
    memset(set_voltage_digit_boxes, 0, sizeof(set_voltage_digit_boxes));
    memset(set_voltage_digit_labels, 0, sizeof(set_voltage_digit_labels));
    memset(main_wave_digit_boxes, 0, sizeof(main_wave_digit_boxes));
    memset(main_wave_digit_labels, 0, sizeof(main_wave_digit_labels));
    invalidate_idle_render_cache();
    lv_obj_t *screen = lv_obj_create(nullptr);
    ui_screen = screen;
    style_clean(screen);
    lv_obj_set_style_border_width(screen, 0, 0);
    lv_obj_set_style_radius(screen, 0, 0);
    lv_obj_set_style_bg_color(screen, color(UI_BG_TOP), 0);
    lv_obj_set_style_bg_grad_color(screen, color(UI_BG_BOTTOM), 0);
    lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER, 0);
    lv_scr_load(screen);

    create_reference_skin_ui(screen);

    refresh_mode_buttons();
    refresh_power_state();
    refresh_main_values();
    lv_obj_update_layout(screen);
}

void splash_confirm_event(lv_event_t *) {
    if (splash_screen == nullptr) {
        return;
    }

    if (splash_auto_timer != nullptr) {
        lv_timer_del(splash_auto_timer);
        splash_auto_timer = nullptr;
    }
    if (splash_anim_timer != nullptr) {
        lv_timer_del(splash_anim_timer);
        splash_anim_timer = nullptr;
    }

    lv_obj_t *old_splash = splash_screen;
    splash_screen = nullptr;
    splash_anim_image = nullptr;
    output_running = false;
    create_ui();
    request_device_mode(JcyScpiMode::Battery, false);
    lv_obj_del_async(old_splash);
    Serial.println("EVENT splash entered");
}

void splash_auto_enter_timer(lv_timer_t *) {
    Serial.println("EVENT splash auto_enter");
    splash_confirm_event(nullptr);
}

// 启动鲸鱼动画：完整源视频均匀抽成 30 帧，以 100 ms 推进一帧，
// 约 3 秒播放完成后自动进入主界面。
// 不创建点击区，因此触摸和 USB 截图命令都不能提前跳过动画。
static void splash_anim_timer_cb(lv_timer_t *timer) {
    if (splash_anim_image == nullptr ||
        jcy_boot_animation_frame_count == 0) {
        return;
    }
    if (splash_anim_index + 1U < jcy_boot_animation_frame_count) {
        ++splash_anim_index;
        lv_img_set_src(splash_anim_image,
                       jcy_boot_animation_frames[splash_anim_index]);
        if (splash_anim_index + 1U == jcy_boot_animation_frame_count) {
            // Final logo frame: play a soft sea-wave cue and keep the logo
            // visible until the cue finishes before entering the main UI.
            jcy_audio::play(jcy_audio::Cue::Wave);
            lv_timer_set_period(timer, SPLASH_FINAL_HOLD_MS);
        }
        return;
    }

    lv_timer_del(timer);
    splash_anim_timer = nullptr;
    Serial.println("EVENT splash animation complete");
    splash_confirm_event(nullptr);
}

void create_splash_ui() {
    splash_screen = lv_obj_create(nullptr);
    style_clean(splash_screen);
    lv_obj_set_size(splash_screen, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_style_radius(splash_screen, 0, 0);
    lv_obj_set_style_border_width(splash_screen, 0, 0);
    lv_obj_set_style_bg_color(splash_screen, color(0x000000), 0);
    lv_obj_set_style_bg_opa(splash_screen, LV_OPA_COVER, 0);

    // 使用低分辨率鲸鱼开机动画替代原静态 splash 图。动画继续使用
    // 800x480逻辑画布；高分屏配置由flush层统一缩放到物理面板。
    splash_anim_image = lv_img_create(splash_screen);
    splash_anim_index = 0;
    if (jcy_boot_animation_frame_count > 0) {
        lv_img_set_src(splash_anim_image,
                       jcy_boot_animation_frames[splash_anim_index]);
        lv_obj_set_size(splash_anim_image, SPLASH_ANIM_WIDTH,
                        SPLASH_ANIM_HEIGHT);
        lv_obj_center(splash_anim_image);
    } else {
        lv_img_set_src(splash_anim_image, &lv_img_jcy_splash);
        lv_obj_set_pos(splash_anim_image, 0, 0);
        lv_obj_set_size(splash_anim_image, SCREEN_WIDTH, SCREEN_HEIGHT);
    }
    lv_obj_clear_flag(splash_anim_image, LV_OBJ_FLAG_CLICKABLE);

    // 开机动画完整播完前不提供任何可点按的 UI，避免被跳过。
    lv_scr_load(splash_screen);
    lv_obj_update_layout(splash_screen);

    // 固定 100 ms 切换一帧；最终 logo 出现时播放海浪声并短暂停留。
    // 不创建透明点击按钮，用户无法跳过开机动画。
    if (jcy_boot_animation_frame_count > 1) {
        splash_anim_timer = lv_timer_create(
            splash_anim_timer_cb, SPLASH_ANIM_FRAME_MS, nullptr);
    }
    Serial.println("EVENT splash animation running 30f/100ms recorded wave (no skip)");
}

bool init_lvgl() {
    lv_init();

    // TLS/OTA needs a large contiguous internal-heap block for the mbedTLS
    // record buffers.  Keeping the LVGL strip buffer in internal RAM used
    // about 96 KB permanently and made HTTPS fail once the full UI was alive.
    // The RGB framebuffers already live in PSRAM, so keep this staging strip
    // there as well and reserve internal RAM for Wi-Fi/TLS and task stacks.
    constexpr size_t PRIMARY_BUFFER_LINES = 80;
    constexpr size_t FALLBACK_INTERNAL_BUFFER_LINES = 24;
    // Downscaling dirty strips independently can leave gaps where multiple UI
    // pixels collapse onto one physical pixel. Render a complete logical frame
    // for the 800x480 -> 480x272 profile, then scale it in one flush.
    // Keep the established strip-buffer rendering path. Full-frame LVGL
    // refresh stalls the UI task while the 800x480 frame is downscaled.
    const size_t primary_buffer_lines = PRIMARY_BUFFER_LINES;
    size_t draw_buffer_pixels = SCREEN_WIDTH * primary_buffer_lines;
    draw_pixels = static_cast<lv_color_t *>(heap_caps_malloc(
        sizeof(lv_color_t) * draw_buffer_pixels,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (draw_pixels) {
        Serial.printf("LVGL draw buffer spiram lines=%u bytes=%u\n",
                      static_cast<unsigned>(primary_buffer_lines),
                      static_cast<unsigned>(sizeof(lv_color_t) *
                                            draw_buffer_pixels));
    } else {
        draw_buffer_pixels =
            SCREEN_WIDTH * FALLBACK_INTERNAL_BUFFER_LINES;
        draw_pixels = static_cast<lv_color_t *>(heap_caps_malloc(
            sizeof(lv_color_t) * draw_buffer_pixels,
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        Serial.printf("LVGL draw buffer internal fallback lines=%u bytes=%u\n",
                      static_cast<unsigned>(FALLBACK_INTERNAL_BUFFER_LINES),
                      static_cast<unsigned>(sizeof(lv_color_t) *
                                            draw_buffer_pixels));
    }
    if (!draw_pixels) {
        Serial.println("LVGL draw buffer allocation failed");
        return false;
    }

    lv_disp_draw_buf_init(
        &draw_buffer, draw_pixels, nullptr, draw_buffer_pixels);
    lv_disp_drv_init(&display_driver);
    display_driver.hor_res = SCREEN_WIDTH;
    display_driver.ver_res = SCREEN_HEIGHT;
    display_driver.flush_cb = display_flush;
    display_driver.draw_buf = &draw_buffer;
    display_driver.full_refresh = 0;
#if defined(JCY_HY200C_UI) && JCY_HY200C_UI
    // The HY200-C assembly is mounted opposite to the earlier installation.
    // Native orientation therefore rotates the visible UI 180 degrees relative
    // to the previous software-rotated A84 display.
    display_driver.sw_rotate = 0;
    display_driver.rotated = LV_DISP_ROT_NONE;
#else
    display_driver.sw_rotate = 1;
    display_driver.rotated = LV_DISP_ROT_180;
#endif
    lv_disp_drv_register(&display_driver);

#if !defined(JCY_HY200C_UI) || !JCY_HY200C_UI
    // GT911 is fitted on the general JCY2001D panel. The HY200-C 4.3-inch
    // display-only assembly has no TP, so do not initialize or poll I2C touch.
    Wire1.setBufferSize(256);
    touch.begin();
    touch.setRotation(ROTATION_NORMAL);
    lv_indev_drv_init(&touch_driver);
    touch_driver.type = LV_INDEV_TYPE_POINTER;
    touch_driver.read_cb = touch_read;
    lv_indev_drv_register(&touch_driver);
#else
    Serial.println("HY200-C display-only build: touch disabled");
#endif
    return true;
}
}  // namespace

void setup() {
    Serial.begin(115200);
    jcy_install_mbedtls_psram_allocator();
    Serial.printf(JCY_PRODUCT_NAME " %s (%s) UI init\n",
                  JCY1000_FIRMWARE_VERSION, JCY1000_BUILD_TAG);

#if !defined(JCY_HY200C_UI) || !JCY_HY200C_UI
    jcy_audio::begin();
#else
    Serial.println("HY200-C audio disabled: GPIO17/18 assigned to TTL1 UART");
#endif

#if defined(JCY_HY200C_UI) && JCY_HY200C_UI
    hy200_link.begin(TTL_BAUD, TTL_RX_PIN, TTL_TX_PIN);
    Serial.println("HY200-C production UART enabled: 115200 8N1 binary");
#else
    if (TTL_ENABLED) {
        // UART空闲电平必须为高。先把TX拉高，再交给UART外设，尽量减少
        // ESP32复位期间的低脉冲被下位机误判为命令起始位。
        pinMode(TTL_TX_PIN, OUTPUT);
        digitalWrite(TTL_TX_PIN, HIGH);
        delay(2);
        scpi_link.begin(TTL_BAUD, TTL_RX_PIN, TTL_TX_PIN, TTL_ADDRESS);
        scpi_link.set_poll_interval(350);
        last_identity_request_ms = millis() - 3000;
        last_identity_sequence = 0;
        boot_safe_output_off_active = true;
        boot_safe_output_off_attempts = 0;
        boot_safe_output_off_last_request_ms = millis() - 250;
        request_boot_safe_output_off(millis());

        // 在LCD、LVGL和无线初始化之前先发送并等待一次安全关断确认，
        // 下位机无响应时最多等待250ms，后续在主循环中继续重试。
        const uint32_t boot_safe_deadline = millis() + 250;
        while (boot_safe_output_off_active &&
               static_cast<int32_t>(millis() - boot_safe_deadline) < 0) {
            const uint32_t now = millis();
            scpi_link.loop(now);
            process_scpi_events();
            process_boot_output_safety(now);
            delay(1);
        }
    } else {
        Serial.println("TTL UART disabled");
    }
#endif

    gfx.begin();
    Serial.printf(
        "RGB panel profile=%s physical=%dx%d ui=%dx%d viewport=%dx%d+%d+%d pclk=%ld nominal_hz=%lu\n",
        jcy_panel::kModel, PANEL_WIDTH, PANEL_HEIGHT,
        SCREEN_WIDTH, SCREEN_HEIGHT,
        jcy_panel::kContentWidth, jcy_panel::kContentHeight,
        jcy_panel::kContentX, jcy_panel::kContentY,
        static_cast<long>(RGB_PCLK_HZ),
        static_cast<unsigned long>(LCD_REFRESH_HZ));
    Serial.printf(
        "RGB cache-safe driver bounce=%u lines total_internal=%u bytes "
        "lcd_iram=%d gdma_iram=%d psram_xip=%d\n",
        static_cast<unsigned>(JcyEsp32RgbPanel::kBounceBufferLines),
        static_cast<unsigned>(JcyEsp32RgbPanel::kBounceBufferBytesPerPanelLinePair(
            PANEL_WIDTH)),
        static_cast<int>(CONFIG_LCD_RGB_ISR_IRAM_SAFE),
        static_cast<int>(CONFIG_GDMA_ISR_IRAM_SAFE),
        static_cast<int>(CONFIG_SPIRAM_XIP_FROM_PSRAM));
    display_vsync_semaphore = xSemaphoreCreateBinary();
    if (display_vsync_semaphore != nullptr) {
        rgb_bus.setFrameTransDoneCallback(
            rgb_frame_trans_done_callback,
            display_vsync_semaphore);
        Serial.println("RGB VSYNC anti-tear guard enabled");
    } else {
        Serial.println("RGB VSYNC anti-tear semaphore allocation failed");
    }
    gfx.fillScreen(BLACK);
    if (DISPLAY_VSYNC_GUARD && display_vsync_semaphore != nullptr) {
        while (xSemaphoreTake(display_vsync_semaphore, 0) == pdTRUE) {
        }
    }
    if (gfx.submitFrame()) {
        set_pending_full_frame();
        display_frame_needs_vsync =
            DISPLAY_VSYNC_GUARD && display_vsync_semaphore != nullptr;
    } else {
        Serial.println("RGB initial frame submit failed");
        display_frame_needs_vsync = false;
    }
    pinMode(BACKLIGHT_PIN, OUTPUT);
    digitalWrite(BACKLIGHT_PIN, HIGH);

    if (!init_lvgl()) {
        return;
    }

#if defined(JCY_HY200C_UI) && JCY_HY200C_UI
    hy200_ui::begin(&hy200_link, JCY1000_FIRMWARE_VERSION);
    Serial.println("HY200-C 4.3 UI ready");
#else
    jcy_wireless::begin();
    jcy_ota::begin(JCY_PRODUCT_NAME, JCY1000_FIRMWARE_VERSION);
    jcy_cloud_ota::begin(JCY_PRODUCT_NAME, JCY1000_FIRMWARE_VERSION);
    jcy_lower_mcu_ota::begin(&scpi_link);
    create_splash_ui();
    Serial.println(JCY_PRODUCT_NAME " splash ready; auto-entering main UI in 3 seconds");
#endif
}

void loop() {
    static uint32_t last_wireless_refresh_ms = 0;
    const uint32_t now = millis();
#if defined(JCY_HY200C_UI) && JCY_HY200C_UI
    hy200_link.loop(now);
    hy200_ui::loop(now);
    poll_serial_commands();
    lv_timer_handler();
    delay(5);
    return;
#else
    if (communication_mode == CommunicationMode::Passthrough) {
        // 透传时暂停内部SCPI轮询、无线状态输出和OTA，避免任何调试文本
        // 混入USB<->TTL原始数据流。LVGL仍持续运行，用户可触屏退出。
        poll_passthrough_bridge(now);
        refresh_debug_overlay(now);
    } else {
        const bool ota_update_allowed =
            !boot_safe_output_off_active && !output_running &&
            !requested_output_state && !output_change_pending;
        const JcyScpiIdentity &lower_identity = scpi_link.identity();
        jcy_lower_mcu_ota::loop(
            ota_update_allowed && !cutoff_running,
            lower_identity.valid ? lower_identity.firmware : nullptr);
        update_cloud_ota_quiet_mode();
        if (TTL_ENABLED) {
            scpi_link.loop(now);
            process_scpi_events();
            if (!cloud_ota_quiet_active) {
                continue_device_mode_transition();
                const JcyScpiIdentity &identity = scpi_link.identity();
                if (!boot_safe_output_off_active && !identity.valid &&
                    now - last_identity_request_ms >= 3000 &&
                    !scpi_link.command_pending(
                        JcyScpiCommandKind::QueryIdentity)) {
                    if (scpi_link.request_identity()) {
                        last_identity_request_ms = now;
                    }
                }
            }
        }
        if (!cloud_ota_quiet_active) {
            process_boot_output_safety(now);
            process_cutoff_state(now);
        }
        if (now - last_wireless_refresh_ms > 300) {
            if (!cloud_ota_quiet_active) {
                refresh_wireless_ui();
                refresh_ota_ui();
            }
            refresh_cloud_ota_ui();
            refresh_lower_ota_ui();
            last_wireless_refresh_ms = now;
        }
        if (!cloud_ota_quiet_active) {
            refresh_debug_overlay(now);
        }
        if (!cloud_ota_quiet_active) {
            jcy_ota::loop(ota_update_allowed);
        }
        jcy_cloud_ota::loop(ota_update_allowed);
        if (!cloud_ota_quiet_active) {
            const bool live_activity = output_running || cutoff_running;
            const uint32_t live_refresh_interval_ms =
                live_activity ? 100U : 250U;
            const bool idle_content_changed =
                !live_activity && idle_render_state_changed(now);
            if (now - last_live_ui_refresh_ms > live_refresh_interval_ms &&
                ((live_activity &&
                 (scpi_link.measurements().sequence !=
                      last_measurement_sequence ||
                  output_change_pending)) ||
                 (!live_activity && idle_content_changed))) {
                refresh_live_measurements();
                if (!live_activity) {
                    cache_idle_render_state(now);
                }
            } else if (live_activity &&
                       now - last_live_ui_refresh_ms > 1000) {
                refresh_live_measurements();
            }
            poll_serial_commands();
        }
    }
    lv_timer_handler();
    delay(cloud_ota_quiet_active ? 10 : 5);
#endif
}
