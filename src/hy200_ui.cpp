#include "hy200_ui.h"

#include <Preferences.h>
#include <WiFi.h>
#include <cstdio>
#include <cstring>

LV_FONT_DECLARE(lv_font_jcy16);
LV_FONT_DECLARE(lv_font_jcy18);
LV_FONT_DECLARE(lv_font_jcy20);
LV_IMG_DECLARE(lv_img_hy200_main);

namespace hy200_ui {
namespace {
enum class Page : uint8_t { Main, Settings, Language, Device, Warranty, About };
enum class MachineView : uint8_t { Disconnected, Ready, Running, Calibrating, Alarm };

hy200::Protocol *link = nullptr;
const char *firmware_version = "V1.0.0";
Preferences preferences;
Page page = Page::Main;
bool english = false;
uint8_t duration_seconds = 5;
uint8_t reset_failures = 0;
uint32_t last_ack_generation = 0;
uint32_t observed_reset_action = 0;
bool reset_action_denied = false;
uint32_t last_status_sequence = 0;
uint32_t last_refresh_ms = 0;
uint32_t last_rendered_signature = UINT32_MAX;
uint32_t last_rendered_centiseconds = UINT32_MAX;
MachineView last_rendered_view = static_cast<MachineView>(0xFF);
bool render_dirty = true;
uint32_t running_started_ms = 0;
bool was_running = false;
bool countdown_active = false;

lv_obj_t *screen = nullptr;
lv_obj_t *main_page = nullptr;
lv_obj_t *sub_page = nullptr;
lv_obj_t *frequency_value = nullptr;
lv_obj_t *time_value = nullptr;
lv_obj_t *state_label = nullptr;
lv_obj_t *state_hint = nullptr;
lv_obj_t *gear28 = nullptr;
lv_obj_t *gear35 = nullptr;
lv_obj_t *gear28_box = nullptr;
lv_obj_t *gear35_box = nullptr;
lv_obj_t *progress_cells[27] = {};
lv_obj_t *version_label = nullptr;
lv_obj_t *sub_title = nullptr;
lv_obj_t *sub_content = nullptr;
lv_obj_t *scan_button = nullptr;
lv_obj_t *mode_button = nullptr;
lv_obj_t *language_button = nullptr;
lv_obj_t *time_button = nullptr;
lv_obj_t *focus_targets[6] = {};
uint8_t focus_index = 0;
bool editing_time = false;
bool selected_gear35 = false;
bool cutting_mode = false;
constexpr int kLeftKeyPin = 10;
constexpr int kRightKeyPin = 11;
constexpr int kConfirmKeyPin = 12;
constexpr uint32_t kKeyDebounceMs = 35;
bool key_raw[3] = {true, true, true};
bool key_stable[3] = {true, true, true};
uint32_t key_changed_ms[3] = {};
uint32_t key_pressed_ms[3] = {};
uint32_t key_repeat_ms[3] = {};
uint32_t last_key_poll_ms = 0;
constexpr uint32_t kKeyRepeatStartMs = 500;
constexpr uint32_t kKeyRepeatSlowMs = 180;
constexpr uint32_t kKeyRepeatFastMs = 70;

lv_color_t color(uint32_t rgb) { return lv_color_hex(rgb); }
const lv_font_t *zh_font() { return &lv_font_jcy16; }

void clean(lv_obj_t *object) {
    lv_obj_set_style_pad_all(object, 0, 0);
    lv_obj_set_style_border_width(object, 0, 0);
    lv_obj_set_style_radius(object, 0, 0);
    lv_obj_set_style_bg_opa(object, LV_OPA_TRANSP, 0);
}

lv_obj_t *label(lv_obj_t *parent, const char *text, int x, int y, int w, int h,
                const lv_font_t *font, uint32_t rgb,
                lv_text_align_t align = LV_TEXT_ALIGN_CENTER) {
    lv_obj_t *item = lv_label_create(parent);
    lv_label_set_text(item, text);
    lv_obj_set_pos(item, x, y);
    lv_obj_set_size(item, w, h);
    lv_obj_set_style_text_font(item, font, 0);
    lv_obj_set_style_text_color(item, color(rgb), 0);
    lv_obj_set_style_text_align(item, align, 0);
    lv_label_set_long_mode(item, LV_LABEL_LONG_CLIP);
    return item;
}

lv_obj_t *button(lv_obj_t *parent, const char *text, int x, int y, int w, int h,
                 lv_event_cb_t callback, void *data = nullptr,
                 const lv_font_t *caption_font = nullptr) {
    lv_obj_t *item = lv_btn_create(parent);
    lv_obj_set_pos(item, x, y);
    lv_obj_set_size(item, w, h);
    lv_obj_set_style_radius(item, 7, 0);
    lv_obj_set_style_bg_color(item, color(0x073657), 0);
    lv_obj_set_style_bg_grad_color(item, color(0x041526), 0);
    lv_obj_set_style_bg_grad_dir(item, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(item, 1, 0);
    lv_obj_set_style_border_color(item, color(0x168DFF), 0);
    lv_obj_set_style_bg_color(item, color(0x0A6B8F), LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(item, 5, 0);
    lv_obj_set_style_shadow_color(item, color(0x00101F), 0);
    lv_obj_t *caption = lv_label_create(item);
    lv_label_set_text(caption, text);
    lv_obj_set_style_text_font(caption,
                               caption_font != nullptr ? caption_font : zh_font(),
                               0);
    lv_obj_set_style_text_color(caption, color(0xF2F8FC), 0);
    lv_obj_set_style_text_letter_space(caption, 1, 0);
    lv_obj_center(caption);
    lv_obj_add_event_cb(item, callback, LV_EVENT_CLICKED, data);
    return item;
}

MachineView current_view(uint32_t now_ms) {
    if (link == nullptr || !link->online(now_ms)) return MachineView::Disconnected;
    const hy200::Status &s = link->status();
    if (s.calibrating) return MachineView::Calibrating;
    // Reason 1 is the expected end-of-countdown interlock, not an alarm.
    if (s.lock_reason >= 3 || (s.fault && s.lock_reason != 1)) return MachineView::Alarm;
    if (s.running) return MachineView::Running;
    return MachineView::Ready;
}

void show_page(Page next);
void apply_key_focus();

const char *localized_time_label() { return english ? "TIME" : "时间"; }
const char *localized_mode_label() {
    return cutting_mode ? (english ? "CUT" : "切割")
                        : (english ? "WELD" : "电焊");
}
const char *localized_scan_label() { return english ? "SCAN" : "扫频"; }
// Display the language that pressing the button will switch to. This gives
// immediate, unambiguous feedback that the language control is functional.
const char *localized_language_label() { return english ? "中文" : "EN"; }

void refresh_main_static_labels() {
    if (time_button == nullptr) return;
    lv_label_set_text(lv_obj_get_child(time_button, 0), localized_time_label());
    lv_label_set_text(lv_obj_get_child(mode_button, 0), localized_mode_label());
    lv_label_set_text(lv_obj_get_child(scan_button, 0), localized_scan_label());
    lv_label_set_text(lv_obj_get_child(language_button, 0), localized_language_label());
    render_dirty = true;
}

void deferred_show_page(void *data) {
    show_page(static_cast<Page>(reinterpret_cast<intptr_t>(data)));
}

void schedule_page(Page next) {
    lv_async_call(deferred_show_page,
                  reinterpret_cast<void *>(static_cast<intptr_t>(next)));
}

void back_event(lv_event_t *) {
    if (page == Page::Settings) schedule_page(Page::Main);
    else schedule_page(Page::Settings);
}

void language_select_event(lv_event_t *event) {
    english = reinterpret_cast<intptr_t>(lv_event_get_user_data(event)) != 0;
    preferences.putBool("english", english);
    schedule_page(Page::Language);
}

void open_page_event(lv_event_t *event) {
    schedule_page(static_cast<Page>(reinterpret_cast<intptr_t>(lv_event_get_user_data(event))));
}

bool local_edit_allowed() {
    const MachineView view = current_view(millis());
    return view == MachineView::Ready || view == MachineView::Disconnected;
}

void time_display_event(lv_event_t *) {}

void duration_event(lv_event_t *event) {
    if (!local_edit_allowed()) return;
    const int delta = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    duration_seconds = static_cast<uint8_t>(constrain(static_cast<int>(duration_seconds) + delta, 1, 99));
    preferences.putUChar("duration", duration_seconds);
    link->set_duration_seconds(duration_seconds);
}

void calibration_event(lv_event_t *) {
    if (current_view(millis()) != MachineView::Ready) return;
    const hy200::Status &s = link->status();
    if (s.running || s.fault || s.handle_pressed || s.calibrating ||
        s.lock_reason != 0) return;
    link->request_calibration();
    lv_label_set_text(state_hint, english ? "CALIBRATION REQUESTED" : "已发送频率校准请求");
}

void scan_event(lv_event_t *) {
    calibration_event(nullptr);
}

void mode_event(lv_event_t *) {
    if (!local_edit_allowed()) return;
    cutting_mode = !cutting_mode;
    preferences.putBool("cutting", cutting_mode);
    lv_label_set_text(lv_obj_get_child(mode_button, 0),
                      cutting_mode ? (english ? "CUT" : "切割")
                                   : (english ? "WELD" : "电焊"));
}

void language_event(lv_event_t *) {
    if (!local_edit_allowed()) return;
    english = !english;
    preferences.putBool("english", english);
    editing_time = false;
    refresh_main_static_labels();
    show_page(Page::Main);
    apply_key_focus();
}

void gear_event(lv_event_t *event) {
    if (!local_edit_allowed()) return;
    selected_gear35 = reinterpret_cast<intptr_t>(lv_event_get_user_data(event)) != 0;
    preferences.putBool("gear35", selected_gear35);
    render_dirty = true;
}

void apply_key_focus() {
    for (uint8_t index = 0; index < 6; ++index) {
        if (focus_targets[index] == nullptr) continue;
        const bool focused = page == Page::Main && index == focus_index;
        lv_obj_set_style_outline_width(focus_targets[index], focused ? 3 : 0, 0);
        lv_obj_set_style_outline_color(
            focus_targets[index],
            color(focused && editing_time && focus_targets[index] == time_button
                      ? 0x00FF78
                      : 0xFFF000),
            0);
        lv_obj_set_style_outline_pad(focus_targets[index], 2, 0);
    }
}

void adjust_time(int direction) {
    if (!local_edit_allowed()) return;
    duration_seconds = static_cast<uint8_t>(constrain(
        static_cast<int>(duration_seconds) + direction, 1, 99));
    preferences.putUChar("duration", duration_seconds);
    link->set_duration_seconds(duration_seconds);
    lv_label_set_text_fmt(time_value, "%u.00s", static_cast<unsigned>(duration_seconds));
    render_dirty = true;
}

void navigate_key(int direction) {
    if (page != Page::Main) {
        schedule_page(Page::Main);
        return;
    }
    if (editing_time) {
        adjust_time(direction);
        return;
    }
    focus_index = static_cast<uint8_t>((focus_index + (direction > 0 ? 1 : 5)) % 6);
    apply_key_focus();
}

void confirm_key() {
    if (page != Page::Main) {
        schedule_page(Page::Main);
        return;
    }
    lv_obj_t *target = focus_targets[focus_index];
    if (target == time_button) {
        if (!local_edit_allowed()) return;
        editing_time = !editing_time;
        lv_label_set_text(lv_obj_get_child(time_button, 0),
                          editing_time ? localized_time_label()
                                       : localized_time_label());
        apply_key_focus();
        return;
    }
    if (target != nullptr && !lv_obj_has_state(target, LV_STATE_DISABLED)) {
        lv_event_send(target, LV_EVENT_CLICKED, nullptr);
    }
}

void poll_keys(uint32_t now_ms) {
    if (static_cast<uint32_t>(now_ms - last_key_poll_ms) < 5U) return;
    last_key_poll_ms = now_ms;
    const int pins[3] = {kLeftKeyPin, kRightKeyPin, kConfirmKeyPin};
    for (uint8_t index = 0; index < 3; ++index) {
        const bool raw = digitalRead(pins[index]) != LOW;
        if (raw != key_raw[index]) {
            key_raw[index] = raw;
            key_changed_ms[index] = now_ms;
        }
        if (raw != key_stable[index] &&
            static_cast<uint32_t>(now_ms - key_changed_ms[index]) >= kKeyDebounceMs) {
            key_stable[index] = raw;
            if (!raw) {
                key_pressed_ms[index] = now_ms;
                key_repeat_ms[index] = now_ms;
                if (index == 0) navigate_key(-1);
                else if (index == 1) navigate_key(1);
                else confirm_key();
            }
        }
        // Repeat left/right only in the dedicated time-edit state. Navigation
        // itself remains one step per press so a long hold cannot skip controls.
        if (index < 2 && !key_stable[index] && editing_time) {
            const uint32_t held_ms = now_ms - key_pressed_ms[index];
            const uint32_t interval = held_ms >= 1800U
                                          ? kKeyRepeatFastMs
                                          : kKeyRepeatSlowMs;
            if (held_ms >= kKeyRepeatStartMs &&
                static_cast<uint32_t>(now_ms - key_repeat_ms[index]) >= interval) {
                key_repeat_ms[index] = now_ms;
                adjust_time(index == 0 ? -1 : 1);
            }
        }
    }
}

lv_obj_t *panel(lv_obj_t *parent, int x, int y, int w, int h) {
    lv_obj_t *item = lv_obj_create(parent);
    clean(item);
    lv_obj_set_pos(item, x, y);
    lv_obj_set_size(item, w, h);
    lv_obj_set_style_radius(item, 10, 0);
    lv_obj_set_style_bg_color(item, color(0x0B2438), 0);
    lv_obj_set_style_bg_grad_color(item, color(0x06131F), 0);
    lv_obj_set_style_bg_grad_dir(item, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(item, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(item, 1, 0);
    lv_obj_set_style_border_color(item, color(0x2A79A5), 0);
    lv_obj_set_style_shadow_width(item, 4, 0);
    lv_obj_set_style_shadow_color(item, color(0x00101C), 0);
    lv_obj_set_style_shadow_opa(item, LV_OPA_30, 0);
    lv_obj_clear_flag(item, LV_OBJ_FLAG_CLICKABLE);
    return item;
}

void create_progress(lv_obj_t *parent) {
    lv_obj_t *track = lv_obj_create(parent);
    clean(track);
    lv_obj_set_pos(track, 51, 166);
    lv_obj_set_size(track, 370, 28);
    lv_obj_set_style_radius(track, 8, 0);
    lv_obj_set_style_bg_color(track, color(0x01060B), 0);
    lv_obj_set_style_bg_opa(track, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(track, 1, 0);
    lv_obj_set_style_border_color(track, color(0x173D58), 0);
    for (int i = 0; i < 27; ++i) {
        progress_cells[i] = lv_obj_create(track);
        clean(progress_cells[i]);
        lv_obj_set_pos(progress_cells[i], 8 + i * 13, 5);
        lv_obj_set_size(progress_cells[i], 10, 17);
        lv_obj_set_style_radius(progress_cells[i], 2, 0);
        lv_obj_set_style_bg_opa(progress_cells[i], LV_OPA_COVER, 0);
    }
}

void create_main() {
    main_page = lv_obj_create(screen);
    clean(main_page);
    lv_obj_set_size(main_page, 480, 272);
    lv_obj_set_style_bg_color(main_page, color(0x010711), 0);
    lv_obj_set_style_bg_opa(main_page, LV_OPA_COVER, 0);

    // Use the customer's supplied 480x272 artwork pixel-for-pixel. Dynamic
    // values and states are overlaid at the coordinates defined by that UI.
    lv_obj_t *background = lv_img_create(main_page);
    lv_img_set_src(background, &lv_img_hy200_main);
    lv_obj_set_pos(background, 0, 0);
    lv_obj_clear_flag(background, LV_OBJ_FLAG_CLICKABLE);

    frequency_value = label(main_page, "28.00", 44, 91, 100, 37,
                            &lv_font_montserrat_36, 0xFFFFFF);
    time_value = label(main_page, "5.00s", 331, 94, 130, 34,
                       &lv_font_montserrat_30, 0xFFFFFF);
    state_label = label(main_page, "准备就绪", 158, 76, 166, 30,
                        &lv_font_jcy20, 0x2561FA);
    state_hint = label(main_page, "等待主机连接", 150, 112, 180, 38,
                       zh_font(), 0xB9D5E8);
    lv_label_set_long_mode(state_hint, LV_LABEL_LONG_WRAP);
    create_progress(main_page);

    // Six controls fill the complete 480-pixel row: 12px side margins, 4px
    // gaps, and 73/74px integer widths. This avoids the previous empty tail.
    gear28_box = button(main_page, "28KHz", 12, 198, 73, 40, gear_event,
                        reinterpret_cast<void *>(0), &lv_font_montserrat_14);
    gear28 = lv_obj_get_child(gear28_box, 0);
    focus_targets[0] = gear28_box;
    gear35_box = button(main_page, "35KHz", 89, 198, 73, 40, gear_event,
                        reinterpret_cast<void *>(1), &lv_font_montserrat_14);
    gear35 = lv_obj_get_child(gear35_box, 0);
    focus_targets[1] = gear35_box;
    time_button = button(main_page, localized_time_label(),
                         166, 198, 73, 40, time_display_event, nullptr,
                         &lv_font_jcy18);
    focus_targets[2] = time_button;
    mode_button = button(main_page, localized_mode_label(),
                         243, 198, 74, 40, mode_event, nullptr,
                         &lv_font_jcy18);
    focus_targets[3] = mode_button;
    scan_button = button(main_page, localized_scan_label(), 321, 198, 73, 40,
                         scan_event, nullptr, &lv_font_jcy18);
    focus_targets[4] = scan_button;
    language_button = button(main_page, localized_language_label(),
                             398, 198, 70, 40, language_event, nullptr,
                             &lv_font_jcy18);
    focus_targets[5] = language_button;
    lv_obj_t *version_box = lv_obj_create(main_page);
    clean(version_box);
    lv_obj_set_pos(version_box, 348, 7);
    lv_obj_set_size(version_box, 124, 25);
    lv_obj_set_style_radius(version_box, 5, 0);
    lv_obj_set_style_bg_color(version_box, color(0x031321), 0);
    lv_obj_set_style_bg_opa(version_box, LV_OPA_80, 0);
    lv_obj_set_style_border_width(version_box, 1, 0);
    lv_obj_set_style_border_color(version_box, color(0x245C82), 0);
    lv_obj_clear_flag(version_box, LV_OBJ_FLAG_CLICKABLE);
    version_label = label(version_box, "", 4, 4, 116, 18,
                          &lv_font_montserrat_14, 0xB9DDF5,
                          LV_TEXT_ALIGN_RIGHT);
    lv_label_set_text_fmt(version_label, "FW %s", firmware_version);
}

void create_sub_page() {
    sub_page = lv_obj_create(screen);
    clean(sub_page);
    lv_obj_set_size(sub_page, 480, 272);
    lv_obj_set_style_bg_color(sub_page, color(0x03101F), 0);
    lv_obj_set_style_bg_grad_color(sub_page, color(0x01060C), 0);
    lv_obj_set_style_bg_grad_dir(sub_page, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(sub_page, LV_OPA_COVER, 0);
    sub_title = label(sub_page, "设置", 20, 12, 300, 30, &lv_font_jcy20,
                      0xFFFFFF, LV_TEXT_ALIGN_LEFT);
    button(sub_page, "返回", 398, 8, 66, 36, back_event);
    sub_content = lv_obj_create(sub_page);
    clean(sub_content);
    lv_obj_set_pos(sub_content, 16, 54);
    lv_obj_set_size(sub_content, 448, 204);
}

void build_settings() {
    button(sub_content, english ? "Language" : "语言设置", 0, 0, 216, 56,
           open_page_event, reinterpret_cast<void *>(static_cast<intptr_t>(Page::Language)));
    button(sub_content, english ? "Device information" : "设备信息", 232, 0, 216, 56,
           open_page_event, reinterpret_cast<void *>(static_cast<intptr_t>(Page::Device)));
    button(sub_content, english ? "Warranty" : "保修信息", 0, 70, 216, 56,
           open_page_event, reinterpret_cast<void *>(static_cast<intptr_t>(Page::Warranty)));
    button(sub_content, english ? "About" : "关于本机", 232, 70, 216, 56,
           open_page_event, reinterpret_cast<void *>(static_cast<intptr_t>(Page::About)));
    label(sub_content, english ? "Settings are available only while ready."
                               : "设置仅在准备就绪状态下开放",
          0, 151, 448, 28, zh_font(), 0x8AA8C6);
}

void build_language() {
    button(sub_content, "简体中文", 32, 30, 176, 62, language_select_event,
           reinterpret_cast<void *>(0));
    button(sub_content, "English", 240, 30, 176, 62, language_select_event,
           reinterpret_cast<void *>(1));
    label(sub_content, english ? "Current: English" : "当前：简体中文",
          0, 120, 448, 30, zh_font(), 0x38D79A);
}

void build_device() {
    const hy200::Status &s = link->status();
    char text[320] = {};
    snprintf(text, sizeof(text),
             english ? "Model: HY200-C\nHost link: %s\nBand: %s\nFrequency: %.2f kHz\nTracking: %s\nI_OUT: %u"
                     : "设备型号：HY200-C\n主机通讯：%s\n当前频段：%s\n实时频率：%.2f KHz\n锁频状态：%s\nI_OUT：%u",
             link->online(millis()) ? (english ? "Connected" : "已连接") : (english ? "Disconnected" : "未连接"),
             s.gear35 ? "35KHz" : "28KHz", s.frequency_hz / 1000.0,
             s.tracking_locked ? (english ? "Locked" : "已锁定") : (english ? "Searching" : "追频中"),
             static_cast<unsigned>(s.iout_adc));
    lv_obj_t *info = label(sub_content, text, 20, 5, 408, 190, zh_font(),
                           0xD8F5FF, LV_TEXT_ALIGN_LEFT);
    lv_obj_set_style_text_line_space(info, 8, 0);
}

void make_serial(char *out, size_t size) {
    const uint64_t mac = ESP.getEfuseMac();
    snprintf(out, size, "HY200C-%06llX",
             static_cast<unsigned long long>(mac & 0xFFFFFFULL));
}

void build_warranty() {
    char serial[32] = {};
    make_serial(serial, sizeof(serial));
    char text[360] = {};
    snprintf(text, sizeof(text),
             english ? "Product: HY200-C\nSerial number: %s\nFactory date: Not registered\nWarranty start: Not registered\nWarranty period: 12 months\nService contact: Not registered"
                     : "产品型号：HY200-C\n设备序列号：%s\n出厂日期：未登记\n保修开始：未登记\n保修期限：12个月\n售后联系方式：未登记",
             serial);
    lv_obj_t *info = label(sub_content, text, 20, 5, 408, 190, zh_font(),
                           0xD8F5FF, LV_TEXT_ALIGN_LEFT);
    lv_obj_set_style_text_line_space(info, 8, 0);
}

void build_about() {
    const hy200::Version &v = link->version();
    char serial[32] = {};
    make_serial(serial, sizeof(serial));
    char host_product[32] = "--";
    char host_firmware[32] = "--";
    char protocol[16] = "--";
    if (v.valid) {
        snprintf(host_product, sizeof(host_product), "V%u.%u.%u-RC%u",
                 v.product_major, v.product_minor, v.product_patch, v.product_rc);
        snprintf(host_firmware, sizeof(host_firmware), "V%u.%u.%u",
                 v.host_major, v.host_minor, v.host_patch);
        snprintf(protocol, sizeof(protocol), "V%u", v.protocol);
    }
    char text[420] = {};
    snprintf(text, sizeof(text),
             english ? "HY200-C 4.3-inch control panel\nPanel firmware: %s\nHost product: %s\nHost firmware: %s\nProtocol: %s\nDisplay: 480 x 272\nInterface: UART 115200 8N1\nSerial: %s"
                     : "HY200-C 4.3英寸控制面板\n面板固件：%s\n主机产品：%s\n主机固件：%s\n通讯协议：%s\n屏幕规格：480×272\n通讯接口：UART 115200 8N1\n设备序列号：%s",
             firmware_version, host_product, host_firmware, protocol, serial);
    lv_obj_t *info = label(sub_content, text, 12, 0, 424, 200, zh_font(),
                           0xD8F5FF, LV_TEXT_ALIGN_LEFT);
    lv_obj_set_style_text_line_space(info, 5, 0);
}

void show_page(Page next) {
    page = next;
    const bool main = page == Page::Main;
    if (main) {
        lv_obj_clear_flag(main_page, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(sub_page, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_add_flag(main_page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(sub_page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clean(sub_content);
    const char *title = "设置";
    if (page == Page::Language) title = english ? "Language" : "语言设置";
    else if (page == Page::Device) title = english ? "Device information" : "设备信息";
    else if (page == Page::Warranty) title = english ? "Warranty" : "保修信息";
    else if (page == Page::About) title = english ? "About" : "关于本机";
    lv_label_set_text(sub_title, title);
    if (page == Page::Settings) build_settings();
    else if (page == Page::Language) build_language();
    else if (page == Page::Device) build_device();
    else if (page == Page::Warranty) build_warranty();
    else if (page == Page::About) build_about();
}

void refresh_main(uint32_t now_ms) {
    const MachineView view = current_view(now_ms);
    const hy200::Status &s = link->status();
    if (page != Page::Main && view != MachineView::Ready) {
        show_page(Page::Main);
        render_dirty = true;
    }

    if (s.running && !was_running) {
        running_started_ms = now_ms;
        countdown_active = true;
    } else if (!s.running && was_running) {
        countdown_active = false;
        link->set_run(false);
    }
    // Mirror the physical handle only while it remains pressed. Releasing it
    // must immediately remove the UART demand; otherwise OR-ing both run
    // sources in the host would keep ultrasonic output active unexpectedly.
    if (s.running && s.handle_pressed && countdown_active) link->set_run(true);
    if (!s.handle_pressed) link->set_run(false);
    was_running = s.running;
    if (countdown_active && s.running &&
        static_cast<uint32_t>(now_ms - running_started_ms) >=
            static_cast<uint32_t>(duration_seconds) * 1000U) {
        link->set_run(false);
        countdown_active = false;
    }
    uint32_t shown_centiseconds = static_cast<uint32_t>(duration_seconds) * 100U;
    if (s.running) {
        const uint32_t total_ms = static_cast<uint32_t>(duration_seconds) * 1000U;
        const uint32_t elapsed_ms = now_ms - running_started_ms;
        const uint32_t remaining_ms = elapsed_ms >= total_ms ? 0U : total_ms - elapsed_ms;
        // Round up to the next centisecond so the display starts at N.00s and
        // reaches 0.00s without showing an early zero.
        shown_centiseconds = (remaining_ms + 9U) / 10U;
    }
    // STATUS arrives every 50 ms, but most frames contain identical display
    // values. Avoid rebuilding the same LVGL objects continuously: on RGB
    // panels that creates needless buffer swaps and visible static shimmer.
    const uint32_t display_signature =
        s.frequency_hz ^ (static_cast<uint32_t>(s.raw_flags) << 24) ^
        (static_cast<uint32_t>(s.lock_reason) << 16);
    const bool animation_due = view == MachineView::Running;
    const bool needs_render = render_dirty ||
                              display_signature != last_rendered_signature ||
                              shown_centiseconds != last_rendered_centiseconds ||
                              view != last_rendered_view || animation_due;
    if (!needs_render) return;
    render_dirty = false;
    last_rendered_signature = display_signature;
    last_rendered_centiseconds = shown_centiseconds;
    last_rendered_view = view;

    char frequency[24] = {};
    snprintf(frequency, sizeof(frequency), "%.2f", s.frequency_hz > 0 ? s.frequency_hz / 1000.0 : (selected_gear35 ? 35.0 : 28.0));
    lv_label_set_text(frequency_value, frequency);
    lv_label_set_text_fmt(time_value, "%u.%02us",
                          static_cast<unsigned>(shown_centiseconds / 100U),
                          static_cast<unsigned>(shown_centiseconds % 100U));
    lv_label_set_text(gear28, "28KHz");
    lv_label_set_text(gear35, "35KHz");
    const bool gear28_active = !selected_gear35;
    lv_obj_set_style_bg_opa(gear28_box, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(gear28_box,
                              color(gear28_active ? 0x0B65C2 : 0x071522), 0);
    lv_obj_set_style_border_color(gear28_box,
                                  color(gear28_active ? 0x43A5FF : 0x3A4A58), 0);
    lv_obj_set_style_text_color(gear28,
                                color(gear28_active ? 0xFFFFFF : 0x81909D), 0);
    lv_obj_set_style_bg_opa(gear35_box, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(gear35_box,
                              color(selected_gear35 ? 0x0B65C2 : 0x071522), 0);
    lv_obj_set_style_border_color(gear35_box,
                                  color(selected_gear35 ? 0x43A5FF : 0x3A4A58), 0);
    lv_obj_set_style_text_color(gear35,
                                color(selected_gear35 ? 0xFFFFFF : 0x81909D), 0);

    uint32_t accent = 0x2561FA;
    const char *title = english ? "READY" : "准备就绪";
    const char *hint = english ? "AUTO FREQUENCY TRACKING" : "自动扫频 / 锁频 / 追频";
    if (view == MachineView::Disconnected) {
        accent = 0xE69010; title = english ? "OFFLINE" : "通讯断开";
        hint = english ? "CHECK UART CONNECTION" : "请检查主机串口连接";
    } else if (view == MachineView::Running) {
        accent = 0x00D419; title = english ? "RUNNING" : "运行中";
        hint = s.tracking_locked ? (english ? "FREQUENCY LOCKED" : "已锁频，实时追频中")
                                 : (english ? "SEARCHING FREQUENCY" : "正在自动扫频追频");
    } else if (view == MachineView::Calibrating) {
        accent = 0x00D419; title = english ? "CALIBRATING" : "频率校准中";
        hint = english ? "KEEP HANDLE RELEASED" : "请保持手柄释放并等待";
    } else if (view == MachineView::Alarm) {
        accent = 0xE61010; title = english ? "ALARM" : "主机报警";
        if (reset_failures >= 3) hint = english ? "WAIT FOR MACHINE TO COOL" : "请等待机器冷却后再试";
        else if (s.lock_reason == 3) hint = english ? "OVERCURRENT PROTECTION" : "过载 / 过流保护";
        else if (s.lock_reason == 4) hint = english ? "TEMPERATURE PROTECTION" : "温度保护，请等待冷却";
        else if (s.lock_reason == 5) hint = english ? "DEVICE LOCKED" : "设备已锁定";
        else hint = english ? "HOST FAULT" : "主机故障";
    } else if (s.calibration_ok) {
        hint = english ? "CALIBRATION SUCCEEDED" : "频率校准成功";
    } else if (s.calibration_invalid) {
        hint = english ? "INVALID CALIBRATION STATUS" : "校准状态异常";
    } else if (s.calibration_ng) {
        hint = english ? "CALIBRATION FAILED" : "频率校准失败";
    }
    lv_label_set_text(state_label, title);
    lv_label_set_text(state_hint, hint);
    lv_obj_set_style_text_color(state_label, color(accent), 0);
    lv_obj_set_style_text_color(state_hint, color(view == MachineView::Alarm ? 0xE61010 : 0xB9D5E8), 0);

    const int active = view == MachineView::Running
                           ? static_cast<int>(((now_ms - running_started_ms) % 1500U) * 27U / 1500U)
                           : 26;
    for (int i = 0; i < 27; ++i) {
        uint32_t cell_color = 0x2561FA;
        if (view == MachineView::Running || view == MachineView::Calibrating) cell_color = 0x00D419;
        else if (view == MachineView::Alarm) cell_color = 0xE61010;
        else if (view == MachineView::Disconnected) cell_color = 0x5A6570;
        lv_obj_set_style_bg_color(progress_cells[i], color(cell_color), 0);
        lv_obj_set_style_bg_opa(progress_cells[i], i <= active ? LV_OPA_COVER : LV_OPA_20, 0);
    }
    const bool ready = view == MachineView::Ready;
    const bool local_ready = ready || view == MachineView::Disconnected;
    if (ready) lv_obj_clear_state(scan_button, LV_STATE_DISABLED);
    else lv_obj_add_state(scan_button, LV_STATE_DISABLED);
    if (local_ready) {
        lv_obj_clear_state(time_button, LV_STATE_DISABLED);
        lv_obj_clear_state(mode_button, LV_STATE_DISABLED);
        lv_obj_clear_state(language_button, LV_STATE_DISABLED);
        lv_obj_clear_state(gear28_box, LV_STATE_DISABLED);
        lv_obj_clear_state(gear35_box, LV_STATE_DISABLED);
    } else {
        lv_obj_add_state(time_button, LV_STATE_DISABLED);
        lv_obj_add_state(mode_button, LV_STATE_DISABLED);
        lv_obj_add_state(language_button, LV_STATE_DISABLED);
        lv_obj_add_state(gear28_box, LV_STATE_DISABLED);
        lv_obj_add_state(gear35_box, LV_STATE_DISABLED);
    }
}
}  // namespace

void begin(hy200::Protocol *protocol, const char *panel_version) {
    link = protocol;
    firmware_version = panel_version;
    preferences.begin("hy200-ui", false);
    english = preferences.getBool("english", false);
    selected_gear35 = preferences.getBool("gear35", false);
    cutting_mode = preferences.getBool("cutting", false);
    duration_seconds = preferences.getUChar("duration", 5);
    duration_seconds = static_cast<uint8_t>(constrain(duration_seconds, 1, 99));
    link->set_duration_seconds(duration_seconds);
    pinMode(kLeftKeyPin, INPUT_PULLUP);
    pinMode(kRightKeyPin, INPUT_PULLUP);
    pinMode(kConfirmKeyPin, INPUT_PULLUP);

    screen = lv_obj_create(nullptr);
    clean(screen);
    lv_obj_set_size(screen, 480, 272);
    create_main();
    create_sub_page();
    show_page(Page::Main);
    lv_scr_load(screen);
    apply_key_focus();
}

void loop(uint32_t now_ms) {
    if (link == nullptr || screen == nullptr) return;
    poll_keys(now_ms);
    const uint32_t reset_action = link->reset_action_generation();
    if (reset_action != observed_reset_action) {
        observed_reset_action = reset_action;
        reset_action_denied = false;
    }
    if (link->ack().generation != last_ack_generation) {
        last_ack_generation = link->ack().generation;
        // Three wire pulses belong to one user action. Count at most one
        // denied reset per click instead of one per ACK.
        if (link->ack().result == 3 && !reset_action_denied) {
            reset_action_denied = true;
            ++reset_failures;
        }
    }
    if (link->status().sequence != last_status_sequence) {
        last_status_sequence = link->status().sequence;
        if (!link->status().fault && link->status().lock_reason == 0) reset_failures = 0;
    }
    if (static_cast<uint32_t>(now_ms - last_refresh_ms) >= 50) {
        refresh_main(now_ms);
        last_refresh_ms = now_ms;
    }
}

void debug_key_left() { navigate_key(-1); }
void debug_key_right() { navigate_key(1); }
void debug_key_confirm() { confirm_key(); }

bool active() { return screen != nullptr; }

}  // namespace hy200_ui
