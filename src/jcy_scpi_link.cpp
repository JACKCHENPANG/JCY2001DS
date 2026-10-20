#include "jcy_scpi_link.h"

#include <cctype>
#include <cstdarg>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace {
char *trim_text(char *text) {
    if (text == nullptr) {
        return text;
    }
    while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) {
        ++text;
    }
    char *end = text + strlen(text);
    while (end > text && isspace(static_cast<unsigned char>(end[-1]))) {
        --end;
    }
    *end = '\0';
    return text;
}

bool starts_with_ignore_case(const char *text, const char *prefix) {
    if (text == nullptr || prefix == nullptr) {
        return false;
    }
    while (*prefix != '\0') {
        if (*text == '\0' ||
            tolower(static_cast<unsigned char>(*text)) !=
                tolower(static_cast<unsigned char>(*prefix))) {
            return false;
        }
        ++text;
        ++prefix;
    }
    return true;
}

bool equals_ignore_case(const char *text, const char *expected) {
    return text != nullptr && expected != nullptr &&
           starts_with_ignore_case(text, expected) &&
           text[strlen(expected)] == '\0';
}

bool ends_with_ignore_case(const char *text, const char *suffix) {
    if (text == nullptr || suffix == nullptr) {
        return false;
    }
    const size_t text_length = strlen(text);
    const size_t suffix_length = strlen(suffix);
    if (suffix_length > text_length) {
        return false;
    }
    return starts_with_ignore_case(text + text_length - suffix_length, suffix);
}

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

int parse_error_code(const char *text) {
    if (text == nullptr) {
        return -1;
    }
    const char *open = strchr(text, '<');
    if (open == nullptr) {
        return starts_with_ignore_case(text, "OK") ? 0 : -1;
    }
    return atoi(open + 1);
}

bool parse_number_with_suffix(const char *text, const char *suffix,
                              double &value) {
    if (!ends_with_ignore_case(text, suffix)) {
        return false;
    }
    char *end = nullptr;
    value = strtod(text, &end);
    if (end == text) {
        return false;
    }
    while (*end != '\0' && isspace(static_cast<unsigned char>(*end))) {
        ++end;
    }
    return starts_with_ignore_case(end, suffix);
}

bool parse_primary_rating(const char *text, char *rating,
                          size_t rating_length,
                          int &voltage_mv, int &current_ma) {
    if (text == nullptr || rating == nullptr || rating_length == 0) {
        return false;
    }
    const char *end = strpbrk(text, "/(");
    const size_t length = end == nullptr
                              ? strlen(text)
                              : static_cast<size_t>(end - text);
    if (length == 0 || length >= rating_length) {
        return false;
    }
    memcpy(rating, text, length);
    rating[length] = '\0';

    char *voltage_end = nullptr;
    const double voltage = strtod(rating, &voltage_end);
    if (voltage_end == rating ||
        toupper(static_cast<unsigned char>(*voltage_end)) != 'V') {
        return false;
    }
    char *current_end = nullptr;
    const double current = strtod(voltage_end + 1, &current_end);
    if (current_end == voltage_end + 1 ||
        toupper(static_cast<unsigned char>(*current_end)) != 'A') {
        return false;
    }
    voltage_mv = static_cast<int>(lround(voltage * 1000.0));
    current_ma = static_cast<int>(lround(current * 1000.0));
    return voltage_mv > 0 && current_ma > 0;
}

void format_current_parameter(char *buffer, size_t length, int current_ua) {
    if (current_ua % 1000 == 0) {
        snprintf(buffer, length, "%dmA", current_ua / 1000);
    } else {
        snprintf(buffer, length, "%duA", current_ua);
    }
}

JcyScpiRampState parse_ramp_state(const char *text) {
    if (starts_with_ignore_case(text, "PROCESSING")) {
        return JcyScpiRampState::Processing;
    }
    if (starts_with_ignore_case(text, "PASS")) {
        return JcyScpiRampState::Pass;
    }
    if (starts_with_ignore_case(text, "UP")) {
        return JcyScpiRampState::Up;
    }
    if (starts_with_ignore_case(text, "DOWN")) {
        return JcyScpiRampState::Down;
    }
    if (starts_with_ignore_case(text, "STOP")) {
        return JcyScpiRampState::Stop;
    }
    if (contains_ignore_case(text, "OCP")) {
        return JcyScpiRampState::ErrorOcp;
    }
    if (contains_ignore_case(text, "OVP")) {
        return JcyScpiRampState::ErrorOvp;
    }
    if (starts_with_ignore_case(text, "ERROR")) {
        return JcyScpiRampState::Error;
    }
    return JcyScpiRampState::Unknown;
}

bool parse_mode_response(const char *text, JcyScpiMode &mode) {
    // SCPI 0015 specifies BATTER_CC (not BATTERY_CC) for the constant-current
    // battery mode. Parse an exact token so an unexpected reply cannot be
    // accepted as a verified mode transition.
    if (equals_ignore_case(text, "BATTERY")) {
        mode = JcyScpiMode::Battery;
        return true;
    }
    if (equals_ignore_case(text, "BATTER_CC")) {
        mode = JcyScpiMode::BatteryConstantCurrent;
        return true;
    }
    if (equals_ignore_case(text, "SOURCE")) {
        mode = JcyScpiMode::Source;
        return true;
    }
    if (equals_ignore_case(text, "LOAD")) {
        mode = JcyScpiMode::Load;
        return true;
    }
    if (equals_ignore_case(text, "CONST_R")) {
        mode = JcyScpiMode::ConstantResistance;
        return true;
    }
    return false;
}
}  // namespace

JcyScpiLink::JcyScpiLink(uint8_t uart_number) : serial_(uart_number) {}

void JcyScpiLink::begin(uint32_t baud, int rx_pin, int tx_pin, uint8_t address) {
    address_ = constrain(address, 1, 120);
    serial_.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
    started_ = true;
    passthrough_active_ = false;
    iap_transport_active_ = false;
    polling_suspended_ = false;
    queue_count_ = 0;
    command_active_ = false;
    receive_length_ = 0;
    identity_ = {};
    last_poll_request_ms_ = millis() - poll_interval_ms_;
    Serial.printf("TTL UART ready baud=%lu RX=%d TX=%d CH=%u\n",
                  static_cast<unsigned long>(baud), rx_pin, tx_pin,
                  static_cast<unsigned>(address_));
}

void JcyScpiLink::set_passthrough(bool enabled) {
    if (!started_ || iap_transport_active_ || passthrough_active_ == enabled) {
        return;
    }

    passthrough_active_ = enabled;
    queue_count_ = 0;
    active_command_ = {};
    command_active_ = false;
    receive_length_ = 0;
    event_count_ = 0;
    while (serial_.available() > 0) {
        serial_.read();
    }
    last_poll_request_ms_ = millis() - poll_interval_ms_;
}

bool JcyScpiLink::enter_iap_transport() {
    if (!started_ || passthrough_active_ || iap_transport_active_) {
        return false;
    }
    iap_transport_active_ = true;
    polling_suspended_ = true;
    queue_count_ = 0;
    active_command_ = {};
    command_active_ = false;
    receive_length_ = 0;
    event_count_ = 0;
    while (serial_.available() > 0) serial_.read();
    Serial.println("TTL IAP transport active");
    return true;
}

void JcyScpiLink::exit_iap_transport() {
    if (!iap_transport_active_) return;
    while (serial_.available() > 0) serial_.read();
    iap_transport_active_ = false;
    polling_suspended_ = false;
    last_poll_request_ms_ = millis();
    Serial.println("TTL IAP transport closed");
}

bool JcyScpiLink::iap_transport_active() const {
    return iap_transport_active_;
}

int JcyScpiLink::iap_available() {
    return started_ && iap_transport_active_ ? serial_.available() : 0;
}

int JcyScpiLink::iap_read() {
    return started_ && iap_transport_active_ ? serial_.read() : -1;
}

size_t JcyScpiLink::iap_write(const uint8_t *data, size_t length) {
    if (!started_ || !iap_transport_active_ || data == nullptr || length == 0) {
        return 0;
    }
    return serial_.write(data, length);
}

void JcyScpiLink::iap_flush() {
    if (started_ && iap_transport_active_) serial_.flush();
}

bool JcyScpiLink::passthrough_active() const {
    return passthrough_active_;
}

int JcyScpiLink::passthrough_available() {
    return started_ && passthrough_active_ ? serial_.available() : 0;
}

int JcyScpiLink::passthrough_available_for_write() {
    return started_ && passthrough_active_
               ? serial_.availableForWrite()
               : 0;
}

int JcyScpiLink::passthrough_read() {
    return started_ && passthrough_active_ ? serial_.read() : -1;
}

size_t JcyScpiLink::passthrough_write(const uint8_t *data, size_t length) {
    if (!started_ || !passthrough_active_ || data == nullptr || length == 0) {
        return 0;
    }
    return serial_.write(data, length);
}

void JcyScpiLink::set_poll_interval(uint32_t interval_ms) {
    poll_interval_ms_ = constrain(interval_ms, 100U, 5000U);
}

void JcyScpiLink::set_polling_suspended(bool suspended) {
    if (polling_suspended_ == suspended) {
        return;
    }
    polling_suspended_ = suspended;
    if (suspended) {
        // 丢弃尚未发送的周期查询；正在等待回应的命令仍由 loop() 正常
        // 收尾，输出关闭等安全命令也不能被 OTA 静默模式取消。
        size_t write_index = 0;
        for (size_t read_index = 0; read_index < queue_count_; ++read_index) {
            if (queue_[read_index].kind == JcyScpiCommandKind::MeasureAll) {
                continue;
            }
            if (write_index != read_index) {
                queue_[write_index] = queue_[read_index];
            }
            ++write_index;
        }
        queue_count_ = write_index;
    } else {
        // 恢复后等待一个完整周期再查询，避免刚退出 OTA 就立即形成突发流量。
        last_poll_request_ms_ = millis();
    }
    Serial.printf("TTL polling %s\n", suspended ? "SUSPENDED" : "RESUMED");
}

bool JcyScpiLink::enqueue(JcyScpiCommandKind kind, int requested_value,
                          const char *format, ...) {
    if (!started_ || passthrough_active_ || iap_transport_active_ ||
        format == nullptr) {
        return false;
    }

    char command_text[96] = {};
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(command_text, sizeof(command_text), format, arguments);
    va_end(arguments);

    // If the currently transmitted output command already matches the newest
    // user intent, discard a stale queued output override. This keeps rapid
    // ON->OFF->ON taps from sending an unnecessary final command.
    if (kind == JcyScpiCommandKind::SetOutput && command_active_ &&
        active_command_.kind == kind &&
        active_command_.requested_value == requested_value) {
        for (size_t i = 0; i < queue_count_;) {
            if (queue_[i].kind != kind) {
                ++i;
                continue;
            }
            for (size_t j = i + 1; j < queue_count_; ++j) {
                queue_[j - 1] = queue_[j];
            }
            --queue_count_;
        }
        return true;
    }

    // 连续点按加减键时只保留同类命令的最新值，避免底层队列积压。
    if (kind != JcyScpiCommandKind::MeasureAll) {
        for (size_t i = 0; i < queue_count_; ++i) {
            if (queue_[i].kind == kind) {
                strlcpy(queue_[i].text, command_text,
                        sizeof(queue_[i].text));
                queue_[i].requested_value = requested_value;

                // 输出关闭是安全优先命令。若它已经在队列中，将它移到
                // 最前面，不能被普通测量或参数设置阻塞。
                if (kind == JcyScpiCommandKind::SetOutput &&
                    requested_value == 0 && i > 0) {
                    const Command priority_command = queue_[i];
                    for (size_t j = i; j > 0; --j) {
                        queue_[j] = queue_[j - 1];
                    }
                    queue_[0] = priority_command;
                }
                return true;
            }
        }
    } else if (queued(kind) ||
               (command_active_ && active_command_.kind == kind)) {
        return true;
    }

    if (queue_count_ >= COMMAND_QUEUE_CAPACITY &&
        kind == JcyScpiCommandKind::SetOutput && requested_value == 0) {
        // Make room for emergency OFF by dropping a queued periodic poll.
        // Never discard a configuration/write command for this purpose.
        for (size_t i = queue_count_; i > 0; --i) {
            if (queue_[i - 1].kind != JcyScpiCommandKind::MeasureAll) {
                continue;
            }
            for (size_t j = i; j < queue_count_; ++j) {
                queue_[j - 1] = queue_[j];
            }
            --queue_count_;
            break;
        }
    }

    if (queue_count_ >= COMMAND_QUEUE_CAPACITY) {
        Serial.printf("TTL queue full kind=%u\n",
                      static_cast<unsigned>(kind));
        return false;
    }

    size_t insert_index = queue_count_;
    if (kind == JcyScpiCommandKind::SetOutput && requested_value == 0) {
        for (size_t i = queue_count_; i > 0; --i) {
            queue_[i] = queue_[i - 1];
        }
        insert_index = 0;
    }
    ++queue_count_;
    Command &command = queue_[insert_index];
    command.kind = kind;
    command.requested_value = requested_value;
    strlcpy(command.text, command_text, sizeof(command.text));
    return true;
}

bool JcyScpiLink::request_measurements() {
    return enqueue(JcyScpiCommandKind::MeasureAll, 0,
                   "CH %u;MEASure:ALL?", address_);
}

bool JcyScpiLink::request_identity() {
    return enqueue(JcyScpiCommandKind::QueryIdentity, 0,
                   "CH %u;*IDN?", address_);
}

bool JcyScpiLink::set_voltage_mv(int voltage_mv) {
    return enqueue(JcyScpiCommandKind::SetVoltage, voltage_mv,
                   "CH %u;SETup:VOLTage %dmV", address_, voltage_mv);
}

bool JcyScpiLink::set_current_ma(int current_ma) {
    return enqueue(JcyScpiCommandKind::SetCurrent, current_ma,
                   "CH %u;SETup:CURRent %dmA", address_, current_ma);
}

bool JcyScpiLink::set_output(bool enabled) {
    return enqueue(JcyScpiCommandKind::SetOutput, enabled ? 1 : 0,
                   "CH %u;SETup:OUT %s", address_, enabled ? "ON" : "OFF");
}

bool JcyScpiLink::set_mode(JcyScpiMode mode) {
    const char *name = "BATTERY";
    if (mode == JcyScpiMode::Source) {
        name = "SOURCE";
    } else if (mode == JcyScpiMode::Load) {
        name = "LOAD";
    } else if (mode == JcyScpiMode::BatteryConstantCurrent) {
        // 协议原文拼写为 BATTER_CC，不能改成 BATTERY_CC。
        name = "BATTER_CC";
    } else if (mode == JcyScpiMode::ConstantResistance) {
        name = "CONST_R";
    }
    return enqueue(JcyScpiCommandKind::SetMode, static_cast<int>(mode),
                   "CH %u;SETup:MODE %s", address_, name);
}

bool JcyScpiLink::query_mode() {
    return enqueue(JcyScpiCommandKind::QueryMode, -1,
                   "CH %u;SETup:MODE?", address_);
}

bool JcyScpiLink::set_protection_delay_ms(int delay_ms) {
    return enqueue(JcyScpiCommandKind::SetProtectionDelay, delay_ms,
                   "CH %u;SETup:PROTect:DELAY %dmS", address_, delay_ms);
}

bool JcyScpiLink::set_current_mode(bool high_range) {
    return enqueue(JcyScpiCommandKind::SetCurrentMode,
                   high_range ? 1 : 0,
                   "CH %u;SETup:CURRent:MODE %s", address_,
                   high_range ? "HIGH" : "LOW");
}

bool JcyScpiLink::set_ramp(int start_mv, int end_mv, int time_ms,
                           int limit_up_ua, int limit_down_ua) {
    char limit_up[24] = {};
    char limit_down[24] = {};
    format_current_parameter(limit_up, sizeof(limit_up), limit_up_ua);
    format_current_parameter(limit_down, sizeof(limit_down), limit_down_ua);
    return enqueue(JcyScpiCommandKind::SetRamp, 0,
                   "CH %u;SETup:RAMP %dmV,%dmV,%dmS,%s,%s",
                   address_, start_mv, end_mv, time_ms,
                   limit_up, limit_down);
}

bool JcyScpiLink::trigger_ramp() {
    return enqueue(JcyScpiCommandKind::TriggerRamp, 0,
                   "CH %u;SETup:TRIGger", address_);
}

bool JcyScpiLink::query_ramp_state() {
    return enqueue(JcyScpiCommandKind::QueryRampState, 0,
                   "CH %u;SETup:RAMP:STATe?", address_);
}

const JcyScpiMeasurements &JcyScpiLink::measurements() const {
    return measurements_;
}

const JcyScpiIdentity &JcyScpiLink::identity() const {
    return identity_;
}

const JcyScpiRampResult &JcyScpiLink::ramp_result() const {
    return ramp_result_;
}

JcyScpiDiagnostics JcyScpiLink::diagnostics() const {
    JcyScpiDiagnostics result;
    result.raw_rx_bytes = raw_rx_bytes_;
    result.complete_lines = complete_lines_;
    result.accepted_lines = accepted_lines_;
    result.parse_failures = parse_failures_;
    result.unsolicited_lines = unsolicited_lines_;
    result.timeouts = timeout_count_;
    result.line_overflows = line_overflows_;
    result.last_byte_ms = last_byte_ms_;
    result.last_line_ms = last_line_ms_;
    result.last_accepted_ms = last_accepted_ms_;
    result.command_active = command_active_;
    result.active_kind = active_command_.kind;
    result.queued_commands = queue_count_;
    strlcpy(result.last_line, last_received_line_, sizeof(result.last_line));
    return result;
}

int JcyScpiLink::pending_rx_bytes() {
    return started_ ? serial_.available() : 0;
}

void JcyScpiLink::reset_diagnostics() {
    raw_rx_bytes_ = 0;
    complete_lines_ = 0;
    accepted_lines_ = 0;
    parse_failures_ = 0;
    unsolicited_lines_ = 0;
    timeout_count_ = 0;
    line_overflows_ = 0;
    last_byte_ms_ = 0;
    last_line_ms_ = 0;
    last_accepted_ms_ = 0;
    last_received_line_[0] = '\0';
}

void JcyScpiLink::clear_ramp_result() {
    ramp_result_ = {};
}

bool JcyScpiLink::measurements_fresh(uint32_t now_ms,
                                     uint32_t maximum_age_ms) const {
    return measurements_.valid &&
           now_ms - measurements_.updated_ms <= maximum_age_ms;
}

bool JcyScpiLink::online(uint32_t now_ms, uint32_t maximum_age_ms) const {
    return last_response_ms_ != 0 && now_ms - last_response_ms_ <= maximum_age_ms;
}

bool JcyScpiLink::queued(JcyScpiCommandKind kind) const {
    for (size_t i = 0; i < queue_count_; ++i) {
        if (queue_[i].kind == kind) {
            return true;
        }
    }
    return false;
}

bool JcyScpiLink::command_pending(JcyScpiCommandKind kind) const {
    return (command_active_ && active_command_.kind == kind) || queued(kind);
}

void JcyScpiLink::start_next_command(uint32_t now_ms) {
    if (!started_ || command_active_ || queue_count_ == 0) {
        return;
    }
    active_command_ = queue_[0];
    for (size_t i = 1; i < queue_count_; ++i) {
        queue_[i - 1] = queue_[i];
    }
    --queue_count_;
    command_active_ = true;
    command_sent_ms_ = now_ms;
    serial_.print(active_command_.text);
    serial_.write('\r');
    serial_.write('\n');
    Serial.printf("TTL TX: %s\n", active_command_.text);
}

void JcyScpiLink::push_event(const JcyScpiEvent &event) {
    if (event_count_ >= EVENT_QUEUE_CAPACITY) {
        for (size_t i = 1; i < event_count_; ++i) {
            events_[i - 1] = events_[i];
        }
        --event_count_;
    }
    events_[event_count_++] = event;
}

bool JcyScpiLink::take_event(JcyScpiEvent &event) {
    if (event_count_ == 0) {
        return false;
    }
    event = events_[0];
    for (size_t i = 1; i < event_count_; ++i) {
        events_[i - 1] = events_[i];
    }
    --event_count_;
    return true;
}

void JcyScpiLink::complete_active(bool success, int error_code,
                                  uint32_t now_ms) {
    if (!command_active_) {
        return;
    }
    if (success) {
        last_response_ms_ = now_ms;
    }
    if (active_command_.kind != JcyScpiCommandKind::MeasureAll) {
        JcyScpiEvent event;
        event.kind = active_command_.kind;
        event.success = success;
        event.requested_value = active_command_.requested_value;
        event.error_code = error_code;
        push_event(event);
    }
    command_active_ = false;
    active_command_ = {};
}

bool JcyScpiLink::parse_measurements(char *payload, uint32_t now_ms) {
    char *fields[5] = {};
    char *save = nullptr;
    char *token = strtok_r(payload, ",", &save);
    size_t count = 0;
    while (token != nullptr && count < 5) {
        fields[count++] = trim_text(token);
        token = strtok_r(nullptr, ",", &save);
    }
    if (count != 5) {
        return false;
    }

    double voltage = 0.0;
    if (!parse_number_with_suffix(fields[0], "mV", voltage)) {
        return false;
    }

    double current = 0.0;
    int current_ua = 0;
    if (parse_number_with_suffix(fields[1], "mA", current)) {
        current_ua = static_cast<int>(lround(current * 1000.0));
    } else if (parse_number_with_suffix(fields[1], "uA", current)) {
        current_ua = static_cast<int>(lround(current));
    } else if (parse_number_with_suffix(fields[1], "A", current)) {
        current_ua = static_cast<int>(lround(current * 1000000.0));
    } else {
        return false;
    }

    char *temperature_end = nullptr;
    const double temperature = strtod(fields[3], &temperature_end);
    if (temperature_end == fields[3]) {
        return false;
    }

    const int error_code = parse_error_code(fields[4]);
    measurements_.voltage_mv = static_cast<int>(lround(voltage));
    measurements_.current_ua = current_ua;
    measurements_.output_on = starts_with_ignore_case(fields[2], "ON");
    measurements_.temperature_c10 = static_cast<int>(lround(temperature * 10.0));
    measurements_.error_code = max(error_code, 0);
    strlcpy(measurements_.status, fields[4], sizeof(measurements_.status));
    measurements_.valid = true;
    measurements_.updated_ms = now_ms;
    ++measurements_.sequence;
    last_response_ms_ = now_ms;
    return true;
}

bool JcyScpiLink::parse_identity(char *payload, uint32_t now_ms) {
    char *fields[3] = {};
    char *save = nullptr;
    char *token = strtok_r(payload, ",", &save);
    size_t count = 0;
    while (token != nullptr && count < 3) {
        fields[count++] = trim_text(token);
        token = strtok_r(nullptr, ",", &save);
    }
    if (count != 3 || token != nullptr) {
        return false;
    }

    char rating[sizeof(identity_.primary_rating)] = {};
    int voltage_mv = 0;
    int current_ma = 0;
    if (!parse_primary_rating(fields[1], rating, sizeof(rating),
                              voltage_mv, current_ma)) {
        return false;
    }

    strlcpy(identity_.model, fields[0], sizeof(identity_.model));
    strlcpy(identity_.specification, fields[1],
            sizeof(identity_.specification));
    strlcpy(identity_.firmware, fields[2], sizeof(identity_.firmware));
    strlcpy(identity_.primary_rating, rating,
            sizeof(identity_.primary_rating));
    identity_.primary_voltage_mv = voltage_mv;
    identity_.primary_current_ma = current_ma;
    identity_.valid = true;
    identity_.updated_ms = now_ms;
    ++identity_.sequence;
    last_response_ms_ = now_ms;
    return true;
}

bool JcyScpiLink::parse_ramp_result(char *payload, uint32_t now_ms) {
    char *fields[4] = {};
    char *save = nullptr;
    char *token = strtok_r(payload, ",", &save);
    size_t count = 0;
    while (token != nullptr && count < 4) {
        fields[count++] = trim_text(token);
        token = strtok_r(nullptr, ",", &save);
    }
    if (count != 4 || token != nullptr) {
        return false;
    }

    double voltage = 0.0;
    if (!parse_number_with_suffix(fields[0], "mV", voltage)) {
        return false;
    }

    double current = 0.0;
    int current_ua = 0;
    if (parse_number_with_suffix(fields[1], "mA", current)) {
        current_ua = static_cast<int>(lround(current * 1000.0));
    } else if (parse_number_with_suffix(fields[1], "uA", current)) {
        current_ua = static_cast<int>(lround(current));
    } else if (parse_number_with_suffix(fields[1], "A", current)) {
        current_ua = static_cast<int>(lround(current * 1000000.0));
    } else {
        return false;
    }

    double elapsed = 0.0;
    if (!parse_number_with_suffix(fields[2], "mS", elapsed)) {
        return false;
    }

    const JcyScpiRampState state = parse_ramp_state(fields[3]);
    if (state == JcyScpiRampState::Unknown) {
        return false;
    }
    ramp_result_.voltage_mv = static_cast<int>(lround(voltage));
    ramp_result_.current_ua = current_ua;
    ramp_result_.elapsed_ms = static_cast<int>(lround(elapsed));
    ramp_result_.state = state;
    ramp_result_.error_code = max(parse_error_code(fields[3]), 0);
    strlcpy(ramp_result_.state_text, fields[3],
            sizeof(ramp_result_.state_text));
    ramp_result_.valid = true;
    ramp_result_.terminal = state != JcyScpiRampState::Processing;
    ramp_result_.updated_ms = now_ms;
    ++ramp_result_.sequence;
    last_response_ms_ = now_ms;
    return true;
}

void JcyScpiLink::process_line(char *line, uint32_t now_ms) {
    char *clean = trim_text(line);
    if (*clean == '\0') {
        return;
    }
    ++complete_lines_;
    last_line_ms_ = now_ms;
    strlcpy(last_received_line_, clean, sizeof(last_received_line_));
    Serial.printf("TTL RX: %s\n", clean);

    char *payload = strchr(clean, ';');
    payload = trim_text(payload == nullptr ? clean : payload + 1);

    // TRIGger 的完成行是异步返回，可能在没有活动命令时到达，也可能
    // 穿插在 MEASure:ALL? 的响应之前。先识别 RAMP 结果，不能错误完成
    // 当前的普通测量命令。
    char ramp_copy[192] = {};
    strlcpy(ramp_copy, payload, sizeof(ramp_copy));
    if (parse_ramp_result(ramp_copy, now_ms)) {
        ++accepted_lines_;
        last_accepted_ms_ = now_ms;
        if (command_active_ &&
            active_command_.kind == JcyScpiCommandKind::QueryRampState) {
            complete_active(true, ramp_result_.error_code, now_ms);
        }
        return;
    }

    if (!command_active_) {
        ++unsolicited_lines_;
        return;
    }

    if (active_command_.kind == JcyScpiCommandKind::MeasureAll) {
        if (starts_with_ignore_case(payload, "ERROR")) {
            ++accepted_lines_;
            last_accepted_ms_ = now_ms;
            complete_active(false, parse_error_code(payload), now_ms);
            return;
        }
        const bool parsed = parse_measurements(payload, now_ms);
        if (parsed) {
            ++accepted_lines_;
            last_accepted_ms_ = now_ms;
        } else {
            ++parse_failures_;
        }
        complete_active(parsed, parsed ? 0 : -2, now_ms);
        return;
    }

    if (active_command_.kind == JcyScpiCommandKind::QueryIdentity) {
        if (starts_with_ignore_case(payload, "ERROR")) {
            ++accepted_lines_;
            last_accepted_ms_ = now_ms;
            complete_active(false, parse_error_code(payload), now_ms);
            return;
        }
        char identity_copy[192] = {};
        strlcpy(identity_copy, payload, sizeof(identity_copy));
        const bool parsed = parse_identity(identity_copy, now_ms);
        if (parsed) {
            ++accepted_lines_;
            last_accepted_ms_ = now_ms;
        } else {
            ++parse_failures_;
        }
        complete_active(parsed, parsed ? 0 : -2, now_ms);
        return;
    }

    if (active_command_.kind == JcyScpiCommandKind::QueryMode) {
        if (starts_with_ignore_case(payload, "ERROR")) {
            ++accepted_lines_;
            last_accepted_ms_ = now_ms;
            complete_active(false, parse_error_code(payload), now_ms);
            return;
        }
        JcyScpiMode reported_mode = JcyScpiMode::Battery;
        const bool parsed = parse_mode_response(payload, reported_mode);
        if (parsed) {
            // JcyScpiEvent carries the mode reported by SETup:MODE?, not the
            // mode we asked SETup:MODE to select.
            active_command_.requested_value =
                static_cast<int>(reported_mode);
            ++accepted_lines_;
            last_accepted_ms_ = now_ms;
        } else {
            ++parse_failures_;
        }
        complete_active(parsed, parsed ? 0 : -2, now_ms);
        return;
    }

    if (active_command_.kind == JcyScpiCommandKind::QueryRampState) {
        if (starts_with_ignore_case(payload, "ERROR")) {
            ++accepted_lines_;
            last_accepted_ms_ = now_ms;
            complete_active(false, parse_error_code(payload), now_ms);
        } else {
            ++parse_failures_;
            complete_active(false, -2, now_ms);
        }
        return;
    }

    if (starts_with_ignore_case(payload, "OK")) {
        ++accepted_lines_;
        last_accepted_ms_ = now_ms;
        complete_active(true, parse_error_code(payload), now_ms);
    } else if (starts_with_ignore_case(payload, "ERROR")) {
        ++accepted_lines_;
        last_accepted_ms_ = now_ms;
        complete_active(false, parse_error_code(payload), now_ms);
    } else {
        ++parse_failures_;
        complete_active(false, -2, now_ms);
    }
}

void JcyScpiLink::consume_receive_data(uint32_t now_ms) {
    while (serial_.available() > 0) {
        const char ch = static_cast<char>(serial_.read());
        ++raw_rx_bytes_;
        last_byte_ms_ = now_ms;
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            if (receive_length_ > 0) {
                receive_line_[receive_length_] = '\0';
                process_line(receive_line_, now_ms);
                receive_length_ = 0;
            }
            continue;
        }
        if (receive_length_ < sizeof(receive_line_) - 1) {
            receive_line_[receive_length_++] = ch;
        } else {
            receive_length_ = 0;
            ++line_overflows_;
            Serial.println("TTL RX line overflow");
        }
    }
}

void JcyScpiLink::loop(uint32_t now_ms) {
    if (!started_ || passthrough_active_ || iap_transport_active_) {
        return;
    }

    consume_receive_data(now_ms);

    if (command_active_ && now_ms - command_sent_ms_ > command_timeout_ms_) {
        ++timeout_count_;
        Serial.printf("TTL timeout kind=%u\n",
                      static_cast<unsigned>(active_command_.kind));
        complete_active(false, -1, now_ms);
    }

    if (!polling_suspended_ &&
        now_ms - last_poll_request_ms_ >= poll_interval_ms_) {
        request_measurements();
        last_poll_request_ms_ = now_ms;
    }

    start_next_command(now_ms);
}
