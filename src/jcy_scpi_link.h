#pragma once

#include <Arduino.h>

enum class JcyScpiCommandKind : uint8_t {
    None = 0,
    MeasureAll,
    QueryIdentity,
    SetVoltage,
    SetCurrent,
    SetOutput,
    SetMode,
    SetProtectionDelay,
    SetCurrentMode,
    SetRamp,
    TriggerRamp,
    QueryRampState,
    // SETup:MODE? verifies the lower instrument's reported mode after a
    // SETup:MODE acknowledgement.
    QueryMode,
};

enum class JcyScpiMode : uint8_t {
    Battery,
    Source,
    Load,
    BatteryConstantCurrent,
    ConstantResistance,
};

enum class JcyScpiRampState : uint8_t {
    Unknown = 0,
    Processing,
    Pass,
    Up,
    Down,
    Stop,
    ErrorOcp,
    ErrorOvp,
    Error,
};

struct JcyScpiMeasurements {
    int voltage_mv = 0;
    int current_ua = 0;
    int temperature_c10 = 0;
    bool output_on = false;
    int error_code = 0;
    char status[28] = "WAITING";
    bool valid = false;
    uint32_t updated_ms = 0;
    uint32_t sequence = 0;
};

struct JcyScpiIdentity {
    char model[24] = "--";
    char specification[48] = "--";
    char firmware[36] = "--";
    char primary_rating[20] = "25V10A";
    int primary_voltage_mv = 25000;
    int primary_current_ma = 10000;
    bool valid = false;
    uint32_t updated_ms = 0;
    uint32_t sequence = 0;
};

struct JcyScpiEvent {
    JcyScpiCommandKind kind = JcyScpiCommandKind::None;
    bool success = false;
    int requested_value = 0;
    int error_code = 0;
};

struct JcyScpiRampResult {
    int voltage_mv = 0;
    int current_ua = 0;
    int elapsed_ms = 0;
    int error_code = 0;
    JcyScpiRampState state = JcyScpiRampState::Unknown;
    char state_text[28] = "WAITING";
    bool valid = false;
    bool terminal = false;
    uint32_t updated_ms = 0;
    uint32_t sequence = 0;
};

struct JcyScpiDiagnostics {
    uint32_t raw_rx_bytes = 0;
    uint32_t complete_lines = 0;
    uint32_t accepted_lines = 0;
    uint32_t parse_failures = 0;
    uint32_t unsolicited_lines = 0;
    uint32_t timeouts = 0;
    uint32_t line_overflows = 0;
    uint32_t last_byte_ms = 0;
    uint32_t last_line_ms = 0;
    uint32_t last_accepted_ms = 0;
    bool command_active = false;
    JcyScpiCommandKind active_kind = JcyScpiCommandKind::None;
    size_t queued_commands = 0;
    char last_line[192] = {};
};

class JcyScpiLink {
public:
    explicit JcyScpiLink(uint8_t uart_number);

    void begin(uint32_t baud, int rx_pin, int tx_pin, uint8_t address = 1);
    void loop(uint32_t now_ms);
    void set_poll_interval(uint32_t interval_ms);
    // OTA 写 Flash 时暂停周期测量，保留正在执行的安全命令和串口接收，
    // 避免 350 ms 轮询与 Wi-Fi/Flash/RGB DMA 同时抢占资源。
    void set_polling_suspended(bool suspended);

    // Suspend SCPI and expose the lower-controller UART as a binary transport
    // for the resident STM32 IAP bootloader. Entering/exiting clears queues
    // and stale bytes; callers must keep the instrument output confirmed OFF.
    bool enter_iap_transport();
    void exit_iap_transport();
    bool iap_transport_active() const;
    int iap_available();
    int iap_read();
    size_t iap_write(const uint8_t *data, size_t length);
    void iap_flush();

    // 暂停内部SCPI队列，让上层直接桥接外部透传串口与底层TTL串口。
    // 进入或退出时会清空未完成命令和残留接收数据，避免两种模式串包。
    void set_passthrough(bool enabled);
    bool passthrough_active() const;
    int passthrough_available();
    int passthrough_available_for_write();
    int passthrough_read();
    size_t passthrough_write(const uint8_t *data, size_t length);

    bool request_measurements();
    bool request_identity();
    bool set_voltage_mv(int voltage_mv);
    bool set_current_ma(int current_ma);
    bool set_output(bool enabled);
    bool set_mode(JcyScpiMode mode);
    bool query_mode();
    bool set_protection_delay_ms(int delay_ms);
    bool set_current_mode(bool high_range);
    bool set_ramp(int start_mv, int end_mv, int time_ms,
                  int limit_up_ua, int limit_down_ua);
    bool trigger_ramp();
    bool query_ramp_state();

    const JcyScpiMeasurements &measurements() const;
    const JcyScpiIdentity &identity() const;
    const JcyScpiRampResult &ramp_result() const;
    JcyScpiDiagnostics diagnostics() const;
    int pending_rx_bytes();
    void reset_diagnostics();
    void clear_ramp_result();
    bool measurements_fresh(uint32_t now_ms, uint32_t maximum_age_ms = 1500) const;
    bool online(uint32_t now_ms, uint32_t maximum_age_ms = 2000) const;
    bool command_pending(JcyScpiCommandKind kind) const;
    bool take_event(JcyScpiEvent &event);

private:
    struct Command {
        JcyScpiCommandKind kind = JcyScpiCommandKind::None;
        char text[96] = {};
        int requested_value = 0;
    };

    static constexpr size_t COMMAND_QUEUE_CAPACITY = 10;
    static constexpr size_t EVENT_QUEUE_CAPACITY = 8;

    HardwareSerial serial_;
    uint8_t address_ = 1;
    bool started_ = false;
    bool passthrough_active_ = false;
    bool iap_transport_active_ = false;
    Command queue_[COMMAND_QUEUE_CAPACITY] = {};
    size_t queue_count_ = 0;
    Command active_command_;
    bool command_active_ = false;
    uint32_t command_sent_ms_ = 0;
    uint32_t command_timeout_ms_ = 450;
    uint32_t poll_interval_ms_ = 350;
    uint32_t last_poll_request_ms_ = 0;
    bool polling_suspended_ = false;
    uint32_t last_response_ms_ = 0;
    char receive_line_[192] = {};
    size_t receive_length_ = 0;
    JcyScpiMeasurements measurements_;
    JcyScpiIdentity identity_;
    JcyScpiRampResult ramp_result_;
    JcyScpiEvent events_[EVENT_QUEUE_CAPACITY] = {};
    size_t event_count_ = 0;
    uint32_t raw_rx_bytes_ = 0;
    uint32_t complete_lines_ = 0;
    uint32_t accepted_lines_ = 0;
    uint32_t parse_failures_ = 0;
    uint32_t unsolicited_lines_ = 0;
    uint32_t timeout_count_ = 0;
    uint32_t line_overflows_ = 0;
    uint32_t last_byte_ms_ = 0;
    uint32_t last_line_ms_ = 0;
    uint32_t last_accepted_ms_ = 0;
    char last_received_line_[192] = {};

    bool enqueue(JcyScpiCommandKind kind, int requested_value,
                 const char *format, ...);
    void start_next_command(uint32_t now_ms);
    void consume_receive_data(uint32_t now_ms);
    void process_line(char *line, uint32_t now_ms);
    bool parse_measurements(char *payload, uint32_t now_ms);
    bool parse_identity(char *payload, uint32_t now_ms);
    bool parse_ramp_result(char *payload, uint32_t now_ms);
    void complete_active(bool success, int error_code, uint32_t now_ms);
    void push_event(const JcyScpiEvent &event);
    bool queued(JcyScpiCommandKind kind) const;
};
