#pragma once

#include <Arduino.h>

namespace hy200 {

struct Status {
    bool valid = false;
    bool running = false;
    bool fault = false;
    bool gear35 = false;
    bool handle_pressed = false;
    bool tracking_locked = false;
    bool calibrating = false;
    bool calibration_ok = false;
    bool calibration_ng = false;
    bool calibration_invalid = false;
    uint8_t raw_flags = 0;
    uint8_t lock_reason = 0;
    uint32_t frequency_hz = 0;
    uint16_t iout_adc = 0;
    uint32_t received_ms = 0;
    uint32_t sequence = 0;
};

struct Version {
    bool valid = false;
    uint8_t product_major = 0;
    uint8_t product_minor = 0;
    uint8_t product_patch = 0;
    uint8_t product_rc = 0;
    uint8_t host_major = 0;
    uint8_t host_minor = 0;
    uint8_t host_patch = 0;
    uint8_t protocol = 0;
    uint32_t received_ms = 0;
};

struct Ack {
    bool valid = false;
    uint8_t command_sequence = 0;
    uint8_t result = 0;
    uint32_t received_ms = 0;
    uint32_t generation = 0;
};

class Protocol {
public:
    explicit Protocol(uint8_t uart_number = 2);
    void begin(uint32_t baud, int rx_pin, int tx_pin);
    void loop(uint32_t now_ms);

    void set_duration_seconds(uint8_t seconds);
    void set_run(bool enabled);
    void request_reset();
    void request_calibration();

    bool online(uint32_t now_ms) const;
    const Status &status() const { return status_; }
    const Version &version() const { return version_; }
    const Ack &ack() const { return ack_; }
    uint8_t duration_seconds() const { return duration_seconds_; }
    bool run_requested() const { return run_requested_; }
    uint32_t reset_action_generation() const { return reset_action_generation_; }
    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t frame_errors() const { return frame_errors_; }

private:
    static constexpr uint8_t kMaximumPayload = 24;
    static constexpr uint32_t kCommandPeriodMs = 100;
    static constexpr uint32_t kLinkTimeoutMs = 300;

    HardwareSerial serial_;
    uint8_t rx_state_ = 0;
    uint8_t header_[4] = {};
    uint8_t payload_[kMaximumPayload] = {};
    uint8_t header_pos_ = 0;
    uint8_t payload_pos_ = 0;
    uint8_t crc_low_ = 0;
    uint8_t tx_sequence_ = 0;
    uint8_t duration_seconds_ = 5;
    uint8_t pending_sequence_ = 0;
    uint8_t pending_frame_[10] = {};
    uint8_t pending_frame_length_ = 0;
    uint8_t pending_retries_ = 0;
    uint32_t pending_sent_ms_ = 0;
    bool awaiting_ack_ = false;
    bool link_was_online_ = false;
    bool run_requested_ = false;
    uint8_t reset_pulses_ = 0;
    uint8_t calibration_pulses_ = 0;
    uint32_t reset_action_generation_ = 0;
    uint32_t last_command_ms_ = 0;
    uint32_t crc_errors_ = 0;
    uint32_t frame_errors_ = 0;
    Status status_;
    Version version_;
    Ack ack_;

    static uint16_t crc16_update(uint16_t crc, uint8_t value);
    void receive_byte(uint8_t value, uint32_t now_ms);
    void accept_frame(uint32_t now_ms);
    void send_command(uint32_t now_ms);
    uint8_t encode_frame(uint8_t type, uint8_t sequence,
                         const uint8_t *payload, uint8_t length,
                         uint8_t *output);
    void send_frame(uint8_t type, uint8_t sequence,
                    const uint8_t *payload, uint8_t length);
};

}  // namespace hy200
