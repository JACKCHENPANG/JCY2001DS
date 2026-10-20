#include "hy200_protocol.h"

#include <algorithm>

namespace hy200 {
namespace {
constexpr uint8_t kSof0 = 0xA5;
constexpr uint8_t kSof1 = 0x5A;
constexpr uint8_t kProtocolVersion = 0x01;
constexpr uint8_t kTypeCommand = 0x10;
constexpr uint8_t kTypeStatus = 0x20;
constexpr uint8_t kTypeVersion = 0x21;
constexpr uint8_t kTypeAck = 0x30;
constexpr uint8_t kCommandRun = 0x01;
constexpr uint8_t kCommandReset = 0x02;
constexpr uint8_t kCommandCalibrate = 0x04;

uint32_t read_u32_le(const uint8_t *value) {
    return static_cast<uint32_t>(value[0]) |
           (static_cast<uint32_t>(value[1]) << 8) |
           (static_cast<uint32_t>(value[2]) << 16) |
           (static_cast<uint32_t>(value[3]) << 24);
}
}  // namespace

Protocol::Protocol(uint8_t uart_number) : serial_(uart_number) {}

void Protocol::begin(uint32_t baud, int rx_pin, int tx_pin) {
    pinMode(tx_pin, OUTPUT);
    digitalWrite(tx_pin, HIGH);
    delay(2);
    serial_.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
    last_command_ms_ = millis() - kCommandPeriodMs;
}

void Protocol::set_duration_seconds(uint8_t seconds) {
    duration_seconds_ = std::max<uint8_t>(1, std::min<uint8_t>(99, seconds));
}

void Protocol::set_run(bool enabled) {
    run_requested_ = enabled;
}

void Protocol::request_reset() {
    run_requested_ = false;
    reset_pulses_ = 3;
    ++reset_action_generation_;
}

void Protocol::request_calibration() {
    run_requested_ = false;
    calibration_pulses_ = 3;
}

bool Protocol::online(uint32_t now_ms) const {
    return status_.valid &&
           static_cast<uint32_t>(now_ms - status_.received_ms) <= kLinkTimeoutMs;
}

uint16_t Protocol::crc16_update(uint16_t crc, uint8_t value) {
    crc ^= static_cast<uint16_t>(value) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x8000U) != 0U
                  ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                  : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

uint8_t Protocol::encode_frame(uint8_t type, uint8_t sequence,
                               const uint8_t *payload, uint8_t length,
                               uint8_t *frame) {
    frame[0] = kSof0;
    frame[1] = kSof1;
    frame[2] = kProtocolVersion;
    frame[3] = type;
    frame[4] = sequence;
    frame[5] = length;
    uint16_t crc = 0xFFFFU;
    for (uint8_t index = 2; index < 6; ++index) crc = crc16_update(crc, frame[index]);
    for (uint8_t index = 0; index < length; ++index) {
        frame[6 + index] = payload[index];
        crc = crc16_update(crc, payload[index]);
    }
    frame[6 + length] = static_cast<uint8_t>(crc);
    frame[7 + length] = static_cast<uint8_t>(crc >> 8);
    return static_cast<uint8_t>(8 + length);
}

void Protocol::send_frame(uint8_t type, uint8_t sequence,
                          const uint8_t *payload, uint8_t length) {
    uint8_t frame[8 + kMaximumPayload] = {};
    const uint8_t frame_length = encode_frame(type, sequence, payload, length, frame);
    serial_.write(frame, frame_length);
}

void Protocol::send_command(uint32_t now_ms) {
    uint8_t payload[2] = {0, duration_seconds_};
    if (run_requested_) payload[0] |= kCommandRun;
    if (reset_pulses_ != 0) payload[0] |= kCommandReset;
    if (calibration_pulses_ != 0) payload[0] |= kCommandCalibrate;
    pending_sequence_ = tx_sequence_++;
    pending_frame_length_ = encode_frame(kTypeCommand, pending_sequence_, payload,
                                         sizeof(payload), pending_frame_);
    serial_.write(pending_frame_, pending_frame_length_);
    pending_retries_ = 0;
    pending_sent_ms_ = now_ms;
    awaiting_ack_ = true;
    if (reset_pulses_ != 0) --reset_pulses_;
    if (calibration_pulses_ != 0) --calibration_pulses_;
    last_command_ms_ = now_ms;
}

void Protocol::accept_frame(uint32_t now_ms) {
    if (header_[0] != kProtocolVersion) {
        ++frame_errors_;
        return;
    }
    const uint8_t type = header_[1];
    const uint8_t sequence = header_[2];
    const uint8_t length = header_[3];
    if (type == kTypeStatus && length == 8) {
        const uint8_t flags = payload_[0];
        status_.valid = true;
        status_.raw_flags = flags;
        status_.running = (flags & 0x01U) != 0U;
        status_.fault = (flags & 0x02U) != 0U;
        status_.gear35 = (flags & 0x04U) != 0U;
        status_.handle_pressed = (flags & 0x08U) != 0U;
        status_.tracking_locked = (flags & 0x10U) != 0U;
        status_.calibrating = (flags & 0x20U) != 0U;
        status_.calibration_invalid = (flags & 0xC0U) == 0xC0U;
        status_.calibration_ok = !status_.calibration_invalid && (flags & 0x40U) != 0U;
        status_.calibration_ng = !status_.calibration_invalid && (flags & 0x80U) != 0U;
        status_.lock_reason = payload_[1];
        status_.frequency_hz = read_u32_le(&payload_[2]);
        status_.iout_adc = static_cast<uint16_t>(payload_[6]) |
                           (static_cast<uint16_t>(payload_[7]) << 8);
        status_.received_ms = now_ms;
        ++status_.sequence;
        return;
    }
    if (type == kTypeVersion && length == 8) {
        version_.valid = true;
        version_.product_major = payload_[0];
        version_.product_minor = payload_[1];
        version_.product_patch = payload_[2];
        version_.product_rc = payload_[3];
        version_.host_major = payload_[4];
        version_.host_minor = payload_[5];
        version_.host_patch = payload_[6];
        version_.protocol = payload_[7];
        version_.received_ms = now_ms;
        return;
    }
    if (type == kTypeAck && length == 3 && payload_[0] == kTypeCommand) {
        // Production ACK uses the command sequence both outside and inside.
        // Validate fully before publishing it to application state.
        if (sequence != payload_[1]) {
            ++frame_errors_;
            return;
        }
        ack_.valid = true;
        ack_.command_sequence = payload_[1];
        ack_.result = payload_[2];
        ack_.received_ms = now_ms;
        ++ack_.generation;
        if (awaiting_ack_ && payload_[1] == pending_sequence_) awaiting_ack_ = false;
        return;
    }
    ++frame_errors_;
}

void Protocol::receive_byte(uint8_t value, uint32_t now_ms) {
    switch (rx_state_) {
    case 0:
        rx_state_ = value == kSof0 ? 1 : 0;
        break;
    case 1:
        if (value == kSof1) {
            rx_state_ = 2;
            header_pos_ = 0;
        } else {
            rx_state_ = value == kSof0 ? 1 : 0;
        }
        break;
    case 2:
        header_[header_pos_++] = value;
        if (header_pos_ == 4) {
            if (header_[3] > kMaximumPayload) {
                ++frame_errors_;
                rx_state_ = value == kSof0 ? 1 : 0;
            } else {
                payload_pos_ = 0;
                rx_state_ = header_[3] == 0 ? 4 : 3;
            }
        }
        break;
    case 3:
        payload_[payload_pos_++] = value;
        if (payload_pos_ >= header_[3]) rx_state_ = 4;
        break;
    case 4:
        crc_low_ = value;
        rx_state_ = 5;
        break;
    default: {
        uint16_t crc = 0xFFFFU;
        for (uint8_t index = 0; index < 4; ++index) {
            crc = crc16_update(crc, header_[index]);
        }
        for (uint8_t index = 0; index < header_[3]; ++index) {
            crc = crc16_update(crc, payload_[index]);
        }
        if (crc_low_ == static_cast<uint8_t>(crc) &&
            value == static_cast<uint8_t>(crc >> 8)) {
            accept_frame(now_ms);
        } else {
            ++crc_errors_;
        }
        rx_state_ = value == kSof0 ? 1 : 0;
        break;
    }
    }
}

void Protocol::loop(uint32_t now_ms) {
    while (serial_.available() > 0) {
        const int value = serial_.read();
        if (value >= 0) receive_byte(static_cast<uint8_t>(value), now_ms);
    }
    const bool link_online = online(now_ms);
    if (link_was_online_ && !link_online) {
        run_requested_ = false;
        reset_pulses_ = 0;
        calibration_pulses_ = 0;
        awaiting_ack_ = false;
    }
    link_was_online_ = link_online;
    // Retry a lost command transport ACK with the identical sequence/frame.
    // These retries do not reapply one-shot commands on the production host.
    if (awaiting_ack_ && static_cast<uint32_t>(now_ms - pending_sent_ms_) >= 40U) {
        if (pending_retries_ < 2U) {
            serial_.write(pending_frame_, pending_frame_length_);
            ++pending_retries_;
            pending_sent_ms_ = now_ms;
            last_command_ms_ = now_ms;
            return;
        }
        awaiting_ack_ = false;
    }
    if (!awaiting_ack_ &&
        static_cast<uint32_t>(now_ms - last_command_ms_) >= kCommandPeriodMs) {
        send_command(now_ms);
    }
}

}  // namespace hy200
