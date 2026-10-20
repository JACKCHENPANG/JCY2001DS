#pragma once

#include <Arduino.h>

enum class JcyOtaState : uint8_t {
    WaitingForWifi = 0,
    Ready,
    Uploading,
    Success,
    Error,
};

struct JcyOtaStatus {
    JcyOtaState state = JcyOtaState::WaitingForWifi;
    char ip[16] = {};
    char message[72] = "等待 Wi-Fi";
    size_t received_bytes = 0;
    bool update_allowed = false;
};

namespace jcy_ota {

void begin(const char *device_name, const char *firmware_version);
void loop(bool update_allowed);
const JcyOtaStatus &status();

constexpr const char *username() { return "jcy"; }
constexpr const char *password() { return "jcy1000"; }

}  // namespace jcy_ota
