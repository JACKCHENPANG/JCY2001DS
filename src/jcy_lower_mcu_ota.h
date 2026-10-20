#pragma once

#include <Arduino.h>

class JcyScpiLink;

enum class JcyLowerMcuOtaState : uint8_t {
    Idle = 0,
    Checking,
    UpToDate,
    UpdateAvailable,
    Downloading,
    EnteringBootloader,
    Transferring,
    Verifying,
    Reconnecting,
    Success,
    Error,
};

struct JcyLowerMcuOtaStatus {
    JcyLowerMcuOtaState state = JcyLowerMcuOtaState::Idle;
    char current_version[36] = {};
    char latest_version[36] = {};
    char notes[96] = {};
    char message[96] = {};
    uint32_t total_bytes = 0;
    uint32_t transferred_bytes = 0;
    uint8_t percent = 0;
    bool update_allowed = false;
    bool has_update = false;
    bool same_version_test = false;
};

namespace jcy_lower_mcu_ota {

void begin(JcyScpiLink *link);
void loop(bool update_allowed, const char *current_version);
bool start_check();
bool start_upgrade();
void set_same_version_test(bool enabled);
bool same_version_test_enabled();
const JcyLowerMcuOtaStatus &status();

constexpr const char *manifest_url() {
    return "https://www.jcytest.com/api/firmware/JCY2001D-CONTROLLER/manifest.json";
}

}  // namespace jcy_lower_mcu_ota
