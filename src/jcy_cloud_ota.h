#pragma once

#include <Arduino.h>

enum class JcyCloudOtaState : uint8_t {
    Idle = 0,
    Checking,
    UpToDate,
    UpdateAvailable,
    Downloading,
    Verifying,
    Success,
    Error,
};

struct JcyCloudOtaStatus {
    JcyCloudOtaState state = JcyCloudOtaState::Idle;
    char current_version[20] = {};
    char latest_version[20] = {};
    char notes[96] = {};
    char message[80] = {};
    uint32_t total_bytes = 0;
    uint32_t received_bytes = 0;
    uint8_t percent = 0;
    bool update_allowed = false;
    bool has_update = false;
};

namespace jcy_cloud_ota {

// Begin must be called once after WiFi stack is up.
// device_name is included in the manifest User-Agent for server-side logs.
void begin(const char *device_name, const char *firmware_version);

// loop() must be called every main loop iteration; it is non-blocking and
// advances the in-flight state machine one step at a time.
void loop(bool update_allowed);

// Manually trigger a manifest check. Returns true if the request was kicked
// off; false if a check or download is already in progress.
bool start_check();

// Begin downloading the latest version returned by the last manifest fetch.
// The download is non-blocking; poll loop() until state becomes Success or
// Error. Returns true on accept, false otherwise.
bool start_download();

const JcyCloudOtaStatus &status();

constexpr const char *manifest_url() {
    return "https://www.jcytest.com/api/firmware/JCY2001D/manifest.json";
}

}  // namespace jcy_cloud_ota
