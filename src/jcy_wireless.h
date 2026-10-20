#pragma once
#include <Arduino.h>

enum class JcyLinkState : uint8_t {
    Off = 0,
    Idle = 1,
    Busy = 2,
    Connected = 3,
    Error = 4,
};

struct WifiStatus {
    JcyLinkState state;
    char ssid[33];
    char ip[16];
    int8_t rssi;
    int16_t scan_count;
    int16_t selected_index;
    bool scan_in_progress;
};

struct BleStatus {
    JcyLinkState state;
    char device_name[24];
    char peer_name[32];
    uint8_t connections;
    uint16_t bytes_sent;
    uint16_t bytes_received;
};

namespace jcy_wireless {

void begin();

bool wifi_is_enabled();
void wifi_enable();
void wifi_disable();
void wifi_reconnect();
void wifi_start_scan();
bool wifi_scan_in_progress();
int16_t wifi_scan_count();
int32_t wifi_scan_age_ms();
const char *wifi_ssid_for_index(int16_t index);
int8_t wifi_rssi_for_index(int16_t index);
bool wifi_connect_to_ssid(const char *ssid, const char *password);
bool wifi_save_credentials(const char *ssid, const char *password);
void wifi_forget_credentials();
void wifi_load_credentials(char *ssid, size_t ssid_size,
                           char *password, size_t password_size);
void wifi_loop();
const WifiStatus &wifi_status();

bool ble_is_enabled();
void ble_enable();
void ble_disable();
void ble_loop();
const BleStatus &ble_status();
void ble_send_line(const char *line);

}  // namespace jcy_wireless
