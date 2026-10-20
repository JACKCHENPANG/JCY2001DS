#include "jcy_wireless.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <Preferences.h>
#include <esp_wifi.h>

namespace {

constexpr char NVS_NAMESPACE[] = "jcywifi";
constexpr char BLE_DEVICE_NAME[] = "JCY2001D BLE";

constexpr uint32_t SCAN_TIMEOUT_MS = 15000;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;
constexpr uint32_t WIFI_RADIO_POLICY_RETRY_MS = 500;
constexpr uint32_t BLE_NOTIFY_MIN_INTERVAL_MS = 50;
constexpr uint8_t BLE_RX_BUFFER_SIZE = 96;
// 本机到已验证路由器 RSSI 约 -39dBm，默认 19.5dBm 没有必要。限制为
// 11dBm 能降低 Wi-Fi 发射瞬态电流与对 RGB 边缘的射频/电源耦合，同时
// 对正常室内连接保留充足链路余量。
constexpr wifi_power_t WIFI_TX_POWER_LIMIT = WIFI_POWER_11dBm;

BLECharacteristic *ble_tx_characteristic = nullptr;
BLECharacteristic *ble_rx_characteristic = nullptr;
bool ble_client_connected = false;
char ble_recent_message[BLE_RX_BUFFER_SIZE] = {};
size_t ble_recent_message_length = 0;
bool ble_recent_message_unread = false;
uint32_t ble_last_notify_ms = 0;
uint32_t ble_total_bytes_sent = 0;
uint32_t ble_total_bytes_received = 0;

Preferences wifi_prefs;
WifiStatus wifi_status_cache{};
BleStatus ble_status_cache{};
uint32_t wifi_state_changed_ms = 0;
uint32_t wifi_last_scan_started_ms = 0;
bool wifi_scan_pending = false;
char wifi_scan_ssid_cache[33] = {};
bool wifi_radio_policy_applied = false;
bool wifi_radio_policy_failure_logged = false;
uint32_t wifi_radio_policy_retry_at_ms = 0;

void clear_wifi_status() {
    wifi_status_cache = {};
    wifi_status_cache.state = JcyLinkState::Off;
    wifi_status_cache.selected_index = -1;
}

void clear_ble_status() {
    ble_status_cache = {};
    snprintf(ble_status_cache.device_name, sizeof(ble_status_cache.device_name),
             "%s", BLE_DEVICE_NAME);
    ble_status_cache.state = JcyLinkState::Off;
}

bool ble_is_active() {
    return ble_status_cache.state != JcyLinkState::Off;
}

bool wifi_is_active() {
    return wifi_status_cache.state != JcyLinkState::Off;
}

void wifi_apply_status() {
    switch (WiFi.status()) {
        case WL_CONNECTED: {
            // WiFi.SSID() 返回临时 String。只在连接状态切换时读取一次，
            // 平稳连接期间只更新 RSSI，避免每 300ms 制造内部堆碎片。
            const bool newly_connected =
                wifi_status_cache.state != JcyLinkState::Connected;
            wifi_status_cache.state = JcyLinkState::Connected;
            if (newly_connected) {
                const String ssid_str = WiFi.SSID();
                const IPAddress ip = WiFi.localIP();
                snprintf(wifi_status_cache.ssid,
                         sizeof(wifi_status_cache.ssid), "%s",
                         ssid_str.c_str());
                snprintf(wifi_status_cache.ip, sizeof(wifi_status_cache.ip),
                         "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
            }
            wifi_status_cache.rssi = static_cast<int8_t>(WiFi.RSSI());
            return;
        }
        case WL_CONNECT_FAILED:
        case WL_NO_SSID_AVAIL:
            if (wifi_status_cache.state == JcyLinkState::Busy) {
                wifi_status_cache.state = JcyLinkState::Error;
            }
            wifi_status_cache.ip[0] = '\0';
            wifi_status_cache.rssi = 0;
            return;
        case WL_CONNECTION_LOST:
        case WL_DISCONNECTED:
        default:
            if (wifi_status_cache.state == JcyLinkState::Connected) {
                wifi_status_cache.state = JcyLinkState::Idle;
                wifi_status_cache.ip[0] = '\0';
                wifi_status_cache.rssi = 0;
            }
            return;
    }
}

class BleServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *) override {
        ble_client_connected = true;
        ble_status_cache.connections = 1;
        ble_status_cache.state = JcyLinkState::Connected;
        snprintf(ble_status_cache.peer_name, sizeof(ble_status_cache.peer_name),
                 "BLE Client");
    }
    void onDisconnect(BLEServer *) override {
        ble_client_connected = false;
        ble_status_cache.connections = 0;
        ble_status_cache.state = JcyLinkState::Idle;
        ble_status_cache.peer_name[0] = '\0';
        BLEAdvertising *advertising = BLEDevice::getAdvertising();
        if (advertising != nullptr) {
            advertising->start();
        }
    }
};

class BleRxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override {
        if (characteristic == nullptr) {
            return;
        }
        const String value = characteristic->getValue();
        if (value.isEmpty()) {
            return;
        }
        ble_recent_message_length =
            min(value.length(), static_cast<size_t>(BLE_RX_BUFFER_SIZE - 1));
        memcpy(ble_recent_message, value.c_str(), ble_recent_message_length);
        ble_recent_message[ble_recent_message_length] = '\0';
        ble_recent_message_unread = true;
        ble_total_bytes_received += ble_recent_message_length;
    }
};

void reset_wifi_radio_policy() {
    wifi_radio_policy_applied = false;
    wifi_radio_policy_failure_logged = false;
    wifi_radio_policy_retry_at_ms = 0;
}

void try_apply_wifi_radio_policy() {
    if (wifi_radio_policy_applied) {
        return;
    }
    const uint32_t now = millis();
    if (static_cast<int32_t>(now - wifi_radio_policy_retry_at_ms) < 0) {
        return;
    }

    // WiFi.mode(WIFI_STA) 在当前 Arduino-ESP32 栈中还不会启动 STA；
    // setTxPower 此时必然失败。必须等 WiFi.begin()/scanNetworks() 真正
    // 启动无线驱动后再下发，并在异步启动阶段重试。
    const bool sleep_ok = WiFi.setSleep(false);
    const bool tx_power_ok = WiFi.setTxPower(WIFI_TX_POWER_LIMIT);
    const bool verified = tx_power_ok && sleep_ok &&
                          WiFi.getTxPower() == WIFI_TX_POWER_LIMIT &&
                          WiFi.getSleep() == WIFI_PS_NONE;
    if (verified) {
        wifi_radio_policy_applied = true;
        Serial.printf("WIFI radio policy applied tx_qdbm=%d sleep=OFF\n",
                      static_cast<int>(WiFi.getTxPower()));
        return;
    }

    if (!wifi_radio_policy_failure_logged) {
        Serial.printf("WIFI radio policy pending tx_ok=%d sleep_ok=%d\n",
                      tx_power_ok, sleep_ok);
        wifi_radio_policy_failure_logged = true;
    }
    wifi_radio_policy_retry_at_ms = now + WIFI_RADIO_POLICY_RETRY_MS;
}

void wifi_start_sta() {
    WiFi.mode(WIFI_STA);
    // 默认省电模式会按信标周期唤醒无线电，形成较大的周期性电流和DMA突发。
    // 对固定5V供电的仪器界面改为常开，换成更均匀的负载，减少RGB右缘抖动。
    if (!WiFi.setSleep(false)) {
        Serial.println("WIFI sleep disable failed");
    }
    WiFi.disconnect();
    char ssid[33] = {};
    char password[65] = {};
    jcy_wireless::wifi_load_credentials(ssid, sizeof(ssid), password,
                                       sizeof(password));
    reset_wifi_radio_policy();
    if (ssid[0] != '\0') {
        WiFi.begin(ssid, password[0] != '\0' ? password : nullptr);
        try_apply_wifi_radio_policy();
        wifi_status_cache.state = JcyLinkState::Busy;
    } else {
        wifi_status_cache.state = JcyLinkState::Idle;
    }
    wifi_state_changed_ms = millis();
}

void ble_start_server() {
    if (ble_is_active() && ble_tx_characteristic != nullptr) {
        return;
    }
    BLEDevice::init(BLE_DEVICE_NAME);
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new BleServerCallbacks());

    BLEService *service = server->createService(
        "6e400001-7700-46d0-8b3b-3a6b1f6a55c1");
    ble_tx_characteristic = service->createCharacteristic(
        "6e400003-7700-46d0-8b3b-3a6b1f6a55c1",
        BLECharacteristic::PROPERTY_NOTIFY);
    if (ble_tx_characteristic != nullptr) {
        ble_tx_characteristic->addDescriptor(new BLE2902());
    }
    ble_rx_characteristic = service->createCharacteristic(
        "6e400002-7700-46d0-8b3b-3a6b1f6a55c1",
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    if (ble_rx_characteristic != nullptr) {
        ble_rx_characteristic->setCallbacks(new BleRxCallbacks());
    }
    service->start();
    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    if (advertising != nullptr) {
        advertising->addServiceUUID(service->getUUID());
        advertising->setScanResponse(true);
        advertising->start();
    }
    ble_status_cache.state = JcyLinkState::Idle;
    ble_status_cache.connections = 0;
    ble_total_bytes_sent = 0;
    ble_total_bytes_received = 0;
}

void ble_stop_server() {
    if (!ble_is_active()) {
        return;
    }
    if (BLEDevice::getAdvertising() != nullptr) {
        BLEDevice::getAdvertising()->stop();
    }
    BLEDevice::deinit(true);
    ble_tx_characteristic = nullptr;
    ble_rx_characteristic = nullptr;
    ble_client_connected = false;
    clear_ble_status();
}

void notify_ble_recent_message() {
    if (ble_tx_characteristic == nullptr || !ble_client_connected) {
        return;
    }
    if (!ble_recent_message_unread) {
        return;
    }
    const uint32_t now = millis();
    if (ble_last_notify_ms != 0 && now - ble_last_notify_ms < BLE_NOTIFY_MIN_INTERVAL_MS) {
        return;
    }
    ble_tx_characteristic->setValue(
        reinterpret_cast<uint8_t *>(ble_recent_message),
        ble_recent_message_length);
    ble_tx_characteristic->notify();
    ble_recent_message_unread = false;
    ble_total_bytes_sent += ble_recent_message_length;
    ble_last_notify_ms = now;
}

}  // namespace

namespace jcy_wireless {

void begin() {
    clear_wifi_status();
    clear_ble_status();
}

bool wifi_is_enabled() {
    return wifi_is_active();
}

void wifi_enable() {
    if (wifi_is_active()) return;
    wifi_status_cache.state = JcyLinkState::Idle;
    wifi_start_sta();
    Serial.println("WIFI enable");
}

void wifi_disable() {
    if (!wifi_is_active()) return;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    reset_wifi_radio_policy();
    clear_wifi_status();
    wifi_status_cache.state = JcyLinkState::Off;
    Serial.println("WIFI disable");
}

void wifi_reconnect() {
    if (!wifi_is_active()) return;
    wifi_start_sta();
    Serial.println("WIFI reconnect");
}

void wifi_start_scan() {
    if (!wifi_is_active()) {
        WiFi.mode(WIFI_STA);
        wifi_status_cache.state = JcyLinkState::Idle;
    }
    // 先停止之前的连接/扫描，避免失败连接占用射频导致异步扫描一直为 -1。
    WiFi.disconnect(false, false);
    reset_wifi_radio_policy();
    esp_wifi_scan_stop();
    WiFi.scanDelete();
    wifi_status_cache.scan_count = -1;
    wifi_status_cache.ip[0] = '\0';
    wifi_status_cache.rssi = 0;
    const int16_t start_result = WiFi.scanNetworks(
        true, true, false, 300);
    try_apply_wifi_radio_policy();
    wifi_scan_pending = start_result == WIFI_SCAN_RUNNING;
    wifi_last_scan_started_ms = millis();
    if (start_result >= 0) {
        wifi_status_cache.scan_count = start_result;
        wifi_status_cache.state = JcyLinkState::Idle;
        Serial.printf("WIFI scan complete immediate count=%d\n",
                      start_result);
    } else if (start_result == WIFI_SCAN_RUNNING) {
        wifi_status_cache.state = JcyLinkState::Busy;
        Serial.println("WIFI scan begin");
    } else {
        wifi_status_cache.state = JcyLinkState::Error;
        wifi_status_cache.scan_count = 0;
        Serial.printf("WIFI scan start failed code=%d\n", start_result);
    }
}

bool wifi_scan_in_progress() {
    return wifi_scan_pending;
}

int16_t wifi_scan_count() {
    if (wifi_scan_pending) return WIFI_SCAN_RUNNING;
    const int16_t result = WiFi.scanComplete();
    return result >= 0 ? result : wifi_status_cache.scan_count;
}

int32_t wifi_scan_age_ms() {
    if (wifi_last_scan_started_ms == 0) return -1;
    return static_cast<int32_t>(millis() - wifi_last_scan_started_ms);
}

const char *wifi_ssid_for_index(int16_t index) {
    if (WiFi.scanComplete() <= 0) return "";
    if (index < 0 || index >= WiFi.scanComplete()) return "";
    const String ssid = WiFi.SSID(index);
    snprintf(wifi_scan_ssid_cache, sizeof(wifi_scan_ssid_cache),
             "%s", ssid.c_str());
    return wifi_scan_ssid_cache;
}

int8_t wifi_rssi_for_index(int16_t index) {
    if (WiFi.scanComplete() <= 0) return 0;
    if (index < 0 || index >= WiFi.scanComplete()) return 0;
    return static_cast<int8_t>(WiFi.RSSI(index));
}

bool wifi_connect_to_ssid(const char *ssid, const char *password) {
    if (ssid == nullptr || ssid[0] == '\0') return false;
    if (!wifi_is_active()) wifi_enable();
    WiFi.disconnect();
    reset_wifi_radio_policy();
    WiFi.begin(ssid, password);
    try_apply_wifi_radio_policy();
    wifi_status_cache.state = JcyLinkState::Busy;
    wifi_state_changed_ms = millis();
    Serial.printf("WIFI connect ssid=%s\n", ssid);
    return true;
}

bool wifi_save_credentials(const char *ssid, const char *password) {
    if (ssid == nullptr || ssid[0] == '\0') return false;
    if (!wifi_prefs.begin(NVS_NAMESPACE, false)) return false;
    wifi_prefs.putString("ssid", ssid);
    if (password != nullptr) wifi_prefs.putString("password", password);
    wifi_prefs.end();
    return true;
}

void wifi_forget_credentials() {
    if (wifi_prefs.begin(NVS_NAMESPACE, false)) {
        wifi_prefs.clear();
        wifi_prefs.end();
    }
}

void wifi_load_credentials(char *ssid, size_t ssid_size,
                           char *password, size_t password_size) {
    if (ssid == nullptr || ssid_size == 0) return;
    ssid[0] = '\0';
    if (password != nullptr && password_size > 0) password[0] = '\0';
    if (!wifi_prefs.begin(NVS_NAMESPACE, true)) return;
    const String ssid_value = wifi_prefs.getString("ssid", "");
    if (ssid_value.length() > 0) {
        snprintf(ssid, ssid_size, "%s", ssid_value.c_str());
    }
    if (password != nullptr && password_size > 0) {
        const String password_value = wifi_prefs.getString("password", "");
        if (password_value.length() > 0) {
            snprintf(password, password_size, "%s", password_value.c_str());
        }
    }
    wifi_prefs.end();
}

void wifi_loop() {
    if (!wifi_is_active()) return;
    try_apply_wifi_radio_policy();
    if (!wifi_scan_pending) {
        wifi_apply_status();
    }
    if (!wifi_scan_pending &&
        wifi_status_cache.state == JcyLinkState::Busy &&
        (millis() - wifi_state_changed_ms) > WIFI_CONNECT_TIMEOUT_MS) {
        wifi_status_cache.state = JcyLinkState::Error;
        Serial.println("WIFI connect_timeout");
    }
    if (wifi_scan_pending) {
        const int16_t result = WiFi.scanComplete();
        if (result >= 0) {
            wifi_scan_pending = false;
            wifi_status_cache.scan_count = result;
            wifi_status_cache.state = JcyLinkState::Idle;
            Serial.printf("WIFI scan complete count=%d\n", result);
        } else if ((millis() - wifi_last_scan_started_ms) > SCAN_TIMEOUT_MS) {
            esp_wifi_scan_stop();
            wifi_scan_pending = false;
            wifi_status_cache.scan_count = 0;
            wifi_status_cache.state = JcyLinkState::Error;
            Serial.println("WIFI scan timeout");
        }
    }
}

const WifiStatus &wifi_status() {
    return wifi_status_cache;
}

bool ble_is_enabled() {
    return ble_is_active();
}

void ble_enable() {
    if (ble_is_active()) return;
    ble_start_server();
    Serial.println("BLE enable");
}

void ble_disable() {
    if (!ble_is_active()) return;
    ble_stop_server();
    Serial.println("BLE disable");
}

void ble_loop() {
    notify_ble_recent_message();
    if (ble_client_connected) {
        BleStatus &status = ble_status_cache;
        status.bytes_sent = ble_total_bytes_sent > 65535
                                ? 65535
                                : static_cast<uint16_t>(ble_total_bytes_sent);
        status.bytes_received = ble_total_bytes_received > 65535
                                    ? 65535
                                    : static_cast<uint16_t>(ble_total_bytes_received);
    }
}

const BleStatus &ble_status() {
    return ble_status_cache;
}

void ble_send_line(const char *line) {
    if (ble_tx_characteristic == nullptr || line == nullptr) return;
    if (!ble_client_connected) return;
    const size_t length = strnlen(line, 96);
    String payload;
    payload.reserve(length + 2);
    payload.concat(line, length);
    payload += '\n';
    ble_tx_characteristic->setValue(payload);
    ble_tx_characteristic->notify();
    ble_total_bytes_sent += length;
}

}  // namespace jcy_wireless
