#include "jcy_cloud_ota.h"

#include <HTTPClient.h>
#include <MD5Builder.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_ota_ops.h>

#include "assets/tls/jcy_tls_bundle.h"
#include "jcy_version.h"

namespace {
constexpr uint32_t HTTP_CONNECT_TIMEOUT_MS = 10000;
constexpr uint32_t HTTP_READ_TIMEOUT_MS = 15000;
constexpr uint8_t HTTP_MAX_ATTEMPTS = 3;
constexpr uint32_t HTTP_RETRY_DELAY_MS = 600;
constexpr uint32_t REBOOT_DELAY_MS = 1800;
// 大块 Flash 写入会在 PSRAM RGB 取帧期间形成更长的不可服务区。512B 分片
// 让每次页写的占用更短，两个写片之间保留一个 RTOS tick 给 LCD/Wi-Fi 任务。
constexpr size_t DOWNLOAD_BUFFER_SIZE = 512;
constexpr uint32_t DISPLAY_GUARD_SETTLE_MS = 1200;

enum class WorkerAction : uint8_t { None = 0, Check, Download };

JcyCloudOtaStatus cloud_status;
volatile bool worker_running = false;
volatile WorkerAction requested_action = WorkerAction::None;
uint32_t reboot_at_ms = 0;
char device_name[32] = "JCY2001D";
char firmware_url[256] = {};
char expected_md5[33] = {};

// 下载前和清单解析后均强制校验版本顺序，不能把旧服务器清单当作可升级版本。
bool manifest_version_is_strictly_newer() {
    return jcy_version::is_strictly_newer(cloud_status.latest_version,
                                          cloud_status.current_version);
}

void set_message(JcyCloudOtaState state, const char *message) {
    cloud_status.state = state;
    snprintf(cloud_status.message, sizeof(cloud_status.message), "%s",
             message == nullptr ? "" : message);
}

String json_string(const String &json, const char *key) {
    const String needle = String("\"") + key + "\"";
    int pos = json.indexOf(needle);
    if (pos < 0) return String();
    pos = json.indexOf(':', pos + needle.length());
    if (pos < 0) return String();
    pos = json.indexOf('"', pos + 1);
    if (pos < 0) return String();
    String out;
    bool escaped = false;
    for (int i = pos + 1; i < static_cast<int>(json.length()); ++i) {
        const char c = json[i];
        if (escaped) {
            if (c == 'n') out += '\n';
            else if (c == 'r') out += '\r';
            else if (c == 't') out += '\t';
            else out += c;
            escaped = false;
        } else if (c == '\\') {
            escaped = true;
        } else if (c == '"') {
            return out;
        } else {
            out += c;
        }
    }
    return String();
}

uint32_t json_uint(const String &json, const char *key) {
    const String needle = String("\"") + key + "\"";
    int pos = json.indexOf(needle);
    if (pos < 0) return 0;
    pos = json.indexOf(':', pos + needle.length());
    if (pos < 0) return 0;
    ++pos;
    while (pos < static_cast<int>(json.length()) &&
           (json[pos] == ' ' || json[pos] == '\t')) ++pos;
    return static_cast<uint32_t>(strtoul(json.c_str() + pos, nullptr, 10));
}

bool valid_md5(const char *value) {
    if (value == nullptr || strlen(value) != 32) return false;
    for (size_t i = 0; i < 32; ++i) {
        if (!isxdigit(static_cast<unsigned char>(value[i]))) return false;
    }
    return true;
}

bool begin_https(HTTPClient &http, WiFiClientSecure &client,
                 const char *url) {
    client.setCACert(JCY_OTA_CA_BUNDLE);
    http.setConnectTimeout(HTTP_CONNECT_TIMEOUT_MS);
    http.setTimeout(HTTP_READ_TIMEOUT_MS);
    http.setUserAgent(String(device_name) + "/" +
                      cloud_status.current_version);
    return http.begin(client, url);
}

int https_get_with_retry(HTTPClient &http, WiFiClientSecure &client,
                         const char *url, const char *operation,
                         JcyCloudOtaState retry_state,
                         const char *retry_message) {
    int code = -1;
    for (uint8_t attempt = 1; attempt <= HTTP_MAX_ATTEMPTS; ++attempt) {
        if (attempt > 1) {
            http.end();
            client.stop();
            char message[80] = {};
            snprintf(message, sizeof(message), "%s %u/%u",
                     retry_message,
                     static_cast<unsigned>(attempt),
                     static_cast<unsigned>(HTTP_MAX_ATTEMPTS));
            set_message(retry_state, message);
            vTaskDelay(pdMS_TO_TICKS(HTTP_RETRY_DELAY_MS));
        }

        if (begin_https(http, client, url)) {
            code = http.GET();
        } else {
            code = -1;
        }
        if (code == HTTP_CODE_OK) {
            if (attempt > 1) {
                Serial.printf("Cloud OTA %s recovered on attempt %u/%u\n",
                              operation,
                              static_cast<unsigned>(attempt),
                              static_cast<unsigned>(HTTP_MAX_ATTEMPTS));
            }
            return code;
        }
        Serial.printf("Cloud OTA %s attempt %u/%u failed HTTP %d\n",
                      operation,
                      static_cast<unsigned>(attempt),
                      static_cast<unsigned>(HTTP_MAX_ATTEMPTS), code);
    }
    return code;
}

void perform_manifest_check() {
    if (WiFi.status() != WL_CONNECTED) {
        set_message(JcyCloudOtaState::Error, "Wi-Fi 未连接");
        return;
    }
    set_message(JcyCloudOtaState::Checking, "正在连接升级服务器");
    Serial.printf(
        "Cloud OTA check memory heap=%lu max=%lu psram=%lu\n",
        static_cast<unsigned long>(ESP.getFreeHeap()),
        static_cast<unsigned long>(ESP.getMaxAllocHeap()),
        static_cast<unsigned long>(ESP.getFreePsram()));
    cloud_status.has_update = false;
    cloud_status.latest_version[0] = '\0';
    cloud_status.notes[0] = '\0';

    WiFiClientSecure client;
    HTTPClient http;
    const int code = https_get_with_retry(
        http, client, jcy_cloud_ota::manifest_url(), "manifest",
        JcyCloudOtaState::Checking, "连接失败，正在重试");
    if (code != HTTP_CODE_OK) {
        char message[80] = {};
        snprintf(message, sizeof(message), "检查失败 HTTP %d", code);
        set_message(JcyCloudOtaState::Error, message);
        http.end();
        return;
    }
    const String json = http.getString();
    http.end();

    const String product = json_string(json, "product");
    const String version = json_string(json, "version");
    const String url = json_string(json, "url");
    const String md5 = json_string(json, "md5");
    const String notes = json_string(json, "notes");
    const uint32_t size = json_uint(json, "size");
    if (product != device_name || version.length() == 0 ||
        url.length() == 0 || size == 0 || !valid_md5(md5.c_str())) {
        set_message(JcyCloudOtaState::Error, "升级清单格式错误");
        return;
    }

    jcy_version::Version current_version;
    jcy_version::Version remote_version;
    if (!jcy_version::parse(cloud_status.current_version, current_version)) {
        set_message(JcyCloudOtaState::Error, "当前固件版本格式错误");
        Serial.printf("Cloud OTA invalid current version: %s\n",
                      cloud_status.current_version);
        return;
    }
    if (!jcy_version::parse(version.c_str(), remote_version)) {
        set_message(JcyCloudOtaState::Error, "云端版本格式错误");
        Serial.printf("Cloud OTA invalid manifest version: %s\n",
                      version.c_str());
        return;
    }

    snprintf(cloud_status.latest_version,
             sizeof(cloud_status.latest_version), "%s", version.c_str());
    snprintf(cloud_status.notes, sizeof(cloud_status.notes), "%s",
             notes.c_str());
    snprintf(firmware_url, sizeof(firmware_url), "%s", url.c_str());
    snprintf(expected_md5, sizeof(expected_md5), "%s", md5.c_str());
    cloud_status.total_bytes = size;
    cloud_status.received_bytes = 0;
    cloud_status.percent = 0;

    const int version_order = jcy_version::compare(remote_version,
                                                   current_version);
    if (version_order == 0) {
        set_message(JcyCloudOtaState::UpToDate, "当前已是最新版本");
        return;
    }
    if (version_order < 0) {
        set_message(JcyCloudOtaState::UpToDate, "云端版本低于当前，已拒绝降级");
        Serial.printf("Cloud OTA downgrade rejected current=%s remote=%s\n",
                      cloud_status.current_version, version.c_str());
        return;
    }
    cloud_status.has_update = true;
    set_message(JcyCloudOtaState::UpdateAvailable, "发现新版本");
}

void perform_download() {
    if (!cloud_status.update_allowed) {
        set_message(JcyCloudOtaState::Error, "请先关闭仪器输出");
        return;
    }
    if (WiFi.status() != WL_CONNECTED || firmware_url[0] == '\0') {
        set_message(JcyCloudOtaState::Error, "Wi-Fi 或固件地址无效");
        return;
    }
    if (!manifest_version_is_strictly_newer()) {
        cloud_status.has_update = false;
        set_message(JcyCloudOtaState::UpToDate, "云端版本未高于当前，已拒绝升级");
        Serial.printf("Cloud OTA download rejected current=%s remote=%s\n",
                      cloud_status.current_version,
                      cloud_status.latest_version);
        return;
    }

    set_message(JcyCloudOtaState::Downloading, "正在准备安全升级界面");
    // UI线程在另一核上建立并提交静态OTA保护页；必须在第一次擦写前让它完整
    // 扫描至少数帧，不能由OTA工作线程直接调用LVGL。
    vTaskDelay(pdMS_TO_TICKS(DISPLAY_GUARD_SETTLE_MS));
    set_message(JcyCloudOtaState::Downloading, "正在下载固件");
    Serial.printf(
        "Cloud OTA download memory heap=%lu max=%lu psram=%lu\n",
        static_cast<unsigned long>(ESP.getFreeHeap()),
        static_cast<unsigned long>(ESP.getMaxAllocHeap()),
        static_cast<unsigned long>(ESP.getFreePsram()));
    cloud_status.received_bytes = 0;
    cloud_status.percent = 0;

    WiFiClientSecure client;
    HTTPClient http;
    const int code = https_get_with_retry(
        http, client, firmware_url, "download",
        JcyCloudOtaState::Downloading, "下载连接失败，正在重试");
    if (code != HTTP_CODE_OK) {
        char message[80] = {};
        snprintf(message, sizeof(message), "下载失败 HTTP %d", code);
        set_message(JcyCloudOtaState::Error, message);
        http.end();
        return;
    }

    const int content_length = http.getSize();
    const size_t expected_size = cloud_status.total_bytes > 0
                                     ? cloud_status.total_bytes
                                     : (content_length > 0
                                            ? static_cast<size_t>(content_length)
                                            : 0U);
    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(nullptr);
    if (expected_size == 0 || update_partition == nullptr ||
        expected_size > update_partition->size) {
        set_message(JcyCloudOtaState::Error, "OTA 分区或固件大小无效");
        http.end();
        return;
    }

    // Arduino Update erases 64 KiB blocks.  On this RGB-panel board a block
    // erase can hold the flash/cache critical section long enough to trip the
    // 300 ms interrupt watchdog.  IDF's sequential OTA mode erases one sector
    // incrementally while data is written, avoiding that long blocking erase.
    esp_ota_handle_t ota_handle = 0;
    esp_err_t ota_error = esp_ota_begin(
        update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (ota_error != ESP_OK) {
        char message[80] = {};
        snprintf(message, sizeof(message), "OTA 初始化失败 0x%X",
                 static_cast<unsigned>(ota_error));
        set_message(JcyCloudOtaState::Error, message);
        http.end();
        return;
    }

    MD5Builder downloaded_md5;
    downloaded_md5.begin();

    WiFiClient *stream = http.getStreamPtr();
    uint8_t *buffer = static_cast<uint8_t *>(malloc(DOWNLOAD_BUFFER_SIZE));
    if (buffer == nullptr) {
        esp_ota_abort(ota_handle);
        http.end();
        set_message(JcyCloudOtaState::Error, "下载缓存不足");
        return;
    }

    uint32_t last_data_ms = millis();
    bool failed = false;
    uint8_t last_logged_percent = 0;
    while (http.connected() &&
           (content_length < 0 ||
            cloud_status.received_bytes < static_cast<uint32_t>(content_length))) {
        if (!cloud_status.update_allowed) {
            set_message(JcyCloudOtaState::Error, "输出状态变化，升级已停止");
            failed = true;
            break;
        }
        const size_t available = stream->available();
        if (available == 0) {
            if (millis() - last_data_ms > HTTP_READ_TIMEOUT_MS) {
                set_message(JcyCloudOtaState::Error, "固件下载超时");
                failed = true;
                break;
            }
            delay(2);
            continue;
        }
        const size_t request = min(available, DOWNLOAD_BUFFER_SIZE);
        const size_t read = stream->readBytes(buffer, request);
        if (read == 0) continue;
        last_data_ms = millis();
        ota_error = esp_ota_write(ota_handle, buffer, read);
        if (ota_error != ESP_OK) {
            char message[80] = {};
            snprintf(message, sizeof(message), "固件写入失败 0x%X",
                     static_cast<unsigned>(ota_error));
            set_message(JcyCloudOtaState::Error, message);
            failed = true;
            break;
        }
        downloaded_md5.add(buffer, read);
        cloud_status.received_bytes += read;
        if (cloud_status.total_bytes > 0) {
            cloud_status.percent = static_cast<uint8_t>(min(
                100UL,
                cloud_status.received_bytes * 100UL /
                    cloud_status.total_bytes));
            if (cloud_status.percent >= last_logged_percent + 10U) {
                last_logged_percent =
                    static_cast<uint8_t>((cloud_status.percent / 10U) * 10U);
                Serial.printf("Cloud OTA progress %u%% %lu/%lu\n",
                              static_cast<unsigned>(cloud_status.percent),
                              static_cast<unsigned long>(cloud_status.received_bytes),
                              static_cast<unsigned long>(cloud_status.total_bytes));
            }
        }
        delay(1);
    }
    free(buffer);
    http.end();

    if (failed) {
        esp_ota_abort(ota_handle);
        return;
    }
    if (cloud_status.total_bytes > 0 &&
        cloud_status.received_bytes != cloud_status.total_bytes) {
        esp_ota_abort(ota_handle);
        set_message(JcyCloudOtaState::Error, "固件长度校验失败");
        return;
    }

    set_message(JcyCloudOtaState::Verifying, "正在校验固件");
    downloaded_md5.calculate();
    const String actual_md5 = downloaded_md5.toString();
    if (!actual_md5.equalsIgnoreCase(expected_md5)) {
        esp_ota_abort(ota_handle);
        set_message(JcyCloudOtaState::Error, "固件 MD5 校验失败");
        Serial.printf("Cloud OTA MD5 mismatch actual=%s expected=%s\n",
                      actual_md5.c_str(), expected_md5);
        return;
    }

    ota_error = esp_ota_end(ota_handle);
    if (ota_error != ESP_OK) {
        char message[80] = {};
        snprintf(message, sizeof(message), "固件校验失败 0x%X",
                 static_cast<unsigned>(ota_error));
        set_message(JcyCloudOtaState::Error, message);
        return;
    }
    ota_error = esp_ota_set_boot_partition(update_partition);
    if (ota_error != ESP_OK) {
        char message[80] = {};
        snprintf(message, sizeof(message), "启动分区设置失败 0x%X",
                 static_cast<unsigned>(ota_error));
        set_message(JcyCloudOtaState::Error, message);
        return;
    }

    cloud_status.percent = 100;
    set_message(JcyCloudOtaState::Success, "升级成功，准备重启");
    Serial.printf("Cloud OTA verified md5=%s bytes=%lu\n",
                  actual_md5.c_str(),
                  static_cast<unsigned long>(cloud_status.received_bytes));
    reboot_at_ms = millis() + REBOOT_DELAY_MS;
}

void worker_task(void *) {
    const WorkerAction action = requested_action;
    requested_action = WorkerAction::None;
    if (action == WorkerAction::Check) perform_manifest_check();
    else if (action == WorkerAction::Download) perform_download();
    worker_running = false;
    vTaskDelete(nullptr);
}

bool start_worker(WorkerAction action) {
    if (worker_running) return false;
    requested_action = action;
    worker_running = true;
    const BaseType_t result = xTaskCreatePinnedToCore(
        worker_task, "jcy_cloud_ota", 12288, nullptr, 1, nullptr, 0);
    if (result != pdPASS) {
        worker_running = false;
        requested_action = WorkerAction::None;
        set_message(JcyCloudOtaState::Error, "无法创建 OTA 任务");
        return false;
    }
    return true;
}
}  // namespace

namespace jcy_cloud_ota {

void begin(const char *name, const char *firmware_version) {
    if (name != nullptr && name[0] != '\0') {
        snprintf(device_name, sizeof(device_name), "%s", name);
    }
    cloud_status = {};
    snprintf(cloud_status.current_version,
             sizeof(cloud_status.current_version), "%s",
             firmware_version == nullptr ? "unknown" : firmware_version);
    set_message(JcyCloudOtaState::Idle, "等待检查云端版本");
}

void loop(bool update_allowed) {
    cloud_status.update_allowed = update_allowed;
    if (reboot_at_ms != 0 &&
        static_cast<int32_t>(millis() - reboot_at_ms) >= 0) {
        Serial.println("Cloud OTA reboot");
        delay(100);
        ESP.restart();
    }
}

bool start_check() {
    return start_worker(WorkerAction::Check);
}

bool start_download() {
    if (!cloud_status.has_update || !cloud_status.update_allowed) return false;
    return start_worker(WorkerAction::Download);
}

const JcyCloudOtaStatus &status() {
    return cloud_status;
}

}  // namespace jcy_cloud_ota
