#include "jcy_lower_mcu_ota.h"

#include "jcy_scpi_link.h"
#include "assets/tls/jcy_tls_bundle.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_heap_caps.h>
#include <mbedtls/sha256.h>

#include <cctype>
#include <cstring>

namespace {
constexpr uint8_t SOF0 = 0x4A;
constexpr uint8_t SOF1 = 0x43;
constexpr size_t MAX_PAYLOAD = 256;
constexpr size_t MAX_FRAME = 11 + MAX_PAYLOAD + 4;
constexpr uint32_t APPLICATION_CAPACITY = 238U * 1024U;
constexpr uint8_t CMD_INFO = 0x01;
constexpr uint8_t CMD_BEGIN = 0x02;
constexpr uint8_t CMD_DATA = 0x03;
constexpr uint8_t CMD_END = 0x04;
constexpr uint8_t CMD_ABORT = 0x05;
constexpr uint8_t CMD_RUN = 0x06;
constexpr uint8_t RESPONSE_ACK = 0x80;
constexpr uint32_t RESPONSE_TIMEOUT_MS = 1200;
constexpr uint32_t BOOTLOADER_HANDSHAKE_MS = 5000;
constexpr uint32_t BOOTLOADER_COMMAND_SETTLE_MS = 120;
constexpr uint8_t FRAME_RETRIES = 4;
constexpr size_t DOWNLOAD_BUFFER_SIZE = 2048;

enum class WorkerAction : uint8_t { None, Check, Upgrade };

struct IapFrame {
    uint8_t command = 0;
    uint16_t sequence = 0;
    uint32_t offset = 0;
    uint16_t length = 0;
    uint8_t payload[MAX_PAYLOAD] = {};
};

JcyScpiLink *scpi_link = nullptr;
JcyLowerMcuOtaStatus ota_status;
volatile bool worker_running = false;
volatile WorkerAction requested_action = WorkerAction::None;
char firmware_url[256] = {};
char expected_sha256[65] = {};
uint32_t expected_crc32 = 0;
uint8_t *downloaded_image = nullptr;

void release_image();

void set_message(JcyLowerMcuOtaState state, const char *message) {
    ota_status.state = state;
    snprintf(ota_status.message, sizeof(ota_status.message), "%s",
             message == nullptr ? "" : message);
}

String json_string(const String &json, const char *key) {
    String needle = String('"') + key + '"';
    int start = json.indexOf(needle);
    if (start < 0) return {};
    start = json.indexOf(':', start + needle.length());
    if (start < 0) return {};
    start = json.indexOf('"', start + 1);
    if (start < 0) return {};
    int end = start + 1;
    while (end < static_cast<int>(json.length())) {
        if (json[end] == '"' && json[end - 1] != '\\') break;
        ++end;
    }
    return end < static_cast<int>(json.length())
               ? json.substring(start + 1, end) : String();
}

uint32_t json_uint(const String &json, const char *key) {
    String needle = String('"') + key + '"';
    int start = json.indexOf(needle);
    if (start < 0) return 0;
    start = json.indexOf(':', start + needle.length());
    if (start < 0) return 0;
    while (++start < static_cast<int>(json.length()) &&
           isspace(static_cast<unsigned char>(json[start]))) {}
    return static_cast<uint32_t>(strtoul(json.c_str() + start, nullptr, 10));
}

bool parse_hex_u32(const String &text, uint32_t &value) {
    if (text.length() != 8) return false;
    char *end = nullptr;
    value = strtoul(text.c_str(), &end, 16);
    return end != nullptr && *end == '\0';
}

bool valid_sha256(const String &text) {
    if (text.length() != 64) return false;
    for (size_t i = 0; i < text.length(); ++i)
        if (!isxdigit(static_cast<unsigned char>(text[i]))) return false;
    return true;
}

bool parse_version_triplet(const char *text, int values[3]) {
    if (text == nullptr) return false;
    while (*text != '\0' && !isdigit(static_cast<unsigned char>(*text))) ++text;
    char *end = nullptr;
    for (int i = 0; i < 3; ++i) {
        if (!isdigit(static_cast<unsigned char>(*text))) return false;
        values[i] = static_cast<int>(strtol(text, &end, 10));
        if (end == text) return false;
        text = end;
        if (i < 2) {
            if (*text != '.') return false;
            ++text;
        }
    }
    return true;
}

int compare_versions(const char *a, const char *b) {
    int av[3] = {};
    int bv[3] = {};
    if (!parse_version_triplet(a, av) || !parse_version_triplet(b, bv))
        return strcmp(a == nullptr ? "" : a, b == nullptr ? "" : b);
    for (int i = 0; i < 3; ++i) {
        if (av[i] < bv[i]) return -1;
        if (av[i] > bv[i]) return 1;
    }
    return 0;
}

uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            const uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return crc;
}

uint32_t crc32(const uint8_t *data, size_t length) {
    return ~crc32_update(0xFFFFFFFFU, data, length);
}

void write_u16(uint8_t *data, uint16_t value) {
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
}

void write_u32(uint8_t *data, uint32_t value) {
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
    data[2] = static_cast<uint8_t>(value >> 16);
    data[3] = static_cast<uint8_t>(value >> 24);
}

uint16_t read_u16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}

uint32_t read_u32(const uint8_t *data) {
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

size_t encode_frame(const IapFrame &frame, uint8_t *buffer, size_t capacity) {
    const size_t total = 11U + frame.length + 4U;
    if (frame.length > MAX_PAYLOAD || capacity < total) return 0;
    buffer[0] = SOF0;
    buffer[1] = SOF1;
    buffer[2] = frame.command;
    write_u16(&buffer[3], frame.sequence);
    write_u32(&buffer[5], frame.offset);
    write_u16(&buffer[9], frame.length);
    if (frame.length > 0) memcpy(&buffer[11], frame.payload, frame.length);
    write_u32(&buffer[11 + frame.length], crc32(&buffer[2], 9 + frame.length));
    return total;
}

bool read_exact(uint8_t *data, size_t length, uint32_t timeout_ms) {
    const uint32_t deadline = millis() + timeout_ms;
    size_t received = 0;
    while (received < length && static_cast<int32_t>(millis() - deadline) < 0) {
        while (scpi_link->iap_available() > 0 && received < length) {
            const int value = scpi_link->iap_read();
            if (value >= 0) data[received++] = static_cast<uint8_t>(value);
        }
        delay(1);
    }
    return received == length;
}

bool read_response(IapFrame &frame, uint32_t timeout_ms) {
    const uint32_t deadline = millis() + timeout_ms;
    uint8_t byte = 0;
    bool sof0 = false;
    while (static_cast<int32_t>(millis() - deadline) < 0) {
        if (scpi_link->iap_available() <= 0) {
            delay(1);
            continue;
        }
        const int value = scpi_link->iap_read();
        if (value < 0) continue;
        byte = static_cast<uint8_t>(value);
        if (!sof0) {
            sof0 = byte == SOF0;
            continue;
        }
        if (byte == SOF1) break;
        sof0 = byte == SOF0;
    }
    if (!sof0 || byte != SOF1) return false;

    uint8_t header[9];
    if (!read_exact(header, sizeof(header), timeout_ms)) return false;
    frame.command = header[0];
    frame.sequence = read_u16(&header[1]);
    frame.offset = read_u32(&header[3]);
    frame.length = read_u16(&header[7]);
    if (frame.length > MAX_PAYLOAD) return false;
    if (!read_exact(frame.payload, frame.length, timeout_ms)) return false;
    uint8_t trailer[4];
    if (!read_exact(trailer, sizeof(trailer), timeout_ms)) return false;
    uint8_t crc_input[9 + MAX_PAYLOAD];
    memcpy(crc_input, header, sizeof(header));
    memcpy(crc_input + sizeof(header), frame.payload, frame.length);
    return read_u32(trailer) == crc32(crc_input, sizeof(header) + frame.length);
}

bool transact(const IapFrame &request, IapFrame &response,
              uint32_t timeout_ms = RESPONSE_TIMEOUT_MS) {
    uint8_t encoded[MAX_FRAME];
    const size_t length = encode_frame(request, encoded, sizeof(encoded));
    if (length == 0) return false;
    for (uint8_t attempt = 0; attempt < FRAME_RETRIES; ++attempt) {
        while (scpi_link->iap_available() > 0) scpi_link->iap_read();
        if (scpi_link->iap_write(encoded, length) != length) continue;
        scpi_link->iap_flush();
        if (!read_response(response, timeout_ms)) continue;
        if (response.sequence != request.sequence || response.length < 2 ||
            response.payload[0] != request.command) continue;
        if (response.command == RESPONSE_ACK && response.payload[1] == 0)
            return true;
        Serial.printf("Lower MCU IAP NACK cmd=%u status=%u seq=%u off=%lu\n",
                      request.command, response.payload[1], request.sequence,
                      static_cast<unsigned long>(request.offset));
        return false;
    }
    return false;
}

bool begin_https(HTTPClient &http, WiFiClientSecure &client, const char *url) {
    client.setCACert(JCY_OTA_CA_BUNDLE);
    http.setConnectTimeout(10000);
    http.setTimeout(15000);
    http.setUserAgent("JCY2001D/lower-mcu-ota");
    return http.begin(client, url);
}

bool load_manifest() {
    if (WiFi.status() != WL_CONNECTED) {
        set_message(JcyLowerMcuOtaState::Error, "Wi-Fi 未连接");
        return false;
    }
    set_message(JcyLowerMcuOtaState::Checking, "正在检查底层控制器版本");
    WiFiClientSecure client;
    HTTPClient http;
    if (!begin_https(http, client, jcy_lower_mcu_ota::manifest_url()) ||
        http.GET() != HTTP_CODE_OK) {
        http.end();
        set_message(JcyLowerMcuOtaState::Error, "底层升级清单下载失败");
        return false;
    }
    const String json = http.getString();
    http.end();
    const String product = json_string(json, "product");
    const String version = json_string(json, "version");
    const String url = json_string(json, "url");
    const String sha256 = json_string(json, "sha256");
    const String crc_text = json_string(json, "crc32");
    const String notes = json_string(json, "notes");
    const uint32_t size = json_uint(json, "size");
    uint32_t crc = 0;
    if (product != "JCY2001D-CONTROLLER" || version.isEmpty() ||
        url.isEmpty() || size == 0 || size > APPLICATION_CAPACITY ||
        !valid_sha256(sha256) || !parse_hex_u32(crc_text, crc)) {
        set_message(JcyLowerMcuOtaState::Error, "底层升级清单格式错误");
        return false;
    }
    snprintf(ota_status.latest_version, sizeof(ota_status.latest_version),
             "%s", version.c_str());
    snprintf(ota_status.notes, sizeof(ota_status.notes), "%s", notes.c_str());
    snprintf(firmware_url, sizeof(firmware_url), "%s", url.c_str());
    snprintf(expected_sha256, sizeof(expected_sha256), "%s", sha256.c_str());
    expected_crc32 = crc;
    ota_status.total_bytes = size;
    ota_status.transferred_bytes = 0;
    ota_status.percent = 0;
    if (ota_status.current_version[0] == '\0') {
        set_message(JcyLowerMcuOtaState::Error, "尚未读取到底层固件版本");
        return false;
    }
    const int version_compare = compare_versions(
        ota_status.latest_version, ota_status.current_version);
    if (version_compare < 0) {
        ota_status.has_update = false;
        set_message(JcyLowerMcuOtaState::UpToDate,
                    "云端版本低于当前，已拒绝降级");
        return true;
    }
    if (version_compare == 0 && !ota_status.same_version_test) {
        ota_status.has_update = false;
        set_message(JcyLowerMcuOtaState::UpToDate,
                    "当前已是最新版本");
        return true;
    }
    ota_status.has_update = true;
    set_message(JcyLowerMcuOtaState::UpdateAvailable,
                version_compare == 0
                    ? "测试模式：允许同版本重刷一次"
                    : "发现底层控制器新版本");
    return true;
}

bool download_image() {
    set_message(JcyLowerMcuOtaState::Downloading, "正在下载底层固件到 PSRAM");
    downloaded_image = static_cast<uint8_t *>(heap_caps_malloc(
        ota_status.total_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (downloaded_image == nullptr) {
        set_message(JcyLowerMcuOtaState::Error, "PSRAM 无法分配底层固件缓存");
        return false;
    }
    WiFiClientSecure client;
    HTTPClient http;
    if (!begin_https(http, client, firmware_url) ||
        http.GET() != HTTP_CODE_OK) {
        http.end();
        release_image();
        set_message(JcyLowerMcuOtaState::Error, "底层固件下载失败");
        return false;
    }
    WiFiClient *stream = http.getStreamPtr();
    uint32_t received = 0;
    uint32_t last_data_ms = millis();
    while (received < ota_status.total_bytes) {
        if (!ota_status.update_allowed) {
            http.end();
            release_image();
            set_message(JcyLowerMcuOtaState::Error, "输出状态变化，升级已停止");
            return false;
        }
        const size_t available = stream->available();
        if (available == 0) {
            if (millis() - last_data_ms > 15000U) {
                http.end();
                release_image();
                set_message(JcyLowerMcuOtaState::Error, "底层固件下载超时");
                return false;
            }
            delay(2);
            continue;
        }
        size_t chunk = available < DOWNLOAD_BUFFER_SIZE
                           ? available : DOWNLOAD_BUFFER_SIZE;
        const size_t remaining =
            static_cast<size_t>(ota_status.total_bytes - received);
        if (chunk > remaining) chunk = remaining;
        const size_t count = stream->readBytes(downloaded_image + received, chunk);
        if (count == 0) continue;
        received += static_cast<uint32_t>(count);
        last_data_ms = millis();
        ota_status.transferred_bytes = received;
        ota_status.percent = static_cast<uint8_t>(received * 100UL /
                                                  ota_status.total_bytes);
    }
    http.end();

    uint8_t digest[32];
    mbedtls_sha256(downloaded_image, ota_status.total_bytes, digest, 0);
    char actual_sha[65];
    for (size_t i = 0; i < sizeof(digest); ++i)
        snprintf(actual_sha + i * 2, 3, "%02x", digest[i]);
    if (strcasecmp(actual_sha, expected_sha256) != 0 ||
        crc32(downloaded_image, ota_status.total_bytes) != expected_crc32) {
        release_image();
        set_message(JcyLowerMcuOtaState::Error, "底层固件 SHA-256/CRC32 校验失败");
        return false;
    }
    ota_status.transferred_bytes = 0;
    ota_status.percent = 0;
    return true;
}

bool enter_bootloader() {
    set_message(JcyLowerMcuOtaState::EnteringBootloader, "正在让底层进入安全升级模式");
    if (!scpi_link->enter_iap_transport()) return false;
    const char command[] = "CH 1;SYSTem:FIRMware:BOOTloader\r\n";
    if (scpi_link->iap_write(reinterpret_cast<const uint8_t *>(command),
                             sizeof(command) - 1U) != sizeof(command) - 1U) {
        return false;
    }
    scpi_link->iap_flush();
    /* Give the application time to transmit its text ACK and reset, then
     * discard that line before switching to binary request/response frames. */
    delay(BOOTLOADER_COMMAND_SETTLE_MS);
    while (scpi_link->iap_available() > 0) scpi_link->iap_read();
    /* Probe INFO until reset and bootloader startup complete. INFO is
     * side-effect free and safe to retry. */
    const uint32_t deadline = millis() + BOOTLOADER_HANDSHAKE_MS;
    IapFrame request;
    IapFrame response;
    request.command = CMD_INFO;
    request.sequence = 1;
    while (static_cast<int32_t>(millis() - deadline) < 0) {
        delay(100);
        if (transact(request, response, 400)) return true;
    }
    return false;
}

bool transfer_image() {
    IapFrame request;
    IapFrame response;
    uint16_t sequence = 2;
    request.command = CMD_BEGIN;
    request.sequence = sequence++;
    request.length = 8;
    write_u32(&request.payload[0], ota_status.total_bytes);
    write_u32(&request.payload[4], expected_crc32);
    if (!transact(request, response, 5000)) return false;

    set_message(JcyLowerMcuOtaState::Transferring, "正在写入底层控制器");
    for (uint32_t offset = 0; offset < ota_status.total_bytes;) {
        const uint32_t remaining = ota_status.total_bytes - offset;
        const uint16_t chunk = static_cast<uint16_t>(
            remaining < MAX_PAYLOAD ? remaining : MAX_PAYLOAD);
        request = {};
        request.command = CMD_DATA;
        request.sequence = sequence++;
        request.offset = offset;
        request.length = chunk;
        memcpy(request.payload, downloaded_image + offset, chunk);
        if (!transact(request, response)) return false;
        offset += chunk;
        ota_status.transferred_bytes = offset;
        ota_status.percent = static_cast<uint8_t>(offset * 100UL /
                                                  ota_status.total_bytes);
        delay(1);
    }

    set_message(JcyLowerMcuOtaState::Verifying, "正在校验并提交底层固件");
    request = {};
    request.command = CMD_END;
    request.sequence = sequence++;
    request.offset = ota_status.total_bytes;
    if (!transact(request, response, 10000)) return false;
    request = {};
    request.command = CMD_RUN;
    request.sequence = sequence;
    if (!transact(request, response, 1500)) return false;
    return true;
}

void release_image() {
    if (downloaded_image != nullptr) {
        free(downloaded_image);
        downloaded_image = nullptr;
    }
}

void perform_upgrade() {
    if (!ota_status.update_allowed || !ota_status.has_update ||
        WiFi.status() != WL_CONNECTED) {
        set_message(JcyLowerMcuOtaState::Error, "升级条件不满足");
        return;
    }
    const int version_compare = compare_versions(
        ota_status.latest_version, ota_status.current_version);
    if (version_compare < 0 ||
        (version_compare == 0 && !ota_status.same_version_test)) {
        set_message(JcyLowerMcuOtaState::UpToDate,
                    version_compare < 0 ? "已拒绝降级" : "已拒绝同版本升级");
        return;
    }
    // 测试模式只授权一次写入尝试，不写入 NVS；无论本次成功与否，
    // 下一次都必须由操作员重新显式开启。
    ota_status.same_version_test = false;
    if (!download_image()) {
        release_image();
        return;
    }
    if (!enter_bootloader()) {
        scpi_link->exit_iap_transport();
        release_image();
        set_message(JcyLowerMcuOtaState::Error, "底层 Bootloader 握手失败");
        return;
    }
    if (!transfer_image()) {
        IapFrame abort;
        IapFrame response;
        abort.command = CMD_ABORT;
        abort.sequence = 0xFFFF;
        transact(abort, response, 300);
        scpi_link->exit_iap_transport();
        release_image();
        set_message(JcyLowerMcuOtaState::Error, "底层固件传输或校验失败");
        return;
    }
    scpi_link->exit_iap_transport();
    /* Keep periodic MEASure traffic stopped while the normal SCPI parser
     * reconnects. The Arduino main loop remains the sole owner of loop(); the
     * worker only observes the identity sequence to avoid cross-core races. */
    scpi_link->set_polling_suspended(true);
    release_image();
    ota_status.percent = 100;
    ota_status.transferred_bytes = ota_status.total_bytes;
    set_message(JcyLowerMcuOtaState::Reconnecting, "底层已重启，等待版本回读");
    delay(800);
    const uint32_t previous_identity_sequence = scpi_link->identity().sequence;
    scpi_link->request_identity();
    const uint32_t deadline = millis() + 3000U;
    while (static_cast<int32_t>(millis() - deadline) < 0) {
        const JcyScpiIdentity &identity = scpi_link->identity();
        if (identity.valid && identity.sequence != previous_identity_sequence) {
            snprintf(ota_status.current_version,
                     sizeof(ota_status.current_version), "%s",
                     identity.firmware);
            if (compare_versions(identity.firmware,
                                 ota_status.latest_version) == 0) {
                ota_status.has_update = false;
                set_message(JcyLowerMcuOtaState::Success,
                            "底层控制器升级完成，版本已确认");
                return;
            }
            set_message(JcyLowerMcuOtaState::Error,
                        "底层已重启，但版本回读与升级包不一致");
            return;
        }
        delay(5);
    }
    set_message(JcyLowerMcuOtaState::Error,
                "底层已写入，但重启后版本回读超时");
}

void worker(void *) {
    const WorkerAction action = requested_action;
    requested_action = WorkerAction::None;
    if (action == WorkerAction::Check) load_manifest();
    else if (action == WorkerAction::Upgrade) perform_upgrade();
    worker_running = false;
    vTaskDelete(nullptr);
}

bool start_worker(WorkerAction action) {
    if (worker_running || scpi_link == nullptr) return false;
    requested_action = action;
    worker_running = true;
    if (xTaskCreatePinnedToCore(worker, "lower_mcu_ota", 12288,
                                nullptr, 1, nullptr, 0) != pdPASS) {
        worker_running = false;
        requested_action = WorkerAction::None;
        set_message(JcyLowerMcuOtaState::Error, "无法创建底层升级任务");
        return false;
    }
    return true;
}
}  // namespace

namespace jcy_lower_mcu_ota {
void begin(JcyScpiLink *link) {
    scpi_link = link;
    ota_status = {};
    set_message(JcyLowerMcuOtaState::Idle, "等待检查底层控制器版本");
}

void loop(bool update_allowed, const char *current_version) {
    ota_status.update_allowed = update_allowed;
    if (!worker_running && current_version != nullptr && current_version[0])
        snprintf(ota_status.current_version, sizeof(ota_status.current_version),
                 "%s", current_version);
}

bool start_check() { return start_worker(WorkerAction::Check); }

bool start_upgrade() {
    if (!ota_status.has_update || !ota_status.update_allowed) return false;
    return start_worker(WorkerAction::Upgrade);
}

void set_same_version_test(bool enabled) {
    if (worker_running) return;
    ota_status.same_version_test = enabled;
    ota_status.has_update = false;
    set_message(JcyLowerMcuOtaState::Idle,
                enabled ? "测试模式已开启，请重新检查底层版本"
                        : "测试模式已关闭");
}

bool same_version_test_enabled() { return ota_status.same_version_test; }

const JcyLowerMcuOtaStatus &status() { return ota_status; }
}  // namespace jcy_lower_mcu_ota
