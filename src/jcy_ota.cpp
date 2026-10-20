#include "jcy_ota.h"

#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>

namespace {
constexpr uint16_t OTA_HTTP_PORT = 80;
constexpr uint32_t REBOOT_DELAY_MS = 1200;

WebServer ota_server(OTA_HTTP_PORT);
JcyOtaStatus ota_status_cache;
bool ota_server_started = false;
bool ota_upload_accepted = false;
uint32_t ota_reboot_at_ms = 0;
char ota_device_name[32] = "JCY2001D";
char ota_firmware_version[20] = "unknown";

bool request_authenticated() {
    if (ota_server.authenticate(jcy_ota::username(), jcy_ota::password())) {
        return true;
    }
    ota_server.requestAuthentication();
    return false;
}

void set_state(JcyOtaState state, const char *message) {
    ota_status_cache.state = state;
    snprintf(ota_status_cache.message, sizeof(ota_status_cache.message),
             "%s", message == nullptr ? "" : message);
}

void handle_root() {
    if (!request_authenticated()) return;

    char html[1400] = {};
    snprintf(
        html, sizeof(html),
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>%s OTA</title><style>"
        "body{font-family:sans-serif;max-width:620px;margin:40px auto;padding:20px;"
        "background:#102d54;color:#fff}section{background:#1f5c9d;padding:24px;"
        "border-radius:14px}button{padding:12px 28px;background:#2fd0b4;color:#fff;"
        "border:0;border-radius:8px;font-size:16px}input{margin:18px 0;width:100%%}"
        "</style></head><body><section><h2>%s Wi-Fi OTA</h2>"
        "<p>当前固件：%s</p><p>升级前必须关闭仪器输出。</p>"
        "<form method='POST' action='/update' enctype='multipart/form-data'>"
        "<input type='file' name='firmware' accept='.bin' required>"
        "<button type='submit'>上传并升级</button></form></section></body></html>",
        ota_device_name, ota_device_name, ota_firmware_version);
    ota_server.send(200, "text/html; charset=utf-8", html);
}

void handle_status() {
    if (!request_authenticated()) return;
    char json[256] = {};
    snprintf(json, sizeof(json),
             "{\"state\":%u,\"ip\":\"%s\",\"allowed\":%s,"
             "\"received\":%lu,\"message\":\"%s\"}",
             static_cast<unsigned>(ota_status_cache.state),
             ota_status_cache.ip,
             ota_status_cache.update_allowed ? "true" : "false",
             static_cast<unsigned long>(ota_status_cache.received_bytes),
             ota_status_cache.message);
    ota_server.send(200, "application/json", json);
}

void handle_update_finished() {
    if (!request_authenticated()) return;
    if (!ota_upload_accepted) {
        ota_server.send(409, "text/plain; charset=utf-8",
                        "升级被拒绝：请先关闭仪器输出");
        return;
    }
    const bool success = !Update.hasError();
    ota_server.sendHeader("Connection", "close");
    ota_server.send(success ? 200 : 500,
                    "text/plain; charset=utf-8",
                    success ? "升级成功，设备即将重启"
                            : "升级失败，请查看串口日志");
    if (success) {
        set_state(JcyOtaState::Success, "升级成功，准备重启");
        ota_reboot_at_ms = millis() + REBOOT_DELAY_MS;
    } else {
        char message[64] = {};
        snprintf(message, sizeof(message), "升级失败，错误码 %u",
                 static_cast<unsigned>(Update.getError()));
        set_state(JcyOtaState::Error, message);
    }
}

void handle_update_upload() {
    if (!ota_server.authenticate(jcy_ota::username(),
                                 jcy_ota::password())) {
        return;
    }
    HTTPUpload &upload = ota_server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        ota_status_cache.received_bytes = 0;
        ota_upload_accepted = ota_status_cache.update_allowed;
        if (!ota_upload_accepted) {
            set_state(JcyOtaState::Error, "请先关闭输出再升级");
            return;
        }
        Serial.printf("OTA upload begin: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
            ota_upload_accepted = false;
            char message[64] = {};
            snprintf(message, sizeof(message), "OTA begin 错误 %u",
                     static_cast<unsigned>(Update.getError()));
            set_state(JcyOtaState::Error, message);
            return;
        }
        set_state(JcyOtaState::Uploading, "正在接收固件");
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (!ota_upload_accepted) return;
        const size_t written = Update.write(upload.buf, upload.currentSize);
        ota_status_cache.received_bytes += written;
        if (written != upload.currentSize) {
            ota_upload_accepted = false;
            char message[64] = {};
            snprintf(message, sizeof(message), "OTA 写入错误 %u",
                     static_cast<unsigned>(Update.getError()));
            set_state(JcyOtaState::Error, message);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (!ota_upload_accepted) return;
        if (!Update.end(true)) {
            ota_upload_accepted = false;
            char message[64] = {};
            snprintf(message, sizeof(message), "OTA 完成错误 %u",
                     static_cast<unsigned>(Update.getError()));
            set_state(JcyOtaState::Error, message);
            return;
        }
        Serial.printf("OTA upload complete: %lu bytes\n",
                      static_cast<unsigned long>(upload.totalSize));
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        ota_upload_accepted = false;
        set_state(JcyOtaState::Error, "OTA 上传已中止");
    }
}

void start_server() {
    if (ota_server_started) return;
    ota_server.on("/", HTTP_GET, handle_root);
    ota_server.on("/status", HTTP_GET, handle_status);
    ota_server.on("/update", HTTP_POST,
                  handle_update_finished, handle_update_upload);
    ota_server.begin();
    ota_server_started = true;
    Serial.printf("OTA web ready: http://%s/ user=%s\n",
                  WiFi.localIP().toString().c_str(), jcy_ota::username());
}
}  // namespace

namespace jcy_ota {

void begin(const char *device_name, const char *firmware_version) {
    if (device_name != nullptr && device_name[0] != '\0') {
        snprintf(ota_device_name, sizeof(ota_device_name), "%s", device_name);
    }
    if (firmware_version != nullptr && firmware_version[0] != '\0') {
        snprintf(ota_firmware_version, sizeof(ota_firmware_version),
                 "%s", firmware_version);
    }
    ota_status_cache = {};
    set_state(JcyOtaState::WaitingForWifi, "等待 Wi-Fi");
}

void loop(bool update_allowed) {
    ota_status_cache.update_allowed = update_allowed;
    if (WiFi.status() != WL_CONNECTED) {
        ota_status_cache.ip[0] = '\0';
        if (ota_status_cache.state != JcyOtaState::Uploading &&
            ota_status_cache.state != JcyOtaState::Success) {
            set_state(JcyOtaState::WaitingForWifi, "等待 Wi-Fi");
        }
        return;
    }

    // 此函数在主循环约每 5ms 被调用；不能通过 toString() 反复分配临时
    // String，否则 Wi-Fi 开启后会快速碎片化内部 RAM，连带 RGB 刷新抖动。
    const IPAddress ip = WiFi.localIP();
    snprintf(ota_status_cache.ip, sizeof(ota_status_cache.ip),
             "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    start_server();
    ota_server.handleClient();

    if (ota_status_cache.state == JcyOtaState::WaitingForWifi ||
        ota_status_cache.state == JcyOtaState::Ready) {
        set_state(JcyOtaState::Ready,
                  update_allowed ? "OTA 已就绪" : "请先关闭仪器输出");
    }

    if (ota_reboot_at_ms != 0 &&
        static_cast<int32_t>(millis() - ota_reboot_at_ms) >= 0) {
        Serial.println("OTA reboot");
        delay(100);
        ESP.restart();
    }
}

const JcyOtaStatus &status() {
    return ota_status_cache;
}

}  // namespace jcy_ota
