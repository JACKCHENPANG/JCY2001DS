#include "jcy_secondary_uart.h"

namespace {
constexpr int SECONDARY_UART_TX_PIN = 10;
constexpr int SECONDARY_UART_RX_PIN = 11;
// 板上 5.0 寸屏原理图把 IO10/IO11 标成 "TF(CS)/TF(MISO)"；本固件未启用
// SD/SPI，IO10/IO11 在屏幕正面只与底板的 TF 卡座直连。开启透传 UART 后需要
// 拔除 SD 卡槽上的卡，避免 SPI 总线空闲电平干扰串口波形。
}  // namespace

void JcySecondaryUart::begin(uint32_t baud) {
    if (started_) {
        return;
    }
    serial_.begin(baud, SERIAL_8N1,
                  SECONDARY_UART_RX_PIN, SECONDARY_UART_TX_PIN);
    started_ = true;
}

void JcySecondaryUart::end() {
    if (!started_) {
        return;
    }
    serial_.end();
    started_ = false;
}

bool JcySecondaryUart::active() const {
    return started_;
}

int JcySecondaryUart::available() {
    return started_ ? serial_.available() : 0;
}

int JcySecondaryUart::available_for_write() {
    return started_ ? serial_.availableForWrite() : 0;
}

int JcySecondaryUart::read() {
    return started_ ? serial_.read() : -1;
}

size_t JcySecondaryUart::write(const uint8_t *data, size_t length) {
    if (!started_ || data == nullptr || length == 0) {
        return 0;
    }
    return serial_.write(data, length);
}
