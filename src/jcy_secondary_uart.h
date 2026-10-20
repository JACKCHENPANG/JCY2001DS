#pragma once

#include <Arduino.h>

// 外部透传硬件 UART，IO10=TX、IO11=RX，速率固定 115200 8N1。
// 底层控制串口继续使用 IO13=TX、IO12=RX；进入透传模式后由固件在
// 两路 UART 之间原样转发数据。
class JcySecondaryUart {
public:
    void begin(uint32_t baud = 115200);
    void end();
    bool active() const;

    int available();
    int available_for_write();
    int read();
    size_t write(const uint8_t *data, size_t length);

private:
    // UART2 已被底层 SCPI 链路占用；外部透传使用剩余的 UART1。
    HardwareSerial serial_{1};
    bool started_ = false;
};
