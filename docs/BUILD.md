# 构建说明

- 面板：JC8048W550 / ESP32-S3，800x480 RGB，GT911 触控。
- 面板实际分辨率：1024×600；对应 PlatformIO 环境为 `SH051_1024X600_CACHE64_30M`（默认构建环境）。
- `JC8048W550_VENDOR8` 是 800×480 默认兼容环境，不适用于当前 1024×600 面板。
- 在该开发机上，`lib-idf55` 是指向已验证 A44 快照库的本地符号链接，不提交到 Git。
- A68 起，JCY 专用 RGB 双缓冲/bounce-buffer 封装位于
  `src/jcy_rgb_bus.*` 与 `src/jcy_rgb_display.*`，会提交到 Git；不再依赖
  被忽略的 Arduino_GFX RGB 驱动文件承载产品修改。
- 还需要厂商 Demo 库：`/Users/jcy/Downloads/JC8048W550/1-Demo/Demo_Arduino/Libraries`，其路径由 `platformio.ini` 的 `lib_extra_dirs` 引用。

## A68 RGB cache-safe 构建

`platformio.ini` 通过 `custom_sdkconfig` 重编译相关 ESP-IDF 组件，启用：

- LCD RGB ISR 驻留 IRAM；
- GDMA ISR/控制路径驻留 IRAM；
- Flash 写入期间保持 PSRAM XIP/缓存可访问；
- 两个内部 SRAM bounce buffer，PSRAM 中保留双帧缓冲。

首次构建会比普通增量构建更久，因为 pioarduino 需要生成匹配该
sdkconfig 的 Arduino/ESP-IDF 静态库。

构建：

```bash
pio run -e JC8048W550_VENDOR8
```

烧录必须先确认硬件端口和用户授权；本仓库不自动烧录。
