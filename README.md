# JCY2001D Panel Firmware

JCY2001DS 面板固件：ESP32-S3，1024×600 RGB + GT911 触控。

项目同时保留 JC8048W550 800×480 兼容配置，但当前目标硬件是 1024×600 SH051 面板。

## 实板已验证基线

`releases/verified/JCY2001D-V0.8.4-A44-OTA-FIX-app.bin` 是 2026-08-09 已刷入实板并观察稳定的应用分区镜像：

- MD5：`8abb7e34b09e1165075e5e0c8a6003fb`
- SHA-256：`094ed8406aa45aae39a67442d0a7faf2f259ed4501d1e41aba8b0e6608e8578c`
- 写入地址：`0x10000`（仅 app 分区）

`src/` 初始来源为 A44 历史源快照。该快照与上述实板 `.bin` 的逐字节同源关系尚未完全证明；因此此镜像是**唯一的实板验证基线**，源码构建产物必须分别验证后才能替代它。

## 状态边界

- A44：实板稳定基线。
- A56：已确认会造成 RGB 整体错位，禁止烧录。
- 后续 OTA 版本从 A44 源快照最小改动派生，并且只允许高于当前版本的升级。

详见 [docs/BUILD.md](docs/BUILD.md)。

## V0.8.5-A57 OTA candidate

`releases/JCY2001D-V0.8.5-A57-OTA-PROGRESS-20260809-1224/` contains the compile-verified OTA image. It is derived from the A44 source snapshot without RGB timing/restart changes; it adds a persistent, visible OTA transfer page and strict no-downgrade version checking. It must still receive an OTA-on-panel confirmation before it is promoted to the **verified baseline** section above.
