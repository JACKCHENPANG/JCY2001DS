#!/usr/bin/env python3
"""Capture the JCY1000-5 RGB-panel framebuffer over its USB serial port."""

from __future__ import annotations

import argparse
import glob
import re
import time
import zlib
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
from PIL import Image

HEADER_RE = re.compile(
    rb"^JCYSHOT1 (\d+) (\d+) RGB565BE PACKBITS (\d+) ([0-9A-Fa-f]{8})$"
)
FOOTER_RE = re.compile(rb"^JCYSHOT1 END ([0-9A-Fa-f]{8})$")


def find_serial_port() -> str:
    candidates = sorted(glob.glob("/dev/cu.usbserial-*") +
                        glob.glob("/dev/cu.wchusbserial*") +
                        glob.glob("/dev/cu.SLAB_USBtoUART*"))
    if not candidates:
        raise RuntimeError("未找到 USB 串口，请用 --port 指定设备")
    return candidates[0]


def read_exact(port: serial.Serial, size: int) -> bytes:
    data = bytearray()
    deadline = time.monotonic() + 120.0
    while len(data) < size:
        chunk = port.read(size - len(data))
        if chunk:
            data.extend(chunk)
            deadline = time.monotonic() + 120.0
            continue
        if time.monotonic() >= deadline:
            raise TimeoutError(f"截图数据超时：{len(data)}/{size} bytes")
    return bytes(data)


def wait_header(port: serial.Serial, timeout: float = 15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = port.readline().strip()
        if not line:
            continue
        match = HEADER_RE.match(line)
        if match:
            return tuple(int(value, 16 if index == 4 else 10)
                         for index, value in enumerate(match.groups(), start=1))
    raise TimeoutError("未收到 JCYSHOT1 截图头")


def decode_packbits(port: serial.Serial, raw_size: int) -> bytes:
    raw = bytearray()
    while len(raw) < raw_size:
        control = read_exact(port, 1)[0]
        count = (control & 0x7F) + 1
        if control & 0x80:
            pixel = read_exact(port, 2)
            raw.extend(pixel * count)
        else:
            raw.extend(read_exact(port, count * 2))
        if len(raw) > raw_size:
            raise ValueError("PACKBITS 数据超过预期尺寸")
    return bytes(raw)


def wait_footer(port: serial.Serial, timeout: float = 5.0) -> int:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = port.readline().strip()
        if not line:
            continue
        match = FOOTER_RE.match(line)
        if match:
            return int(match.group(1), 16)
    raise TimeoutError("未收到 JCYSHOT1 截图尾")


def rgb565be_to_image(raw: bytes, width: int, height: int) -> Image.Image:
    # 当前固件截图流中的 RGB565 两字节按小端顺序发送；屏幕本体又做了
    # 180°安装旋转。按实际数据顺序解码并旋转，得到与实机方向一致的图。
    pixels = np.frombuffer(raw, dtype="<u2").reshape((height, width))
    red = ((pixels >> 11) & 0x1F).astype(np.uint16)
    green = ((pixels >> 5) & 0x3F).astype(np.uint16)
    blue = (pixels & 0x1F).astype(np.uint16)
    rgb = np.empty((height, width, 3), dtype=np.uint8)
    rgb[:, :, 0] = ((red * 255 + 15) // 31).astype(np.uint8)
    rgb[:, :, 1] = ((green * 255 + 31) // 63).astype(np.uint8)
    rgb[:, :, 2] = ((blue * 255 + 15) // 31).astype(np.uint8)
    return Image.fromarray(rgb, mode="RGB").rotate(180)


def main() -> int:
    parser = argparse.ArgumentParser(description="JCY1000-5 USB 截屏")
    parser.add_argument("--port", help="USB 串口，例如 /dev/cu.usbserial-1110")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", "-o", type=Path, help="输出 PNG 文件")
    parser.add_argument("--settle", type=float, default=1.0,
                        help="打开串口后等待秒数（默认 1.0）")
    args = parser.parse_args()

    port_name = args.port or find_serial_port()
    output = args.output
    if output is None:
        project_root = Path(__file__).resolve().parent.parent
        output = project_root / "screenshots" / (
            "JCY1000-5_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".png"
        )
    output.parent.mkdir(parents=True, exist_ok=True)

    print(f"USB: {port_name} @ {args.baud}")
    with serial.Serial(port_name, args.baud, timeout=1,
                       write_timeout=5, rtscts=False, dsrdtr=False) as port:
        # 不要依次改写 DTR/RTS。此屏板的 USB-UART 将两根握手线接入
        # ESP32-S3 自动下载电路，逐根切换会短暂形成复位组合，导致设备
        # 重启并丢失当前 Wi-Fi/BLE 运行状态。pyserial 打开后的默认
        # DTR=True、RTS=True 是该电路的安全组合，直接保持即可。
        time.sleep(args.settle)
        port.reset_input_buffer()
        port.write(b"screenshot\n")
        port.flush()

        width, height, raw_size, header_crc = wait_header(port)
        expected_size = width * height * 2
        if raw_size != expected_size:
            raise ValueError(
                f"截图尺寸不匹配：header={raw_size}, expected={expected_size}"
            )
        print(f"接收: {width}x{height}, RGB565BE, {raw_size} raw bytes")
        raw = decode_packbits(port, raw_size)
        footer_crc = wait_footer(port)

    actual_crc = zlib.crc32(raw) & 0xFFFFFFFF
    if actual_crc != header_crc or footer_crc != header_crc:
        raise ValueError(
            f"CRC错误：header={header_crc:08X}, footer={footer_crc:08X}, "
            f"actual={actual_crc:08X}"
        )

    rgb565be_to_image(raw, width, height).save(output)
    print(f"完成: {output}")
    print(f"CRC32: {actual_crc:08X}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
