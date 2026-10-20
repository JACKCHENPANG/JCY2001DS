#!/usr/bin/env python3
"""Convert an RGB/RGBA PNG into an LVGL 8 true-color RGB565 C asset."""

import argparse
from pathlib import Path
from PIL import Image


def rgb565(r: int, g: int, b: int) -> int:
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("symbol")
    args = parser.parse_args()

    image = Image.open(args.input).convert("RGB")
    width, height = image.size
    macro = args.symbol.upper()
    values = []
    for r, g, b in image.getdata():
        pixel = rgb565(r, g, b)
        values.extend((pixel & 0xFF, pixel >> 8))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as out:
        out.write("#include <lvgl.h>\n\n")
        out.write("#ifndef LV_ATTRIBUTE_MEM_ALIGN\n#define LV_ATTRIBUTE_MEM_ALIGN\n#endif\n")
        out.write(f"#ifndef LV_ATTRIBUTE_IMG_{macro}\n#define LV_ATTRIBUTE_IMG_{macro}\n#endif\n\n")
        out.write(
            f"const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST "
            f"LV_ATTRIBUTE_IMG_{macro} uint8_t {args.symbol}_map[] = {{\n"
        )
        for offset in range(0, len(values), 24):
            chunk = values[offset:offset + 24]
            out.write("    " + ", ".join(f"0x{value:02X}" for value in chunk) + ",\n")
        out.write("};\n\n")
        out.write(f"const lv_img_dsc_t {args.symbol} = {{\n")
        out.write("    .header.cf = LV_IMG_CF_TRUE_COLOR,\n")
        out.write("    .header.always_zero = 0,\n")
        out.write("    .header.reserved = 0,\n")
        out.write(f"    .header.w = {width},\n")
        out.write(f"    .header.h = {height},\n")
        out.write(f"    .data_size = {width} * {height} * 2,\n")
        out.write(f"    .data = {args.symbol}_map,\n")
        out.write("};\n")


if __name__ == "__main__":
    main()
