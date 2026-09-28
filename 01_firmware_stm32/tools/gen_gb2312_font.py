# -*- coding: utf-8 -*-
"""
生成 GB2312 全字库点阵，供 lcd_spi_200.c 的 LCD_DisplayChinese 使用。

取模格式（与驱动 LCD_DisplayChinese 的渲染循环严格对应）：
  - 阴码（1=亮点）
  - 逐行式，行优先，从上到下
  - 每行 ceil(width/8) 个字节
  - 每字节内 LSB(bit0) = 最左像素（“逆向”）
字模索引：addr = (GBH-0xA1)*94 + (GBL-0xA1)，GBH:0xA1..0xF7, GBL:0xA1..0xFE
"""
import sys, math
from PIL import Image, ImageFont, ImageDraw

FONT_PATH = r"C:\Windows\Fonts\simsun.ttc"

GBH_LO, GBH_HI = 0xA1, 0xF7   # 区码范围
GBL_LO, GBL_HI = 0xA1, 0xFE   # 位码范围


def make_glyph(ch, font, size):
    """把单个字符渲染成 size×size 的 1bit 位图，返回 bytes（按上面的取模格式）。"""
    W = H = size
    img = Image.new("L", (W, H), 0)
    d = ImageDraw.Draw(img)
    # 居中绘制
    bbox = d.textbbox((0, 0), ch, font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    x = (W - tw) // 2 - bbox[0]
    y = (H - th) // 2 - bbox[1]
    d.text((x, y), ch, fill=255, font=font)
    px = img.load()

    bytes_per_row = (W + 7) // 8
    out = bytearray()
    for row in range(H):
        for bcol in range(bytes_per_row):
            b = 0
            for bit in range(8):
                col = bcol * 8 + bit
                if col < W and px[col, row] >= 128:
                    b |= (1 << bit)          # LSB = 最左像素
            out.append(b)
    return bytes(out)


def render_ascii(data, size):
    """把字模按驱动的方式还原成字符画，用于人工核对。"""
    bytes_per_row = (size + 7) // 8
    lines = []
    for row in range(size):
        s = ""
        for col in range(size):
            byte = data[row * bytes_per_row + col // 8]
            s += "#" if (byte >> (col % 8)) & 1 else "."
        lines.append(s)
    return "\n".join(lines)


def main():
    size = int(sys.argv[1]) if len(sys.argv) > 1 else 24
    font = ImageFont.truetype(FONT_PATH, size)

    if "--preview" in sys.argv:
        for ch in "你好中文测试":
            print(f"[{ch}]")
            print(render_ascii(make_glyph(ch, font, size), size))
            print()
        return

    bytes_per_glyph = ((size + 7) // 8) * size
    out_c = fr"E:\Embedded\touch_board_host\lcd_driver\gb2312_font.c"
    total = 0
    with open(out_c, "wb") as f:
        head = (
            "/* 本文件由 tools/gen_gb2312_font.py 自动生成，请勿手改 */\n"
            "/* GB2312 全字库 %d x %d 点阵，供 LCD_DisplayChinese 偏移索引使用 */\n"
            '#include "lcd_fonts.h"\n\n'
            "const unsigned char GB2312_%d_Table[] = {\n" % (size, size, size)
        )
        f.write(head.encode("gb2312"))
        for gbh in range(GBH_LO, GBH_HI + 1):
            for gbl in range(GBL_LO, GBL_HI + 1):
                try:
                    ch = bytes([gbh, gbl]).decode("gb2312")
                    data = make_glyph(ch, font, size)
                except Exception:
                    data = bytes(bytes_per_glyph)   # 无效码位填 0
                f.write(("  " + ",".join("0x%02X" % b for b in data) + ",\n").encode("ascii"))
                total += 1
        tail = (
            "};\n\n"
            "pFONT GB2312_Font%d = { GB2312_%d_Table, %d, %d, %d, 0 };\n"
            % (size, size, size, size, bytes_per_glyph)
        )
        f.write(tail.encode("gb2312"))
    print(f"生成 {out_c}: {total} 个字模, 每字 {bytes_per_glyph} 字节, 共 {total*bytes_per_glyph} 字节")


if __name__ == "__main__":
    main()
