# -*- coding: utf-8 -*-
"""2-second last-frame card: high-util resources + timing only."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

_DIR = Path(__file__).resolve().parent
OUT = next(_DIR.glob("01_FPGA*_2K.png"))
W, H = 2560, 1440

BG = (244, 247, 252)
PANEL = (255, 255, 255)
PANEL2 = (248, 250, 255)
LINE = (198, 210, 230)
INK = (28, 36, 52)
MUTED = (90, 105, 130)
CYAN = (0, 130, 170)
CYAN_BAR = (0, 168, 210)
GREEN = (16, 140, 90)
GREEN_BAR = (40, 170, 110)
AMBER_BAR = (230, 150, 40)
MAGENTA_BAR = (210, 70, 150)
TITLE = (10, 70, 110)


def font(size, bold=False):
    path = r"C:\Windows\Fonts\msyhbd.ttc" if bold else r"C:\Windows\Fonts\msyh.ttc"
    return ImageFont.truetype(path, size)


def round_rect(draw, xy, r, fill, outline=None, width=1):
    draw.rounded_rectangle(xy, radius=r, fill=fill, outline=outline, width=width)


def bar(draw, x, y, w, h, pct, color, label, value):
    draw.text((x, y - 40), label, font=font(32, True), fill=INK)
    tw = draw.textlength(value, font=font(30))
    draw.text((x + w - tw, y - 38), value, font=font(30), fill=CYAN)
    round_rect(draw, (x, y, x + w, y + h), 10, (232, 238, 248), LINE, 1)
    fill_w = max(int(w * pct / 100), 16)
    round_rect(draw, (x + 3, y + 3, x + fill_w - 3, y + h - 3), 8, color)
    pct_s = ("%.2f%%" % pct) if pct < 100 else ("%.1f%%" % pct)
    draw.text((x + 18, y + 10), pct_s, font=font(26, True), fill=(255, 255, 255))


def main():
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)

    # centered card for 2s glance
    card = (180, 220, W - 180, H - 220)
    round_rect(d, card, 24, PANEL, LINE, 2)
    d.rectangle((180, 220, 200, H - 220), fill=CYAN_BAR)

    d.text(
        (240, 270),
        "\u5b89\u8def EG4S20BG256  \u00b7  \u8d44\u6e90\u4e0e\u65f6\u5e8f",
        font=font(52, True),
        fill=TITLE,
    )
    d.text(
        (240, 340),
        "Tang Dynasty V6.2.1 (V6.2.168116)  |  Slow & Fast \u5168\u6b63\u88d5\u91cf",
        font=font(28),
        fill=MUTED,
    )
    d.line((240, 400, W - 240, 400), fill=LINE, width=2)

    items = [
        ("LUT", "18,816 / 19,600", 96.00, CYAN_BAR),
        ("MSlice", "4,813 / 4,900", 98.22, GREEN_BAR),
        ("BRAM9K", "61 / 64", 95.31, AMBER_BAR),
        ("MULT18 DSP", "28 / 29", 96.55, MAGENTA_BAR),
    ]
    y0 = 470
    for i, (lab, val, pct, col) in enumerate(items):
        bar(d, 240, y0 + i * 130, 1400, 48, pct, col, lab, val)

    # timing KPIs on the right of bars area
    kx = 1780
    for i, (k, v) in enumerate(
        [("Setup SWNS", "+0.151 ns"), ("Hold HWNS", "+0.024 ns")]
    ):
        yy = 500 + i * 220
        round_rect(d, (kx, yy, W - 240, yy + 180), 16, PANEL2, GREEN, 3)
        d.text((kx + 36, yy + 36), k, font=font(28), fill=MUTED)
        d.text((kx + 36, yy + 88), v, font=font(48, True), fill=GREEN)

    img.save(OUT, "PNG")
    print("wrote", OUT, img.size)


if __name__ == "__main__":
    main()
