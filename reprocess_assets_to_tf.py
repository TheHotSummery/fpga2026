from __future__ import annotations

import hashlib
import shutil
import struct
import sys
from pathlib import Path

from PIL import Image

ROOT = Path(r"e:\SOCCHINA2026FPGA\constantwork_train_V95")
ASSETS = ROOT / "assets"
READY = ROOT / "TF-Ready"
BACKUP = ROOT / "TF-Backup-G"
DRIVE = Path(r"G:/")
CHECK = ROOT / "TF-Check" / "check_tf.py"

BMP_640 = 54 + 640 * 480 * 3  # 921654
BMP_800 = 54 + 800 * 600 * 3  # 1440054


def cover_resize(im: Image.Image, tw: int, th: int) -> Image.Image:
    im = im.convert("RGB")
    sw, sh = im.size
    scale = max(tw / sw, th / sh)
    nw = max(tw, int(round(sw * scale)))
    nh = max(th, int(round(sh * scale)))
    im = im.resize((nw, nh), Image.Resampling.LANCZOS)
    left = max(0, (nw - tw) // 2)
    top = max(0, (nh - th) // 2)
    return im.crop((left, top, left + tw, top + th))


def save_bmp24(im: Image.Image, path: Path) -> bytes:
    path.parent.mkdir(parents=True, exist_ok=True)
    im.save(path, format="BMP")
    data = path.read_bytes()
    if data[:2] != b"BM":
        raise RuntimeError(f"{path.name}: not a BMP")
    w, h = struct.unpack_from("<ii", data, 18)
    bpp, comp = struct.unpack_from("<HI", data, 28)
    if bpp != 24 or comp != 0:
        raise RuntimeError(f"{path.name}: need 24-bit uncompressed, got bpp={bpp} comp={comp}")
    if h < 0:
        raise RuntimeError(f"{path.name}: top-down BMP not accepted by FPGA")
    expected = 54 + abs(w) * abs(h) * 3
    if len(data) != expected:
        raise RuntimeError(f"{path.name}: size {len(data)} != {expected}")
    return data


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def inplace_overwrite(dst: Path, data: bytes) -> None:
    old = dst.stat().st_size
    if old != len(data):
        raise RuntimeError(f"{dst}: size {old} != {len(data)}, refuse in-place")
    with dst.open("r+b") as f:
        f.write(data)
        f.flush()
    if dst.stat().st_size != len(data):
        raise RuntimeError(f"{dst}: size changed after in-place write")


def write_new_file(dst: Path, data: bytes) -> None:
    if dst.exists():
        dst.unlink()
    with dst.open("wb") as f:
        f.write(data)
        f.flush()
    if dst.stat().st_size != len(data):
        raise RuntimeError(f"{dst}: write size mismatch")


def main() -> int:
    BACKUP.mkdir(parents=True, exist_ok=True)
    READY.mkdir(parents=True, exist_ok=True)

    jobs = [
        ("0001.png", "0001.BMP", 640, 480),
        ("0002.png", "0002.BMP", 640, 480),
        ("0003.png", "0003.BMP", 640, 480),
        ("0004.png", "0004.BMP", 640, 480),
        ("0005.png", "0005.BMP", 640, 480),
        ("0007.png", "0007.BMP", 640, 480),
        ("0008.png", "0008.BMP", 800, 600),
    ]

    converted = {}
    print("=== convert PNGs ===")
    for src_name, bmp_name, w, h in jobs:
        src = ASSETS / src_name
        if not src.exists():
            raise FileNotFoundError(src)
        im = cover_resize(Image.open(src), w, h)
        data = save_bmp24(im, READY / bmp_name)
        converted[bmp_name] = data
        print(f"  {src_name} {Image.open(src).size} -> {bmp_name} {w}x{h} {len(data)} bytes sha={sha(data)[:12]}")

    # Keep a 640x480 revert copy of 0008 for easy rollback.
    revert = cover_resize(Image.open(ASSETS / "0008.png"), 640, 480)
    save_bmp24(revert, READY / "0008_640x480.BMP")
    print("  revert copy TF-Ready/0008_640x480.BMP ready")

    if not DRIVE.exists():
        print("G:\\ not found, converted files left in TF-Ready only")
        return 1

    print("=== backup current G: BMPs ===")
    for name in ["0001.BMP", "0002.BMP", "0003.BMP", "0004.BMP", "0005.BMP", "0006.BMP", "0007.BMP", "0008.BMP"]:
        src = DRIVE / name
        if src.exists():
            dst = BACKUP / name
            shutil.copy2(src, dst)
            print(f"  backup {name} -> TF-Backup-G ({src.stat().st_size} bytes)")

    print("=== write G: (0006 unchanged) ===")
    for name, data in converted.items():
        dst = DRIVE / name
        if not dst.exists():
            raise FileNotFoundError(dst)
        if len(data) == dst.stat().st_size:
            inplace_overwrite(dst, data)
            print(f"  in-place {name} {len(data)}")
        else:
            print(f"  replace {name} {dst.stat().st_size} -> {len(data)}")
            write_new_file(dst, data)

    print("=== verify G: BMP headers ===")
    for name, data in converted.items():
        on_card = (DRIVE / name).read_bytes()
        if on_card != data:
            raise RuntimeError(f"{name} on G: does not match TF-Ready")
        w, h = struct.unpack_from("<ii", on_card, 18)
        print(f"  {name} {w}x{h} {len(on_card)} sha={sha(on_card)[:12]} MATCH")

    print("=== FAT32 contiguity ===")
    sys.argv = [str(CHECK), "G:/"]
    import runpy
    runpy.run_path(str(CHECK), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
