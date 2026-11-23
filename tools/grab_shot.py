#!/usr/bin/env python3
"""grab_shot.py - grab a MOONSKIM screenshot from the device over USB serial.

Usage:
    grab_shot.py COMx out.png [--scale 2] [--timeout 10] [--baud 115200]

Sends the single-character serial command 'g'. The device answers with

    [SHOT] begin w=120 h=68 fmt=idx8+pal888 bytes=8928
    <base64 lines, 96 chars each = 72 payload bytes>
    [SHOT] end crc=<hex8>

The payload is w*h palette indices (row-major) followed by 256 RGB888 palette entries
(768 bytes). crc is the IEEE CRC-32 (zlib.crc32) of the decoded payload.

The PNG is the logical frame expanded 2x nearest-neighbour exactly like the panel path:
logical row y -> output rows 2y and 2y+1, except the last logical row which emits only one
row, so the default output is 240x135. --scale 1 writes the raw 120x68 frame; --scale 4
doubles the panel image again (480x270) for viewing.

Requires pyserial. Run it with PlatformIO's Python, which already has it:
    %USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe tools\\grab_shot.py COM3 shot.png
Only the standard library is used for PNG writing (zlib + struct).
"""

import argparse
import base64
import struct
import sys
import time
import zlib

PANEL_W, PANEL_H = 240, 135


def parse_kv(line):
    out = {}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def png_bytes(width, height, rows):
    """rows: list of bytes objects, each width*3 RGB bytes."""
    raw = bytearray()
    for r in rows:
        raw.append(0)  # filter: none
        raw.extend(r)

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) +
            chunk(b"IDAT", zlib.compress(bytes(raw), 9)) + chunk(b"IEND", b""))


def render_rows(idx, pal, w, h, scale):
    """Return (width, height, rows) for the requested scale."""
    def logical_row(y):
        row = bytearray()
        base = y * w
        for x in range(w):
            i = idx[base + x] * 3
            row.extend(pal[i:i + 3])
        return bytes(row)

    if scale == 1:
        return w, h, [logical_row(y) for y in range(h)]

    # Panel image: 2x nearest, the last logical row emits only one output row.
    panel = []
    for y in range(h):
        lr = logical_row(y)
        wide = bytearray()
        for x in range(w):
            px = lr[x * 3:x * 3 + 3]
            wide.extend(px)
            wide.extend(px)
        wide = bytes(wide)
        panel.append(wide)
        if 2 * y + 1 < 2 * h - 1:
            panel.append(wide)
    pw, ph = 2 * w, 2 * h - 1
    if scale == 2:
        return pw, ph, panel

    k = scale // 2
    rows = []
    for r in panel:
        big = bytearray()
        for x in range(pw):
            big.extend(r[x * 3:x * 3 + 3] * k)
        big = bytes(big)
        rows.extend([big] * k)
    return pw * k, ph * k, rows


def grab(port, baud, timeout):
    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("pyserial missing: run this with %USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe")

    with serial.Serial(port, baud, timeout=0.2) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(b"g")
        ser.flush()
        deadline = time.time() + timeout
        header = None
        b64 = []
        buf = b""
        while time.time() < deadline:
            data = ser.read(4096)
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                if header is None:
                    if line.startswith("[SHOT] begin"):
                        header = parse_kv(line)
                    continue
                if line.startswith("[SHOT] abort"):
                    sys.exit("device aborted the screenshot (TX deadline)")
                if line.startswith("[SHOT] end"):
                    return header, "".join(b64), parse_kv(line)
                if line.startswith("["):
                    continue  # another log line interleaved: ignore
                b64.append(line)
        sys.exit("timeout waiting for [SHOT] %s" % ("end" if header else "begin"))


def main():
    ap = argparse.ArgumentParser(description="Grab a MOONSKIM screenshot over serial.")
    ap.add_argument("port", help="serial port, e.g. COM3")
    ap.add_argument("out", help="output PNG path")
    ap.add_argument("--scale", type=int, default=2, choices=(1, 2, 4, 6, 8),
                    help="1 = 120x68 logical, 2 = 240x135 panel (default), 4+ = enlarged panel")
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    header, text, end = grab(args.port, args.baud, args.timeout)
    w, h = int(header.get("w", 120)), int(header.get("h", 68))
    if header.get("fmt") != "idx8+pal888":
        sys.exit("unsupported format %r" % header.get("fmt"))
    payload = base64.b64decode(text)
    want = w * h + 768
    if len(payload) != want:
        sys.exit("payload is %d bytes, expected %d" % (len(payload), want))
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    dev_crc = int(end.get("crc", "0"), 16)
    if crc != dev_crc:
        sys.exit("CRC mismatch: got %08x, device says %08x" % (crc, dev_crc))

    idx, pal = payload[:w * h], payload[w * h:]
    pw, ph, rows = render_rows(idx, pal, w, h, args.scale)
    with open(args.out, "wb") as f:
        f.write(png_bytes(pw, ph, rows))
    print("wrote %s (%dx%d) crc=%08x" % (args.out, pw, ph, crc))


if __name__ == "__main__":
    main()
