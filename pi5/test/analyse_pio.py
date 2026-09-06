#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Analyse an 8-ch S32_LE 16 kHz raw capture from the seeed8micpio card.

usage: analyse_pio.py FILE [--tone HZ] [--skip FRAMES]
Prints frame count, per-slot RMS dBFS / peak / low-byte OR / constant check, per-quarter loudest
slot, and (with --tone) the spectral peak frequency and its level above the slot's noise floor.
"""

import sys
import numpy as np

CH = 8
FS = 16000


def dbfs(x):
    r = np.sqrt(np.mean(x.astype(np.float64) ** 2)) / 2**31
    return 20 * np.log10(max(r, 1e-12))


def main():
    args = sys.argv[1:]
    tone = None
    skip = 1
    if "--tone" in args:
        i = args.index("--tone")
        tone = float(args[i + 1])
        del args[i : i + 2]
    if "--skip" in args:
        i = args.index("--skip")
        skip = int(args[i + 1])
        del args[i : i + 2]
    raw = np.fromfile(args[0], dtype="<i4")
    nframes = len(raw) // CH
    print(f"frames={nframes} ({nframes / FS:.3f} s) trailing_words={len(raw) % CH}")
    x = raw[: nframes * CH].reshape(-1, CH)[
        skip:
    ]  # frame 0 = DMA start-up junk (documented)
    lowbyte = int(np.bitwise_or.reduce((x & 0xFF).ravel()))
    print(f"low_byte_or=0x{lowbyte:02x} (0 => 24-bit-in-32 alignment intact)")
    q = len(x) // 4
    for c in range(CH):
        s = x[:, c]
        const = "CONSTANT" if s.min() == s.max() else ""
        line = f"slot{c}: rms={dbfs(s):7.1f} dBFS min={s.min():12d} max={s.max():12d} {const}"
        if tone:
            w = np.hanning(len(s))
            spec = np.abs(np.fft.rfft(s.astype(np.float64) * w))
            f = np.fft.rfftfreq(len(s), 1 / FS)
            k = np.argmax(spec[1:]) + 1
            band = (f > tone - 50) & (f < tone + 50)
            noise = np.median(spec[~band & (f > 100)])
            line += f" peak={f[k]:7.1f} Hz snr={20 * np.log10(spec[k] / max(noise, 1e-9)):5.1f} dB"
        print(line)
    loud = [
        int(np.argmax([dbfs(x[i * q : (i + 1) * q, c]) for c in range(CH)]))
        for i in range(4)
    ]
    print(f"loudest_slot_per_quarter={loud}")


if __name__ == "__main__":
    main()
