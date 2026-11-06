#!/usr/bin/env python3
"""Validate the Dolby E decoder against the FATE reference PCM.

The 16-11 sample and its reference PCM are not bundled.  Point the
DOLBY_E_SAMPLE environment variable at the sample file (the reference PCM
is expected next to it as "<sample>.pcm"), or place both under
tests/data/dolby_e/.  The test is skipped when no sample is available.
"""
import os
import struct
import subprocess
import sys
import tempfile
import wave

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = os.path.join(ROOT, "build", "dolby_e_example.exe")
if not os.path.exists(EXE):
    EXE = os.path.join(ROOT, "build", "dolby_e_example")


def find_sample():
    env = os.environ.get("DOLBY_E_SAMPLE")
    if env and os.path.exists(env):
        return env
    for name in ("16-11", "16-11.dbe"):
        p = os.path.join(ROOT, "tests", "data", "dolby_e", name)
        if os.path.exists(p):
            return p
    return None


def main():
    sample = find_sample()
    if sample is None:
        print("skip: no Dolby E sample (set DOLBY_E_SAMPLE)")
        return 0
    ref = sample + ".pcm"
    if not os.path.exists(ref):
        print("skip: no reference PCM next to %s" % sample)
        return 0
    if not os.path.exists(EXE):
        print("fail: %s not built (run make dolby-e)" % EXE)
        return 1

    tmp_name = os.path.join(ROOT, "build", "dolby_e_check.wav")
    try:
        subprocess.run([EXE, sample, tmp_name], check=True)
        w = wave.open(tmp_name, "rb")
        channels = w.getnchannels()
        width = w.getsampwidth()
        data = w.readframes(w.getnframes())
        w.close()

        with open(ref, "rb") as f:
            refdata = f.read()

        if width != 2:
            print("fail: unexpected sample width %d" % width)
            return 1
        got = struct.unpack("<%dh" % (len(data) // 2), data)
        exp = struct.unpack("<%dh" % (len(refdata) // 2), refdata[:len(data)])
        n = min(len(got), len(exp))
        if n == 0:
            print("fail: no samples decoded")
            return 1
        err = 0.0
        sig = 0.0
        for i in range(n):
            e = float(got[i]) - float(exp[i])
            err += e * e
            sig += float(exp[i]) ** 2
        snr = 10.0 * __import__("math").log10(sig / err) if err else 999.0
        print("Dolby E: %d samples, %d ch, SNR %.1f dB" % (n, channels, snr))
        if snr < 40.0:
            print("fail: SNR below 40 dB")
            return 1
        return 0
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


if __name__ == "__main__":
    sys.exit(main())
