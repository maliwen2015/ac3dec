#!/usr/bin/env python3
"""End-to-end tests for the ac3dec example decoder.

The bundled streams are decoded and their frame counts and stream parameters
are checked.  If ffmpeg is available, the decoded PCM is also compared with
ffmpeg's AC-3 decoder.

Zero-bit mantissas are filled with dither noise.  AC-3 decoders pick their own
pseudo-random sequence, so two correct decoders differ by the sum of their two
noise sequences (about sqrt(2) times one sequence).  The comparison therefore
bounds the difference by 2.5x the dither energy our decoder injected, which
still catches any real decoding error while allowing the unavoidable noise.

Usage:
    python3 tests/run_tests.py [--decoder build/ac3dec_example] [--no-ffmpeg]
"""
import argparse
import os
import shutil
import subprocess
import sys

try:
    import numpy as np
except ImportError:  # pragma: no cover - numpy is optional
    np = None

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# name, frames, channels, sample rate, eac3
STREAMS = [
    ("tiny_stereo.ac3", 10, 2, 48000, 0),
    ("tiny_51.ac3", 10, 6, 48000, 0),
    ("mono.ac3", 4, 1, 48000, 0),
    ("stereo_nocpl.ac3", 4, 2, 48000, 0),
    ("eac3_mono.eac3", 10, 1, 48000, 1),
    ("eac3_stereo.eac3", 10, 2, 48000, 1),
    ("eac3_51.eac3", 10, 6, 48000, 1),
]

failures = []


def fail(msg):
    failures.append(msg)
    print("FAIL: " + msg)


def find_decoder(explicit):
    candidates = [
        explicit,
        os.path.join(ROOT, "build", "ac3dec_example"),
        os.path.join(ROOT, "build", "ac3dec_example.exe"),
    ]
    for path in candidates:
        if path and os.path.isfile(path):
            return path
        if path and os.path.isfile(path + ".exe"):
            return path + ".exe"
    return None


def run(cmd):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          text=True)


def decode(decoder, stream, out, extra):
    res = run([decoder, stream, out] + extra)
    if res.returncode != 0:
        fail("decoder exited %d for %s: %s" %
             (res.returncode, os.path.basename(stream), res.stderr.strip()))
        return None
    return res.stderr


def read_f32(path, channels):
    with open(path, "rb") as f:
        data = f.read()
    if len(data) % (4 * channels):
        fail("%s: size %d is not a multiple of %d channels" %
             (path, len(data), channels))
        return None
    return np.frombuffer(data, dtype="<f4").reshape(-1, channels)


def rms(x):
    return float(np.sqrt(np.mean(x.astype(np.float64) ** 2)))


def check_against_ffmpeg(decoder, stream, channels, tmp):
    base = os.path.splitext(os.path.basename(stream))[0]
    ref = os.path.join(tmp, base + ".ref.f32")
    got = os.path.join(tmp, base + ".got.f32")
    nod = os.path.join(tmp, base + ".nodither.f32")

    res = run(["ffmpeg", "-v", "error", "-y", "-i", stream,
               "-f", "f32le", "-acodec", "pcm_f32le", "-ac", str(channels), ref])
    if res.returncode != 0:
        print("SKIP ffmpeg comparison for %s" % base)
        return

    if decode(decoder, stream, got, ["-r", "-f", "f32"]) is None:
        return
    if decode(decoder, stream, nod, ["-r", "-f", "f32", "-D"]) is None:
        return

    a = read_f32(got, channels)
    b = read_f32(ref, channels)
    c = read_f32(nod, channels)
    if a is None or b is None or c is None:
        return
    n = min(len(a), len(b), len(c))
    a, b, c = a[:n], b[:n], c[:n]

    ref_rms = rms(b)
    measured = rms(a - b)
    dither = rms(a - c)
    limit = 2.5 * dither + 5e-5 * max(ref_rms, 1e-9)

    snr = 20 * np.log10(max(ref_rms / max(measured, 1e-12), 1e-12))
    extra = ("measured=%.3e dither=%.3e limit=%.3e snr=%.1f dB" %
             (measured, dither, limit, snr))
    if measured > limit:
        fail("%s: PCM differs from ffmpeg (%s)" % (base, extra))
    else:
        print("ok   %-16s %s" % (base, extra))


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--decoder", default=None)
    parser.add_argument("--no-ffmpeg", action="store_true")
    args = parser.parse_args(argv)

    decoder = find_decoder(args.decoder)
    if not decoder:
        print("FAIL: decoder binary not found; build the project first")
        return 1

    use_ffmpeg = not args.no_ffmpeg and shutil.which("ffmpeg") is not None
    if np is None:
        use_ffmpeg = False
        print("note: numpy not installed, skipping PCM comparison")

    # Work inside the build tree: system temporary directories may be
    # read-only in sandboxed environments.
    tmp = os.path.join(ROOT, "build", "test-tmp")
    os.makedirs(tmp, exist_ok=True)
    try:
        for name, frames, channels, rate, eac3 in STREAMS:
            stream = os.path.join(HERE, "data", name)
            if not os.path.isfile(stream):
                fail("missing test stream %s" % stream)
                continue

            out = os.path.join(tmp, name + ".wav")
            stderr = decode(decoder, stream, out, ["-t"])
            if stderr is None:
                continue

            if "decoded %d frames" % frames not in stderr:
                fail("%s: expected %d frames, got: %s" %
                     (name, frames, stderr.strip().splitlines()[-1]))
                continue
            if "header:" not in stderr:
                fail("%s: no stream header reported" % name)
                continue
            if eac3 and "E-AC-3" not in stderr:
                fail("%s: expected an E-AC-3 stream header" % name)
                continue
            if "-> %d ch" % channels not in stderr:
                fail("%s: expected %d output channels" % (name, channels))
                continue
            if "%d Hz" % rate not in stderr:
                fail("%s: expected %d Hz" % (name, rate))
                continue
            print("ok   %-16s %d frames, %d ch, %d Hz" %
                  (name, frames, channels, rate))

            if use_ffmpeg:
                check_against_ffmpeg(decoder, stream, channels, tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print("\n%d test(s) failed" % len(failures))
        return 1
    print("\nall end-to-end tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
