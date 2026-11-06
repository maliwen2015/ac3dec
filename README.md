# ac3dec

A self-contained ATSC A/52 AC-3 and E-AC-3 (Dolby Digital Plus) audio decoder
written in C99, plus a Dolby E (SMPTE 337M) decoder and small command line
tools that write WAV or raw PCM.

The decoder implements the full AC-3 decoding pipeline: the syncframe header
and BSI, exponent decoding, parametric bit allocation, mantissa dequantisation
(including the grouped quantisers), coupling, rematrixing, block switching,
dither, dynamic range control, dialogue normalisation, the inverse MDCT with
overlap-add and the A/52 downmix matrices.  E-AC-3 adds its own syncframe and
BSI syntax, per-block strategies, 1/2/3/6-block frames and the E-AC-3 bit
allocation tables on top of the same decoding core.  Input and output are
handled through a small packet/frame API in the style of the sibling
`mpeg2dec` and `h264dec` projects.

## Features

* AC-3 (A/52) and E-AC-3 (A/52 Annex E, Dolby Digital Plus)
* 32, 44.1 and 48 kHz streams, including AC-3 half-rate bitstreams (bsid 9/10)
  and E-AC-3 reduced-rate streams (fscod 3)
* E-AC-3 syncframes with 1, 2, 3 or 6 audio blocks and dependent-substream
  skipping
* E-AC-3 adaptive hybrid transform (AHT): vector quantisation, gain-adaptive
  quantisation and the 6-point inverse transform
* E-AC-3 spectral extension (SPX): band structure, coordinates, copy and
  noise-blend reconstruction
* All audio coding modes: 1/0, 2/0, 3/0, 2/1, 3/1, 2/2, 3/2 and 1+1 (dual mono)
* LFE channel, coupling (including phase flags), rematrixing
* Long and short blocks (block switching)
* Dither of zero-bit mantissas (can be disabled)
* Dynamic range control (line and RF mode) and heavy compression
* Optional dialogue normalisation
* Downmix to mono, stereo, 3/0, 2/1, 3/1, 2/2, 3/2, Dolby Surround (Lt/Rt),
  or a single channel of a 1+1 stream.  A requested layout always produces
  exactly that layout; channels the stream does not carry are silent and
  sources the layout cannot carry are folded into the nearest channels.
* 32-bit float or 16-bit signed output, planar or interleaved
* Optional CRC verification
* Dolby E: 1-8 channels, 16/20/24-bit SMPTE 337M carriage, metadata, gains
* No external dependencies beyond libm

## Building

### Make

```sh
make                # build/libac3dec.a and build/ac3dec_example
make dolby-e        # build/dolby_e_example (Dolby E / SMPTE 337M)
make test           # build and run the unit tests
make check          # unit tests + end-to-end tests (Python 3, optional ffmpeg)
make clean
```

Set `CC`, `AR`, `CFLAGS` or `BUILD` to override the defaults.  `make check`
uses `python3`; on platforms where only `python` exists (common on Windows)
pass `PYTHON=python`.

### CMake

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## Example decoder

Both AC-3 and E-AC-3 bitstreams are accepted (the container/extension does not
matter; sync and header detection is automatic).

```sh
build/ac3dec_example input.ac3 output.wav
build/ac3dec_example input.eac3 output.wav -o stereo
build/ac3dec_example input.eac3 output.f32 -r -f f32 -t
```

Dolby E is carried in SMPTE 337M (AES3) streams:

```sh
build/dolby_e_example input.s337m output.wav
```

Options:

| Option | Meaning |
| --- | --- |
| `-f f32\|s16` | output sample format (default `s16`) |
| `-r` | write raw PCM instead of a WAV file |
| `-p` | planar output (raw only) |
| `-o MODE` | `same`, `mono`, `stereo`, `dolby`, `3f`, `2f1r`, `3f1r`, `2f2r`, `3f2r`, `ch1`, `ch2` |
| `-L` | keep LFE in stereo/mono downmix |
| `-N` | drop the LFE channel |
| `-n` | disable dynamic range compression |
| `-D` | disable dither of zero-bit coefficients |
| `-c` | verify frame CRCs |
| `-g SCALE` | DRC exponent (default 1.0) |
| `-d` | apply dialogue normalisation |
| `-t` | print stream information |
| `-b BYTES` | input packet size (default 8192; `1` exercises the byte-wise input path) |

## API

`ac3dec.h` declares the complete interface.  A decoder is opened with
`ac3_dec_open()`, fed with `ac3_dec_send_packet()` and drained with
`ac3_dec_recv_frame()`.  `ac3_dec_flush()` marks the end of the stream and
drains any buffered frames.  Every returned frame must be released with
`ac3_dec_frame_unref()`.

```c
#include "ac3dec.h"

ac3_param_t param = {0};          /* defaults: float, planar, same layout */
void *dec = ac3_dec_open(&param);

ac3_packet_t pkt = {0};
pkt.data = data;
pkt.len = size;
ac3_dec_send_packet(dec, &pkt);

ac3_stream_info info;
ac3_frame_t frame;
for (;;) {
    int ret = ac3_dec_recv_frame(dec, &info, &frame);
    if (ret == AC3_DEC_GOT_HEADER) {
        printf("%d Hz, %d channels\n", info.sample_rate, info.output_channels);
    } else if (ret == AC3_DEC_GOT_FRAME) {
        /* frame.planes[i], frame.strides[i], frame.samples == 1536 */
        ac3_dec_frame_unref(dec, &frame);
    } else {
        break;
    }
}
ac3_dec_close(dec);
```

A packet may contain any number of bytes: whole syncframes, several
syncframes, a partial frame or a single byte.  Timestamps attached to a packet
are copied to every frame completed while that packet is processed.

`ac3_stream_info` describes the stream and is reported through
`AC3_DEC_GOT_HEADER` whenever the parameters change.  `ac3_frame_t` describes
one decoded frame: `ac3_stream_info.samples` samples per channel (1536 for
AC-3, `num_blocks * 256` for E-AC-3) in the requested format and layout.

### Channel maps

`ac3_stream_info.channel_map` and `ac3_frame_t.channel_map` use the speaker
bits `AC3_SPEAKER_L`, `_R`, `_C`, `_LFE`, `_LS` and `_RS`.  Planar frames
store their channels in ascending speaker-bit order (L, R, C, LFE, LS, RS).
A single surround channel is reported as `AC3_SPEAKER_LS`.

## Testing

`tests/test_imdct.c` compares the fast long/short inverse MDCT against a
direct O(N^2) evaluation of the A/52 transform equation.  `tests/test_api.c`
exercises the public API, stream parameters, packet splitting, output formats
and downmix modes against the AC-3 and E-AC-3 streams in `tests/data`.  When
Python 3 and ffmpeg are available, `tests/run_tests.py` also compares the
decoded PCM with ffmpeg's decoder.

Zero-bit mantissas are filled with dither.  Each decoder uses its own
pseudo-random sequence, so two correct decoders differ by roughly the sum of
their noise sequences; the comparison bounds the difference by the dither
energy the decoder injected, which still detects real decoding errors.

The Dolby E module can be checked against the FATE `16-11` sample with
`tests/run_dolby_e.py`.  The sample and its reference PCM are not bundled;
set `DOLBY_E_SAMPLE` to the sample path (the reference is read from
`<sample>.pcm`).  The current implementation decodes it at about 77 dB SNR
against the reference (differences are 16-bit rounding only).

## Implementation notes

* Tables that can be derived (the KBD window, FFT twiddles, scale factors,
  the dither LFSR and the CRC table) are generated at run time, once, with a
  small spin lock.
* The inverse MDCT uses the A/52 decomposition: a complex pre-twiddle, a
  radix-2 complex IFFT, a post-twiddle and the de-interleaving permutation.
  Short blocks run two 256-point transforms and assemble the synthesis buffer
  from their quarter-wave components.
* E-AC-3 audio blocks are decoded with the strategies carried in the frame
  BSI (exponent strategies, coupling strategy, delta bit allocation and the
  per-channel SNR offsets / fast gains), using the E-AC-3 bit allocation
  tables.  The mantissa, coupling, rematrixing and IMDCT stages are shared
  with the AC-3 path.
* AHT codes all six blocks of a channel together in the first block of the
  frame; the 6-point inverse transform is applied per frequency bin before
  the normal IMDCT.  Spectral extension reconstructs the high band from a
  copy of the low band, shaped by the SPX coordinates and blended with
  noise.
* Input buffering resynchronises on the next `0x0B77` syncword after damaged
  data.  Frames that fail to decode are dropped and counted internally.
* The decoder is not thread safe per instance; separate instances may be used
  from separate threads.

## Limitations

* E-AC-3 enhanced coupling (ECPL) is not implemented.  A syncframe that uses
  it is detected and dropped instead of being decoded incorrectly.
* E-AC-3 dependent substreams are skipped, so the output is limited to the
  independent substream (up to 5.1 channels).
* E-AC-3 reduced sample rates (fscod 3) are decoded on a best-effort basis;
  the A/52 specification does not define the bit allocation for them.
* No error concealment: a damaged syncframe produces no audio frame.
* `ac3_dec_cpu_flags()` always returns 0; the implementation is scalar C.
* Dolby E is a proprietary format without a public specification.  The
  Dolby E module follows the publicly documented reverse-engineered
  algorithm; it accepts SMPTE 337M streams (16/20/24-bit words) and does not
  implement Dolby ED2 or any other proprietary carriage format.
* Dolby E is a trademark of Dolby Laboratories.  Users are responsible for
  obtaining any license required for decoding Dolby E content.

## License

The AC-3/E-AC-3 decoder is BSD 2-Clause; see `COPYING`.  The decoded
algorithms come from ATSC A/52 and the pipeline structure is informed by
liba52; no liba52 source code is included.

The Dolby E module (`dolby_e.h`, `dolby_e_compat.h`, `src/dolby_e*.c` and
`src/dolby_e_core.inc`) is derived from the FFmpeg Dolby E decoder and is
licensed under the GNU LGPL, version 2.1 or later; see `COPYING.LGPL`.
