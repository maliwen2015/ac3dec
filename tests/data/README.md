# Test streams

Small AC-3 and E-AC-3 streams used by `tests/test_api.c` and
`tests/run_tests.py`.  All are 48 kHz and deliberately short so the test suite
stays fast.

| File | Frames | Layout | Bit rate | Exercises |
| --- | --- | --- | --- | --- |
| `tiny_stereo.ac3` | 10 | 2/0 | 192 kbps | coupling, grouped mantissas, dither |
| `tiny_51.ac3` | 10 | 3/2 + LFE | 384 kbps | full 5.1, LFE, coupling |
| `mono.ac3` | 4 | 1/0 | 192 kbps | single channel, no coupling |
| `stereo_nocpl.ac3` | 4 | 2/0 | 640 kbps | no coupling, high bit rate |
| `eac3_mono.eac3` | 10 | 1/0 | 96 kbps | E-AC-3 framing and BSI |
| `eac3_stereo.eac3` | 10 | 2/0 | 192 kbps | E-AC-3 exponent strategies |
| `eac3_51.eac3` | 10 | 3/2 + LFE | 448 kbps | E-AC-3 coupling, LFE, 5.1 |

The streams are synthetic test material; they do not contain copyrighted
programme audio.
