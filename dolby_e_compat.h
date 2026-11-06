/*
 * The Dolby E decoding algorithm and tables in this file are derived from
 * the FFmpeg "dolby_e" decoder (Copyright (C) 2017 foo86), which is
 * distributed under the GNU Lesser General Public License, version 2.1 or
 * later.  This file is therefore licensed under the LGPL-2.1-or-later as
 * well; see COPYING.LGPL.
 */
/*
 * Minimal compatibility layer used by the Dolby E decoder.
 *
 * The decoder core is derived from the reference algorithm; this header
 * provides the small set of bitstream, transform and DSP helpers it needs so
 * the module stays dependency free.
 */
#ifndef DOLBY_E_COMPAT_H
#define DOLBY_E_COMPAT_H

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define AV_INPUT_BUFFER_PADDING_SIZE 64
#define MAX_CHANNELS 8
#define FRAME_SAMPLES 1792

#define av_assert0(x) assert(x)
#define av_cold
#define av_always_inline inline
#define av_freep(p) do { free(*(void **)(p)); *(void **)(p) = NULL; } while (0)
#define av_mallocz(n) calloc(1, (n))
#define av_log(...) ((void)0)
#define AV_LOG_ERROR 0
#define AV_LOG_WARNING 1
#define AVERROR(e) (-(e))
#define AVERROR_INVALIDDATA (-1)
#define AVERROR_PATCHWELCOME (-2)
#define AVERROR_UNKNOWN (-3)
#define AVERROR_EOF (-4)
#define AVERROR_ENOMEM (-5)

#define FFMIN(a, b) ((a) > (b) ? (b) : (a))
#define FFMAX(a, b) ((a) > (b) ? (a) : (b))
#define FFABS(a) ((a) >= 0 ? (a) : -(a))
#define FFSWAP(type, a, b) do { type tmp_ = (a); (a) = (b); (b) = tmp_; } while (0)
#define av_clip(x, lo, hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))

static inline int av_clip_uintp2(int a, int p)
{
    if (a & ~((1 << p) - 1))
        return a < 0 ? 0 : (1 << p) - 1;
    return a;
}

#define DECLARE_ALIGNED(n, type, name) type __attribute__((aligned(n))) name
#define LOCAL_ALIGNED_32(type, name, size) \
    type __attribute__((aligned(32))) name size

#define AV_RB16(p) ((uint16_t)(((const uint8_t *)(p))[0] << 8 | ((const uint8_t *)(p))[1]))
#define AV_RB24(p) ((uint32_t)(((const uint8_t *)(p))[0] << 16 | \
                               ((const uint8_t *)(p))[1] << 8 | \
                               ((const uint8_t *)(p))[2]))
#define AV_RL16(p) ((uint16_t)((const uint8_t *)(p))[0] | \
                    (uint16_t)(((const uint8_t *)(p))[1] << 8))
#define AV_RL24(p) ((uint32_t)((const uint8_t *)(p))[0] | \
                    ((uint32_t)((const uint8_t *)(p))[1] << 8) | \
                    ((uint32_t)((const uint8_t *)(p))[2] << 16))
#define AV_WB16(p, v) do { \
    ((uint8_t *)(p))[0] = (uint8_t)((v) >> 8); \
    ((uint8_t *)(p))[1] = (uint8_t)(v); \
} while (0)
#define AV_WB24(p, v) do { \
    ((uint8_t *)(p))[0] = (uint8_t)((v) >> 16); \
    ((uint8_t *)(p))[1] = (uint8_t)((v) >> 8); \
    ((uint8_t *)(p))[2] = (uint8_t)(v); \
} while (0)

/* ------------------------------------------------------------------ */
/* bit reader                                                          */
/* ------------------------------------------------------------------ */

typedef struct GetBitContext {
    const uint8_t *buf;
    int size_bits;
    int pos;
    int error;
} GetBitContext;

static inline int init_get_bits(GetBitContext *gb, const uint8_t *buf,
                                int size_bits)
{
    gb->buf = buf;
    gb->size_bits = size_bits;
    gb->pos = 0;
    gb->error = 0;
    return 0;
}

static inline void init_get_bits8(GetBitContext *gb, const uint8_t *buf,
                                  int size_bytes)
{
    init_get_bits(gb, buf, size_bytes * 8);
}

static inline int get_bits_count(const GetBitContext *gb)
{
    return gb->pos;
}

static inline int get_bits_left(const GetBitContext *gb)
{
    return gb->size_bits - gb->pos;
}

static inline int get_bits(GetBitContext *gb, int n)
{
    unsigned v = 0;
    int i;

    if (n == 0)
        return 0;
    if (gb->pos + n > gb->size_bits) {
        gb->error = 1;
        gb->pos = gb->size_bits;
        return 0;
    }
    for (i = 0; i < n; i++) {
        int bit = (gb->buf[(gb->pos + i) >> 3] >> (7 - ((gb->pos + i) & 7))) & 1;
        v = (v << 1) | (unsigned)bit;
    }
    gb->pos += n;
    return (int)v;
}

static inline int get_bits1(GetBitContext *gb)
{
    return get_bits(gb, 1);
}

static inline int get_sbits(GetBitContext *gb, int n)
{
    int v = get_bits(gb, n);
    if (n > 0 && (v & (1 << (n - 1))))
        v -= 1 << n;
    return v;
}

static inline void skip_bits(GetBitContext *gb, int n)
{
    gb->pos += n;
    if (gb->pos > gb->size_bits) {
        gb->pos = gb->size_bits;
        gb->error = 1;
    }
}

static inline void skip_bits1(GetBitContext *gb)
{
    skip_bits(gb, 1);
}

static inline void skip_bits_long(GetBitContext *gb, int n)
{
    skip_bits(gb, n);
}

/* ------------------------------------------------------------------ */
/* bit writer (input conversion)                                       */
/* ------------------------------------------------------------------ */

typedef struct PutBitContext {
    uint8_t *buf;
    int size_bits;
    int pos;
} PutBitContext;

static inline void init_put_bits(PutBitContext *pb, uint8_t *buf, int size)
{
    pb->buf = buf;
    pb->size_bits = size * 8;
    pb->pos = 0;
    memset(buf, 0, (size_t)size);
}

static inline void put_bits(PutBitContext *pb, int n, unsigned v)
{
    int i;

    for (i = n - 1; i >= 0; i--) {
        int bit = (int)((v >> i) & 1);
        if (pb->pos < pb->size_bits)
            pb->buf[pb->pos >> 3] |= (uint8_t)(bit << (7 - (pb->pos & 7)));
        pb->pos++;
    }
}

static inline void flush_put_bits(PutBitContext *pb)
{
    (void)pb;
}

/* ------------------------------------------------------------------ */
/* transform and DSP shims                                             */
/* ------------------------------------------------------------------ */

typedef struct AVTXContext {
    int len;    /* number of input coefficients */
    int full;   /* full inverse MDCT (2*len output) */
    float scale;
    float *cos_d;
    float *cos_u;
} AVTXContext;

typedef void (*av_tx_fn)(AVTXContext *tx, void *out, const void *in,
                         ptrdiff_t stride);

#define AV_TX_FLOAT_MDCT 0
#define AV_TX_FULL_IMDCT (1 << 0)

int av_tx_init(AVTXContext **ctx, av_tx_fn *fn, int type, int inv, int len,
               const void *scale, uint64_t flags);
void av_tx_uninit(AVTXContext **ctx);

typedef struct AVFloatDSPContext {
    void (*vector_fmul_add)(float *dst, const float *src0, const float *src1,
                            const float *src2, int len);
    void (*vector_fmul_scalar)(float *dst, const float *src, float mul, int len);
} AVFloatDSPContext;

AVFloatDSPContext *avpriv_float_dsp_alloc(int bitexact);

/* KBD window (Kaiser-Bessel derived) */
void ff_kbd_window_init(float *window, float alpha, int n);

/* Minimal AVCodecContext used only for logging hooks in the decoder core. */
typedef struct AVCodecContext {
    void *priv_data;
    int err_recognition;
} AVCodecContext;

typedef struct AVClass AVClass;

#define AV_EF_EXPLODE 0
#define avpriv_report_missing_feature(...) ((void)0)
#define avpriv_request_sample(...) ((void)0)

#endif /* DOLBY_E_COMPAT_H */
