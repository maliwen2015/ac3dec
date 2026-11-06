/*
 * The Dolby E decoding algorithm and tables in this file are derived from
 * the FFmpeg "dolby_e" decoder (Copyright (C) 2017 foo86), which is
 * distributed under the GNU Lesser General Public License, version 2.1 or
 * later.  This file is therefore licensed under the LGPL-2.1-or-later as
 * well; see COPYING.LGPL.
 */
/*
 * Dolby E decoder - platform helpers and public API.
 *
 * The codec core in dolby_e_core.inc is an independent implementation of
 * the reverse-engineered Dolby E decoding algorithm.  This file provides
 * the KBD window, the inverse MDCT, the small float DSP helpers and the
 * public decoder interface.
 */
#include "dolby_e_core.inc"

/* ------------------------------------------------------------------ */
/* Bessel I0 and KBD window                                            */
/* ------------------------------------------------------------------ */

static double bessel_i0(double x)
{
    double sum = 1.0;
    double term = 1.0;
    double y = x * x * 0.25;
    int k;

    for (k = 1; k < 64; k++) {
        term *= y / ((double)k * k);
        sum += term;
        if (term < 1e-14 * sum)
            break;
    }
    return sum;
}

void ff_kbd_window_init(float *win, float alpha, int n)
{
    double temp[1025];
    double scale = 0.0;
    double sum = 0.0;
    double alpha2 = 4.0 * (alpha * M_PI / n) * (alpha * M_PI / n);
    int i;

    for (i = 0; i <= n / 2; i++) {
        temp[i] = bessel_i0(sqrt(i * (n - i) * alpha2));
        scale += temp[i] * (1 + (i && i < n / 2));
    }
    scale = 1.0 / (scale + 1);

    for (i = 0; i <= n / 2; i++) {
        sum += temp[i];
        win[i] = (float)sqrt(sum * scale);
    }
    for (; i < n; i++) {
        sum += temp[n - i];
        win[i] = (float)sqrt(sum * scale);
    }
}

/* ------------------------------------------------------------------ */
/* float DSP                                                           */
/* ------------------------------------------------------------------ */

static void vector_fmul_add_c(float *dst, const float *src0,
                              const float *src1, const float *src2, int len)
{
    int i;

    for (i = 0; i < len; i++)
        dst[i] = src0[i] * src1[i] + src2[i];
}

static void vector_fmul_scalar_c(float *dst, const float *src, float mul,
                                 int len)
{
    int i;

    for (i = 0; i < len; i++)
        dst[i] = src[i] * mul;
}

AVFloatDSPContext *avpriv_float_dsp_alloc(int bitexact)
{
    AVFloatDSPContext *dsp = (AVFloatDSPContext *)calloc(1, sizeof(*dsp));

    (void)bitexact;
    if (dsp == NULL)
        return NULL;
    dsp->vector_fmul_add = vector_fmul_add_c;
    dsp->vector_fmul_scalar = vector_fmul_scalar_c;
    return dsp;
}

/* ------------------------------------------------------------------ */
/* inverse MDCT                                                        */
/* ------------------------------------------------------------------ */

static void imdct_init_tables(AVTXContext *tx)
{
    int len = tx->len;
    int half = len >> 1;
    double phase = M_PI / (4.0 * len);
    int i, j;

    tx->cos_d = (float *)malloc((size_t)half * len * sizeof(float));
    tx->cos_u = (float *)malloc((size_t)half * len * sizeof(float));
    if (tx->cos_d == NULL || tx->cos_u == NULL)
        return;

    for (i = 0; i < half; i++) {
        double i_d = phase * (4 * (len / 2) - 2 * i - 1);
        double i_u = phase * (3 * len + 2 * i + 1);

        for (j = 0; j < len; j++) {
            double a = 2 * j + 1;

            tx->cos_d[(size_t)i * len + j] = (float)cos(a * i_d);
            tx->cos_u[(size_t)i * len + j] = (float)cos(a * i_u);
        }
    }
}

static void imdct_half(AVTXContext *tx, float *dst, const float *src)
{
    int len = tx->len;
    int half = len >> 1;
    int i, j;

    for (i = 0; i < half; i++) {
        const float *cd = tx->cos_d + (size_t)i * len;
        const float *cu = tx->cos_u + (size_t)i * len;
        float sum_d = 0.0f, sum_u = 0.0f;

        for (j = 0; j < len; j++) {
            float v = src[j];

            sum_d += v * cd[j];
            sum_u += v * cu[j];
        }
        dst[i] = sum_d * tx->scale;
        dst[i + half] = -sum_u * tx->scale;
    }
}

static void dbe_imdct(AVTXContext *tx, void *out, const void *in,
                      ptrdiff_t stride)
{
    float *dst = (float *)out;
    const float *src = (const float *)in;
    int len = tx->len;

    (void)stride;
    if (!tx->full) {
        imdct_half(tx, dst, src);
    } else {
        int total = len << 1;
        int half_total = total >> 1;
        int quarter = total >> 2;
        int i;

        imdct_half(tx, dst + quarter, src);
        for (i = 0; i < quarter; i++) {
            dst[i] = -dst[half_total - i - 1];
            dst[total - i - 1] = dst[half_total + i];
        }
    }
}

int av_tx_init(AVTXContext **ctx, av_tx_fn *fn, int type, int inv, int len,
               const void *scale, uint64_t flags)
{
    AVTXContext *tx;

    (void)type;
    (void)inv;
    tx = (AVTXContext *)calloc(1, sizeof(*tx));
    if (tx == NULL)
        return AVERROR_ENOMEM;
    tx->len = len;
    tx->full = (flags & AV_TX_FULL_IMDCT) != 0;
    tx->scale = *(const float *)scale;
    imdct_init_tables(tx);
    if (tx->cos_d == NULL || tx->cos_u == NULL) {
        av_tx_uninit(&tx);
        return AVERROR_ENOMEM;
    }
    *ctx = tx;
    *fn = dbe_imdct;
    return 0;
}

void av_tx_uninit(AVTXContext **ctx)
{
    if (*ctx != NULL) {
        free((*ctx)->cos_d);
        free((*ctx)->cos_u);
        free(*ctx);
        *ctx = NULL;
    }
}

/* ------------------------------------------------------------------ */
/* decoder                                                             */
/* ------------------------------------------------------------------ */

struct dbe_decoder {
    DBEDecodeContext s;
    float *buf[MAX_CHANNELS];
    int channels;
    int sample_rate;
};

static int dbe_tables_ready;

dbe_decoder *dbe_open(void)
{
    dbe_decoder *dec = (dbe_decoder *)calloc(1, sizeof(*dec));
    float scale = 2.0f;
    int i;

    if (dec == NULL)
        return NULL;

    if (!dbe_tables_ready) {
        init_tables();
        dbe_tables_ready = 1;
    }

    dec->s.fdsp = avpriv_float_dsp_alloc(0);
    if (dec->s.fdsp == NULL)
        goto fail;

    for (i = 0; i < 3; i++) {
        if (av_tx_init(&dec->s.imdct[0][i], &dec->s.imdct_fn[0][i],
                       AV_TX_FLOAT_MDCT, 1,
                       1 << (imdct_bits_tab[i] - 1), &scale, 0) < 0)
            goto fail;
        if (av_tx_init(&dec->s.imdct[1][i], &dec->s.imdct_fn[1][i],
                       AV_TX_FLOAT_MDCT, 1,
                       1 << (imdct_bits_tab[i] - 1), &scale,
                       AV_TX_FULL_IMDCT) < 0)
            goto fail;
    }

    for (i = 0; i < MAX_CHANNELS; i++) {
        dec->buf[i] = (float *)calloc(FRAME_SAMPLES, sizeof(float));
        if (dec->buf[i] == NULL)
            goto fail;
    }
    return dec;

fail:
    dbe_close(dec);
    return NULL;
}

void dbe_close(dbe_decoder *dec)
{
    int i;

    if (dec == NULL)
        return;
    for (i = 0; i < 3; i++) {
        av_tx_uninit(&dec->s.imdct[0][i]);
        av_tx_uninit(&dec->s.imdct[1][i]);
    }
    av_freep(&dec->s.fdsp);
    for (i = 0; i < MAX_CHANNELS; i++)
        free(dec->buf[i]);
    free(dec);
}

static void output_frame(DBEDecodeContext *s, float *out[MAX_CHANNELS])
{
    const DolbyEHeaderInfo *const metadata = &s->dectx.metadata;
    const uint8_t *reorder;

    if (metadata->nb_channels == 4)
        reorder = ch_reorder_4;
    else if (metadata->nb_channels == 6)
        reorder = ch_reorder_6;
    else if (metadata->nb_programs == 1 &&
             metadata->output_channel_order == CHANNEL_ORDER_DEFAULT)
        reorder = ch_reorder_8;
    else
        reorder = ch_reorder_n;

    for (int ch = 0; ch < metadata->nb_channels; ch++) {
        float *dst = out[reorder[ch]];

        transform(s, &s->channels[0][ch], s->history[ch], dst);
        transform(s, &s->channels[1][ch], s->history[ch],
                  dst + FRAME_SAMPLES / 2);
        apply_gain(s, metadata->begin_gain[ch], metadata->end_gain[ch], dst);
    }
}

int dbe_decode(dbe_decoder *dec, const uint8_t *data, int size,
               float *out[MAX_CHANNELS], int *channels, int *sample_rate)
{
    DBEDecodeContext *s1 = &dec->s;
    DBEContext *s = &s1->dectx;
    int nch, half, ret, ch;

    if (ff_dolby_e_parse_header(s, data, size) < 0)
        return -1;

    nch = s->metadata.nb_channels;
    if (nch < 1 || nch > MAX_CHANNELS)
        return -2;
    half = nch / 2;

    if ((ret = parse_audio(s1, 0, half, 0)) < 0)
        return ret;
    if ((ret = parse_audio(s1, half, nch, 0)) < 0)
        return ret;
    if ((ret = parse_metadata_ext(s1)) < 0)
        return ret;
    if ((ret = parse_audio(s1, 0, half, 1)) < 0)
        return ret;
    if ((ret = parse_audio(s1, half, nch, 1)) < 0)
        return ret;
    if ((ret = parse_meter(s1)) < 0)
        return ret;

    for (ch = 0; ch < nch; ch++)
        out[ch] = dec->buf[ch];
    output_frame(s1, out);

    dec->channels = nch;
    dec->sample_rate = s->metadata.sample_rate;
    if (channels)
        *channels = nch;
    if (sample_rate)
        *sample_rate = s->metadata.sample_rate;
    return nch;
}

int dbe_channels(const dbe_decoder *dec)
{
    return dec ? dec->channels : 0;
}

/* ------------------------------------------------------------------ */
/* SMPTE 337M / AES3 demultiplexing                                    */
/* ------------------------------------------------------------------ */

#define S337M_MARKER_16 0x72F81F4EULL
#define S337M_MARKER_20 0x20876FF0E154ULL
#define S337M_MARKER_24 0x72F8961F4EA5ULL

static int s337m_offset(int word_bits, int data_size, int *offset)
{
    int words;

    if (word_bits == 20)
        data_size >>= 4;
    words = data_size / word_bits;
    switch (words) {
    case 3648:
        *offset = 1920;
        break;
    case 3644:
        *offset = 2002;
        break;
    case 3640:
        *offset = 2000;
        break;
    case 3040:
        *offset = 1601;
        break;
    default:
        return -1;
    }
    *offset -= 4;
    *offset *= ((word_bits + 7) >> 3) * 2;
    return 0;
}

int dbe_s337m_next(const uint8_t *data, int size, int *pos,
                   uint8_t *out, int out_cap, int *out_size)
{
    uint64_t state = 0;
    int p = *pos;

    while (p < size) {
        state = (state << 8) | data[p++];
        if (state == S337M_MARKER_16) {
            int data_type, data_size, offset, word_bits = 16;

            if (p + 4 > size)
                return 0;
            data_type = AV_RL16(data + p);
            data_size = AV_RL16(data + p + 2);
            p += 4;
            if ((data_type & 0x1F) != 0x1C ||
                s337m_offset(word_bits, data_size, &offset) < 0)
                continue;
            if (offset > out_cap)
                return -1;
            if (p + offset > size)
                return 0;
            memcpy(out, data + p, (size_t)offset);
            for (int i = 0; i + 1 < offset; i += 2) {
                uint8_t t = out[i];

                out[i] = out[i + 1];
                out[i + 1] = t;
            }
            *out_size = offset;
            *pos = p + offset;
            return 1;
        }
        if ((state & 0xFFFFFFFFFFFFULL) == S337M_MARKER_24 ||
            (state & 0xF0FFFFF0FFFFULL) == S337M_MARKER_20) {
            int word_bits = (state & 0xFFFFFFFFFFFFULL) == S337M_MARKER_24
                            ? 24 : 20;
            int data_type, data_size, offset;

            if (p + 6 > size)
                return 0;
            data_type = (int)AV_RL24(data + p);
            data_size = (int)AV_RL24(data + p + 3);
            p += 6;
            if (word_bits == 20) {
                data_type >>= 8;
                data_size >>= 4;
            } else {
                data_type >>= 8;
            }
            if ((data_type & 0x1F) != 0x1C ||
                s337m_offset(word_bits, data_size, &offset) < 0)
                continue;
            if (offset > out_cap)
                return -1;
            if (p + offset > size)
                return 0;
            memcpy(out, data + p, (size_t)offset);
            for (int i = 0; i + 2 < offset; i += 3) {
                uint8_t t = out[i];

                out[i] = out[i + 2];
                out[i + 2] = t;
            }
            *out_size = offset;
            *pos = p + offset;
            return 1;
        }
    }
    *pos = p;
    return 0;
}

