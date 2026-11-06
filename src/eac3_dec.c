/*
 * E-AC-3 audio block decoding (ATSC A/52 Annex E).
 *
 * The frame-level BSI has already selected the per-block strategies, so an
 * audio block only carries the data those strategies select.  In addition to
 * the plain transform coder this implements the adaptive hybrid transform
 * (AHT), spectral extension (SPX) and the common bit allocation syntax.
 * Enhanced coupling is detected and rejected.
 */
#include <stdio.h>
#include <stdlib.h>

#include "ac3_internal.h"

#define EAC3_UNSUPPORTED (-2)

/* ------------------------------------------------------------------ */
/* noise generator                                                     */
/* ------------------------------------------------------------------ */

/* Initial state of the lagged-Fibonacci generator seeded with 0.  Keeping
 * the same sequence as the reference decoder makes dither, AHT zero-bit
 * noise and spectral extension noise match it sample for sample. */
static const uint32_t eac3_lfg_seed[64] = {
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x3b376b5eu, 0xf56be763u, 0xcba2a7fcu, 0x9751b2c7u,
    0x5b1fe69au, 0x860b05afu, 0x2b2650cau, 0x92a97fb0u,
    0x6d1d0e6du, 0x0f2f8d37u, 0x4a0f4d1au, 0x8b8f0e2cu,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u
};

void ac3_eac3_lfg_init(ac3_codec_t *c)
{
    memcpy(c->lfg_state, eac3_lfg_seed, sizeof(c->lfg_state));
    c->lfg_index = 0;
}

static uint32_t lfg_get(ac3_codec_t *c)
{
    unsigned int index = c->lfg_index;
    uint32_t v = c->lfg_state[index & 63] =
        c->lfg_state[(index - 24) & 63] + c->lfg_state[(index - 55) & 63];

    c->lfg_index = index + 1;
    return v;
}

/* Dither mantissa for a zero-bit coefficient (24-bit scale). */
float ac3_eac3_dither_coeff(ac3_codec_t *c, int exp)
{
    uint32_t r = lfg_get(c);
    int64_t mant = (int64_t)(((r >> 8) * 181u) >> 8) - 5931008;

    return (float)((int32_t)mant >> exp) * (1.0f / 8388608.0f);
}

/* ------------------------------------------------------------------ */
/* bit allocation                                                      */
/* ------------------------------------------------------------------ */

static int calc_lowcomp1(int a, int b0, int b1, int c)
{
    if ((b0 + 256) == b1)
        a = c;
    else if (b0 > b1)
        a = AC3_MAX(a - 64, 0);
    return a;
}

static int calc_lowcomp(int a, int b0, int b1, int bin)
{
    if (bin < 7)
        return calc_lowcomp1(a, b0, b1, 384);
    if (bin < 20)
        return calc_lowcomp1(a, b0, b1, 320);
    return AC3_MAX(a - 128, 0);
}

/* Exponent mapping into PSD and PSD integration over critical bands. */
static void calc_psd(const uint8_t *exp, int start, int end,
                     int16_t *psd, int16_t *band_psd)
{
    int bin, band;

    for (bin = start; bin < end; bin++)
        psd[bin] = (int16_t)(3072 - (exp[bin] << 7));

    bin = start;
    band = ac3_bin_to_band_tab[start];
    do {
        int v = psd[bin++];
        int band_end = AC3_MIN(ac3_band_start_tab[band + 1], end);

        while (bin < band_end) {
            int max = AC3_MAX(v, psd[bin]);
            int adr = AC3_MIN(max - ((v + psd[bin] + 1) >> 1), 255);
            v = max + ac3_log_add_tab[adr];
            bin++;
        }
        band_psd[band++] = (int16_t)v;
    } while (end > ac3_band_start_tab[band]);
}

/* Excitation function, masking curve and delta bit allocation. */
static int calc_mask(const ac3_header_t *h, const int16_t *band_psd,
                     int start, int end, int fast_gain, int is_lfe,
                     int dba_mode, int dba_nsegs, const uint8_t *dba_offsets,
                     const uint8_t *dba_lengths, const uint8_t *dba_values,
                     int16_t *mask)
{
    int16_t excite[AC3_MAX_BANDS];
    int band, band_start, band_end, begin, end1;
    int lowcomp, fastleak, slowleak;

    if (end <= start)
        return -1;

    band_start = ac3_bin_to_band_tab[start];
    band_end = ac3_bin_to_band_tab[end - 1] + 1;

    if (band_start == 0) {
        lowcomp = 0;
        lowcomp = calc_lowcomp1(lowcomp, band_psd[0], band_psd[1], 384);
        excite[0] = band_psd[0] - fast_gain - lowcomp;
        lowcomp = calc_lowcomp1(lowcomp, band_psd[1], band_psd[2], 384);
        excite[1] = band_psd[1] - fast_gain - lowcomp;
        begin = 7;
        for (band = 2; band < 7; band++) {
            if (!(is_lfe && band == 6))
                lowcomp = calc_lowcomp1(lowcomp, band_psd[band],
                                        band_psd[band + 1], 384);
            fastleak = band_psd[band] - fast_gain;
            slowleak = band_psd[band] - h->slow_gain;
            excite[band] = fastleak - lowcomp;
            if (!(is_lfe && band == 6)) {
                if (band_psd[band] <= band_psd[band + 1]) {
                    begin = band + 1;
                    break;
                }
            }
        }

        end1 = AC3_MIN(band_end, 22);
        for (band = begin; band < end1; band++) {
            if (!(is_lfe && band == 6))
                lowcomp = calc_lowcomp(lowcomp, band_psd[band],
                                       band_psd[band + 1], band);
            fastleak = AC3_MAX(fastleak - h->fast_decay,
                               band_psd[band] - fast_gain);
            slowleak = AC3_MAX(slowleak - h->slow_decay,
                               band_psd[band] - h->slow_gain);
            excite[band] = AC3_MAX(fastleak - lowcomp, slowleak);
        }
        begin = 22;
    } else {
        begin = band_start;
        fastleak = (h->cpl_fast_leak << 8) + 768;
        slowleak = (h->cpl_slow_leak << 8) + 768;
    }

    for (band = begin; band < band_end; band++) {
        fastleak = AC3_MAX(fastleak - h->fast_decay,
                           band_psd[band] - fast_gain);
        slowleak = AC3_MAX(slowleak - h->slow_decay,
                           band_psd[band] - h->slow_gain);
        excite[band] = AC3_MAX(fastleak, slowleak);
    }

    for (band = band_start; band < band_end; band++) {
        int tmp = h->db_per_bit - band_psd[band];

        if (tmp > 0)
            excite[band] += tmp >> 2;
        mask[band] = AC3_MAX(
            ac3_hearing_threshold_tab[band >> h->sr_shift][h->fscod],
            excite[band]);
    }

    if (dba_mode == 1 || dba_mode == 2) {
        int i, seg, delta;

        if (dba_nsegs > AC3_MAX_DBA_SEGMENTS)
            return -1;
        band = band_start;
        for (seg = 0; seg < dba_nsegs; seg++) {
            band += dba_offsets[seg];
            if (band >= AC3_MAX_BANDS ||
                dba_lengths[seg] > AC3_MAX_BANDS - band)
                return -1;
            if (dba_values[seg] >= 4)
                delta = (dba_values[seg] - 3) * 128;
            else
                delta = (dba_values[seg] - 4) * 128;
            for (i = 0; i < dba_lengths[seg]; i++) {
                mask[band] = (int16_t)(mask[band] + delta);
                band++;
            }
        }
    }
    return 0;
}

/* Bit allocation pointer for one channel. */
static void calc_bap(const int16_t *mask, const int16_t *psd,
                     int start, int end, int snr_offset, int floor,
                     const uint8_t *bap_tab, uint8_t *bap)
{
    int bin, band, band_end;

    if (snr_offset == -960) {
        memset(bap + start, 0, (size_t)(end - start));
        return;
    }

    bin = start;
    band = ac3_bin_to_band_tab[start];
    do {
        int m = (AC3_MAX(mask[band] - snr_offset - floor, 0) & 0x1FE0) + floor;

        band_end = AC3_MIN(ac3_band_start_tab[band + 1], end);
        band++;
        for (; bin < band_end; bin++) {
            int address = (psd[bin] - m) >> 5;

            if (address < 0)
                address = 0;
            else if (address > 63)
                address = 63;
            bap[bin] = bap_tab[address];
        }
    } while (end > band_end);
}

/*
 * Run the E-AC-3 bit allocation for one channel.  AHT channels use the
 * hybrid-exponent bap table and keep the raw value; the other channels are
 * mapped to the quantiser codes used by the shared mantissa decoder.
 */
static int allocate_channel(const ac3_header_t *h,
                            const uint8_t *exp, int start, int end,
                            int snr_offset, int fast_gain,
                            int dba_mode, int dba_nsegs,
                            const uint8_t *dba_offsets,
                            const uint8_t *dba_lengths,
                            const uint8_t *dba_values,
                            int use_aht, int8_t *bap_out, int is_lfe)
{
    const uint8_t *bap_tab = use_aht ? ac3_eac3_hebap_tab : ac3_ff_bap_tab;
    int16_t psd[AC3_COEFFS];
    int16_t band_psd[AC3_MAX_BANDS + 2];
    int16_t mask[AC3_MAX_BANDS + 2];
    uint8_t bap[AC3_COEFFS];
    int i;

    if (end <= start)
        return 0;

    calc_psd(exp, start, end, psd, band_psd);
    if (calc_mask(h, band_psd, start, end, fast_gain, is_lfe,
                  dba_mode, dba_nsegs, dba_offsets, dba_lengths, dba_values,
                  mask) < 0)
        return -1;
    calc_bap(mask, psd, start, end, snr_offset, h->floor, bap_tab, bap);

    for (i = start; i < end; i++) {
        if (use_aht) {
            if (bap[i] > 19)
                return -1;
            bap_out[i] = (int8_t)bap[i];
        } else {
            if (bap[i] > 15)
                return -1;
            bap_out[i] = ac3_eac3_bap_map[bap[i]];
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* adaptive hybrid transform                                           */
/* ------------------------------------------------------------------ */

#define COEFF_0 10273905LL
#define COEFF_1 11863283LL
#define COEFF_2  3070444LL

/* 6-point IDCT of the pre-mantissas (24-bit fixed point). */
static void idct6(int32_t *pre)
{
    int64_t tmp;
    int64_t even0, even1, even2, odd0, odd1, odd2;

    odd1 = (int64_t)pre[1] - pre[3] - pre[5];

    even2 = ((int64_t)pre[2] * COEFF_0) >> 23;
    tmp = ((int64_t)pre[4] * COEFF_1) >> 23;
    odd0 = (((int64_t)pre[1] + pre[5]) * COEFF_2) >> 23;

    even0 = pre[0] + (tmp >> 1);
    even1 = pre[0] - tmp;

    tmp = even0;
    even0 = tmp + even2;
    even2 = tmp - even2;

    tmp = odd0;
    odd0 = tmp + pre[1] + pre[3];
    odd2 = tmp + pre[5] - pre[3];

    pre[0] = (int32_t)(even0 + odd0);
    pre[1] = (int32_t)(even1 + odd1);
    pre[2] = (int32_t)(even2 + odd2);
    pre[3] = (int32_t)(even2 - odd2);
    pre[4] = (int32_t)(even1 - odd1);
    pre[5] = (int32_t)(even0 - odd0);
}

/*
 * Decode the AHT mantissas of one channel.  All six blocks are coded
 * together in the first block of the frame, either with vector quantisation,
 * gain-adaptive quantisation or plain fixed-point mantissas.
 */
static int decode_aht_channel(ac3_codec_t *c, ac3_bits_t *br,
                              const int8_t *bap, int start, int end,
                              int32_t pre[AC3_COEFFS][AC3_MAX_BLOCKS])
{
    int gaq_gain[AC3_COEFFS];
    int gaq_mode = (int)ac3_bits_get(br, 2);
    int end_bap = (gaq_mode < 2) ? 12 : 17;
    int gs = 0, bin, blk;

    if (gaq_mode == 1 || gaq_mode == 2) {
        for (bin = start; bin < end; bin++) {
            if (bap[bin] > 7 && bap[bin] < end_bap)
                gaq_gain[gs++] = (int)ac3_bits_get(br, 1) << (gaq_mode - 1);
        }
    } else if (gaq_mode == 3) {
        int gc = 2;

        for (bin = start; bin < end; bin++) {
            if (bap[bin] > 7 && bap[bin] < 17) {
                if (gc++ == 2) {
                    int group = (int)ac3_bits_get(br, 5);

                    if (group > 26)
                        group = 26;
                    gaq_gain[gs++] = ac3_ungroup_3_in_5_bits_tab[group][0];
                    gaq_gain[gs++] = ac3_ungroup_3_in_5_bits_tab[group][1];
                    gaq_gain[gs++] = ac3_ungroup_3_in_5_bits_tab[group][2];
                    gc = 0;
                }
            }
        }
    }

    gs = 0;
    for (bin = start; bin < end; bin++) {
        int hebap = bap[bin];
        int bits = ac3_eac3_bits_vs_hebap[hebap];

        if (hebap == 0) {
            for (blk = 0; blk < AC3_MAX_BLOCKS; blk++) {
                uint32_t r = lfg_get(c);
                int32_t v = (int32_t)((r & 0x7FFFFFu) - 0x400000u);

                pre[bin][blk] = c->dither_enabled ? v : 0;
            }
        } else if (hebap < 8) {
            int v;

            if (bits <= 0 || bits > 9)
                return -1;
            v = (int)ac3_bits_get(br, bits);
            for (blk = 0; blk < AC3_MAX_BLOCKS; blk++)
                pre[bin][blk] = (int32_t)ac3_eac3_mantissa_vq[hebap][v][blk]
                                * 256;
        } else {
            int gbits, log_gain;
            int idx = hebap - 8;

            if (idx < 0 || idx >= 12)
                return -1;
            if (gaq_mode != 0 && hebap < end_bap)
                log_gain = gaq_gain[gs++];
            else
                log_gain = 0;
            gbits = bits - log_gain;
            if (gbits <= 0 || gbits > 16)
                return -1;

            for (blk = 0; blk < AC3_MAX_BLOCKS; blk++) {
                int32_t mant = ac3_bits_get_signed(br, gbits);

                if (log_gain && mant == -(1 << (gbits - 1))) {
                    int b;
                    int mbits = bits - (2 - log_gain);
                    int32_t m;

                    if (mbits <= 0 || mbits > 16)
                        return -1;
                    m = ac3_bits_get_signed(br, mbits);
                    m = (int32_t)((uint32_t)m << (23 - (mbits - 1)));
                    if (m >= 0)
                        b = 1 << (23 - log_gain);
                    else
                        b = ac3_eac3_gaq_remap_2_4_b[idx][log_gain - 1] << 8;
                    m += (int32_t)(((int64_t)
                                    ac3_eac3_gaq_remap_2_4_a[idx][log_gain - 1]
                                    * m) >> 15) + b;
                    mant = m;
                } else {
                    mant = (int32_t)((uint32_t)mant << (24 - bits));
                    if (!log_gain)
                        mant += (int32_t)(((int64_t)
                                           ac3_eac3_gaq_remap_1[idx] * mant)
                                          >> 15);
                }
                pre[bin][blk] = mant;
            }
        }
        idct6(pre[bin]);
    }
    return br->error ? -1 : 0;
}

/* Convert an AHT pre-mantissa to the float coefficient scale used here. */
static float aht_coeff(int32_t mant, int exp)
{
    return (float)(mant >> exp) * (1.0f / 8388608.0f);
}

/* ------------------------------------------------------------------ */
/* band structures                                                     */
/* ------------------------------------------------------------------ */

static void parse_band_structure(ac3_bits_t *br, int blk, int eac3,
                                 int start_sub, int end_sub,
                                 const uint8_t *def, int def_size,
                                 uint8_t *band_struct, int struct_size,
                                 int *num_bands, uint8_t *band_sizes)
{
    uint8_t sizes[AC3_MAX_CPL_SUBBANDS];
    int nsub = end_sub - start_sub;
    int sub, bnd, n;

    (void)struct_size;
    if (!blk)
        memcpy(band_struct, def, (size_t)def_size);
    if (!eac3 || ac3_bits_get(br, 1)) {
        for (sub = 0; sub < nsub - 1; sub++)
            band_struct[start_sub + 1 + sub] = (uint8_t)ac3_bits_get(br, 1);
    }

    n = nsub;
    sizes[0] = 12;
    for (bnd = 0, sub = 1; sub < nsub; sub++) {
        if (band_struct[start_sub + sub]) {
            n--;
            sizes[bnd] += 12;
        } else {
            sizes[++bnd] = 12;
        }
    }
    *num_bands = n;
    memcpy(band_sizes, sizes, (size_t)n);
}

/* ------------------------------------------------------------------ */
/* spectral extension                                                  */
/* ------------------------------------------------------------------ */

static int parse_spx_strategy(ac3_codec_t *c, ac3_header_t *h, ac3_bits_t *br,
                              int blk)
{
    int dst_start, src_start, dst_end, start_sub, end_sub;

    if (h->acmod == 1) {
        h->channel_uses_spx[1] = 1;
    } else {
        int ch;

        for (ch = 1; ch <= h->nfchans; ch++)
            h->channel_uses_spx[ch] = (int)ac3_bits_get(br, 1);
    }

    dst_start = (int)ac3_bits_get(br, 2);
    start_sub = (int)ac3_bits_get(br, 3) + 2;
    if (start_sub > 7)
        start_sub += start_sub - 7;
    end_sub = (int)ac3_bits_get(br, 3) + 5;
    if (end_sub > 7)
        end_sub += end_sub - 7;

    dst_start = dst_start * 12 + 25;
    src_start = start_sub * 12 + 25;
    dst_end = end_sub * 12 + 25;

    if (start_sub >= end_sub || dst_start >= src_start)
        return -1;

    c->spx_dst_start = dst_start;
    c->spx_src_start = src_start;
    c->spx_dst_end = dst_end;

    parse_band_structure(br, blk, 1, start_sub, end_sub,
                         ac3_eac3_default_spx_band_struct, 17,
                         c->spx_band_struct, 17,
                         &c->num_spx_bands, c->spx_band_sizes);
        return 0;
}

static void parse_spx_coords(ac3_codec_t *c, ac3_header_t *h, ac3_bits_t *br)
{
    int ch, bnd;

    for (ch = 1; ch <= h->nfchans; ch++) {
        if (!h->channel_uses_spx[ch]) {
            c->first_spx_coords[ch - 1] = 1;
            continue;
        }
        if (c->first_spx_coords[ch - 1] || ac3_bits_get(br, 1)) {
            float blend = (float)ac3_bits_get(br, 5) * (1.0f / 32.0f);
            int master = 3 * (int)ac3_bits_get(br, 2);
            int bin = c->spx_src_start;

            c->first_spx_coords[ch - 1] = 0;
            for (bnd = 0; bnd < c->num_spx_bands; bnd++) {
                int bandsize = c->spx_band_sizes[bnd];
                float nratio, nblend, sblend, coord;
                int cexp, cmant;

                nratio = ((float)(bin + (bandsize >> 1))) /
                         (float)c->spx_dst_end - blend;
                if (nratio < 0.0f)
                    nratio = 0.0f;
                else if (nratio > 1.0f)
                    nratio = 1.0f;
                nblend = sqrtf(3.0f * nratio);
                sblend = sqrtf(1.0f - nratio);
                bin += bandsize;

                cexp = (int)ac3_bits_get(br, 4);
                cmant = (int)ac3_bits_get(br, 2);
                if (cexp == 15)
                    cmant <<= 1;
                else
                    cmant += 4;
                cmant <<= (25 - cexp - master);
                coord = (float)cmant * (1.0f / (float)(1 << 23));
                c->spx_noise_blend[ch - 1][bnd] = nblend * coord;
                c->spx_signal_blend[ch - 1][bnd] = sblend * coord;
            }
        }
    }
}

static void apply_spx(ac3_codec_t *c, const ac3_header_t *h)
{
    uint8_t wrapflag[17] = {1, 0};
    uint8_t copy_sizes[17];
    float rms_energy[17];
    int num_copy = 0;
    int ch, bnd, i, bin;

    /* build the copy-index mapping and mark wrap points */
    bin = c->spx_dst_start;
    for (bnd = 0; bnd < c->num_spx_bands; bnd++) {
        int bandsize = c->spx_band_sizes[bnd];
        int copysize = 1;

        if (bin + bandsize > c->spx_src_start) {
            copy_sizes[num_copy++] = (uint8_t)(bin - c->spx_dst_start);
            bin = c->spx_dst_start;
            wrapflag[bnd] = 1;
        }
        for (i = 0; i < bandsize; i += copysize) {
            if (bin == c->spx_src_start) {
                copy_sizes[num_copy++] = (uint8_t)(bin - c->spx_dst_start);
                bin = c->spx_dst_start;
            }
            copysize = AC3_MIN(bandsize - i, c->spx_src_start - bin);
            bin += copysize;
        }
    }
    copy_sizes[num_copy++] = (uint8_t)(bin - c->spx_dst_start);

    for (ch = 0; ch < h->nfchans; ch++) {
        float *coeff = c->ch[ch].coeff;

        if (!h->channel_uses_spx[ch + 1])
            continue;

        /* copy low-band coefficients into the extension region */
        bin = c->spx_src_start;
        for (i = 0; i < num_copy; i++) {
            memcpy(&coeff[bin], &coeff[c->spx_dst_start],
                   (size_t)copy_sizes[i] * sizeof(float));
            bin += copy_sizes[i];
        }

        /* RMS energy of each extension band */
        bin = c->spx_src_start;
        for (bnd = 0; bnd < c->num_spx_bands; bnd++) {
            int bandsize = c->spx_band_sizes[bnd];
            float acc = 0.0f;

            for (i = 0; i < bandsize; i++) {
                float v = coeff[bin++];

                acc += v * v;
            }
            rms_energy[bnd] = sqrtf(acc / (float)bandsize);
        }

        /* notch filter at wrap points */
        if (h->spx_atten_code[ch + 1] >= 0) {
            const float *atten =
                ac3_eac3_spx_atten_tab[h->spx_atten_code[ch + 1]];

            bin = c->spx_src_start - 2;
            for (bnd = 0; bnd < c->num_spx_bands; bnd++) {
                if (wrapflag[bnd]) {
                    coeff[bin] *= atten[0];
                    coeff[bin + 1] *= atten[1];
                    coeff[bin + 2] *= atten[2];
                    coeff[bin + 3] *= atten[1];
                    coeff[bin + 4] *= atten[0];
                }
                bin += c->spx_band_sizes[bnd];
            }
        }

        /* blend copied signal with noise, scaled by the coordinates */
        bin = c->spx_src_start;
        for (bnd = 0; bnd < c->num_spx_bands; bnd++) {
            float nscale = c->spx_noise_blend[ch][bnd] *
                           rms_energy[bnd] * (1.0f / -2147483648.0f);
            float sscale = c->spx_signal_blend[ch][bnd];

            for (i = 0; i < c->spx_band_sizes[bnd]; i++) {
                float noise = nscale * (float)(int32_t)lfg_get(c);

                if (!c->dither_enabled)
                    noise = 0.0f;
                coeff[bin] = coeff[bin] * sscale + noise;
                bin++;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* coupling                                                            */
/* ------------------------------------------------------------------ */

static int parse_dba_segments(ac3_bits_t *br, int *nsegs,
                              uint8_t *offsets, uint8_t *lengths,
                              uint8_t *values)
{
    int seg, n;

    n = (int)ac3_bits_get(br, 3) + 1;
    for (seg = 0; seg < n; seg++) {
        offsets[seg] = (uint8_t)ac3_bits_get(br, 5);
        lengths[seg] = (uint8_t)ac3_bits_get(br, 4);
        values[seg] = (uint8_t)ac3_bits_get(br, 3);
    }
    *nsegs = n;
    return br->error ? -1 : 0;
}

static int parse_coupling_strategy(ac3_codec_t *c, ac3_header_t *h,
                                   ac3_bits_t *br, int blk)
{
    int acmod = h->acmod;
    int i, start_sub, end_sub, nsub, nbnd = 0;

    if (!h->cpl_in_use[blk]) {
        c->chincpl = 0;
        c->phsflginu = 0;
        for (i = 0; i < AC3_MAX_FBW; i++)
            c->first_cpl_coords[i] = 1;
        c->first_cpl_leak = 1;
        c->cpl_nbnd = 0;
        c->cplbndstrc = 0;
        return 0;
    }

    if (ac3_bits_get(br, 1))       /* enhanced coupling */
        return EAC3_UNSUPPORTED;

    if (acmod == 2) {
        c->chincpl = 3;
    } else {
        c->chincpl = 0;
        for (i = 0; i < h->nfchans; i++)
            c->chincpl |= (int)ac3_bits_get(br, 1) << i;
        if (!c->chincpl)
            return -1;
    }

    if (acmod == 2)
        c->phsflginu = (int)ac3_bits_get(br, 1);
    else
        c->phsflginu = 0;

    start_sub = (int)ac3_bits_get(br, 4);
    if (h->spx_in_use)
        end_sub = (c->spx_src_start - 37) / 12;
    else
        end_sub = (int)ac3_bits_get(br, 4) + 3;
    if (start_sub >= end_sub)
        return -1;

    c->cpl_nsub = nsub = end_sub - start_sub;
    c->cpl_strtbnd = ac3_cpl_bndtab[start_sub];
    c->cpl_start = start_sub * 12 + 37;
    c->cpl_end = end_sub * 12 + 37;

    if (blk == 0) {
        for (i = 0; i < AC3_MAX_CPL_SUBBANDS; i++)
            c->cpl_band_struct[i] = ac3_eac3_default_cpl_band_struct[i];
    }
    if (ac3_bits_get(br, 1)) {
        for (i = 0; i < nsub - 1; i++)
            c->cpl_band_struct[start_sub + 1 + i] =
                (uint8_t)ac3_bits_get(br, 1);
    }

    c->cplbndstrc = 0;
    for (i = 0; i < nsub - 1; i++) {
        if (c->cpl_band_struct[start_sub + 1 + i])
            c->cplbndstrc |= 1u << i;
        else
            nbnd++;
    }
    c->cpl_nbnd = nbnd + 1;
    return 0;
}

static int parse_coupling_coords(ac3_codec_t *c, ac3_header_t *h,
                                 ac3_bits_t *br, int blk)
{
    int acmod = h->acmod;
    int i, j, any = 0;

    for (i = 0; i < h->nfchans; i++) {
        if (!(c->chincpl & (1 << i))) {
            c->first_cpl_coords[i] = 1;
            continue;
        }
        if (c->first_cpl_coords[i] || ac3_bits_get(br, 1)) {
            int mstr = 3 * (int)ac3_bits_get(br, 2);

            c->first_cpl_coords[i] = 0;
            any = 1;
            for (j = 0; j < c->cpl_nbnd; j++) {
                int cplcoexp = (int)ac3_bits_get(br, 4);
                int cplcomant = (int)ac3_bits_get(br, 4);

                if (cplcoexp == 15)
                    c->cplco[i][j] = (float)cplcomant *
                                     (float)ldexp(1.0, -(16 + mstr));
                else
                    c->cplco[i][j] = (float)(cplcomant | 0x10) *
                                     (float)ldexp(1.0, -2 - cplcoexp - mstr);
            }
        } else if (blk == 0) {
            return -1;
        }
    }

    if (acmod == 2 && any) {
        if (c->phsflginu) {
            for (j = 0; j < c->cpl_nbnd; j++)
                c->cpl_phase[j] = (int)ac3_bits_get(br, 1);
        } else {
            for (j = 0; j < c->cpl_nbnd; j++)
                c->cpl_phase[j] = 0;
        }
    }
    return 0;
}

/*
 * Decode the coupling channel for one block and fold it into the coupled
 * channels.  Dither is generated for the coupling channel itself (and then
 * removed for channels whose dither flag is clear), matching the reference
 * behaviour and keeping the noise generator in step.
 */
static int decode_coupling_block(ac3_codec_t *c, ac3_header_t *h,
                                 ac3_bits_t *br, ac3_quant_t *q,
                                 const int dithflag[AC3_MAX_FBW], int blk)
{
    int use_aht = h->channel_uses_aht[0];
    float coord[AC3_MAX_FBW];
    float cpl_coeff[AC3_COEFFS];
    int band = 0;
    int i = c->cpl_start;
    uint32_t strc = c->cplbndstrc;
    int ch;

    while (i < c->cpl_end) {
        int i_end = i + 12;

        while (strc & 1) {
            strc >>= 1;
            i_end += 12;
        }
        strc >>= 1;
        if (i_end > c->cpl_end)
            i_end = c->cpl_end;

        for (ch = 0; ch < h->nfchans; ch++) {
            if (c->chincpl & (1 << ch)) {
                coord[ch] = c->cplco[ch][band];
                if (ch == 1 && h->acmod == 2 && c->cpl_phase[band])
                    coord[ch] = -coord[ch];
            } else {
                coord[ch] = 0.0f;
            }
        }

        for (; i < i_end; i++) {
            int bap = c->cpl_bap[i];

            if (use_aht) {
                cpl_coeff[i] = aht_coeff(c->pre_mant[0][i][blk],
                                         c->cpl_exp[i]);
            } else if (bap == 0) {
                cpl_coeff[i] = ac3_eac3_dither_coeff(c, c->cpl_exp[i]);
                if (!c->dither_enabled)
                    cpl_coeff[i] = 0.0f;
            } else {
                cpl_coeff[i] = ac3_quant_coeff(br, q, bap, c->cpl_exp[i]);
            }

            for (ch = 0; ch < h->nfchans; ch++) {
                if (!(c->chincpl & (1 << ch)))
                    continue;
                if (bap == 0 && !dithflag[ch])
                    c->ch[ch].coeff[i] = 0.0f;
                else
                    c->ch[ch].coeff[i] = cpl_coeff[i] * coord[ch];
            }
        }
        band++;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* audio block                                                         */
/* ------------------------------------------------------------------ */

int ac3_decode_block_eac3(ac3_codec_t *c, ac3_header_t *h, ac3_bits_t *br,
                          int blk)
{
    int nfchans = h->nfchans;
    int lfe = h->lfeon;
    int cpl_in_use;
    int i, j, ch;
    int blksw[AC3_MAX_FBW] = {0};
    int dithflag[AC3_MAX_FBW] = {0};
    ac3_quant_t quant;
    int ret;

    for (ch = 0; ch < AC3_MAX_CHANNELS; ch++)
        memset(c->ch[ch].coeff, 0, sizeof(c->ch[ch].coeff));
    if (lfe)
        c->ch[5].endmant = 7;

    /* Block switch flags. */
    if (h->block_switch_syntax) {
        for (i = 0; i < nfchans; i++)
            blksw[i] = (int)ac3_bits_get(br, 1);
    }

    /* Dither flags. */
    if (h->dither_flag_syntax) {
        for (i = 0; i < nfchans; i++)
            dithflag[i] = (int)ac3_bits_get(br, 1) && c->dither_enabled;
    } else {
        for (i = 0; i < nfchans; i++)
            dithflag[i] = c->dither_enabled;
    }

    /* Dynamic range control. */
    {
        int nwords = (h->acmod == 0) ? 2 : 1;

        for (i = 0; i < nwords; i++) {
            if (ac3_bits_get(br, 1)) {
                int word = (int)ac3_bits_get_signed(br, 8);
                float g = ac3_apply_drc(c, ac3_drc_word_to_gain(word));

                if (h->acmod == 0) {
                    c->dynrng[i] = g;
                } else {
                    for (ch = 0; ch < nfchans; ch++)
                        c->dynrng[ch] = g;
                }
            } else if (blk == 0) {
                if (h->acmod == 0) {
                    c->dynrng[i] = 1.0f;
                } else {
                    for (ch = 0; ch < nfchans; ch++)
                        c->dynrng[ch] = 1.0f;
                }
            }
        }
        c->dynrng[5] = c->dynrng[0];
    }

    /* Spectral extension strategy. */
    if (!blk || ac3_bits_get(br, 1)) {
        h->spx_in_use = (int)ac3_bits_get(br, 1);
        if (h->spx_in_use) {
            ret = parse_spx_strategy(c, h, br, blk);
            if (ret < 0)
                return ret;
        }
    }
    if (!h->spx_in_use) {
        for (i = 0; i < AC3_MAX_CHANNELS; i++) {
            h->channel_uses_spx[i] = 0;
            c->first_spx_coords[i] = 1;
        }
    }
    if (h->spx_in_use)
        parse_spx_coords(c, h, br);

    /* Coupling strategy and coordinates. */
    if (h->cpl_strategy_exists[blk]) {
        ret = parse_coupling_strategy(c, h, br, blk);
        if (ret < 0)
            return ret;
    }
    cpl_in_use = (c->chincpl != 0);
    if (cpl_in_use) {
        ret = parse_coupling_coords(c, h, br, blk);
        if (ret < 0)
            return ret;
    }

    /* Rematrixing strategy. */
    if (h->acmod == 2) {
        if (!blk || ac3_bits_get(br, 1)) {
            int nb = 4;

            if (cpl_in_use && c->cpl_start <= 61)
                nb -= 1 + (c->cpl_start == 37);
            else if (h->spx_in_use && c->spx_src_start <= 61)
                nb--;
            c->rematflg = 0;
            for (i = 0; i < nb; i++)
                c->rematflg |= (int)ac3_bits_get(br, 1) << i;
        }
    }

    /* Channel bandwidths for channels with new exponents. */
    for (i = 0; i < nfchans; i++) {
        int strat = h->exp_strategy[blk][i + 1];

        if (strat != 0) {
            if (c->chincpl & (1 << i)) {
                c->ch[i].endmant = c->cpl_start;
            } else if (h->spx_in_use && h->channel_uses_spx[i + 1]) {
                c->ch[i].endmant = c->spx_src_start;
            } else {
                int chbwcod = (int)ac3_bits_get(br, 6);

                if (chbwcod > 60)
                    return -1;
                c->ch[i].endmant = chbwcod * 3 + 73;
            }
        }
    }

    /* Decode exponents: the coupling channel comes first, then the
     * full-band channels, then the LFE channel. */
    if (cpl_in_use && h->exp_strategy[blk][0] != 0) {
        int strat = h->exp_strategy[blk][0];
        int absexp = (int)ac3_bits_get(br, 4) << 1;
        int grp = 3 << (strat - 1);
        int ngrp = (c->cpl_end - c->cpl_start) / grp;

        if (ac3_parse_exponents(br, strat, ngrp, absexp,
                                c->cpl_exp + c->cpl_start) < 0)
            return -1;
    }
    for (i = 0; i < nfchans; i++) {
        int strat = h->exp_strategy[blk][i + 1];

        if (strat != 0) {
            int absexp = (int)ac3_bits_get(br, 4);
            int grp = 3 << (strat - 1);
            int ngrp = (c->ch[i].endmant + grp - 4) / grp;

            c->ch[i].exp[0] = (uint8_t)absexp;
            if (ac3_parse_exponents(br, strat, ngrp, absexp,
                                    c->ch[i].exp + 1) < 0)
                return -1;
            ac3_bits_skip(br, 2);      /* gainrng */
        }
    }
    if (lfe && h->lfe_exp_strategy[blk] != 0) {
        int absexp = (int)ac3_bits_get(br, 4);

        c->ch[5].exp[0] = (uint8_t)absexp;
        if (ac3_parse_exponents(br, 1, 2, absexp, c->ch[5].exp + 1) < 0)
            return -1;
        c->ch[5].endmant = 7;
    }

    /* Bit allocation information. */
    if (h->bit_allocation_syntax) {
        if (ac3_bits_get(br, 1)) {
            h->slow_decay = (int)ac3_slow_decay_tab[ac3_bits_get(br, 2)]
                            >> h->sr_shift;
            h->fast_decay = (int)ac3_fast_decay_tab[ac3_bits_get(br, 2)]
                            >> h->sr_shift;
            h->slow_gain = ac3_slow_gain_tab[ac3_bits_get(br, 2)];
            h->db_per_bit = ac3_db_per_bit_tab[ac3_bits_get(br, 2)];
            h->floor = ac3_floor_tab[ac3_bits_get(br, 3)];
        } else if (blk == 0) {
            return -1;
        }
    }

    /* SNR offsets (block 0) and fast gains. */
    if (blk == 0 && h->snr_offset_strategy && ac3_bits_get(br, 1)) {
        int csnr = ((int)ac3_bits_get(br, 6) - 15) << 4;
        int snr = 0;
        int first = cpl_in_use ? 0 : 1;

        if (cpl_in_use) {
            if (h->snr_offset_strategy == 2 || first == 0)
                snr = (csnr + (int)ac3_bits_get(br, 4)) << 2;
            h->cpl_snr_offset = snr;
        }
        for (i = 0; i < nfchans; i++) {
            if (h->snr_offset_strategy == 2 || i + 1 == first)
                snr = (csnr + (int)ac3_bits_get(br, 4)) << 2;
            h->snr_offset[i] = snr;
        }
        if (lfe) {
            if (h->snr_offset_strategy == 2 || nfchans + 1 == first)
                snr = (csnr + (int)ac3_bits_get(br, 4)) << 2;
            h->snr_offset[nfchans] = snr;
        }
    }

    if (h->fast_gain_syntax && ac3_bits_get(br, 1)) {
        if (cpl_in_use)
            h->cpl_fast_gain = ac3_fast_gain_tab[ac3_bits_get(br, 3)];
        for (i = 0; i < nfchans; i++)
            h->fast_gain[i] = ac3_fast_gain_tab[ac3_bits_get(br, 3)];
        if (lfe)
            h->fast_gain[nfchans] = ac3_fast_gain_tab[ac3_bits_get(br, 3)];
    } else if (blk == 0) {
        int g = ac3_fast_gain_tab[4];

        h->cpl_fast_gain = g;
        for (i = 0; i < nfchans; i++)
            h->fast_gain[i] = g;
        if (lfe)
            h->fast_gain[nfchans] = g;
    }

    /* E-AC-3 to AC-3 converter SNR offset. */
    if (h->frame_type == 0 && ac3_bits_get(br, 1))
        ac3_bits_skip(br, 10);

    /* Coupling leak. */
    if (cpl_in_use) {
        if (c->first_cpl_leak || ac3_bits_get(br, 1)) {
            h->cpl_fast_leak = (int)ac3_bits_get(br, 3);
            h->cpl_slow_leak = (int)ac3_bits_get(br, 3);
            c->first_cpl_leak = 0;
        }
    }

    /* Delta bit allocation. */
    if (h->dba_syntax && ac3_bits_get(br, 1)) {
        if (cpl_in_use) {
            h->cpl_dba_mode = (int)ac3_bits_get(br, 2);
            if (h->cpl_dba_mode == 3)
                return -1;
        }
        for (i = 0; i < nfchans; i++) {
            h->dba_mode[i] = (int)ac3_bits_get(br, 2);
            if (h->dba_mode[i] == 3)
                return -1;
        }
        if (cpl_in_use && h->cpl_dba_mode == 2) {
            if (parse_dba_segments(br, &h->cpl_dba_nsegs, h->cpl_dba_offsets,
                                   h->cpl_dba_lengths, h->cpl_dba_values) < 0)
                return -1;
        }
        for (i = 0; i < nfchans; i++) {
            if (h->dba_mode[i] == 2) {
                if (parse_dba_segments(br, &h->dba_nsegs[i],
                                       h->dba_offsets[i], h->dba_lengths[i],
                                       h->dba_values[i]) < 0)
                    return -1;
            }
        }
    } else if (blk == 0) {
        h->cpl_dba_mode = 0;
        for (i = 0; i < nfchans; i++)
            h->dba_mode[i] = 0;
    }

    /* Bit allocation. */
    for (i = 0; i < nfchans; i++) {
        ret = allocate_channel(h, c->ch[i].exp, 0, c->ch[i].endmant,
                               h->snr_offset[i], h->fast_gain[i],
                               h->dba_mode[i], h->dba_nsegs[i],
                               h->dba_offsets[i], h->dba_lengths[i],
                               h->dba_values[i], h->channel_uses_aht[i + 1],
                               c->ch[i].bap, 0);
        if (ret < 0)
            return -1;
    }
    if (lfe) {
        ret = allocate_channel(h, c->ch[5].exp, 0, 7,
                               h->snr_offset[nfchans], h->fast_gain[nfchans],
                               0, 0, NULL, NULL, NULL,
                               h->channel_uses_aht[nfchans + 1],
                               c->ch[5].bap, 1);
        if (ret < 0)
            return -1;
    }
    if (cpl_in_use) {
        ret = allocate_channel(h, c->cpl_exp, c->cpl_start, c->cpl_end,
                               h->cpl_snr_offset, h->cpl_fast_gain,
                               h->cpl_dba_mode, h->cpl_dba_nsegs,
                               h->cpl_dba_offsets, h->cpl_dba_lengths,
                               h->cpl_dba_values, h->channel_uses_aht[0],
                               c->cpl_bap, 0);
        if (ret < 0)
            return -1;
    }

                if (h->skip_syntax && ac3_bits_get(br, 1)) {
        int skipl = (int)ac3_bits_get(br, 9);

        ac3_bits_skip(br, skipl * 8);
    }

    if (br->error)
        return -1;

    /* Mantissas.  AHT channels carry all six blocks in block 0. */
    quant.q1_ptr = quant.q2_ptr = quant.q4_ptr = -1;
    quant.q4 = 0.0f;
    {
        int done_cpl = 0;

        for (i = 0; i < nfchans; i++) {
            int coupled = (c->chincpl & (1 << i)) != 0;
            int use_aht = h->channel_uses_aht[i + 1];

            if (use_aht) {
                if (blk == 0) {
                    ret = decode_aht_channel(c, br, c->ch[i].bap, 0,
                                             c->ch[i].endmant,
                                             c->pre_mant[i + 1]);
                    if (ret < 0)
                        return ret;
                }
                for (j = 0; j < c->ch[i].endmant; j++)
                    c->ch[i].coeff[j] = aht_coeff(c->pre_mant[i + 1][j][blk],
                                                  c->ch[i].exp[j]);
            } else {
                ac3_decode_channel(c, &c->ch[i], br, &quant, dithflag[i],
                                   c->ch[i].endmant);
            }

            if (coupled && !done_cpl) {
                done_cpl = 1;
                if (h->channel_uses_aht[0]) {
                    if (blk == 0) {
                        ret = decode_aht_channel(c, br, c->cpl_bap,
                                                 c->cpl_start, c->cpl_end,
                                                 c->pre_mant[0]);
                        if (ret < 0)
                            return ret;
                    }
                }
                ret = decode_coupling_block(c, h, br, &quant, dithflag, blk);
                if (ret < 0)
                    return ret;
            }

            j = coupled ? c->cpl_end : c->ch[i].endmant;
            for (; j < AC3_COEFFS; j++)
                c->ch[i].coeff[j] = 0.0f;
        }
    }
    if (lfe) {
        int use_aht = h->channel_uses_aht[nfchans + 1];

        if (use_aht) {
            if (blk == 0) {
                ret = decode_aht_channel(c, br, c->ch[5].bap, 0, 7,
                                         c->pre_mant[nfchans + 1]);
                if (ret < 0)
                    return ret;
            }
            for (j = 0; j < 7; j++)
                c->ch[5].coeff[j] = aht_coeff(c->pre_mant[nfchans + 1][j][blk],
                                              c->ch[5].exp[j]);
        } else {
            ac3_decode_channel(c, &c->ch[5], br, &quant, 0, 7);
        }
        for (j = 7; j < AC3_COEFFS; j++)
            c->ch[5].coeff[j] = 0.0f;
    }

    if (br->error)
        return -1;

                if (h->acmod == 2)
        ac3_apply_rematrix(c);

    for (i = 0; i < nfchans; i++) {
        float g = c->dynrng[i];

        if (g != 1.0f) {
            for (j = 0; j < AC3_COEFFS; j++)
                c->ch[i].coeff[j] *= g;
        }
    }
    if (lfe) {
        float g = c->dynrng[5];

        if (g != 1.0f) {
            for (j = 0; j < AC3_COEFFS; j++)
                c->ch[5].coeff[j] *= g;
        }
    }

    if (h->spx_in_use)
        apply_spx(c, h);

    for (i = 0; i < nfchans; i++)
        c->blksw[i] = blksw[i];
    return 0;
}





