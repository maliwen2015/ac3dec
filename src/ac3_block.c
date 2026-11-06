/*
 * AC-3 audio block: side information, exponents, bit allocation,
 * mantissas, coupling and rematrixing.
 */
#include "ac3_internal.h"

#define AC3_FAIL() (-1)

float ac3_scale_for_exp(int exp)
{
    if (exp < 0)
        exp = 0;
    else if (exp > 24)
        exp = 24;
    return ac3_scale_factor[exp];
}

float ac3_drc_word_to_gain(int word)
{
    int top = word >> 5;          /* arithmetic shift: -4..3 */
    int low = word & 0x1f;

    return (float)(low | 0x20) * (float)ldexp(1.0, top - 5);
}

float ac3_apply_drc(ac3_codec_t *c, float gain)
{
    if (!c->drc_enabled)
        return 1.0f;
    if (c->drc_scale == 1.0f || gain == 1.0f)
        return gain;
    return (float)pow((double)gain, (double)c->drc_scale);
}

int ac3_parse_exponents(ac3_bits_t *br, int expstr, int ngrps,
                        int exponent, uint8_t *dest)
{
    int rep = 1 << (expstr - 1);
    int g;

    for (g = 0; g < ngrps; g++) {
        int code = (int)ac3_bits_get(br, 7);
        int d1, d2, d3;
        int k;

        d1 = (code < 25) ? -2 :
             (code < 50) ? -1 :
             (code < 75) ? 0 :
             (code < 100) ? 1 :
             (code < 125) ? 2 : 25;
        d2 = (code % 25) / 5 - 2;
        d3 = (code % 5) - 2;

        exponent += d1;
        if (exponent < 0 || exponent > 24)
            return AC3_FAIL();
        for (k = 0; k < rep; k++)
            *dest++ = (uint8_t)exponent;

        exponent += d2;
        if (exponent < 0 || exponent > 24)
            return AC3_FAIL();
        for (k = 0; k < rep; k++)
            *dest++ = (uint8_t)exponent;

        exponent += d3;
        if (exponent < 0 || exponent > 24)
            return AC3_FAIL();
        for (k = 0; k < rep; k++)
            *dest++ = (uint8_t)exponent;
    }
    return 0;
}

int ac3_parse_delta(ac3_bits_t *br, int8_t *deltba)
{
    int nseg, seg, j;

    memset(deltba, 0, AC3_MAX_BANDS);
    nseg = (int)ac3_bits_get(br, 3);
    j = 0;
    for (seg = 0; seg <= nseg; seg++) {
        int offset = (int)ac3_bits_get(br, 5);
        int len = (int)ac3_bits_get(br, 4);
        int value = (int)ac3_bits_get(br, 3);
        int delta;

        j += offset;
        delta = value - ((value >= 4) ? 3 : 4);
        if (len == 0)
            continue;
        if (j + len > AC3_MAX_BANDS)
            return AC3_FAIL();
        while (len--)
            deltba[j++] = (int8_t)delta;
    }
    if (br->error)
        return AC3_FAIL();
    return 0;
}

/*
 * Quantize one mantissa.  The bap values are the raw table entries: 0 means
 * no bits, -1 (3 levels, 5 bits), -2 (5 levels, 7 bits), -3 (11 levels,
 * 7 bits per pair), 3 (7 levels, 3 bits), 4 (15 levels, 4 bits), 5..16
 * (signed two's complement of bap bits).  The negative codes carry both
 * the quantizer and the fact that one code word covers several mantissas.
 */
float ac3_quant_coeff(ac3_bits_t *br, ac3_quant_t *q, int bap, int exp)
{
    float v;
    float scale = ac3_scale_for_exp(exp);

    /* The grouped quantizers below produce a normalised value in [-1, 1].
     * A/52 stores mantissas on a 16-bit scale, so bring them up to the same
     * 2^15 range as the signed two's-complement quantizers (bap 5..16). */
    switch (bap) {
    case 0:
        return 0.0f;

    case -1:
        if (q->q1_ptr >= 0) {
            v = q->q1[q->q1_ptr--];
        } else {
            int code = (int)ac3_bits_get(br, 5);
            int d0 = code / 9;
            int d1 = (code / 3) % 3;
            int d2 = code % 3;

            if (code >= 27) {
                q->q1[0] = q->q1[1] = 0.0f;
                v = 0.0f;
            } else {
                q->q1[0] = ((float)d2 - 1.0f) * (2.0f / 3.0f);
                q->q1[1] = ((float)d1 - 1.0f) * (2.0f / 3.0f);
                v = ((float)d0 - 1.0f) * (2.0f / 3.0f);
            }
            q->q1_ptr = 1;
        }
        break;

    case -2:
        if (q->q2_ptr >= 0) {
            v = q->q2[q->q2_ptr--];
        } else {
            int code = (int)ac3_bits_get(br, 7);
            int d0 = code / 25;
            int d1 = (code / 5) % 5;
            int d2 = code % 5;

            if (code >= 125) {
                q->q2[0] = q->q2[1] = 0.0f;
                v = 0.0f;
            } else {
                q->q2[0] = ((float)d2 - 2.0f) * (2.0f / 5.0f);
                q->q2[1] = ((float)d1 - 2.0f) * (2.0f / 5.0f);
                v = ((float)d0 - 2.0f) * (2.0f / 5.0f);
            }
            q->q2_ptr = 1;
        }
        break;

    case -3:
        if (q->q4_ptr == 0) {
            q->q4_ptr = -1;
            v = q->q4;
        } else {
            int code = (int)ac3_bits_get(br, 7);
            int d0 = code / 11;
            int d1 = code % 11;

            if (code >= 121) {
                q->q4 = 0.0f;
                v = 0.0f;
            } else {
                q->q4 = ((float)d1 - 5.0f) * (2.0f / 11.0f);
                v = ((float)d0 - 5.0f) * (2.0f / 11.0f);
            }
            q->q4_ptr = 0;
        }
        break;

    case 3: {
        int code = (int)ac3_bits_get(br, 3);
        v = (code < 7) ? ((float)(2 * code - 6) * (1.0f / 7.0f)) : 0.0f;
        break;
    }

    case 4: {
        int code = (int)ac3_bits_get(br, 4);
        v = (code < 15) ? ((float)(2 * code - 14) * (1.0f / 15.0f)) : 0.0f;
        break;
    }

    default:
        if (bap >= 5 && bap <= 16) {
            int32_t s = ac3_bits_get_signed(br, bap);
            int32_t qi = s * (int32_t)(1u << (16 - bap));
            return (float)qi * scale;
        }
        return 0.0f;
    }

    return v * 32768.0f * scale;
}

/* 16-bit LFSR noise generator, scaled by the A/52 3 dB level factor. */
static float dither_next(ac3_codec_t *c)
{
    uint16_t s = (uint16_t)c->dither_state;
    uint16_t ns = (uint16_t)(ac3_dither_step[s >> 8] ^ (uint16_t)(s << 8));

    c->dither_state = ns;
    return (float)(int16_t)ns * 0.70710678f;
}

void ac3_decode_channel(ac3_codec_t *c, ac3_channel_t *ch,
                        ac3_bits_t *br, ac3_quant_t *q,
                        int dither, int end)
{
    int i;

    for (i = 0; i < end; i++) {
        int bap = ch->bap[i];

        if (bap == 0) {
            if (dither)
                if (c->eac3)
                ch->coeff[i] = ac3_eac3_dither_coeff(c, ch->exp[i]);
            else
                ch->coeff[i] = dither_next(c) * ac3_scale_for_exp(ch->exp[i]);
            else
                ch->coeff[i] = 0.0f;
        } else {
            ch->coeff[i] = ac3_quant_coeff(br, q, bap, ch->exp[i]);
        }
    }
}

void ac3_decode_coupling_channel(ac3_codec_t *c, const ac3_header_t *h,
                                 ac3_bits_t *br, ac3_quant_t *q,
                                 const int dithflag[AC3_MAX_FBW])
{
    int nfchans = h->nfchans;
    int band = 0;
    int i = c->cpl_start;
    uint32_t strc = c->cplbndstrc;
    float coord[AC3_MAX_FBW];
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

        for (ch = 0; ch < nfchans; ch++) {
            if (c->chincpl & (1 << ch)) {
                coord[ch] = c->cplco[ch][band];
                /* Phase flags invert the right channel of a 2/0 stream. */
                if (ch == 1 && h->acmod == 2 && c->cpl_phase[band])
                    coord[ch] = -coord[ch];
            } else {
                coord[ch] = 0.0f;
            }
        }

        while (i < i_end) {
            int bap = c->cpl_bap[i];
            int exp = c->cpl_exp[i];

            if (bap == 0) {
                for (ch = 0; ch < nfchans; ch++) {
                    if (c->chincpl & (1 << ch)) {
                        if (dithflag[ch])
                            c->ch[ch].coeff[i] =
                                dither_next(c) *
                                ac3_scale_for_exp(exp) * coord[ch];
                        else
                            c->ch[ch].coeff[i] = 0.0f;
                    }
                }
            } else {
                float v = ac3_quant_coeff(br, q, bap, exp);

                for (ch = 0; ch < nfchans; ch++) {
                    if (c->chincpl & (1 << ch))
                        c->ch[ch].coeff[i] = v * coord[ch];
                }
            }
            i++;
        }
        band++;
    }
}

static int zero_snr_offsets(const ac3_codec_t *c, const ac3_header_t *h)
{
    int i;

    if (c->csnroffst)
        return 0;
    if (c->chincpl && (c->cplba.bai >> 3))
        return 0;
    if (h->lfeon && (c->lfeba.bai >> 3))
        return 0;
    for (i = 0; i < h->nfchans; i++)
        if (c->ba[i].bai >> 3)
            return 0;
    return 1;
}

void ac3_apply_rematrix(ac3_codec_t *c)
{
    int end = AC3_MIN(c->ch[0].endmant, c->ch[1].endmant);
    int j = 13;
    int i = 0;
    int flags = c->rematflg;

    do {
        int band;

        if (!(flags & 1)) {
            flags >>= 1;
            j = ac3_remat_band[i++];
            continue;
        }
        flags >>= 1;
        band = ac3_remat_band[i++];
        if (band > end)
            band = end;
        while (j < band) {
            float tmp0 = c->ch[0].coeff[j];
            float tmp1 = c->ch[1].coeff[j];

            c->ch[0].coeff[j] = tmp0 + tmp1;
            c->ch[1].coeff[j] = tmp0 - tmp1;
            j++;
        }
    } while (j < end);
}

int ac3_decode_block(ac3_codec_t *c, const ac3_header_t *h, ac3_bits_t *br)
{
    int nfchans = h->nfchans;
    int blksw[AC3_MAX_FBW] = {0};
    int dithflag[AC3_MAX_FBW] = {0};
    int chexpstr[AC3_MAX_FBW] = {0};
    int cplexpstr = 0, lfeexpstr = 0;
    int i, j, ch;
    ac3_quant_t quant;

    for (i = 0; i < AC3_MAX_CHANNELS; i++)
        memset(c->ch[i].coeff, 0, sizeof(c->ch[i].coeff));
    if (h->lfeon)
        c->ch[5].endmant = 7;

    for (i = 0; i < nfchans; i++)
        blksw[i] = (int)ac3_bits_get(br, 1);
    for (i = 0; i < nfchans; i++)
        dithflag[i] = (int)ac3_bits_get(br, 1) && c->dither_enabled;

    /* Dynamic range control.  Only one word is present unless the stream
     * uses 1+1 (dual mono), where each channel has its own word. */
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
            } else if (c->drc_rf && h->compr_present[i]) {
                float g = ac3_apply_drc(c, c->compr_gain[i]);

                if (h->acmod == 0) {
                    c->dynrng[i] = g;
                } else {
                    for (ch = 0; ch < nfchans; ch++)
                        c->dynrng[ch] = g;
                }
            }
        }
        c->dynrng[5] = c->dynrng[0];
    }

    /* Coupling strategy. */
    if (ac3_bits_get(br, 1)) {           /* cplstre */
        c->chincpl = 0;
        if (ac3_bits_get(br, 1)) {       /* cplinu */
            int cplbegf, cplendf, ncplsub;

            for (i = 0; i < nfchans; i++)
                c->chincpl |= (int)ac3_bits_get(br, 1) << i;

            if (h->acmod == 0 || h->acmod == 1)
                return AC3_FAIL();

            if (h->acmod == 2)
                c->phsflginu = (int)ac3_bits_get(br, 1);

            cplbegf = (int)ac3_bits_get(br, 4);
            cplendf = (int)ac3_bits_get(br, 4);
            ncplsub = cplendf + 3 - cplbegf;
            if (ncplsub < 1)
                return AC3_FAIL();

            c->cpl_nsub = ncplsub;
            c->cpl_nbnd = ncplsub;
            c->cpl_strtbnd = ac3_cpl_bndtab[cplbegf];
            c->cpl_start = cplbegf * 12 + 37;
            c->cpl_end = cplendf * 12 + 73;
            c->cplbndstrc = 0;

            for (i = 0; i < ncplsub - 1; i++) {
                if (ac3_bits_get(br, 1)) {
                    c->cplbndstrc |= 1u << i;
                    c->cpl_nbnd--;
                }
            }
            if (c->cpl_nbnd < 1)
                return AC3_FAIL();
        }
    }

    /* Coupling coordinates. */
    if (c->chincpl) {
        int any_coord = 0;

        for (i = 0; i < nfchans; i++) {
            if (!(c->chincpl & (1 << i)))
                continue;
            if (ac3_bits_get(br, 1)) {   /* cplcoe */
                int mstr = 3 * (int)ac3_bits_get(br, 2);

                any_coord = 1;
                for (j = 0; j < c->cpl_nbnd; j++) {
                    int cplcoexp = (int)ac3_bits_get(br, 4);
                    int cplcomant = (int)ac3_bits_get(br, 4);
                    float coord;

                    if (cplcoexp == 15)
                        coord = (float)cplcomant *
                                (float)ldexp(1.0, -(16 + mstr));
                    else
                        coord = (float)(cplcomant | 0x10) *
                                (float)ldexp(1.0, -2 - cplcoexp - mstr);
                    c->cplco[i][j] = coord;
                }
            }
        }

        if (h->acmod == 2 && any_coord) {
            if (c->phsflginu) {
                for (j = 0; j < c->cpl_nbnd; j++)
                    c->cpl_phase[j] = (int)ac3_bits_get(br, 1);
            } else {
                for (j = 0; j < c->cpl_nbnd; j++)
                    c->cpl_phase[j] = 0;
            }
        }
    }

    /* Rematrixing. */
    if (h->acmod == 2) {
        if (ac3_bits_get(br, 1)) {       /* rematstr */
            int end = c->chincpl ? c->cpl_start : 253;

            c->rematflg = 0;
            i = 0;
            do {
                c->rematflg |= (int)ac3_bits_get(br, 1) << i;
            } while (ac3_remat_band[i++] < end);
        }
    }

    /* Exponent strategies and channel bandwidths. */
    if (c->chincpl)
        cplexpstr = (int)ac3_bits_get(br, 2);
    for (i = 0; i < nfchans; i++)
        chexpstr[i] = (int)ac3_bits_get(br, 2);
    if (h->lfeon)
        lfeexpstr = (int)ac3_bits_get(br, 1);

    for (i = 0; i < nfchans; i++) {
        if (chexpstr[i] != 0) {
            if (c->chincpl & (1 << i)) {
                c->ch[i].endmant = c->cpl_start;
            } else {
                int chbwcod = (int)ac3_bits_get(br, 6);

                if (chbwcod > 60)
                    return AC3_FAIL();
                c->ch[i].endmant = chbwcod * 3 + 73;
            }
        }
    }

    /* Coupling exponents. */
    if (c->chincpl && cplexpstr != 0) {
        int absexp = (int)ac3_bits_get(br, 4) << 1;
        int grp = 3 << (cplexpstr - 1);
        int ngrp = (c->cpl_end - c->cpl_start) / grp;

        if (ac3_parse_exponents(br, cplexpstr, ngrp, absexp,
                            c->cpl_exp + c->cpl_start) < 0)
            return AC3_FAIL();
    }

    /* Full-bandwidth channel exponents. */
    for (i = 0; i < nfchans; i++) {
        if (chexpstr[i] != 0) {
            int absexp = (int)ac3_bits_get(br, 4);
            int grp = 3 << (chexpstr[i] - 1);
            int ngrp = (c->ch[i].endmant + grp - 4) / grp;

            c->ch[i].exp[0] = (uint8_t)absexp;
            if (ac3_parse_exponents(br, chexpstr[i], ngrp, absexp,
                                c->ch[i].exp + 1) < 0)
                return AC3_FAIL();
            (void)ac3_bits_get(br, 2);   /* gainrng */
        }
    }

    /* LFE exponents. */
    if (h->lfeon && lfeexpstr != 0) {
        int absexp = (int)ac3_bits_get(br, 4);

        c->ch[5].exp[0] = (uint8_t)absexp;
        if (ac3_parse_exponents(br, 1, 2, absexp, c->ch[5].exp + 1) < 0)
            return AC3_FAIL();
        c->ch[5].endmant = 7;
    }

    /* Bit allocation side information. */
    if (ac3_bits_get(br, 1))            /* baie */
        c->bai = (int)ac3_bits_get(br, 11);

    if (ac3_bits_get(br, 1)) {          /* snroffste */
        c->csnroffst = (int)ac3_bits_get(br, 6);
        if (c->chincpl)
            c->cplba.bai = (uint8_t)ac3_bits_get(br, 7);
        for (i = 0; i < nfchans; i++)
            c->ba[i].bai = (uint8_t)ac3_bits_get(br, 7);
        if (h->lfeon)
            c->lfeba.bai = (uint8_t)ac3_bits_get(br, 7);
    }

    if (c->chincpl && ac3_bits_get(br, 1)) {   /* cplleake */
        c->cplfleak = 9 - (int)ac3_bits_get(br, 3);
        c->cplsleak = 9 - (int)ac3_bits_get(br, 3);
    }

    if (ac3_bits_get(br, 1)) {          /* deltbaie */
        int cpl_new = 0;

        if (c->chincpl) {
            c->cplba.deltbae = (uint8_t)ac3_bits_get(br, 2);
            cpl_new = (c->cplba.deltbae == 1);
        }
        for (i = 0; i < nfchans; i++)
            c->ba[i].deltbae = (uint8_t)ac3_bits_get(br, 2);

        if (cpl_new && ac3_parse_delta(br, c->cplba.deltba) < 0)
            return AC3_FAIL();
        for (i = 0; i < nfchans; i++) {
            if (c->ba[i].deltbae == 1 &&
                ac3_parse_delta(br, c->ba[i].deltba) < 0)
                return AC3_FAIL();
        }
    }

    if (ac3_bits_get(br, 1)) {          /* skiple */
        int skipl = (int)ac3_bits_get(br, 9);

        ac3_bits_skip(br, skipl * 8);
    }

    if (br->error)
        return AC3_FAIL();

    /* Bit allocation. */
    if (zero_snr_offsets(c, h)) {
        for (i = 0; i < nfchans; i++)
            memset(c->ch[i].bap, 0, AC3_COEFFS);
        if (c->chincpl)
            memset(c->cpl_bap, 0, AC3_COEFFS);
        if (h->lfeon)
            memset(c->ch[5].bap, 0, AC3_COEFFS);
    } else {
        if (c->chincpl) {
            ac3_bit_allocate(&c->cplba, c->cpl_strtbnd,
                             c->cpl_start, c->cpl_end,
                             c->cplfleak << 8, c->cplsleak << 8,
                             c->bai, c->csnroffst, h->fscod, h->halfrate,
                             c->cpl_exp, c->cpl_bap,
                             c->cplba.deltbae == 2 ? NULL
                                                   : c->cplba.deltba);
        }
        for (i = 0; i < nfchans; i++) {
            ac3_bit_allocate(&c->ba[i], 0, 0, c->ch[i].endmant,
                             0, 0, c->bai, c->csnroffst,
                             h->fscod, h->halfrate,
                             c->ch[i].exp, c->ch[i].bap,
                             c->ba[i].deltbae == 2 ? NULL
                                                   : c->ba[i].deltba);
        }
        if (h->lfeon) {
            c->lfeba.deltbae = 2;
            ac3_bit_allocate(&c->lfeba, 0, 0, 7,
                             0, 0, c->bai, c->csnroffst,
                             h->fscod, h->halfrate,
                             c->ch[5].exp, c->ch[5].bap, NULL);
            c->ch[5].endmant = 7;
        }
    }

    /* Mantissas.  Coupling mantissas are carried in the first coupled
     * channel's segment, in bitstream order. */
    quant.q1_ptr = quant.q2_ptr = quant.q4_ptr = -1;
    quant.q4 = 0.0f;

    {
        int done_cpl = 0;

        for (i = 0; i < nfchans; i++) {
            int coupled = (c->chincpl & (1 << i)) != 0;

            ac3_decode_channel(c, &c->ch[i], br, &quant, dithflag[i],
                           c->ch[i].endmant);

            if (coupled && !done_cpl) {
                done_cpl = 1;
                ac3_decode_coupling_channel(c, h, br, &quant, dithflag);
            }

            j = coupled ? c->cpl_end : c->ch[i].endmant;
            for (; j < AC3_COEFFS; j++)
                c->ch[i].coeff[j] = 0.0f;
        }
    }

    if (h->lfeon) {
        ac3_decode_channel(c, &c->ch[5], br, &quant, 0, 7);
        for (j = 7; j < AC3_COEFFS; j++)
            c->ch[5].coeff[j] = 0.0f;
    }

    if (br->error)
        return AC3_FAIL();

    /* Rematrixing is only defined for 2/0. */
    if (h->acmod == 2)
        ac3_apply_rematrix(c);

    /* Dynamic range gain, after decoupling and rematrixing. */
    for (i = 0; i < nfchans; i++) {
        float g = c->dynrng[i];

        if (g != 1.0f) {
            for (j = 0; j < AC3_COEFFS; j++)
                c->ch[i].coeff[j] *= g;
        }
    }
    if (h->lfeon) {
        float g = c->dynrng[5];

        if (g != 1.0f) {
            for (j = 0; j < AC3_COEFFS; j++)
                c->ch[5].coeff[j] *= g;
        }
    }

    for (i = 0; i < nfchans; i++)
        c->blksw[i] = blksw[i];

    return 0;
}

