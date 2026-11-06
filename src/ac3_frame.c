/*
 * AC-3 and E-AC-3 syncframe header parsing, E-AC-3 BSI parsing and CRC.
 */
#include "ac3_internal.h"

static const int ac3_bitrate_kbps[19] = {
    32, 40, 48, 56, 64, 80, 96, 112, 128, 160,
    192, 224, 256, 320, 384, 448, 512, 576, 640
};

static const int ac3_sample_rates[3] = { 48000, 44100, 32000 };

static const int ac3_halfrate[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3};

static const int eac3_blocks[4] = { 1, 2, 3, 6 };

/* 3-bit E-AC-3 mix level codes to linear gain. */
static const float eac3_gain_levels[8] = {
    1.41421356f, 1.18920712f, 1.00000000f, 0.84089642f,
    0.70710678f, 0.59460356f, 0.50000000f, 0.00000000f
};

static int floor_log2_int(int v)
{
    int n = 0;

    while (v > 1) {
        v >>= 1;
        n++;
    }
    return n;
}

static int frame_geometry(int fscod, int frmsizecod, int bsid,
                          int *frame_size, int *sample_rate, int *bit_rate,
                          int *halfrate)
{
    int half, rate, size;

    if (fscod == 3 || frmsizecod >= 38 || bsid > 10)
        return -1;

    half = ac3_halfrate[bsid];
    rate = ac3_bitrate_kbps[frmsizecod >> 1];
    if (half > 2)
        return -1;

    switch (fscod) {
    case 0:
        *sample_rate = 48000 >> half;
        size = 4 * rate;
        break;
    case 1:
        *sample_rate = 44100 >> half;
        size = 2 * (320 * rate / 147 + (frmsizecod & 1));
        break;
    case 2:
        *sample_rate = 32000 >> half;
        size = 6 * rate;
        break;
    default:
        return -1;
    }

    *bit_rate = (rate * 1000) >> half;
    *frame_size = size;
    *halfrate = half;
    return 0;
}

static int eac3_probe_frame_size(const uint8_t *data, int size)
{
    int bsid, frame_type, frame_size, fscod, fscod2;

    if (size < 7)
        return 0;
    bsid = data[5] >> 3;
    if (bsid < 11 || bsid > 16)
        return 0;
    frame_type = data[2] >> 6;
    if (frame_type == 3)
        return 0;
    frame_size = ((((data[2] & 0x07) << 8) | data[3]) + 1) * 2;
    if (frame_size < 7 || frame_size > AC3_MAX_EAC3_FRAME_BYTES)
        return 0;
    fscod = data[4] >> 6;
    if (fscod == 3) {
        fscod2 = (data[4] >> 4) & 3;
        if (fscod2 == 3)
            return 0;
    }
    return frame_size;
}

int ac3_probe_frame_size(const uint8_t *data, int size)
{
    int frame_size, sample_rate, bit_rate, half;

    if (size < 7)
        return 0;
    if (data[0] != 0x0b || data[1] != 0x77)
        return 0;
    if ((data[5] >> 3) > 10)
        return eac3_probe_frame_size(data, size);
    if (frame_geometry(data[4] >> 6, data[4] & 0x3f, data[5] >> 3,
                       &frame_size, &sample_rate, &bit_rate, &half) < 0)
        return 0;
    if (frame_size < 128 || frame_size > AC3_MAX_FRAME_BYTES)
        return 0;
    return frame_size;
}

static int parse_ac3_header(const uint8_t *data, int size, ac3_header_t *h)
{
    ac3_bits_t b;
    int fscod, frmsizecod, bsid;
    int i, chaninfo, nchaninfo;

    ac3_bits_init(&b, data, size);
    if (ac3_bits_get(&b, 16) != 0x0b77)
        return -1;
    (void)ac3_bits_get(&b, 16);            /* crc1 */

    fscod = (int)ac3_bits_get(&b, 2);
    frmsizecod = (int)ac3_bits_get(&b, 6);
    bsid = (int)ac3_bits_get(&b, 5);
    h->bsmod = (int)ac3_bits_get(&b, 3);

    if (frame_geometry(fscod, frmsizecod, bsid, &h->frame_size,
                       &h->sample_rate, &h->bit_rate, &h->halfrate) < 0)
        return -1;
    if (h->frame_size > size)
        return -1;

    h->fscod = fscod;
    h->frmsizecod = frmsizecod;
    h->bsid = bsid;
    h->eac3 = 0;
    h->num_blocks = 6;
    h->samples = AC3_FRAME_SAMPLES;

    h->acmod = (int)ac3_bits_get(&b, 3);
    h->nfchans = ac3_acmod_nch[h->acmod];
    h->dsurmod = 0;
    h->cmix_code = -1;
    h->smix_code = -1;

    if ((h->acmod & 1) && h->acmod != 1)
        h->cmix_code = (int)ac3_bits_get(&b, 2);
    if (h->acmod & 4)
        h->smix_code = (int)ac3_bits_get(&b, 2);
    if (h->acmod == 2)
        h->dsurmod = (int)ac3_bits_get(&b, 2);

    h->lfeon = (int)ac3_bits_get(&b, 1);

    h->channel_map = ac3_acmod_map[h->acmod];
    if (h->lfeon)
        h->channel_map |= AC3_SPEAKER_LFE;

    h->clev = (h->cmix_code >= 0) ? ac3_clev_table[h->cmix_code & 3]
                                  : ac3_clev_table[1];
    h->slev = (h->smix_code >= 0) ? ac3_slev_table[h->smix_code & 3]
                                  : ac3_slev_table[1];

    nchaninfo = (h->acmod == 0) ? 2 : 1;
    for (chaninfo = 0; chaninfo < nchaninfo; chaninfo++) {
        h->dialnorm[chaninfo] = (int)ac3_bits_get(&b, 5);

        h->compr_present[chaninfo] = (int)ac3_bits_get(&b, 1);
        if (h->compr_present[chaninfo])
            h->compr[chaninfo] = ac3_bits_get_signed(&b, 8);

        h->langcod_present[chaninfo] = (int)ac3_bits_get(&b, 1);
        if (h->langcod_present[chaninfo])
            h->langcod[chaninfo] = (int)ac3_bits_get(&b, 8);

        h->audprodie[chaninfo] = (int)ac3_bits_get(&b, 1);
        if (h->audprodie[chaninfo]) {
            h->mixlevel[chaninfo] = (int)ac3_bits_get(&b, 3);
            h->roomtyp[chaninfo] = (int)ac3_bits_get(&b, 2);
        }
    }
    if (nchaninfo == 1) {
        h->dialnorm[1] = h->dialnorm[0];
        h->compr_present[1] = h->compr_present[0];
        h->compr[1] = h->compr[0];
    }

    h->copyrightb = (int)ac3_bits_get(&b, 1);
    h->origbs = (int)ac3_bits_get(&b, 1);

    h->timecod1e = (int)ac3_bits_get(&b, 1);
    if (h->timecod1e)
        h->timecod1 = (int)ac3_bits_get(&b, 14);
    h->timecod2e = (int)ac3_bits_get(&b, 1);
    if (h->timecod2e)
        h->timecod2 = (int)ac3_bits_get(&b, 14);

    h->addbsie = (int)ac3_bits_get(&b, 1);
    if (h->addbsie) {
        h->addbsil = (int)ac3_bits_get(&b, 6);
        for (i = 0; i <= h->addbsil; i++)
            (void)ac3_bits_get(&b, 8);
    }

    if (b.error)
        return -1;

    h->bits_used = b.pos;
    h->dialnorm_db = (h->dialnorm[0] >= 1 && h->dialnorm[0] <= 31)
                     ? -h->dialnorm[0] : 0;
    return 0;
}

/*
 * E-AC-3 bit stream information.  This follows ATSC A/52 Annex E and
 * carries per-block strategies, so the audio blocks themselves only carry
 * the data those strategies select.
 */
static int parse_eac3_header(const uint8_t *data, int size, ac3_header_t *h)
{
    ac3_bits_t b;
    int i, blk, ch, acmod;
    int ac3_exponent_strategy, parse_aht_info, parse_transient_proc_info;
    int parse_spx_atten_data;
    int num_cpl_blocks = 0;
    int nchaninfo;

    ac3_bits_init(&b, data, size);
    if (ac3_bits_get(&b, 16) != 0x0b77)
        return -1;

    h->eac3 = 1;
    h->frame_type = (int)ac3_bits_get(&b, 2);
    if (h->frame_type == 3)
        return -1;
    h->substreamid = (int)ac3_bits_get(&b, 3);

    h->frame_size = ((int)ac3_bits_get(&b, 11) + 1) * 2;
    if (h->frame_size < 7 || h->frame_size > size ||
        h->frame_size > AC3_MAX_EAC3_FRAME_BYTES)
        return -1;

    h->fscod = (int)ac3_bits_get(&b, 2);
    if (h->fscod == 3) {
        int fscod2 = (int)ac3_bits_get(&b, 2);

        if (fscod2 == 3)
            return -1;
        h->sample_rate = ac3_sample_rates[fscod2] / 2;
        h->fscod = fscod2;          /* base rate for the hearing threshold */
        h->sr_shift = 1;
        h->num_blocks = 6;
        h->halfrate = 1;
    } else {
        int numblkscod = (int)ac3_bits_get(&b, 2);

        h->sample_rate = ac3_sample_rates[h->fscod];
        h->num_blocks = eac3_blocks[numblkscod];
        h->sr_shift = 0;
        h->halfrate = 0;
    }

    h->acmod = acmod = (int)ac3_bits_get(&b, 3);
    h->lfeon = (int)ac3_bits_get(&b, 1);
    h->bsid = (int)ac3_bits_get(&b, 5);
    if (h->bsid < 11 || h->bsid > 16)
        return -1;

    h->nfchans = ac3_acmod_nch[acmod];
    h->channel_map = ac3_acmod_map[acmod];
    if (h->lfeon)
        h->channel_map |= AC3_SPEAKER_LFE;
    h->samples = h->num_blocks * AC3_BLOCK_SAMPLES;
    h->bit_rate = 8 * h->frame_size * h->sample_rate / h->samples;
    h->frmsizecod = -1;

    /* Defaults when the mixing metadata is absent. */
    h->clev = 0.59460356f;      /* -4.5 dB */
    h->slev = 0.5f;             /* -6.0 dB */

    nchaninfo = (acmod == 0) ? 2 : 1;
    for (i = 0; i < nchaninfo; i++) {
        h->dialnorm[i] = (int)ac3_bits_get(&b, 5);
        h->compr_present[i] = (int)ac3_bits_get(&b, 1);
        if (h->compr_present[i])
            h->compr[i] = (int)ac3_bits_get(&b, 8);
    }
    if (nchaninfo == 1) {
        h->dialnorm[1] = h->dialnorm[0];
        h->compr_present[1] = h->compr_present[0];
        h->compr[1] = h->compr[0];
    }
    h->dialnorm_db = (h->dialnorm[0] >= 1 && h->dialnorm[0] <= 31)
                     ? -h->dialnorm[0] : 0;

    /* Dependent stream channel map. */
    if (h->frame_type == 1) {
        if (ac3_bits_get(&b, 1))
            ac3_bits_skip(&b, 16);
    }

    /* Mixing metadata. */
    if (ac3_bits_get(&b, 1)) {
        if (acmod > 2) {
            (void)ac3_bits_get(&b, 2);      /* preferred downmix */
            if (acmod & 1) {
                (void)ac3_bits_get(&b, 3);  /* LTRT centre mix level */
                h->clev = eac3_gain_levels[(int)ac3_bits_get(&b, 3)];
            }
            if (acmod & 4) {
                int s = (int)ac3_bits_get(&b, 3);

                (void)s;                    /* LTRT surround mix level */
                s = (int)ac3_bits_get(&b, 3);
                if (s < 3)
                    s = 3;
                else if (s > 7)
                    s = 7;
                h->slev = eac3_gain_levels[s];
            }
        }
        if (h->lfeon) {
            if (ac3_bits_get(&b, 1))
                ac3_bits_skip(&b, 5);       /* LFE mix level */
        }
        if (h->frame_type == 0) {
            for (i = 0; i < (acmod ? 1 : 2); i++) {
                if (ac3_bits_get(&b, 1))
                    ac3_bits_skip(&b, 6);   /* program scale factor */
            }
            if (ac3_bits_get(&b, 1))
                ac3_bits_skip(&b, 6);       /* external program scale */
            switch (ac3_bits_get(&b, 2)) {
            case 1:
                ac3_bits_skip(&b, 5);
                break;
            case 2:
                ac3_bits_skip(&b, 12);
                break;
            case 3: {
                int mix_data_size = ((int)ac3_bits_get(&b, 5) + 2) << 3;

                ac3_bits_skip(&b, mix_data_size);
                break;
            }
            default:
                break;
            }
            if (acmod < 2) {
                for (i = 0; i < (acmod ? 1 : 2); i++) {
                    if (ac3_bits_get(&b, 1)) {
                        ac3_bits_skip(&b, 8);   /* pan mean direction */
                        ac3_bits_skip(&b, 6);   /* reserved pan info */
                    }
                }
            }
            if (ac3_bits_get(&b, 1)) {
                for (blk = 0; blk < h->num_blocks; blk++) {
                    if (h->num_blocks == 1 || ac3_bits_get(&b, 1))
                        ac3_bits_skip(&b, 5);
                }
            }
        }
    }

    /* Informational metadata. */
    if (ac3_bits_get(&b, 1)) {
        h->bsmod = (int)ac3_bits_get(&b, 3);
        ac3_bits_skip(&b, 2);           /* copyright, original bitstream */
        if (acmod == 2) {
            h->dsurmod = (int)ac3_bits_get(&b, 2);
            ac3_bits_skip(&b, 2);       /* headphone mode */
        }
        if (acmod >= 6)
            ac3_bits_skip(&b, 2);       /* Dolby Surround EX mode */
        for (i = 0; i < (acmod ? 1 : 2); i++) {
            if (ac3_bits_get(&b, 1))
                ac3_bits_skip(&b, 8);
        }
        if (!h->sr_shift)
            ac3_bits_skip(&b, 1);       /* source sample rate code */
    }

    /* Converter synchronisation flag. */
    if (h->frame_type == 0 && h->num_blocks != 6)
        ac3_bits_skip(&b, 1);

    /* Original frame size code for AC-3 converted streams. */
    if (h->frame_type == 2 &&
        (h->num_blocks == 6 || ac3_bits_get(&b, 1)))
        ac3_bits_skip(&b, 6);

    /* Additional bit stream information. */
    if (ac3_bits_get(&b, 1)) {
        int addbsil = (int)ac3_bits_get(&b, 6);

        for (i = 0; i <= addbsil; i++)
            ac3_bits_skip(&b, 8);
    }

    /* Audio frame syntax flags. */
    if (h->num_blocks == 6) {
        ac3_exponent_strategy = (int)ac3_bits_get(&b, 1);
        parse_aht_info = (int)ac3_bits_get(&b, 1);
    } else {
        ac3_exponent_strategy = 1;      /* AC-3 style strategy bits */
        parse_aht_info = 0;
    }
    h->snr_offset_strategy = (int)ac3_bits_get(&b, 2);
    parse_transient_proc_info = (int)ac3_bits_get(&b, 1);

    h->block_switch_syntax = (int)ac3_bits_get(&b, 1);
    h->dither_flag_syntax = (int)ac3_bits_get(&b, 1);
    h->bit_allocation_syntax = (int)ac3_bits_get(&b, 1);
    if (!h->bit_allocation_syntax) {
        h->slow_decay = ac3_slow_decay_tab[2];
        h->fast_decay = ac3_fast_decay_tab[1];
        h->slow_gain = ac3_slow_gain_tab[1];
        h->db_per_bit = ac3_db_per_bit_tab[2];
        h->floor = ac3_floor_tab[7];
    }
    h->fast_gain_syntax = (int)ac3_bits_get(&b, 1);
    h->dba_syntax = (int)ac3_bits_get(&b, 1);
    h->skip_syntax = (int)ac3_bits_get(&b, 1);
    parse_spx_atten_data = (int)ac3_bits_get(&b, 1);

    /* Coupling strategy per block. */
    if (acmod > 1) {
        for (blk = 0; blk < h->num_blocks; blk++) {
            h->cpl_strategy_exists[blk] = (!blk || ac3_bits_get(&b, 1));
            if (h->cpl_strategy_exists[blk])
                h->cpl_in_use[blk] = (int)ac3_bits_get(&b, 1);
            else
                h->cpl_in_use[blk] = h->cpl_in_use[blk - 1];
            num_cpl_blocks += h->cpl_in_use[blk];
        }
    }

    /* Exponent strategies. */
    if (ac3_exponent_strategy) {
        for (blk = 0; blk < h->num_blocks; blk++) {
            for (ch = !h->cpl_in_use[blk]; ch <= h->nfchans; ch++)
                h->exp_strategy[blk][ch] = (int)ac3_bits_get(&b, 2);
        }
    } else {
        for (ch = !(acmod > 1 && num_cpl_blocks); ch <= h->nfchans; ch++) {
            int frmchexpstr = (int)ac3_bits_get(&b, 5);

            for (blk = 0; blk < 6; blk++)
                h->exp_strategy[blk][ch] = ac3_eac3_frm_expstr[frmchexpstr][blk];
        }
    }
    if (h->lfeon) {
        for (blk = 0; blk < h->num_blocks; blk++)
            h->lfe_exp_strategy[blk] = (int)ac3_bits_get(&b, 1);
    }

    /* Original exponent strategies for AC-3 converted streams. */
    if (h->frame_type == 0 &&
        (h->num_blocks == 6 || ac3_bits_get(&b, 1)))
        ac3_bits_skip(&b, 5 * h->nfchans);

    /* Adaptive hybrid transform. */
    if (parse_aht_info) {
        int total_ch = h->nfchans + (h->lfeon ? 1 : 0);

        h->channel_uses_aht[0] = 0;
        for (ch = (num_cpl_blocks != 6); ch <= total_ch; ch++) {
            int use_aht = 1;

            for (blk = 1; blk < 6; blk++) {
                int st = (ch <= h->nfchans) ? h->exp_strategy[blk][ch]
                                            : h->lfe_exp_strategy[blk];

                if (st != 0 || (ch == 0 && h->cpl_strategy_exists[blk])) {
                    use_aht = 0;
                    break;
                }
            }
            h->channel_uses_aht[ch] = use_aht && ac3_bits_get(&b, 1);
        }
    }

    /* Per-frame SNR offset. */
    if (h->snr_offset_strategy == 0) {
        int csnr = ((int)ac3_bits_get(&b, 6) - 15) << 4;
        int snr = (csnr + (int)ac3_bits_get(&b, 4)) << 2;

        h->cpl_snr_offset = snr;
        for (i = 0; i < h->nfchans; i++)
            h->snr_offset[i] = snr;
        if (h->lfeon)
            h->snr_offset[h->nfchans] = snr;
    }

    /* Transient pre-noise processing data. */
    if (parse_transient_proc_info) {
        for (ch = 1; ch <= h->nfchans; ch++) {
            if (ac3_bits_get(&b, 1)) {
                ac3_bits_skip(&b, 10);
                ac3_bits_skip(&b, 8);
            }
        }
    }

    /* Spectral extension attenuation data. */
    for (ch = 0; ch <= h->nfchans; ch++)
        h->spx_atten_code[ch] = -1;
    for (ch = 1; ch <= h->nfchans; ch++) {
        if (parse_spx_atten_data && ac3_bits_get(&b, 1))
            h->spx_atten_code[ch] = (int)ac3_bits_get(&b, 5);
    }

    /* Block start information. */
    if (h->num_blocks > 1 && ac3_bits_get(&b, 1)) {
        int block_start_bits = (h->num_blocks - 1) *
                               (4 + floor_log2_int(h->frame_size - 2));

        ac3_bits_skip(&b, block_start_bits);
    }

    if (b.error)
        return -1;

    h->bits_used = b.pos;
    return 0;
}

int ac3_parse_header(const uint8_t *data, int size, ac3_header_t *h)
{
    int bsid;

    if (size < 7 || h == NULL)
        return -1;
    memset(h, 0, sizeof(*h));
    if (data[0] != 0x0b || data[1] != 0x77)
        return -1;

    bsid = data[5] >> 3;
    if (bsid > 10)
        return parse_eac3_header(data, size, h);
    return parse_ac3_header(data, size, h);
}

static uint16_t crc16_block(const uint8_t *data, int len)
{
    uint16_t crc = 0;
    int i;

    for (i = 0; i < len; i++)
        crc = (uint16_t)((crc << 8) ^ ac3_crc_table[(crc >> 8) ^ data[i]]);
    return crc;
}

int ac3_frame_crc_ok(const uint8_t *data, int size)
{
    int len58;
    uint16_t crc1, crc2;

    if (size < 8)
        return 0;
    if ((data[5] >> 3) > 10)
        return 1;               /* E-AC-3 syncframes carry no CRC */
    len58 = ((size >> 2) + (size >> 4)) << 1;
    if (len58 < 4 || len58 > size)
        return 0;
    crc1 = crc16_block(data + 2, len58 - 2);
    crc2 = crc16_block(data + len58, size - len58);
    return (crc1 == 0 && crc2 == 0) ? 1 : 0;
}
