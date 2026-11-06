/*
 * AC-3 channel remapping and downmixing.
 *
 * The decoder first reconstructs the coded channels in bitstream order.
 * This module maps them to the canonical output order L, R, C, LFE, LS, RS
 * and applies the A/52 downmix equations for the requested layout.
 */
#include "ac3_internal.h"

static const int canonical_roles[6] = {
    AC3_SPEAKER_L, AC3_SPEAKER_R, AC3_SPEAKER_C,
    AC3_SPEAKER_LFE, AC3_SPEAKER_LS, AC3_SPEAKER_RS
};

static int role_to_index(const ac3_mix_t *mix, int role)
{
    int i;

    for (i = 0; i < mix->out_count; i++)
        if (mix->out_roles[i] == role)
            return i;
    return -1;
}

static void add_input(ac3_mix_t *mix, int out_role, int in_col, float gain)
{
    int idx = role_to_index(mix, out_role);
    if (idx >= 0 && in_col >= 0)
        mix->matrix[idx][in_col] += gain;
}

static void add_both(ac3_mix_t *mix, int in_col, float left, float right)
{
    add_input(mix, AC3_SPEAKER_L, in_col, left);
    add_input(mix, AC3_SPEAKER_R, in_col, right);
}

static float downmix_gain(const ac3_header_t *h, int mode, int out_count)
{
    float clev = h->clev, slev = h->slev;

    (void)out_count;
    switch (mode) {
    case AC3_OUTPUT_SAME:
        return 1.0f;
    case AC3_OUTPUT_MONO:
        switch (h->acmod) {
        case 0: return 0.70710678f;
        case 1: return 1.0f;
        case 2: return 0.70710678f;
        case 3: return 0.70710678f / (1.0f + clev);
        case 4: return 1.41421356f / (2.0f + slev);
        case 5: return 0.70710678f / (1.0f + clev + slev * 0.5f);
        case 6: return 0.70710678f / (1.0f + slev);
        default: return 0.70710678f / (1.0f + clev + slev);
        }
    case AC3_OUTPUT_STEREO:
        switch (h->acmod) {
        case 0:
        case 1:
        case 2: return 1.0f;
        case 3: return 1.0f / (1.0f + clev);
        case 4: return 1.0f / (1.0f + slev * 0.70710678f);
        case 5: return 1.0f / (1.0f + clev + slev * 0.70710678f);
        case 6: return 1.0f / (1.0f + slev);
        default: return 1.0f / (1.0f + clev + slev);
        }
    case AC3_OUTPUT_DOLBY:
        switch (h->acmod) {
        case 0:
        case 1:
        case 2: return 1.0f;
        case 3:
        case 4: return 1.0f / (1.0f + 0.70710678f);
        case 5:
        case 6: return 1.0f / (1.0f + 2.0f * 0.70710678f);
        default: return 1.0f / (1.0f + 3.0f * 0.70710678f);
        }
    case AC3_OUTPUT_CHANNEL1:
    case AC3_OUTPUT_CHANNEL2:
        return 1.0f;
    default:
        if (out_count < ac3_acmod_nch[h->acmod])
            return 1.0f / (1.0f + clev + slev);
        return 1.0f;
    }
}

static int keep_lfe(const ac3_param_t *p, int mode)
{
    if (p->output & AC3_OUTPUT_LFE)
        return 1;
    if (p->output & AC3_OUTPUT_NO_LFE)
        return 0;
    switch (mode) {
    case AC3_OUTPUT_SAME:
    case AC3_OUTPUT_3F:
    case AC3_OUTPUT_2F1R:
    case AC3_OUTPUT_3F1R:
    case AC3_OUTPUT_2F2R:
    case AC3_OUTPUT_3F2R:
        return 1;
    default:
        return 0;
    }
}

void ac3_mix_init(ac3_mix_t *mix, const ac3_header_t *h,
                  const ac3_param_t *param)
{
    int mode = param->output & 15;
    int lfe = h->lfeon && keep_lfe(param, mode);
    int base_map = 0;
    int i, r;
    int has_ls = 0, has_rs = 0;
    int input_has_surround_pair = 0;
    float clev = h->clev, slev = h->slev;

    memset(mix, 0, sizeof(*mix));
    mix->in_count = h->nfchans;
    mix->have_lfe = lfe;

    switch (mode) {
    case AC3_OUTPUT_SAME:
        base_map = h->channel_map;
        break;
    case AC3_OUTPUT_MONO:
        base_map = AC3_SPEAKER_C;
        break;
    case AC3_OUTPUT_STEREO:
    case AC3_OUTPUT_DOLBY:
        base_map = AC3_SPEAKER_L | AC3_SPEAKER_R;
        break;
    case AC3_OUTPUT_3F:
        base_map = AC3_SPEAKER_L | AC3_SPEAKER_R | AC3_SPEAKER_C;
        break;
    case AC3_OUTPUT_2F1R:
        base_map = AC3_SPEAKER_L | AC3_SPEAKER_R | AC3_SPEAKER_LS;
        break;
    case AC3_OUTPUT_3F1R:
        base_map = AC3_SPEAKER_L | AC3_SPEAKER_R | AC3_SPEAKER_C |
                   AC3_SPEAKER_LS;
        break;
    case AC3_OUTPUT_2F2R:
        base_map = AC3_SPEAKER_L | AC3_SPEAKER_R |
                   AC3_SPEAKER_LS | AC3_SPEAKER_RS;
        break;
    case AC3_OUTPUT_3F2R:
        base_map = AC3_SPEAKER_L | AC3_SPEAKER_R | AC3_SPEAKER_C |
                   AC3_SPEAKER_LS | AC3_SPEAKER_RS;
        break;
    case AC3_OUTPUT_CHANNEL1:
        base_map = AC3_SPEAKER_L;
        break;
    case AC3_OUTPUT_CHANNEL2:
        base_map = AC3_SPEAKER_R;
        break;
    default:
        base_map = h->channel_map;
        break;
    }

    if (lfe)
        base_map |= AC3_SPEAKER_LFE;

    for (i = 0; i < 6; i++) {
        if (base_map & canonical_roles[i])
            mix->out_roles[mix->out_count++] = canonical_roles[i];
    }
    mix->out_map = base_map;

    for (i = 0; i < h->nfchans; i++) {
        r = ac3_acmod_speakers[h->acmod][i];
        if (r == AC3_SPEAKER_LS)
            has_ls = 1;
        if (r == AC3_SPEAKER_RS)
            has_rs = 1;
    }
    input_has_surround_pair = (has_ls && has_rs);

    if (mode == AC3_OUTPUT_SAME) {
        for (i = 0; i < h->nfchans; i++)
            add_input(mix, ac3_acmod_speakers[h->acmod][i], i, 1.0f);
    } else {
        int out_mono = (mode == AC3_OUTPUT_MONO);
        int out_stereo = (mode == AC3_OUTPUT_STEREO ||
                          mode == AC3_OUTPUT_DOLBY);
        int out_dolby = (mode == AC3_OUTPUT_DOLBY);
        int out_has_surround = (base_map & (AC3_SPEAKER_LS | AC3_SPEAKER_RS)) != 0;

        for (i = 0; i < h->nfchans; i++) {
            r = ac3_acmod_speakers[h->acmod][i];

            switch (r) {
            case AC3_SPEAKER_L:
                if (role_to_index(mix, AC3_SPEAKER_L) >= 0)
                    add_input(mix, AC3_SPEAKER_L, i, 1.0f);
                else if (out_mono)
                    add_input(mix, AC3_SPEAKER_C, i, 0.70710678f);
                break;

            case AC3_SPEAKER_R:
                if (role_to_index(mix, AC3_SPEAKER_R) >= 0)
                    add_input(mix, AC3_SPEAKER_R, i, 1.0f);
                else if (out_mono)
                    add_input(mix, AC3_SPEAKER_C, i, 0.70710678f);
                break;

            case AC3_SPEAKER_C:
                if (role_to_index(mix, AC3_SPEAKER_C) >= 0) {
                    add_input(mix, AC3_SPEAKER_C, i, 1.0f);
                } else if (out_stereo) {
                    float g = (h->acmod == 1) ? 1.0f : clev;
                    add_both(mix, i, g, g);
                } else if (out_mono) {
                    add_input(mix, AC3_SPEAKER_C, i, 1.0f);
                } else if (role_to_index(mix, AC3_SPEAKER_L) >= 0) {
                    add_input(mix, AC3_SPEAKER_L, i, clev);
                } else if (role_to_index(mix, AC3_SPEAKER_R) >= 0) {
                    add_input(mix, AC3_SPEAKER_R, i, clev);
                }
                break;

            case AC3_SPEAKER_LS:
                if (role_to_index(mix, AC3_SPEAKER_LS) >= 0) {
                    add_input(mix, AC3_SPEAKER_LS, i, 1.0f);
                    /* A single surround source feeding a 2/2 layout is
                     * duplicated into the right surround. */
                    if (!input_has_surround_pair &&
                        role_to_index(mix, AC3_SPEAKER_RS) >= 0)
                        add_input(mix, AC3_SPEAKER_RS, i, 1.0f);
                } else if (role_to_index(mix, AC3_SPEAKER_RS) >= 0) {
                    add_input(mix, AC3_SPEAKER_RS, i,
                              input_has_surround_pair ? 0.70710678f : 1.0f);
                } else if (out_stereo && role_to_index(mix, AC3_SPEAKER_L) >= 0) {
                    if (input_has_surround_pair) {
                        add_input(mix, AC3_SPEAKER_L, i,
                                  out_dolby ? -slev : slev);
                        /* Dolby Surround adds both surrounds to Rt as
                         * well, out of phase with Lt. */
                        if (out_dolby)
                            add_input(mix, AC3_SPEAKER_R, i, slev);
                    } else {
                        add_input(mix, AC3_SPEAKER_L, i,
                                  out_dolby ? -slev : slev);
                        add_input(mix, AC3_SPEAKER_R, i,
                                  out_dolby ? slev : slev);
                    }
                } else if (out_mono) {
                    add_input(mix, AC3_SPEAKER_C, i, slev);
                } else if (role_to_index(mix, AC3_SPEAKER_L) >= 0) {
                    add_input(mix, AC3_SPEAKER_L, i,
                              out_dolby ? -slev : slev);
                    if (!input_has_surround_pair)
                        add_input(mix, AC3_SPEAKER_R, i,
                                  out_dolby ? -slev : slev);
                }
                break;

            case AC3_SPEAKER_RS:
                if (role_to_index(mix, AC3_SPEAKER_RS) >= 0) {
                    add_input(mix, AC3_SPEAKER_RS, i, 1.0f);
                } else if (role_to_index(mix, AC3_SPEAKER_LS) >= 0) {
                    add_input(mix, AC3_SPEAKER_LS, i, 0.70710678f);
                } else if (out_mono) {
                    add_input(mix, AC3_SPEAKER_C, i, slev);
                } else if (role_to_index(mix, AC3_SPEAKER_R) >= 0) {
                    add_input(mix, AC3_SPEAKER_R, i, slev);
                    add_input(mix, AC3_SPEAKER_L, i,
                              out_dolby ? -slev : 0.0f);
                }
                break;

            default:
                break;
            }
        }

        (void)out_has_surround;
    }

    if (lfe) {
        int col = AC3_MAX_CHANNELS - 1;      /* LFE column is fixed at 5 */
        add_input(mix, AC3_SPEAKER_LFE, col, 1.0f);
    }

    mix->gain = downmix_gain(h, mode, mix->out_count);
    if (param->disable_level_adjust)
        mix->gain = 1.0f;
    if (param->apply_dialnorm && h->dialnorm_db)
        mix->gain *= (float)pow(10.0, h->dialnorm_db / 20.0);
}

void ac3_mix_block(const ac3_mix_t *mix,
                   const float pcm[AC3_MAX_CHANNELS][AC3_BLOCK_SAMPLES],
                   float out[AC3_MAX_CHANNELS][AC3_BLOCK_SAMPLES])
{
    int o, i, n;

    for (o = 0; o < mix->out_count; o++) {
        float *dst = out[o];
        int first = 1;

        for (i = 0; i < mix->in_count; i++) {
            float g = mix->matrix[o][i];
            const float *src;

            if (g == 0.0f)
                continue;
            src = pcm[i];
            if (first) {
                for (n = 0; n < AC3_BLOCK_SAMPLES; n++)
                    dst[n] = g * src[n];
                first = 0;
            } else {
                for (n = 0; n < AC3_BLOCK_SAMPLES; n++)
                    dst[n] += g * src[n];
            }
        }
        if (mix->have_lfe) {
            float g = mix->matrix[o][AC3_MAX_CHANNELS - 1];
            const float *src = pcm[AC3_MAX_CHANNELS - 1];
            if (g != 0.0f) {
                if (first) {
                    for (n = 0; n < AC3_BLOCK_SAMPLES; n++)
                        dst[n] = g * src[n];
                    first = 0;
                } else {
                    for (n = 0; n < AC3_BLOCK_SAMPLES; n++)
                        dst[n] += g * src[n];
                }
            }
        }
        if (first) {
            for (n = 0; n < AC3_BLOCK_SAMPLES; n++)
                dst[n] = 0.0f;
        }
        for (n = 0; n < AC3_BLOCK_SAMPLES; n++)
            dst[n] *= mix->gain;
    }
}
