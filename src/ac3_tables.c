/*
 * Tables and one-time initialisation.
 *
 * The fixed tables below are the constants defined by ATSC A/52 for the
 * AC-3 bit allocation, exponent and rematrixing algorithms.  Everything
 * that can be derived (window, FFT twiddles, scale factors, dither LFSR,
 * CRC table) is generated at run time instead of being copied.
 */
#include "ac3_internal.h"

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795029
#endif

/* ------------------------------------------------------------------ */
/* fixed A/52 tables                                                   */
/* ------------------------------------------------------------------ */

/* Bit-allocation pointer as a function of (mask + 4 * exponent).  The
 * useful part of the standard 305-entry table is the range -63..0; values
 * below -63 clamp to 16 bits and values above 0 to zero bits. */
const int8_t ac3_bap_table[64] = {
    16, 16, 16, 16, 16, 16, 16, 16, 16,
    14, 14, 14, 14, 14, 14, 14, 14,
    12, 12, 12, 12,
    11, 11, 11, 11,
    10, 10, 10, 10,
     9,  9,  9,  9,
     8,  8,  8,  8,
     7,  7,  7,  7,
     6,  6,  6,  6,
     5,  5,  5,  5,
     4,  4, -3, -3,  3,  3,  3, -2, -2,
    -1, -1, -1, -1, -1,  0
};

/* Hearing threshold curves for 48, 44.1 and 32 kHz, in 1/256 dB units. */
const int16_t ac3_hth_table[3][AC3_MAX_BANDS] = {
    {0x730, 0x730, 0x7c0, 0x800, 0x820, 0x840, 0x850, 0x850, 0x860, 0x860,
     0x860, 0x860, 0x860, 0x870, 0x870, 0x870, 0x880, 0x880, 0x890, 0x890,
     0x8a0, 0x8a0, 0x8b0, 0x8b0, 0x8c0, 0x8c0, 0x8d0, 0x8e0, 0x8f0, 0x900,
     0x910, 0x910, 0x910, 0x910, 0x900, 0x8f0, 0x8c0, 0x870, 0x820, 0x7e0,
     0x7a0, 0x770, 0x760, 0x7a0, 0x7c0, 0x7c0, 0x6e0, 0x400, 0x3c0, 0x3c0},
    {0x710, 0x710, 0x7a0, 0x7f0, 0x820, 0x830, 0x840, 0x850, 0x850, 0x860,
     0x860, 0x860, 0x860, 0x860, 0x870, 0x870, 0x870, 0x880, 0x880, 0x880,
     0x890, 0x890, 0x8a0, 0x8a0, 0x8b0, 0x8b0, 0x8c0, 0x8c0, 0x8e0, 0x8f0,
     0x900, 0x910, 0x910, 0x910, 0x910, 0x900, 0x8e0, 0x8b0, 0x870, 0x820,
     0x7e0, 0x7b0, 0x760, 0x770, 0x7a0, 0x7c0, 0x780, 0x5d0, 0x3c0, 0x3c0},
    {0x680, 0x680, 0x750, 0x7b0, 0x7e0, 0x810, 0x820, 0x830, 0x840, 0x850,
     0x850, 0x850, 0x860, 0x860, 0x860, 0x860, 0x860, 0x860, 0x860, 0x860,
     0x870, 0x870, 0x870, 0x870, 0x880, 0x880, 0x880, 0x890, 0x8a0, 0x8b0,
     0x8c0, 0x8d0, 0x8e0, 0x8f0, 0x900, 0x910, 0x910, 0x910, 0x900, 0x8f0,
     0x8d0, 0x8b0, 0x840, 0x7f0, 0x790, 0x760, 0x7a0, 0x7c0, 0x7b0, 0x720}
};

/* Critical-band upper boundaries. */
const int ac3_bndtab[30] = {
    21, 22, 23, 24, 25, 26, 27, 28, 31, 34,
    37, 40, 43, 46, 49, 55, 61, 67, 73, 79,
    85, 97, 109, 121, 133, 157, 181, 205, 229, 253
};

/* Log-add correction table used when integrating power spectral density. */
const int8_t ac3_latab[256] = {
    -64, -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53,
    -52, -52, -51, -50, -49, -48, -47, -47, -46, -45, -44, -44,
    -43, -42, -41, -41, -40, -39, -38, -38, -37, -36, -36, -35,
    -35, -34, -33, -33, -32, -32, -31, -30, -30, -29, -29, -28,
    -28, -27, -27, -26, -26, -25, -25, -24, -24, -23, -23, -22,
    -22, -21, -21, -21, -20, -20, -19, -19, -19, -18, -18, -18,
    -17, -17, -17, -16, -16, -16, -15, -15, -15, -14, -14, -14,
    -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -11, -11,
    -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -8, -8,
    -8, -8, -8, -8, -7, -7, -7, -7, -7, -7, -6, -6,
    -6, -6, -6, -6, -6, -6, -5, -5, -5, -5, -5, -5,
    -5, -5, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4,
    -4, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3,
    -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0
};

/* Coupling start band (used by the bit allocation routine). */
const uint8_t ac3_cpl_bndtab[16] = {
    31, 35, 37, 39, 41, 42, 43, 44, 45, 45, 46, 46, 47, 47, 48, 48
};

/* Rematrix band upper boundaries. */
const int ac3_remat_band[4] = {25, 37, 61, 253};

const int ac3_slowgain[4]  = {0x540, 0x4d8, 0x478, 0x410};
const int ac3_dbpbtab[4]   = {0xc00, 0x500, 0x300, 0x100};
const int ac3_floortab[8]  = {0x910, 0x950, 0x990, 0x9d0,
                              0xa10, 0xa90, 0xb10, 0x1400};

const float ac3_clev_table[4] = {
    0.7071067811865476f, 0.5946035575013605f,
    0.5f,                0.5946035575013605f
};

const float ac3_slev_table[4] = {
    0.7071067811865476f, 0.5f, 0.0f, 0.5f
};

/* Coded channel layout for every audio coding mode.  The channel order is
 * the order used in an AC-3 block. */
const int ac3_acmod_nch[8] = {2, 1, 2, 3, 3, 4, 4, 5};

const int ac3_acmod_map[8] = {
    AC3_SPEAKER_L | AC3_SPEAKER_R,
    AC3_SPEAKER_C,
    AC3_SPEAKER_L | AC3_SPEAKER_R,
    AC3_SPEAKER_L | AC3_SPEAKER_R | AC3_SPEAKER_C,
    AC3_SPEAKER_L | AC3_SPEAKER_R | AC3_SPEAKER_LS,
    AC3_SPEAKER_L | AC3_SPEAKER_R | AC3_SPEAKER_C | AC3_SPEAKER_LS,
    AC3_SPEAKER_L | AC3_SPEAKER_R | AC3_SPEAKER_LS | AC3_SPEAKER_RS,
    AC3_SPEAKER_L | AC3_SPEAKER_R | AC3_SPEAKER_C | AC3_SPEAKER_LS | AC3_SPEAKER_RS
};

const int ac3_acmod_speakers[8][5] = {
    {AC3_SPEAKER_L, AC3_SPEAKER_R, 0, 0, 0},
    {AC3_SPEAKER_C, 0, 0, 0, 0},
    {AC3_SPEAKER_L, AC3_SPEAKER_R, 0, 0, 0},
    {AC3_SPEAKER_L, AC3_SPEAKER_C, AC3_SPEAKER_R, 0, 0},
    {AC3_SPEAKER_L, AC3_SPEAKER_R, AC3_SPEAKER_LS, 0, 0},
    {AC3_SPEAKER_L, AC3_SPEAKER_C, AC3_SPEAKER_R, AC3_SPEAKER_LS, 0},
    {AC3_SPEAKER_L, AC3_SPEAKER_R, AC3_SPEAKER_LS, AC3_SPEAKER_RS, 0},
    {AC3_SPEAKER_L, AC3_SPEAKER_C, AC3_SPEAKER_R, AC3_SPEAKER_LS, AC3_SPEAKER_RS}
};

/* ------------------------------------------------------------------ */
/* generated tables                                                    */
/* ------------------------------------------------------------------ */

float ac3_scale_factor[25];
float ac3_window[AC3_BLOCK_SAMPLES];
float ac3_fft_cos[64];
float ac3_fft_sin[64];
float ac3_imdct1_cos[128];
float ac3_imdct1_sin[128];
float ac3_imdct2_cos[64];
float ac3_imdct2_sin[64];
uint8_t ac3_bitrev7[128];
uint8_t ac3_bitrev6[64];
uint16_t ac3_dither_step[256];
uint16_t ac3_crc_table[256];

static volatile int ac3_init_done;
#if defined(__GNUC__) || defined(__clang__)
static volatile int ac3_init_lock;
#endif

static double bessel_i0(double x)
{
    double b = 1.0;
    int i = 100;

    do {
        b = b * x / ((double)i * (double)i) + 1.0;
    } while (--i);
    return b;
}

static uint8_t reverse_bits(unsigned v, int bits)
{
    unsigned r = 0;
    int i;

    for (i = 0; i < bits; i++) {
        r = (r << 1) | (v & 1);
        v >>= 1;
    }
    return (uint8_t)r;
}

static uint16_t dither_step8(unsigned b)
{
    unsigned state = b << 8;
    int i;

    for (i = 0; i < 8; i++) {
        if (state & 0x8000)
            state = ((state << 1) ^ 0xa011) & 0xffff;
        else
            state = (state << 1) & 0xffff;
    }
    return (uint16_t)state;
}

void ac3_tables_init(void)
{
    int i, j;
    double sum;

    if (ac3_init_done)
        return;
#if defined(__GNUC__) || defined(__clang__)
    while (__sync_lock_test_and_set(&ac3_init_lock, 1))
        ;
#endif
    if (ac3_init_done) {
#if defined(__GNUC__) || defined(__clang__)
        __sync_lock_release(&ac3_init_lock);
#endif
        return;
    }

    for (i = 0; i < 25; i++)
        ac3_scale_factor[i] = (float)ldexp(1.0, -15 - i);

    /* Kaiser-Bessel derived window, alpha = 5.0, 256 distinct values.
     * The full 512-point window is symmetric: w[511 - n] = w[n]. */
    sum = 0.0;
    {
        double local[AC3_BLOCK_SAMPLES];
        double k = 5.0 * M_PI / 256.0;

        for (i = 0; i < AC3_BLOCK_SAMPLES; i++) {
            sum += bessel_i0((double)i * (256 - i) * k * k);
            local[i] = sum;
        }
        sum += 1.0;
        for (i = 0; i < AC3_BLOCK_SAMPLES; i++)
            ac3_window[i] = (float)sqrt(local[i] / sum);
    }

    for (i = 0; i < 64; i++) {
        double a = 2.0 * M_PI * i / 128.0;
        ac3_fft_cos[i] = (float)cos(a);
        ac3_fft_sin[i] = (float)sin(a);
    }

    /* IMDCT pre/post twiddles from A/52 section 7.9.4. */
    for (i = 0; i < 128; i++) {
        double a = 2.0 * M_PI * (8.0 * i + 1.0) / (8.0 * 512.0);
        ac3_imdct1_cos[i] = (float)-cos(a);
        ac3_imdct1_sin[i] = (float)-sin(a);
    }
    for (i = 0; i < 64; i++) {
        double a = 2.0 * M_PI * (8.0 * i + 1.0) / (4.0 * 512.0);
        ac3_imdct2_cos[i] = (float)-cos(a);
        ac3_imdct2_sin[i] = (float)-sin(a);
    }

    for (i = 0; i < 128; i++)
        ac3_bitrev7[i] = reverse_bits((unsigned)i, 7);
    for (i = 0; i < 64; i++)
        ac3_bitrev6[i] = reverse_bits((unsigned)i, 6);

    for (i = 0; i < 256; i++)
        ac3_dither_step[i] = dither_step8((unsigned)i);

    for (i = 0; i < 256; i++) {
        unsigned crc = (unsigned)i << 8;
        for (j = 0; j < 8; j++)
            crc = ((crc << 1) ^ ((crc & 0x8000) ? 0x8005 : 0)) & 0xffff;
        ac3_crc_table[i] = (uint16_t)crc;
    }

    /* E-AC-3 bin -> critical band map, derived from the band boundaries. */
    {
        int band = 0;
        for (i = 0; i < 253; i++) {
            while (band < AC3_MAX_BANDS - 1 && i >= ac3_band_start_tab[band + 1])
                band++;
            ac3_bin_to_band_tab[i] = (uint8_t)band;
        }
    }

    ac3_init_done = 1;
#if defined(__GNUC__) || defined(__clang__)
    __sync_lock_release(&ac3_init_lock);
#endif
}

/* ------------------------------------------------------------------ */
/* E-AC-3 tables (ATSC A/52 Annex E)                                   */
/* ------------------------------------------------------------------ */

const uint8_t ac3_band_start_tab[AC3_MAX_BANDS + 1] = {
      0,  1,   2,   3,   4,   5,   6,   7,   8,   9,
     10,  11, 12,  13,  14,  15,  16,  17,  18,  19,
     20,  21, 22,  23,  24,  25,  26,  27,  28,  31,
     34,  37, 40,  43,  46,  49,  55,  61,  67,  73,
     79,  85, 97, 109, 121, 133, 157, 181, 205, 229, 253
};

uint8_t ac3_bin_to_band_tab[253];

const uint8_t ac3_log_add_tab[260] = {
0x40,0x3f,0x3e,0x3d,0x3c,0x3b,0x3a,0x39,0x38,0x37,
0x36,0x35,0x34,0x34,0x33,0x32,0x31,0x30,0x2f,0x2f,
0x2e,0x2d,0x2c,0x2c,0x2b,0x2a,0x29,0x29,0x28,0x27,
0x26,0x26,0x25,0x24,0x24,0x23,0x23,0x22,0x21,0x21,
0x20,0x20,0x1f,0x1e,0x1e,0x1d,0x1d,0x1c,0x1c,0x1b,
0x1b,0x1a,0x1a,0x19,0x19,0x18,0x18,0x17,0x17,0x16,
0x16,0x15,0x15,0x15,0x14,0x14,0x13,0x13,0x13,0x12,
0x12,0x12,0x11,0x11,0x11,0x10,0x10,0x10,0x0f,0x0f,
0x0f,0x0e,0x0e,0x0e,0x0d,0x0d,0x0d,0x0d,0x0c,0x0c,
0x0c,0x0c,0x0b,0x0b,0x0b,0x0b,0x0a,0x0a,0x0a,0x0a,
0x0a,0x09,0x09,0x09,0x09,0x09,0x08,0x08,0x08,0x08,
0x08,0x08,0x07,0x07,0x07,0x07,0x07,0x07,0x06,0x06,
0x06,0x06,0x06,0x06,0x06,0x06,0x05,0x05,0x05,0x05,
0x05,0x05,0x05,0x05,0x04,0x04,0x04,0x04,0x04,0x04,
0x04,0x04,0x04,0x04,0x04,0x03,0x03,0x03,0x03,0x03,
0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x02,
0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,
0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

const int16_t ac3_hearing_threshold_tab[AC3_MAX_BANDS][3] = {
{ 0x04d0,0x04f0,0x0580 }, { 0x04d0,0x04f0,0x0580 }, { 0x0440,0x0460,0x04b0 },
{ 0x0400,0x0410,0x0450 }, { 0x03e0,0x03e0,0x0420 }, { 0x03c0,0x03d0,0x03f0 },
{ 0x03b0,0x03c0,0x03e0 }, { 0x03b0,0x03b0,0x03d0 }, { 0x03a0,0x03b0,0x03c0 },
{ 0x03a0,0x03a0,0x03b0 }, { 0x03a0,0x03a0,0x03b0 }, { 0x03a0,0x03a0,0x03b0 },
{ 0x03a0,0x03a0,0x03a0 }, { 0x0390,0x03a0,0x03a0 }, { 0x0390,0x0390,0x03a0 },
{ 0x0390,0x0390,0x03a0 }, { 0x0380,0x0390,0x03a0 }, { 0x0380,0x0380,0x03a0 },
{ 0x0370,0x0380,0x03a0 }, { 0x0370,0x0380,0x03a0 }, { 0x0360,0x0370,0x0390 },
{ 0x0360,0x0370,0x0390 }, { 0x0350,0x0360,0x0390 }, { 0x0350,0x0360,0x0390 },
{ 0x0340,0x0350,0x0380 }, { 0x0340,0x0350,0x0380 }, { 0x0330,0x0340,0x0380 },
{ 0x0320,0x0340,0x0370 }, { 0x0310,0x0320,0x0360 }, { 0x0300,0x0310,0x0350 },
{ 0x02f0,0x0300,0x0340 }, { 0x02f0,0x02f0,0x0330 }, { 0x02f0,0x02f0,0x0320 },
{ 0x02f0,0x02f0,0x0310 }, { 0x0300,0x02f0,0x0300 }, { 0x0310,0x0300,0x02f0 },
{ 0x0340,0x0320,0x02f0 }, { 0x0390,0x0350,0x02f0 }, { 0x03e0,0x0390,0x0300 },
{ 0x0420,0x03e0,0x0310 }, { 0x0460,0x0420,0x0330 }, { 0x0490,0x0450,0x0350 },
{ 0x04a0,0x04a0,0x03c0 }, { 0x0460,0x0490,0x0410 }, { 0x0440,0x0460,0x0470 },
{ 0x0440,0x0440,0x04a0 }, { 0x0520,0x0480,0x0460 }, { 0x0800,0x0630,0x0440 },
{ 0x0840,0x0840,0x0450 }, { 0x0840,0x0840,0x04e0 }
};

/* Bit allocation pointer as a function of the address computed by the
 * E-AC-3 bit allocation routine.  Values 1..5 select the grouped
 * quantisers, 6..15 the signed quantisers. */
const uint8_t ac3_ff_bap_tab[64] = {
    0, 1, 1, 1, 1, 1, 2, 2, 3, 3,
    3, 4, 4, 5, 5, 6, 6, 6, 6, 7,
    7, 7, 7, 8, 8, 8, 8, 9, 9, 9,
    9, 10, 10, 10, 10, 11, 11, 11, 11, 12,
    12, 12, 12, 13, 13, 13, 13, 14, 14, 14,
    14, 14, 14, 14, 14, 15, 15, 15, 15, 15,
    15, 15, 15, 15
};

const uint8_t ac3_slow_decay_tab[4] = { 0x0f, 0x11, 0x13, 0x15 };
const uint8_t ac3_fast_decay_tab[4] = { 0x3f, 0x53, 0x67, 0x7b };
const uint16_t ac3_slow_gain_tab[4] = { 0x540, 0x4d8, 0x478, 0x410 };
const uint16_t ac3_db_per_bit_tab[4] = { 0x000, 0x700, 0x900, 0xb00 };
const int16_t ac3_floor_tab[8] = {
    0x2f0, 0x2b0, 0x270, 0x230, 0x1f0, 0x170, 0x0f0, -2048
};
const uint16_t ac3_fast_gain_tab[8] = {
    0x080, 0x100, 0x180, 0x200, 0x280, 0x300, 0x380, 0x400
};

/* Frame exponent strategy combinations (0 = reuse, 1 = D15, 2 = D25,
 * 3 = D45). */
const uint8_t ac3_eac3_frm_expstr[32][6] = {
{1,0,0,0,0,0}, {1,0,0,0,0,3}, {1,0,0,0,2,0}, {1,0,0,0,3,3},
{2,0,0,2,0,0}, {2,0,0,2,0,3}, {2,0,0,3,2,0}, {2,0,0,3,3,3},
{2,0,1,0,0,0}, {2,0,2,0,0,3}, {2,0,2,0,2,0}, {2,0,2,0,3,3},
{2,0,3,2,0,0}, {2,0,3,2,0,3}, {2,0,3,3,2,0}, {2,0,3,3,3,3},
{3,1,0,0,0,0}, {3,1,0,0,0,3}, {3,2,0,0,2,0}, {3,2,0,0,3,3},
{3,2,0,2,0,0}, {3,2,0,2,0,3}, {3,2,0,3,2,0}, {3,2,0,3,3,3},
{3,3,1,0,0,0}, {3,3,2,0,0,3}, {3,3,2,0,2,0}, {3,3,2,0,3,3},
{3,3,3,2,0,0}, {3,3,3,2,0,3}, {3,3,3,3,2,0}, {3,3,3,3,3,3}
};

/* Default coupling band structure (1 = merge with the previous subband). */
const uint8_t ac3_eac3_default_cpl_band_struct[18] = {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1
};

/* E-AC-3 bap -> the AC-3 raw quantiser codes used by ac3_block.c. */
const int8_t ac3_eac3_bap_map[16] = {
    0, -1, -2, 3, -3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16
};
