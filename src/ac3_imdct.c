/*
 * AC-3 inverse MDCT, windowing and overlap-add.
 *
 * The transform uses the A/52 section 7.9.4 decomposition: a complex
 * pre-twiddle, a quarter-size complex IFFT, a post-twiddle and the
 * spec's de-interleaving permutation.  The FFT is a plain radix-2
 * decimation-in-time transform; AC-3 only needs sizes 128 and 64.
 */
#include "ac3_internal.h"

typedef struct {
    float r, i;
} ac3_cplx_t;

static void fft_inverse(ac3_cplx_t *x, int n, const uint8_t *bitrev)
{
    int i, len, half, step, base, j;

    for (i = 0; i < n; i++) {
        int k = bitrev[i];
        if (k > i) {
            ac3_cplx_t t = x[i];
            x[i] = x[k];
            x[k] = t;
        }
    }

    for (len = 2; len <= n; len <<= 1) {
        half = len >> 1;
        step = 128 / len;
        for (base = 0; base < n; base += len) {
            for (j = 0; j < half; j++) {
                float wr = ac3_fft_cos[j * step];
                float wi = ac3_fft_sin[j * step];
                ac3_cplx_t a = x[base + j];
                ac3_cplx_t b = x[base + j + half];
                float tr = wr * b.r - wi * b.i;
                float ti = wr * b.i + wi * b.r;
                ac3_cplx_t t;

                t.r = a.r + tr;
                t.i = a.i + ti;
                x[base + j + half].r = a.r - tr;
                x[base + j + half].i = a.i - ti;
                x[base + j] = t;
            }
        }
    }
}

void ac3_imdct_long(const float *coeff, float *out)
{
    ac3_cplx_t z[128];
    float yr[128], yi[128];
    int k, n;

    for (k = 0; k < 128; k++) {
        float a = coeff[256 - 2 * k - 1];
        float b = coeff[2 * k];
        float cr = ac3_imdct1_cos[k];
        float ci = ac3_imdct1_sin[k];

        z[k].r = a * cr - b * ci;
        z[k].i = b * cr + a * ci;
    }

    fft_inverse(z, 128, ac3_bitrev7);

    for (n = 0; n < 128; n++) {
        float cr = ac3_imdct1_cos[n];
        float ci = ac3_imdct1_sin[n];

        yr[n] = z[n].r * cr - z[n].i * ci;
        yi[n] = z[n].r * ci + z[n].i * cr;
    }

    for (n = 0; n < 64; n++) {
        out[2 * n]         = -yi[64 + n];
        out[2 * n + 1]     =  yr[63 - n];
        out[128 + 2 * n]   = -yr[n];
        out[128 + 2 * n + 1] = yi[127 - n];
        out[256 + 2 * n]   = -yr[64 + n];
        out[256 + 2 * n + 1] = yi[63 - n];
        out[384 + 2 * n]   =  yi[n];
        out[384 + 2 * n + 1] = -yr[127 - n];
    }
}

/*
 * One 256-point IMDCT: 128 coefficients in, 256 time samples out.  Uses the
 * same complex-FFT decomposition as the long transform with half the sizes
 * (64-point complex FFT), followed by the A/52 de-interleaving permutation.
 */
static void imdct256(const float *x, float *y)
{
    ac3_cplx_t z[64];
    float yr[64], yi[64];
    int k, n;

    for (k = 0; k < 64; k++) {
        float cr = ac3_imdct2_cos[k];
        float ci = ac3_imdct2_sin[k];
        float a = x[127 - 2 * k];
        float b = x[2 * k];

        z[k].r = a * cr - b * ci;
        z[k].i = b * cr + a * ci;
    }

    fft_inverse(z, 64, ac3_bitrev6);

    for (n = 0; n < 64; n++) {
        float cr = ac3_imdct2_cos[n];
        float ci = ac3_imdct2_sin[n];

        yr[n] = z[n].r * cr - z[n].i * ci;
        yi[n] = z[n].r * ci + z[n].i * cr;
    }

    for (n = 0; n < 32; n++) {
        y[2 * n]           = -yi[32 + n];
        y[2 * n + 1]       =  yr[31 - n];
        y[64 + 2 * n]      = -yr[n];
        y[64 + 2 * n + 1]  =  yi[63 - n];
        y[128 + 2 * n]     = -yr[32 + n];
        y[128 + 2 * n + 1] =  yi[31 - n];
        y[192 + 2 * n]     =  yi[n];
        y[192 + 2 * n + 1] = -yr[63 - n];
    }
}

/*
 * Block switching.  The 256 coefficients are split into an even and an odd
 * set, each transformed by a 256-point IMDCT.  A/52 then assembles the
 * 512-sample synthesis buffer from the quarter-wave components of both
 * transforms: the even transform supplies the current block and the odd
 * transform becomes the overlap history for the next one.
 */
void ac3_imdct_short(const float *coeff, float *out)
{
    float x1[128], x2[128];
    float y1[256], y2[256];
    int k;

    for (k = 0; k < 128; k++) {
        x1[k] = coeff[2 * k];
        x2[k] = coeff[2 * k + 1];
    }

    imdct256(x1, y1);
    imdct256(x2, y2);

    /* The 256-point transforms expose quarter-wave symmetry, so only the
     * first and third quarters of y1/y2 carry independent samples. */
    for (k = 0; k < 64; k++) {
        out[k] = -y1[191 - k];
        out[64 + k] = y1[k];
        out[128 + k] = -y1[63 - k];
        out[192 + k] = y1[128 + k];

        out[256 + k] = -y2[63 - k];
        out[320 + k] = y2[128 + k];
        out[384 + k] = y2[191 - k];
        out[448 + k] = -y2[k];
    }
}

void ac3_imdct_block(ac3_channel_t *ch, int short_block, float *out)
{
    float time[512];
    int n;

    if (short_block)
        ac3_imdct_short(ch->coeff, time);
    else
        ac3_imdct_long(ch->coeff, time);

    for (n = 0; n < AC3_BLOCK_SAMPLES; n++) {
        time[n] *= ac3_window[n];
        time[511 - n] *= ac3_window[n];
    }

    for (n = 0; n < AC3_BLOCK_SAMPLES; n++) {
        out[n] = 2.0f * (time[n] + ch->delay[n]);
        ch->delay[n] = time[256 + n];
    }
}
