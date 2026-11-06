/*
 * Unit tests for the AC-3 inverse MDCT.
 *
 * The fast transforms are compared against a direct O(N^2) evaluation of
 * the A/52 inverse transform equation, so an error in the FFT twiddles or
 * in the de-interleaving permutation is caught immediately.
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "ac3_internal.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int failures;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                                     \
        }                                                                   \
    } while (0)

/* Direct inverse MDCT: N time samples from N/2 coefficients. */
static void imdct_direct(const float *x, int n, float *y)
{
    int i, k;

    for (i = 0; i < n; i++) {
        double sum = 0.0;
        for (k = 0; k < n / 2; k++)
            sum += (double)x[k] *
                   cos(M_PI / (2.0 * n) * (2.0 * i + 1 + n / 2) * (2.0 * k + 1));
        y[i] = (float)sum;
    }
}

static double max_abs_diff(const float *a, const float *b, int n)
{
    double m = 0.0;
    int i;

    for (i = 0; i < n; i++) {
        double d = fabs((double)a[i] - b[i]);
        if (d > m)
            m = d;
    }
    return m;
}

static double rms(const float *a, int n)
{
    double s = 0.0;
    int i;

    for (i = 0; i < n; i++)
        s += (double)a[i] * a[i];
    return sqrt(s / n);
}

static void fill_random(float *x, int n, unsigned *seed)
{
    int i;

    for (i = 0; i < n; i++) {
        *seed = *seed * 1103515245u + 12345u;
        x[i] = (float)((int)((*seed >> 8) % 2001) - 1000) / 1000.0f;
    }
}

static void test_long(void)
{
    static float coeff[256], fast[512], direct[512];
    unsigned seed = 1;
    double err, level;

    fill_random(coeff, 256, &seed);
    imdct_direct(coeff, 512, direct);
    ac3_imdct_long(coeff, fast);

    /* The fast transform uses the opposite sign convention. */
    for (int i = 0; i < 512; i++)
        direct[i] = -direct[i];

    level = rms(direct, 512);
    err = max_abs_diff(fast, direct, 512);
    CHECK(level > 0.1);
    CHECK(err < 1e-4 * level);
    if (err >= 1e-4 * level)
        fprintf(stderr, "  long: max err %.6e level %.6e\n", err, level);
}

static void test_short(void)
{
    static float coeff[256], fast[512], direct[512];
    static float even[128], odd[128], ne[256], no[256];
    unsigned seed = 7;
    double err, level;
    int i, k;

    fill_random(coeff, 256, &seed);
    for (i = 0; i < 128; i++) {
        even[i] = coeff[2 * i];
        odd[i] = coeff[2 * i + 1];
    }
    imdct_direct(even, 256, ne);
    imdct_direct(odd, 256, no);

    /* Expected synthesis buffer: the even transform fills the first half,
     * the odd transform becomes the overlap history. */
    for (k = 0; k < 64; k++) {
        direct[k] = ne[191 - k];
        direct[64 + k] = -ne[k];
        direct[128 + k] = ne[63 - k];
        direct[192 + k] = -ne[128 + k];

        direct[256 + k] = no[63 - k];
        direct[320 + k] = -no[128 + k];
        direct[384 + k] = -no[191 - k];
        direct[448 + k] = no[k];
    }

    ac3_imdct_short(coeff, fast);

    level = rms(direct, 512);
    err = max_abs_diff(fast, direct, 512);
    CHECK(level > 0.1);
    CHECK(err < 1e-4 * level);
    if (err >= 1e-4 * level)
        fprintf(stderr, "  short: max err %.6e level %.6e\n", err, level);
}

static void test_window(void)
{
    int i;

    /* The KBD window is positive and increases over its first 256 values. */
    CHECK(ac3_window[0] > 0.0f);
    CHECK(ac3_window[0] < ac3_window[128]);
    CHECK(ac3_window[128] < ac3_window[255]);
    for (i = 1; i < 256; i++)
        CHECK(ac3_window[i] >= ac3_window[i - 1]);
}

static void test_overlap_add(void)
{
    ac3_channel_t ch;
    static float out[256];
    int i;

    memset(&ch, 0, sizeof(ch));
    for (i = 0; i < 256; i++)
        ch.coeff[i] = (i == 32) ? 1.0f : 0.0f;

    ac3_imdct_block(&ch, 0, out);
    /* First block only sees its own window; all samples must be finite. */
    for (i = 0; i < 256; i++)
        CHECK(isfinite(out[i]));

    ac3_imdct_block(&ch, 0, out);
    for (i = 0; i < 256; i++)
        CHECK(isfinite(out[i]));
}

int main(void)
{
    ac3_tables_init();

    test_long();
    test_short();
    test_window();
    test_overlap_add();

    if (failures) {
        fprintf(stderr, "%d IMDCT test(s) failed\n", failures);
        return 1;
    }
    printf("test_imdct: all checks passed\n");
    return 0;
}
