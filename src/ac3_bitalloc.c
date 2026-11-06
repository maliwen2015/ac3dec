/*
 * AC-3 parametric bit allocation (A/52 section 7.2.2).
 *
 * The routine computes one bit-allocation pointer per transform
 * coefficient from the decoded exponents, the five masking parameters
 * and the optional delta bit allocation.
 */
#include "ac3_internal.h"

static int floor_div512(int v)
{
    return (v >= 0) ? (v / 512) : -((-v + 511) / 512);
}

static int bap_lookup(int index)
{
    if (index < -63)
        return 16;
    if (index > 0)
        return 0;
    return ac3_bap_table[index + 63];
}

void ac3_bit_allocate(ac3_bainfo_t *ba, int bndstart, int start, int end,
                      int fastleak, int slowleak,
                      int bai, int csnroffst, int fscod, int halfrate,
                      const uint8_t *exp, int8_t *bap,
                      const int8_t *deltba)
{
    static const int8_t zero_delta[AC3_MAX_BANDS] = {0};
    const int16_t *hth;
    int fdecay, fgain, sdecay, sgain, dbknee, floor;
    int snroffset;
    int i, j;
    int psd, mask, lowcomp;

    if (end <= start)
        return;

    if (deltba == NULL || ba->deltbae == 2)
        deltba = zero_delta;

    fdecay = (63 + 20 * ((bai >> 7) & 3)) >> halfrate;
    fgain = 128 + 128 * (ba->bai & 7);
    sdecay = (15 + 2 * (bai >> 9)) >> halfrate;
    sgain = ac3_slowgain[(bai >> 5) & 3];
    dbknee = ac3_dbpbtab[(bai >> 3) & 3];
    hth = ac3_hth_table[fscod];
    floor = ac3_floortab[bai & 7];
    snroffset = 960 - 64 * csnroffst - 4 * (ba->bai >> 3) + floor;
    floor >>= 5;

#define UPDATE_LEAK()                                                   \
    do {                                                                \
        fastleak += fdecay;                                             \
        if (fastleak > psd + fgain)                                     \
            fastleak = psd + fgain;                                     \
        slowleak += sdecay;                                             \
        if (slowleak > psd + sgain)                                     \
            slowleak = psd + sgain;                                     \
    } while (0)

#define COMPUTE_MASK(band)                                              \
    do {                                                                \
        if (psd > dbknee)                                               \
            mask -= (psd - dbknee) >> 2;                                \
        if (mask > hth[(band) >> halfrate])                             \
            mask = hth[(band) >> halfrate];                             \
        mask -= snroffset + 128 * deltba[band];                         \
        mask = (mask > 0) ? 0 : ((-mask) >> 5);                         \
        mask -= floor;                                                  \
    } while (0)

    i = start == 0 ? 0 : bndstart;
    j = start;

    if (start == 0) {
        /* First seven bins are treated individually. */
        lowcomp = 0;
        j = end - 1;
        do {
            if (i < j) {
                if (exp[i + 1] == exp[i] - 2)
                    lowcomp = 384;
                else if (lowcomp && exp[i + 1] > exp[i])
                    lowcomp -= 64;
            }
            psd = 128 * exp[i];
            mask = psd + fgain + lowcomp;
            COMPUTE_MASK(i);
            bap[i] = (int8_t)bap_lookup(mask + 4 * exp[i]);
            i++;
        } while ((i < 3) || ((i < 7) && (exp[i] > exp[i - 1])));

        fastleak = psd + fgain;
        slowleak = psd + sgain;

        while (i < 7) {
            if (i < j) {
                if (exp[i + 1] == exp[i] - 2)
                    lowcomp = 384;
                else if (lowcomp && exp[i + 1] > exp[i])
                    lowcomp -= 64;
            }
            psd = 128 * exp[i];
            UPDATE_LEAK();
            mask = (fastleak + lowcomp < slowleak)
                   ? fastleak + lowcomp : slowleak;
            COMPUTE_MASK(i);
            bap[i] = (int8_t)bap_lookup(mask + 4 * exp[i]);
            i++;
        }

        if (end == 7)              /* LFE channel */
            return;

        do {
            if (exp[i + 1] == exp[i] - 2)
                lowcomp = 320;
            else if (lowcomp && exp[i + 1] > exp[i])
                lowcomp -= 64;
            psd = 128 * exp[i];
            UPDATE_LEAK();
            mask = (fastleak + lowcomp < slowleak)
                   ? fastleak + lowcomp : slowleak;
            COMPUTE_MASK(i);
            bap[i] = (int8_t)bap_lookup(mask + 4 * exp[i]);
            i++;
        } while (i < 20);

        while (lowcomp > 128) {
            lowcomp -= 128;
            psd = 128 * exp[i];
            UPDATE_LEAK();
            mask = (fastleak + lowcomp < slowleak)
                   ? fastleak + lowcomp : slowleak;
            COMPUTE_MASK(i);
            bap[i] = (int8_t)bap_lookup(mask + 4 * exp[i]);
            i++;
        }
        j = i;
    }

    do {
        int startband = j;
        int endband = AC3_MIN(ac3_bndtab[i - 20], end);
        int next, delta;

        psd = 128 * exp[j++];
        while (j < endband) {
            next = 128 * exp[j++];
            delta = next - psd;
            switch (floor_div512(delta)) {
            case -6:
            case -5:
            case -4:
            case -3:
            case -2:
                psd = next;
                break;
            case -1:
                psd = next + ac3_latab[(-delta) >> 1];
                break;
            case 0:
                psd += ac3_latab[delta >> 1];
                break;
            default:
                break;
            }
        }

        UPDATE_LEAK();
        mask = (fastleak < slowleak) ? fastleak : slowleak;
        COMPUTE_MASK(i);
        i++;
        j = startband;
        do {
            bap[j] = (int8_t)bap_lookup(mask + 4 * exp[j]);
        } while (++j < endband);
    } while (j < end);

#undef UPDATE_LEAK
#undef COMPUTE_MASK
}
