/*
 * ac3dec internal definitions.
 *
 * The decoder is written from the ATSC A/52 specification.  The layout of
 * the decoding pipeline (exponents, bit allocation, coupling, rematrixing,
 * IMDCT and overlap-add) follows the algorithms popularised by liba52, but
 * this is an independent implementation.
 */
#ifndef AC3_INTERNAL_H
#define AC3_INTERNAL_H

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "../ac3dec.h"

#define AC3_MAX_FBW          5       /* 3/2 is the largest full-band layout */
#define AC3_MAX_CHANNELS     6       /* 5 full-band + LFE */
#define AC3_COEFFS           256     /* transform coefficients per block */
#define AC3_BLOCK_SAMPLES    256     /* new PCM samples per block */
#define AC3_FRAME_SAMPLES    1536    /* six blocks per syncframe */
#define AC3_MAX_FRAME_BYTES  3840
#define AC3_MAX_EAC3_FRAME_BYTES 4096
#define AC3_MAX_BLOCKS       6
#define AC3_MAX_BANDS        50
#define AC3_MAX_CPL_BANDS    18
#define AC3_MAX_CPL_SUBBANDS 18
#define AC3_MAX_DBA_SEGMENTS 8

#define AC3_MIN(a, b) ((a) < (b) ? (a) : (b))
#define AC3_MAX(a, b) ((a) > (b) ? (a) : (b))
#define AC3_CLIP(x, lo, hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))
#define AC3_ARRAY_ELEMS(a) (sizeof(a) / sizeof((a)[0]))

/* ------------------------------------------------------------------ */
/* bit reader                                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    const uint8_t *data;
    int size_bits;
    int pos;
    int error;
} ac3_bits_t;

static inline void ac3_bits_init(ac3_bits_t *b, const uint8_t *data, int size)
{
    b->data = data;
    b->size_bits = size * 8;
    b->pos = 0;
    b->error = 0;
}

static inline unsigned ac3_bits_get(ac3_bits_t *b, int n)
{
    unsigned v = 0;
    int p = b->pos;

    if (n <= 0)
        return 0;
    if (p + n > b->size_bits) {
        b->error = 1;
        b->pos = b->size_bits;
        return 0;
    }
    while (n--) {
        v = (v << 1) | ((b->data[p >> 3] >> (7 - (p & 7))) & 1);
        p++;
    }
    b->pos = p;
    return v;
}

static inline int32_t ac3_bits_get_signed(ac3_bits_t *b, int n)
{
    unsigned v;

    if (n <= 0)
        return 0;
    v = ac3_bits_get(b, n);
    if (v & (1u << (n - 1)))
        v |= ~((1u << n) - 1u);
    return (int32_t)v;
}

static inline void ac3_bits_skip(ac3_bits_t *b, int n)
{
    if (n <= 0)
        return;
    if (b->pos + n > b->size_bits) {
        b->error = 1;
        b->pos = b->size_bits;
        return;
    }
    b->pos += n;
}

/* ------------------------------------------------------------------ */
/* persistent codec state                                              */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t exp[AC3_COEFFS];
    int8_t bap[AC3_COEFFS];
    int endmant;                 /* number of mantissas for this channel */
    float delay[AC3_BLOCK_SAMPLES];   /* overlap-add history */
    float coeff[AC3_COEFFS];
} ac3_channel_t;

typedef struct {
    uint8_t bai;                 /* fine SNR offset and fine gain code */
    uint8_t deltbae;             /* 0 reuse, 1 new, 2 none, 3 reserved */
    int8_t deltba[AC3_MAX_BANDS];
} ac3_bainfo_t;

typedef struct {
    ac3_channel_t ch[AC3_MAX_CHANNELS];   /* 0..4 fbw, 5 = LFE */
    ac3_bainfo_t ba[AC3_MAX_FBW];
    ac3_bainfo_t cplba;
    ac3_bainfo_t lfeba;

    int chincpl;                 /* bit mask of coupled fbw channels */
    int phsflginu;
    int cpl_start;               /* first coupling mantissa */
    int cpl_end;                 /* one past the last coupling mantissa */
    int cpl_strtbnd;             /* band index used by bit allocation */
    int cpl_nsub;                /* number of coupling sub-bands */
    int cpl_nbnd;                /* number of coupling bands */
    uint32_t cplbndstrc;         /* sub-band merge flags */
    float cplco[AC3_MAX_FBW][AC3_MAX_CPL_BANDS];
    uint8_t cpl_exp[AC3_COEFFS];
    int8_t cpl_bap[AC3_COEFFS];
    uint8_t cpl_band_struct[AC3_MAX_CPL_SUBBANDS];

    int rematflg;                /* rematrix flags, one bit per band */

    int bai;                     /* 11-bit bit allocation parameters */
    int csnroffst;               /* coarse SNR offset */
    int cplfleak;                /* coupling fast leak initial value */
    int cplsleak;                /* coupling slow leak initial value */

    uint32_t dither_state;       /* 16-bit LFSR for bap == 0 dither */

    int blksw[AC3_MAX_FBW];      /* current block's block-switch flags */
    float dynrng[AC3_MAX_CHANNELS]; /* current block's DRC gains */
    int drc_enabled;
    int dither_enabled;
    int drc_rf;
    float drc_scale;
    float compr_gain[2];

    /* E-AC-3 per-frame state that is refined block by block */
    int eac3;
    int cpl_phase[AC3_MAX_CPL_BANDS];   /* coupling phase flags */
    int phase_flags_in_use;
    int first_cpl_coords[AC3_MAX_FBW];  /* coordinates not yet read */
    int first_cpl_leak;

    /* E-AC-3 spectral extension state */
    int first_spx_coords[AC3_MAX_CHANNELS];
    uint8_t spx_band_struct[17];
    uint8_t spx_band_sizes[17];
    int num_spx_bands;
    int spx_dst_start;
    int spx_src_start;
    int spx_dst_end;
    float spx_noise_blend[AC3_MAX_FBW][17];
    float spx_signal_blend[AC3_MAX_FBW][17];

    /* E-AC-3 adaptive hybrid transform mantissas: [channel][bin][block] */
    int32_t pre_mant[AC3_MAX_CHANNELS + 1][AC3_COEFFS][AC3_MAX_BLOCKS];

    /* Reference-compatible noise generator (shared by dither, AHT and SPX) */
    uint32_t lfg_state[64];
    unsigned int lfg_index;

    /* per-block scratch, kept here to avoid a large stack frame */
    float cpl_coeff[AC3_COEFFS];
    float fbw_coeff[AC3_MAX_CHANNELS][AC3_COEFFS];
} ac3_codec_t;

/* ------------------------------------------------------------------ */
/* frame header                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    int sample_rate;
    int bit_rate;
    int frame_size;
    int fscod;
    int frmsizecod;
    int bsid;
    int bsmod;
    int acmod;
    int lfeon;
    int halfrate;
    int nfchans;
    int channel_map;

    int dsurmod;
    int cmix_code;               /* -1 when not present */
    int smix_code;               /* -1 when not present */
    float clev;
    float slev;

    int dialnorm[2];
    int compr_present[2];
    int compr[2];
    int langcod_present[2];
    int langcod[2];
    int audprodie[2];
    int mixlevel[2];
    int roomtyp[2];

    int copyrightb;
    int origbs;
    int timecod1e, timecod1;
    int timecod2e, timecod2;
    int addbsie, addbsil;
    int bits_used;               /* bit position after the BSI */

    int dialnorm_db;

    /* ---- E-AC-3 ---- */
    int eac3;
    int num_blocks;              /* audio blocks in this syncframe */
    int samples;                 /* PCM samples per frame (1536 or 256*n) */
    int frame_type;              /* 0 independent, 1 dependent, 2 AC-3 convert */
    int substreamid;

    int block_switch_syntax;
    int dither_flag_syntax;
    int bit_allocation_syntax;
    int fast_gain_syntax;
    int dba_syntax;
    int skip_syntax;
    int snr_offset_strategy;

    /* exp_strategy is indexed [block][channel]; channel 0 is the coupling
     * channel, 1..nfchans are the full-band channels. */
    int exp_strategy[AC3_MAX_BLOCKS][AC3_MAX_CHANNELS];
    int lfe_exp_strategy[AC3_MAX_BLOCKS];
    int cpl_in_use[AC3_MAX_BLOCKS];
    int cpl_strategy_exists[AC3_MAX_BLOCKS];
    int channel_uses_aht[AC3_MAX_CHANNELS + 1];
    int channel_uses_spx[AC3_MAX_CHANNELS + 1];
    int spx_atten_code[AC3_MAX_CHANNELS + 1];
    int spx_in_use;

    int snr_offset[AC3_MAX_CHANNELS];   /* [0..nfchans-1] fbw, [nfchans] LFE */
    int fast_gain[AC3_MAX_CHANNELS];
    int cpl_snr_offset;
    int cpl_fast_gain;
    int slow_decay, fast_decay, slow_gain, db_per_bit, floor;
    int cpl_fast_leak, cpl_slow_leak;
    int sr_shift;

    int dba_mode[AC3_MAX_CHANNELS];     /* full-band channels only */
    int dba_nsegs[AC3_MAX_CHANNELS];
    uint8_t dba_offsets[AC3_MAX_CHANNELS][AC3_MAX_DBA_SEGMENTS];
    uint8_t dba_lengths[AC3_MAX_CHANNELS][AC3_MAX_DBA_SEGMENTS];
    uint8_t dba_values[AC3_MAX_CHANNELS][AC3_MAX_DBA_SEGMENTS];
    int cpl_dba_mode, cpl_dba_nsegs;
    uint8_t cpl_dba_offsets[AC3_MAX_DBA_SEGMENTS];
    uint8_t cpl_dba_lengths[AC3_MAX_DBA_SEGMENTS];
    uint8_t cpl_dba_values[AC3_MAX_DBA_SEGMENTS];
} ac3_header_t;

/* ------------------------------------------------------------------ */
/* downmix                                                             */
/* ------------------------------------------------------------------ */

typedef struct {
    int out_map;
    int out_count;
    int out_roles[AC3_MAX_CHANNELS];
    int in_count;                /* full-band input channels */
    int have_lfe;
    float matrix[AC3_MAX_CHANNELS][AC3_MAX_CHANNELS]; /* [out][in] */
    float gain;
} ac3_mix_t;

/* ------------------------------------------------------------------ */
/* output frame and event queue                                        */
/* ------------------------------------------------------------------ */

typedef struct ac3_out_frame {
    int ref;
    ac3_frame_t pub;
    void *mem;
} ac3_out_frame_t;

typedef struct ac3_event {
    int type;                    /* AC3_DEC_GOT_HEADER / AC3_DEC_GOT_FRAME */
    ac3_stream_info info;
    ac3_out_frame_t *frame;
    struct ac3_event *next;
} ac3_event_t;

typedef struct ac3_decoder {
    ac3_param_t param;
    ac3_codec_t codec;
    int codec_ready;

    ac3_stream_info info;
    int info_valid;

    int64_t pkt_pts;
    int64_t pkt_dts;
    void *pkt_opaque;

    uint8_t *in;
    int in_cap;
    int in_len;
    int in_pos;

    int flushing;
    int end_reached;
    int error_count;

    ac3_event_t *ev_head;
    ac3_event_t *ev_tail;
} ac3_decoder_t;

/* ------------------------------------------------------------------ */
/* tables and one-time initialisation                                  */
/* ------------------------------------------------------------------ */

void ac3_tables_init(void);

extern float ac3_scale_factor[25];
extern float ac3_window[AC3_BLOCK_SAMPLES];
extern float ac3_fft_cos[64];
extern float ac3_fft_sin[64];
extern float ac3_imdct1_cos[128];
extern float ac3_imdct1_sin[128];
extern float ac3_imdct2_cos[64];
extern float ac3_imdct2_sin[64];
extern uint8_t ac3_bitrev7[128];
extern uint8_t ac3_bitrev6[64];
extern uint16_t ac3_dither_step[256];
extern uint16_t ac3_crc_table[256];

extern const int8_t ac3_bap_table[64];
extern const int16_t ac3_hth_table[3][AC3_MAX_BANDS];
extern const int ac3_bndtab[30];
extern const int8_t ac3_latab[256];
extern const uint8_t ac3_cpl_bndtab[16];
extern const int ac3_remat_band[4];
extern const int ac3_slowgain[4];
extern const int ac3_dbpbtab[4];
extern const int ac3_floortab[8];
extern const float ac3_clev_table[4];
extern const float ac3_slev_table[4];
extern const int ac3_acmod_nch[8];
extern const int ac3_acmod_map[8];
extern const int ac3_acmod_speakers[8][5];

/* E-AC-3 bit allocation / exponent tables (ATSC A/52 Annex E) */
extern const uint8_t ac3_band_start_tab[AC3_MAX_BANDS + 1];
extern uint8_t ac3_bin_to_band_tab[253];
extern const uint8_t ac3_log_add_tab[260];
extern const int16_t ac3_hearing_threshold_tab[AC3_MAX_BANDS][3];
extern const uint8_t ac3_ff_bap_tab[64];
extern const uint8_t ac3_slow_decay_tab[4];
extern const uint8_t ac3_fast_decay_tab[4];
extern const uint16_t ac3_slow_gain_tab[4];
extern const uint16_t ac3_db_per_bit_tab[4];
extern const int16_t ac3_floor_tab[8];
extern const uint16_t ac3_fast_gain_tab[8];
extern const uint8_t ac3_eac3_frm_expstr[32][6];
extern const uint8_t ac3_eac3_default_cpl_band_struct[18];
extern const uint8_t ac3_eac3_default_spx_band_struct[17];
extern const int8_t ac3_eac3_bap_map[16];

/* E-AC-3 AHT / SPX tables (eac3_tables.c) */
extern const uint8_t ac3_eac3_bits_vs_hebap[20];
extern const int16_t ac3_eac3_gaq_remap_1[12];
extern const int16_t ac3_eac3_gaq_remap_2_4_a[9][2];
extern const int16_t ac3_eac3_gaq_remap_2_4_b[9][2];
extern const int16_t (*const ac3_eac3_mantissa_vq[8])[6];
extern const float ac3_eac3_spx_atten_tab[32][3];
extern const uint8_t ac3_eac3_hebap_tab[64];
extern const uint8_t ac3_ungroup_3_in_5_bits_tab[32][3];

void ac3_eac3_lfg_init(ac3_codec_t *c);
float ac3_eac3_dither_coeff(ac3_codec_t *c, int exp);

/* ------------------------------------------------------------------ */
/* modules                                                             */
/* ------------------------------------------------------------------ */

int ac3_parse_header(const uint8_t *data, int size, ac3_header_t *h);
int ac3_probe_frame_size(const uint8_t *data, int size);
int ac3_frame_crc_ok(const uint8_t *data, int size);

/* Shared quantiser state and block primitives (ac3_block.c). */
typedef struct {
    float q1[2];
    float q2[2];
    float q4;
    int q1_ptr;
    int q2_ptr;
    int q4_ptr;
} ac3_quant_t;

float ac3_scale_for_exp(int exp);
float ac3_drc_word_to_gain(int word);
float ac3_apply_drc(ac3_codec_t *c, float gain);
int ac3_parse_exponents(ac3_bits_t *br, int expstr, int ngrps,
                        int exponent, uint8_t *dest);
int ac3_parse_delta(ac3_bits_t *br, int8_t *deltba);
float ac3_quant_coeff(ac3_bits_t *br, ac3_quant_t *q, int bap, int exp);
void ac3_decode_channel(ac3_codec_t *c, ac3_channel_t *ch,
                        ac3_bits_t *br, ac3_quant_t *q, int dither, int end);
void ac3_decode_coupling_channel(ac3_codec_t *c, const ac3_header_t *h,
                                 ac3_bits_t *br, ac3_quant_t *q,
                                 const int dithflag[AC3_MAX_FBW]);
void ac3_apply_rematrix(ac3_codec_t *c);

void ac3_bit_allocate(ac3_bainfo_t *ba, int bndstart, int start, int end,
                      int fastleak, int slowleak,
                      int bai, int csnroffst, int fscod, int halfrate,
                      const uint8_t *exp, int8_t *bap,
                      const int8_t *deltba);

void ac3_imdct_long(const float *coeff, float *out512);
void ac3_imdct_short(const float *coeff, float *out512);
void ac3_imdct_block(ac3_channel_t *ch, int short_block, float *out256);

void ac3_mix_init(ac3_mix_t *mix, const ac3_header_t *h,
                  const ac3_param_t *param);
void ac3_mix_block(const ac3_mix_t *mix, const float pcm[AC3_MAX_CHANNELS][AC3_BLOCK_SAMPLES],
                   float out[AC3_MAX_CHANNELS][AC3_BLOCK_SAMPLES]);

int ac3_decode_block(ac3_codec_t *c, const ac3_header_t *h, ac3_bits_t *br);
int ac3_decode_block_eac3(ac3_codec_t *c, ac3_header_t *h, ac3_bits_t *br,
                          int blk);

int ac3_decode_syncframe(ac3_decoder_t *dec, const uint8_t *data, int size,
                         int64_t pts, int64_t dts, void *opaque);

#endif /* AC3_INTERNAL_H */
