/*
 * ac3dec - a self-contained ATSC A/52 (AC-3 and E-AC-3) audio decoder.
 *
 * The implementation follows the algorithms described in ATSC A/52 (and its
 * Annex E for E-AC-3) and is informed by the well-known liba52 decoder, but
 * the code, data structures and API are an independent implementation.  See
 * README.md and COPYING.
 *
 * E-AC-3 support covers the independent substream (up to 5.1 channels),
 * including the adaptive hybrid transform and spectral extension.
 * Syncframes that use enhanced coupling are detected and dropped instead of
 * being decoded incorrectly.
 *
 * Public API.  The interface deliberately mirrors the mpeg2dec / h264dec
 * style used by the sibling projects: open a decoder, send packets, receive
 * decoded audio frames (owned by the caller until released), flush, close.
 */
#ifndef AC3DEC_H
#define AC3DEC_H

#include <stdint.h>

/* Return codes (same values as the sibling video decoders). */
#define AC3_DEC_ERROR       -1   /* invalid API use or out of memory */
#define AC3_DEC_DEFAULT      0   /* no output available */
#define AC3_DEC_GOT_FRAME    1   /* a decoded audio frame is available */
#define AC3_DEC_GOT_HEADER   2   /* a new stream header has been parsed */
#define AC3_DEC_END          3   /* end of stream after flush */

/* Requested output configuration.  These match the channel modes defined
 * by A/52.  AC3_OUTPUT_SAME keeps the coded layout.  Other modes always
 * produce exactly the requested layout: sources that do not exist in the
 * stream are silent, and sources that the layout cannot carry are folded
 * into the nearest available channels. */
#define AC3_OUTPUT_SAME       0
#define AC3_OUTPUT_MONO       1
#define AC3_OUTPUT_STEREO     2
#define AC3_OUTPUT_3F         3
#define AC3_OUTPUT_2F1R       4
#define AC3_OUTPUT_3F1R       5
#define AC3_OUTPUT_2F2R       6
#define AC3_OUTPUT_3F2R       7
#define AC3_OUTPUT_CHANNEL1   8   /* first channel of a 1+1 stream */
#define AC3_OUTPUT_CHANNEL2   9   /* second channel of a 1+1 stream */
#define AC3_OUTPUT_DOLBY     10   /* Dolby Surround compatible Lt/Rt */

/* Flags that may be OR'ed into ac3_param_t::output. */
#define AC3_OUTPUT_LFE       16   /* keep the LFE channel when present */
#define AC3_OUTPUT_NO_LFE    32   /* drop the LFE channel */

/* Speaker map bits used by ac3_stream_info::channel_map and
 * ac3_frame_t::channel_map.  Channels are stored in ascending bit order:
 * L, R, C, LFE, LS, RS.  A single surround channel (2/1 and 3/1 modes)
 * is reported as AC3_SPEAKER_LS. */
#define AC3_SPEAKER_L        0x01
#define AC3_SPEAKER_R        0x02
#define AC3_SPEAKER_C        0x04
#define AC3_SPEAKER_LFE      0x08
#define AC3_SPEAKER_LS       0x10
#define AC3_SPEAKER_RS       0x20

/* Output sample formats. */
#define AC3_SAMPLE_F32       0
#define AC3_SAMPLE_S16       1

/* Stream parameters.  Updated at the beginning of every AC-3 syncframe
 * and reported through AC3_DEC_GOT_HEADER whenever they change. */
struct ac3_stream_info
{
    int sample_rate;        /* Hz (32000, 44100, 48000, or a lower rate) */
    int bit_rate;           /* bits per second */
    int frame_size;         /* syncframe size in bytes */
    int eac3;               /* 1 for E-AC-3 (Dolby Digital Plus) */
    int num_blocks;         /* audio blocks per syncframe (1, 2, 3 or 6) */
    int samples;            /* PCM samples per frame (num_blocks * 256) */
    int bsid;               /* bit stream identification (16 for E-AC-3) */
    int bsmod;              /* bit stream mode */
    int acmod;              /* audio coding mode, 0..7 */
    int lfeon;              /* LFE channel present */
    int channels;           /* coded full-bandwidth channels, 1..5 */
    int channel_map;        /* coded speaker map, LFE included if present */
    int frmsizecod;
    int halfrate;           /* 0, 1 or 2 (bsid 8, 9 or 10) */
    int dsurmod;            /* Dolby Surround mode, 0..3 */
    int cmixlev_code;       /* 2-bit centre mix level code, -1 if absent */
    int surmixlev_code;     /* 2-bit surround mix level code, -1 if absent */
    int dialnorm;           /* raw 5-bit dialogue normalisation word */
    int dialnorm_db;        /* -dialnorm, 0 when the field is reserved */
    int compr_present;      /* frame-level compression word present */
    int compr;              /* signed 8-bit compression word */
    int compr2_present;     /* second word for a 1+1 stream */
    int compr2;
    int output_channels;    /* channels produced for the current settings */
    int output_channel_map; /* speaker map of the produced audio */
};

typedef struct ac3_stream_info ac3_stream_info;

/* Input packet.  A packet may contain any number of bytes: whole
 * syncframes, partial syncframes, several syncframes, or a single byte.
 * The decoder buffers input internally.  The packet's timestamps are
 * attached to frames completed while that packet is being processed. */
struct ac3_packet_t
{
    const uint8_t* data;
    int len;
    int64_t pts;
    int64_t dts;
    void* opaque;
};

typedef struct ac3_packet_t ac3_packet_t;

/* Decoded audio frame.  A frame contains frame->samples samples per channel
 * (1536 for AC-3; num_blocks * 256 for E-AC-3).
 *
 * planar != 0:  planes[i] points at channel i, strides[i] is the byte
 *               distance between consecutive samples (2 or 4).
 * planar == 0:  planes[0] points at interleaved samples, strides[0] is the
 *               byte distance between two consecutive frames of channels.
 *
 * The decoder keeps the sample buffer alive until ac3_dec_frame_unref() is
 * called with the same decoder and frame.  dec_frame is an internal handle.
 */
struct ac3_frame_t
{
    void* planes[6];
    int strides[6];
    int num_channels;
    int samples;            /* 1536 */
    int sample_rate;
    int sample_format;      /* AC3_SAMPLE_* */
    int planar;             /* 1 = planar, 0 = interleaved */
    int channel_map;        /* output speaker map */
    int acmod;
    int lfeon;
    int64_t pts;
    int64_t dts;
    void* dec_frame;        /* internal frame handle */
    void* opaque;
};

typedef struct ac3_frame_t ac3_frame_t;

/* Decoder parameters.  A zero-initialised structure selects the defaults:
 * same channel layout as the stream, LFE kept for multichannel layouts,
 * 32-bit float planar output, DRC enabled with scale 1.0, no CRC check. */
struct ac3_param_t
{
    int output;             /* AC3_OUTPUT_* plus optional flags */
    int sample_format;      /* AC3_SAMPLE_F32 (default) or S16 */
    int interleaved;        /* 0 = planar (default), 1 = interleaved */
    int disable_drc;        /* 1 = do not apply dynamic range compression */
    int disable_dither;     /* 1 = do not generate dither for zero-bit bins */
    float drc_scale;        /* DRC exponent, 0 selects 1.0 */
    int check_crc;          /* 1 = verify the frame CRCs */
    int disable_level_adjust; /* 1 = do not attenuate downmixes */
    int apply_dialnorm;     /* 1 = apply dialogue normalisation */
    int drc_mode;           /* 0 = line mode (dynrng), 1 = RF mode */
};

typedef struct ac3_param_t ac3_param_t;

#ifdef AC3DECLIB_SHARED
#define AC3DEC_EXPORT __attribute__((visibility("default")))
#else
#define AC3DEC_EXPORT
#endif

AC3DEC_EXPORT void* ac3_dec_open(ac3_param_t* param);
AC3DEC_EXPORT int   ac3_dec_send_packet(void* decoder, ac3_packet_t* packet);
AC3DEC_EXPORT int   ac3_dec_recv_frame(void* decoder, ac3_stream_info* headerset,
                                       ac3_frame_t* out_frame);
AC3DEC_EXPORT int   ac3_dec_flush(void* decoder, ac3_stream_info* headerset,
                                  ac3_frame_t* out_frame);
AC3DEC_EXPORT void  ac3_dec_frame_unref(void* decoder, ac3_frame_t* out_frame);
AC3DEC_EXPORT void  ac3_dec_close(void* decoder);

/* Build information. */
AC3DEC_EXPORT const char* ac3_dec_version(void);
AC3DEC_EXPORT unsigned ac3_dec_cpu_flags(void);
AC3DEC_EXPORT const char* ac3_dec_cpu_name(void);

#endif /* AC3DEC_H */
