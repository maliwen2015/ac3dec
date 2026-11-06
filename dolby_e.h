/*
 * The Dolby E decoding algorithm and tables in this file are derived from
 * the FFmpeg "dolby_e" decoder (Copyright (C) 2017 foo86), which is
 * distributed under the GNU Lesser General Public License, version 2.1 or
 * later.  This file is therefore licensed under the LGPL-2.1-or-later as
 * well; see COPYING.LGPL.
 */
/*
 * Dolby E decoder.
 *
 * Dolby E is a professional broadcast audio codec carried in AES3 /
 * SMPTE 337M.  This module is an independent implementation of the decoder
 * following the publicly documented reverse-engineered algorithm.
 */
#ifndef DOLBY_E_H
#define DOLBY_E_H

#include <stdint.h>

#include "dolby_e_compat.h"

#define MAX_PROG_CONF 23
#define MAX_PROGRAMS 8

typedef struct DolbyEHeaderInfo {
    int prog_conf;
    int nb_channels;
    int nb_programs;

    int fr_code;
    int fr_code_orig;

    int ch_size[MAX_CHANNELS];
    int mtd_ext_size;
    int meter_size;

    int rev_id[MAX_CHANNELS];
    int begin_gain[MAX_CHANNELS];
    int end_gain[MAX_CHANNELS];

    int multi_prog_warned;
    int output_channel_order;

    int sample_rate;
} DolbyEHeaderInfo;

typedef struct DBEContext {
    void *avctx;
    GetBitContext gb;

    const uint8_t *input;
    int input_size;

    int word_bits;
    int word_bytes;
    int key_present;

    DolbyEHeaderInfo metadata;

    uint8_t buffer[1024 * 3 + AV_INPUT_BUFFER_PADDING_SIZE];
} DBEContext;

int ff_dolby_e_convert_input(DBEContext *s, int nb_words, int key);
int ff_dolby_e_parse_header(DBEContext *s, const uint8_t *buf, int buf_size);

/* ------------------------------------------------------------------ */
/* public API                                                          */
/* ------------------------------------------------------------------ */

typedef struct dbe_decoder dbe_decoder;

/* Create a decoder.  Returns NULL on allocation failure. */
dbe_decoder *dbe_open(void);
void dbe_close(dbe_decoder *dec);

/*
 * Decode one Dolby E frame.  `data` must point at a complete raw Dolby E
 * frame as delivered by the S337M/AES3 demultiplexer.  On success the
 * decoder returns the number of channels (1..8) and stores planar float
 * samples (FRAME_SAMPLES per channel) in `out`; the caller owns the arrays.
 * Returns a negative value on error.
 */
int dbe_decode(dbe_decoder *dec, const uint8_t *data, int size,
               float *out[MAX_CHANNELS], int *channels, int *sample_rate);

/* Channel count of the last decoded frame. */
int dbe_channels(const dbe_decoder *dec);

/* ------------------------------------------------------------------ */
/* SMPTE 337M / AES3 demultiplexing                                    */
/* ------------------------------------------------------------------ */

/*
 * Extract the next Dolby E frame from an SMPTE 337M stream.  `pos` is the
 * search position and is advanced past the frame.  The (byte-swapped) raw
 * Dolby E frame is written to `out`.  Returns 1 on success, 0 at the end of
 * the stream, or a negative value on error.
 */
int dbe_s337m_next(const uint8_t *data, int size, int *pos,
                   uint8_t *out, int out_cap, int *out_size);

#endif /* DOLBY_E_H */
