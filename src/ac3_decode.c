/*
 * Public decoder API, input buffering, syncframe orchestration and PCM
 * output.
 */
#include <limits.h>
#include "ac3_internal.h"

#define AC3_VERSION_STRING "ac3dec 1.2"

static float drc_word_gain(int word)
{
    int top = word >> 5;
    int low = word & 0x1f;

    return (float)(low | 0x20) * (float)ldexp(1.0, top - 5);
}

static int find_sync(const uint8_t *data, int len)
{
    int i;

    for (i = 0; i + 1 < len; i++)
        if (data[i] == 0x0b && data[i + 1] == 0x77)
            return i;
    return -1;
}

static int enqueue_event(ac3_decoder_t *dec, int type,
                         const ac3_stream_info *info, ac3_out_frame_t *frame)
{
    ac3_event_t *e = (ac3_event_t *)calloc(1, sizeof(*e));

    if (e == NULL)
        return -1;
    e->type = type;
    if (info)
        e->info = *info;
    e->frame = frame;
    if (dec->ev_tail)
        dec->ev_tail->next = e;
    else
        dec->ev_head = e;
    dec->ev_tail = e;
    return 0;
}

static int pop_event(ac3_decoder_t *dec, ac3_stream_info *info,
                     ac3_frame_t *frame)
{
    ac3_event_t *e = dec->ev_head;

    if (e == NULL)
        return AC3_DEC_DEFAULT;
    dec->ev_head = e->next;
    if (dec->ev_head == NULL)
        dec->ev_tail = NULL;

    if (e->type == AC3_DEC_GOT_HEADER) {
        if (info)
            *info = e->info;
        free(e);
        return AC3_DEC_GOT_HEADER;
    }
    if (frame)
        *frame = e->frame->pub;
    free(e);
    return AC3_DEC_GOT_FRAME;
}

static ac3_out_frame_t *out_frame_alloc(const ac3_header_t *h,
                                        const ac3_mix_t *mix,
                                        const ac3_param_t *param,
                                        int samples,
                                        int64_t pts, int64_t dts,
                                        void *opaque)
{
    ac3_out_frame_t *f;
    int sample_size = (param->sample_format == AC3_SAMPLE_S16) ? 2 : 4;
    size_t total;
    int i;

    if (mix->out_count < 1 || mix->out_count > AC3_MAX_CHANNELS)
        return NULL;

    f = (ac3_out_frame_t *)calloc(1, sizeof(*f));
    if (f == NULL)
        return NULL;

    total = (size_t)mix->out_count * (size_t)samples * (size_t)sample_size;
    f->mem = malloc(total);
    if (f->mem == NULL) {
        free(f);
        return NULL;
    }
    f->ref = 1;

    f->pub.num_channels = mix->out_count;
    f->pub.samples = samples;
    f->pub.sample_rate = h->sample_rate;
    f->pub.sample_format = param->sample_format;
    f->pub.planar = !param->interleaved;
    f->pub.channel_map = mix->out_map;
    f->pub.acmod = h->acmod;
    f->pub.lfeon = h->lfeon;
    f->pub.pts = pts;
    f->pub.dts = dts;
    f->pub.opaque = opaque;
    f->pub.dec_frame = f;

    if (f->pub.planar) {
        for (i = 0; i < mix->out_count; i++) {
            f->pub.planes[i] = (uint8_t *)f->mem +
                               (size_t)i * (size_t)samples * sample_size;
            f->pub.strides[i] = sample_size;
        }
    } else {
        f->pub.planes[0] = f->mem;
        f->pub.strides[0] = mix->out_count * sample_size;
    }
    return f;
}

static int16_t float_to_s16(float v)
{
    long s;

    if (v > 1.0f)
        v = 1.0f;
    else if (v < -1.0f)
        v = -1.0f;
    s = lrintf(v * 32768.0f);
    if (s > 32767)
        s = 32767;
    else if (s < -32768)
        s = -32768;
    return (int16_t)s;
}

static void out_frame_write(ac3_out_frame_t *f,
                            const float pcm[AC3_MAX_CHANNELS][AC3_BLOCK_SAMPLES],
                            int nch, int block)
{
    int offset = block * AC3_BLOCK_SAMPLES;
    int ch, n;

    if (f->pub.sample_format == AC3_SAMPLE_F32) {
        if (f->pub.planar) {
            for (ch = 0; ch < nch; ch++) {
                float *dst = (float *)f->pub.planes[ch] + offset;
                memcpy(dst, pcm[ch], AC3_BLOCK_SAMPLES * sizeof(float));
            }
        } else {
            float *dst = (float *)f->pub.planes[0] +
                         (size_t)offset * nch;
            for (n = 0; n < AC3_BLOCK_SAMPLES; n++)
                for (ch = 0; ch < nch; ch++)
                    dst[n * nch + ch] = pcm[ch][n];
        }
    } else {
        if (f->pub.planar) {
            for (ch = 0; ch < nch; ch++) {
                int16_t *dst = (int16_t *)f->pub.planes[ch] + offset;
                for (n = 0; n < AC3_BLOCK_SAMPLES; n++)
                    dst[n] = float_to_s16(pcm[ch][n]);
            }
        } else {
            int16_t *dst = (int16_t *)f->pub.planes[0] +
                           (size_t)offset * nch;
            for (n = 0; n < AC3_BLOCK_SAMPLES; n++)
                for (ch = 0; ch < nch; ch++)
                    dst[n * nch + ch] = float_to_s16(pcm[ch][n]);
        }
    }
}

static void fill_stream_info(ac3_stream_info *info, const ac3_header_t *h,
                             const ac3_mix_t *mix)
{
    memset(info, 0, sizeof(*info));
    info->sample_rate = h->sample_rate;
    info->bit_rate = h->bit_rate;
    info->frame_size = h->frame_size;
    info->eac3 = h->eac3;
    info->num_blocks = h->num_blocks;
    info->samples = h->samples;
    info->bsid = h->bsid;
    info->bsmod = h->bsmod;
    info->acmod = h->acmod;
    info->lfeon = h->lfeon;
    info->channels = h->nfchans;
    info->channel_map = h->channel_map;
    info->frmsizecod = h->frmsizecod;
    info->halfrate = h->halfrate;
    info->dsurmod = h->dsurmod;
    info->cmixlev_code = h->cmix_code;
    info->surmixlev_code = h->smix_code;
    info->dialnorm = h->dialnorm[0];
    info->dialnorm_db = h->dialnorm_db;
    info->compr_present = h->compr_present[0];
    info->compr = h->compr[0];
    info->compr2_present = h->compr_present[1];
    info->compr2 = h->compr[1];
    info->output_channels = mix->out_count;
    info->output_channel_map = mix->out_map;
}

int ac3_decode_syncframe(ac3_decoder_t *dec, const uint8_t *data, int size,
                         int64_t pts, int64_t dts, void *opaque)
{
    ac3_header_t h;
    ac3_codec_t codec;
    ac3_mix_t mix;
    ac3_out_frame_t *frame;
    ac3_bits_t br;
    float pcm[AC3_MAX_CHANNELS][AC3_BLOCK_SAMPLES];
    float mixed[AC3_MAX_CHANNELS][AC3_BLOCK_SAMPLES];
    ac3_stream_info info;
    int blk, ch;

    if (ac3_parse_header(data, size, &h) < 0)
        return -1;

    /* Dependent substreams add channels to a preceding independent frame.
     * This decoder outputs the independent substream only. */
    if (h.eac3 && h.frame_type == 1)
        return 0;

    codec = dec->codec;
    codec.eac3 = h.eac3;
    for (ch = 0; ch < AC3_MAX_CHANNELS; ch++)
        codec.dynrng[ch] = 1.0f;
    for (ch = 0; ch < AC3_MAX_FBW; ch++)
        codec.ba[ch].deltbae = 2;
    codec.cplba.deltbae = 2;
    codec.lfeba.deltbae = 2;
    codec.drc_enabled = !dec->param.disable_drc;
    codec.dither_enabled = !dec->param.disable_dither;
    codec.drc_rf = dec->param.drc_mode != 0;
    codec.drc_scale = (dec->param.drc_scale > 0.0f)
                      ? dec->param.drc_scale : 1.0f;
    /* E-AC-3 carries heavy compression words, not the AC-3 compr word. */
    codec.compr_gain[0] = (!h.eac3 && h.compr_present[0])
                          ? drc_word_gain(h.compr[0]) : 1.0f;
    codec.compr_gain[1] = (!h.eac3 && h.compr_present[1])
                          ? drc_word_gain(h.compr[1]) : 1.0f;

    if (h.eac3) {
        for (ch = 0; ch < AC3_MAX_FBW; ch++)
            codec.first_cpl_coords[ch] = 1;
        codec.first_cpl_leak = 1;
        memset(codec.cpl_phase, 0, sizeof(codec.cpl_phase));
        codec.chincpl = 0;
        codec.rematflg = 0;
        codec.cpl_nbnd = 0;
        codec.cplbndstrc = 0;
        codec.phase_flags_in_use = 0;
        for (ch = 0; ch < AC3_MAX_CHANNELS; ch++)
            codec.first_spx_coords[ch] = 1;
    }

    ac3_mix_init(&mix, &h, &dec->param);
    frame = out_frame_alloc(&h, &mix, &dec->param, h.samples,
                            pts, dts, opaque);
    if (frame == NULL)
        return -1;

    ac3_bits_init(&br, data, size);
    ac3_bits_skip(&br, h.bits_used);

    for (blk = 0; blk < h.num_blocks; blk++) {
        if (h.eac3) {
            if (ac3_decode_block_eac3(&codec, &h, &br, blk) < 0)
                goto fail;
        } else {
            if (ac3_decode_block(&codec, &h, &br) < 0)
                goto fail;
        }

        for (ch = 0; ch < h.nfchans; ch++)
            ac3_imdct_block(&codec.ch[ch], codec.blksw[ch], pcm[ch]);
        if (h.lfeon)
            ac3_imdct_block(&codec.ch[5], 0, pcm[5]);
        else
            memset(pcm[5], 0, sizeof(pcm[5]));

        ac3_mix_block(&mix, (const float (*)[AC3_BLOCK_SAMPLES])pcm, mixed);
        out_frame_write(frame, (const float (*)[AC3_BLOCK_SAMPLES])mixed,
                        mix.out_count, blk);
    }

    dec->codec = codec;
    dec->codec_ready = 1;

    fill_stream_info(&info, &h, &mix);
    if (!dec->info_valid ||
        memcmp(&info, &dec->info, sizeof(info)) != 0) {
        if (enqueue_event(dec, AC3_DEC_GOT_HEADER, &info, NULL) < 0)
            goto fail;
        dec->info = info;
        dec->info_valid = 1;
    }
    if (enqueue_event(dec, AC3_DEC_GOT_FRAME, NULL, frame) < 0)
        goto fail;
    return 0;

fail:
    free(frame->mem);
    free(frame);
    return -1;
}

static int append_input(ac3_decoder_t *dec, const uint8_t *data, int len)
{
    int need;
    uint8_t *p;

    if (len <= 0)
        return 0;
    if (len > INT_MAX - dec->in_len)
        return -1;
    need = dec->in_len + len;
    if (need > dec->in_cap) {
        int cap = dec->in_cap ? dec->in_cap : 8192;
        while (cap < need) {
            if (cap > INT_MAX / 2) {
                cap = need;
                break;
            }
            cap *= 2;
        }
        p = (uint8_t *)realloc(dec->in, (size_t)cap);
        if (p == NULL)
            return -1;
        dec->in = p;
        dec->in_cap = cap;
    }
    memcpy(dec->in + dec->in_len, data, (size_t)len);
    dec->in_len += len;
    return 0;
}

static void process_buffer(ac3_decoder_t *dec, int flush)
{
    for (;;) {
        int avail = dec->in_len - dec->in_pos;
        int off, frame_size;

        if (avail < 2)
            break;

        off = find_sync(dec->in + dec->in_pos, avail);
        if (off < 0) {
            /* Keep the last byte: it might be the first half of a syncword. */
            dec->in_pos = dec->in_len > 0 ? dec->in_len - 1 : 0;
            break;
        }
        dec->in_pos += off;
        avail = dec->in_len - dec->in_pos;
        if (avail < 7)
            break;

        frame_size = ac3_probe_frame_size(dec->in + dec->in_pos, avail);
        if (frame_size <= 0) {
            dec->in_pos++;
            continue;
        }
        if (avail < frame_size) {
            if (flush)
                dec->in_pos = dec->in_len;
            break;
        }

        if (dec->param.check_crc &&
            !ac3_frame_crc_ok(dec->in + dec->in_pos, frame_size)) {
            dec->error_count++;
        } else if (ac3_decode_syncframe(dec, dec->in + dec->in_pos, frame_size,
                                        dec->pkt_pts, dec->pkt_dts,
                                        dec->pkt_opaque) < 0) {
            dec->error_count++;
        }
        dec->in_pos += frame_size;
    }

    if (dec->in_pos > 0) {
        int left = dec->in_len - dec->in_pos;

        if (left > 0)
            memmove(dec->in, dec->in + dec->in_pos, (size_t)left);
        dec->in_len = left;
        dec->in_pos = 0;
    }
}

void *ac3_dec_open(ac3_param_t *param)
{
    ac3_decoder_t *dec;

    ac3_tables_init();

    dec = (ac3_decoder_t *)calloc(1, sizeof(*dec));
    if (dec == NULL)
        return NULL;

    if (param)
        dec->param = *param;
    if (dec->param.sample_format != AC3_SAMPLE_S16)
        dec->param.sample_format = AC3_SAMPLE_F32;
    dec->param.interleaved = dec->param.interleaved ? 1 : 0;
    if (dec->param.drc_scale <= 0.0f)
        dec->param.drc_scale = 1.0f;

    dec->codec.dither_state = 1;
    ac3_eac3_lfg_init(&dec->codec);
    dec->in_cap = 8192;
    dec->in = (uint8_t *)malloc((size_t)dec->in_cap);
    if (dec->in == NULL) {
        free(dec);
        return NULL;
    }
    return dec;
}

int ac3_dec_send_packet(void *decoder, ac3_packet_t *packet)
{
    ac3_decoder_t *dec = (ac3_decoder_t *)decoder;

    if (dec == NULL || packet == NULL || packet->len < 0 ||
        (packet->len > 0 && packet->data == NULL))
        return AC3_DEC_ERROR;

    if (append_input(dec, packet->data, packet->len) < 0)
        return AC3_DEC_ERROR;

    dec->pkt_pts = packet->pts;
    dec->pkt_dts = packet->dts;
    dec->pkt_opaque = packet->opaque;
    dec->end_reached = 0;
    dec->flushing = 0;

    process_buffer(dec, 0);
    return 0;
}

int ac3_dec_recv_frame(void *decoder, ac3_stream_info *headerset,
                       ac3_frame_t *out_frame)
{
    ac3_decoder_t *dec = (ac3_decoder_t *)decoder;

    if (dec == NULL)
        return AC3_DEC_ERROR;
    return pop_event(dec, headerset, out_frame);
}

int ac3_dec_flush(void *decoder, ac3_stream_info *headerset,
                  ac3_frame_t *out_frame)
{
    ac3_decoder_t *dec = (ac3_decoder_t *)decoder;
    int ret;

    if (dec == NULL)
        return AC3_DEC_ERROR;

    if (!dec->flushing) {
        dec->flushing = 1;
        process_buffer(dec, 1);
    }

    ret = pop_event(dec, headerset, out_frame);
    if (ret != AC3_DEC_DEFAULT)
        return ret;
    dec->end_reached = 1;
    return AC3_DEC_END;
}

void ac3_dec_frame_unref(void *decoder, ac3_frame_t *out_frame)
{
    ac3_out_frame_t *f;

    (void)decoder;
    if (out_frame == NULL || out_frame->dec_frame == NULL)
        return;
    f = (ac3_out_frame_t *)out_frame->dec_frame;
    free(f->mem);
    free(f);
    out_frame->dec_frame = NULL;
}

void ac3_dec_close(void *decoder)
{
    ac3_decoder_t *dec = (ac3_decoder_t *)decoder;

    if (dec == NULL)
        return;
    while (dec->ev_head) {
        ac3_event_t *e = dec->ev_head;
        dec->ev_head = e->next;
        if (e->frame) {
            free(e->frame->mem);
            free(e->frame);
        }
        free(e);
    }
    free(dec->in);
    free(dec);
}

const char *ac3_dec_version(void)
{
    return AC3_VERSION_STRING;
}

unsigned ac3_dec_cpu_flags(void)
{
    return 0;
}

const char *ac3_dec_cpu_name(void)
{
    return "scalar C";
}

