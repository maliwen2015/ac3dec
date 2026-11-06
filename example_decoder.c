/*
 * Minimal AC-3 decoder example.
 *
 *   ac3dec_example input.ac3 output.wav [options]
 *
 * By default the output is a 16-bit PCM WAV file with the same channel
 * layout as the coded stream (LFE included for multichannel layouts).
 * Use -f f32 -r to write raw interleaved 32-bit float samples, which is
 * convenient for comparison with other decoders.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "ac3dec.h"

static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s <input.ac3> <output.wav> [options]\n"
        "  -f f32|s16   sample format (default s16)\n"
        "  -r           write raw PCM instead of a WAV file\n"
        "  -p           planar output (raw only)\n"
        "  -o MODE      same|mono|stereo|dolby|3f|2f1r|3f1r|2f2r|3f2r|ch1|ch2\n"
        "  -L           keep LFE in stereo/mono downmix\n"
        "  -N           drop the LFE channel\n"
        "  -n           disable dynamic range compression\n"
        "  -D           disable dither of zero-bit coefficients\n"
        "  -c           verify frame CRCs\n"
        "  -g SCALE     dynamic range compression exponent (default 1.0)\n"
        "  -d           apply dialogue normalisation\n"
        "  -t           print stream information\n"
        "  -b BYTES     input packet size (default 8192; 1 for stress test)\n",
        argv0);
}

static int parse_output_mode(const char *s)
{
    if (!strcmp(s, "same"))   return AC3_OUTPUT_SAME;
    if (!strcmp(s, "mono"))   return AC3_OUTPUT_MONO;
    if (!strcmp(s, "stereo")) return AC3_OUTPUT_STEREO;
    if (!strcmp(s, "dolby"))  return AC3_OUTPUT_DOLBY;
    if (!strcmp(s, "3f"))     return AC3_OUTPUT_3F;
    if (!strcmp(s, "2f1r"))   return AC3_OUTPUT_2F1R;
    if (!strcmp(s, "3f1r"))   return AC3_OUTPUT_3F1R;
    if (!strcmp(s, "2f2r"))   return AC3_OUTPUT_2F2R;
    if (!strcmp(s, "3f2r"))   return AC3_OUTPUT_3F2R;
    if (!strcmp(s, "ch1"))    return AC3_OUTPUT_CHANNEL1;
    if (!strcmp(s, "ch2"))    return AC3_OUTPUT_CHANNEL2;
    return -1;
}

static void put32(FILE *f, uint32_t v)
{
    uint8_t b[4];
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
    fwrite(b, 1, 4, f);
}

static void put16(FILE *f, uint16_t v)
{
    uint8_t b[2];
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
    fwrite(b, 1, 2, f);
}

static int write_wav_header(FILE *f, const ac3_frame_t *frame,
                            int *riff_size_pos, int *data_size_pos)
{
    int bytes = (frame->sample_format == AC3_SAMPLE_S16) ? 2 : 4;
    int format = (frame->sample_format == AC3_SAMPLE_S16) ? 1 : 3;
    uint32_t byte_rate = (uint32_t)frame->sample_rate * frame->num_channels * bytes;
    uint16_t block_align = (uint16_t)(frame->num_channels * bytes);

    fseek(f, 0, SEEK_SET);
    fwrite("RIFF", 1, 4, f);
    *riff_size_pos = (int)ftell(f);
    put32(f, 0);
    fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f);
    put32(f, 16);
    put16(f, (uint16_t)format);
    put16(f, (uint16_t)frame->num_channels);
    put32(f, (uint32_t)frame->sample_rate);
    put32(f, byte_rate);
    put16(f, block_align);
    put16(f, (uint16_t)(bytes * 8));
    if (format != 1) {
        fwrite("fact", 1, 4, f);
        put32(f, 4);
        put32(f, 0);
    }
    fwrite("data", 1, 4, f);
    *data_size_pos = (int)ftell(f);
    put32(f, 0);
    return 0;
}

static int write_frame(FILE *f, const ac3_frame_t *frame, int planar)
{
    int ch, n;
    int bytes = (frame->sample_format == AC3_SAMPLE_S16) ? 2 : 4;

    if (!planar) {
        size_t total = (size_t)frame->num_channels * frame->samples * bytes;
        if (fwrite(frame->planes[0], 1, total, f) != total)
            return -1;
        return 0;
    }

    for (n = 0; n < frame->samples; n++) {
        for (ch = 0; ch < frame->num_channels; ch++) {
            const uint8_t *p = (const uint8_t *)frame->planes[ch] +
                               (size_t)n * frame->strides[ch];
            if (fwrite(p, 1, (size_t)bytes, f) != (size_t)bytes)
                return -1;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    FILE *in = NULL, *out = NULL;
    ac3_param_t param;
    ac3_packet_t pkt;
    ac3_stream_info info;
    ac3_frame_t pic;
    void *dec = NULL;
    uint8_t *buf = NULL;
    int chunk = 8192;
    int raw = 0;
    int planar = 0;
    int print_info = 0;
    int eof = 0;
    int frames = 0;
    int64_t pts = 0;
    int wav_ready = 0;
    int riff_pos = 0, data_pos = 0;
    int wav_rate = 0, wav_channels = 0, wav_format = 0;
    int64_t wav_samples = 0;
    int i;

    if (argc < 3) {
        usage(argv[0]);
        return 2;
    }

    memset(&param, 0, sizeof(param));
    param.sample_format = AC3_SAMPLE_S16;

    for (i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "-f") && i + 1 < argc) {
            const char *v = argv[++i];
            if (!strcmp(v, "f32"))
                param.sample_format = AC3_SAMPLE_F32;
            else if (!strcmp(v, "s16"))
                param.sample_format = AC3_SAMPLE_S16;
            else {
                fprintf(stderr, "bad sample format: %s\n", v);
                return 2;
            }
        } else if (!strcmp(argv[i], "-r")) {
            raw = 1;
        } else if (!strcmp(argv[i], "-p")) {
            planar = 1;
        } else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
            int mode = parse_output_mode(argv[++i]);
            if (mode < 0) {
                fprintf(stderr, "bad output mode: %s\n", argv[i]);
                return 2;
            }
            param.output = mode;
        } else if (!strcmp(argv[i], "-L")) {
            param.output |= AC3_OUTPUT_LFE;
        } else if (!strcmp(argv[i], "-N")) {
            param.output |= AC3_OUTPUT_NO_LFE;
        } else if (!strcmp(argv[i], "-n")) {
            param.disable_drc = 1;
        } else if (!strcmp(argv[i], "-D")) {
            param.disable_dither = 1;
        } else if (!strcmp(argv[i], "-c")) {
            param.check_crc = 1;
        } else if (!strcmp(argv[i], "-g") && i + 1 < argc) {
            param.drc_scale = (float)atof(argv[++i]);
        } else if (!strcmp(argv[i], "-d")) {
            param.apply_dialnorm = 1;
        } else if (!strcmp(argv[i], "-t")) {
            print_info = 1;
        } else if (!strcmp(argv[i], "-b") && i + 1 < argc) {
            chunk = atoi(argv[++i]);
            if (chunk < 1)
                chunk = 1;
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    if (!raw && planar) {
        fprintf(stderr, "WAV output is always interleaved; -p ignored\n");
        planar = 0;
    }
    param.interleaved = raw ? !planar : 1;

    in = fopen(argv[1], "rb");
    if (!in) {
        perror(argv[1]);
        return 2;
    }
    out = fopen(argv[2], raw ? "wb" : "wb+");
    if (!out) {
        perror(argv[2]);
        fclose(in);
        return 2;
    }

    buf = (uint8_t *)malloc((size_t)chunk);
    if (!buf) {
        fprintf(stderr, "out of memory\n");
        return 2;
    }

    dec = ac3_dec_open(&param);
    if (!dec) {
        fprintf(stderr, "ac3_dec_open failed\n");
        return 2;
    }

    while (!eof) {
        size_t n = fread(buf, 1, (size_t)chunk, in);
        int ret;

        if (n == 0) {
            eof = 1;
        } else {
            memset(&pkt, 0, sizeof(pkt));
            pkt.data = buf;
            pkt.len = (int)n;
            pkt.pts = pts;
            pkt.dts = pts;
            pts += 1536;
            if (ac3_dec_send_packet(dec, &pkt) < 0)
                fprintf(stderr, "send_packet reported an error\n");
        }

        for (;;) {
            memset(&info, 0, sizeof(info));
            memset(&pic, 0, sizeof(pic));
            ret = ac3_dec_recv_frame(dec, &info, &pic);
            if (ret == AC3_DEC_GOT_HEADER) {
                if (print_info) {
                    fprintf(stderr, "header: %d Hz, %d kbps, %.2f ms, "
                            "%s, acmod=%d lfe=%d map=0x%02x -> %d ch map=0x%02x\n",
                            info.sample_rate, info.bit_rate / 1000,
                            1000.0 * (double)info.samples / info.sample_rate,
                            info.eac3 ? "E-AC-3" : "AC-3",
                            info.acmod, info.lfeon, info.channel_map,
                            info.output_channels, info.output_channel_map);
                }
                continue;
            }
            if (ret != AC3_DEC_GOT_FRAME)
                break;

            if (!raw && !wav_ready) {
                if (write_wav_header(out, &pic, &riff_pos, &data_pos) < 0) {
                    fprintf(stderr, "write failed\n");
                    return 2;
                }
                wav_rate = pic.sample_rate;
                wav_channels = pic.num_channels;
                wav_format = pic.sample_format;
                wav_ready = 1;
            }
            if (!raw && (pic.sample_rate != wav_rate ||
                         pic.num_channels != wav_channels ||
                         pic.sample_format != wav_format)) {
                fprintf(stderr, "stream format changed; stopping\n");
                eof = 1;
                break;
            }
            if (write_frame(out, &pic, raw && planar) < 0) {
                fprintf(stderr, "write failed\n");
                return 2;
            }
            wav_samples += pic.samples;
            frames++;
            ac3_dec_frame_unref(dec, &pic);
        }
    }

    for (;;) {
        int ret;

        memset(&info, 0, sizeof(info));
        memset(&pic, 0, sizeof(pic));
        ret = ac3_dec_flush(dec, &info, &pic);
        if (ret == AC3_DEC_GOT_HEADER) {
            if (print_info)
                fprintf(stderr, "flush header: %d Hz, acmod=%d\n",
                        info.sample_rate, info.acmod);
            continue;
        }
        if (ret == AC3_DEC_GOT_FRAME) {
            if (!raw && !wav_ready) {
                if (write_wav_header(out, &pic, &riff_pos, &data_pos) < 0)
                    return 2;
                wav_rate = pic.sample_rate;
                wav_channels = pic.num_channels;
                wav_format = pic.sample_format;
                wav_ready = 1;
            }
            if (write_frame(out, &pic, raw && planar) < 0)
                return 2;
            wav_samples += pic.samples;
            frames++;
            ac3_dec_frame_unref(dec, &pic);
            continue;
        }
        if (ret == AC3_DEC_END || ret == AC3_DEC_DEFAULT)
            break;
        break;
    }

    if (!raw && wav_ready) {
        int bytes = (wav_format == AC3_SAMPLE_S16) ? 2 : 4;
        uint32_t data_bytes = (uint32_t)(wav_samples * wav_channels * bytes);
        /* data_pos points at the data chunk's size field, so the payload
         * starts four bytes later. */
        uint32_t riff_bytes = (uint32_t)(data_pos + 4 - 8 + data_bytes);

        fseek(out, riff_pos, SEEK_SET);
        put32(out, riff_bytes);
        fseek(out, data_pos, SEEK_SET);
        put32(out, data_bytes);
        if (wav_format != AC3_SAMPLE_S16) {
            /* Patch the fact chunk sample count when present. */
            fseek(out, data_pos - 8, SEEK_SET);
            put32(out, (uint32_t)wav_samples);
        }
    }

    ac3_dec_close(dec);
    fclose(out);
    fclose(in);
    free(buf);
    fprintf(stderr, "decoded %d frames (%lld samples/ch), %s\n",
            frames, (long long)wav_samples, ac3_dec_version());
    return 0;
}
