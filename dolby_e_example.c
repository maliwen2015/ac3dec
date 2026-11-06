/*
 * dolby_e_example - decode an SMPTE 337M (Dolby E) stream to a WAV file.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dolby_e.h"

static void write_u16(FILE *f, unsigned v)
{
    fputc(v & 0xFF, f);
    fputc((v >> 8) & 0xFF, f);
}

static void write_u32(FILE *f, unsigned long v)
{
    fputc(v & 0xFF, f);
    fputc((v >> 8) & 0xFF, f);
    fputc((v >> 16) & 0xFF, f);
    fputc((v >> 24) & 0xFF, f);
}

static int write_wav_header(FILE *f, int channels, int rate)
{
    unsigned long data_size = 0;

    fwrite("RIFF", 1, 4, f);
    write_u32(f, 36 + data_size);
    fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f);
    write_u32(f, 16);
    write_u16(f, 1);                    /* PCM */
    write_u16(f, (unsigned)channels);
    write_u32(f, (unsigned long)rate);
    write_u32(f, (unsigned long)(rate * channels * 2));
    write_u16(f, (unsigned)(channels * 2));
    write_u16(f, 16);
    fwrite("data", 1, 4, f);
    write_u32(f, 0);
    return 0;
}

static void patch_wav_sizes(FILE *f, long data_bytes)
{
    fseek(f, 4, SEEK_SET);
    write_u32(f, (unsigned long)(36 + data_bytes));
    fseek(f, 40, SEEK_SET);
    write_u32(f, (unsigned long)data_bytes);
    fseek(f, 0, SEEK_END);
}

static short float_to_s16(float v)
{
    int s;

    if (v > 1.0f)
        v = 1.0f;
    if (v < -1.0f)
        v = -1.0f;
    s = (int)lrintf(v * 32768.0f);
    if (s > 32767)
        s = 32767;
    if (s < -32768)
        s = -32768;
    return (short)s;
}

int main(int argc, char **argv)
{
    const char *in_name, *out_name;
    FILE *in, *out;
    uint8_t *data;
    long size;
    int pos = 0, frames = 0, channels = 0, rate = 0;
    long data_bytes = 0;
    dbe_decoder *dec;
    static uint8_t frame[8192];

    if (argc != 3) {
        fprintf(stderr, "usage: %s input.s337m output.wav\n", argv[0]);
        return 1;
    }
    in_name = argv[1];
    out_name = argv[2];

    in = fopen(in_name, "rb");
    if (!in) {
        perror(in_name);
        return 1;
    }
    fseek(in, 0, SEEK_END);
    size = ftell(in);
    fseek(in, 0, SEEK_SET);
    data = (uint8_t *)malloc((size_t)size);
    if (!data || fread(data, 1, (size_t)size, in) != (size_t)size) {
        fprintf(stderr, "cannot read %s\n", in_name);
        fclose(in);
        free(data);
        return 1;
    }
    fclose(in);

    dec = dbe_open();
    if (!dec) {
        fprintf(stderr, "cannot create decoder\n");
        free(data);
        return 1;
    }

    out = fopen(out_name, "wb");
    if (!out) {
        perror(out_name);
        dbe_close(dec);
        free(data);
        return 1;
    }
    write_wav_header(out, 0, 0);

    for (;;) {
        int frame_size = 0;
        float *planes[MAX_CHANNELS] = { 0 };
        int nch = 0, sr = 0;
        int ret = dbe_s337m_next(data, (int)size, &pos, frame, (int)sizeof(frame),
                                 &frame_size);

        if (ret == 0)
            break;
        if (ret < 0) {
            fprintf(stderr, "S337M framing error at byte %d\n", pos);
            break;
        }
        if (dbe_decode(dec, frame, frame_size, planes, &nch, &sr) < 0) {
            fprintf(stderr, "frame %d: decode error\n", frames);
            continue;
        }
        if (frames == 0) {
            channels = nch;
            rate = sr;
            fclose(out);
            out = fopen(out_name, "wb");
            write_wav_header(out, channels, rate);
        }
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            for (int c = 0; c < channels; c++) {
                short s = float_to_s16(planes[c][i]);

                fwrite(&s, 2, 1, out);
            }
        }
        data_bytes += (long)FRAME_SAMPLES * channels * 2;
        frames++;
    }

    patch_wav_sizes(out, data_bytes);
    fclose(out);
    dbe_close(dec);
    free(data);

    fprintf(stderr, "decoded %d frames, %d ch, %d Hz, %.2f s\n", frames,
            channels, rate, rate ? (double)data_bytes / (channels * 2.0 * rate)
                                 : 0.0);
    return frames > 0 ? 0 : 1;
}
