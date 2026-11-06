/*
 * Public API tests for ac3dec.
 *
 * Usage: test_api <data-directory>
 *
 * The data directory must contain the .ac3 streams shipped with the source
 * tree: tiny_stereo.ac3, tiny_51.ac3, mono.ac3, stereo_nocpl.ac3.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "ac3dec.h"

static int failures;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                                     \
        }                                                                   \
    } while (0)

static uint8_t *read_file(const char *path, int *size)
{
    FILE *f = fopen(path, "rb");
    uint8_t *buf;
    long n;

    if (!f)
        return NULL;
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return NULL;
    }
    n = ftell(f);
    if (n < 0 || fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return NULL;
    }
    buf = (uint8_t *)malloc((size_t)n ? (size_t)n : 1);
    if (!buf) {
        fclose(f);
        return NULL;
    }
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) {
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);
    *size = (int)n;
    return buf;
}

static uint64_t hash_bytes(uint64_t h, const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t i;

    for (i = 0; i < len; i++) {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
    return h;
}

struct decode_result {
    int frames;
    int headers;
    int channels;
    int sample_rate;
    int sample_format;
    int planar;
    int channel_map;
    int acmod;
    int lfeon;
    int samples;            /* samples per output frame */
    int info_samples;       /* samples per syncframe from the header */
    int eac3;
    int num_blocks;
    int output_channels;
    uint64_t hash;
};

/* Decode a whole buffer with the given parameters and packet size. */
static int decode_buffer(const uint8_t *data, int size, const ac3_param_t *param,
                         int packet_size, struct decode_result *res)
{
    void *dec;
    ac3_packet_t pkt;
    ac3_frame_t frame;
    ac3_stream_info info;
    int64_t pts = 0;
    int pos = 0;
    int ret;

    memset(res, 0, sizeof(*res));
    res->hash = 1469598103934665603ULL;
    dec = ac3_dec_open((ac3_param_t *)param);
    if (!dec)
        return -1;

    while (pos < size) {
        int n = size - pos;
        if (packet_size > 0 && n > packet_size)
            n = packet_size;
        memset(&pkt, 0, sizeof(pkt));
        pkt.data = data + pos;
        pkt.len = n;
        pkt.pts = pts;
        pkt.dts = pts;
        if (ac3_dec_send_packet(dec, &pkt) != 0) {
            ac3_dec_close(dec);
            return -1;
        }
        pts += 1536;

        for (;;) {
            memset(&info, 0, sizeof(info));
            memset(&frame, 0, sizeof(frame));
            ret = ac3_dec_recv_frame(dec, &info, &frame);
            if (ret == AC3_DEC_GOT_HEADER) {
                res->headers++;
                res->channels = info.channels;
                res->sample_rate = info.sample_rate;
                res->channel_map = info.channel_map;
                res->acmod = info.acmod;
                res->lfeon = info.lfeon;
                res->info_samples = info.samples;
                res->eac3 = info.eac3;
                res->num_blocks = info.num_blocks;
                res->output_channels = info.output_channels;
                continue;
            }
            if (ret != AC3_DEC_GOT_FRAME)
                break;
            res->frames++;
            res->samples = frame.samples;
            res->sample_format = frame.sample_format;
            res->planar = frame.planar;
            if (frame.planar) {
                int ch;
                for (ch = 0; ch < frame.num_channels; ch++)
                    res->hash = hash_bytes(res->hash, frame.planes[ch],
                                           (size_t)frame.samples *
                                           frame.strides[ch]);
            } else {
                res->hash = hash_bytes(res->hash, frame.planes[0],
                                       (size_t)frame.samples *
                                       frame.strides[0]);
            }
            ac3_dec_frame_unref(dec, &frame);
        }
        pos += n;
    }

    for (;;) {
        memset(&info, 0, sizeof(info));
        memset(&frame, 0, sizeof(frame));
        ret = ac3_dec_flush(dec, &info, &frame);
        if (ret == AC3_DEC_GOT_HEADER)
            continue;
        if (ret == AC3_DEC_GOT_FRAME) {
            res->frames++;
            ac3_dec_frame_unref(dec, &frame);
            continue;
        }
        CHECK(ret == AC3_DEC_END);
        break;
    }

    ac3_dec_close(dec);
    return 0;
}

static void test_stream(const char *dir, const char *name, int frames,
                        int coded_channels, int output_channels,
                        int map, int acmod, int lfeon)
{
    char path[1024];
    uint8_t *data;
    int size = 0;
    ac3_param_t param;
    struct decode_result a, b;

    snprintf(path, sizeof(path), "%s/%s", dir, name);
    data = read_file(path, &size);
    if (!data) {
        fprintf(stderr, "SKIP: cannot read %s\n", path);
        return;
    }

    memset(&param, 0, sizeof(param));
    CHECK(decode_buffer(data, size, &param, 8192, &a) == 0);
    CHECK(a.frames == frames);
    CHECK(a.samples == 1536);
    CHECK(a.channels == coded_channels);
    CHECK(a.output_channels == output_channels);
    CHECK(a.sample_rate == 48000);
    CHECK(a.channel_map == map);
    CHECK(a.acmod == acmod);
    CHECK(a.lfeon == lfeon);
    CHECK(a.sample_format == AC3_SAMPLE_F32);
    CHECK(a.planar == 1);
    CHECK(a.headers >= 1);

    /* One-byte packets must produce bit-identical output. */
    CHECK(decode_buffer(data, size, &param, 1, &b) == 0);
    CHECK(b.frames == a.frames);
    CHECK(b.hash == a.hash);

    free(data);
}

/* E-AC-3 streams use the same API but report the syncframe structure. */
static void test_eac3_stream(const char *dir, const char *name, int frames,
                             int coded_channels, int output_channels,
                             int map, int acmod, int lfeon)
{
    char path[1024];
    uint8_t *data;
    int size = 0;
    ac3_param_t param;
    struct decode_result a, b;

    snprintf(path, sizeof(path), "%s/%s", dir, name);
    data = read_file(path, &size);
    if (!data) {
        fprintf(stderr, "SKIP: cannot read %s\n", path);
        return;
    }

    memset(&param, 0, sizeof(param));
    CHECK(decode_buffer(data, size, &param, 8192, &a) == 0);
    CHECK(a.frames == frames);
    CHECK(a.eac3 == 1);
    CHECK(a.num_blocks == 6);
    CHECK(a.info_samples == 1536);
    CHECK(a.samples == 1536);
    CHECK(a.channels == coded_channels);
    CHECK(a.output_channels == output_channels);
    CHECK(a.sample_rate == 48000);
    CHECK(a.channel_map == map);
    CHECK(a.acmod == acmod);
    CHECK(a.lfeon == lfeon);
    CHECK(a.sample_format == AC3_SAMPLE_F32);
    CHECK(a.planar == 1);
    CHECK(a.headers >= 1);

    /* One-byte packets must produce bit-identical output. */
    CHECK(decode_buffer(data, size, &param, 1, &b) == 0);
    CHECK(b.frames == a.frames);
    CHECK(b.hash == a.hash);

    free(data);
}

static void test_eac3_downmix(const char *dir)
{
    char path[1024];
    uint8_t *data;
    int size = 0;
    ac3_param_t param;
    struct decode_result a;

    snprintf(path, sizeof(path), "%s/eac3_51.eac3", dir);
    data = read_file(path, &size);
    if (!data)
        return;

    memset(&param, 0, sizeof(param));
    param.output = AC3_OUTPUT_STEREO;
    CHECK(decode_buffer(data, size, &param, 8192, &a) == 0);
    CHECK(a.output_channels == 2);
    CHECK(a.frames == 10);

    param.output = AC3_OUTPUT_MONO;
    CHECK(decode_buffer(data, size, &param, 8192, &a) == 0);
    CHECK(a.output_channels == 1);

    free(data);
}

static void test_formats(const char *dir)
{
    char path[1024];
    uint8_t *data;
    int size = 0;
    ac3_param_t param;
    struct decode_result a, b;

    snprintf(path, sizeof(path), "%s/tiny_stereo.ac3", dir);
    data = read_file(path, &size);
    if (!data)
        return;

    memset(&param, 0, sizeof(param));
    param.sample_format = AC3_SAMPLE_S16;
    param.interleaved = 1;
    CHECK(decode_buffer(data, size, &param, 4096, &a) == 0);
    CHECK(a.frames == 10);
    CHECK(a.sample_format == AC3_SAMPLE_S16);
    CHECK(a.planar == 0);

    /* Default float planar and S16 interleaved must both decode every frame. */
    memset(&param, 0, sizeof(param));
    CHECK(decode_buffer(data, size, &param, 4096, &b) == 0);
    CHECK(b.frames == a.frames);

    /* Disabling dither must change the output (the streams use zero-bit
     * mantle coefficients).  It must not change the frame count. */
    param.disable_dither = 1;
    CHECK(decode_buffer(data, size, &param, 4096, &a) == 0);
    CHECK(a.frames == b.frames);
    CHECK(a.hash != b.hash);

    free(data);
}

static void test_downmix(const char *dir)
{
    char path[1024];
    uint8_t *data;
    int size = 0;
    ac3_param_t param;
    struct decode_result a;

    snprintf(path, sizeof(path), "%s/tiny_51.ac3", dir);
    data = read_file(path, &size);
    if (!data)
        return;

    memset(&param, 0, sizeof(param));
    CHECK(decode_buffer(data, size, &param, 8192, &a) == 0);
    CHECK(a.output_channels == 6);

    param.output = AC3_OUTPUT_STEREO;
    CHECK(decode_buffer(data, size, &param, 8192, &a) == 0);
    CHECK(a.output_channels == 2);
    CHECK(a.frames == 10);

    param.output = AC3_OUTPUT_MONO;
    CHECK(decode_buffer(data, size, &param, 8192, &a) == 0);
    CHECK(a.output_channels == 1);
    CHECK(a.frames == 10);

    param.output = AC3_OUTPUT_DOLBY;
    CHECK(decode_buffer(data, size, &param, 8192, &a) == 0);
    CHECK(a.output_channels == 2);
    CHECK(a.frames == 10);

    free(data);
}

static void test_api_misuse(void)
{
    ac3_packet_t pkt;
    ac3_frame_t frame;
    ac3_stream_info info;

    memset(&pkt, 0, sizeof(pkt));
    memset(&frame, 0, sizeof(frame));
    memset(&info, 0, sizeof(info));

    CHECK(ac3_dec_send_packet(NULL, &pkt) == AC3_DEC_ERROR);
    CHECK(ac3_dec_recv_frame(NULL, &info, &frame) == AC3_DEC_ERROR);
    CHECK(ac3_dec_flush(NULL, &info, &frame) == AC3_DEC_ERROR);
    ac3_dec_close(NULL);
    ac3_dec_frame_unref(NULL, NULL);
    ac3_dec_frame_unref(NULL, &frame);
    CHECK(ac3_dec_version() != NULL);
    CHECK(ac3_dec_cpu_name() != NULL);

    {
        void *dec = ac3_dec_open(NULL);
        CHECK(dec != NULL);
        if (dec) {
            pkt.data = NULL;
            pkt.len = 4;
            CHECK(ac3_dec_send_packet(dec, &pkt) == AC3_DEC_ERROR);
            pkt.len = -1;
            CHECK(ac3_dec_send_packet(dec, &pkt) == AC3_DEC_ERROR);
            pkt.data = (const uint8_t *)"abcd";
            pkt.len = 4;
            CHECK(ac3_dec_send_packet(dec, &pkt) == 0);
            ac3_dec_close(dec);
        }
    }
}

static void test_garbage(void)
{
    uint8_t junk[4096];
    ac3_param_t param;
    void *dec;
    ac3_packet_t pkt;
    ac3_frame_t frame;
    ac3_stream_info info;
    int ret;
    int frames = 0;
    int i;

    for (i = 0; i < (int)sizeof(junk); i++)
        junk[i] = (uint8_t)(i * 37 + 11);

    memset(&param, 0, sizeof(param));
    dec = ac3_dec_open(&param);
    CHECK(dec != NULL);
    if (!dec)
        return;

    memset(&pkt, 0, sizeof(pkt));
    pkt.data = junk;
    pkt.len = (int)sizeof(junk);
    CHECK(ac3_dec_send_packet(dec, &pkt) == 0);
    for (;;) {
        memset(&info, 0, sizeof(info));
        memset(&frame, 0, sizeof(frame));
        ret = ac3_dec_recv_frame(dec, &info, &frame);
        if (ret != AC3_DEC_GOT_FRAME)
            break;
        frames++;
        ac3_dec_frame_unref(dec, &frame);
    }
    CHECK(frames == 0);
    for (;;) {
        memset(&frame, 0, sizeof(frame));
        ret = ac3_dec_flush(dec, &info, &frame);
        if (ret == AC3_DEC_GOT_FRAME) {
            frames++;
            ac3_dec_frame_unref(dec, &frame);
            continue;
        }
        break;
    }
    CHECK(ret == AC3_DEC_END);
    ac3_dec_close(dec);
}

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : "tests/data";

    test_stream(dir, "tiny_stereo.ac3", 10, 2, 2, 0x03, 2, 0);
    test_stream(dir, "tiny_51.ac3", 10, 5, 6, 0x3f, 7, 1);
    test_stream(dir, "mono.ac3", 4, 1, 1, 0x04, 1, 0);
    test_stream(dir, "stereo_nocpl.ac3", 4, 2, 2, 0x03, 2, 0);
    test_eac3_stream(dir, "eac3_mono.eac3", 10, 1, 1, 0x04, 1, 0);
    test_eac3_stream(dir, "eac3_stereo.eac3", 10, 2, 2, 0x03, 2, 0);
    test_eac3_stream(dir, "eac3_51.eac3", 10, 5, 6, 0x3f, 7, 1);
    test_eac3_downmix(dir);
    test_formats(dir);
    test_downmix(dir);
    test_api_misuse();
    test_garbage();

    if (failures) {
        fprintf(stderr, "%d API test(s) failed\n", failures);
        return 1;
    }
    printf("test_api: all checks passed\n");
    return 0;
}
