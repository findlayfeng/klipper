// Frame API test for the klipper host library
//
// Requires a library built with `CONFIG_HOST_FRAME_API=y`, and checks what that
// option promises: klipper_host_output_frame() hands over exactly one message
// block per call. Klipper already frames each message before buffering it, so the
// boundary is the first SYNC byte - including for the tiny ack frame, which is
// only five bytes, and for a response that is much larger than it.
//
// Build the library with the option, then:
//
//   cd third_party/klipper
//   make olddefconfig && make                     # with CONFIG_HOST_FRAME_API=y
//   cd src/host/example && make OUT=../../../out frames
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "autoconf.h" // CONFIG_HOST_FRAME_API, from the library's own build
#include "klipper_host.h"

#if !CONFIG_HOST_FRAME_API
#error "this test needs a library built with CONFIG_HOST_FRAME_API=y"
#endif

#define FRAME_BUF 128

static void *
klipper_thread_func(void *arg)
{
    (void)arg;
    klipper_host_run();
    return NULL;
}

static uint16_t
crc16_ccitt(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < len; i++) {
        uint8_t data = buf[i];
        data ^= crc & 0xff;
        data ^= data << 4;
        crc = ((((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4)
               ^ ((uint16_t)data << 3));
    }
    return crc;
}

// A "get_clock" request: msgid 5, no parameters
static size_t
build_get_clock(uint8_t *frame)
{
    frame[0] = 6;    // LEN
    frame[1] = 0x10; // SEQ, MESSAGE_DEST
    frame[2] = 5;    // MSGID
    uint16_t crc = crc16_ccitt(frame, 3);
    frame[3] = crc >> 8;
    frame[4] = crc & 0xff;
    frame[5] = 0x7e;
    return 6;
}

// One call must return exactly one whole frame. The length byte is what says so:
// a payload may contain 0x7e bytes (the identify data is compressed), so counting
// SYNC bytes to find frame boundaries would be wrong.
static int
check_single_frame(const uint8_t *buf, size_t len)
{
    if (len < 5 || len > FRAME_BUF) {
        fprintf(stderr, "FAIL: implausible frame length %zu\n", len);
        return 0;
    }
    if (buf[0] != len) {
        fprintf(stderr, "FAIL: length byte %u does not match %zu bytes\n",
                buf[0], len);
        return 0;
    }
    if (buf[len - 1] != 0x7e) {
        fprintf(stderr, "FAIL: frame does not end with SYNC\n");
        return 0;
    }
    return 1;
}

int
main(void)
{
    if (klipper_host_init() != 0) {
        fprintf(stderr, "FAIL: klipper_host_init()\n");
        return 1;
    }

    pthread_t kt;
    if (pthread_create(&kt, NULL, klipper_thread_func, NULL) != 0) {
        fprintf(stderr, "FAIL: pthread_create()\n");
        return 1;
    }

    uint8_t buf[FRAME_BUF];
    size_t n;

    // A buffer that cannot hold the largest frame is refused rather than split.
    if (klipper_host_output_frame(buf, 8, 0) != 0) {
        fprintf(stderr, "FAIL: a too small buffer should return nothing\n");
        return 1;
    }

    // The startup message arrives as one frame.
    int got_startup = 0;
    for (int i = 0; i < 100 && !got_startup; i++)
        got_startup = (n = klipper_host_output_frame(buf, sizeof(buf), 100)) > 0;
    if (!got_startup || !check_single_frame(buf, n)) {
        fprintf(stderr, "FAIL: startup frame\n");
        return 1;
    }
    printf("startup frame: %zu bytes\n", n);

    uint8_t request[6];
    size_t request_len = build_get_clock(request);
    if (klipper_host_input(request, request_len) != (ssize_t)request_len) {
        fprintf(stderr, "FAIL: klipper_host_input()\n");
        return 1;
    }

    // The answer to one request is a response frame and the ack frame that
    // follows it: two calls, two frames, each with its own boundary. Gluing or
    // splitting them here is exactly what this test is for.
    int frames = 0, have_clock = 0, have_ack = 0;
    while (frames < 2) {
        n = klipper_host_output_frame(buf, sizeof(buf), 500);
        if (!n)
            break;
        if (!check_single_frame(buf, n))
            return 1;
        frames++;
        if (buf[2] == 18)
            have_clock = 1;
        else if (n == 5)
            have_ack = 1;
        printf("frame %d: %zu bytes, msgid %u\n", frames, n, buf[2]);
    }
    if (!have_clock || !have_ack) {
        fprintf(stderr, "FAIL: expected a clock response and an ack frame\n");
        return 1;
    }

    klipper_host_shutdown();
    pthread_join(kt, NULL);
    printf("PASS\n");
    return 0;
}
