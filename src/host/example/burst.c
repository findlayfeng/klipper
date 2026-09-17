// Buffer handling test for the klipper host library
//
// The library buffers both directions in fixed 4096 byte arrays, which makes two
// failure modes possible if the boundaries are handled carelessly:
//
//   1. input that does not fit must not be cut in half (a truncated message block
//      desynchronises the parser), and
//   2. output produced while nobody is reading must not be thrown away (a dropped
//      response leaves its caller waiting for a reply that never comes).
//
// Both are checked here with a burst that exceeds the buffers: many `get_clock`
// commands are handed over without draining anything, and every response has to
// come back. Run it after `make` in the klipper tree:
//
//   cd src/host/example && make test
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "klipper_host.h"

#define COMMANDS 2048         // 12KB in, ~16KB out: both initial buffers overflow
#define IDLE_LIMIT 500        // polls (1ms apart) without output before giving up

static void *
klipper_thread_func(void *arg)
{
    (void)arg;
    klipper_host_run();
    return NULL;
}

// CRC16-CCITT over a buffer, as used by the message block trailer
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

// Frame a message id with no parameters
static size_t
build_frame(uint8_t *frame, uint8_t msgid, uint8_t seq)
{
    frame[0] = 5 + 1;                       // LEN
    frame[1] = 0x10 | (seq & 0x0f);         // SEQ with MESSAGE_DEST
    frame[2] = msgid;                       // MSGID
    uint16_t crc = crc16_ccitt(frame, 3);
    frame[3] = crc >> 8;
    frame[4] = crc & 0xff;
    frame[5] = 0x7e;                        // trailing SYNC
    return 6;
}

// Count the message blocks in a byte stream. Frames always end with SYNC, so the
// number of SYNC bytes is the number of blocks - enough for this test, and it
// does not depend on the parser being correct.
static int
count_blocks(const uint8_t *buf, size_t len)
{
    int count = 0;
    for (size_t i = 0; i < len; i++)
        if (buf[i] == 0x7e)
            count++;
    return count;
}

int
main(void)
{
    const uint8_t GET_CLOCK = 5;            // msgid in this firmware build

    if (klipper_host_init() != 0) {
        fprintf(stderr, "FAIL: klipper_host_init()\n");
        return 1;
    }

    pthread_t kt;
    if (pthread_create(&kt, NULL, klipper_thread_func, NULL) != 0) {
        fprintf(stderr, "FAIL: pthread_create()\n");
        return 1;
    }

    // Drain klipper's startup output (the "starting" message). Only the plain
    // (non-waiting) read is used here, so this test also compiles and runs
    // against a library without klipper_host_output_wait().
    uint8_t buf[256];
    int startup = 0;
    for (int i = 0; i < 100 && !startup; i++) {
        size_t n;
        while ((n = klipper_host_output(buf, sizeof(buf))) > 0)
            startup += count_blocks(buf, n);
        if (!startup)
            usleep(1000);
    }
    if (!startup) {
        fprintf(stderr, "FAIL: no startup output\n");
        return 1;
    }
    printf("startup output: %d block(s)\n", startup);

    // Hand over a burst of commands without reading any output. This is what
    // fills klipper's transmit buffer, so it is the case where a dropped or
    // truncated response would show up.
    int sent = 0;
    for (int i = 0; i < COMMANDS; i++) {
        uint8_t frame[6];
        size_t len = build_frame(frame, GET_CLOCK, i & 0x0f);
        ssize_t taken = klipper_host_input(frame, len);
        if (taken != (ssize_t)len) {
            fprintf(stderr, "FAIL: command %d: klipper_host_input() took %zd of "
                    "%zu bytes\n", i, taken, len);
            return 1;
        }
        sent++;
    }
    printf("sent %d commands without reading\n", sent);

    // Now read everything back. Every command must be answered.
    int blocks = 0;
    int idle = 0;
    while (blocks < sent && idle < IDLE_LIMIT) {
        size_t n = klipper_host_output(buf, sizeof(buf));
        if (!n) {
            idle++;
            usleep(1000);
            continue;
        }
        idle = 0;
        blocks += count_blocks(buf, n);
    }
    printf("received %d block(s) for %d command(s)\n", blocks, sent);
    if (blocks < sent) {
        fprintf(stderr, "FAIL: %d response(s) lost\n", sent - blocks);
        return 1;
    }

    klipper_host_shutdown();
    pthread_join(kt, NULL);
    printf("PASS\n");
    return 0;
}
