// Example program demonstrating the klipper host library
//
// This program links against libklipper_host.a and demonstrates
// the thread-safe input/output functions.
//
// Usage:
//   make            (from third_party/klipper/src/host/example)
//   ./klipper_host_example
//
// The example:
//   1. Initializes the klipper host library
//   2. Starts klipper (klipper_host_run) in a background thread
//   3. Reads the framed "starting" message via klipper_host_output()
//   4. Sends a "get_clock" command (binary protocol) via
//      klipper_host_input() and reads the response
//   5. Shuts down klipper gracefully
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>
#include "klipper_host.h"

// Thread function to run klipper
static void *
klipper_thread_func(void *arg)
{
    (void)arg;
    printf("[klipper] entering klipper_host_run()...\n");
    klipper_host_run();
    printf("[klipper] klipper_host_run() returned\n");
    return NULL;
}

// Print framed message bytes (for demonstration)
static void
print_framed(const uint8_t *buf, size_t len)
{
    printf("[out] %3zu bytes:", len);
    for (size_t i = 0; i < len; i++)
        printf(" %02x", buf[i]);
    printf("\n");
}

// Send a "get_clock" command and wait for the "clock" response.
//
// Klipper binary protocol frame (no leading sync, trailing sync only):
//   LEN SEQ MSGID PARAMS... CRC16 SYNC(0x7e)
//   LEN = msglen (header 2 + payload + trailer 3)
//   SEQ = 0x10 | sequence (MESSAGE_DEST)
//   CRC16-CCITT over bytes [0 .. LEN-3]
//
// "get_clock" has msgid 5 in this firmware build (see
// out/compile_time_request.c).
static void
demo_command_roundtrip(void)
{
    // Build a "get_clock" command frame
    uint8_t frame[6];
    frame[0] = 0x06;   // LEN
    frame[1] = 0x10;   // SEQ (MESSAGE_DEST)
    frame[2] = 0x05;   // MSGID for "get_clock"
    frame[3] = frame[4] = 0x00;
    frame[5] = 0x7e;   // trailing SYNC

    // Compute CRC16-CCITT over bytes [0..LEN-3] = frame[0..2]
    uint16_t crc = 0xffff;
    for (int i = 0; i < 3; i++) {
        uint8_t data = frame[i];
        data ^= crc & 0xff;
        data ^= data << 4;
        crc = ((((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4)
               ^ ((uint16_t)data << 3));
    }
    frame[3] = crc >> 8;
    frame[4] = crc & 0xff;

    printf("\n[send] sending framed get_clock command:\n");
    print_framed(frame, sizeof(frame));
    ssize_t n = klipper_host_input(frame, sizeof(frame));
    printf("[send] klipper_host_input() consumed %zd bytes\n", n);

    // Give klipper a moment to process and respond
    usleep(50000);

    // Read any output (should contain the ack and the clock response)
    uint8_t buf[256];
    size_t out;
    while ((out = klipper_host_output(buf, sizeof(buf))) > 0)
        print_framed(buf, out);
}

int
main(int argc, char **argv)
{
    pthread_t kt;
    int ret;
    uint8_t buf[256];
    size_t n;

    printf("Klipper Host Library Example\n");
    printf("============================\n\n");

    // 1. Initialize the library
    printf("[main] klipper_host_init()...\n");
    ret = klipper_host_init();
    if (ret != 0) {
        fprintf(stderr, "klipper_host_init() failed\n");
        return 1;
    }

    // 2. Start klipper in a background thread
    printf("[main] starting klipper thread...\n");
    ret = pthread_create(&kt, NULL, klipper_thread_func, NULL);
    if (ret != 0) {
        fprintf(stderr, "pthread_create() failed: %s\n", strerror(ret));
        return 1;
    }

    // 3. Wait for klipper to initialize and read the "starting" message
    printf("[main] waiting for klipper startup output...\n");
    for (int i = 0; i < 50; i++) {
        while ((n = klipper_host_output(buf, sizeof(buf))) > 0)
            print_framed(buf, n);
        if (n || i > 20)
            break;
        usleep(20000);
    }

    // 4. Send a command and read the response
    demo_command_roundtrip();

    // 5. Check status and shutdown
    printf("\n[main] klipper shutdown state: %s\n",
           klipper_host_is_shutdown() ? "shutdown" : "running");
    printf("[main] klipper_host_shutdown()...\n");
    klipper_host_shutdown();

    ret = pthread_join(kt, NULL);
    if (ret != 0) {
        fprintf(stderr, "pthread_join() failed: %s\n", strerror(ret));
        return 1;
    }

    printf("\n[main] done.\n");
    return 0;
}
