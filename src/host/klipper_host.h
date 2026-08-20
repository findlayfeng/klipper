// Public API for Klipper host library
//
// This library provides a thread-safe interface for interacting with
// the Klipper firmware running on the host. Instead of using stdin/stdout,
// it provides explicit input/output functions that can be called from
// any thread.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __KLIPPER_HOST_H
#define __KLIPPER_HOST_H

// Required for cpu_set_t and other POSIX extensions
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h> // ssize_t

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the klipper host library.
 * Must be called before any other klipper_host_* functions.
 * Returns 0 on success, -1 on failure.
 */
int klipper_host_init(void);

/**
 * Feed input data into the klipper console.
 * Thread-safe: can be called from any thread at any time.
 * The data is copied into an internal buffer and processed
 * by the klipper task loop.
 *
 * @param data  pointer to input bytes (klipper binary protocol)
 * @param len   number of bytes in data
 * @return      number of bytes consumed (always len on success)
 */
ssize_t klipper_host_input(const uint8_t *data, size_t len);

/**
 * Retrieve output data from the klipper console.
 * Thread-safe: can be called from any thread at any time.
 * Returns available output bytes encoded in klipper message format.
 *
 * @param buf       output buffer to fill
 * @param buf_size  size of output buffer
 * @return          number of bytes written to buf, 0 if no data available
 */
size_t klipper_host_output(uint8_t *buf, size_t buf_size);

/**
 * Run the klipper main scheduling loop (blocking).
 * This function blocks until klipper_host_shutdown() is called.
 * Internal shutdowns (e.g. emergency_stop) keep the loop running
 * and can be recovered with a config_reset command.
 * Should be called from a dedicated thread or the main thread.
 */
void klipper_host_run(void);

/**
 * Run a single iteration of the klipper task loop (non-blocking).
 * Processes pending timers, runs tasks once, returns immediately.
 * Useful for integrating with external event loops (epoll, select, etc).
 */
void klipper_host_step(void);

/**
 * Request klipper to shut down.
 * Wakes up klipper_host_run() so it can return.
 * Thread-safe: can be called from any thread.
 */
void klipper_host_shutdown(void);

/**
 * Check if klipper is in shutdown state.
 * Returns non-zero if klipper has entered shutdown.
 */
uint8_t klipper_host_is_shutdown(void);

/**
 * Get the current klipper time in clock ticks.
 */
uint32_t klipper_host_get_clock(void);

#ifdef __cplusplus
}
#endif

#endif // __KLIPPER_HOST_H
