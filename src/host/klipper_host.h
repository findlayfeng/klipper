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
 * Set the identity this library answers the serial mux with.
 *
 * The mux addresses devices by identity: this is what the library's own klipper
 * instance reports when it is scanned, and what a host names when it hands the
 * device its node id. The host assigns it - the library is a file the host
 * loads, so it has nothing of its own to derive one from - and it has to be set
 * before klipper_host_init(), because the identity is fixed once the device is
 * up. Left unset, the identity is all zeros.
 *
 * @param uuid  6 bytes, most significant byte first - the order a `[mcu] uuid:`
 *              option is written in
 * @return      0 on success, -1 when klipper_host_init() has already run
 */
int klipper_host_set_uuid(const uint8_t uuid[6]);

/**
 * Feed input data into the klipper console.
 * Thread-safe: can be called from any thread at any time.
 * The data is copied into an internal buffer and processed
 * by the klipper task loop.
 *
 * The call is all-or-nothing: if the internal buffer has no room for
 * all of `len` bytes, it waits up to 250ms for the klipper task loop to
 * consume what is already buffered, and returns 0 if it still does not
 * fit. A partial copy is never made, because half a message block in
 * the stream would desynchronise the parser. A single call must fit in
 * the buffer (4096 bytes).
 *
 * @param data  pointer to input bytes (klipper binary protocol)
 * @param len   number of bytes in data
 * @return      len when the data was accepted, 0 when none of it was
 *              (the caller may retry the same block)
 */
ssize_t klipper_host_input(const uint8_t *data, size_t len);

/**
 * Retrieve output data from the klipper console.
 * Thread-safe: can be called from any thread at any time.
 * Returns available output bytes encoded in klipper message format.
 *
 * Equivalent to klipper_host_output_wait() with a timeout of 0.
 *
 * @param buf       output buffer to fill
 * @param buf_size  size of output buffer
 * @return          number of bytes written to buf, 0 if no data available
 */
size_t klipper_host_output(uint8_t *buf, size_t buf_size);

/**
 * Retrieve output data, waiting for it to arrive.
 * Thread-safe: can be called from any thread at any time.
 *
 * Blocks until at least one byte is available, the timeout expires, or
 * klipper_host_shutdown() is called. This is the call to use from a
 * dedicated reader thread, instead of polling klipper_host_output().
 *
 * Note that output is produced from inside the klipper task loop, so a
 * program that never drains it will eventually stall klipper once its
 * transmit buffer (4096 bytes) is full - the same way a real MCU stalls
 * while the host is not reading.
 *
 * @param buf         output buffer to fill
 * @param buf_size    size of output buffer
 * @param timeout_ms  how long to wait for data; 0 returns immediately
 * @return            number of bytes written to buf, 0 if no data arrived
 */
size_t klipper_host_output_wait(uint8_t *buf, size_t buf_size,
                                uint32_t timeout_ms);

/**
 * Retrieve one complete protocol frame, waiting for it to arrive.
 * Thread-safe: can be called from any thread at any time.
 *
 * Only present when the library was built with CONFIG_HOST_FRAME_API. Every
 * message klipper sends is framed before it is buffered, so this returns exactly
 * one frame - from its length byte through its trailing SYNC, found with that
 * length byte rather than by looking for SYNC, since a payload may contain one -
 * and never a piece of one. Callers that always want frames do not need to
 * reassemble the byte stream that klipper_host_output_wait() hands over.
 *
 * @param buf         output buffer to fill; must hold MESSAGE_MAX (64) bytes,
 *                    otherwise nothing is returned
 * @param buf_size    size of buffer
 * @param timeout_ms  how long to wait for a frame; 0 returns immediately
 * @return            frame bytes written to buf, 0 if no frame arrived in time
 */
#if CONFIG_HOST_FRAME_API
size_t klipper_host_output_frame(uint8_t *buf, size_t buf_size,
                                 uint32_t timeout_ms);
#endif

/**
 * Feed one complete mux unit: the frame header and the frame behind it.
 *
 * Only present when the library was built with CONFIG_HOST_FRAME_API *and*
 * bridge mode (CONFIG_BRIDGE), because only a bridge sends units upstream. It
 * is the framed counterpart of klipper_host_input(): the caller hands over
 * exactly one unit it already holds, instead of a byte stream. Half a unit is
 * refused rather than taken, because the mux finds its units by the header's
 * two-byte boundary and a truncated one would desynchronise that parse.
 *
 * The byte stream API stays available either way, and is the one the
 * assignment conversation runs on.
 *
 * @param unit  pointer to the unit: the header, then one whole frame
 * @param len   number of bytes in unit
 * @return      len when the unit was taken, 0 when none of it was (the caller
 *              may retry the same unit)
 */
ssize_t klipper_host_input_unit(const uint8_t *unit, size_t len);

/**
 * Retrieve one complete mux unit, waiting for it to arrive.
 *
 * Only present when the library was built with CONFIG_HOST_FRAME_API *and*
 * bridge mode (CONFIG_BRIDGE). Where klipper_host_output_frame() hands over a
 * whole frame, this hands over a whole unit: the header naming the device the
 * frame came from, plus that frame, so a host that speaks the mux never has to
 * find the header's boundary itself.
 *
 * @param buf         output buffer to fill; must hold UNIT_MAX (the header plus
 *                    the largest frame), otherwise nothing is returned
 * @param buf_size    size of buffer
 * @param timeout_ms  how long to wait for a unit; 0 returns immediately
 * @return            unit bytes written to buf, 0 if no unit arrived in time
 */
size_t klipper_host_output_unit(uint8_t *buf, size_t buf_size,
                                uint32_t timeout_ms);

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
