// Thread-safe console I/O for klipper host library
//
// Implements console_sendf() and console_receive_buffer() as required
// by klipper's command infrastructure, using mutex-protected buffers
// instead of file descriptors.
//
//   - klipper_host_input():  appends bytes to the receive buffer and
//     wakes the console task (thread-safe).
//   - klipper_host_output(): drains the transmit buffer (thread-safe).
//   - console_task():        parses and dispatches complete message
//     blocks from the receive buffer (runs in the klipper task loop).
//
// Note on shutdown: klipper's shutdown() longjmps out of the current
// stack, which can leave a mutex locked. A DECL_SHUTDOWN handler (and
// klipper_console_unlock() after an external shutdown longjmp) unlock
// the mutexes to keep the library usable afterwards.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#define _GNU_SOURCE
#include <string.h> // memcpy, memmove
#include <pthread.h> // pthread_mutex_t
#include "board/misc.h" // console_sendf, console_receive_buffer
#include "command.h" // command_encode_and_frame, command_find_and_dispatch
#include "sched.h" // sched_wake_task, DECL_TASK, DECL_SHUTDOWN
#include "internal.h" // klipper_console_unlock
#include "klipper_host.h" // klipper_host_input, klipper_host_output

#define TX_BUFFER_SIZE 4096
#define RX_BUFFER_SIZE 4096

// Output (klipper -> caller) buffer. tx_pos is the read offset and
// tx_max the number of bytes currently buffered.
static uint8_t tx_buf[TX_BUFFER_SIZE];
static volatile size_t tx_pos, tx_max;
static pthread_mutex_t tx_mutex = PTHREAD_MUTEX_INITIALIZER;

// Input (caller -> klipper) buffer. rx_count is the number of bytes
// currently buffered at the start of rx_buf.
static uint8_t rx_buf[RX_BUFFER_SIZE];
static volatile size_t rx_count;
static pthread_mutex_t rx_mutex = PTHREAD_MUTEX_INITIALIZER;

// Wake flag for the console task
static struct task_wake console_wake;

// Provide pointer to receive buffer for klipper's command parsing.
// Command pointer arguments are decoded relative to this buffer, so
// dispatch must run while the data is still in rx_buf (rx_mutex held).
void *
console_receive_buffer(void)
{
    return rx_buf;
}

// Encode and buffer a "response" message (called from klipper's sendf)
void
console_sendf(const struct command_encoder *ce, va_list args)
{
    uint8_t msg_buf[MESSAGE_MAX];
    uint_fast8_t msglen = command_encode_and_frame(
        msg_buf, sizeof(msg_buf), ce, args);
    if (!msglen)
        return;

    pthread_mutex_lock(&tx_mutex);
    // Compact the buffer if needed to make room for the new message
    if (tx_max + msglen > TX_BUFFER_SIZE) {
        size_t used = tx_max - tx_pos;
        if (used)
            memmove(tx_buf, &tx_buf[tx_pos], used);
        tx_max = used;
        tx_pos = 0;
    }
    if (tx_max + msglen <= TX_BUFFER_SIZE) {
        memcpy(&tx_buf[tx_max], msg_buf, msglen);
        tx_max += msglen;
    }
    // else: output buffer full - drop the message
    pthread_mutex_unlock(&tx_mutex);
}

// Feed input data (public API, thread-safe)
ssize_t
klipper_host_input(const uint8_t *data, size_t len)
{
    if (!data || !len)
        return 0;
    pthread_mutex_lock(&rx_mutex);
    size_t space = RX_BUFFER_SIZE - rx_count;
    size_t n = len < space ? len : space;
    if (n) {
        memcpy(&rx_buf[rx_count], data, n);
        rx_count += n;
    }
    pthread_mutex_unlock(&rx_mutex);
    // Wake the console task to process the new data
    sched_wake_task(&console_wake);
    return n;
}

// Retrieve output data (public API, thread-safe)
size_t
klipper_host_output(uint8_t *buf, size_t buf_size)
{
    if (!buf || !buf_size)
        return 0;
    pthread_mutex_lock(&tx_mutex);
    size_t avail = tx_max - tx_pos;
    size_t n = avail < buf_size ? avail : buf_size;
    if (n) {
        memcpy(buf, &tx_buf[tx_pos], n);
        tx_pos += n;
    }
    if (tx_pos == tx_max)
        tx_pos = tx_max = 0;
    pthread_mutex_unlock(&tx_mutex);
    return n;
}

// Parse and dispatch complete message blocks from the receive buffer.
// Runs in the klipper task loop; holds rx_mutex while dispatching so
// that pointer arguments into rx_buf stay valid.
void
console_task(void)
{
    if (!sched_check_wake(&console_wake))
        return;
    pthread_mutex_lock(&rx_mutex);
    for (;;) {
        if (rx_count < MESSAGE_MIN)
            break;
        uint_fast8_t buf_len = rx_count > 255 ? 255 : (uint_fast8_t)rx_count;
        uint_fast8_t pop_count;
        int_fast8_t ret = command_find_and_dispatch(rx_buf, buf_len, &pop_count);
        if (!ret)
            // Need more data for a complete block
            break;
        // Block processed (ret > 0) or discarded/retransmit (ret < 0)
        memmove(rx_buf, &rx_buf[pop_count], rx_count - pop_count);
        rx_count -= pop_count;
    }
    pthread_mutex_unlock(&rx_mutex);
}
DECL_TASK(console_task);

// Unlock console mutexes. This is invoked both by the klipper shutdown
// handlers and by klipper_host_run() after an external shutdown longjmp,
// since a longjmp unwinding console_task can otherwise leave a mutex
// permanently locked.
void
klipper_console_unlock(void)
{
    pthread_mutex_unlock(&tx_mutex);
    pthread_mutex_unlock(&rx_mutex);
}
DECL_SHUTDOWN(klipper_console_unlock);
