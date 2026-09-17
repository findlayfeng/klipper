// Thread-safe console I/O for klipper host library
//
// Implements console_sendf() and console_receive_buffer() as required
// by klipper's command infrastructure, using mutex-protected buffers
// instead of file descriptors.
//
//   - klipper_host_input():  appends bytes to the receive buffer and
//     wakes the console task (thread-safe).
//   - klipper_host_output(): drains the transmit buffer (thread-safe);
//     klipper_host_output_wait() additionally blocks until data arrives
//     or a timeout expires, for hosts that would otherwise have to poll.
//   - console_task():        parses and dispatches complete message
//     blocks from the receive buffer (runs in the klipper task loop).
//
// Both buffers are bounded, and neither path is allowed to lose part of
// a message: the firmware this library emulates blocks on a full
// transmit buffer and lets the operating system buffer its input, so a
// message is either taken whole or not taken at all. Callers therefore
// get backpressure (a short count, never a truncated message) instead
// of a stream that silently lost or split a block.
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
#include <stdio.h> // fprintf, stderr
#include <stdlib.h> // realloc, free
#include <string.h> // memcpy, memmove
#include <time.h> // clock_gettime, struct timespec
#include <pthread.h> // pthread_mutex_t, pthread_cond_t
#include "board/misc.h" // console_sendf, console_receive_buffer
#include "command.h" // command_encode_and_frame, command_find_and_dispatch
#include "sched.h" // sched_wake_task, DECL_TASK, DECL_SHUTDOWN
#include "internal.h" // klipper_console_unlock
#include "klipper_host.h" // klipper_host_input, klipper_host_output

// The transmit buffer starts at the size of the firmware's own receive buffer
// and grows while the host is behind, because the alternative - stalling klipper
// until the host reads - can wedge a single-threaded caller that is still busy
// feeding commands in. Growth is capped, and hitting the cap falls back to
// waiting and then (loudly) dropping, so a host that stops reading cannot make
// the library allocate without bound.
#define TX_BUFFER_MIN 4096
#define TX_BUFFER_MAX (256 * 1024)
#define RX_BUFFER_SIZE 4096

// How long a caller waits for buffer space before giving up. The console task
// drains the receive buffer on every scheduler pass and a host that reads its
// responses drains the transmit buffer within microseconds, so reaching these
// limits means the other side has stopped reading, not that it is slow.
#define INPUT_WAIT_MS 250
#define OUTPUT_WAIT_MS 250

// Output (klipper -> caller) buffer. tx_pos is the read offset, tx_max the
// number of bytes currently buffered, and tx_size how much has been allocated.
static uint8_t *tx_buf;
static size_t tx_size;
static volatile size_t tx_pos, tx_max;
static pthread_mutex_t tx_mutex = PTHREAD_MUTEX_INITIALIZER;
// Signalled when the buffer gains data (a waiter wants to read) or loses
// buffered bytes (a waiter wants room), so both directions broadcast.
static pthread_cond_t tx_cond = PTHREAD_COND_INITIALIZER;

// Input (caller -> klipper) buffer. rx_count is the number of bytes
// currently buffered at the start of rx_buf.
static uint8_t rx_buf[RX_BUFFER_SIZE];
static volatile size_t rx_count;
static pthread_mutex_t rx_mutex = PTHREAD_MUTEX_INITIALIZER;
// Signalled when console_task frees space in the receive buffer.
static pthread_cond_t rx_cond = PTHREAD_COND_INITIALIZER;

// Wake flag for the console task
static struct task_wake console_wake;

// Add milliseconds to an absolute deadline
static void
add_ms(struct timespec *ts, unsigned ms)
{
    ts->tv_sec += ms / 1000;
    ts->tv_nsec += (long)(ms % 1000) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec += 1;
        ts->tv_nsec -= 1000000000L;
    }
}

// Milliseconds remaining until an absolute deadline (0 once it has passed)
static unsigned
ms_left(const struct timespec *deadline)
{
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    long sec = (long)deadline->tv_sec - (long)now.tv_sec;
    long nsec = (long)deadline->tv_nsec - (long)now.tv_nsec;
    if (nsec < 0) {
        sec -= 1;
        nsec += 1000000000L;
    }
    if (sec < 0)
        return 0;
    return (unsigned)(sec * 1000 + nsec / 1000000L);
}

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
    if (tx_max + msglen > tx_size) {
        size_t used = tx_max - tx_pos;
        if (used)
            memmove(tx_buf, &tx_buf[tx_pos], used);
        tx_max = used;
        tx_pos = 0;

        // Then grow, so a host that is momentarily behind does not stall klipper.
        size_t wanted = tx_max + msglen;
        if (wanted > tx_size && tx_size < TX_BUFFER_MAX) {
            size_t size = tx_size ? tx_size : TX_BUFFER_MIN;
            while (size < wanted && size < TX_BUFFER_MAX)
                size *= 2;
            uint8_t *grown = realloc(tx_buf, size);
            if (grown) {
                tx_buf = grown;
                tx_size = size;
            }
        }

        // At the cap: wait for the host to read, the way the firmware stalls on a
        // full transmit buffer. Dropping the message would leave its caller
        // waiting for a response that can never arrive, so it is the last resort -
        // and it is reported rather than silent.
        struct timespec deadline;
        clock_gettime(CLOCK_REALTIME, &deadline);
        add_ms(&deadline, OUTPUT_WAIT_MS);
        while (tx_max + msglen > tx_size && ms_left(&deadline))
            pthread_cond_timedwait(&tx_cond, &tx_mutex, &deadline);
    }

    if (tx_max + msglen <= tx_size) {
        memcpy(&tx_buf[tx_max], msg_buf, msglen);
        tx_max += msglen;
        pthread_cond_broadcast(&tx_cond);
    } else {
        fprintf(stderr, "klipper host: no room for a %u byte response after "
                "%ums, dropping it\n", (unsigned)msglen, OUTPUT_WAIT_MS);
    }
    pthread_mutex_unlock(&tx_mutex);
}

// Feed input data (public API, thread-safe)
ssize_t
klipper_host_input(const uint8_t *data, size_t len)
{
    if (!data || !len)
        return 0;
    if (len > RX_BUFFER_SIZE)
        // One call has to fit: a partial copy would leave a truncated message
        // block in the buffer, which is worse than refusing the whole call.
        return 0;

    pthread_mutex_lock(&rx_mutex);
    if (rx_count + len > RX_BUFFER_SIZE) {
        // Wait for the console task to parse and drop what it has already been
        // given. Taking part of the data would split a message block.
        struct timespec deadline;
        clock_gettime(CLOCK_REALTIME, &deadline);
        add_ms(&deadline, INPUT_WAIT_MS);
        while (rx_count + len > RX_BUFFER_SIZE && ms_left(&deadline))
            pthread_cond_timedwait(&rx_cond, &rx_mutex, &deadline);
        if (rx_count + len > RX_BUFFER_SIZE) {
            pthread_mutex_unlock(&rx_mutex);
            return 0; // nothing taken - the caller can retry this frame
        }
    }
    memcpy(&rx_buf[rx_count], data, len);
    rx_count += len;
    pthread_mutex_unlock(&rx_mutex);
    // Wake the console task to process the new data
    sched_wake_task(&console_wake);
    return len;
}

// Retrieve output data (public API, thread-safe), optionally waiting
size_t
klipper_host_output_wait(uint8_t *buf, size_t buf_size, uint32_t timeout_ms)
{
    if (!buf || !buf_size)
        return 0;

    pthread_mutex_lock(&tx_mutex);
    if (tx_pos == tx_max && timeout_ms) {
        struct timespec deadline;
        clock_gettime(CLOCK_REALTIME, &deadline);
        add_ms(&deadline, timeout_ms);
        while (tx_pos == tx_max && ms_left(&deadline))
            pthread_cond_timedwait(&tx_cond, &tx_mutex, &deadline);
    }

    size_t avail = tx_buf ? tx_max - tx_pos : 0;
    size_t n = avail < buf_size ? avail : buf_size;
    if (n) {
        memcpy(buf, &tx_buf[tx_pos], n);
        tx_pos += n;
    }
    if (tx_pos == tx_max)
        tx_pos = tx_max = 0;
    pthread_mutex_unlock(&tx_mutex);

    // Reading freed room for a writer that was waiting for it.
    if (n)
        pthread_cond_broadcast(&tx_cond);
    return n;
}

// Retrieve output data without waiting (public API, thread-safe)
size_t
klipper_host_output(uint8_t *buf, size_t buf_size)
{
    return klipper_host_output_wait(buf, buf_size, 0);
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
    // Space was freed (or the buffer was never full): wake up a caller that is
    // waiting to hand over its next message.
    pthread_cond_broadcast(&rx_cond);
}
DECL_TASK(console_task);

// Wake both waiters: called when klipper is asked to shut down, so a host
// blocked in klipper_host_output_wait() does not sit out its timeout.
void
klipper_console_wake(void)
{
    pthread_cond_broadcast(&tx_cond);
    pthread_cond_broadcast(&rx_cond);
}

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
