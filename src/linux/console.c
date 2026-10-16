// TTY based IO
//
// Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#define _GNU_SOURCE
#include <errno.h> // errno
#include <fcntl.h> // fcntl
#include <poll.h> // ppoll
#include <pty.h> // openpty
#include <stdio.h> // fprintf
#include <string.h> // memmove
#include <sys/stat.h> // chmod
#include <time.h> // struct timespec
#include <unistd.h> // ttyname
#include "autoconf.h" // CONFIG_MUX_SERIAL
#include "board/irq.h" // irq_wait
#include "board/misc.h" // console_sendf
#include "command.h" // command_find_block
#include "internal.h" // console_setup
#include "mux.h" // mux_input, mux_out_header, mux_send
#include "sched.h" // sched_wake_task

// Fd 0 is the pty this file opens: the protocol console, or in a CAN build the
// marker of it. Fd 1 is a console's other source of input - the CAN socket
// (linux/can.c) - and stays -1 when there is none, an fd ppoll ignores.
static struct pollfd main_pfd[2] = { {.fd = -1}, {.fd = -1} };
#define MP_TTY_IDX   0
#define MP_EXT_IDX   1

// Report 'errno' in a message written to stderr
void
report_errno(char *where, int rc)
{
    int e = errno;
    fprintf(stderr, "Got error %d in %s: (%d)%s\n", rc, where, e, strerror(e));
}


/****************************************************************
 * Setup
 ****************************************************************/

int
set_non_blocking(int fd)
{
    int flags = fcntl(fd, F_GETFL);
    if (flags < 0) {
        report_errno("fcntl getfl", flags);
        return -1;
    }
    int ret = fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    if (ret < 0) {
        report_errno("fcntl setfl", flags);
        return -1;
    }
    return 0;
}

int
set_close_on_exec(int fd)
{
    int ret = fcntl(fd, F_SETFD, FD_CLOEXEC);
    if (ret < 0) {
        report_errno("fcntl set cloexec", ret);
        return -1;
    }
    return 0;
}

// Open the pseudo-tty named by the `-I` path and link that path to it.
//
// Without a CAN console the protocol runs over this pty. With one
// (CONFIG_CANSERIAL) the protocol moves to the CAN socket and this pty stays as
// the "firmware is up" marker a launcher watches: the `-I` path is a symlink to
// the tty, and a launcher waiting for that symlink sees the same thing in
// either build. So the pty is opened and linked unconditionally, and only its
// use as the protocol console is compiled out below.
int
console_setup(char *name)
{
    // Open pseudo-tty
    struct termios ti;
    memset(&ti, 0, sizeof(ti));
    int mfd, sfd, ret = openpty(&mfd, &sfd, NULL, &ti, NULL);
    if (ret) {
        report_errno("openpty", ret);
        return -1;
    }
    ret = set_non_blocking(mfd);
    if (ret)
        return -1;
    ret = set_close_on_exec(mfd);
    if (ret)
        return -1;
    ret = set_close_on_exec(sfd);
    if (ret)
        return -1;
    main_pfd[MP_TTY_IDX].fd = mfd;
    main_pfd[MP_TTY_IDX].events = POLLIN;

    // Create symlink to tty
    unlink(name);
    char *tname = ttyname(sfd);
    if (!tname) {
        report_errno("ttyname", 0);
        return -1;
    }
    ret = symlink(tname, name);
    if (ret) {
        report_errno("symlink", ret);
        return -1;
    }
    ret = chmod(tname, 0660);
    if (ret) {
        report_errno("chmod", ret);
        return -1;
    }

    // Make sure stderr is non-blocking
    ret = set_non_blocking(STDERR_FILENO);
    if (ret)
        return -1;

    return 0;
}

// Give this file a second descriptor to poll, and to wake the scheduler on: the
// CAN console's socket (linux/can.c), which no task here owns.
void
console_set_extra_fd(int fd)
{
    main_pfd[MP_EXT_IDX].fd = fd;
    main_pfd[MP_EXT_IDX].events = POLLIN;
}


#if !CONFIG_CANSERIAL
/****************************************************************
 * Console handling
 ****************************************************************/

static struct task_wake console_wake;
static uint8_t receive_buf[4096];
static int receive_pos;

void *
console_receive_buffer(void)
{
    return receive_buf;
}

#if CONFIG_MUX_SERIAL
// Where the console's own output queues up.
//
// The pty is non-blocking, so a write can take only part of what it is given.
// What must not happen is losing the tail: the mux receiver reassembles units
// from the byte stream anyway, but a unit dropped here loses a response its
// caller is waiting for, and a later write must not push another unit's bytes
// into the gap. Units are therefore queued whole, and whatever the pty will not
// take right now stays in the queue
// for console_flush() on the next scheduler pass. The queue is bounded, and a
// unit that does not fit is dropped and reported: waiting here would stall the
// whole micro-controller, and a full queue means the host has stopped reading,
// which no amount of waiting on this side can fix.
#define CONSOLE_TX_SIZE 4096
static uint8_t transmit_buf[CONSOLE_TX_SIZE];
static size_t transmit_pos, transmit_max;

// Write out as much of the queued bytes as the pty takes right now, keeping the
// unsent tail for the next pass.
static void
console_flush(void)
{
    while (transmit_pos < transmit_max) {
        int ret = write(main_pfd[MP_TTY_IDX].fd, &transmit_buf[transmit_pos]
                        , transmit_max - transmit_pos);
        if (ret > 0) {
            transmit_pos += ret;
            continue;
        }
        if (ret < 0 && errno == EWOULDBLOCK)
            // The pty is full - the host is behind on reading. Retry next pass.
            return;
        if (ret < 0) {
            // A pty that is gone or broken: dropping the queue beats wedging it.
            report_errno("write", ret);
            transmit_pos = transmit_max = 0;
        }
        return;
    }
    transmit_pos = transmit_max = 0;
}

// Append one whole unit (a frame, its mux header included) to the queue.
static void
console_queue(const uint8_t *bytes, uint_fast8_t len)
{
    console_flush();
    size_t used = transmit_max - transmit_pos;
    if (used + len > sizeof(transmit_buf)) {
        fprintf(stderr, "linux console: no room for a %u byte unit, dropping it\n"
                , (unsigned)len);
        return;
    }
    memmove(transmit_buf, &transmit_buf[transmit_pos], used);
    transmit_pos = 0;
    transmit_max = used + len;
    memcpy(&transmit_buf[used], bytes, len);
}
#endif // CONFIG_MUX_SERIAL

// Process any incoming commands
void
console_task(void)
{
#if CONFIG_MUX_SERIAL
    // Whatever the pty would not take last time goes out first, so the queue
    // drains on every scheduler pass even with no new input.
    console_flush();
#endif
    if (!sched_check_wake(&console_wake))
        return;

    // Read data
    int ret = read(main_pfd[MP_TTY_IDX].fd, &receive_buf[receive_pos]
                   , sizeof(receive_buf) - receive_pos);
    if (ret < 0) {
        if (errno == EWOULDBLOCK) {
            ret = 0;
        } else {
            report_errno("read", ret);
            return;
        }
    }
    if (ret == 15 && receive_buf[receive_pos+14] == '\n'
        && memcmp(&receive_buf[receive_pos], "FORCE_SHUTDOWN\n", 15) == 0)
        shutdown("Force shutdown command");

    // Find and dispatch message blocks in the input
    int len = receive_pos + ret;
#if CONFIG_MUX_SERIAL
    // Units first: the admin channel is answered in here, and one of our own
    // frames is left at the front with its header stripped, for the parser
    // below. A unit that has not arrived whole stops this pass - its leading
    // bytes are not a frame, and parsing them as one would lose the unit.
    size_t count = len;
    if (!mux_input(receive_buf, &count)) {
        receive_pos = (int)count;
        return;
    }
    len = (int)count;
#endif
    uint_fast8_t pop_count, msglen = len > MESSAGE_MAX ? MESSAGE_MAX : len;
    ret = command_find_and_dispatch(receive_buf, msglen, &pop_count);
    if (ret) {
        len -= pop_count;
        if (len) {
            memmove(receive_buf, &receive_buf[pop_count], len);
            sched_wake_task(&console_wake);
        }
    }
    receive_pos = len;
}
DECL_TASK(console_task);

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
#if CONFIG_MUX_SERIAL
    // Our own frames go out as units: the reply header names this device the
    // way the incoming unit named it, and it writes nothing while there is no
    // node id - which keeps the old wire format (bare frames).
    uint8_t buf[MUX_HEADER_BYTES + MESSAGE_MAX];
    uint_fast8_t offset = mux_out_header(buf);
    uint_fast8_t msglen = command_encode_and_frame(
        buf + offset, sizeof(buf) - offset, ce, args);
    if (!msglen)
        return;
    console_queue(buf, offset + msglen);
#else
    // Generate message
    uint8_t buf[MESSAGE_MAX];
    uint_fast8_t msglen = command_encode_and_frame(buf, sizeof(buf), ce, args);
    if (!msglen)
        return;

    // Transmit message
    int ret = write(main_pfd[MP_TTY_IDX].fd, buf, msglen);
    if (ret < 0)
        report_errno("write", ret);
#endif
}

#if CONFIG_MUX_SERIAL
// Bytes the mux sends upward: its answers on the admin channel and the units a
// bridge forwards, whole units already (mux.h mux_send). They share the one
// transmit queue the responses drain, so units never interleave on the wire.
void
mux_send(const uint8_t *bytes, uint_fast8_t len)
{
    console_queue(bytes, len);
}
#endif // CONFIG_MUX_SERIAL
#endif // !CONFIG_CANSERIAL

// Sleep until a signal received (waking early for console input if needed)
void
console_sleep(sigset_t *sigset)
{
    int ret = ppoll(main_pfd, ARRAY_SIZE(main_pfd), NULL, sigset);
    if (ret <= 0) {
        if (errno != EINTR)
            report_errno("ppoll main_pfd", ret);
        return;
    }
#if CONFIG_CANSERIAL
    if (main_pfd[MP_EXT_IDX].revents)
        can_rx_notify();
#else
    if (main_pfd[MP_TTY_IDX].revents)
        sched_wake_task(&console_wake);
#endif
}
