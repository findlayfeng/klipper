// Bridge mode: the linux process's byte-stream UART backend
//
// Bridge mode gives the console more ports and relays between them. The router
// is CONFIG_BRIDGE and the device half CONFIG_MUX, both transport-neutral (see
// src/Kconfig); this file is the arch backend that gives a bridge its stream on
// a linux process (CONFIG_LINUX_BRIDGE_SERIAL): it opens the tty the launcher
// named for the port when the host creates a UART object (config_uart,
// src/uart.c), puts it in the raw mode the protocol needs, registers it as a
// downstream port, and polls it from the task loop (bridge_poll_task) so its
// bytes reach the router next to the process's other work instead of blocking a
// thread on it, the way src/host/bridge.c polls its own ttys. The task loop
// sleeps when no task asks for another pass, so a timer of this port's own
// wakes it (bridge_poll_timer, one millisecond) - the same cadence the host
// library's bridge_poll gets from the sleep in its irq_wait.
//
// Which tty the port is on is the launcher's to say at run time, in the
// environment (KLIPPER_BRIDGE_PORT_0): the host that declares the port and the
// process that serves it are two processes, so unlike the host library - whose
// host names the port with klipper_host_port() in the same image - there is no
// call to hand the path over. A launcher sets the variable before the process
// starts; a port with no path in the environment fails its configuration
// (UART_OPEN_NO_DEVICE), the same way a tty that cannot be opened does.
//
// The port's rate is the parameter the host sent with config_uart; nothing here
// is opened at init, because which stream a port carries is a configuration
// fact. One stream is served: the whole backend is one static stream, and a
// second port (any slot but 0) is refused rather than silently sharing it. The
// limit is the honest one for a process with no port table of its own, and the
// host learns it while configuring, through the shutdown uart.c issues.
//
// There is no reset line to drive: a port's RESET_PORT (MUX_CMD_RESET_PORT) is
// dropped in silence, because a tty has no modem lines behind it that reach the
// device (the host backend's TIOCMGET answers for a pty exactly as this NULL
// does). A named device behind the port is unaffected - its restart travels the
// admin channel - while a legacy device gets no line-level reset here.
//
// The termios part below is src/host/bridge.c's, kept the same on purpose: the
// same job on the same kernel interface, so the two backends set a port up
// identically and neither drifts into a mode the protocol does not expect.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#define _GNU_SOURCE
#include <asm/termbits.h> // struct termios2, TCGETS2/TCSETS2, BOTHER
#include <errno.h> // errno
#include <fcntl.h> // open, O_RDWR, O_NOCTTY, O_NONBLOCK, O_CLOEXEC
#include <stdio.h> // fprintf, stderr
#include <stdlib.h> // getenv
#include <string.h> // memcpy, memmove, strerror
#include <sys/ioctl.h> // ioctl
#include <unistd.h> // close, read, write
#include "autoconf.h" // CONFIG_BRIDGE
#include "board/misc.h" // timer_from_us, timer_read_time
#include "bridge.h" // bridge_add_port, bridge_port_input
#include "command.h" // MESSAGE_MAX
#include "mux.h" // MUX_HEADER_BYTES
#include "sched.h" // DECL_TASK, sched_add_timer, sched_check_wake
#include "uart.h" // uart_backend_*

// The tty this process serves as its bridge port, read from the environment
// when the host declares the port. One name, because one port is served (see
// uart_backend_open): the launcher that starts this process sets it.
#define LINUX_BRIDGE_PORT_ENV "KLIPPER_BRIDGE_PORT_0"

// How often the port is polled once it is open, in microseconds.
//
// The task loop sleeps when no task asks for another pass - that is how a
// linux process idles - so the port needs something to wake it: this timer
// does, and its callback asks for the task (bridge_poll_timer_event). A
// millisecond is the cadence the host library's bridge_poll gets from the
// sleep its irq_wait does on every pass (IRQ_WAIT_MSECS, src/host/timer.c),
// and it is short enough that a device behind the port never waits on a stale
// read.
#define LINUX_BRIDGE_POLL_US 1000

// The one stream this backend serves. Static zero init gives it no owner and no
// fd, and 'owner' is what marks it open: it and 'fd' are written together, only
// after the open has fully succeeded, so an untouched stream never reads as one
// (fd 0 would, if fd alone marked it).
struct bridge_stream {
    int fd;                    // the tty, valid while the stream has an owner
    struct uart_s *owner;      // the object that opened it; NULL = no stream
    int port;                  // the router's port index, -1 once open
    // The queued output: the router hands each unit over as one blob and this
    // holds it until the pty takes it, so a unit is written whole and a later
    // one never cuts in front of it.
    uint8_t tx[2 * (MUX_HEADER_BYTES + MESSAGE_MAX)];
    size_t tx_len;
};
static struct bridge_stream bridge_stream;

// The timer that keeps the port polled, and the wake it gives bridge_poll_task:
// a linux process's task loop sleeps in irq_wait when no task asks for another
// pass, and this is what asks - a port whose device answers between two console
// bytes would otherwise be read whenever the console or a timer next woke the
// loop. The callback arms itself for the next pass, so the port is polled for
// as long as the process lives once a port exists to poll.
static struct task_wake bridge_wake;

static uint_fast8_t
bridge_poll_timer_event(struct timer *t)
{
    sched_wake_task(&bridge_wake);
    t->waketime = timer_read_time() + timer_from_us(LINUX_BRIDGE_POLL_US);
    return SF_RESCHEDULE;
}

static struct timer bridge_poll_timer = {
    .func = bridge_poll_timer_event,
};

// Report 'errno' in a message written to stderr, naming the call that failed,
// and answer -1 so callers can 'return bridge_errno("...")' directly.
static int
bridge_errno(const char *where)
{
    int e = errno;
    fprintf(stderr, "linux bridge serial: %s failed: (%d)%s\n", where, e
            , strerror(e));
    return -1;
}

// Put the port in raw 8N1 at 'baud'.
//
// The rate goes through termios2/BOTHER, which takes an arbitrary number:
// Klipper's 250000 is not one of the portable B* constants, and the usual
// alternative (TIOCSSERIAL) needs privileges. Raw mode is what cfmakeraw()
// would do - no echo, no line editing, no signal characters, no CR/LF
// translation and no software flow control - so the only bytes on the wire are
// the ones the protocol put there.
static int
set_raw_mode(int fd, unsigned int baud)
{
    struct termios2 tio;
    if (ioctl(fd, TCGETS2, &tio) < 0)
        return bridge_errno("TCGETS2");

    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR
                     | ICRNL | IXON);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | PARENB | CRTSCTS);
    tio.c_cflag |= CS8 | CREAD | CLOCAL;
    // The reads are non-blocking, so the two timeouts below say nothing; they
    // are set to "return what is there" so the port means the same thing if a
    // blocking read is ever put on it.
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    // Arbitrary rate: the kernel takes the numbers from c_ispeed/c_ospeed when
    // BOTHER is selected.
    tio.c_cflag = (tio.c_cflag & ~CBAUD) | BOTHER;
    tio.c_ispeed = baud;
    tio.c_ospeed = baud;

    if (ioctl(fd, TCSETS2, &tio) < 0)
        return bridge_errno("TCSETS2");
    return 0;
}

// Open the tty this process serves as its bridge port. Called from config_uart
// (src/uart.c) while the host is configuring, so a tty that is not there - or a
// path the launcher never named - fails the configuration rather than the
// process coming up without its port.
//
// 'slot' is the port's index on the bridge (the order the host declared its
// port sections in, src/bridge.c bridge_add_port): this backend serves the
// first one, and any other is refused, since there is one static stream behind
// this call and no tty of its own for a second port.
enum uart_open_result
uart_backend_open(struct uart_s *u, uint32_t baud, int slot)
{
    if (slot != 0) {
        fprintf(stderr, "linux bridge serial: only port slot 0 is served,"
                " not %d\n", slot);
        return UART_OPEN_NO_DEVICE;
    }
    struct bridge_stream *s = &bridge_stream;
    // This stream already carries a port: refuse the second object rather than
    // hand both of them the same fd.
    if (s->owner)
        return UART_OPEN_BUSY;

    const char *path = getenv(LINUX_BRIDGE_PORT_ENV);
    if (!path || !*path) {
        fprintf(stderr, "linux bridge serial: %s names no tty\n"
                , LINUX_BRIDGE_PORT_ENV);
        return UART_OPEN_NO_DEVICE;
    }
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        bridge_errno(path);
        return UART_OPEN_NO_DEVICE;
    }
    if (set_raw_mode(fd, baud) < 0) {
        close(fd);
        return UART_OPEN_BAD_BAUD;
    }
    // Both steps succeeded, and only now does the stream become open - the
    // owner marks it open - while a failure above leaves it exactly as it was,
    // with no half-open state behind it.
    s->fd = fd;
    s->owner = u;
    s->port = -1;
    s->tx_len = 0;
    // The port is read and its output flushed from the task loop, which sleeps
    // when no task asks for another pass, so the poll's own timer starts here -
    // the point where there is a port to poll. The loop's scheduler owns the
    // timer from now on: the callback re-arms it for as long as the process
    // lives, which is what a port it serves is for.
    bridge_poll_timer.waketime =
        timer_read_time() + timer_from_us(LINUX_BRIDGE_POLL_US);
    sched_add_timer(&bridge_poll_timer);
    fprintf(stderr, "linux bridge serial: %s open at %u baud (fd %d, slot %d)\n"
            , path, (unsigned int)baud, fd, slot);
    return UART_OPEN_OK;
}

#if CONFIG_BRIDGE
// The stream a router hook means: 'ctx' is the UART object the port carries
// (bridge_add_port() stored it as the port's ctx), and that object opened this
// backend's one stream, so the two agree exactly while it is open. NULL when
// the hook's object is not the one that opened it - an object whose config_uart
// failed never gets this far, but a hook must not assume that.
static struct bridge_stream *
bridge_stream_for(void *ctx)
{
    if (!ctx || bridge_stream.owner != ctx)
        return NULL;
    return &bridge_stream;
}

// Push as much of the queued output at the stream as its (non-blocking) fd
// takes.
static void
bridge_flush(struct bridge_stream *s)
{
    while (s->tx_len) {
        ssize_t n = write(s->fd, s->tx, s->tx_len);
        if (n > 0) {
            memmove(s->tx, &s->tx[n], s->tx_len - n);
            s->tx_len -= n;
            continue;
        }
        if (n < 0 && errno == EINTR)
            continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            return;              // the device will get it on the next pass
        bridge_errno("write");
        s->tx_len = 0;           // the port is gone: nothing more can be sent
        return;
    }
}

// The router's write hook: queue one whole blob, then give the port a chance.
// Nothing here blocks - the tail that the tty would not take stays queued for
// bridge_poll_task.
static int
bridge_port_write(void *ctx, const uint8_t *data, size_t len)
{
    struct bridge_stream *s = bridge_stream_for(ctx);
    if (!s)
        return -1;
    if (len > sizeof(s->tx) - s->tx_len)
        return -1;
    memcpy(&s->tx[s->tx_len], data, len);
    s->tx_len += len;
    bridge_flush(s);
    return 0;
}

static const struct bridge_port_ops bridge_serial_ops = {
    .write = bridge_port_write,
    // No modem lines behind a tty: a port's RESET_PORT is dropped in silence,
    // which is the port-layer convention for a backend without a reset line
    // (bridge.h, MUX_CMD_RESET_PORT).
    .reset = NULL,
};

// The router's ops for a UART object: one shared pair of hooks, because the
// hooks differ only by the stream they find, and there is one - 'ctx' (the
// object, carried by bridge_add_port) is what bridge_stream_for() checks.
const struct bridge_port_ops *
uart_backend_port_ops(struct uart_s *u)
{
    (void)u;
    return &bridge_serial_ops;
}

// The router gave the object this port index; the poll below feeds the bytes it
// reads from the stream under that port.
void
uart_backend_set_port(struct uart_s *u, int index)
{
    struct bridge_stream *s = bridge_stream_for(u);
    // An object that reached config_bridge_port has an open stream (config_uart
    // opened it or shut the process down), so there is always a stream to
    // record; without one there is no fd to poll and nothing to record.
    if (s)
        s->port = index;
}

// Poll the port: push out what the tty would not take last time, then read
// whatever the device sent and hand it to the router.
//
// This runs from the task loop, not from an ISR: the bytes are scanned and
// forwarded (bridge_port_input), which touches buffers the loop owns. Nothing
// here waits - the tty is non-blocking, and bridge_poll_timer is what brings
// the task back every millisecond - so a pass costs the loop only what the port
// has to say. Reading until the port is empty keeps a burst from waiting on the
// next pass.
void
bridge_poll_task(void)
{
    struct bridge_stream *s = &bridge_stream;
    // An unopened stream has no fd to poll, and one whose port the router has
    // not taken yet has no one to feed its bytes to.
    if (!s->owner || s->port < 0)
        return;
    // The timer is the only waker, and clearing its flag is what keeps one
    // wake from being served twice.
    if (!sched_check_wake(&bridge_wake))
        return;
    bridge_flush(s);
    uint8_t buf[256];
    for (;;) {
        ssize_t n = read(s->fd, buf, sizeof(buf));
        if (n > 0) {
            bridge_port_input(s->port, buf, n);
            continue;
        }
        if (n < 0 && errno == EINTR)
            continue;
        break;               // EAGAIN, or the port is gone
    }
}
DECL_TASK(bridge_poll_task);
#endif // CONFIG_BRIDGE
