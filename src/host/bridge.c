// Bridge mode: the host library's byte-stream UART backend
//
// Bridge mode gives the console more ports and relays between them. The router
// is CONFIG_BRIDGE and the device half CONFIG_MUX, both transport-neutral (see
// src/Kconfig); this file is the arch backend that gives a bridge its streams on
// the host library (CONFIG_BRIDGE_PORT_SERIAL): it opens the tty the host named
// for that port when the host creates a port object (config_uart, src/port.c),
// puts it in the raw mode the protocol needs, registers it as a downstream
// port, and polls it from the task loop's sleep (bridge_poll, called by
// irq_wait) so its bytes reach the router next to the console's other work
// instead of blocking a thread on it, the way src/linux/console.c polls its
// tty.
//
// Nothing here runs at init: which port a bridge owns is a configuration fact,
// so the stream is opened while the host is configuring, and a tty that cannot
// be opened (or a rate it cannot set) fails that configuration explicitly
// instead of leaving a bridge that comes up without its port.
//
// Which tty a port is on is the host's to say at run time: it names one per port
// with klipper_host_port() before klipper comes up (host.c), and the port object
// carries the port's slot, so the stream opened here is the one that port was
// named. A slot the host never named falls back to CONFIG_HOST_BRIDGE_SERIAL_PATH,
// the path a plain bridge build opens.
//
// This backend serves one stream per port slot: every port object the host
// creates opens its own slot's tty and keeps its own output queue and reset
// pulse beside it, so up to KLIPPER_HOST_PORT_SLOTS of them are live at once.
// A second port object on a slot that is already open is refused
// (PORT_OPEN_BUSY) rather than silently sharing the fd. Which tty a port is on
// is not a serial parameter; the rate is the parameter, and it arrives with
// config_uart.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#define _GNU_SOURCE
#include <asm/termbits.h> // struct termios2, TCGETS2/TCSETS2, BOTHER
#include <errno.h> // errno
#include <fcntl.h> // open, O_RDWR, O_NOCTTY, O_NONBLOCK
#include <stdio.h> // fprintf, stderr
#include <string.h> // strerror
#include <sys/ioctl.h> // ioctl
#include <unistd.h> // close
#include "autoconf.h" // CONFIG_BRIDGE, CONFIG_HOST_BRIDGE_SERIAL_PATH
#include "bridge.h" // bridge_add_port, bridge_port_input
#include "command.h" // DECL_CONSTANT, MESSAGE_MAX
#include "generic/misc.h" // timer_read_time, timer_is_before, timer_from_us
#include "mux.h" // MUX_HEADER_BYTES
#include "internal.h" // bridge_poll
#include "klipper_host.h" // KLIPPER_HOST_PORT_SLOTS
#include "port.h" // port_backend_*

// How many port objects this backend can give the bridge: one per port slot of
// the host's table, because that table is what a slot's tty is named in
// (klipper_host_port) and port_backend_open() refuses any other slot. The count
// is the backend's fact - the same table it enforces - and the host reads it
// back as BRIDGE_UART_COUNT to list the ports a bridge can be given before it
// declares any (klipperx's `admin bridge config`).
DECL_CONSTANT("BRIDGE_UART_COUNT", KLIPPER_HOST_PORT_SLOTS);

// One open stream per port slot: the table is exactly the host's port slot
// table (KLIPPER_HOST_PORT_SLOTS, host/klipper_host.h). Static zero init gives
// every slot no owner, and a slot is open exactly while it has one - 'owner'
// and 'fd' are written together, only after the open has fully succeeded, so
// an untouched slot never reads as a stream (fd 0 would, if fd alone marked
// it). One slot carries one stream: a second UART object on a slot already
// open is refused rather than sharing the fd, while different slots open
// independently and are live together.
//
// The router's hooks reach a slot through the port object that opened it
// ('owner'): bridge_add_port() hands that object to the router as the port's
// ctx, and it is what write()/reset() carry back. 'port' is the index the
// router gave that object, so bytes read from this fd are fed to
// bridge_port_input() under the right port.
struct bridge_stream {
    int fd;                    // the tty, valid while the slot has an owner
    struct port_s *owner;      // the object that opened it; NULL = no stream
    int port;                  // the router's port index, -1 once open (a slot
                               // no object opened is guarded by its NULL owner)
    // The queued output: the router hands each unit over as one blob and this
    // holds it until the device can take it, so a unit is written whole and a
    // later one never cuts in front of it.
    uint8_t tx[2 * (MUX_HEADER_BYTES + MESSAGE_MAX)];
    size_t tx_len;
    // The reset pulse in flight for this slot: whether one is active, when its
    // hold expires, and the modem bits to put back. Each slot has its own, so
    // one port's reset neither reads nor disturbs another's.
    int reset_active;
    uint32_t reset_restore_at;
    int reset_saved;
};
static struct bridge_stream bridge_streams[KLIPPER_HOST_PORT_SLOTS];

// Report 'errno' in a message written to stderr, naming the call that failed,
// and answer -1 so callers can 'return bridge_errno("...")' directly.
static int
bridge_errno(const char *where)
{
    int e = errno;
    fprintf(stderr, "bridge serial: %s failed: (%d)%s\n", where, e, strerror(e));
    return -1;
}

// Put the port in raw 8N1 at 'baud'.
//
// The rate goes through termios2/BOTHER, which takes an arbitrary number:
// Klipper's 250000 is not one of the portable B* constants, and the usual
// alternative (TIOCSSERIAL) needs privileges.
//
// Raw mode is what cfmakeraw() would do: no echo, no line editing, no signal
// characters, no CR/LF translation and no software flow control, so the only
// bytes on the wire are the ones the protocol put there.
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

// Open the tty the port object sits on. Called from config_uart (src/port.c)
// while the host is configuring, so a tty that is not there fails the
// configuration rather than the library coming up without its port.
//
// 'slot' is the port's index on the bridge, which is the number the host named
// its tty with (klipper_host_port, host.c); a slot it never named opens the path
// this build was configured with.
enum port_open_result
port_backend_open(struct port_s *p, uint32_t rate, int slot)
{
    // The slot is where this object lives on the backend. A slot outside the
    // host's table could never be named a tty (klipper_host_port refuses it),
    // so it is refused here too instead of falling back to the build's path
    // with no host table row behind it.
    if (slot < 0 || slot >= KLIPPER_HOST_PORT_SLOTS) {
        fprintf(stderr, "bridge serial: slot %d is outside 0..%d\n", slot,
                KLIPPER_HOST_PORT_SLOTS - 1);
        return PORT_OPEN_NO_DEVICE;
    }
    struct bridge_stream *s = &bridge_streams[slot];
    // This slot already carries a stream: refuse the second object rather than
    // hand both of them the same fd. Other slots are unaffected and open on
    // their own.
    if (s->owner)
        return PORT_OPEN_BUSY;

    const char *path = klipper_host_port_path(slot);
    if (!path)
        path = CONFIG_HOST_BRIDGE_SERIAL_PATH;
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        bridge_errno(path);
        return PORT_OPEN_NO_DEVICE;
    }
    if (set_raw_mode(fd, rate) < 0) {
        close(fd);
        return PORT_OPEN_BAD_BAUD;
    }
    // Both steps succeeded, and only now does the slot become open - the owner
    // marks it open - while a failure above leaves the slot exactly as it was,
    // with no half-open state behind it.
    s->fd = fd;
    s->owner = p;
    s->port = -1;
    s->tx_len = 0;
    s->reset_active = 0;
    fprintf(stderr, "bridge serial: %s open at %u baud (fd %d, slot %d)\n",
            path, (unsigned int)rate, fd, slot);
    return PORT_OPEN_OK;
}

#if CONFIG_BRIDGE
// The stream a router hook means: 'ctx' is the port object the port carries
// (bridge_add_port() stored it as the port's ctx), and that object opened
// exactly one slot, so the table answers which stream the hook drives. NULL
// when no open slot carries it - an object whose config_uart failed never gets
// this far, but a hook must not assume that.
static struct bridge_stream *
bridge_stream_for(void *ctx)
{
    struct port_s *p = ctx;
    int i;
    if (!p)
        return NULL;
    for (i = 0; i < KLIPPER_HOST_PORT_SLOTS; i++)
        if (bridge_streams[i].owner == p)
            return &bridge_streams[i];
    return NULL;
}

// Push as much of the queued output at the stream as its (non-blocking) fd
// takes. Each stream has its own queue, so a port blocked on its device never
// holds up another port's bytes.
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
            return;              // the device will get it on the next task
        bridge_errno("write");
        s->tx_len = 0;           // the port is gone: nothing more can be sent
        return;
    }
}

// The router's write hook: queue one whole blob on the calling port's stream,
// then give that port a chance.
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

// The router's reset hook: put the port's lines in their reset state and
// schedule the release. Returns 0 once the pulse is started, -1 when the port
// has no modem lines to drive (a pty, or an fd that refused the ioctl) - the
// command carries no receipt, so the caller drops either result in silence.
//
// The pulse is a state machine driven from the task loop's poll, not a sleep:
// the lines assert now, and the release happens when bridge_reset_poll() sees
// the hold below expire against the loop's own clock. Nothing here blocks or
// busy-waits the scheduler.
//
// The reset convention is the RS-232 one: asserting DTR (and RTS) drives the
// lines that many MCUs strap to their reset input. A port whose device resets
// differently is a different backend; the hold is the bounded 250ms the
// protocol allows, and the reset layer (reset-only vs bootloader entry) is the
// host's to choose later - this only pulses the line.
//
// The hook drives one slot only: ctx names the UART object, hence that slot's
// own stream, and every open stream carries its own hold below, so ports reset
// independently of each other.
#define BRIDGE_RESET_HOLD_US 250000

static int
bridge_port_reset(void *ctx)
{
    struct bridge_stream *s = bridge_stream_for(ctx);
    if (!s)
        return -1;
    // A pulse is already in flight on this slot: take the request as done
    // rather than re-reading the asserted lines as "original state", which
    // would restore the asserted level at the deadline and hold the device in
    // reset forever. The guard is per slot: another port's pulse in flight does
    // not make this one done.
    if (s->reset_active)
        return 0;
    int bits;
    if (ioctl(s->fd, TIOCMGET, &bits) < 0)
        return -1;               // no modem lines: a pty or similar
    s->reset_saved = bits;
    int out = bits | TIOCM_DTR | TIOCM_RTS;
    if (ioctl(s->fd, TIOCMSET, &out) < 0)
        return -1;
    s->reset_restore_at =
        timer_read_time() + timer_from_us(BRIDGE_RESET_HOLD_US);
    s->reset_active = 1;
    return 0;
}

// Release a reset pulse on 's' whose hold has run out. Called from the task
// loop's poll (bridge_poll) for every open stream, so the hold costs the loop
// no sleeping: the lines simply stay asserted until this sees the deadline
// past and puts them back.
static void
bridge_reset_poll(struct bridge_stream *s)
{
    if (!s->reset_active)
        return;
    if (timer_is_before(timer_read_time(), s->reset_restore_at))
        return;                   // still holding the port in reset
    // The hold is over: put the lines back as they were. A port that is already
    // gone has nothing to restore, so the flag clears either way.
    int bits = s->reset_saved;
    ioctl(s->fd, TIOCMSET, &bits);
    s->reset_active = 0;
}

static const struct bridge_port_ops bridge_serial_ops = {
    .write = bridge_port_write,
    .reset = bridge_port_reset,
};

// The router's ops for a port object: one shared pair of hooks for every port,
// because the hooks differ only by the stream they find - 'ctx' (the object,
// carried by bridge_add_port) selects its slot in the table above.
const struct bridge_port_ops *
port_backend_port_ops(struct port_s *p)
{
    (void)p;
    return &bridge_serial_ops;
}

// The router gave the object this port index; the poll below feeds the bytes it
// reads from that object's slot under that port. Each open slot records its own
// index, so the ports never overwrite each other's.
void
port_backend_set_port(struct port_s *p, int index)
{
    struct bridge_stream *s = bridge_stream_for(p);
    // An object that reached config_bridge_port has an open slot (config_uart
    // opened it or shut the device down), so there is always a stream to
    // record; without one there is no fd to poll and nothing to record.
    if (s)
        s->port = index;
}

// Read whatever each downstream device has sent and hand it to the router.
//
// This runs from inside the task loop's sleep (irq_wait), where the host
// emulation already yields for a millisecond: the fds are non-blocking, so the
// ports are polled next to the console's other work instead of blocking a
// thread on them, the way src/linux/console.c polls its tty. Reading a port
// until it is empty keeps a burst from waiting on the next sleep, and every
// open port gets its own turn each pass.
void
bridge_poll(void)
{
    int i;
    for (i = 0; i < KLIPPER_HOST_PORT_SLOTS; i++) {
        struct bridge_stream *s = &bridge_streams[i];
        // An unopened slot has no stream to poll, and one whose port the
        // router has not taken yet has no one to feed its bytes to.
        if (!s->owner || s->port < 0)
            continue;
        bridge_reset_poll(s);
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
}
#endif // CONFIG_BRIDGE
