// Bridge mode: the host's downstream serial port
//
// Bridge mode gives the console more ports and relays between them. The router
// is CONFIG_BRIDGE and the device half CONFIG_MUX, both transport-neutral (see
// src/Kconfig); which ports a bridge owns is the arch's to say, and this file
// is the host's serial port backend (CONFIG_BRIDGE_PORT_SERIAL): it opens the
// tty during klipper_host_init() and puts it in the raw mode the protocol
// needs.
//
// Feeding the port into the console is bridge.c's job (the router): this file
// opens the tty, registers it as a downstream port, and polls it from the task
// loop's sleep (bridge_poll, called by irq_wait) so its bytes reach the router
// next to the console's other work instead of blocking a thread on it, the way
// src/linux/console.c polls its tty.
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
#include "autoconf.h" // CONFIG_BRIDGE_PORT_SERIAL*, CONFIG_BRIDGE
#include "bridge.h" // bridge_add_port, bridge_port_input
#include "command.h" // MESSAGE_MAX, DECL_CONSTANT*
#include "generic/misc.h" // timer_read_time, timer_is_before, timer_from_us
#include "mux.h" // MUX_HEADER_BYTES
#include "internal.h" // bridge_setup, bridge_poll

// The bridge's serial port, or -1 when it has not been opened.
static int bridge_fd = -1;

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

#if CONFIG_BRIDGE
// The router's end of the port. bridge.c hands each unit over as one blob and
// this holds it until the device can take it, so a unit is written whole and a
// later one never cuts in front of it.
static uint8_t bridge_tx[2 * (MUX_HEADER_BYTES + MESSAGE_MAX)];
static size_t bridge_tx_len;
static int bridge_port = -1;

// Push as much of the queued output at the port as the (non-blocking) fd takes.
static void
bridge_flush(void)
{
    while (bridge_tx_len) {
        ssize_t n = write(bridge_fd, bridge_tx, bridge_tx_len);
        if (n > 0) {
            memmove(bridge_tx, &bridge_tx[n], bridge_tx_len - n);
            bridge_tx_len -= n;
            continue;
        }
        if (n < 0 && errno == EINTR)
            continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            return;              // the device will get it on the next task
        bridge_errno("write");
        bridge_tx_len = 0;       // the port is gone: nothing more can be sent
        return;
    }
}

// The router's write hook: queue one whole blob, then give the port a chance.
static int
bridge_port_write(void *ctx, const uint8_t *data, size_t len)
{
    (void)ctx;
    if (len > sizeof(bridge_tx) - bridge_tx_len)
        return -1;
    memcpy(&bridge_tx[bridge_tx_len], data, len);
    bridge_tx_len += len;
    bridge_flush();
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
// The hook ignores ctx: this backend serves exactly one port and drives the
// global bridge_fd. A second port in this backend would have to carry its own
// fd, and this would have to become per-port state.
#define BRIDGE_RESET_HOLD_US 250000
static int bridge_reset_active;           // a reset pulse is in flight
static uint32_t bridge_reset_restore_at;  // when the lines come back
static int bridge_reset_saved;            // the modem bits to put back

static int
bridge_port_reset(void *ctx)
{
    (void)ctx;
    if (bridge_fd < 0)
        return -1;
    // A pulse is already in flight: take the request as done rather than
    // re-reading the asserted lines as "original state", which would restore
    // the asserted level at the deadline and hold the device in reset forever.
    if (bridge_reset_active)
        return 0;
    int bits;
    if (ioctl(bridge_fd, TIOCMGET, &bits) < 0)
        return -1;               // no modem lines: a pty or similar
    bridge_reset_saved = bits;
    int out = bits | TIOCM_DTR | TIOCM_RTS;
    if (ioctl(bridge_fd, TIOCMSET, &out) < 0)
        return -1;
    bridge_reset_restore_at =
        timer_read_time() + timer_from_us(BRIDGE_RESET_HOLD_US);
    bridge_reset_active = 1;
    return 0;
}

// Release a reset pulse whose hold has run out. Called from the task loop's
// poll (bridge_poll), so the hold costs the loop no sleeping: the lines simply
// stay asserted until this sees the deadline past and puts them back.
static void
bridge_reset_poll(void)
{
    if (!bridge_reset_active)
        return;
    if (timer_is_before(timer_read_time(), bridge_reset_restore_at))
        return;                   // still holding the port in reset
    // The hold is over: put the lines back as they were. A port that is already
    // gone has nothing to restore, so the flag clears either way.
    int bits = bridge_reset_saved;
    ioctl(bridge_fd, TIOCMSET, &bits);
    bridge_reset_active = 0;
}

static const struct bridge_port_ops bridge_serial_ops = {
    .write = bridge_port_write,
    .reset = bridge_port_reset,
};

// The port table this bridge reports in its identify dictionary: today the one
// serial port above, named "uart0" - the oid the host config refers to the port
// by. The name belongs to the firmware (the host must never guess it), and the
// index its row carries (0 here) is what the port-level SET_PORT_NODEID travels
// with, so this table is how the host learns it.
DECL_CONSTANT("BRIDGE_PORT_COUNT", 1);
DECL_CONSTANT_STR("BRIDGE_PORT0_NAME", "uart0");
DECL_CONSTANT_STR("BRIDGE_PORT0_TYPE", "uart");
DECL_CONSTANT_STR("BRIDGE_PORT0_PATH", CONFIG_HOST_BRIDGE_SERIAL_PATH);

// Read whatever the downstream device has sent and hand it to the router.
//
// This runs from inside the task loop's sleep (irq_wait), where the host
// emulation already yields for a millisecond: the fd is non-blocking, so the
// port is polled next to the console's other work instead of blocking a thread
// on it, the way src/linux/console.c polls its tty. Reading until the port is
// empty keeps a burst from waiting on the next sleep.
void
bridge_poll(void)
{
    if (bridge_fd < 0 || bridge_port < 0)
        return;
    bridge_reset_poll();
    bridge_flush();
    uint8_t buf[256];
    for (;;) {
        ssize_t n = read(bridge_fd, buf, sizeof(buf));
        if (n > 0) {
            bridge_port_input(bridge_port, buf, n);
            continue;
        }
        if (n < 0 && errno == EINTR)
            continue;
        break;                   // EAGAIN, or the port is gone
    }
}
#endif // CONFIG_BRIDGE

// Open the bridge's serial port. Called once from klipper_host_init(); returns
// 0 when the port is ready, -1 when it cannot be brought up.
//
// A failure is not swallowed: bridge mode was asked for at build time, so a
// library that comes up without its port would be lying about what it is.
int
bridge_setup(void)
{
    if (bridge_fd >= 0)
        return 0;

    const char *path = CONFIG_HOST_BRIDGE_SERIAL_PATH;
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
        return bridge_errno(path);
    if (set_raw_mode(fd, CONFIG_BRIDGE_PORT_SERIAL_BAUD) < 0) {
        close(fd);
        return -1;
    }
    bridge_fd = fd;
#if CONFIG_BRIDGE
    // The bridge's first downstream port: this tty. It registers with no flags
    // - a legacy port until the host says what is behind it, which it does with
    // the port-level SET_PORT_NODEID once it knows (the table above is what
    // tells it which index this port answers to).
    bridge_port = bridge_add_port(&bridge_serial_ops);
#endif
    fprintf(stderr, "bridge serial: %s open at %u baud (fd %d)\n", path,
            (unsigned int)CONFIG_BRIDGE_PORT_SERIAL_BAUD, fd);
    return 0;
}
