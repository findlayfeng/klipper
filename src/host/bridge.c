// Bridge mode: the host's downstream serial port
//
// Bridge mode gives the console more ports and relays between them. The router
// is CONFIG_BRIDGE and the device half CONFIG_MUX, both transport-neutral (see
// src/Kconfig); which ports a bridge owns is the arch's to say, and this file
// is the host's serial port backend (CONFIG_BRIDGE_PORT_SERIAL): it opens the
// tty during klipper_host_init() and puts it in the raw mode the protocol
// needs.
//
// Feeding the port into the console - a reader and a writer in front of
// console_sendf()/console_receive_buffer() - is the next step, together with
// the device half (being addressed, answering the scan/admin channel, and the
// identity the host assigns) in the common layer.
// The port is opened non-blocking, so whoever reads it later can poll it next
// to the console's other work instead of blocking a thread on it, the way
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
#include "autoconf.h" // CONFIG_BRIDGE_PORT_SERIAL*
#include "internal.h" // bridge_setup

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
    fprintf(stderr, "bridge serial: %s open at %u baud (fd %d)\n", path,
            (unsigned int)CONFIG_BRIDGE_PORT_SERIAL_BAUD, fd);
    return 0;
}
