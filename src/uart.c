// UART objects: the byte streams a host creates at configuration time.
//
// Bridge mode gives the console more ports than the one it comes up with, and
// which stream each of those ports carries is a configuration fact, not a build
// one: the host creates a UART object with config_uart (an oid it allocates, the
// line rate it wants, and the port slot whose tty it named with
// klipper_host_port), and then hands it to the bridge as a downstream port with
// config_bridge_port. So a bridge build opens no device on its own - a stream it
// cannot bring up, or a rate it cannot set, is reported while the host is still
// configuring, instead of surfacing later as a port that never speaks.
//
// The object itself is the transport-neutral half (uart.h): the oid, the rate,
// and the seam the arch fills (CONFIG_SERIAL_UART). Everything arch-shaped -
// opening the stream, writing queued blobs, pulsing reset lines - is the
// backend's; src/host/bridge.c is the host library's.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_BRIDGE, CONFIG_UART
#include "basecmd.h" // oid_alloc, oid_lookup
#include "bridge.h" // bridge_add_port
#include "command.h" // DECL_COMMAND, shutdown
#include "sched.h" // sched_shutdown (used by shutdown())
#include "uart.h" // the object layer's seam

// One UART object. The rate is what the host asked for; the stream behind it is
// the backend's, kept in 'ctx' when a backend needs per-object state.
struct uart_s {
    uint32_t baud;
    void *ctx;
};

// Create a UART object at the rate the host asks for, on the bridge port (the
// "slot" on the wire) the object belongs to. A duplicate oid is refused by
// oid_alloc; everything else that can go wrong is opening the stream, and each
// outcome gets its own message so the host learns why.
//
// The slot is the port's index on the bridge, the same number the host named the
// port's tty with before the device came up (klipper_host_port), so the backend
// opens exactly the stream that port was given.
void
command_config_uart(uint32_t *args)
{
    uint8_t oid = args[0];
    uint32_t baud = args[1];
    int slot = args[2];
    struct uart_s *u = oid_alloc(oid, command_config_uart, sizeof(*u));
    u->baud = baud;
    u->ctx = NULL;
    if (!baud)
        shutdown("config_uart: baud must be nonzero");
    switch (uart_backend_open(u, baud, slot)) {
    case UART_OPEN_OK:
        break;
    case UART_OPEN_BAD_BAUD:
        shutdown("config_uart: the UART cannot be set to the requested baud");
    case UART_OPEN_BUSY:
        shutdown("config_uart: this build's UART is already in use");
    case UART_OPEN_NO_DEVICE:
    default:
        shutdown("config_uart: the UART device could not be opened");
    }
}
DECL_COMMAND(command_config_uart, "config_uart oid=%c baud=%u slot=%c");

#if CONFIG_BRIDGE
// A bridge port object: the oid the host names the port by, and the UART object
// it carries. The router owns the port itself (bridge.c); this keeps the oid's
// identity only so a second config_bridge_port on the same oid is refused by
// oid_alloc rather than silently overwriting the first port.
struct bridge_port_s {
    struct uart_s *uart;
    int index;
};

// Hand 'uart' to the bridge as a downstream port. The port's index is the order
// the bridge took it in - the same order the host declared the port sections -
// and is what the port-level SET_PORT_NODEID addresses it by.
void
command_config_bridge_port(uint32_t *args)
{
    uint8_t oid = args[0];
    uint8_t uart_oid = args[1];
    struct bridge_port_s *p = oid_alloc(oid, command_config_bridge_port
                                        , sizeof(*p));
    struct uart_s *u = oid_lookup(uart_oid, command_config_uart);
    int index = bridge_add_port(uart_backend_port_ops(u), u);
    if (index < 0)
        shutdown("config_bridge_port: no room for another bridge port");
    p->uart = u;
    p->index = index;
    uart_backend_set_port(u, index);
}
DECL_COMMAND(command_config_bridge_port, "config_bridge_port oid=%c uart=%c");
#endif // CONFIG_BRIDGE
