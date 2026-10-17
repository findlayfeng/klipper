// Port objects: the ports a host creates at configuration time.
//
// Bridge mode gives the console more ports than the one it comes up with, and
// which wire each of those ports carries is a configuration fact, not a build
// one: the host creates a port object with config_uart (an oid it allocates, the
// line rate it wants, and the port slot whose tty it named with
// klipper_host_port), and then hands it to the bridge as a downstream port with
// config_bridge_port. So a bridge build opens no device on its own - a port it
// cannot bring up, or a rate it cannot set, is reported while the host is still
// configuring, instead of surfacing later as a port that never speaks.
//
// The object itself is the transport-neutral half (port.h): the oid, the rate,
// and the seam the arch fills (CONFIG_SERIAL_UART). Everything arch-shaped -
// opening the wire, writing queued blobs, pulsing reset lines - is the
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
#include "port.h" // the object layer's seam

// One port object. The rate is what the host asked for (a baud for a UART port,
// a bitrate for a CAN port); the wire behind it is the backend's, kept in 'ctx'
// when a backend needs per-object state.
struct port_s {
    uint32_t rate;
    void *ctx;
};

// Create a port object at the rate the host asks for, on the bridge port (the
// "slot" on the wire) the object belongs to. A duplicate oid is refused by
// oid_alloc; everything else that can go wrong is opening the wire, and each
// outcome gets its own message so the host learns why.
//
// The slot is the port's index on the bridge, the same number the host named the
// port's tty with before the device came up (klipper_host_port), so the backend
// opens exactly the wire that port was given.
void
command_config_uart(uint32_t *args)
{
    uint8_t oid = args[0];
    uint32_t baud = args[1];
    int slot = args[2];
    struct port_s *p = oid_alloc(oid, command_config_uart, sizeof(*p));
    p->rate = baud;
    p->ctx = NULL;
    if (!baud)
        shutdown("config_uart: baud must be nonzero");
    switch (port_backend_open(p, baud, slot)) {
    case PORT_OPEN_OK:
        break;
    case PORT_OPEN_BAD_BAUD:
        shutdown("config_uart: the UART cannot be set to the requested baud");
    case PORT_OPEN_BUSY:
        shutdown("config_uart: the port slot is already in use");
    case PORT_OPEN_NO_DEVICE:
    default:
        shutdown("config_uart: the UART device could not be opened");
    }
}
DECL_COMMAND(command_config_uart, "config_uart oid=%c baud=%u slot=%c");

// Create a CAN port object at the bitrate the host asks for. The same object as
// the UART command above - only the rate is a bus bitrate instead of a line
// baud, and the messages say so: a rate a CAN controller cannot be set to is
// not a baud the host could lower, it is the rate the bus it is on runs at.
//
// The wire is not picked here: this build offers one CAN controller to the
// bridge, so which slot config_can names on the wire cannot change what opens.
// The trailing slot argument is still on the wire (the host sends the port's
// index for now) and this command ignores it - the UART slot numbering is what
// port_backend_open() resolves, and a CAN port is not part of it.
void
command_config_can(uint32_t *args)
{
    uint8_t oid = args[0];
    uint32_t bitrate = args[1];
    // A CAN port is a port object like the UART one, so both commands allocate
    // it under config_uart's tag - the way config_spi_without_cs uses
    // config_spi's (src/spicmds.c). config_bridge_port looks an oid up by that
    // one tag, so it takes either command's object and does not care which made
    // it.
    struct port_s *p = oid_alloc(oid, command_config_uart, sizeof(*p));
    p->rate = bitrate;
    p->ctx = NULL;
    if (!bitrate)
        shutdown("config_can: bitrate must be nonzero");
#if CONFIG_BRIDGE_PORT_CAN
    switch (port_backend_can_open(p, bitrate)) {
    case PORT_OPEN_OK:
        break;
    case PORT_OPEN_BAD_BAUD:
        shutdown("config_can: the CAN port cannot be set to the requested"
                 " bitrate");
    case PORT_OPEN_BUSY:
        shutdown("config_can: the CAN port is already in use");
    case PORT_OPEN_NO_DEVICE:
    default:
        shutdown("config_can: the CAN controller could not be opened");
    }
#else
    shutdown("config_can: this build has no CAN downstream port");
#endif
}
DECL_COMMAND(command_config_can, "config_can oid=%c bitrate=%u slot=%c");

#if CONFIG_BRIDGE
// A bridge port object: the oid the host names the port by, and the port object
// it carries. The router owns the port itself (bridge.c); this keeps the oid's
// identity only so a second config_bridge_port on the same oid is refused by
// oid_alloc rather than silently overwriting the first port.
struct bridge_port_s {
    struct port_s *port;
    int index;
};

// Hand 'port' to the bridge as a downstream port. The port's index is the order
// the bridge took it in - the same order the host declared the port sections -
// and is what the port-level SET_PORT_NODEID addresses it by.
void
command_config_bridge_port(uint32_t *args)
{
    uint8_t oid = args[0];
    uint8_t port_oid = args[1];
    struct bridge_port_s *p = oid_alloc(oid, command_config_bridge_port
                                        , sizeof(*p));
    struct port_s *port = oid_lookup(port_oid, command_config_uart);
    int index = bridge_add_port(port_backend_port_ops(port), port);
    if (index < 0)
        shutdown("config_bridge_port: no room for another bridge port");
    p->port = port;
    p->index = index;
    port_backend_set_port(port, index);
}
DECL_COMMAND(command_config_bridge_port, "config_bridge_port oid=%c port=%c");
#endif // CONFIG_BRIDGE
