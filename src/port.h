#ifndef __PORT_H
#define __PORT_H

#include <stdint.h> // uint32_t

// A bridge port object (CONFIG_UART): the oid a host creates at configuration
// time, the rate it asked for, and the seam below. It is the object a bridge
// port is made of, not a UART as such - the wire behind it is whichever backend
// the arch offers for the slot it was created on, and the two commands that
// create one differ only in the rate they carry: config_uart for a UART port (a
// baud), config_can for a CAN port (a bitrate) - both in src/port.c.
//
// The object is the transport-neutral half - the oid, the rate, and the seam
// below. Which wires a build can offer is the arch's (CONFIG_SERIAL_UART): the
// host library's backend is src/host/bridge.c, which opens the tty the host
// named for that slot (klipper_host_port, src/host/host.c) - or
// CONFIG_HOST_BRIDGE_SERIAL_PATH when it named none - and polls it from the
// task loop; a micro-controller arch would wire its serial task here instead.
//
// The rate and the port's wire are configuration facts, not build-time ones:
// config_uart carries the rate and the slot whose wire it opens, config_can the
// bitrate of this build's one CAN controller, so a rate the build cannot offer,
// a wire it cannot open, or a second object on a slot already open is reported
// while the host is configuring - not left behind as a dead port later.

struct port_s;

// How opening the wire behind a port object went.
enum port_open_result {
    PORT_OPEN_OK = 0,
    PORT_OPEN_NO_DEVICE,   // the wire could not be opened at all
    PORT_OPEN_BAD_BAUD,    // opened, but the rate could not be set on it
    PORT_OPEN_BUSY,        // the port slot already carries a port object
};

// Open the wire behind 'p' at 'rate' on bridge port 'slot' - the slot the
// object was created with, whose tty the host named before the device came up
// (klipper_host_port). 'rate' is the baud the host asked for when it created
// the object (config_uart). The slot selects the wire among the ones this build
// offers of one kind - the USART slots, or the host library's tty - so it is a
// UART port's call only: a CAN port has one controller and no slot to choose
// (port_backend_can_open). Called once per object, when the host configures it;
// a slot out of the build's range, or one object another already opened, is
// refused instead of served. The result names why a failure failed; the caller
// reports it (shutdown()) with a message of its own.
enum port_open_result port_backend_open(struct port_s *p, uint32_t rate,
                                        int slot);

// Open the wire behind 'p' at 'bitrate' for a CAN port (config_can). This build
// offers at most one CAN controller to the bridge, so there is nothing to pick:
// the slot config_can still carries on the wire is not passed here and plays no
// part in which wire opens. Same contract otherwise as port_backend_open(),
// which the UART port keeps using.
enum port_open_result port_backend_can_open(struct port_s *p, uint32_t bitrate);

#if CONFIG_BRIDGE
// The router's ops for 'p' as a downstream port: write() queues a whole blob on
// the object's wire and reset() pulses whatever lines reset the device behind
// it. 'ctx' is the object, so each port's bytes go to its own wire.
const struct bridge_port_ops *port_backend_port_ops(struct port_s *p);

// The bridge took 'p' as a downstream port with this index. A backend that
// polls its own wire (the host library) records it, so a byte it read can be
// fed to bridge_port_input() under the port it belongs to.
void port_backend_set_port(struct port_s *p, int index);
#endif

#endif // __PORT_H
