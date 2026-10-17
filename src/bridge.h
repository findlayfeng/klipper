#ifndef __BRIDGE_H
#define __BRIDGE_H

#include <stddef.h> // size_t
#include <stdint.h> // uint8_t, uint32_t

// The bridge: the router that gives the console more than one port.
//
// A bridge owns downstream ports and forwards units between them. Its own
// klipper instance is one more port - the "self" port, which is just the device
// half in mux.c - and every other port is a path the arch provides. This header
// is the seam between the transport-neutral router (bridge.c) and those ports:
// the arch registers one struct bridge_port_ops per port and feeds the bytes it
// reads into bridge_port_input(); the router writes a port's bytes through the
// ops' write().
//
// The upstream side is the console byte stream, and mux.c's mux_input() is the
// loop that reads it: it calls the two hooks below where a unit is either an
// admin broadcast (fan it out) or a data unit this device does not own (route
// it by id). A build without CONFIG_BRIDGE has neither the router nor the
// hooks, and mux_input() drops those units exactly as it did before.
//
// The wire format is the klipperx checkout's developer manual
// (serial-mux-protocol.md).

// What the router needs from one downstream port.
struct bridge_port_ops {
    // Write one whole blob - a unit (header + frame) or, on a single-device
    // path, a raw frame - to the port. The blob is queued whole: the backend
    // buffers what the device cannot take right now, so units never interleave
    // on a port. Returns 0 when the bytes were queued, -1 on an error the
    // backend reports itself.
    int (*write)(void *ctx, const uint8_t *data, size_t len);
    // Pulse the electrical lines that reset whatever sits behind this port -
    // the port-level RESET_PORT (mux.h MUX_CMD_RESET_PORT), which is the only
    // reset a device with no admin channel of its own can ever get. Optional:
    // NULL for a port with no line-level reset, whose RESET_PORT is dropped in
    // silence. Returns 0 when the pulse was started and -1 when the port has no
    // modem lines to drive (a pty, for instance). The command carries no
    // receipt, so neither value is reported onward; a backend must not block or
    // busy-wait in here - it schedules the release and returns.
    int (*reset)(void *ctx);
    // The node id this port's device answers to, when the bridge learns or is
    // told one: the port-level SET_PORT_NODEID, which names the device behind
    // the port, and an assignment that turned out to be for a device on it. A
    // transport that has to act on the id takes it here - a CAN port arms its
    // hardware receive filter, since the ids of the devices behind it are the
    // ids it must hear. Optional: NULL for a port whose transport needs nothing
    // from the id (a USART port writes to its own wire and reads all of it).
    // Called for every id the port is given, repeats included; a port that
    // cannot take one must report it (shutdown()), never ignore it - a port that
    // cannot hear the device it was told about is a dead port.
    void (*node_id)(void *ctx, uint8_t node_id);
    void *ctx;
};

// Register a downstream port, with no flags set: a legacy port, whose device
// speaks raw frames - a unit is stripped to its frame on the way down, and a
// raw frame is given the reply header of the port's id on the way up. What a
// port's device actually speaks is the host's call, not a build time one: it
// sets MUX_PORT_UNITS per port with the port-level SET_PORT_NODEID once it
// knows what is behind the port. 'ctx' is the pointer the port's ops receive
// (the object whose stream the port carries). Returns a non-negative port
// index, or -1 when there is no room.
int bridge_add_port(const struct bridge_port_ops *ops, void *ctx);

// Feed bytes read from downstream port 'port' into the router. Whole units and
// frames are forwarded upstream as they arrive; a partial one is buffered until
// the rest of it shows up.
void bridge_port_input(int port, const uint8_t *data, size_t len);

// The two points where the upstream loop consults the router. Neither exists on
// a build without CONFIG_BRIDGE. bridge_admin_unit() answers whether the unit
// was consumed: 1 for a port-level command the bridge handles on its own (and
// which therefore never reaches the downstream ports or the local device
// half), 0 for everything the bridge only remembers from and fans out - which
// is what the caller's own admin handler still has to look at.
int bridge_admin_unit(uint32_t arb, uint8_t *frame, uint_fast8_t frame_len);
void bridge_route_unit(uint32_t arb, uint8_t *frame, uint_fast8_t frame_len);

#endif // __BRIDGE_H
