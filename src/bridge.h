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
    void *ctx;
};

// Register a downstream port. 'single_device' says the path carries exactly one
// device: a unit is then stripped to its frame on the way down, and a raw frame
// is given the reply header of the port's id on the way up. Returns a
// non-negative port index, or -1 when there is no room.
int bridge_add_port(const struct bridge_port_ops *ops, int single_device);

// Feed bytes read from downstream port 'port' into the router. Whole units and
// frames are forwarded upstream as they arrive; a partial one is buffered until
// the rest of it shows up.
void bridge_port_input(int port, const uint8_t *data, size_t len);

// The two points where the upstream loop consults the router. Neither exists on
// a build without CONFIG_BRIDGE.
void bridge_admin_unit(uint32_t arb, uint8_t *frame, uint_fast8_t frame_len);
void bridge_route_unit(uint32_t arb, uint8_t *frame, uint_fast8_t frame_len);

#endif // __BRIDGE_H
