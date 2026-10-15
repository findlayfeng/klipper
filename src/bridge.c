// The bridge: routing units between the console's ports (CONFIG_BRIDGE)
//
// A bridge gives the console more than one port and relays addressed units
// between them. Its own klipper instance is the "self" port (the device half in
// mux.c, which answers the admin channel and owns a node id of its own); the
// other ports are downstream paths the arch provides (host/bridge.c is the
// serial one). This file is the transport-neutral router: the port table, the
// two routes an id can have, and the fan-out. The seam to a port is
// struct bridge_port_ops in bridge.h; the seam to the upstream byte stream is
// the pair of hooks mux.c calls from mux_input().
//
// Two tables do the routing:
//
//   uuid -> port   learned from the NEED_NODEID a downstream port answers a
//                  scan with. The uuid is located by the frame's length byte,
//                  so a uuid that contains 0x7e is ordinary content.
//   id   -> port   learned from an upstream SET_KLIPPER_NODEID, which the
//                  bridge forwards downstream and remembers by looking the
//                  uuid up, and from the port-level SET_PORT_NODEID, which
//                  names the port outright - the route a device with no admin
//                  channel of its own gets. One port may hold many ids.
//
// The assignment has no acknowledgement in the protocol, so nothing here waits
// for one and nothing here invents one. If an assignment names a uuid whose
// port is not known yet, the bridge remembers it and probes once with a
// QUERY_UNASSIGNED; the answer completes both tables.
//
// A downstream unit carrying the admin reply id is forwarded upstream exactly
// as it arrived, one at a time: the host deduplicates scan answers, so the
// bridge must not merge or drop them. A data unit going down is stripped of its
// header when the path has one device on it (the exclusive wire format that
// path speaks); a raw frame coming up from such a path is given the reply
// header of the id that path was assigned. A headed unit coming up is forwarded
// unchanged: the port it arrived on is what a unit is attributed to, never a
// guess from the id in its header.
//
// Whether a port's device speaks units is the host's to say, in the flags of
// the port-level SET_PORT_NODEID (mux.h); a port that has not been told - or
// whose device was told it does not - is legacy, and the strip/head rule below
// is what keeps the wire format of a device that predates the mux working.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcmp, memcpy, memmove
#include "autoconf.h" // CONFIG_BRIDGE, CONFIG_MACH_HOST
#include "bridge.h" // struct bridge_port_ops
#include "command.h" // MESSAGE_MIN, MESSAGE_MAX
#include "mux.h" // the mux unit format
#if CONFIG_MACH_HOST
#include "host/klipper_host.h" // KLIPPER_HOST_PORT_SLOTS
#endif

// Downstream ports are the UART objects the host creates at configuration
// time (uart.c hands each one here); the table is a small fixed array
// because nothing adds and removes ports at run time.
#define BRIDGE_MAX_PORTS 4

// On the host library the host names a tty per bridge port before klipper comes
// up (klipper_host_port, host/host.c), and its table has one slot per port
// (KLIPPER_HOST_PORT_SLOTS, host/klipper_host.h): a bridge with more ports than
// that would leave the extra ports with no tty the host could name, and they
// would open the path the library was built with instead. Only the host library
// has the constraint - a micro-controller's bridge never sees the host's table -
// so the header is read only in that build, and the two counts are otherwise
// unrelated. Keep them from drifting apart silently.
#if CONFIG_MACH_HOST
#if BRIDGE_MAX_PORTS > KLIPPER_HOST_PORT_SLOTS
#error "BRIDGE_MAX_PORTS exceeds KLIPPER_HOST_PORT_SLOTS: not every bridge port could be named a tty"
#endif
#endif

// One uuid per device that has answered a scan; a handful is plenty.
#define BRIDGE_UUID_SLOTS 16
// Assignments made before the uuid's port is known, waiting for the probe.
#define BRIDGE_PENDING_SLOTS 8

// One port, and the bytes read from it that do not form a whole unit or frame
// yet. The buffer holds the largest unit (header plus the largest frame) with
// room to spare, so a partial one always fits.
struct bridge_port {
    struct bridge_port_ops ops;
    // The port's MUX_PORT_* flags (mux.h): MUX_PORT_UNITS set when the device
    // behind it speaks units and the port forwards them unchanged, clear for a
    // legacy device (the port strips the header down and heads raw frames up).
    // Cleared at registration; the host sets them per port with the port-level
    // SET_PORT_NODEID.
    uint8_t flags;
    // The node id of the device on a legacy port: a raw frame it sends is
    // forwarded upstream under this id's reply header. 0 until assigned.
    uint32_t node_id;
    int in_use;
    uint8_t in[2 * (MUX_HEADER_BYTES + MESSAGE_MAX)];
    size_t in_len;
};

static struct bridge_port bridge_ports[BRIDGE_MAX_PORTS];

// The node id's port, one past the index so 0 means "no port owns it".
static uint8_t bridge_id_port[MUX_MAX_NODE_ID + 1];

// The uuid's port, learned from NEED_NODEID answers.
struct bridge_uuid_entry {
    uint8_t uuid[MUX_UUID_LEN];
    uint8_t port;
};
static struct bridge_uuid_entry bridge_uuids[BRIDGE_UUID_SLOTS];
static int bridge_uuid_count;

// An assignment whose uuid had not been seen yet.
struct bridge_pending_entry {
    uint8_t uuid[MUX_UUID_LEN];
    uint32_t nodeid;
};
static struct bridge_pending_entry bridge_pending[BRIDGE_PENDING_SLOTS];
static int bridge_pending_count;


/****************************************************************
 * The two tables
 ****************************************************************/

static int
bridge_uuid_port(const uint8_t *uuid)
{
    int i;
    for (i = 0; i < bridge_uuid_count; i++)
        if (!memcmp(bridge_uuids[i].uuid, uuid, MUX_UUID_LEN))
            return bridge_uuids[i].port;
    return -1;
}

static void
bridge_set_uuid_port(const uint8_t *uuid, int port)
{
    int i;
    for (i = 0; i < bridge_uuid_count; i++) {
        if (!memcmp(bridge_uuids[i].uuid, uuid, MUX_UUID_LEN)) {
            bridge_uuids[i].port = port;
            return;
        }
    }
    if (bridge_uuid_count >= BRIDGE_UUID_SLOTS)
        return;
    struct bridge_uuid_entry *e = &bridge_uuids[bridge_uuid_count++];
    memcpy(e->uuid, uuid, MUX_UUID_LEN);
    e->port = port;
}

static void
bridge_map_id(uint32_t nodeid, int port)
{
    if (nodeid < 1 || nodeid > MUX_MAX_NODE_ID)
        return;
    bridge_id_port[nodeid] = port + 1;
    if (!(bridge_ports[port].flags & MUX_PORT_UNITS))
        // A legacy port speaks for the one device behind it, so the id is also
        // what its raw frames are headed with on the way up.
        bridge_ports[port].node_id = nodeid;
}

static void
bridge_remember_pending(const uint8_t *uuid, uint32_t nodeid)
{
    int i;
    for (i = 0; i < bridge_pending_count; i++) {
        if (!memcmp(bridge_pending[i].uuid, uuid, MUX_UUID_LEN)) {
            bridge_pending[i].nodeid = nodeid;
            return;
        }
    }
    if (bridge_pending_count >= BRIDGE_PENDING_SLOTS)
        return;
    struct bridge_pending_entry *e = &bridge_pending[bridge_pending_count++];
    memcpy(e->uuid, uuid, MUX_UUID_LEN);
    e->nodeid = nodeid;
}

// A port just answered for `uuid`: complete any assignment that was waiting on
// it and forget it.
static void
bridge_apply_pending(const uint8_t *uuid, int port)
{
    int i = 0;
    while (i < bridge_pending_count) {
        if (memcmp(bridge_pending[i].uuid, uuid, MUX_UUID_LEN)) {
            i++;
            continue;
        }
        bridge_map_id(bridge_pending[i].nodeid, port);
        bridge_pending[i] = bridge_pending[--bridge_pending_count];
    }
}

// A port-level SET_PORT_NODEID: the host telling the bridge which device sits
// behind which port, and whether that device speaks units. This is the route a
// device gets without participating at all - it is answered by the tables
// alone: no receipt, no copy downstream, and a repeat simply writes the same
// values again. Anything that names no port the bridge has, or an id the wire
// cannot address, is dropped in silence.
static void
bridge_set_port_nodeid(const uint8_t *args, uint_fast8_t args_len)
{
    if (args_len < MUX_SET_PORT_NODEID_LEN)
        return;
    uint_fast8_t index = args[1];
    uint32_t nodeid = args[2];
    uint8_t flags = args[3];
    if (index >= BRIDGE_MAX_PORTS || !bridge_ports[index].in_use)
        return;
    if (!mux_arb_id(nodeid))
        return;
    bridge_ports[index].flags = flags & MUX_PORT_UNITS;
    bridge_map_id(nodeid, index);
}


/****************************************************************
 * Writing to a port
 ****************************************************************/

// Write a whole unit (header naming `arb`, then `frame`) to one port in a
// single call, so the port's blob is never interleaved with a later one.
static void
bridge_write_unit(struct bridge_port *p, uint32_t arb, const uint8_t *frame
                  , uint_fast8_t frame_len)
{
    uint8_t unit[MUX_HEADER_BYTES + MESSAGE_MAX];
    uint_fast8_t header = mux_encode_header(unit, arb);
    if (!header)
        return;
    memcpy(&unit[header], frame, frame_len);
    p->ops.write(p->ops.ctx, unit, header + frame_len);
}

// Forward one whole `KXID` unit to a single port: headed for a port whose
// device speaks units, stripped to its bare frame for a legacy one. The arb is
// the one the unit arrived on (the admin id), so the device behind a UNITS
// port sees the command on the channel it listens on.
static void
bridge_forward_to_port(int index, uint32_t arb, const uint8_t *frame
                       , uint_fast8_t frame_len)
{
    struct bridge_port *p = &bridge_ports[index];
    if (!(p->flags & MUX_PORT_UNITS)) {
        // A legacy device predates the mux: its frame goes out without the
        // header, and its parser will reject the pseudo-frame's 0x20 sequence
        // byte - which is exactly right. A device with no admin channel
        // executes neither of the device commands (REQUEST_BOOTLOADER,
        // RESTART_DEVICE), so this can only ever take effect on a port whose
        // device speaks units; a port with no such device is reset at the line
        // instead (RESET_PORT, mux.h MUX_CMD_RESET_PORT).
        p->ops.write(p->ops.ctx, frame, frame_len);
        return;
    }
    bridge_write_unit(p, arb, frame, frame_len);
}

// Ask every downstream port to identify itself, so a uuid a pending assignment
// names can be located.
static void
bridge_probe_unassigned(void)
{
    uint8_t unit[MUX_HEADER_BYTES + MESSAGE_MAX];
    uint_fast8_t len = mux_query_unassigned(unit, sizeof(unit));
    if (!len)
        return;
    int i;
    for (i = 0; i < BRIDGE_MAX_PORTS; i++)
        if (bridge_ports[i].in_use)
            bridge_ports[i].ops.write(bridge_ports[i].ops.ctx, unit, len);
}

// An upstream SET_KLIPPER_NODEID: remember which port the named uuid's device
// is on, probing for it if a scan has not named it yet. The assignment's own
// acknowledgement is the host's business (the protocol has none on this wire).
static void
bridge_learn_assignment(const uint8_t *uuid, uint32_t nodeid)
{
    int port = bridge_uuid_port(uuid);
    if (port >= 0) {
        bridge_map_id(nodeid, port);
        return;
    }
    bridge_remember_pending(uuid, nodeid);
    bridge_probe_unassigned();
}


/****************************************************************
 * Reading from a port
 ****************************************************************/

// Drop `n` bytes off the front of a port's input.
static void
bridge_port_drop(struct bridge_port *p, size_t n)
{
    memmove(p->in, &p->in[n], p->in_len - n);
    p->in_len -= n;
}

// A raw frame came up from a legacy path: head it with the reply id of the id
// that path was assigned. Without one there is no address to send it under, so
// it is dropped.
static void
bridge_port_raw_up(struct bridge_port *p, uint_fast8_t frame_len)
{
    if ((p->flags & MUX_PORT_UNITS) || !p->node_id)
        return;
    uint8_t unit[MUX_HEADER_BYTES + MESSAGE_MAX];
    uint_fast8_t header = mux_encode_header(unit, mux_reply_arb_id(p->node_id));
    if (!header)
        return;
    memcpy(&unit[header], p->in, frame_len);
    mux_send(unit, header + frame_len);
}

// An admin reply unit from a downstream port: a NEED_NODEID names the uuid that
// answered, which is what the uuid->port table is built from.
static void
bridge_port_learn_uuid(struct bridge_port *p, int index, size_t unit_len)
{
    struct mux_kxid kxid;
    if (!mux_parse_kxid(&p->in[MUX_HEADER_BYTES], unit_len - MUX_HEADER_BYTES
                        , &kxid))
        return;
    if (kxid.args_len < 1 + MUX_UUID_LEN
        || kxid.args[0] != MUX_RESP_NEED_NODEID)
        return;
    bridge_set_uuid_port(&kxid.args[1], index);
    bridge_apply_pending(&kxid.args[1], index);
}

// One headed unit arrived whole. Report it upward one at a time and unchanged:
// the host does the matching and deduplication, so the bridge merges nothing.
static void
bridge_port_unit_up(struct bridge_port *p, int index, uint32_t arb
                    , size_t unit_len)
{
    if (arb == MUX_ADMIN_REPLY)
        bridge_port_learn_uuid(p, index, unit_len);
    mux_send(p->in, unit_len);
}

// Split whatever a port has buffered into whole units and raw frames, in
// arrival order, and act on each. Anything that cannot begin a unit (a framing
// bit that is not a header, a length no frame has) is dropped a byte at a time,
// which is the resynchronisation the frame parser uses too.
static void
bridge_port_scan(int index)
{
    struct bridge_port *p = &bridge_ports[index];
    for (;;) {
        if (!p->in_len)
            return;
        if (p->in[0] & 0x80) {
            uint32_t arb;
            uint_fast8_t consumed;
            enum mux_header_scan scan = mux_decode_header(p->in, p->in_len
                                                          , &arb, &consumed);
            if (scan == MUX_HEADER_NEED_MORE)
                return;
            if (scan == MUX_HEADER_INVALID) {
                bridge_port_drop(p, 1);
                continue;
            }
            if (p->in_len < (size_t)consumed + 1)
                return;                // the length byte is still coming
            uint_fast8_t msglen = p->in[consumed];
            if (msglen < MESSAGE_MIN || msglen > MESSAGE_MAX) {
                bridge_port_drop(p, consumed);
                continue;
            }
            size_t unit_len = (size_t)consumed + msglen;
            if (p->in_len < unit_len)
                return;                // the frame is still arriving
            bridge_port_unit_up(p, index, arb, unit_len);
            bridge_port_drop(p, unit_len);
            continue;
        }
        // No framing bit: a raw frame, the shape a path with one device on it
        // carries.
        uint_fast8_t msglen = p->in[0];
        if (msglen < MESSAGE_MIN || msglen > MESSAGE_MAX) {
            bridge_port_drop(p, 1);
            continue;
        }
        if (p->in_len < msglen)
            return;
        bridge_port_raw_up(p, msglen);
        bridge_port_drop(p, msglen);
    }
}


/****************************************************************
 * The router's public surface
 ****************************************************************/

int
bridge_add_port(const struct bridge_port_ops *ops, void *ctx)
{
    int i;
    for (i = 0; i < BRIDGE_MAX_PORTS; i++) {
        if (bridge_ports[i].in_use)
            continue;
        bridge_ports[i].ops = *ops;
        bridge_ports[i].ops.ctx = ctx;
        // No flags: the port starts legacy, and the host names what is behind
        // it with the port-level SET_PORT_NODEID.
        bridge_ports[i].flags = 0;
        bridge_ports[i].in_use = 1;
        return i;
    }
    return -1;
}

void
bridge_port_input(int port, const uint8_t *data, size_t len)
{
    if (port < 0 || port >= BRIDGE_MAX_PORTS || !bridge_ports[port].in_use)
        return;
    struct bridge_port *p = &bridge_ports[port];
    while (len) {
        size_t room = sizeof(p->in) - p->in_len;
        if (!room) {
            // Unreachable while every partial form fits, but a desynchronised
            // stream must not spin: start the buffer over.
            p->in_len = 0;
            room = sizeof(p->in);
        }
        size_t take = len < room ? len : room;
        memcpy(&p->in[p->in_len], data, take);
        p->in_len += take;
        data += take;
        len -= take;
        bridge_port_scan(port);
    }
}

int
bridge_admin_unit(uint32_t arb, uint8_t *frame, uint_fast8_t frame_len)
{
    // An assignment is also the moment to learn which port owns the id; a
    // unit for the local device itself is not a downstream port's business.
    struct mux_kxid kxid;
    if (mux_parse_kxid(frame, frame_len, &kxid) && kxid.args_len) {
        if (kxid.args[0] == MUX_CMD_SET_PORT_NODEID) {
            // A port-level command: the bridge's own business. It is answered
            // by the tables alone and stays on this side of the wire - it
            // never reaches a downstream port or the local device half.
            bridge_set_port_nodeid(kxid.args, kxid.args_len);
            return 1;
        }
        if (kxid.args[0] == MUX_CMD_REQUEST_BOOTLOADER
            || kxid.args[0] == MUX_CMD_RESTART_DEVICE) {
            // A device command aimed at one device, behind the port the host
            // names ([0x02][index] or [0x05][index]): forward it to that port
            // alone - no fan-out, no receipt. The two share this arm because
            // the bridge's part is the same routing either way; what the device
            // does with it - wait in its bootloader, or restart its firmware -
            // is the device's business, on its own side of the wire.
            //
            // Consumed **only when that port exists and was forwarded to**. An
            // index naming no port of ours means the sender is not addressing a
            // device behind one of them: a device that is itself a mux build
            // (BRIDGE is `default y if MUX_SERIAL`, so this is the common case)
            // has no ports at all, and the host that reached it through some
            // *other* bridge means *this* device. Falling through lets
            // `mux_admin_unit` apply the command to this device; swallowing it
            // here is what made a directly addressed board never restart.
            if (kxid.args_len < 2)
                return 0;
            uint_fast8_t index = kxid.args[1];
            if (index >= BRIDGE_MAX_PORTS || !bridge_ports[index].in_use)
                return 0;
            bridge_forward_to_port(index, arb, frame, frame_len);
            return 1;
        }
        if (kxid.args[0] == MUX_CMD_RESET_PORT) {
            // A reset aimed at a port's line, not at a device on it
            // ([0x04][index]): the bridge owns every port's electrical side,
            // so this is the bridge's alone - consumed, never fanned out,
            // never handed to the self device half. A port with no line-level
            // reset (a NULL reset hook), or an index naming no port, is dropped
            // in silence - the command carries no receipt.
            if (kxid.args_len >= 2) {
                uint_fast8_t index = kxid.args[1];
                if (index < BRIDGE_MAX_PORTS && bridge_ports[index].in_use) {
                    struct bridge_port *p = &bridge_ports[index];
                    if (p->ops.reset)
                        p->ops.reset(p->ops.ctx);
                }
            }
            return 1;
        }
        if (kxid.args[0] == MUX_CMD_SET_KLIPPER_NODEID
            && kxid.args_len >= 1 + MUX_UUID_LEN + 1) {
            const uint8_t *uuid = &kxid.args[1];
            uint32_t nodeid = kxid.args[1 + MUX_UUID_LEN];
            if (nodeid && memcmp(uuid, mux_uuid(), MUX_UUID_LEN))
                bridge_learn_assignment(uuid, nodeid);
        }
    }
    // Every admin broadcast reaches every downstream port, header and all.
    int i;
    for (i = 0; i < BRIDGE_MAX_PORTS; i++)
        if (bridge_ports[i].in_use)
            bridge_write_unit(&bridge_ports[i], arb, frame, frame_len);
    return 0;
}

void
bridge_route_unit(uint32_t arb, uint8_t *frame, uint_fast8_t frame_len)
{
    uint32_t nodeid = mux_node_id_from_arb(arb);
    if (!nodeid)
        return;
    uint8_t slot = bridge_id_port[nodeid];
    if (!slot)
        return;                        // no port owns this id: nothing to route
    struct bridge_port *p = &bridge_ports[slot - 1];
    if (!(p->flags & MUX_PORT_UNITS)) {
        // A legacy port has the one device that predates the mux behind it: its
        // frame goes out without the header that wire format has no use for.
        p->ops.write(p->ops.ctx, frame, frame_len);
        return;
    }
    bridge_write_unit(p, arb, frame, frame_len);
}
