// A bridge build's CAN downstream port: one CAN controller handed to the bridge
//
// A bridge build has ports beside its console, and not all of them are USARTs
// (stm32/bridge_serial.c): a CAN controller wired to a bus of downstream devices
// is one port too, and this file is that backend. It answers the dispatcher's
// seam (stm32/bridge_port.c, uart.h): bridge_can_open() takes the one controller
// this build offers, bridge_can_port_ops() gives the router the hook that writes
// a blob to the bus, bridge_can_set_port() records the router's port index, and
// bridge_can_owns() tells the dispatcher which objects are this backend's - the
// CAN slots come after every UART slot.
//
// The wire is CAN's, not a byte stream's. A blob the router hands over - a whole
// mux unit, a legacy device's raw frame, or a tunnel's katapult block - is queued
// whole and carried in frames of at most 8 bytes, so it is never cut short and a
// later blob never cuts in front of it. The frames heard back are reassembled
// per node id into whole units before the router sees them, because several
// devices share this one port and their streams interleave frame by frame on it.
//
// Which address a blob goes out on, in the order the blob itself says:
//
//   - It starts with a unit header (the port's device speaks units, so bridge.c
//     writes header + frame): the header's own arbitration id, which is the
//     address the host put in it.
//   - It has no header (the router stripped it: a legacy port's device speaks
//     raw frames, or the port is in flashing tunnel mode and the bytes are a
//     bootloader's katapult stream): mux_arb_id() of the node id the host named
//     the port with (the port-level SET_PORT_NODEID), which is the address a
//     device behind this port listens on.
//   - Neither - a header that does not decode and no id named yet: the blob is
//     dropped and counted (tx_unaddressed). There is no address for it and
//     nothing here may block or shut the build down over one unroutable blob.
//
// The rate is not a config_uart baud: a CAN bus runs at one rate for every device
// on it, CONFIG_CANBUS_FREQUENCY, which canbus.c reports to the host
// (DECL_CONSTANT "CANBUS_FREQUENCY"). config_uart's baud is therefore a dummy
// parameter in this backend and is ignored on purpose.
//
// Listening: the devices behind this port answer on ids of their own, which the
// bridge learns only when the host names the port - SET_PORT_NODEID, which reaches
// the ops' node_id hook (bridge.h). That hook arms the hardware receive filter for
// one pair at a time (0x100+2n and its +1, in one bank through a bit-0 mask:
// CONFIG_CANBUS_FILTER_MULTI, generic/canbus.h canhw_add_filter). A build without
// that capability, or with no filter bank left, says so with shutdown() - a port
// that cannot hear the device it was told about is worse than one that refuses the
// configuration.
//
// The admin channel is the one id nobody has to be named on: an unassigned device
// answers on MUX_ADMIN_REPLY (0x3f1), the same id for every one of them, so the port
// arms that filter when it opens - a bridge that cannot hear the answer can never
// be told which device is on its bus, and could never assign it an id.
//
// That channel is not this port's unit framing either. A CAN device's admin message
// is one short frame whose first byte is the command itself (generic/canserial.h
// CANBUS_ID_ADMIN / CANBUS_ID_ADMIN_RESP), so the assignment handshake is
// translated in both directions here: a host's KXID unit on 0x3f0 goes out as the
// frames the channel spells (bridge_can_admin_write), and an answer heard on 0x3f1
// is rebuilt into the unit the router recognises it by (bridge_can_admin_answer).
// An admin broadcast this wire has no frame for is dropped and counted rather than
// sliced onto the bus as if it were a stream. Every other blob keeps the framing
// described above.
//
// Receive is push, like the USART backend's: the CAN interrupt appends a node's
// frame payload to that node's own buffer and marks how much of it is whole
// units, and a task hands those on to bridge_port_input(). The router's work must
// not nest in an interrupt, and its input buffer is one per port - so per-node
// reassembly is also what keeps two devices' streams apart.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // NULL
#include <string.h> // memcpy, memmove
#include "autoconf.h" // CONFIG_CANBUS_FILTER_MULTI
#include "board/irq.h" // irq_save, irq_restore
#include "bridge.h" // bridge_port_input, struct bridge_port_ops
#include "bridge_can.h" // bridge_can_open, bridge_can_process_data
#include "canbus.h" // canhw_add_filter, canbus_send, struct canbus_msg
#include "command.h" // MESSAGE_MIN, MESSAGE_MAX, shutdown
#include "mux.h" // mux_decode_header, mux_arb_id, MUX_TUNNEL_*
#include "sched.h" // DECL_TASK, sched_wake_tasks
#include "uart.h" // struct uart_s, enum uart_open_result

// The CAN slots this build offers, named for the host's `admin bridge config`: a
// bridge's CAN ports come after every USART slot, and the host reserves a port's
// pins for it by this name, the way it reserves a USART backend's
// (stm32/bridge_serial.c, RESERVE_PINS_bridge_serial*). The pins are the ones the
// CAN slot is wired to (CONFIG_STM32_BRIDGE_CAN_PB8_PB9: PB8 RX, PB9 TX).
DECL_CONSTANT_STR("RESERVE_PINS_bridge_can", "PB8,PB9");

// The katapult bootloader's node-id command: the same [uuid][nodeid] body the
// application's MUX_CMD_SET_KLIPPER_NODEID carries, under the command byte
// katapult's own admin message spells it with. The two firmwares disagree on that
// one byte and ignore each other's command, so an assignment goes out as both
// (bridge_can_admin_write) and whichever firmware is on the bus takes it.
#define BRIDGE_CAN_CMD_SET_BOOTLOADER_NODEID 0x11

// The most blobs this port keeps queued for the wire, in frames: a unit is at
// most MUX_HEADER_BYTES + MESSAGE_MAX bytes (9 frames), a tunnel block at most
// MUX_TUNNEL_FRAME_MAX (16 frames), so this holds several of either and the
// router never has to wait for the bus.
#define BRIDGE_CAN_TX_FRAMES 32

// How many admin answers this port holds for its task at once: one bus carries one
// unassigned device in practice, and the few that a bus of them would answer with
// still fit. Past that an answer is dropped and counted, like every other
// overrun here.
#define BRIDGE_CAN_ADMIN_FRAMES 4

// A reassembly buffer per downstream node id: two maximum-size tunnel blocks,
// the largest partial form this port can be handed (bridge.c's per-port buffer is
// the same size for the same reason). One buffer per node, so two devices'
// frames never splice into one unit.
#define BRIDGE_CAN_NODE_BUF (2 * MUX_TUNNEL_FRAME_MAX)

// How many downstream ids this port reassembles at once. A host names a handful
// of devices behind one bus; past this, a node's frames are dropped and counted
// (there is no room to reassemble them, and dropping is the only honest answer).
#ifndef BRIDGE_CAN_NODE_MAX
#define BRIDGE_CAN_NODE_MAX 4
#endif

// One frame waiting for the wire: the address it goes out on and its bytes (a
// blob's tail slice is shorter than 8). Kept as frames rather than as a byte
// queue so that a blob which arrives while another is queued goes out afterwards
// whole, even when the two are addressed differently.
struct bridge_can_frame {
    uint32_t arb;
    uint8_t len;
    uint8_t data[8];
};

// One downstream node's reassembly: the bytes of its stream, and how much of the
// front is whole pieces the task may hand to the router. Written by the CAN
// interrupt, drained by the task (both under irq_save).
struct bridge_can_node {
    uint8_t node_id;      // 0: the entry is free
    uint16_t len;         // bytes in buf
    uint16_t ready;       // bytes at the front that are whole pieces
    uint8_t buf[BRIDGE_CAN_NODE_BUF];
};

static struct bridge_can_node bridge_can_nodes[BRIDGE_CAN_NODE_MAX];

// The bytes a task is handing to the router, one node's worth at a time. The
// router's entry (bridge_port_input) is not a copy: it scans the piece into the
// port's buffer, routes it, and may write another port - work that must not run
// inside an interrupts-off section. So the task copies the whole pieces out of
// the node's buffer under irq_save, hands the node's space back to the
// interrupt, and only then calls the router with the copy. Written by the task
// alone; the interrupt never touches it.
static uint8_t bridge_can_handover[BRIDGE_CAN_NODE_BUF];

// This build's one CAN controller as a port: the object that opened it, the
// router's index for it, the id the host named the port with, and the queue.
static struct {
    struct uart_s *owner;
    // The router's port index, recorded one past itself, so 0 means "not handed
    // to the bridge yet" (the encoding bridge_serial.c uses too).
    int port;
    // The node id the host named this port with (SET_PORT_NODEID), 0 until then.
    // It is the address a blob with no unit header is sent under.
    uint8_t node_id;

    // The queue: head is the frame the wire takes next, tail where the next blob
    // appends. A frame is taken off only once the mailboxes accepted it, so a
    // blob that does not fit in them stays queued in order.
    struct bridge_can_frame tx_frames[BRIDGE_CAN_TX_FRAMES];
    volatile uint8_t tx_head, tx_tail;
    // Set while a frame is being handed to the driver, so a nested call (the
    // driver enables its transmit interrupt from inside canbus_send, which is
    // called from an interrupt here) cannot send the same frame twice.
    volatile uint8_t tx_busy;

    // Frames and blobs this port could not take, kept as the diagnostic a
    // micro-controller has (no stderr, no host-visible channel in this backend).
    uint32_t tx_unaddressed;  // no header and no id named: nowhere to send it
    uint32_t tx_unspelled;    // an admin broadcast this wire has no frame for
    uint32_t tx_overrun;      // the queue was full
    uint32_t rx_ignored;      // another id's traffic, not this port's
    uint32_t rx_overrun;      // a node's buffer was full
} bridge_can;

// The admin answers waiting for the task: the frames heard on MUX_ADMIN_REPLY,
// copied here by the interrupt. An answer is one frame, so nothing has to be
// reassembled - but the unit built around it has to reach the router from a task
// (bridge.c), so this is a handover buffer and not a stream. Written by the
// interrupt under irq_save, drained by the task the same way.
static struct {
    volatile uint8_t head, tail;
    uint8_t len[BRIDGE_CAN_ADMIN_FRAMES];
    uint8_t data[BRIDGE_CAN_ADMIN_FRAMES][8];
} bridge_can_admin;


/****************************************************************
 * Sending: a queued blob, in frames of at most 8 bytes
 ****************************************************************/

// Hand the queued frames to the driver, stopping at the first one the mailboxes
// cannot take - the driver has armed its transmit interrupt then, and its
// canbus_notify_tx() comes back here (canbus.c). Called from a task (a blob was
// just queued) and from that interrupt.
static void
bridge_can_tx_drain(void)
{
    for (;;) {
        irqstatus_t flag = irq_save();
        uint8_t head = bridge_can.tx_head;
        if (bridge_can.tx_busy || head == bridge_can.tx_tail) {
            irq_restore(flag);
            return;
        }
        const struct bridge_can_frame *f = &bridge_can.tx_frames[head];
        struct canbus_msg msg;
        // The driver writes both data words whole, so the bytes past 'len' are
        // cleared rather than left over from an earlier frame.
        msg.id = f->arb;
        msg.dlc = f->len;
        msg.data32[0] = msg.data32[1] = 0;
        memcpy(msg.data, f->data, f->len);
        bridge_can.tx_busy = 1;
        irq_restore(flag);

        // Nothing is held across this call: the driver may enable interrupts to
        // be told when a mailbox frees up, and that interrupt re-enters here.
        int ret = canbus_send(&msg);

        flag = irq_save();
        bridge_can.tx_busy = 0;
        if (ret <= 0) {
            // No mailbox: the frame stays at the head of the queue, and the
            // driver's transmit interrupt retries it.
            irq_restore(flag);
            return;
        }
        bridge_can.tx_head = (head + 1) % BRIDGE_CAN_TX_FRAMES;
        irq_restore(flag);
    }
}

// Queue one whole blob on 'arb' and start the wire. The blob is taken whole or
// not at all: a queue with no room for all of its frames refuses it (-1) rather
// than sending a prefix, and the frames already queued go out first, so blobs
// never interleave.
static int
bridge_can_tx_blob(uint32_t arb, const uint8_t *data, size_t len)
{
    size_t frames = (len + 7) / 8;
    if (!len)
        return 0;
    irqstatus_t flag = irq_save();
    uint8_t head = bridge_can.tx_head, tail = bridge_can.tx_tail;
    size_t used = (tail + BRIDGE_CAN_TX_FRAMES - head)
        % BRIDGE_CAN_TX_FRAMES;
    if (used + frames > BRIDGE_CAN_TX_FRAMES) {
        bridge_can.tx_overrun++;
        irq_restore(flag);
        return -1;
    }
    size_t i;
    for (i = 0; i < frames; i++) {
        size_t off = i * 8;
        size_t n = len - off < 8 ? len - off : 8;
        struct bridge_can_frame *f = &bridge_can.tx_frames[tail];
        f->arb = arb;
        f->len = n;
        memcpy(f->data, &data[off], n);
        tail = (tail + 1) % BRIDGE_CAN_TX_FRAMES;
    }
    bridge_can.tx_tail = tail;
    irq_restore(flag);
    bridge_can_tx_drain();
    return 0;
}

// The transmitter has room again (canbus.c, from the driver's interrupt).
void
bridge_can_notify_tx(void)
{
    bridge_can_tx_drain();
}


/****************************************************************
 * Receiving: one stream per node id, whole units to the router
 ****************************************************************/

// The length of the whole piece at the front of a node's buffer: a mux unit
// (header + frame), a raw frame (what a legacy device sends), or a whole katapult
// block (what a tunnelled bootloader sends). 0 while the piece is still arriving
// and 1 for a byte that cannot begin any of them - the caller drops that byte and
// resynchronises, which is bridge.c's bridge_port_scan policy applied one layer
// down, where the node boundary is known.
static size_t
bridge_can_piece_len(const uint8_t *buf, size_t len)
{
    if (!len)
        return 0;
    if (buf[0] & 0x80) {
        // A unit header. Anything that decodes into no address is junk.
        uint32_t arb;
        uint_fast8_t consumed;
        enum mux_header_scan scan = mux_decode_header(buf, len, &arb, &consumed);
        if (scan == MUX_HEADER_NEED_MORE)
            return 0;
        if (scan == MUX_HEADER_INVALID)
            return 1;
        if (len < (size_t)consumed + 1)
            return 0;                  // the frame's length byte is still coming
        uint_fast8_t msglen = buf[consumed];
        if (msglen < MESSAGE_MIN || msglen > MESSAGE_MAX)
            return 1;
        size_t unit_len = (size_t)consumed + msglen;
        return len < unit_len ? 0 : unit_len;
    }
    if (buf[0] == MUX_TUNNEL_TAG0) {
        // A katapult block: its tag pair and word count are the framing.
        if (len < 2)
            return 0;
        if (buf[1] != MUX_TUNNEL_TAG1)
            return 1;
        if (len <= MUX_TUNNEL_POS_WORDCNT)
            return 0;                  // the word count is still coming
        size_t frame_len = mux_tunnel_frame_len(buf[MUX_TUNNEL_POS_WORDCNT]);
        if (!frame_len)
            return 1;                  // a word count past the limit
        return len < frame_len ? 0 : frame_len;
    }
    // No framing bit: a raw frame, whose first byte is its own length.
    uint_fast8_t msglen = buf[0];
    if (msglen < MESSAGE_MIN || msglen > MESSAGE_MAX)
        return 1;
    return len < msglen ? 0 : msglen;
}

// Judge as much of a node's buffer as has arrived: advance 'ready' over the whole
// pieces at the front, dropping the bytes in between that begin nothing.
static void
bridge_can_judge(struct bridge_can_node *n)
{
    while (n->ready < n->len) {
        size_t piece = bridge_can_piece_len(&n->buf[n->ready]
                                            , n->len - n->ready);
        if (!piece)
            return;
        n->ready += piece;
    }
}

// The node entry for 'node_id', made if there is room. NULL when the table is
// full - a bus carries more ids than one bridge port is asked to reassemble.
static struct bridge_can_node *
bridge_can_node_get(uint32_t node_id)
{
    int i;
    for (i = 0; i < BRIDGE_CAN_NODE_MAX; i++)
        if (bridge_can_nodes[i].node_id == node_id)
            return &bridge_can_nodes[i];
    for (i = 0; i < BRIDGE_CAN_NODE_MAX; i++) {
        struct bridge_can_node *n = &bridge_can_nodes[i];
        if (n->node_id)
            continue;
        n->node_id = node_id;
        n->len = n->ready = 0;
        return n;
    }
    return NULL;
}

// An admin answer heard on the bus: an unassigned device's NEED_NODEID, whole in
// one frame (the channel's payload is the command byte, its uuid, and the
// application byte when the device sends one). Copied here for the task; the
// frame is not a node's stream and has no unit framing to reassemble.
//
// Everything else on this id is somebody else's answer or a shape this port
// cannot forward - dropped and counted, since the channel asks for no receipt.
// Consumed either way: the id is the admin channel of the whole bus, not another
// device's data.
static int
bridge_can_admin_answer(struct canbus_msg *msg)
{
    uint_fast8_t len = CANMSG_DATA_LEN(msg);
    if (bridge_can.port <= 0 || (msg->id & CANMSG_ID_RTR)
        || len < 1 + MUX_UUID_LEN || msg->data[0] != MUX_RESP_NEED_NODEID) {
        bridge_can.rx_ignored++;
        return 1;
    }
    irqstatus_t flag = irq_save();
    uint8_t tail = bridge_can_admin.tail;
    uint8_t next = (tail + 1) % BRIDGE_CAN_ADMIN_FRAMES;
    if (next == bridge_can_admin.head) {
        bridge_can.rx_overrun++;
        irq_restore(flag);
        return 1;
    }
    memcpy(bridge_can_admin.data[tail], msg->data, len);
    bridge_can_admin.len[tail] = len;
    bridge_can_admin.tail = next;
    irq_restore(flag);
    sched_wake_tasks();
    return 1;
}

// A frame heard on the bus (canbus.c, from the driver's interrupt). A frame of a
// node behind this port is one more slice of that node's stream; every other id
// on the bus is somebody else's traffic and is not this port's to consume.
int
bridge_can_process_data(struct canbus_msg *msg)
{
    if (msg->id == MUX_ADMIN_REPLY)
        return bridge_can_admin_answer(msg);
    uint32_t node_id = mux_node_id_from_arb(msg->id);
    if (!node_id)
        return 0;
    // An id of this port's band, but with nobody to hand it to: not yet attached
    // to the bridge, no room in the table, or a remote frame (which carries no
    // data to reassemble). Consumed either way - it was addressed to a device of
    // this bus, not to this build's own node.
    if (bridge_can.port <= 0 || (msg->id & CANMSG_ID_RTR)) {
        bridge_can.rx_ignored++;
        return 1;
    }
    struct bridge_can_node *n = bridge_can_node_get(node_id);
    uint_fast8_t len = CANMSG_DATA_LEN(msg);
    if (!n || n->len + len > BRIDGE_CAN_NODE_BUF) {
        // The buffer holds an unfinished piece that cannot complete in it: hand
        // that fragment on (the router resynchronises on it, the way it cuts a
        // tunnel block at the buffer bound) and let the task make room.
        if (n && !n->ready)
            n->ready = n->len, sched_wake_tasks();
        bridge_can.rx_overrun++;
        return 1;
    }
    memcpy(&n->buf[n->len], msg->data, len);
    n->len += len;
    bridge_can_judge(n);
    if (n->ready)
        sched_wake_tasks();
    return 1;
}

// Hand the whole pieces each node has assembled to the router. Run from a task:
// the router buffers and scans a whole unit and must not nest in an interrupt,
// and it is handed one piece at a time so the interrupts-off section stays short.
void
bridge_can_task(void)
{
    int index = bridge_can.port - 1;
    if (index < 0)
        return;
    // The admin answers first, each as the one whole unit the router knows them
    // by (mux_kxid_unit): the header naming the reply id, then the pseudo-frame
    // around the bytes the device sent. Built here rather than in the interrupt -
    // the frame's CRC is not the interrupt's work - and handed over outside the
    // lock, the way the nodes' pieces are.
    uint8_t unit[MUX_HEADER_BYTES + MESSAGE_MAX];
    for (;;) {
        irqstatus_t flag = irq_save();
        uint8_t head = bridge_can_admin.head;
        if (head == bridge_can_admin.tail) {
            irq_restore(flag);
            break;
        }
        uint_fast8_t frame_len = bridge_can_admin.len[head];
        uint8_t frame[8];
        memcpy(frame, bridge_can_admin.data[head], frame_len);
        bridge_can_admin.head = (head + 1) % BRIDGE_CAN_ADMIN_FRAMES;
        irq_restore(flag);
        uint_fast8_t len = mux_kxid_unit(unit, sizeof(unit), MUX_ADMIN_REPLY
                                        , frame, frame_len);
        if (len)
            bridge_port_input(index, unit, len);
        else
            bridge_can.rx_ignored++;   // cannot be framed: nothing to forward
    }
    int i;
    for (i = 0; i < BRIDGE_CAN_NODE_MAX; i++) {
        struct bridge_can_node *n = &bridge_can_nodes[i];
        if (!n->node_id || !n->ready)
            continue;
        // Take the whole pieces out and give the node's space back to the
        // interrupt; the router is called with the copy, outside the lock.
        irqstatus_t flag = irq_save();
        uint16_t ready = n->ready;
        memcpy(bridge_can_handover, n->buf, ready);
        memmove(n->buf, &n->buf[ready], n->len - ready);
        n->len -= ready;
        n->ready = 0;
        irq_restore(flag);
        bridge_port_input(index, bridge_can_handover, ready);
    }
}
DECL_TASK(bridge_can_task);


/****************************************************************
 * The port's seam (uart.h, answered for this backend by bridge_port.c)
 ****************************************************************/

// Open this build's CAN controller for 'u'. The rate is CONFIG_CANBUS_FREQUENCY
// (canbus.c reports it to the host), so 'baud' is a dummy parameter here - see
// the note at the top of this file - and the controller takes one object.
enum uart_open_result
bridge_can_open(struct uart_s *u, uint32_t baud)
{
    (void)baud;
    if (bridge_can.owner)
        return UART_OPEN_BUSY;
    bridge_can.owner = u;
    bridge_can.port = 0;
    bridge_can.node_id = 0;
    bridge_can.tx_head = bridge_can.tx_tail = 0;
    bridge_can.tx_busy = 0;
    bridge_can_admin.head = bridge_can_admin.tail = 0;
    int i;
    for (i = 0; i < BRIDGE_CAN_NODE_MAX; i++) {
        bridge_can_nodes[i].node_id = 0;
        bridge_can_nodes[i].len = 0;
        bridge_can_nodes[i].ready = 0;
    }
#if CONFIG_CANBUS_FILTER_MULTI
    // The whole bus's admin answer id: every unassigned device answers there, so
    // it is exactly one id and no mask. Armed here rather than with the port's
    // node id, because a device that has no id yet is what this port has to hear
    // to be able to give it one.
    if (canhw_add_filter(MUX_ADMIN_REPLY, 0x7FF) < 0)
        shutdown("bridge CAN port: no filter bank for the CAN admin answer");
#else
    // Without a spare bank the port cannot hear an unassigned device at all, so
    // it cannot be scanned for or assigned: that is a configuration that reads
    // as working and never does, so refuse it here.
    shutdown("bridge CAN port: cannot listen to the CAN admin answer"
             " (CONFIG_CANBUS_FILTER_MULTI is off)");
#endif
    return UART_OPEN_OK;
}

int
bridge_can_owns(const struct uart_s *u)
{
    return u && bridge_can.owner == u;
}

#if CONFIG_BRIDGE
// The id the host named this port's device with (SET_PORT_NODEID, through the
// ops' node_id hook in bridge.c): the address a blob with no unit header is sent
// under, and the pair of ids the hardware filter has to hear.
static void
bridge_can_set_node_id(void *ctx, uint8_t node_id)
{
    if (ctx != bridge_can.owner || !node_id)
        return;
    bridge_can.node_id = node_id;
    uint32_t arb = mux_arb_id(node_id);
    // bridge.c only names an id the wire can address (mux_arb_id() nonzero), so
    // this is belt and braces; it must not fail silently either way.
    if (!arb)
        shutdown("bridge CAN port: node id out of the addressable range");
#if CONFIG_CANBUS_FILTER_MULTI
    // One bank per pair: the bit-0 mask matches 0x100+2n and its +1 together,
    // which are the two addresses a device listens and answers on.
    if (canhw_add_filter(arb & ~1u, 0x7FE) < 0)
        shutdown("bridge CAN port: no filter bank left for another"
                 " downstream id");
#else
    // Without the capability this port would sit on a bus it cannot hear, which
    // is worse than refusing the configuration: say so.
    shutdown("bridge CAN port: cannot listen to another downstream id"
             " (CONFIG_CANBUS_FILTER_MULTI is off)");
#endif
}

// A host's admin broadcast, translated into the frames a CAN device's admin
// channel spells. The host sends a whole KXID unit like it sends every other
// admin broadcast, but the device behind this port listens for one short frame
// per command, its first byte being the command itself (generic/canserial.h
// CANBUS_ID_ADMIN). So the unit's frame is read as a KXID body and answered with
// the frames that mean the same thing on the bus:
//
//   MUX_CMD_QUERY_UNASSIGNED   [0x00]                 one frame, nothing else
//   MUX_CMD_SET_KLIPPER_NODEID [0x01][uuid][nodeid]   the application's command
//                              [0x11][uuid][nodeid]   katapult's, same body
//
// An assignment goes out as both because the port does not know which firmware
// is on the other side of it, and the case that brought the host here - a device
// parked in its bootloader - is the one neither side can rule out. Each firmware
// ignores the command byte it does not know, so exactly one of the two takes
// effect.
//
// The port-level and device commands (0x02..0x06) never reach this: bridge.c
// routes and consumes them itself. Anything else a host puts on the admin id is
// dropped and counted (tx_unspelled) rather than sliced onto the bus as if it
// were a stream, which is what the path below would otherwise do with it.
static int
bridge_can_admin_write(const uint8_t *frame, size_t frame_len)
{
    struct mux_kxid kxid;
    // mux_parse_kxid() only reads the frame, but it spells its parameter as
    // writable (it is called on stream buffers elsewhere).
    if (!mux_parse_kxid((uint8_t *)frame, frame_len, &kxid) || !kxid.args_len) {
        bridge_can.tx_unspelled++;
        return -1;
    }
    uint8_t out[1 + MUX_UUID_LEN + 1];
    if (kxid.args[0] == MUX_CMD_QUERY_UNASSIGNED) {
        out[0] = MUX_CMD_QUERY_UNASSIGNED;
        return bridge_can_tx_blob(MUX_ADMIN_LISTEN, out, 1);
    }
    if (kxid.args[0] == MUX_CMD_SET_KLIPPER_NODEID
        && kxid.args_len >= 1 + MUX_UUID_LEN + 1) {
        // The same [uuid][nodeid] body under both command bytes, one frame
        // after the other in the same queue: the pair is a single broadcast as
        // far as the devices are concerned.
        out[0] = MUX_CMD_SET_KLIPPER_NODEID;
        memcpy(&out[1], &kxid.args[1], MUX_UUID_LEN + 1);
        if (bridge_can_tx_blob(MUX_ADMIN_LISTEN, out, sizeof(out)) < 0)
            return -1;
        out[0] = BRIDGE_CAN_CMD_SET_BOOTLOADER_NODEID;
        return bridge_can_tx_blob(MUX_ADMIN_LISTEN, out, sizeof(out));
    }
    bridge_can.tx_unspelled++;
    return -1;
}

// The router's write hook: one blob, sent whole on the address it names or on the
// port's node id when the router stripped the header.
static int
bridge_can_write(void *ctx, const uint8_t *data, size_t len)
{
    if (ctx != bridge_can.owner)
        return -1;
    uint32_t arb;
    uint_fast8_t consumed;
    if (!len)
        return 0;
    if (mux_decode_header(data, len, &arb, &consumed)
        == MUX_HEADER_COMPLETE) {
        if (arb == MUX_ADMIN_LISTEN)
            // An admin broadcast: this port's devices do not read the mux
            // framing at all, and the unit has to be translated before its
            // header is stripped (the body is a pseudo-frame, not a stream).
            return bridge_can_admin_write(data + consumed, len - consumed);
        // A unit: its header is this port's framing, so the frame after it is
        // what goes on the wire, under the address the header named.
        data += consumed;
        len -= consumed;
    } else if (bridge_can.node_id) {
        // No header - a legacy port's raw frame or a tunnel port's katapult
        // block - so the port's own node id is the address: the one its device
        // listens on.
        arb = mux_arb_id(bridge_can.node_id);
    } else {
        // Neither: there is no address for this blob. Dropped, never blocked on.
        bridge_can.tx_unaddressed++;
        return -1;
    }
    return bridge_can_tx_blob(arb, data, len);
}

static const struct bridge_port_ops bridge_can_ops = {
    .write = bridge_can_write,
    // CAN has no electrical reset line: the port-level RESET_PORT
    // (MUX_CMD_RESET_PORT) has nothing to pulse here, so the router drops it in
    // silence (bridge.h), exactly as it does for a USART port.
    .reset = NULL,
    // The host names the port's device (SET_PORT_NODEID), which is what this port
    // has to listen for.
    .node_id = bridge_can_set_node_id,
};

const struct bridge_port_ops *
bridge_can_port_ops(struct uart_s *u)
{
    (void)u;
    return &bridge_can_ops;
}

// The bridge took this object as the port with this index; the task feeds the
// units it assembles under it. The index belongs to the object that opened the
// controller.
void
bridge_can_set_port(struct uart_s *u, int index)
{
    if (bridge_can.owner == u)
        bridge_can.port = index + 1;
}
#endif // CONFIG_BRIDGE
