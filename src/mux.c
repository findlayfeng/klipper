// The serial mux: unit headers, the `KXID` pseudo-frame and the admin units
//
// The device half of the serial mux (see mux.h for what a unit is and where the
// specification lives). This file owns the encoding and decoding - the checked
// two-byte header, the pseudo-frame every admin command rides, and the units the
// admin channel carries - and the device half that uses them: the identity and
// node id a device answers with, the admin channel it replies on, the
// id-collision rule, and the in-place resolution of an incoming stream.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy, memcmp
#include "autoconf.h" // CONFIG_MUX_SERIAL, CONFIG_BRIDGE
#include "bridge.h" // bridge_admin_unit, bridge_route_unit
#include "command.h" // MESSAGE_*, MESSAGE_SYNC
#include "generic/misc.h" // crc16_ccitt, mcu_restart
#include "mux.h" // the mux unit format
#include "sched.h" // shutdown

// TODO(ring): the input path still runs through the carrier's linear buffer -
// mux_input() compacts the front of it and the frame parser reads a buffer
// head. Feeding the console's bytes straight into a ring, and handing the
// parser a (ptr, len) inside it, is the next step, and these are the
// constraints that make it a redesign rather than a different buffer:
//
//  - USB cannot be handed a short contiguous run: btable_read_packet()
//    (stm32/usbfs.c:130-136) clamps the copy to max_len and then releases the
//    packet (usbfs.c:228-229), so a run shorter than USB_CDC_EP_BULK_OUT_SIZE
//    silently loses the rest of the packet - corruption of the byte stream, not
//    backpressure. generic/usb_cdc.c:159 lives with that by reading only so
//    long as the linear tail holds a whole packet; a ring has to reclaim the
//    wrap before it reads, never read across it.
//  - A raw frame is not a unit. mux_input() hands an unheaded frame straight to
//    the parser, so the "one maximum unit" hole that keeps a *unit* contiguous
//    does not bound it: a raw frame (up to MESSAGE_MAX) can straddle the wrap
//    too. The UART producer is a per-byte interrupt (generic/serial_irq.c:28)
//    and cannot decline to cross a boundary it does not know, so the hole would
//    have to be known to the reader (state in the ring, or a marker) instead of
//    inferred from the bytes.
//  - The critical section may not grow. generic/serial_irq.c:110-116 already
//    masks interrupts across mux_input()'s moves, and tidying up at the wrap is
//    a bounded memmove that needs the producer quiesced the same way.
//
// What those point at is "wrap only once the ring is drained, tidy up lazily":
// frames and units then always start at the read offset and are always
// contiguous, no marker bytes are needed, and the one move happens at a wrap
// instead of after every unit.

// The header is a shortened Hamming code: ten data bits and four check bits
// over fourteen positions, the parity positions being the powers of two and the
// data bits sitting in every other position. Position 15 - the one a full
// fifteen-bit code would add - is what the shortened code drops.
//
// The point of it is detection, never correction: the columns of the
// parity-check matrix are the fourteen positions (every non-zero four-bit
// pattern but 15), so they are distinct and non-zero, the minimum distance is
// three, and every one- or two-bit error changes the syndrome. The frame body's
// CRC does not cover the header, so a flipped bit that landed on another legal
// id would otherwise hand one MCU another's command.
#define MUX_HEADER_CODE_BITS 14
#define MUX_HEADER_DATA_BITS 10
#define MUX_HEADER_CHECK_BITS 4

static const uint8_t mux_data_positions[MUX_HEADER_DATA_BITS] =
    { 3, 5, 6, 7, 9, 10, 11, 12, 13, 14 };
static const uint8_t mux_check_positions[MUX_HEADER_CHECK_BITS] =
    { 1, 2, 4, 8 };

// The codeword for an arbitration id: its data bits, plus the check bits that
// make every column of the parity-check matrix sum to even.
static uint16_t
mux_codeword(uint32_t arb)
{
    uint16_t code = 0;
    uint_fast8_t bit;
    for (bit = 0; bit < MUX_HEADER_DATA_BITS; bit++) {
        if (arb & (1 << bit))
            code |= 1 << (MUX_HEADER_CODE_BITS - mux_data_positions[bit]);
    }
    uint_fast8_t check;
    for (check = 0; check < MUX_HEADER_CHECK_BITS; check++) {
        // This check bit covers the positions whose own encoding has the bit
        // set; it is set to make that group, plus itself, even.
        int covered = 0;
        for (bit = 0; bit < MUX_HEADER_DATA_BITS; bit++) {
            if ((mux_data_positions[bit] >> check) & 1 && (arb & (1 << bit)))
                covered++;
        }
        if (covered % 2)
            code |= 1 << (MUX_HEADER_CODE_BITS - mux_check_positions[check]);
    }
    return code;
}

// The syndrome of a codeword: the exclusive-or of the positions of its set
// bits, zero exactly for a well-formed codeword. A one-bit error leaves the
// flipped position, a two-bit error the exclusive-or of the two; non-zero
// either way, because the positions are distinct and non-zero.
static uint8_t
mux_syndrome(uint16_t code)
{
    uint8_t syndrome = 0;
    uint_fast8_t position;
    for (position = 1; position <= MUX_HEADER_CODE_BITS; position++) {
        if (code & (1 << (MUX_HEADER_CODE_BITS - position)))
            syndrome ^= position;
    }
    return syndrome;
}

// The ten data bits of a well-formed codeword.
static uint32_t
mux_data(uint16_t code)
{
    uint32_t arb = 0;
    uint_fast8_t bit;
    for (bit = 0; bit < MUX_HEADER_DATA_BITS; bit++) {
        if (code & (1 << (MUX_HEADER_CODE_BITS - mux_data_positions[bit])))
            arb |= 1 << bit;
    }
    return arb;
}

// Encode an arbitration id as the two bytes that go on the wire, answering how
// many were written (0 for an id in the reserved band, which names no address:
// a programming error, so nothing is put on the wire).
//
// The leading 0x80 and the clear high bit of the second byte are the framing
// kept from the variable-length encoding this replaces: they are what makes the
// two-byte boundary unambiguous on the stream, and they are also what tells a
// unit from a raw frame.
uint_fast8_t
mux_encode_header(uint8_t *buf, uint32_t arb)
{
    if (arb >= MUX_HEADER_RESERVED_FIRST)
        return 0;
    uint16_t code = mux_codeword(arb);
    buf[0] = 0x80 | ((code >> 7) & 0x7f);
    buf[1] = code & 0x7f;
    return MUX_HEADER_BYTES;
}

// Decode the frame header at the front of the buffer.
//
// Note that a pair of bytes which breaks the two framing bits is not a header
// whatever its check bits say: that catches two more bit errors and keeps the
// two-byte boundary unambiguous.
enum mux_header_scan
mux_decode_header(const uint8_t *buf, uint_fast8_t len
                  , uint32_t *arb, uint_fast8_t *consumed)
{
    if (len < MUX_HEADER_BYTES)
        return MUX_HEADER_NEED_MORE;
    if (!(buf[0] & 0x80) || (buf[1] & 0x80))
        return MUX_HEADER_INVALID;
    uint16_t code = ((uint16_t)(buf[0] & 0x7f) << 7) | (buf[1] & 0x7f);
    if (mux_syndrome(code))
        return MUX_HEADER_INVALID;
    uint32_t id = mux_data(code);
    if (id >= MUX_HEADER_RESERVED_FIRST)
        return MUX_HEADER_INVALID;
    *arb = id;
    *consumed = MUX_HEADER_BYTES;
    return MUX_HEADER_COMPLETE;
}

// The arbitration ids of a node id, as CAN's can_get_nodeid/can_decode_nodeid
// spell them for its single-byte node id.
uint32_t
mux_arb_id(uint32_t nodeid)
{
    if (nodeid < 1 || nodeid > MUX_MAX_NODE_ID)
        return 0;
    return MUX_ARB_BASE + 2 * nodeid;
}

uint32_t
mux_reply_arb_id(uint32_t nodeid)
{
    uint32_t arb = mux_arb_id(nodeid);
    return arb ? arb + 1 : 0;
}

uint32_t
mux_node_id_from_arb(uint32_t arb)
{
    if (arb < MUX_ARB_BASE + 2
        || arb > MUX_ARB_BASE + 2 * MUX_MAX_NODE_ID + 1)
        return 0;
    return (arb - MUX_ARB_BASE) >> 1;
}

int
mux_is_reply_arb(uint32_t arb)
{
    return mux_node_id_from_arb(arb) && ((arb - MUX_ARB_BASE) & 1);
}

// Whether `frame` is a whole, well-formed `KXID` frame, and what follows the
// marker.
//
// Everything has to be self-consistent: the length byte has to match the bytes
// present and be a legal frame length, the sequence byte has to be the value
// only this pseudo-frame uses, the trailer and the CRC have to check out, and
// the payload has to begin with the marker. That is what makes the pseudo-frame
// recognizable without ever being mistaken for - or accepted by - the ordinary
// parser (command_find_block rejects its sequence byte).
int
mux_parse_kxid(uint8_t *frame, uint_fast8_t frame_len, struct mux_kxid *out)
{
    if (frame_len < MESSAGE_MIN || frame_len > MESSAGE_MAX)
        return 0;
    if (frame[MESSAGE_POS_LEN] != frame_len)
        return 0;
    if (frame[MESSAGE_POS_SEQ] != MUX_ASSIGN_SEQ_BYTE)
        return 0;
    if (frame[frame_len - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC)
        return 0;
    uint16_t expected = crc16_ccitt(frame, frame_len - MESSAGE_TRAILER_SIZE);
    uint16_t actual = ((uint16_t)frame[frame_len - MESSAGE_TRAILER_CRC] << 8)
        | frame[frame_len - MESSAGE_TRAILER_CRC + 1];
    if (expected != actual)
        return 0;
    uint_fast8_t payload_len =
        frame_len - MESSAGE_HEADER_SIZE - MESSAGE_TRAILER_SIZE;
    if (payload_len < MUX_MAGIC_LEN
        || memcmp(&frame[MESSAGE_HEADER_SIZE], MUX_MAGIC, MUX_MAGIC_LEN))
        return 0;
    out->args = &frame[MESSAGE_HEADER_SIZE + MUX_MAGIC_LEN];
    out->args_len = payload_len - MUX_MAGIC_LEN;
    return 1;
}

// Frame `payload` the way command_add_frame() frames a message, with the
// pseudo-frame's sequence byte instead of a sequence number.
static uint_fast8_t
mux_frame(uint8_t *buf, uint_fast8_t msglen)
{
    buf[MESSAGE_POS_LEN] = msglen;
    buf[MESSAGE_POS_SEQ] = MUX_ASSIGN_SEQ_BYTE;
    uint16_t crc = crc16_ccitt(buf, msglen - MESSAGE_TRAILER_SIZE);
    buf[msglen - MESSAGE_TRAILER_CRC + 0] = crc >> 8;
    buf[msglen - MESSAGE_TRAILER_CRC + 1] = crc;
    buf[msglen - MESSAGE_TRAILER_SYNC] = MESSAGE_SYNC;
    return msglen;
}

// Build `[len][0x20]["KXID"][payload][crc][0x7e]`, answering its length or 0
// when it does not fit.
uint_fast8_t
mux_kxid_frame(uint8_t *buf, uint_fast8_t buf_size
               , const uint8_t *payload, uint_fast8_t payload_len)
{
    uint_fast8_t msglen = (MESSAGE_HEADER_SIZE + MUX_MAGIC_LEN + payload_len
                           + MESSAGE_TRAILER_SIZE);
    if (msglen < MESSAGE_MIN || msglen > MESSAGE_MAX || msglen > buf_size)
        return 0;
    memcpy(&buf[MESSAGE_HEADER_SIZE], MUX_MAGIC, MUX_MAGIC_LEN);
    if (payload_len)
        memcpy(&buf[MESSAGE_HEADER_SIZE + MUX_MAGIC_LEN], payload, payload_len);
    return mux_frame(buf, msglen);
}

// Build a whole unit: the header naming `arb`, then the pseudo-frame above.
uint_fast8_t
mux_kxid_unit(uint8_t *buf, uint_fast8_t buf_size, uint32_t arb
              , const uint8_t *payload, uint_fast8_t payload_len)
{
    uint_fast8_t header = mux_encode_header(buf, arb);
    if (!header)
        return 0;
    uint_fast8_t frame = mux_kxid_frame(&buf[header], buf_size - header
                                        , payload, payload_len);
    if (!frame)
        return 0;
    return header + frame;
}

// The broadcast that asks every device without a node id to identify itself, on
// the admin id every unassigned device listens on.
uint_fast8_t
mux_query_unassigned(uint8_t *buf, uint_fast8_t buf_size)
{
    uint8_t payload[1] = { MUX_CMD_QUERY_UNASSIGNED };
    return mux_kxid_unit(buf, buf_size, MUX_ADMIN_LISTEN
                         , payload, sizeof(payload));
}

// The broadcast that tells the device with `uuid` to take `nodeid` - the same
// `[cmd][uuid][nodeid]` body CAN's admin message carries.
uint_fast8_t
mux_set_node_id(uint8_t *buf, uint_fast8_t buf_size
                , const uint8_t *uuid, uint32_t nodeid)
{
    if (!mux_arb_id(nodeid))
        return 0;
    uint8_t payload[1 + MUX_UUID_LEN + 1];
    payload[0] = MUX_CMD_SET_KLIPPER_NODEID;
    memcpy(&payload[1], uuid, MUX_UUID_LEN);
    payload[1 + MUX_UUID_LEN] = nodeid;
    return mux_kxid_unit(buf, buf_size, MUX_ADMIN_LISTEN
                         , payload, sizeof(payload));
}

// An unassigned device's answer to the broadcast: its uuid, and - in the builds
// that send one - the application byte saying what it runs. A device that sends
// no byte is read as Klipper, so this one does not (the shorter of the two
// shapes the host accepts).
uint_fast8_t
mux_need_nodeid(uint8_t *buf, uint_fast8_t buf_size
                , const uint8_t *uuid, int with_app, uint8_t app)
{
    uint8_t payload[1 + MUX_UUID_LEN + 1];
    uint_fast8_t args_len = 1 + MUX_UUID_LEN;
    payload[0] = MUX_RESP_NEED_NODEID;
    memcpy(&payload[1], uuid, MUX_UUID_LEN);
    if (with_app)
        payload[args_len++] = app;
    return mux_kxid_unit(buf, buf_size, MUX_ADMIN_REPLY
                         , payload, args_len);
}

// The same answer as `mux_kxid_unit` builds for its device, but travelled: the
// payload a downstream port answered with, with `port` - this bridge's port
// index - appended to its port path, framed again as one whole unit.
//
// `args` is a NEED_NODEID payload as it arrived from below, `[0x20][uuid]
// [app?][path?]`: the application byte and the path are both optional and each
// is recognised by the top bit of the byte that starts it, so the path this
// bridge appends goes after whatever was there. A path already `MUX_PATH_MAX`
// long, or one that does not parse (a count of zero, a count the payload does
// not carry), is dropped rather than grown or half-kept: the answer then says
// no more than "behind this bridge's port `port`", which is true whatever the
// broken field was.
//
// 0 when `args` is not an answer, or the unit does not fit `buf_size`.
uint_fast8_t
mux_need_nodeid_via(uint8_t *buf, uint_fast8_t buf_size
                    , const uint8_t *args, uint_fast8_t args_len
                    , uint8_t port)
{
    // The answer's worst case is the uuid, an application byte, a full path and
    // this bridge's own port appended to it - the marker and the port byte sit
    // *after* the path, so the buffer needs two more bytes than the path bound
    // alone suggests (the last path byte and the two appended ones).
    uint8_t payload[1 + MUX_UUID_LEN + 1 + 1 + MUX_PATH_MAX + 2];
    if (args_len < 1 + MUX_UUID_LEN || args_len > sizeof(payload)
        || args[0] != MUX_RESP_NEED_NODEID)
        return 0;
    uint_fast8_t len = 1 + MUX_UUID_LEN;
    memcpy(payload, args, len);
    const uint8_t *tail = &args[len];
    uint_fast8_t tail_len = args_len - len;
    uint_fast8_t path = 0;
    if (tail_len && !(tail[0] & MUX_PATH_FLAG)) {
        payload[len++] = tail[0];      // the application byte
        tail++;
        tail_len--;
    }
    if (tail_len && (tail[0] & MUX_PATH_FLAG)) {
        uint_fast8_t count = tail[0] & ~MUX_PATH_FLAG;
        if (count && count <= MUX_PATH_MAX && tail_len == 1 + count) {
            memcpy(&payload[len], tail, tail_len);
            len += tail_len;
            path = count;
        }
    }
    if (path >= MUX_PATH_MAX) {
        // Full: the whole path goes, and this bridge's port starts a new one.
        len -= 1 + path;
        path = 0;
    }
    payload[len++] = MUX_PATH_FLAG | (path + 1);
    payload[len++] = port;
    return mux_kxid_unit(buf, buf_size, MUX_ADMIN_REPLY, payload, len);
}

// What a device answers an assignment request with: the same pseudo-frame back
// (the id is in the header, so the body is the marker alone), on its own reply
// id. The request names the device by uuid; this reply is the confirmation the
// host's handshake waits for.
uint_fast8_t
mux_assignment_reply(uint8_t *buf, uint_fast8_t buf_size, uint32_t nodeid)
{
    uint32_t arb = mux_reply_arb_id(nodeid);
    if (!arb)
        return 0;
    return mux_kxid_unit(buf, buf_size, arb, 0, 0);
}


/****************************************************************
 * The device half
 ****************************************************************/

// The identity the host assigned (or the chip id the micro-controller derived),
// and the node id this device answers on. All zeros and 0 mean "nothing yet":
// an unnamed device, told apart by nothing and addressed by nobody.
static uint8_t mux_identity[MUX_UUID_LEN];
static uint32_t mux_assigned;

void
mux_set_uuid(const uint8_t *uuid)
{
    memcpy(mux_identity, uuid, MUX_UUID_LEN);
}

const uint8_t *
mux_uuid(void)
{
    return mux_identity;
}

uint32_t
mux_node_id(void)
{
    return mux_assigned;
}

// Adopt a node id another layer already accepted (generic/canserial.c's admin
// frame is where a CAN build hears one). The device half answers, heads and
// routes by this value, so it has to be the same number.
void
mux_set_assigned(uint32_t nodeid)
{
    mux_assigned = nodeid;
}

// What our own frames go out with.
uint_fast8_t
mux_out_header(uint8_t *buf)
{
    if (!mux_assigned)
        return 0;
    return mux_encode_header(buf, mux_reply_arb_id(mux_assigned));
}

// A REQUEST_BOOTLOADER that reached this device: ask for the bootloader the way
// CAN's CANBUS_CMD_REQUEST_BOOTLOADER does (generic/canserial.c). The command
// carries a port index for a bridge to route by (mux.h
// MUX_CMD_REQUEST_BOOTLOADER); on this wire the routing has already named this
// device, so the index means nothing here and is read no further. There is no
// uuid to check either - the port index is the addressing - and a build with no
// bootloader simply does nothing, the same no-op canserial.c makes without the
// config.
static void
mux_process_request_bootloader(void)
{
    if (!CONFIG_HAVE_BOOTLOADER_REQUEST)
        return;
    bootloader_request();
}

// One unit that arrived on the admin id: the assignment handshake, and the
// commands an unassigned device answers.
//
// A device that has no node id answers the discovery broadcast with its
// identity and adopts the id it is handed when the assignment names that
// identity. It answers nothing else: the handshake is the re-confirmation of an
// id already adopted, so it needs one to answer on.
static void
mux_admin_unit(uint8_t *frame, uint_fast8_t frame_len)
{
    struct mux_kxid kxid;
    if (!mux_parse_kxid(frame, frame_len, &kxid))
        // Not a command of ours: a frame the ordinary parser would reject
        // anyway. Nothing to answer.
        return;
    uint8_t out[MUX_HEADER_BYTES + MESSAGE_MAX];
    if (!kxid.args_len) {
        uint_fast8_t n = mux_assignment_reply(out, sizeof(out), mux_assigned);
        if (n)
            mux_send(out, n);
        return;
    }
    uint32_t nodeid;
    switch (kxid.args[0]) {
    case MUX_CMD_QUERY_UNASSIGNED:
        if (mux_assigned)
            // An addressed device stays quiet, which is what makes a scan list
            // the boards a config still has to name.
            return;
        uint_fast8_t n = mux_need_nodeid(out, sizeof(out), mux_identity, 0, 0);
        if (n)
            mux_send(out, n);
        return;
    case MUX_CMD_SET_KLIPPER_NODEID:
        if (kxid.args_len < 1 + MUX_UUID_LEN + 1)
            return;
        nodeid = kxid.args[1 + MUX_UUID_LEN];
        if (!memcmp(&kxid.args[1], mux_identity, MUX_UUID_LEN)) {
            // Ours: adopt it. Idempotent, and an id the wire cannot address is
            // not adopted at all (there is no header to answer on).
            if (mux_arb_id(nodeid))
                mux_assigned = nodeid;
            return;
        }
        if (nodeid && nodeid == mux_assigned)
            // Someone else was handed the id we already answer on: the CAN
            // rule, spelled the same way (generic/canserial.c:can_id_conflict) -
            // give it up and say so, rather than answer for two boards.
            goto conflict;
        return;
    case MUX_CMD_REQUEST_BOOTLOADER:
        // CAN's CANBUS_CMD_REQUEST_BOOTLOADER by value and meaning: this device
        // hands itself to its bootloader, for a flashing tool to write it. The
        // port index the command carries is a bridge's routing (see
        // mux_process_request_bootloader above), so it is read no further here.
        mux_process_request_bootloader();
        return;
    case MUX_CMD_RESTART_DEVICE:
        // The other reset, and a different destination: restart the firmware
        // itself (mcu_restart, the body of the `reset` command) and come back
        // up, rather than wait in a bootloader. The port index is the bridge's
        // routing here too, so it is read no further.
        mcu_restart();
        return;
    default:
        return;
    }
conflict:
    mux_assigned = 0;
    shutdown("Another node assigned this ID");
}

// One unit that arrived with its header already decoded: what this device does
// with it. Answers MUX_UNIT_OURS for a frame addressed to us (the caller leaves
// it for the parser, its header stripped) and MUX_UNIT_CONSUMED for everything
// else.
enum mux_unit_result { MUX_UNIT_CONSUMED = 0, MUX_UNIT_OURS = 1 };

static int
mux_unit(uint32_t arb, uint8_t *frame, uint_fast8_t frame_len)
{
    if (arb == MUX_ADMIN_LISTEN) {
#if CONFIG_BRIDGE
        // The bridge owns the other ports: an admin broadcast is theirs too, so
        // it goes out before this device handles its own copy. Both are handed
        // the frame unchanged. A port-level command (SET_PORT_NODEID) names a
        // port of the bridge rather than this device, so it is the bridge's
        // alone: once it says so, this device's own handler has nothing to do
        // with the frame.
        if (bridge_admin_unit(arb, frame, frame_len))
            return MUX_UNIT_CONSUMED;
#endif
        mux_admin_unit(frame, frame_len);
        return MUX_UNIT_CONSUMED;
    }
    if (mux_assigned && arb == mux_arb_id(mux_assigned)) {
        // Addressed to us. The re-confirmation handshake arrives this way - it
        // is the `KXID` pseudo-frame with nothing after the marker, sent to the
        // id the assignment named - and it is answered with the same frame on
        // our reply id. Commands travel on the admin id instead (that is where
        // a device with no id is listening), so a marked frame carrying one is
        // left to the parser, which rejects the marker's sequence byte anyway.
        struct mux_kxid kxid;
        if (mux_parse_kxid(frame, frame_len, &kxid) && !kxid.args_len) {
            uint8_t out[MUX_HEADER_BYTES + MESSAGE_MAX];
            uint_fast8_t n = mux_assignment_reply(out, sizeof(out), mux_assigned);
            if (n)
                mux_send(out, n);
            return MUX_UNIT_CONSUMED;
        }
        return MUX_UNIT_OURS;
    }
    // Anything else addresses a device this one is not. A bridge routes those to
    // its other ports (and a single-device port gets its header stripped);
    // without one there is nobody to hand the frame to, so it is dropped.
#if CONFIG_BRIDGE
    bridge_route_unit(arb, frame, frame_len);
#endif
    return MUX_UNIT_CONSUMED;
}

// The tunnel unit contract (the MUX_TUNNEL_* defines and the length rule) lives
// in mux.h with the rest of the shared wire format; the bridge's port read side
// (bridge.c, tunnel mode) uses the same definitions to reassemble blocks.

// One tunnel frame's length for its word count, or 0 when the count is past the
// limit - katapult's own rule, and the one host serial.rs mirrors.
size_t
mux_tunnel_frame_len(uint8_t word_cnt)
{
    size_t frame_len = (size_t)word_cnt * 4 + MUX_TUNNEL_FRAME_MIN;
    if (frame_len > MUX_TUNNEL_FRAME_MAX)
        return 0;
    return frame_len;
}

// The unit at the front of the buffer, or 0 when it has not arrived whole.
static size_t
mux_unit_len(uint8_t *buf, size_t len, uint32_t *arb, uint_fast8_t *consumed)
{
    if (len < MUX_HEADER_BYTES)
        return 0;
    if (mux_decode_header(buf, MUX_HEADER_BYTES, arb, consumed)
        != MUX_HEADER_COMPLETE)
        // A byte with the framing bit set that is not a header: no frame starts
        // that way either, so it is dropped and the parse resynchronises on the
        // next byte - the same policy the frame parser applies to a block it
        // cannot read.
        return 1;
    if (len < (size_t)*consumed + 1)
        return 0;                     // the length byte is still coming
    if (buf[*consumed] == MUX_TUNNEL_TAG0) {
        // A tunnel unit opens with two tag bytes: until the second arrives, a
        // tunnel frame and junk that merely starts with 0x01 cannot be told
        // apart, so a lone 0x01 waits - the host reader waits the same way.
        if (len < (size_t)*consumed + 2)
            return 0;                 // the second tag byte is still coming
        if (buf[*consumed + 1] != MUX_TUNNEL_TAG1)
            return *consumed;         // 0x01 is no klipper length either: junk
        if (len < (size_t)*consumed + MUX_TUNNEL_POS_WORDCNT + 1)
            return 0;                 // the word count is still coming
        size_t frame_len = mux_tunnel_frame_len(
            buf[*consumed + MUX_TUNNEL_POS_WORDCNT]);
        if (!frame_len)
            return *consumed;         // a word count past 128: junk
        if (len < (size_t)*consumed + frame_len)
            return 0;                 // the frame is still arriving
        return (size_t)*consumed + frame_len;
    }
    uint_fast8_t msglen = buf[*consumed];
    if (msglen < MESSAGE_MIN || msglen > MESSAGE_MAX)
        return *consumed;             // a length no frame has: drop the header
    if (len < (size_t)*consumed + msglen)
        return 0;                     // the frame is still arriving
    return (size_t)*consumed + msglen;
}

// Resolve the units at the front of the buffer, in place.
//
// The units are read where they lie: `start` is how many bytes of the front the
// call has already accounted for (units handled, junk dropped), so each unit is
// parsed at &buf[start] and never moves the bytes behind it.
//
// What a call moves, in bytes: one memmove at most, over at most the length the
// call was handed (*len) - the compaction of what is left over at the end, or,
// when the call ends on a unit of ours, the single move that takes that unit's
// frame and the tail behind it down to the front. The frame itself is copied
// once there, and no intermediate buffer holds it. A unit that is junk or not
// ours moves nothing at all: it is only counted in `start`.
//
// That is what takes the quadratic term out of a busy wire. Dropping a unit
// used to move every byte behind it, so resolving N units that arrive together
// cost ~N/2 buffer lengths - O(N^2) in the bytes buffered. Now the same burst
// costs one pass over the buffer, O(N): one move of the unconsumed remainder,
// and one copy of a frame plus its tail in the same move when the call ends on
// a unit of ours.
int
mux_input(uint8_t *buf, size_t *len)
{
    size_t start = 0;
    int ready = 0;
    for (;;) {
        size_t avail = *len - start;
        if (!avail)
            break;                    // nothing left: no frame to hand over
        if (!(buf[start] & 0x80)) {
            // No header: an ordinary frame, which is what an exclusive path (and
            // the old wire format) carries. The parser owns it.
            ready = 1;
            break;
        }

        uint32_t arb;
        uint_fast8_t consumed;
        size_t unit_len = mux_unit_len(&buf[start], avail, &arb, &consumed);
        if (!unit_len)
            break;                    // a unit is on its way: do not parse it
        if (unit_len == 1 || unit_len == consumed) {
            // Junk instead of a unit: drop what cannot be a unit and look again.
            start += unit_len;
            continue;
        }

        uint_fast8_t msglen = unit_len - consumed;
        if (mux_unit(arb, &buf[start + consumed], msglen) == MUX_UNIT_OURS) {
            // Ours: the frame goes to the parser, its header gone. One move
            // takes the frame - and whatever followed the unit, which keeps the
            // tail contiguous behind it - down over the header and over the
            // bytes this call already consumed. The frame is copied once and
            // needs no intermediate buffer.
            memmove(buf, &buf[start + consumed], *len - start - consumed);
            *len -= start + consumed;
            return 1;
        }
        start += unit_len;
    }
    memmove(buf, &buf[start], *len - start);
    *len -= start;
    return ready;
}
