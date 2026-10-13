// The serial mux: unit headers, the `KXID` pseudo-frame and the admin units
//
// The device half of the serial mux (see mux.h for what a unit is and where the
// specification lives). This file owns the encoding and decoding: the checked
// two-byte header, the pseudo-frame every admin command rides, and the units
// the admin channel carries. The state machine that uses them - answering the
// discovery broadcast, adopting the node id it is handed, and the id-collision
// rule - is the next piece, together with the local parser's byte stream.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy, memcmp
#include "autoconf.h" // CONFIG_MUX_SERIAL
#include "command.h" // MESSAGE_*, MESSAGE_SYNC
#include "generic/misc.h" // crc16_ccitt
#include "mux.h" // the mux unit format

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
