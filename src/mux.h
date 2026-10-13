#ifndef __MUX_H
#define __MUX_H

#include <stdint.h> // uint8_t, uint32_t

// The serial mux: the unit format a shared byte stream carries.
//
// A unit is a checked two-byte header naming an arbitration id, followed by one
// unmodified klipper frame. The byte stream a device reads is therefore either
// raw frames (a path with one device on it) or units (a path with several), and
// the first byte says which: a header always starts with a bit set that no
// frame length has.
//
// The specification is docs/klippy/developer-manual/serial-mux-protocol.md in
// the klipperx checkout; the definitions here mirror its single Rust
// implementation (src/core/klippy/interface/devices/serial.rs and
// src/core/klippy/assign.rs). mux.c holds the encoding, the decoders and the
// unit builders; the device half that uses them (answering the admin channel,
// adopting a node id, the id-collision rule) is the next piece.

// The id a device is addressed by, as arbitration ids: CAN's mapping, and the
// same one the host uses (assign.rs). `mux_arb_id` and `mux_reply_arb_id`
// answer 0 for an id the wire format cannot address (the range is the single
// byte the admin message carries), so 0 is also "not a node's id".
#define MUX_ARB_BASE 0x100
#define MUX_MAX_NODE_ID 0xff
uint32_t mux_arb_id(uint32_t nodeid);
uint32_t mux_reply_arb_id(uint32_t nodeid);
uint32_t mux_node_id_from_arb(uint32_t arb);
int mux_is_reply_arb(uint32_t arb);

// The admin channel, spelled with CAN's own values (generic/canserial.h): the
// ids sit outside the assigned band, the even one listens and the odd one
// answers, and the commands and the response byte mean what they mean there.
#define MUX_ADMIN_LISTEN 0x3f0
#define MUX_ADMIN_REPLY 0x3f1
#define MUX_CMD_QUERY_UNASSIGNED 0x00
#define MUX_CMD_SET_KLIPPER_NODEID 0x01
#define MUX_RESP_NEED_NODEID 0x20

// A uuid is six bytes, most significant first.
#define MUX_UUID_LEN 6

// The two bytes a unit's header occupies.
#define MUX_HEADER_BYTES 2

// The band of legal codewords that name no address: a reader drops a unit whose
// header decodes into it, which is also where an extended header would start.
#define MUX_HEADER_RESERVED_FIRST 0x3f2
#define MUX_HEADER_MAX 0x3ff

// What reading a header off the front of a buffer found.
enum mux_header_scan {
    MUX_HEADER_COMPLETE,   // a whole arbitration id, in *consumed bytes
    MUX_HEADER_NEED_MORE,  // the buffer ends inside the header
    MUX_HEADER_INVALID,    // not a header: framing bit, check bits, or reserved
};
enum mux_header_scan mux_decode_header(const uint8_t *buf, uint_fast8_t len
                                       , uint32_t *arb, uint_fast8_t *consumed);
uint_fast8_t mux_encode_header(uint8_t *buf, uint32_t arb);

// The `KXID` pseudo-frame: a frame whose sequence byte is 0x20 (a value no
// ordinary frame uses), whose payload begins with the marker, and which the
// ordinary frame parser therefore always rejects. The assignment handshake and
// every admin command ride it, differing only in the bytes after the marker.
#define MUX_ASSIGN_SEQ_BYTE 0x20
#define MUX_MAGIC "KXID"
#define MUX_MAGIC_LEN 4

// The decoded body of a `KXID` frame: what follows the marker. An empty body is
// the assignment handshake itself; otherwise the first byte is a command.
struct mux_kxid {
    uint8_t *args;
    uint_fast8_t args_len;
};
int mux_parse_kxid(uint8_t *frame, uint_fast8_t frame_len, struct mux_kxid *out);

// The frames and units this protocol carries, each answering the bytes it wrote
// or 0 when the buffer is too small (or the id names no address).
uint_fast8_t mux_kxid_frame(uint8_t *buf, uint_fast8_t buf_size
                            , const uint8_t *payload, uint_fast8_t payload_len);
uint_fast8_t mux_kxid_unit(uint8_t *buf, uint_fast8_t buf_size, uint32_t arb
                           , const uint8_t *payload, uint_fast8_t payload_len);
uint_fast8_t mux_query_unassigned(uint8_t *buf, uint_fast8_t buf_size);
uint_fast8_t mux_set_node_id(uint8_t *buf, uint_fast8_t buf_size
                             , const uint8_t *uuid, uint32_t nodeid);
uint_fast8_t mux_need_nodeid(uint8_t *buf, uint_fast8_t buf_size
                             , const uint8_t *uuid, int with_app, uint8_t app);
uint_fast8_t mux_assignment_reply(uint8_t *buf, uint_fast8_t buf_size
                                  , uint32_t nodeid);

#endif // mux.h
