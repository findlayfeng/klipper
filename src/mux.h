#ifndef __MUX_H
#define __MUX_H

#include <stddef.h> // size_t
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
// src/core/klippy/assign.rs). mux.c holds the encoding, the decoders, the unit
// builders, and the device half that uses them: the identity and node id this
// device answers with, the admin channel it replies on, the id-collision rule,
// and the in-place resolution of an incoming stream (mux_input).

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

// Beyond the two commands CAN spells (QUERY_UNASSIGNED 0x00 and
// SET_KLIPPER_NODEID 0x01 above), the family splits by who executes it. The
// full command table, one row per command (payload after the `KXID` marker):
//
//   0x00 QUERY_UNASSIGNED   []                  every unassigned device answers
//   0x01 SET_KLIPPER_NODEID [uuid][id]          the device `uuid` adopts `id`
//   0x02 REQUEST_BOOTLOADER [index]             the device behind port `index`
//                                               asks for its bootloader - the
//                                               flashing tool's command: a
//                                               bridge routes it by `index` to
//                                               that port and the device
//                                               executes it; the device half
//                                               also runs it on a board with no
//                                               bridge (the index is then read
//                                               no further)
//   0x03 SET_PORT_NODEID    [index][id][flags]  the bridge builds its port
//                                               tables; stays on the bridge,
//                                               never forwarded
//   0x04 RESET_PORT         [index]             the bridge pulses the lines of
//                                               port `index` - the only reset a
//                                               device with no admin channel of
//                                               its own can get
//   0x05 RESTART_DEVICE     [index]             the device behind port `index`
//                                               restarts its own firmware: the
//                                               same routing as 0x02, but the
//                                               device comes back up instead of
//                                               waiting in a bootloader
//   0x06 SET_PORT_TUNNEL    [index]             the bridge toggles port
//                                               `index` into (or out of)
//                                               flashing tunnel mode; stays on
//                                               the bridge, never forwarded
//
// and one response:
//
//   0x20 NEED_NODEID        [uuid][app?][path?] an unassigned device's answer
//                                               to a scan
//
// The two optional fields of the answer are told apart by the top bit of the
// byte that starts each (MUX_PATH_FLAG): clear, it is the application byte;
// set, it is the port path - the port of each bridge the answer came through,
// appended there and in that order, innermost first. A device answered
// directly sends no path at all, which is the shape the answer had before the
// path existed.

// 0x02 and 0x05 name a device (routed by the bridge the same way, executed by
// the device); 0x03, 0x04 and 0x06 name the bridge itself and never travel
// downstream. The five (0x02, 0x03, 0x04, 0x05, 0x06) are addressed by the port
// index the identify dictionary reports; a payload that is short or names no
// such port is dropped in silence, and none of these carries a receipt.
//
// The two device commands are the two resets a device can be asked for, and
// they differ in where it ends up: MUX_CMD_REQUEST_BOOTLOADER is CAN's
// CANBUS_CMD_REQUEST_BOOTLOADER by value and meaning (generic/canserial.c) and
// leaves the device in its bootloader, for a flashing tool to write it;
// MUX_CMD_RESTART_DEVICE restarts the firmware itself (mcu_restart(), the
// body of the `reset` command) and is what names a device that should come back
// up. The port-level commands are addressed to a bridge, not to a device.
#define MUX_CMD_REQUEST_BOOTLOADER 0x02
#define MUX_CMD_SET_PORT_NODEID 0x03
#define MUX_CMD_RESET_PORT 0x04
#define MUX_CMD_RESTART_DEVICE 0x05
#define MUX_CMD_SET_PORT_TUNNEL 0x06

// SET_PORT_NODEID's payload is four bytes: [0x03][index][id][flags]. `index` is
// a 0-based port index (a port the port table does not list is dropped), `id`
// is the node id of the device on that port (an id the wire cannot address,
// mux_arb_id() answering 0 for it, is dropped), and `flags` says how that
// port's device speaks - the reserved bits are ignored. The command builds the
// bridge's own tables and nothing else: no receipt, no copy downstream, and a
// repeat simply writes the same values again.
#define MUX_SET_PORT_NODEID_LEN 4

// SET_PORT_TUNNEL's payload is two bytes: [0x06][index]. The command toggles
// port `index`'s flashing tunnel mode (bridge.c): the same command both enters
// and leaves it, so a repeat simply flips back. In tunnel mode the port's
// downstream units lose their headers (the raw katapult blocks go down whole)
// and the bytes coming up are reassembled into tunnel units under the port's
// reply id - the id its SET_PORT_NODEID named it with, whichever flags that
// command gave it: the mode is about the line, not about what the device on it
// speaks. Like SET_PORT_NODEID it is the bridge's own port state: consumed by
// the bridge, never forwarded, no receipt; a port the table does not list is
// dropped in silence.
#define MUX_SET_PORT_TUNNEL_LEN 2

// A port's flags, from SET_PORT_NODEID: whether the device on it speaks the
// unit format. Set, the port forwards units both ways unchanged; clear, it
// strips the header off a unit going down and heads a raw frame coming up with
// the reply id it was given - the wire format of a device that predates the
// mux. Whether a port's device speaks units is the host's call, not a build
// time one: nothing here is compiled in per port. The flags say nothing about
// the port's flashing tunnel mode (MUX_CMD_SET_PORT_TUNNEL), which is a
// separate bit of port state and legal on either kind of port.
#define MUX_PORT_UNITS 0x01

// The top bit of the byte after an answer's uuid: set, that byte is the head of
// the answer's port path (its low bits are how many port numbers follow) rather
// than the application byte, which never has the bit set.
#define MUX_PATH_FLAG 0x80

// How many port numbers an answer's path may carry. A device is at most this
// many bridges deep in any real wiring, and the bound is what keeps the path a
// fixed part of a frame: a path already this long is dropped rather than grown
// (mux_need_nodeid_via), so a device behind that many bridges is reported no
// deeper than the bridge nearest the host.
#define MUX_PATH_MAX 8

// A uuid is six bytes, most significant first.
#define MUX_UUID_LEN 6

// The two bytes a unit's header occupies.
#define MUX_HEADER_BYTES 2

// The tunnel unit's frame: one katapult message block carried whole inside one
// mux unit - the byte-for-byte contract host `serial.rs` ("The tunnel unit")
// states and this side mirrors. The tag sits at the frame position (the bytes
// after the header) and can never collide with a klipper frame: a klipper
// frame's first byte is its length (MESSAGE_MIN..MESSAGE_MAX) and neither
// 0x01 nor 0x88 is in that range. The unit layer checks only the frame's shape
// - never the inner CRC and never the command byte; those belong to the
// consumer of the block, not to the tunnel.
#define MUX_TUNNEL_TAG0 0x01             // katapult MESSAGE_STX1 / MESSAGE_STX2
#define MUX_TUNNEL_TAG1 0x88
#define MUX_TUNNEL_POS_WORDCNT 3         // katapult MESSAGE_POS_LEN (after cmd)
#define MUX_TUNNEL_FRAME_MIN 8           // katapult MESSAGE_MIN / MESSAGE_MAX
#define MUX_TUNNEL_FRAME_MAX 128

// One tunnel frame's length for its word count - katapult's own rule - or 0
// when the count is past the limit (31 would make 132 bytes).
size_t mux_tunnel_frame_len(uint8_t word_cnt);

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
uint_fast8_t mux_need_nodeid_via(uint8_t *buf, uint_fast8_t buf_size
                                 , const uint8_t *args, uint_fast8_t args_len
                                 , uint8_t port);
uint_fast8_t mux_assignment_reply(uint8_t *buf, uint_fast8_t buf_size
                                  , uint32_t nodeid);

// The device half: who this device is inside the mux, and how a stream reaches
// it. The identity is the six bytes the host assigns (its own device behind a
// bridge) or the micro-controller derives from its chip id; the node id starts
// at 0 ("no address yet") and is adopted from the assignment.
void mux_set_uuid(const uint8_t *uuid);
const uint8_t *mux_uuid(void);
uint32_t mux_node_id(void);

// The header our own frames go out with: 0 bytes while the device has no node
// id (raw frames, exactly the old wire format), else the reply header naming us.
// Whoever writes a frame upward calls this first.
uint_fast8_t mux_out_header(uint8_t *buf);

// Resolve the units at the front of an incoming byte buffer, in place.
//
// This is the device's whole input path: whatever carries the stream (the host
// library's console task in src/host/console.c, the serial consoles in
// src/generic/serial_irq.c and src/linux/console.c, the USB CDC console in
// src/generic/usb_cdc.c, the byte stream CAN's nodes reassemble in
// src/generic/canserial.c) hands its bytes here before the
// ordinary frame parser reads them. Admin units are
// answered or consumed; one of our own frames is left at the front with its
// header stripped, which is the frame the parser then reads. Raw frames - a
// path with one device on it - are left alone, which is what keeps the old wire
// format working.
//
// Returns 1 when the front is an ordinary frame the caller's parser should look
// at, 0 when there is nothing to parse: the front is a unit that was handled, or
// one that has not arrived whole yet. In the second case the caller must not
// parse - a unit's leading bytes are not a frame, and reading them as one would
// lose the unit.
int mux_input(uint8_t *buf, size_t *len);

// Bytes going upward: implemented by whoever carries the stream
// (host/console.c, generic/serial_irq.c, generic/canserial.c, generic/usb_cdc.c,
// linux/console.c),
// because only it knows where the transmit buffer is. A frame of our own needs
// no hook - the input side leaves it at the front of the caller's buffer,
// header stripped.
void mux_send(const uint8_t *bytes, uint_fast8_t len);

#endif // mux.h
