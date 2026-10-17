// Generic handling of serial over CAN support
//
// The bytes this node reassembles out of the CAN frames on its own arbitration
// id are a byte stream like the ones the serial consoles own, and with MUX_CAN
// they carry units (mux.h): a bridge reached over CAN relays several downstream
// devices through this one byte stream, and a frame with no unit header says
// nothing about which of them sent it. The console hooks below wire that in the
// same places src/generic/serial_irq.c does - the receive side resolves units
// before the frame parser reads the buffer, and the transmit side heads this
// node's own frames and implements mux_send.
//
// Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
// Copyright (C) 2020 Pontus Borg <glpontus@gmail.com>
// Copyright (C) 2021-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_HAVE_BOOTLOADER_REQUEST
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // console_sendf
#include "canbus.h" // canbus_send
#include "canserial.h" // canserial_notify_tx
#include "chipid_uuid.h" // chipid_uuid
#include "command.h" // DECL_CONSTANT
#if CONFIG_MUX_CAN
#include "mux.h" // mux_input, mux_send, mux_out_header, MUX_HEADER_BYTES
#endif
#include "sched.h" // sched_wake_task

#define CANBUS_UUID_LEN 6

// Global storage
static struct canbus_data {
    uint32_t assigned_id;
    uint8_t uuid[CANBUS_UUID_LEN];

    // Tx data
    struct task_wake tx_wake;
    uint8_t transmit_pos, transmit_max;

    // Rx data
    struct task_wake rx_wake;
    uint8_t receive_pos;
    uint32_t admin_pull_pos, admin_push_pos;

    // Transfer buffers
    struct canbus_msg admin_queue[8];
    uint8_t transmit_buf[192];
    uint8_t receive_buf[192];
} CanData;


/****************************************************************
 * Data transmission over CAN
 ****************************************************************/

void
canserial_notify_tx(void)
{
    sched_wake_task(&CanData.tx_wake);
}

void
canserial_tx_task(void)
{
    if (!sched_check_wake(&CanData.tx_wake))
        return;
    uint32_t id = CanData.assigned_id;
    if (!id) {
        CanData.transmit_pos = CanData.transmit_max = 0;
        return;
    }
    struct canbus_msg msg;
    msg.id = id + 1;
    uint32_t tpos = CanData.transmit_pos, tmax = CanData.transmit_max;
    for (;;) {
        int avail = tmax - tpos, now = avail > 8 ? 8 : avail;
        if (avail <= 0)
            break;
        msg.dlc = now;
        memcpy(msg.data, &CanData.transmit_buf[tpos], now);
        int ret = canbus_send(&msg);
        if (ret <= 0)
            break;
        tpos += now;
    }
    CanData.transmit_pos = tpos;
}
DECL_TASK(canserial_tx_task);

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
#if CONFIG_MUX_CAN
    // In mux mode this node's own frames go out as units, the reply header
    // naming it the way the incoming unit named it. mux_out_header writes
    // nothing while the node has no id of its own, which keeps the old wire
    // format: bare frames, the kind a path with one device on it carries.
    uint8_t header[MUX_HEADER_BYTES];
    uint_fast8_t offset = mux_out_header(header);
#else
    uint_fast8_t offset = 0;
#endif

    // Verify space for message
    uint32_t tpos = CanData.transmit_pos, tmax = CanData.transmit_max;
    if (tpos >= tmax)
        CanData.transmit_pos = CanData.transmit_max = tpos = tmax = 0;
    if (tmax + ce->max_size + offset > sizeof(CanData.transmit_buf)) {
        if (tmax - tpos + ce->min_size + offset
            > sizeof(CanData.transmit_buf))
            // Not enough space for message
            return;
        // Move buffer
        tmax -= tpos;
        memmove(&CanData.transmit_buf[0], &CanData.transmit_buf[tpos], tmax);
        CanData.transmit_pos = tpos = 0;
        CanData.transmit_max = tmax;
    }
#if CONFIG_MUX_CAN
    memcpy(&CanData.transmit_buf[tmax], header, offset);
#endif

    // Generate message, behind the header this node's frames go out with
    uint32_t msglen = command_encode_and_frame(
        &CanData.transmit_buf[tmax + offset]
        , sizeof(CanData.transmit_buf) - tmax - offset, ce, args);
    if (!msglen)
        return;

    // Start message transmit
    CanData.transmit_max = tmax + offset + msglen;
    canserial_notify_tx();
}

#if CONFIG_MUX_CAN
// Bytes the mux sends upward: its answers on the admin channel and the units a
// bridge forwards, whole units already (mux.h mux_send). They share the one
// transmit buffer the responses drain, and are queued whole so that units never
// interleave on the wire.
//
// A unit is queued or dropped, never cut short: a unit the peer cannot resolve
// is worse than one that never went out. Command frames are resent when they go
// unanswered, so dropping one costs a retry; the admin/handshake units are
// fire-and-forget (the host scan is a single broadcast with a deadline, and the
// port-level commands have no reply either), so those are dropped outright - the
// alternative would be blocking the scheduler on a host that is not reading.
// So the room is verified exactly the way console_sendf
// verifies it - compact the already-sent prefix of the buffer, or report "not
// enough space" when even the message alone does not fit - and nothing here
// blocks: the bytes are queued in the buffer the tx task drains, which is how
// the responses above go out too.
void
mux_send(const uint8_t *bytes, uint_fast8_t len)
{
    uint32_t tpos = CanData.transmit_pos, tmax = CanData.transmit_max;
    if (tpos >= tmax)
        CanData.transmit_pos = CanData.transmit_max = tpos = tmax = 0;
    if (tmax + len > sizeof(CanData.transmit_buf)) {
        if (tmax - tpos + len > sizeof(CanData.transmit_buf))
            // Not enough space for message
            return;
        // Move buffer
        tmax -= tpos;
        memmove(&CanData.transmit_buf[0], &CanData.transmit_buf[tpos], tmax);
        CanData.transmit_pos = tpos = 0;
        CanData.transmit_max = tmax;
    }
    memcpy(&CanData.transmit_buf[tmax], bytes, len);
    CanData.transmit_max = tmax + len;
    canserial_notify_tx();
}
#endif // CONFIG_MUX_CAN


/****************************************************************
 * CAN "admin" command handling
 ****************************************************************/

// Available commands and responses
#define CANBUS_CMD_QUERY_UNASSIGNED 0x00
#define CANBUS_CMD_SET_KLIPPER_NODEID 0x01
#define CANBUS_CMD_REQUEST_BOOTLOADER 0x02
#define CANBUS_RESP_NEED_NODEID 0x20

// Helper to verify a UUID in a command matches this chip's UUID
static int
can_check_uuid(struct canbus_msg *msg)
{
    return (msg->dlc >= 7
            && memcmp(&msg->data[1], CanData.uuid, sizeof(CanData.uuid)) == 0);
}

// Helpers to encode/decode a CAN identifier to a 1-byte "nodeid"
static int
can_get_nodeid(void)
{
    if (!CanData.assigned_id)
        return 0;
    return (CanData.assigned_id - 0x100) >> 1;
}
static uint32_t
can_decode_nodeid(int nodeid)
{
    return (nodeid << 1) + 0x100;
}

static void
can_process_query_unassigned(struct canbus_msg *msg)
{
    if (CanData.assigned_id)
        return;
    struct canbus_msg send;
    send.id = CANBUS_ID_ADMIN_RESP;
    send.dlc = 8;
    send.data[0] = CANBUS_RESP_NEED_NODEID;
    memcpy(&send.data[1], CanData.uuid, sizeof(CanData.uuid));
    send.data[7] = CANBUS_CMD_SET_KLIPPER_NODEID;
    // Send with retry
    for (;;) {
        int ret = canbus_send(&send);
        if (ret >= 0)
            return;
    }
}

static void
can_id_conflict(void)
{
    CanData.assigned_id = 0;
    canbus_set_filter(CanData.assigned_id);
    shutdown("Another CAN node assigned this ID");
}

static void
can_process_set_klipper_nodeid(struct canbus_msg *msg)
{
    if (msg->dlc < 8)
        return;
    uint32_t newid = can_decode_nodeid(msg->data[7]);
    if (can_check_uuid(msg)) {
        if (newid != CanData.assigned_id) {
            CanData.assigned_id = newid;
            canbus_set_filter(CanData.assigned_id);
#if CONFIG_MUX_CAN
            // The mux's device half is what parses the units above this stream
            // and what heads this device's own frames, so it needs the id too:
            // on CAN the assignment arrives as the bus's own admin frame, which
            // this function is the only taker of.
            mux_set_assigned(newid);
#endif
        }
    } else if (newid == CanData.assigned_id) {
        can_id_conflict();
    }
}

static void
can_process_request_bootloader(struct canbus_msg *msg)
{
    if (!CONFIG_HAVE_BOOTLOADER_REQUEST || !can_check_uuid(msg))
        return;
    bootloader_request();
}

// Handle an "admin" command
static void
can_process_admin(struct canbus_msg *msg)
{
    if (!msg->dlc)
        return;
    switch (msg->data[0]) {
    case CANBUS_CMD_QUERY_UNASSIGNED:
        can_process_query_unassigned(msg);
        break;
    case CANBUS_CMD_SET_KLIPPER_NODEID:
        can_process_set_klipper_nodeid(msg);
        break;
    case CANBUS_CMD_REQUEST_BOOTLOADER:
        can_process_request_bootloader(msg);
        break;
    }
}


/****************************************************************
 * CAN packet reading
 ****************************************************************/

static void
canserial_notify_rx(void)
{
    sched_wake_task(&CanData.rx_wake);
}

DECL_CONSTANT("RECEIVE_WINDOW", ARRAY_SIZE(CanData.receive_buf));

// Handle incoming data (called from IRQ handler)
void
canserial_process_data(struct canbus_msg *msg)
{
    uint32_t id = msg->id;
    if (CanData.assigned_id && id == CanData.assigned_id) {
        // Add to incoming data buffer
        int rpos = CanData.receive_pos;
        uint32_t len = CANMSG_DATA_LEN(msg);
        if (len > sizeof(CanData.receive_buf) - rpos)
            return;
        memcpy(&CanData.receive_buf[rpos], msg->data, len);
        CanData.receive_pos = rpos + len;
        canserial_notify_rx();
    } else if (id == CANBUS_ID_ADMIN
               || (CanData.assigned_id && id == CanData.assigned_id + 1)) {
        // Add to admin command queue
        uint32_t pushp = CanData.admin_push_pos;
        if (pushp >= CanData.admin_pull_pos + ARRAY_SIZE(CanData.admin_queue))
            // No space - drop message
            return;
        uint32_t pos = pushp % ARRAY_SIZE(CanData.admin_queue);
        memcpy(&CanData.admin_queue[pos], msg, sizeof(*msg));
        CanData.admin_push_pos = pushp + 1;
        canserial_notify_rx();
    }
}

// Remove from the receive buffer the given number of bytes
static void
console_pop_input(int len)
{
    int copied = 0;
    for (;;) {
        int rpos = readb(&CanData.receive_pos);
        int needcopy = rpos - len;
        if (needcopy) {
            memmove(&CanData.receive_buf[copied]
                    , &CanData.receive_buf[copied + len], needcopy - copied);
            copied = needcopy;
            canserial_notify_rx();
        }
        irqstatus_t flag = irq_save();
        if (rpos != readb(&CanData.receive_pos)) {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        CanData.receive_pos = needcopy;
        irq_restore(flag);
        break;
    }
}

// Task to process incoming commands and admin messages
void
canserial_rx_task(void)
{
    if (!sched_check_wake(&CanData.rx_wake))
        return;

    // Process pending admin messages
    for (;;) {
        uint32_t pushp = readl(&CanData.admin_push_pos);
        uint32_t pullp = CanData.admin_pull_pos;
        if (pushp == pullp)
            break;
        uint32_t pos = pullp % ARRAY_SIZE(CanData.admin_queue);
        struct canbus_msg *msg = &CanData.admin_queue[pos];
        uint32_t id = msg->id;
        if (CanData.assigned_id && id == CanData.assigned_id + 1)
            can_id_conflict();
        else if (id == CANBUS_ID_ADMIN)
            can_process_admin(msg);
        CanData.admin_pull_pos = pullp + 1;
    }

    // Check for a complete message block and process it
#if CONFIG_MUX_CAN
    // Resolve the units at the front before the frame parser reads the buffer,
    // exactly as the serial consoles do (generic/serial_irq.c). mux_input
    // answers the mux's admin channel there - the scan reply, adopting the node
    // id it is handed, the id-collision shutdown - and leaves one of our own
    // frames at the front with its header stripped, which is what the parser
    // then reads. A raw frame - a peer that has not been told to speak units -
    // is left alone, so the byte stream that predates the mux still works.
    //
    // The CAN rx interrupt appends at receive_pos, so it must not run while
    // mux_input rewrites the buffer; the position is read, and set back to the
    // mux's count, inside that section. The section is one memmove over at
    // most sizeof(CanData.receive_buf) bytes - the same one console_pop_input
    // holds - and mux_input writes only below the position the interrupt
    // appends at. Should it shut down (an id collision), the shutdown's own
    // longjmp leaves the interrupts disabled, which is what run_shutdown()
    // expects.
    //
    // The parser reads exactly the mux's count, never the position we started
    // from: mux_input consumes the units it resolves (and the junk it drops)
    // off the front, so that count is the length of what may be parsed.
    irqstatus_t flag = irq_save();
    uint_fast8_t rpos = readb(&CanData.receive_pos);
    size_t count = rpos;
    int ready = mux_input(CanData.receive_buf, &count);
    CanData.receive_pos = count;
    rpos = count;
    irq_restore(flag);
    if (!ready)
        // Nothing the parser may read: the front is a handled unit (already
        // dropped), a unit still arriving whole, or the buffer is empty.
        return;
#else
    uint_fast8_t rpos = readb(&CanData.receive_pos);
#endif
    uint_fast8_t pop_count;
    int ret = command_find_block(CanData.receive_buf, rpos, &pop_count);
    if (ret > 0)
        command_dispatch(CanData.receive_buf, pop_count);
    if (ret) {
        console_pop_input(pop_count);
        if (ret > 0)
            command_send_ack();
    }
}
DECL_TASK(canserial_rx_task);


/****************************************************************
 * Setup and shutdown
 ****************************************************************/

DECL_ENUMERATION("canbus_bus_state", "active", CANBUS_STATE_ACTIVE);
DECL_ENUMERATION("canbus_bus_state", "warn", CANBUS_STATE_WARN);
DECL_ENUMERATION("canbus_bus_state", "passive", CANBUS_STATE_PASSIVE);
DECL_ENUMERATION("canbus_bus_state", "off", CANBUS_STATE_OFF);

void
command_get_canbus_status(uint32_t *args)
{
    struct canbus_status status;
    memset(&status, 0, sizeof(status));
    canhw_get_status(&status);
    sendf("canbus_status rx_error=%u tx_error=%u tx_retries=%u"
          " canbus_bus_state=%u"
          , status.rx_error, status.tx_error, status.tx_retries
          , status.bus_state);
}
DECL_COMMAND_FLAGS(command_get_canbus_status, HF_IN_SHUTDOWN
                   , "get_canbus_status");

void
command_get_canbus_id(uint32_t *args)
{
    sendf("canbus_id canbus_uuid=%.*s canbus_nodeid=%u"
          , sizeof(CanData.uuid), CanData.uuid, can_get_nodeid());
}
DECL_COMMAND_FLAGS(command_get_canbus_id, HF_IN_SHUTDOWN, "get_canbus_id");

void
canserial_set_uuid(uint8_t *raw_uuid, uint32_t raw_uuid_len)
{
    chipid_uuid(CanData.uuid, raw_uuid, raw_uuid_len);
#if CONFIG_MUX_CAN
    // The mux identity is the same six bytes (generic/chipid_uuid.h), so this
    // node answers a mux scan with the uuid its canbus_uuid reports. This is
    // where the CAN carrier takes it from: the arch's chipid.c already calls
    // here in a build that carries a CAN node (CANSERIAL or USBCANBUS), on every
    // arch CAN can be built for, and chipid_init runs before the console comes
    // up.
    mux_set_uuid(CanData.uuid);
#endif
    canserial_notify_rx();
}

void
canserial_shutdown(void)
{
    canserial_notify_tx();
    canserial_notify_rx();
}
DECL_SHUTDOWN(canserial_shutdown);
