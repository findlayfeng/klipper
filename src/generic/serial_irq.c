// Generic interrupt based serial uart helper code
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy, memmove
#include "autoconf.h" // CONFIG_SERIAL_BAUD, CONFIG_MUX_SERIAL
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // console_sendf
#include "board/pgm.h" // READP
#include "command.h" // DECL_CONSTANT
#include "mux.h" // mux_input, mux_send, mux_out_header, MUX_HEADER_BYTES
#include "sched.h" // sched_wake_tasks
#include "serial_irq.h" // serial_enable_tx_irq

#define RX_BUFFER_SIZE 192

static uint8_t receive_buf[RX_BUFFER_SIZE], receive_pos;
static uint8_t transmit_buf[96], transmit_pos, transmit_max;

DECL_CONSTANT("SERIAL_BAUD", CONFIG_SERIAL_BAUD);
DECL_CONSTANT("RECEIVE_WINDOW", RX_BUFFER_SIZE);

// Rx interrupt - store read data
void
serial_rx_byte(uint_fast8_t data)
{
    if (data == MESSAGE_SYNC)
        sched_wake_tasks();
    if (receive_pos >= sizeof(receive_buf))
        // Serial overflow - ignore it as crc error will force retransmit
        return;
    receive_buf[receive_pos++] = data;
}

// Tx interrupt - get next byte to transmit
int
serial_get_tx_byte(uint8_t *pdata)
{
    if (transmit_pos >= transmit_max)
        return -1;
    *pdata = transmit_buf[transmit_pos++];
    return 0;
}

// Remove from the receive buffer the given number of bytes
static void
console_pop_input(uint_fast8_t len)
{
    uint_fast8_t copied = 0;
    for (;;) {
        uint_fast8_t rpos = readb(&receive_pos);
        uint_fast8_t needcopy = rpos - len;
        if (needcopy) {
            memmove(&receive_buf[copied], &receive_buf[copied + len]
                    , needcopy - copied);
            copied = needcopy;
            sched_wake_tasks();
        }
        irqstatus_t flag = irq_save();
        if (rpos != readb(&receive_pos)) {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        receive_pos = needcopy;
        irq_restore(flag);
        break;
    }
}

// Process any incoming commands
void
console_task(void)
{
    uint_fast8_t rpos, pop_count;
#if CONFIG_MUX_SERIAL
    // Resolve the units at the front before the frame parser reads the buffer,
    // exactly as the host library does (host/console.c). mux_input answers the
    // admin channel there - the scan reply, adopting the node id it is handed,
    // the id-collision shutdown - and leaves one of our own frames at the front
    // with its header stripped, which is what the parser then reads. A raw
    // frame (a path with one device on it) is left alone, so the old wire
    // format still works. A unit that has not arrived whole stops the parser
    // here: its leading bytes are not a frame.
    //
    // The receive interrupt appends at receive_pos, so it must not run while
    // mux_input rewrites the buffer; the position is read inside that section
    // too, or bytes appended in between would be stranded past the count taken
    // back out. The section is one memmove over at most RX_BUFFER_SIZE bytes -
    // the same one console_pop_input holds - and mux_input writes only below
    // the position the interrupt appends at. Should it shut down (an id
    // collision), the shutdown's own longjmp leaves the interrupts disabled,
    // which is what run_shutdown() expects.
    irqstatus_t flag = irq_save();
    rpos = readb(&receive_pos);
    size_t count = rpos;
    int ready = mux_input(receive_buf, &count);
    if (count != rpos)
        receive_pos = count;
    irq_restore(flag);
    if (!ready)
        // Nothing the parser may read: the front is a handled unit (already
        // dropped), a unit still arriving whole, or the buffer is empty.
        return;
#else
    rpos = readb(&receive_pos);
#endif
    int_fast8_t ret = command_find_block(receive_buf, rpos, &pop_count);
    if (ret > 0)
        command_dispatch(receive_buf, pop_count);
    if (ret) {
        if (CONFIG_HAVE_BOOTLOADER_REQUEST && ret < 0 && pop_count == 32
            && !memcmp(receive_buf, " \x1c Request Serial Bootloader!! ~", 32))
            bootloader_request();
        console_pop_input(pop_count);
        if (ret > 0)
            command_send_ack();
    }
}
DECL_TASK(console_task);

// Make room at the end of the transmit buffer for a message of
// min_len..max_len bytes and return the offset to write it at, or -1 when not
// even min_len fits. Compacting the already-sent prefix and reporting a full
// buffer are the console's own rules, shared by the responses below and by the
// mux's own units (mux_send): both drain through the one transmit buffer.
static int
transmit_room(uint_fast8_t max_len, uint_fast8_t min_len)
{
    uint_fast8_t tpos = readb(&transmit_pos), tmax = readb(&transmit_max);
    if (tpos >= tmax) {
        tpos = tmax = 0;
        writeb(&transmit_max, 0);
        writeb(&transmit_pos, 0);
    }
    if (tmax + max_len > sizeof(transmit_buf)) {
        if (tmax - tpos + min_len > sizeof(transmit_buf))
            // Not enough space for message
            return -1;
        // Disable TX irq and move buffer
        writeb(&transmit_max, 0);
        tpos = readb(&transmit_pos);
        tmax -= tpos;
        memmove(&transmit_buf[0], &transmit_buf[tpos], tmax);
        writeb(&transmit_pos, 0);
        writeb(&transmit_max, tmax);
        serial_enable_tx_irq();
    }
    return tmax;
}

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
#if CONFIG_MUX_SERIAL
    // In mux mode our own frames go out as units, the reply header naming us the
    // way the incoming unit named us. mux_out_header writes nothing while this
    // device has no node id, which keeps the old wire format: bare frames, the
    // kind a path with one device on it carries.
    uint8_t header[MUX_HEADER_BYTES];
    uint_fast8_t offset = mux_out_header(header);
#else
    uint_fast8_t offset = 0;
#endif

    // Verify space for message
    int tmax = transmit_room(READP(ce->max_size) + offset
                             , READP(ce->min_size) + offset);
    if (tmax < 0)
        return;
#if CONFIG_MUX_SERIAL
    memcpy(&transmit_buf[tmax], header, offset);
#endif

    // Generate message, behind the header our own frames go out with
    uint_fast8_t msglen = command_encode_and_frame(
        &transmit_buf[tmax + offset], sizeof(transmit_buf) - tmax - offset
        , ce, args);
    if (!msglen)
        return;

    // Start message transmit
    writeb(&transmit_max, tmax + offset + msglen);
    serial_enable_tx_irq();
}

#if CONFIG_MUX_SERIAL
// Bytes the mux sends upward: its answers on the admin channel and the units a
// bridge forwards, whole units already (mux.h mux_send). They share the one
// transmit buffer the responses drain, and are queued whole so that units never
// interleave on the wire.
void
mux_send(const uint8_t *bytes, uint_fast8_t len)
{
    int tmax = transmit_room(len, len);
    if (tmax < 0)
        return;
    memcpy(&transmit_buf[tmax], bytes, len);
    writeb(&transmit_max, tmax + len);
    serial_enable_tx_irq();
}
#endif // CONFIG_MUX_SERIAL
