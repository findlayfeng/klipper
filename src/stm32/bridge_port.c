// STM32 bridge port dispatcher: the seam port.h describes, answered per backend
//
// A bridge build's ports are not all drawn from the same kind of wire. This arch
// hands USARTs to the bridge (bridge_serial.c, CONFIG_BRIDGE_PORT_SERIAL), and
// CAN interfaces follow (bridge_can.c, CONFIG_BRIDGE_PORT_CAN). src/port.c names
// one seam for all of them - port_backend_open(), port_backend_can_open(),
// port_backend_port_ops(), port_backend_set_port() (port.h) - so this file is
// where that seam is answered, and this file alone defines those symbols.
//
// Only the UART call names a port slot, and only USARTs are numbered by one:
// the UART slots are 0..bridge_serial_slot_count()-1, in bridge_serial.c's table
// order, and a slot outside that range names no wire. A CAN port names no slot
// at all - one CAN controller is all this build gives the bridge - so it comes
// in through port_backend_can_open() and the slot the host still sends is
// ignored. BRIDGE_UART_COUNT stays the number of port slots the host lists the
// ports with before it declares any (klipperx's `admin bridge
// config`): the UART slots plus the one CAN port, not a count of USARTs. The
// name is the one the host has always read, so it stays.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_BRIDGE, CONFIG_BRIDGE_PORT_CAN
#include "bridge_port.h" // bridge_serial_slot_count, bridge_serial_open
#include "command.h" // DECL_CONSTANT
#include "port.h" // port_backend_*, enum port_open_result

// The CAN backend's half of the seam (bridge_can.c, CONFIG_BRIDGE_PORT_CAN):
// generic/bridge_can.h declares what this file calls - opening the port, the
// ops of the object that opened it, and the port index it feeds bytes under.
// One CAN controller is one port and carries no slot number.
#if CONFIG_BRIDGE_PORT_CAN
#include "generic/bridge_can.h" // bridge_can_open, bridge_can_owns
#endif

// Every port this build can give the bridge. The UART part is a preprocessor
// sum of the options bridge_serial.c's table is built from (bridge_port.h), not
// bridge_serial_slot_count(): the value goes into a compile_time_request, which
// only folds constant expressions. The CAN port is one more, without a slot.
#if CONFIG_BRIDGE_PORT_CAN
#define BRIDGE_CAN_PORT_COUNT 1
#else
#define BRIDGE_CAN_PORT_COUNT 0
#endif
#define BRIDGE_UART_COUNT (BRIDGE_SERIAL_SLOT_COUNT + BRIDGE_CAN_PORT_COUNT)
DECL_CONSTANT("BRIDGE_UART_COUNT", BRIDGE_UART_COUNT);

// Open the USART behind 'slot'. Called once per UART port object, from
// config_uart (src/port.c), so a slot this build does not offer, or a rate the
// backend cannot set, fails the host's configuration with its own message
// instead of leaving a port that never speaks. The slot is the UART one and
// nothing else: a slot no USART covers names no wire.
enum port_open_result
port_backend_open(struct port_s *p, uint32_t rate, int slot)
{
#if CONFIG_BRIDGE_PORT_SERIAL
    // The UART slots, in the order bridge_serial.c numbers them.
    if (slot >= 0 && slot < bridge_serial_slot_count())
        return bridge_serial_open(p, rate, slot);
#else
    (void)p;
    (void)rate;
    (void)slot;
#endif
    return PORT_OPEN_NO_DEVICE;
}

// Open this build's CAN controller for 'p'. Called once per CAN port object,
// from config_can (src/port.c), and there is nothing to pick: this build gives
// the bridge one CAN controller, so no slot is taken here and the one the host
// still sends on the wire is ignored.
enum port_open_result
port_backend_can_open(struct port_s *p, uint32_t bitrate)
{
#if CONFIG_BRIDGE_PORT_CAN
    return bridge_can_open(p, bitrate);
#else
    (void)p;
    (void)bitrate;
    return PORT_OPEN_NO_DEVICE;
#endif
}

#if CONFIG_BRIDGE
// The router's ops for the port 'p' opened. A build can carry more than one kind
// of port, so the object is asked which backend opened it (bridge_serial_owns /
// bridge_can_owns) and gets that backend's ops - an object no backend claims
// falls through to the USART one, the behaviour every build before the CAN port
// had.
const struct bridge_port_ops *
port_backend_port_ops(struct port_s *p)
{
#if CONFIG_BRIDGE_PORT_CAN
    if (bridge_can_owns(p))
        return bridge_can_port_ops(p);
#endif
#if CONFIG_BRIDGE_PORT_SERIAL
    return bridge_serial_port_ops(p);
#else
    // No USART backend in this build (a CAN-only port): every object that
    // opened a port is the CAN backend's, so its ops are the answer.
    return bridge_can_port_ops(p);
#endif
}

// The router took 'p' as the port with this index; the backend that opened it
// records it so the bytes it reads can be fed under that port.
void
port_backend_set_port(struct port_s *p, int index)
{
#if CONFIG_BRIDGE_PORT_CAN
    if (bridge_can_owns(p)) {
        bridge_can_set_port(p, index);
        return;
    }
#endif
#if CONFIG_BRIDGE_PORT_SERIAL
    bridge_serial_set_port(p, index);
#else
    (void)p;
    (void)index;
#endif
}
#endif // CONFIG_BRIDGE
