// STM32 bridge port dispatcher: the seam port.h describes, answered per backend
//
// A bridge build's ports are not all drawn from the same kind of wire. This arch
// hands USARTs to the bridge (bridge_serial.c, CONFIG_BRIDGE_PORT_SERIAL), and
// CAN interfaces follow (bridge_can.c, CONFIG_BRIDGE_PORT_CAN). src/port.c names
// one seam for all of them - port_backend_open(), port_backend_port_ops(),
// port_backend_set_port() (port.h) - so this file is where that seam is
// answered, and this file alone defines the three symbols: each call names a
// port slot, and the slot decides which backend serves it.
//
// The slot numbering is the contract the host reads back. The UART slots come
// first - 0..bridge_serial_slot_count()-1, in bridge_serial.c's table order -
// and the other backends' slots follow, each numbered from zero among its own
// kind. So a slot is a port, not a USART, and BRIDGE_UART_COUNT is what the host
// lists the ports with before it declares any (klipperx's `admin bridge
// config`): the number of port slots this build can give the bridge - the UART
// slots plus the others, not a count of USARTs. The name is the one the host has
// always read, so it stays.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_BRIDGE, CONFIG_BRIDGE_PORT_CAN
#include "bridge_port.h" // bridge_serial_slot_count, bridge_serial_open
#include "command.h" // DECL_CONSTANT
#include "port.h" // port_backend_*, enum port_open_result

// The CAN backend's half of the seam (bridge_can.c, CONFIG_BRIDGE_PORT_CAN):
// generic/bridge_can.h declares what this file calls - opening its slot, the
// ops of the object that opened it, and the port index it feeds bytes under.
// One CAN controller is one slot: the first slot left over after the UART slots
// is CAN slot 0, and this build's CAN backend offers no other.
#if CONFIG_BRIDGE_PORT_CAN
#include "generic/bridge_can.h" // bridge_can_open, bridge_can_owns
#define BRIDGE_CAN_SLOT_COUNT 1
#else
#define BRIDGE_CAN_SLOT_COUNT 0
#endif

// Every port slot this build can give the bridge. The UART part is a
// preprocessor sum of the options bridge_serial.c's table is built from
// (bridge_port.h), not bridge_serial_slot_count(): the value goes into a
// compile_time_request, which only folds constant expressions.
#define BRIDGE_UART_COUNT (BRIDGE_SERIAL_SLOT_COUNT + BRIDGE_CAN_SLOT_COUNT)
DECL_CONSTANT("BRIDGE_UART_COUNT", BRIDGE_UART_COUNT);

// Open the port behind 'slot'. Called once per port object, from config_uart
// (src/port.c), so a slot this build does not offer, or a rate the backend
// cannot set, fails the host's configuration with its own message instead of
// leaving a port that never speaks.
enum port_open_result
port_backend_open(struct port_s *p, uint32_t rate, int slot)
{
    int can_slot = slot;
#if CONFIG_BRIDGE_PORT_SERIAL
    // The UART slots first, in the order bridge_serial.c numbers them.
    if (slot >= 0 && slot < bridge_serial_slot_count())
        return bridge_serial_open(p, rate, slot);
    // Then the CAN slots, counted from zero after the UART slots.
    can_slot = slot - bridge_serial_slot_count();
#endif
#if CONFIG_BRIDGE_PORT_CAN
    if (can_slot == 0)
        return bridge_can_open(p, rate);
#else
    (void)can_slot;
#endif
    return PORT_OPEN_NO_DEVICE;
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
