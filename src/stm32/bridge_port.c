// STM32 bridge port dispatcher: the seam uart.h describes, answered per backend
//
// A bridge build's ports are not all drawn from the same kind of wire. This arch
// hands USARTs to the bridge (bridge_serial.c, CONFIG_BRIDGE_PORT_SERIAL), and
// CAN interfaces follow (bridge_can.c, CONFIG_BRIDGE_PORT_CAN). src/uart.c names
// one seam for all of them - uart_backend_open(), uart_backend_port_ops(),
// uart_backend_set_port() (uart.h) - so this file is where that seam is
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
#include "uart.h" // uart_backend_*, enum uart_open_result

// The CAN backend's half of the seam (bridge_can.c, CONFIG_BRIDGE_PORT_CAN):
// opening its slot. That file is a later unit's - there is no CAN port backend
// in this build, so nothing here may define these, and every build today
// compiles the block out. What the block says is the numbering contract above
// applied to one CAN slot: the first slot left over after the UART slots is CAN
// slot 0, and this build's CAN backend offers no other.
#if CONFIG_BRIDGE_PORT_CAN
enum uart_open_result bridge_can_open(struct uart_s *u, uint32_t baud);
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

// Open the port behind 'slot'. Called once per UART object, from config_uart
// (src/uart.c), so a slot this build does not offer, or a rate the backend
// cannot set, fails the host's configuration with its own message instead of
// leaving a port that never speaks.
enum uart_open_result
uart_backend_open(struct uart_s *u, uint32_t baud, int slot)
{
    // The UART slots first, in the order bridge_serial.c numbers them.
    if (slot >= 0 && slot < bridge_serial_slot_count())
        return bridge_serial_open(u, baud, slot);
#if CONFIG_BRIDGE_PORT_CAN
    // Then the CAN slots, counted from zero after the UART slots.
    if (slot - bridge_serial_slot_count() == 0)
        return bridge_can_open(u, baud);
#endif
    return UART_OPEN_NO_DEVICE;
}

#if CONFIG_BRIDGE
// The router's ops for the port 'u' opened. Every port a build today can hand
// the bridge is a UART slot (the CAN block above is compiled out), so both hooks
// are the USART backend's; a build that adds a backend whose objects need their
// own ops has to ask the object which backend opened it, here and below.
const struct bridge_port_ops *
uart_backend_port_ops(struct uart_s *u)
{
    return bridge_serial_port_ops(u);
}

// The router took 'u' as the port with this index; the backend records it so the
// bytes it reads can be fed under that port.
void
uart_backend_set_port(struct uart_s *u, int index)
{
    bridge_serial_set_port(u, index);
}
#endif // CONFIG_BRIDGE
