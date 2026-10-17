#ifndef __STM32_BRIDGE_PORT_H
#define __STM32_BRIDGE_PORT_H

#include <stdint.h> // uint32_t
#include "bridge.h" // struct bridge_port_ops
#include "port.h" // struct port_s, enum port_open_result

// The seam src/port.c names (port.h) belongs to the dispatcher: bridge_port.c
// defines port_backend_open(), port_backend_can_open(), port_backend_port_ops()
// and port_backend_set_port(), and no backend does. A backend offers its ports
// through the calls below, and the dispatcher decides which USART a slot
// belongs to - the USART slots are 0..bridge_serial_slot_count()-1, in the
// order the backend's own table numbers them. A CAN port names no slot;
// bridge_can.h (generic/bridge_can.h) is its half of the seam.

// How many UART slots this build's USART backend offers: the size of
// bridge_serial.c's table, which is written in slot order.
int bridge_serial_slot_count(void);

// The same count as a compile-time constant, because the dispatcher's
// DECL_CONSTANT("BRIDGE_UART_COUNT", ...) has to be one and a call is not: one
// per USART ticked under CONFIG_STM32_BRIDGE_SERIAL, which are the options
// bridge_serial.c's table is built from. This list and that table have to keep
// naming the same USARTs.
#define BRIDGE_SERIAL_SLOT_COUNT                                          \
    (CONFIG_STM32_BRIDGE_SERIAL_USART1 + CONFIG_STM32_BRIDGE_SERIAL_USART2 \
     + CONFIG_STM32_BRIDGE_SERIAL_USART3 + CONFIG_STM32_BRIDGE_SERIAL_UART4 \
     + CONFIG_STM32_BRIDGE_SERIAL_UART5)

// Open the USART behind 'slot' at 'rate'.
enum port_open_result bridge_serial_open(struct port_s *p, uint32_t rate,
                                         int slot);

// The router's ops for 'p' as a downstream port.
const struct bridge_port_ops *bridge_serial_port_ops(struct port_s *p);

// Whether 'p' is an object this backend opened. One build can carry more than
// one kind of port, so the dispatcher (bridge_port.c) asks each backend which
// objects are its own before answering port.h's hooks.
int bridge_serial_owns(const struct port_s *p);

// The bridge took 'p' as a downstream port with this index.
void bridge_serial_set_port(struct port_s *p, int index);

#endif // __STM32_BRIDGE_PORT_H
