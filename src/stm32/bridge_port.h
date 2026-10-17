#ifndef __STM32_BRIDGE_PORT_H
#define __STM32_BRIDGE_PORT_H

#include <stdint.h> // uint32_t
#include "bridge.h" // struct bridge_port_ops
#include "uart.h" // struct uart_s, enum uart_open_result

// The seam src/uart.c names (uart.h) belongs to the dispatcher: bridge_port.c
// defines uart_backend_open(), uart_backend_port_ops() and
// uart_backend_set_port(), and no backend does. A backend offers its ports
// through the calls below, and the dispatcher decides which backend a slot
// belongs to - the USART slots first, 0..bridge_serial_slot_count()-1, in the
// order the backend's own table numbers them.

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

// Open the USART behind 'slot' at 'baud'.
enum uart_open_result bridge_serial_open(struct uart_s *u, uint32_t baud,
                                         int slot);

// The router's ops for 'u' as a downstream port.
const struct bridge_port_ops *bridge_serial_port_ops(struct uart_s *u);

// The bridge took 'u' as a downstream port with this index.
void bridge_serial_set_port(struct uart_s *u, int index);

#endif // __STM32_BRIDGE_PORT_H
