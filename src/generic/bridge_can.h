#ifndef __BRIDGE_CAN_H
#define __BRIDGE_CAN_H

#include <stdint.h> // uint32_t
#include "bridge.h" // struct bridge_port_ops
#include "uart.h" // struct uart_s, enum uart_open_result

struct canbus_msg;

// A bridge build's CAN downstream port (generic/bridge_can.c): one CAN
// controller handed to the bridge the way a USART is (stm32/bridge_serial.c),
// so the devices wired to that bus are reached through the bridge's port
// commands. The seam itself is the dispatcher's (uart.h, answered by
// stm32/bridge_port.c); this header is what the dispatcher and the CAN wrapper
// call, the way canserial.h is what canbus.c calls.

// Open this build's CAN controller for 'u'. The rate is CONFIG_CANBUS_FREQUENCY,
// so 'baud' is a dummy parameter and ignored, and the one controller takes one
// object (a second is refused with UART_OPEN_BUSY).
enum uart_open_result bridge_can_open(struct uart_s *u, uint32_t baud);

// Whether 'u' is the object that opened the CAN controller - how the dispatcher
// tells a CAN port from a USART one.
int bridge_can_owns(const struct uart_s *u);

// The router's ops for 'u' as a downstream port.
const struct bridge_port_ops *bridge_can_port_ops(struct uart_s *u);

// The bridge took 'u' as a downstream port with this index.
void bridge_can_set_port(struct uart_s *u, int index);

// The CAN driver's two callbacks (generic/canbus.c): a frame heard on the bus,
// and the transmitter having room again. process_data() answers 1 when the frame
// belongs to this port (a device behind it), 0 for everything else on the bus.
int bridge_can_process_data(struct canbus_msg *msg);
void bridge_can_notify_tx(void);

#endif // __BRIDGE_CAN_H
