#ifndef __BRIDGE_CAN_H
#define __BRIDGE_CAN_H

#include <stdint.h> // uint32_t
#include "bridge.h" // struct bridge_port_ops
#include "port.h" // struct port_s, enum port_open_result

struct canbus_msg;

// A bridge build's CAN downstream port (generic/bridge_can.c): one CAN
// controller handed to the bridge the way a USART is (stm32/bridge_serial.c),
// so the devices wired to that bus are reached through the bridge's port
// commands. The seam itself is the dispatcher's (port.h, answered by
// stm32/bridge_port.c); this header is what the dispatcher and the CAN wrapper
// call, the way canserial.h is what canbus.c calls.

// Open this build's CAN controller for 'p' at 'bitrate' bits per second. The
// rate is the port's, not the build's (config_can carries it), and a rate this
// controller cannot be set to is refused with PORT_OPEN_BAD_BAUD. The one
// controller takes one object (a second is refused with PORT_OPEN_BUSY).
enum port_open_result bridge_can_open(struct port_s *p, uint32_t bitrate);

// Whether 'p' is the object that opened the CAN controller - how the dispatcher
// tells a CAN port from a USART one.
int bridge_can_owns(const struct port_s *p);

// The router's ops for 'p' as a downstream port.
const struct bridge_port_ops *bridge_can_port_ops(struct port_s *p);

// The bridge took 'p' as a downstream port with this index.
void bridge_can_set_port(struct port_s *p, int index);

// The CAN driver's two callbacks (generic/canbus.c): a frame heard on the bus,
// and the transmitter having room again. process_data() answers 1 when the frame
// belongs to this port (a device behind it), 0 for everything else on the bus.
int bridge_can_process_data(struct canbus_msg *msg);
void bridge_can_notify_tx(void);

#endif // __BRIDGE_CAN_H
