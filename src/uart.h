#ifndef __UART_H
#define __UART_H

#include <stdint.h> // uint32_t

// A byte-stream UART object (CONFIG_UART): the oid a host creates at
// configuration time with config_uart, and the line rate it asked for.
//
// The object is the transport-neutral half - the oid, the rate, and the seam
// below. Which byte stream a build can offer is the arch's (CONFIG_SERIAL_UART):
// the host library's backend is src/host/bridge.c, which opens the tty
// CONFIG_HOST_BRIDGE_SERIAL_PATH names and polls it from the task loop; a
// micro-controller arch would wire its serial task here instead.
//
// The rate is a configuration fact, not a build-time one: config_uart carries
// it, so a rate the build cannot offer, a stream it cannot open, or a second
// object it cannot serve is reported while the host is configuring - not left
// behind as a dead port later.

struct uart_s;

// How opening the byte stream behind a UART object went.
enum uart_open_result {
    UART_OPEN_OK = 0,
    UART_OPEN_NO_DEVICE,   // the byte stream could not be opened at all
    UART_OPEN_BAD_BAUD,    // opened, but the rate could not be set on it
    UART_OPEN_BUSY,        // this arch's byte stream already carries a UART
};

// Open the byte stream behind 'u' at 'baud'. Called once per object, when the
// host configures it. The result names why a failure failed; the caller reports
// it (shutdown()) with a message of its own.
enum uart_open_result uart_backend_open(struct uart_s *u, uint32_t baud);

#if CONFIG_BRIDGE
// The router's ops for 'u' as a downstream port: write() queues a whole blob on
// the object's stream and reset() pulses whatever lines reset the device behind
// it. 'ctx' is the object, so each port's bytes go to its own stream.
const struct bridge_port_ops *uart_backend_port_ops(struct uart_s *u);

// The bridge took 'u' as a downstream port with this index. A backend that
// polls its own stream (the host library) records it, so a byte it read can be
// fed to bridge_port_input() under the port it belongs to.
void uart_backend_set_port(struct uart_s *u, int index);
#endif

#endif // __UART_H
