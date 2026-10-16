// STM32 bridge port backend: a hardware USART handed to the bridge
//
// A bridge build has one console - here USB CDC, or the console USART - and
// downstream ports beside it; this file is the arch's byte stream for one of
// them: the UART a board wires to the device the host reaches through the
// bridge (the port-level commands, and the flashing tunnel a katapult
// bootloader is driven through). It fills the seam uart.h describes:
// uart_backend_open() brings the port USART up at the rate the host asks for in
// config_uart, uart_backend_port_ops() gives the router the hook that writes a
// whole blob to it, and uart_backend_set_port() records the router's port index
// so bytes read back can be fed to bridge_port_input() under that port.
//
// Which UART is a build-time fact (CONFIG_STM32_BRIDGE_SERIAL_USART1: USART1,
// PA10 RX / PA9 TX), and it is never the console's: one USART has one register
// block and one interrupt vector, so it cannot carry the console and a port at
// the same time. The Kconfig option excludes the console's USART1 choices.
//
// The rate is not a build-time fact: config_uart carries it, so the BRR is
// computed here while the host is configuring the object - never
// CONFIG_SERIAL_BAUD, which belongs to the console's fixed carrier.
//
// Receive is push, the way it is on every arch: the interrupt appends to a ring
// and a task drains it into bridge_port_input() (the router has no poll hook,
// and the host library's poll lives on irq_wait(), which is the host's alone).
// Transmit needs no task - a queued blob is drained by the USART's own TX
// interrupt.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy, memmove
#include "autoconf.h" // CONFIG_BRIDGE_PORT_SERIAL, CONFIG_STM32_BRIDGE_SERIAL_USART1
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/irq.h" // irq_save, irq_restore
#include "bridge.h" // bridge_port_input, struct bridge_port_ops
#include "command.h" // DECL_CONSTANT_STR, MESSAGE_MAX
#include "internal.h" // enable_pclock, get_pclock_frequency, gpio_peripheral, GPIO
#include "mux.h" // MUX_HEADER_BYTES, MUX_TUNNEL_FRAME_MAX
#include "sched.h" // DECL_TASK, sched_wake_tasks
#include "uart.h" // uart_backend_open, struct uart_s

// The UART this board hands to the bridge. The pins are reserved in the
// dictionary under their own name, so the host refuses a config that claims
// them for something else (klippy's mcu_identify reserves every RESERVE_PINS_*).
#if CONFIG_STM32_BRIDGE_SERIAL_USART1
  DECL_CONSTANT_STR("RESERVE_PINS_bridge_serial", "PA10,PA9");
  #define GPIO_Rx GPIO('A', 10)
  #define GPIO_Tx GPIO('A', 9)
  #define GPIO_AF_MODE 7
  #define USARTx USART1
  #define USARTx_IRQn USART1_IRQn
#else
  #error "bridge port serial: select CONFIG_STM32_BRIDGE_SERIAL_USART1"
#endif

// 8N1, receiver and transmitter on, receive interrupt on. The transmit
// interrupt is not here: it is turned on only while a blob is queued.
#define CR1_FLAGS (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE   \
                   | USART_CR1_RXNEIE)

// The receive and transmit buffers hold the largest things that can be in
// flight back to back - a whole tunnel block (MUX_TUNNEL_FRAME_MAX, 128 bytes,
// the bootloader stream a port can be toggled into) plus a klipper unit
// (MUX_HEADER_BYTES + MESSAGE_MAX, 66 bytes) - with room for a burst. A power
// of two, so the receive ring's indices can be masked into the buffer.
#define BUF_SIZE 256

// The receive ring: the interrupt appends at rx_head, the task consumes up to
// rx_tail, and the ring is full exactly when the two would meet, so it holds
// BUF_SIZE-1 bytes - still more than the largest pair above.
static uint8_t rx_buf[BUF_SIZE];
static volatile uint8_t rx_head, rx_tail;
// The bytes the ring had no room for: each one dropped above makes a frame
// the router cannot reassemble on its own side, which is a lost burst rather
// than a report, so the count is the only trace there is on a micro-controller
// (no stderr, and no host-visible channel in this backend). Kept as a
// diagnostic beside the ring it belongs to.
static uint32_t rx_dropped;

// The queued output: the router hands each unit over as one blob and this holds
// it until the USART can take it, so a later blob never cuts in front of a
// queued one. The interrupt sends from tx_pos up to tx_max; the task appends at
// tx_max, after dropping the already-sent prefix.
static uint8_t tx_buf[BUF_SIZE];
static volatile uint16_t tx_pos, tx_max;

// The UART object that opened the port (NULL until uart_backend_open), and the
// router's index for it (-1 until uart_backend_set_port). The index is what
// received bytes are fed under.
static struct uart_s *bridge_serial_owner;
static int bridge_serial_port = -1;

// Receive interrupt. Called directly from USARTx_IRQHandler below, where the
// interrupt's shared status register has already been read.
static void
bridge_serial_rx_byte(uint8_t data)
{
    uint8_t next = (rx_head + 1) & (BUF_SIZE - 1);
    if (next == rx_tail) {
        // Full: drop the newest byte rather than an older one, so what is
        // already queued stays a contiguous, in-order prefix. The byte is
        // gone, and the device's next frame fails its CRC (or the tunnel's own
        // retry notices) - the same trade the console's serial_rx_byte makes.
        rx_dropped++;
        return;
    }
    rx_buf[rx_head] = data;
    rx_head = next;
    // Wake the task on every byte, not only on MESSAGE_SYNC the way the
    // console's serial_rx_byte does: a port can carry a bootloader's katapult
    // stream, which has no klipper sync byte, and the bytes must reach the
    // router as they come - tasks are only dispatched when something wakes
    // them.
    sched_wake_tasks();
}

void
USARTx_IRQHandler(void)
{
    uint32_t sr = USARTx->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        // The ORE flag is automatically cleared by reading SR, followed
        // by reading DR.
        bridge_serial_rx_byte(USARTx->DR);
    }
    if (sr & USART_SR_TXE && USARTx->CR1 & USART_CR1_TXEIE) {
        uint16_t pos = tx_pos;
        if (pos != tx_max) {
            USARTx->DR = tx_buf[pos];
            tx_pos = pos + 1;
        } else {
            // Nothing queued: turn the transmit interrupt off until the next
            // blob, so an idle port costs nothing.
            USARTx->CR1 = CR1_FLAGS;
        }
    }
}

// Hand the bytes the interrupt received to the router. They cannot be fed from
// the interrupt itself (bridge_port_input buffers and scans a whole unit, and
// the router's work must not nest in an interrupt), so they wait in the ring
// for this task.
//
// The router runs with interrupts off, exactly as it does under the console's
// own consumer (generic/serial_irq.c's console_task holds an irq_save across
// mux_input), and at most DRAIN_CHUNK bytes are handed over per pass so the
// section stays short. The bytes are taken straight out of the ring - the
// interrupt only ever writes at or above rx_head, and the not-yet-advanced
// rx_tail is what it reads as full, so it never touches the run being scanned -
// which keeps this task's own frame small: the family's stack is 512 bytes, and
// the router's scan already brings its buffers into whatever frame inlines it.
#define DRAIN_CHUNK 64

void
bridge_serial_task(void)
{
    int port = bridge_serial_port;
    // The port has not been handed to the router yet: the bytes keep waiting in
    // the ring (config_bridge_port follows config_uart closely enough for that).
    if (port < 0)
        return;
    for (;;) {
        irqstatus_t flag = irq_save();
        uint8_t head = rx_head, tail = rx_tail;
        if (tail != head) {
            // The contiguous run: up to the ring's end (the bytes past it wait
            // for the next pass) and up to the chunk bound.
            size_t n = head > tail ? (size_t)(head - tail)
                                   : (size_t)(BUF_SIZE - tail);
            if (n > DRAIN_CHUNK)
                n = DRAIN_CHUNK;
            bridge_port_input(port, &rx_buf[tail], n);
            rx_tail = (tail + n) & (BUF_SIZE - 1);
        }
        irq_restore(flag);
        if (tail == head)
            return;
    }
}
DECL_TASK(bridge_serial_task);

// Open the port's USART at 'baud'. Called once per UART object, from
// config_uart (src/uart.c), so a rate this UART cannot be set to fails the
// host's configuration with its own message instead of leaving a port that
// never speaks.
//
// The line rate is the one the host asked for; the port is otherwise fixed at
// build time, so the only slot it can live on is 0 - the first port the host
// declares. A bridge with more than one MCU-hosted port would need a
// slot-to-USART table, which this build does not carry.
enum uart_open_result
uart_backend_open(struct uart_s *u, uint32_t baud, int slot)
{
    if (slot != 0)
        return UART_OPEN_NO_DEVICE;
    // One UART, one open: a second object on it is refused rather than sharing
    // the register block with the first.
    if (bridge_serial_owner)
        return UART_OPEN_BUSY;

    // The BRR is a 16-bit divisor (12-bit mantissa, 4-bit fraction), so a rate
    // whose divisor does not fit is a rate this UART cannot be set to.
    enable_pclock((uint32_t)USARTx);
    uint32_t pclk = get_pclock_frequency((uint32_t)USARTx);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, baud);
    if (!div || div > 0xffff)
        return UART_OPEN_BAD_BAUD;

    rx_head = rx_tail = 0;
    tx_pos = tx_max = 0;
    USARTx->BRR = (((div / 16) << USART_BRR_DIV_Mantissa_Pos)
                   | ((div % 16) << USART_BRR_DIV_Fraction_Pos));
    USARTx->CR1 = CR1_FLAGS;
    armcm_enable_irq(USARTx_IRQHandler, USARTx_IRQn, 0);

    gpio_peripheral(GPIO_Rx, GPIO_FUNCTION(GPIO_AF_MODE), 1);
    gpio_peripheral(GPIO_Tx, GPIO_FUNCTION(GPIO_AF_MODE), 0);

    // Both the clock and the pins are up, and only now does the slot have an
    // owner; an earlier failure leaves it exactly as it was.
    bridge_serial_owner = u;
    return UART_OPEN_OK;
}

#if CONFIG_BRIDGE
// The router's write hook: queue one whole blob on the port, then let the
// transmit interrupt send it. Nothing here blocks or waits for the wire - the
// blob is bounded by BUF_SIZE, and one that does not fit is refused (-1) rather
// than cutting a queued blob short, the same way the host's backend refuses it.
static int
bridge_serial_write(void *ctx, const uint8_t *data, size_t len)
{
    if (ctx != bridge_serial_owner)
        return -1;
    irqstatus_t flag = irq_save();
    uint16_t max = tx_max;
    // Drop what the interrupt has already sent, so the free space is contiguous
    // and the append below is one memcpy. The queue is the task's to compact and
    // the interrupt's to drain, and interrupts are off in between.
    uint16_t pos = tx_pos;
    if (pos) {
        memmove(tx_buf, &tx_buf[pos], max - pos);
        max -= pos;
        tx_pos = 0;
    }
    if ((size_t)max + len > sizeof(tx_buf)) {
        irq_restore(flag);
        return -1;
    }
    memcpy(&tx_buf[max], data, len);
    tx_max = max + len;
    USARTx->CR1 = CR1_FLAGS | USART_CR1_TXEIE;
    irq_restore(flag);
    return 0;
}

static const struct bridge_port_ops bridge_serial_ops = {
    .write = bridge_serial_write,
    // No modem lines: a USART has none, so there is nothing to pulse for the
    // port-level RESET_PORT (MUX_CMD_RESET_PORT). The router drops it in
    // silence when this is NULL (bridge.h), which is the honest answer - a
    // downstream device with no admin channel of its own (a legacy device
    // configured with uart_restart_method) has no line-level reset on an MCU
    // bridge. A named device is unaffected: its restart goes down as
    // RESTART_DEVICE and the device restarts itself.
    .reset = NULL,
};

// The one pair of hooks above serves the one port this build can carry; 'u' is
// only the object whose write() calls arrive, and they are checked against the
// owner in the hook.
const struct bridge_port_ops *
uart_backend_port_ops(struct uart_s *u)
{
    (void)u;
    return &bridge_serial_ops;
}

// The router took this UART as the port with this index; the task feeds the
// bytes it reads under it. The build has one port, so there is one index to
// record - and it is recorded before the host can send anything to the port.
void
uart_backend_set_port(struct uart_s *u, int index)
{
    (void)u;
    bridge_serial_port = index;
}
#endif // CONFIG_BRIDGE
