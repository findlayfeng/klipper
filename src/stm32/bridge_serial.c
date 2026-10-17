// STM32 bridge port backend: hardware USARTs handed to the bridge
//
// A bridge build has one console - here USB CDC, or the console USART - and
// downstream ports beside it; this file is the arch's USART backend for them:
// the USARTs a board wires to the devices the host reaches through the bridge
// (the port-level commands, and the flashing tunnel a katapult bootloader is
// driven through). The arch's seam itself (uart.h) is the dispatcher's,
// bridge_port.c, which is where a slot is resolved to a backend; this file is
// what that dispatcher calls for the USART slots: bridge_serial_open() brings a
// port's USART up at the rate the host asks for in config_uart,
// bridge_serial_port_ops() gives the router the hook that writes a whole blob to
// it, and bridge_serial_set_port() records the router's port index so bytes read
// back can be fed to bridge_port_input() under that port.
//
// Which USARTs a build offers is a build-time fact: CONFIG_STM32_BRIDGE_SERIAL
// turns the backend on, and each USART ticked under it
// (CONFIG_STM32_BRIDGE_SERIAL_USART1/2/3, _UART4/_UART5) adds one port with a
// fixed pair of pins (the table below). A port's slot - the number the host
// names it by, and the order the host lists the ports in - is its USART's place
// among the ticked ones counted by USART number: USART1, then USART2, USART3,
// UART4, UART5. The table below is written in that order, so a port's slot is
// its index in it, and these are the slots the dispatcher (bridge_port.c) hands
// out first: 0..bridge_serial_slot_count()-1, ahead of any other backend's. An
// unticked USART simply is not there: no slot, no buffer, no interrupt, and the
// host cannot open it.
//
// A port is never the console's USART, and the console is never a port's: one
// USART has one register block and one interrupt vector, so it cannot carry
// both at the same time. Each USART's Kconfig option excludes the console's
// choices for that USART.
//
// The rate is not a build-time fact: config_uart carries it, so the BRR is
// computed here while the host is configuring the object - never
// CONFIG_SERIAL_BAUD, which belongs to the console's fixed carrier.
//
// Receive is push, the way it is on every arch: the interrupt appends to a ring
// and a task drains it into bridge_port_input() (the router has no poll hook,
// and the host library's poll lives on irq_wait(), which is the host's alone).
// Transmit needs no task - a queued blob is drained by the port's own TX
// interrupt. Every open port has its own ring, its own queue and its own
// interrupt, so ports never share bytes or block each other, and one task
// drains all of them.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // NULL
#include <string.h> // memcpy, memmove
#include "autoconf.h" // CONFIG_BRIDGE_PORT_SERIAL, CONFIG_STM32_BRIDGE_SERIAL_USART1
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/irq.h" // irq_save, irq_restore
#include "bridge.h" // bridge_port_input, struct bridge_port_ops
#include "command.h" // DECL_CONSTANT, DECL_CONSTANT_STR, MESSAGE_MAX
#include "compiler.h" // ARRAY_SIZE, DIV_ROUND_CLOSEST
#include "internal.h" // enable_pclock, get_pclock_frequency, gpio_peripheral, GPIO
#include "mux.h" // MUX_HEADER_BYTES, MUX_TUNNEL_FRAME_MAX
#include "sched.h" // DECL_TASK, sched_wake_tasks
#include "bridge_port.h" // bridge_serial_open, bridge_serial_slot_count

// 8N1, receiver and transmitter on, receive interrupt on. The transmit
// interrupt is not here: it is turned on only while a blob is queued.
#define CR1_FLAGS (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE   \
                   | USART_CR1_RXNEIE)

// The pins of every port below are a USART's alternate function in mode 7: that
// is the number the F1 remap emulation reads out of a pin's mode (internal.h's
// GPIO_FUNCTION, stm32/stm32f1.c:gpio_peripheral), and for USART1/2/3 it is
// also the alternate function number of the other families this backend covers.
// UART4 and UART5 are F1 only for that reason - on the F4 and F7 their
// alternate function is 8 even on the same pins.
#define GPIO_AF_MODE 7

// The receive and transmit buffers hold the largest things that can be in
// flight back to back - a whole tunnel block (MUX_TUNNEL_FRAME_MAX, 128 bytes,
// the bootloader stream a port can be toggled into) plus a klipper unit
// (MUX_HEADER_BYTES + MESSAGE_MAX, 66 bytes) - with room for a burst. A power
// of two, so the receive ring's indices can be masked into the buffer.
#define BUF_SIZE 256

// What a port is at build time: the USART it comes out on, the call that arms
// that USART's vector, the pins it is wired to, and the port's run-time state.
// Constant, so it costs flash and no RAM.
struct bridge_serial_hw {
    struct bridge_serial_port *state;
    USART_TypeDef *regs;      // the register block, which is also the clock's
                              // key (enable_pclock, get_pclock_frequency)
    void (*enable_irq)(void); // arms this port's vector. The name and number of
                              // an interrupt are compile-time requests
                              // (armcm_enable_irq), so each port brings its own
                              // call rather than a number this code could pass in
    uint8_t rx_pin, tx_pin;   // the pins the host must not claim for anything
                              // else (RESERVE_PINS_bridge_serial*, below)
};

// What a port is at run time: everything that belongs to it alone, so nothing
// but its own interrupt writes it. A ticked USART gets one static instance of
// this, all zero - hence in .bss rather than in the binary the way a struct with
// a pointer in its initialiser would be.
struct bridge_serial_port {
    // The receive ring: the interrupt appends at rx_head, the task consumes up
    // to rx_tail, and the ring is full exactly when the two would meet, so it
    // holds BUF_SIZE-1 bytes - still more than the largest pair above.
    uint8_t rx_buf[BUF_SIZE];
    volatile uint8_t rx_head, rx_tail;
    // The bytes the ring had no room for: each one dropped above makes a frame
    // the router cannot reassemble on its own side, which is a lost burst rather
    // than a report, so the count is the only trace there is on a micro-
    // controller (no stderr, and no host-visible channel in this backend). Kept
    // as a diagnostic beside the ring it belongs to.
    uint32_t rx_dropped;

    // The queued output: the router hands each unit over as one blob and this
    // holds it until the USART can take it, so a later blob never cuts in front
    // of a queued one. The interrupt sends from tx_pos up to tx_max; the task
    // appends at tx_max, after dropping the already-sent prefix.
    uint8_t tx_buf[BUF_SIZE];
    volatile uint16_t tx_pos, tx_max;

    // The UART object that opened the port (NULL until bridge_serial_open), and
    // the router's index for it, recorded one past itself so that an untouched
    // port means "no index yet" - 0 is a real slot (the same encoding
    // bridge.c's bridge_id_port uses). The index is what received bytes are fed
    // under.
    struct uart_s *owner;
    int port;
};

// Whether this build ticks any USART at all - a preprocessor-level copy of "the
// table below is not empty". #if runs before the compiler could read the table's
// size, so the same five options are listed again here, and the two lists have
// to keep naming the same USARTs.
#if CONFIG_STM32_BRIDGE_SERIAL_USART1 || CONFIG_STM32_BRIDGE_SERIAL_USART2 \
    || CONFIG_STM32_BRIDGE_SERIAL_USART3 || CONFIG_STM32_BRIDGE_SERIAL_UART4 \
    || CONFIG_STM32_BRIDGE_SERIAL_UART5
#define BRIDGE_SERIAL_ANY_PORT 1
#else
#define BRIDGE_SERIAL_ANY_PORT 0
#endif

// No port, no interrupt: a build that turns the backend on and ticks no USART
// below has no vector to serve and nothing to hand on.
#if BRIDGE_SERIAL_ANY_PORT
static void bridge_serial_irq(const struct bridge_serial_hw *hw);
#endif

// One ticked USART, as a port: its state, the call that arms its vector, the
// handler that vector calls, and the dictionary constant the host reserves its
// pins under. 'id' names the state (bridge_serial_usart1); 'usart' is the
// register block the way the CMSIS header spells it (USART1, UART4), so its
// vector is <usart>_IRQn and its handler <usart>_IRQHandler; 'rx'/'tx' are its
// pins, RX first; 'reserve' is the name the host reserves those pins under, and
// 'pins' is what it reserves (a pin is reserved for a feature, and that name is
// what the host reports it as).
#define DECL_BRIDGE_SERIAL_UART(id, usart, rx, tx, reserve, pins)         \
    static struct bridge_serial_port bridge_serial_##id;                  \
    static void bridge_serial_##id##_enable_irq(void)                     \
    {                                                                     \
        armcm_enable_irq(usart##_IRQHandler, usart##_IRQn, 0);            \
    }                                                                     \
    static const struct bridge_serial_hw bridge_serial_##id##_hw = {      \
        .state = &bridge_serial_##id,                                     \
        .regs = usart,                                                    \
        .enable_irq = bridge_serial_##id##_enable_irq,                    \
        .rx_pin = rx,                                                     \
        .tx_pin = tx,                                                     \
    };                                                                    \
    void usart##_IRQHandler(void)                                         \
    {                                                                     \
        bridge_serial_irq(&bridge_serial_##id##_hw);                      \
    }                                                                     \
    DECL_CONSTANT_STR("RESERVE_PINS_" reserve, pins)

// The USARTs this build offers, in the order the slots number them: the first
// ticked USART below is slot 0, the next slot 1, and so on. USART1 keeps the
// constant name the single-port backend always used; the others are named after
// their own USART, so a pin the host has to refuse names the port it belongs to.
#if CONFIG_STM32_BRIDGE_SERIAL_USART1
DECL_BRIDGE_SERIAL_UART(usart1, USART1, GPIO('A', 10), GPIO('A', 9),
                        "bridge_serial", "PA10,PA9");
#endif
#if CONFIG_STM32_BRIDGE_SERIAL_USART2
DECL_BRIDGE_SERIAL_UART(usart2, USART2, GPIO('A', 3), GPIO('A', 2),
                        "bridge_serial_usart2", "PA3,PA2");
#endif
#if CONFIG_STM32_BRIDGE_SERIAL_USART3
DECL_BRIDGE_SERIAL_UART(usart3, USART3, GPIO('B', 11), GPIO('B', 10),
                        "bridge_serial_usart3", "PB11,PB10");
#endif
#if CONFIG_STM32_BRIDGE_SERIAL_UART4
DECL_BRIDGE_SERIAL_UART(uart4, UART4, GPIO('C', 11), GPIO('C', 10),
                        "bridge_serial_uart4", "PC11,PC10");
#endif
#if CONFIG_STM32_BRIDGE_SERIAL_UART5
DECL_BRIDGE_SERIAL_UART(uart5, UART5, GPIO('D', 2), GPIO('C', 12),
                        "bridge_serial_uart5", "PD2,PC12");
#endif

// The ports in slot order. This list is what numbers the slots: a port's slot is
// its index here, and the list's size is the count below - so the order it is
// written in is the contract, not an accident of the declarations above. Every
// entry is ticked in Kconfig, so a build with the backend on and nothing ticked
// has an empty table: it offers no slot, and says so.
static const struct bridge_serial_hw *const bridge_serial_hw[] = {
#if CONFIG_STM32_BRIDGE_SERIAL_USART1
    &bridge_serial_usart1_hw,
#endif
#if CONFIG_STM32_BRIDGE_SERIAL_USART2
    &bridge_serial_usart2_hw,
#endif
#if CONFIG_STM32_BRIDGE_SERIAL_USART3
    &bridge_serial_usart3_hw,
#endif
#if CONFIG_STM32_BRIDGE_SERIAL_UART4
    &bridge_serial_uart4_hw,
#endif
#if CONFIG_STM32_BRIDGE_SERIAL_UART5
    &bridge_serial_uart5_hw,
#endif
};

// How many UART slots this board can give the bridge: one per ticked USART,
// because each is a USART wired to a port and bridge_serial_open() refuses every
// slot but those. The count is the backend's fact - the same table it enforces -
// and it is what the dispatcher (bridge_port.c) counts the first slots with.
int
bridge_serial_slot_count(void)
{
    return ARRAY_SIZE(bridge_serial_hw);
}

// Receive side of a port's interrupt: append the byte to that port's ring.
#if BRIDGE_SERIAL_ANY_PORT
static void
bridge_serial_rx_byte(const struct bridge_serial_hw *hw, uint8_t data)
{
    struct bridge_serial_port *port = hw->state;
    uint8_t next = (port->rx_head + 1) & (BUF_SIZE - 1);
    if (next == port->rx_tail) {
        // Full: drop the newest byte rather than an older one, so what is
        // already queued stays a contiguous, in-order prefix. The byte is
        // gone, and the device's next frame fails its CRC (or the tunnel's own
        // retry notices) - the same trade the console's serial_rx_byte makes.
        port->rx_dropped++;
        return;
    }
    port->rx_buf[port->rx_head] = data;
    port->rx_head = next;
    // Wake the task on every byte, not only on MESSAGE_SYNC the way the
    // console's serial_rx_byte does: a port can carry a bootloader's katapult
    // stream, which has no klipper sync byte, and the bytes must reach the
    // router as they come - tasks are only dispatched when something wakes
    // them.
    sched_wake_tasks();
}

// Receive interrupt for 'hw'. Called directly from its USARTx_IRQHandler above,
// where the interrupt's shared status register is read.
static void
bridge_serial_irq(const struct bridge_serial_hw *hw)
{
    struct bridge_serial_port *port = hw->state;
    USART_TypeDef *regs = hw->regs;
    uint32_t sr = regs->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        // The ORE flag is automatically cleared by reading SR, followed
        // by reading DR.
        bridge_serial_rx_byte(hw, regs->DR);
    }
    if (sr & USART_SR_TXE && regs->CR1 & USART_CR1_TXEIE) {
        uint16_t pos = port->tx_pos;
        if (pos != port->tx_max) {
            regs->DR = port->tx_buf[pos];
            port->tx_pos = pos + 1;
        } else {
            // Nothing queued: turn the transmit interrupt off until the next
            // blob, so an idle port costs nothing.
            regs->CR1 = CR1_FLAGS;
        }
    }
}
#endif // BRIDGE_SERIAL_ANY_PORT

// Hand the bytes 'hw' received to the router. They cannot be fed from the
// interrupt itself (bridge_port_input buffers and scans a whole unit, and the
// router's work must not nest in an interrupt), so they wait in the port's ring
// for the task.
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

static void
bridge_serial_drain(const struct bridge_serial_hw *hw)
{
    struct bridge_serial_port *port = hw->state;
    int index = port->port - 1;
    // The port has not been handed to the router yet: the bytes keep waiting in
    // the ring (config_bridge_port follows config_uart closely enough for that).
    if (index < 0)
        return;
    for (;;) {
        irqstatus_t flag = irq_save();
        uint8_t head = port->rx_head, tail = port->rx_tail;
        if (tail != head) {
            // The contiguous run: up to the ring's end (the bytes past it wait
            // for the next pass) and up to the chunk bound.
            size_t n = head > tail ? (size_t)(head - tail)
                                   : (size_t)(BUF_SIZE - tail);
            if (n > DRAIN_CHUNK)
                n = DRAIN_CHUNK;
            bridge_port_input(index, &port->rx_buf[tail], n);
            port->rx_tail = (tail + n) & (BUF_SIZE - 1);
        }
        irq_restore(flag);
        if (tail == head)
            return;
    }
}

// One task for every port: each port's bytes have their own ring, so a pass over
// the table hands all of them over before the scheduler sleeps.
void
bridge_serial_task(void)
{
    int i;
    for (i = 0; i < bridge_serial_slot_count(); i++)
        bridge_serial_drain(bridge_serial_hw[i]);
}
DECL_TASK(bridge_serial_task);

// Open the USART behind 'slot' at 'baud'. Called once per UART object, from the
// dispatcher (bridge_port.c), which resolved the slot to this backend - so a
// slot this build does not offer, or a rate this USART cannot be set to, fails
// the host's configuration with its own message instead of leaving a port that
// never speaks.
//
// The line rate is the one the host asked for; which USART a slot is stays a
// build-time fact (the table above), so the slot is looked up here.
enum uart_open_result
bridge_serial_open(struct uart_s *u, uint32_t baud, int slot)
{
    if (slot < 0 || slot >= bridge_serial_slot_count())
        return UART_OPEN_NO_DEVICE;
    const struct bridge_serial_hw *hw = bridge_serial_hw[slot];
    struct bridge_serial_port *port = hw->state;
    // One UART, one open: a second object on the same USART is refused rather
    // than sharing the register block with the first.
    if (port->owner)
        return UART_OPEN_BUSY;

    // The BRR is a 16-bit divisor (12-bit mantissa, 4-bit fraction), so a rate
    // whose divisor does not fit is a rate this UART cannot be set to.
    USART_TypeDef *regs = hw->regs;
    enable_pclock((uint32_t)regs);
    uint32_t pclk = get_pclock_frequency((uint32_t)regs);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, baud);
    if (!div || div > 0xffff)
        return UART_OPEN_BAD_BAUD;

    port->rx_head = port->rx_tail = 0;
    port->tx_pos = port->tx_max = 0;
    regs->BRR = (((div / 16) << USART_BRR_DIV_Mantissa_Pos)
                 | ((div % 16) << USART_BRR_DIV_Fraction_Pos));
    regs->CR1 = CR1_FLAGS;
    hw->enable_irq();

    gpio_peripheral(hw->rx_pin, GPIO_FUNCTION(GPIO_AF_MODE), 1);
    gpio_peripheral(hw->tx_pin, GPIO_FUNCTION(GPIO_AF_MODE), 0);

    // Both the clock and the pins are up, and only now does the slot have an
    // owner; an earlier failure leaves it exactly as it was.
    port->owner = u;
    return UART_OPEN_OK;
}

#if CONFIG_BRIDGE
// The port a UART object opened: the router hands the object back as each port's
// ctx, so the object is what says which USART and which buffers a blob belongs
// to. NULL for an object that opened no port (it cannot have been attached to
// the bridge, then).
static const struct bridge_serial_hw *
bridge_serial_hw_of(void *ctx)
{
    int i;
    for (i = 0; i < bridge_serial_slot_count(); i++)
        if (bridge_serial_hw[i]->state->owner == ctx)
            return bridge_serial_hw[i];
    return NULL;
}

// The router's write hook: queue one whole blob on the port, then let its
// transmit interrupt send it. Nothing here blocks or waits for the wire - the
// blob is bounded by BUF_SIZE, and one that does not fit is refused (-1) rather
// than cutting a queued blob short, the same way the host's backend refuses it.
static int
bridge_serial_write(void *ctx, const uint8_t *data, size_t len)
{
    const struct bridge_serial_hw *hw = bridge_serial_hw_of(ctx);
    if (!hw)
        return -1;
    struct bridge_serial_port *port = hw->state;
    irqstatus_t flag = irq_save();
    uint16_t max = port->tx_max;
    // Drop what the interrupt has already sent, so the free space is contiguous
    // and the append below is one memcpy. The queue is the task's to compact and
    // the interrupt's to drain, and interrupts are off in between.
    uint16_t pos = port->tx_pos;
    if (pos) {
        memmove(port->tx_buf, &port->tx_buf[pos], max - pos);
        max -= pos;
        port->tx_pos = 0;
    }
    if ((size_t)max + len > sizeof(port->tx_buf)) {
        irq_restore(flag);
        return -1;
    }
    memcpy(&port->tx_buf[max], data, len);
    port->tx_max = max + len;
    hw->regs->CR1 = CR1_FLAGS | USART_CR1_TXEIE;
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

// The one pair of hooks above serves every port this build carries; which port a
// blob goes to is the object the router hands back as the ctx (checked against
// the owner in the hook), so there is nothing per-object to return here.
const struct bridge_port_ops *
bridge_serial_port_ops(struct uart_s *u)
{
    (void)u;
    return &bridge_serial_ops;
}

// The router took this UART as the port with this index; the task feeds the
// bytes it reads under it. The index belongs to the port the object opened, and
// it is recorded before the host can send anything to the port.
void
bridge_serial_set_port(struct uart_s *u, int index)
{
    const struct bridge_serial_hw *hw = bridge_serial_hw_of(u);
    if (hw)
        hw->state->port = index + 1;
}
#endif // CONFIG_BRIDGE
