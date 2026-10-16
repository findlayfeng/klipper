// Main entry point for klipper host library
//
// The library runs klipper's scheduler via sched_main(). Since
// sched_main() loops forever, an external shutdown request is
// delivered through the library's own wake fd (g_wake_fd):
// klipper_host_shutdown() sets g_shutdown_requested and writes that
// fd, the task loop's sleep (irq_wait, timer.c) wakes on it, and the
// loop then longjmps back into klipper_host_run() so it can return to
// the caller. The fd is per loaded instance, so no process-wide
// state - a signal disposition, say - takes part in it.
//
// Internal shutdowns (e.g. emergency_stop commands) are handled by
// sched_main() itself and keep the library alive (config_reset /
// clear_shutdown can bring it back), exactly like a real MCU.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#define _GNU_SOURCE
#include <errno.h> // errno
#include <setjmp.h> // setjmp, longjmp
#include <stdio.h> // fprintf
#include <string.h> // strerror
#include <sys/eventfd.h> // eventfd, EFD_CLOEXEC, EFD_NONBLOCK
#include <unistd.h> // write
#include "autoconf.h" // CONFIG_MUX_SERIAL
#include "basecmd.h" // stats_update
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time, mcu_restart
#include "sched.h" // sched_main, sched_is_shutdown
#include "internal.h" // klipper_console_unlock
#include "mux.h" // mux_set_uuid, mux_uuid
#include "klipper_host.h"

// Global state
static volatile int g_shutdown_requested;
static volatile int g_klipper_thread_active;
static jmp_buf g_main_jmp;
static int g_initialised;
// The fd the task loop sleeps on and klipper_host_shutdown() writes to end that
// sleep: one eventfd per loaded instance, so that ending the loop needs no
// process-wide state. -1 until klipper_host_init().
static int g_wake_fd = -1;

// The tty the host named for each bridge port, one slot per port index, before
// klipper came up (klipper_host_port). A bit in g_port_named says whether its
// slot was named at all; an unnamed one falls back to the build's own path
// (CONFIG_HOST_BRIDGE_SERIAL_PATH, read by src/host/bridge.c).
#define HOST_PORT_PATH_MAX 256
static char g_port_paths[KLIPPER_HOST_PORT_SLOTS][HOST_PORT_PATH_MAX];
static unsigned int g_port_named;



// End the task loop started by klipper_host_run() when the host asked for a
// shutdown, i.e. leave through the longjmp that returns to klipper_host_run().
// Called from the task loop's interrupt emulation (timer.c), always on the
// thread running that loop. A request that arrives while no loop is running owes
// no unwinding - the loop's own start check acts on it instead.
void
klipper_host_check_shutdown(void)
{
    if (g_shutdown_requested && g_klipper_thread_active)
        longjmp(g_main_jmp, 1);
}

// The fd the task loop sleeps on while it has nothing to do (timer.c), and the
// one klipper_host_shutdown() writes to end that sleep. -1 when the library was
// never initialised: the sleep is then a plain tick.
int
klipper_host_wake_fd(void)
{
    return g_wake_fd;
}

// Set the identity this library answers the serial mux with.
// Has to happen before klipper_host_init(): the identity is fixed once the
// device is up, so a late call is reported rather than half-applied.
int
klipper_host_set_uuid(const uint8_t uuid[6])
{
    if (g_initialised)
        return -1;
    // The mux owns the identity - it is what the device answers a scan with and
    // what an assignment names. Without the mux there is nothing to answer, so
    // the value is accepted and has no effect.
#if CONFIG_MUX_SERIAL
    mux_set_uuid(uuid);
#else
    (void)uuid;
#endif
    return 0;
}

// Name the tty the bridge port 'slot' opens. Has to happen before
// klipper_host_init(): the port opens when the host's config_uart arrives, and
// the stream it gets is fixed once the device is up, so a late call is reported
// rather than half-applied. The path is copied here - the caller keeps no
// lifetime it would have to honour.
int
klipper_host_port(int slot, const char *path)
{
    if (g_initialised)
        return -1;
    if (slot < 0 || slot >= KLIPPER_HOST_PORT_SLOTS || !path)
        return -1;
    int len = snprintf(g_port_paths[slot], sizeof(g_port_paths[slot]), "%s",
                       path);
    if (len < 0 || (size_t)len >= sizeof(g_port_paths[slot]))
        return -1;
    g_port_named |= 1u << slot;
    return 0;
}

// The tty the host named for 'slot', or NULL when it named none.
const char *
klipper_host_port_path(int slot)
{
    if (slot < 0 || slot >= KLIPPER_HOST_PORT_SLOTS
        || !(g_port_named & (1u << slot)))
        return NULL;
    return g_port_paths[slot];
}

// Initialize the host library
int
klipper_host_init(void)
{
    g_shutdown_requested = 0;
    g_klipper_thread_active = 0;

    // The fd klipper_host_shutdown() writes to end the task loop's sleep. One
    // per loaded instance, non-blocking so a read of it that finds it empty does
    // not sit there, and closed across exec so a host that execs something else
    // does not hand it the fd.
    g_wake_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (g_wake_fd < 0) {
        fprintf(stderr, "klipper host: eventfd failed: %s\n", strerror(errno));
        return -1;
    }

#if CONFIG_MUX_SERIAL
    // The identity is the host's to assign (klipper_host_set_uuid); report what
    // this device will answer the mux with, so a host that never set one can
    // see that it is talking to the unnamed device.
    const uint8_t *uuid = mux_uuid();
    fprintf(stderr, "mux uuid: %02x%02x%02x%02x%02x%02x\n",
            uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5]);
#endif

    // Bridge mode takes its downstream ports from the host's configuration
    // (config_uart / config_bridge_port, src/uart.c), so init opens nothing and
    // nothing here can fail on a tty that is not there: a port that cannot be
    // brought up fails the configuration instead.
    g_initialised = 1;
    return 0;
}

// Request klipper to shut down (thread-safe).
// Wakes the thread running klipper_host_run() through the library's wake fd.
void
klipper_host_shutdown(void)
{
    g_shutdown_requested = 1;
    // Release anyone blocked in klipper_host_output_wait(), so a host reading in
    // a loop notices the shutdown immediately instead of after its timeout.
    klipper_console_wake();
    // End the task loop's sleep. The write does not have to be checked: should it
    // fail, the next tick of that sleep (1ms, timer.c) carries the flag into the
    // loop anyway.
    if (g_wake_fd >= 0) {
        uint64_t one = 1;
        ssize_t written = write(g_wake_fd, &one, sizeof(one));
        (void)written;
    }
}

// Check if klipper is in (internal) shutdown state
uint8_t
klipper_host_is_shutdown(void)
{
    return sched_is_shutdown();
}

// Get the current klipper clock value
uint32_t
klipper_host_get_clock(void)
{
    return timer_read_time();
}

// Run the klipper main scheduling loop (blocking).
// Call from a dedicated thread; klipper_host_shutdown() will make
// this function return.
void
klipper_host_run(void)
{
    if (g_klipper_thread_active || g_shutdown_requested)
        return;

    if (setjmp(g_main_jmp) == 0) {
        // The loop is live from here on: timer.c's klipper_host_check_shutdown()
        // may unwind this frame, which is what the flag marks and the reason it
        // is set only now, with the jmp_buf in hand.
        g_klipper_thread_active = 1;
        // Run the klipper scheduler (does not return normally)
        sched_main();
    }

    // External shutdown: sched_main() was unwound by longjmp.
    g_klipper_thread_active = 0;
    // Release any console mutex left locked by the unwinding
    klipper_console_unlock();
}

// Run a single iteration of the klipper task loop (non-blocking).
// Useful for integrating the library into an external event loop.
// NOTE: do not use together with klipper_host_run(), and commands
// that trigger an internal shutdown are not supported in this mode.
void
klipper_host_step(void)
{
    if (g_klipper_thread_active)
        return;
    extern void ctr_run_taskfuncs(void);

    irq_poll();
    if (sched_is_shutdown())
        return;
    uint32_t start = timer_read_time();
    ctr_run_taskfuncs();
    uint32_t cur = timer_read_time();
    stats_update(start, cur);
}

/****************************************************************
 * Restart
 ****************************************************************/

// Restart this firmware: there is nothing here that could.
//
// The firmware this library carries is the process it was linked into, so a
// restart is the host's call, not the library's: it is klipper_host_shutdown()
// plus the host bringing the library up again that means a restart here, and a
// library that shut itself down recovers the way the header comment has it
// (config_reset / clear_shutdown), inside the same process.
//
// So the mux's RESTART_DEVICE command (mux.c) is accepted and does nothing -
// the same shape a board with no bootloader gives that command's sibling,
// REQUEST_BOOTLOADER: the command is understood, and this carrier cannot carry
// it out.
void
mcu_restart(void)
{
}
