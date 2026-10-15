// Main entry point for klipper host library
//
// The library runs klipper's scheduler via sched_main(). Since
// sched_main() loops forever, an external shutdown request is
// delivered with SIGUSR1, whose handler longjmps back into
// klipper_host_run() so it can return to the caller.
//
// Internal shutdowns (e.g. emergency_stop commands) are handled by
// sched_main() itself and keep the library alive (config_reset /
// clear_shutdown can bring it back), exactly like a real MCU.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#define _GNU_SOURCE
#include <setjmp.h> // setjmp, longjmp
#include <signal.h> // sigaction, SIGUSR1
#include <pthread.h> // pthread_t, pthread_self, pthread_kill
#include <stdio.h> // fprintf
#include <string.h> // memset, memcpy
#include "autoconf.h" // CONFIG_MUX_SERIAL
#include "basecmd.h" // stats_update
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "sched.h" // sched_main, sched_is_shutdown
#include "internal.h" // klipper_console_unlock
#include "mux.h" // mux_set_uuid, mux_uuid
#include "klipper_host.h"

// Global state
static volatile sig_atomic_t g_shutdown_requested;
static pthread_t g_klipper_thread;
static volatile int g_klipper_thread_active;
static jmp_buf g_main_jmp;
static int g_initialised;

// The tty the host named for each bridge port, one slot per port index, before
// klipper came up (klipper_host_port). A bit in g_port_named says whether its
// slot was named at all; an unnamed one falls back to the build's own path
// (CONFIG_HOST_BRIDGE_SERIAL_PATH, read by src/host/bridge.c).
#define HOST_PORT_PATH_MAX 256
static char g_port_paths[KLIPPER_HOST_PORT_SLOTS][HOST_PORT_PATH_MAX];
static unsigned int g_port_named;



// SIGUSR1 handler - longjmp out of sched_main() on external shutdown
static void
sigusr1_handler(int sig)
{
    (void)sig;
    if (g_shutdown_requested)
        longjmp(g_main_jmp, 1);
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

    // Install the external shutdown handler
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sigusr1_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGUSR1, &sa, NULL);

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
// Delivers SIGUSR1 to the thread running klipper_host_run().
void
klipper_host_shutdown(void)
{
    g_shutdown_requested = 1;
    // Release anyone blocked in klipper_host_output_wait(), so a host reading in
    // a loop notices the shutdown immediately instead of after its timeout.
    klipper_console_wake();
    if (g_klipper_thread_active)
        pthread_kill(g_klipper_thread, SIGUSR1);
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

    g_klipper_thread = pthread_self();
    g_klipper_thread_active = 1;

    if (setjmp(g_main_jmp) == 0) {
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
