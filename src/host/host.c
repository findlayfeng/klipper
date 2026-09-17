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
#include <string.h> // memset
#include "basecmd.h" // stats_update
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "sched.h" // sched_main, sched_is_shutdown
#include "internal.h" // klipper_console_unlock
#include "klipper_host.h"

// Global state
static volatile sig_atomic_t g_shutdown_requested;
static pthread_t g_klipper_thread;
static volatile int g_klipper_thread_active;
static jmp_buf g_main_jmp;

// SIGUSR1 handler - longjmp out of sched_main() on external shutdown
static void
sigusr1_handler(int sig)
{
    (void)sig;
    if (g_shutdown_requested)
        longjmp(g_main_jmp, 1);
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
