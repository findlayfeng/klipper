// Timer handling for klipper host library
//
// Provides the monotonic clock (timer_read_time) and the interrupt
// emulation layer (irq_*). Timer dispatch (timer_dispatch_many),
// timer_from_us() and timer_is_before() come from generic/timer_irq.c.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#define _GNU_SOURCE
#include <time.h> // clock_gettime, struct timespec
#include <signal.h> // sigaction, sigemptyset, alarm
#include <string.h> // memset
#include <unistd.h> // alarm
#include <poll.h> // ppoll
#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "board/irq.h" // irq_disable, irq_save, irq_poll
#include "board/misc.h" // timer_read_time, timer_kick
#include "board/timer_irq.h" // timer_dispatch_many
#include "sched.h" // DECL_INIT
#include "internal.h" // bridge_poll

#define NSECS 1000000000ULL
#define NSECS_PER_TICK (NSECS / CONFIG_CLOCK_FREQ)

// Global timer state
static struct {
    struct timespec base_ts;      // Reference time
    uint32_t base_counter;        // Counter value at reference time
    uint32_t next_wake_counter;   // Next scheduled wake time
    volatile uint8_t need_timer_dispatch; // Set when timers must run
    sigset_t ss_timer;            // Timer signal set
} TimerState;

// Convert a timespec (relative to base_ts) to a counter value
static inline uint32_t
timespec_to_counter(struct timespec ts)
{
    int64_t sec_diff = (int64_t)ts.tv_sec - (int64_t)TimerState.base_ts.tv_sec;
    int64_t nsec_diff = sec_diff * 1000000000LL
                        + (int64_t)ts.tv_nsec - (int64_t)TimerState.base_ts.tv_nsec;
    return (uint32_t)(nsec_diff / NSECS_PER_TICK);
}

// Convert a counter value to an absolute timespec
static inline void
counter_to_timespec(uint32_t counter, struct timespec *ts)
{
    int64_t nsec_diff = (int64_t)(counter - TimerState.base_counter)
                        * NSECS_PER_TICK;
    ts->tv_sec = TimerState.base_ts.tv_sec + nsec_diff / 1000000000LL;
    ts->tv_nsec = TimerState.base_ts.tv_nsec + nsec_diff % 1000000000LL;
    if (ts->tv_nsec < 0) {
        ts->tv_sec--;
        ts->tv_nsec += 1000000000LL;
    }
}

// SIGALRM handler - wakes up irq_wait()
static void
timer_signal_handler(int sig)
{
    (void)sig;
    TimerState.need_timer_dispatch = 1;
}

// Return the current time (in absolute clock ticks)
uint32_t
timer_read_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return timespec_to_counter(ts);
}

// Activate timer dispatch as soon as possible
void
timer_kick(void)
{
    TimerState.need_timer_dispatch = 1;
}

// Initialize timer subsystem
void
timer_init(void)
{
    // Record base time
    clock_gettime(CLOCK_MONOTONIC, &TimerState.base_ts);
    TimerState.base_counter = 0;
    TimerState.next_wake_counter = timer_read_time();
    TimerState.need_timer_dispatch = 1;

    // Set up SIGALRM handler
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = timer_signal_handler;
    sa.sa_flags = SA_RESTART;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGALRM, &sa, NULL);
    sigemptyset(&TimerState.ss_timer);
    sigaddset(&TimerState.ss_timer, SIGALRM);

    timer_kick();
}
DECL_INIT(timer_init);


/****************************************************************
 * Interrupt emulation
 ****************************************************************/

// Disable hardware interrupts (no-op on host)
void
irq_disable(void)
{
}

// Enable hardware interrupts (no-op on host)
void
irq_enable(void)
{
}

// Save interrupt state (no-op on host)
irqstatus_t
irq_save(void)
{
    return 0;
}

// Restore interrupt state (no-op on host)
void
irq_restore(irqstatus_t flag)
{
    (void)flag;
}

// Atomically enable interrupts and sleep until the next timer irq
void
irq_wait(void)
{
    if (!TimerState.need_timer_dispatch) {
        // Sleep briefly, waking on SIGALRM or timeout
        struct timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = 1000 * 1000; // 1ms poll interval
        ppoll(NULL, 0, &ts, &TimerState.ss_timer);
    }
    irq_poll();
#if CONFIG_BRIDGE && CONFIG_BRIDGE_PORT_SERIAL
    // The bridge's downstream port is one more source of input; poll it on the
    // same beat, while the loop has nothing else to do.
    bridge_poll();
#endif
}

// Check if a timer interrupt is pending
void
irq_poll(void)
{
    if (TimerState.need_timer_dispatch) {
        TimerState.need_timer_dispatch = 0;
        TimerState.next_wake_counter = timer_dispatch_many();
    }
}
