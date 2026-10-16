//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
#ifndef __HOST_INTERNAL_H
#define __HOST_INTERNAL_H
// Internal definitions shared between klipper host library modules.

// host.c - the tty the host named for bridge port 'slot' with
// klipper_host_port(), or NULL when it named none - the port then falls back to
// the build's own path. Used by bridge.c when a UART object opens.
const char *klipper_host_port_path(int slot);

// host.c - the fd the task loop sleeps on while it has nothing to do, or -1
// before klipper_host_init(), and the one klipper_host_shutdown() writes to end
// that sleep. One fd per loaded instance. Used by timer.c's irq_wait().
int klipper_host_wake_fd(void);

// host.c - end the task loop started by klipper_host_run() when
// klipper_host_shutdown() was called; returns when no loop is running (the
// request is then acted on by the loop's start check, and owes no unwinding).
// Called from the task loop's interrupt emulation in timer.c.
void klipper_host_check_shutdown(void);

// console.c - unlock console mutexes (used after longjmp unwinding)
void klipper_console_unlock(void);

// console.c - wake threads waiting in klipper_host_output_wait() (used when
// klipper is asked to shut down)
void klipper_console_wake(void);

// bridge.c - read the bridge's downstream ports and feed them to the router. Called
// from irq_wait() while the task loop is idle, the way src/linux/console.c
// polls its tty from console_sleep().
void bridge_poll(void);

#endif // host/internal.h
