//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
#ifndef __HOST_INTERNAL_H
#define __HOST_INTERNAL_H
// Internal definitions shared between klipper host library modules.

// console.c - unlock console mutexes (used after longjmp unwinding)
void klipper_console_unlock(void);

// console.c - wake threads waiting in klipper_host_output_wait() (used when
// klipper is asked to shut down)
void klipper_console_wake(void);

// bridge.c - open the host's bridge serial port (CONFIG_BRIDGE_PORT_SERIAL).
// Returns 0 when the port is open, -1 when it cannot be brought up.
int bridge_setup(void);

#endif // host/internal.h
