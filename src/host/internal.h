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

#endif // host/internal.h
