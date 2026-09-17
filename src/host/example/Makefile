# Makefile for klipper host example program
#
# Copyright (C) 2026  Feng Cheng <i@pfeng.top>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# From src/host/example/, go up 3 levels to klipper root
KLIPPER_ROOT = ../../..
OUT = $(KLIPPER_ROOT)/out

CROSS_PREFIX ?=
CC = $(CROSS_PREFIX)gcc

# NOTE: do NOT add -I$(KLIPPER_ROOT)/src here. Klipper's src/ contains
# its own sched.h/command.h headers that would shadow the system headers
# (e.g. <sched.h> included by <pthread.h>). Only the public API header
# directory is needed.
CFLAGS = -I$(KLIPPER_ROOT)/src/host \
         -std=gnu11 -O2 -Wall -g \
         -pthread

all: klipper_host_example klipper_host_burst

klipper_host_example: main.c
	$(CC) $(CFLAGS) -o $@ $< -L$(OUT) -lklipper_host -lrt -ldl -pthread

run: klipper_host_example
	LD_LIBRARY_PATH=$(OUT) ./klipper_host_example

# Self-checking buffer test (see burst.c); `make test` runs it.
klipper_host_burst: burst.c
	$(CC) $(CFLAGS) -o $@ $< -L$(OUT) -lklipper_host -lrt -ldl -pthread

# The library is not installed, so point the loader at the build directory.
test: klipper_host_burst
	LD_LIBRARY_PATH=$(OUT) ./klipper_host_burst

clean:
	rm -f klipper_host_example klipper_host_burst

.PHONY: all clean run test
