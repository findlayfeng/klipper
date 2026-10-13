# Additional host library build rules
#
# Copyright (C) 2026  Feng Cheng <i@pfeng.top>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# This file adds host-specific source files to the klipper build system
# and builds the static library for MACH_HOST configurations.
#
# Build:
#   cd third_party/klipper
#   make olddefconfig && make
#
# The static library is at: out/libklipper_host.a
#
# The library provides thread-safe I/O functions:
#   klipper_host_init()       - Initialize the library
#   klipper_host_input()      - Feed input data (thread-safe)
#   klipper_host_output()     - Read output data (thread-safe)
#   klipper_host_run()        - Run the main scheduling loop
#   klipper_host_step()       - Run a single iteration
#   klipper_host_shutdown()   - Request shutdown (thread-safe)
#   klipper_host_is_shutdown()- Check shutdown status
#   klipper_host_get_clock()  - Get current clock time
#
# With CONFIG_BRIDGE (bridge mode) the library also opens a serial port (see
# host/bridge.c) instead of talking only through those functions.
#
# No peripherals (GPIO/SPI/I2C/ADC/PWM/sensors) are implemented.

dirs-y += src/host src/generic

# Host core sources. generic/timer_irq.c provides timer_from_us(),
# timer_is_before(), timer_dispatch_many() and timer_task().
src-y += host/host.c host/console.c host/timer.c
src-y += generic/crc16_ccitt.c generic/alloc.c generic/timer_irq.c

# Bridge mode: this arch's serial port backend (BRIDGE_PORT_SERIAL between
# CONFIG_BRIDGE_PORT_SERIAL) - gives the bridge a downstream port. The router
# itself (CONFIG_BRIDGE) and the device half (CONFIG_MUX) are transport-neutral
# and live outside this directory.
src-$(CONFIG_BRIDGE_PORT_SERIAL) += host/bridge.c

# Link flags for the host library
CFLAGS_klipper.elf += -pthread -lrt -ldl

# For MACH_HOST, build library instead of an executable
# (the library has no main()).
ifneq ($(CONFIG_MACH_HOST),)
# Build static or shared library based on HOST_AR_LIBRARY config
ifdef CONFIG_HOST_AR_LIBRARY
target-y := $(OUT)libklipper_host.a
OBJS_libklipper_host.a = $(OBJS_klipper.elf)

$(OUT)libklipper_host.a: $(OBJS_libklipper_host.a)
	@echo "  Creating static library $@"
	$(Q)rm -f $@
	$(Q)$(AR) rcs $@ $^
else
target-y := $(OUT)libklipper_host.so
# Objects for shared library (need -fPIC)
OBJS_libklipper_host.so := $(patsubst %.o,%.pic.o,$(filter-out $(OUT)compile_time_request.o,$(OBJS_klipper.elf)))
# compile_time_request.o provides command_index/command_identify_* and
# ctr_* symbols required by the console. Build a PIC copy for the .so.
OBJS_libklipper_host.so += $(OUT)compile_time_request.pic.o

# Shared library flags: drop LTO/whole-program so exported symbols are kept.
# -fwhole-program + -flto would garbage-collect all unreferenced globals,
# resulting in an empty .so with no exported klipper_host_* functions.
CFLAGS_libklipper_host.so := $(filter-out -flto=auto -fwhole-program -fno-use-linker-plugin,$(CFLAGS)) -fPIC
LDFLAGS_libklipper_host.so := -pthread -lrt -ldl

$(OUT)src/%.pic.o: src/%.c $(OUT)autoconf.h
	@echo "  Compiling PIC object $@"
	$(Q)$(CC) $(CFLAGS_libklipper_host.so) -c src/$*.c -o $@

# compile_time_request.o rule (in Makefile) generates compile_time_request.c
# and compiles it. Build a PIC copy from that generated source.
$(OUT)compile_time_request.pic.o: $(OUT)compile_time_request.o
	@echo "  Compiling PIC object $@"
	$(Q)$(CC) $(CFLAGS_libklipper_host.so) -c $(OUT)compile_time_request.c -o $@

$(OUT)libklipper_host.so: $(OBJS_libklipper_host.so)
	@echo "  Creating shared library $@"
	$(Q)$(CC) -shared -Wl,-soname,libklipper_host.so -o $@ $^ $(CFLAGS_libklipper_host.so) $(LDFLAGS_libklipper_host.so)
endif
endif
