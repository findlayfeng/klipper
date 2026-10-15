// Main starting point for micro-controller code running on linux systems
//
// Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <sched.h> // sched_setscheduler sched_get_priority_max
#include <stdio.h> // fprintf
#include <string.h> // memset
#include <unistd.h> // getopt
#include <sys/mman.h> // mlockall MCL_CURRENT MCL_FUTURE
#include "autoconf.h" // CONFIG_MUX_SERIAL
#include "board/misc.h" // console_sendf
#include "command.h" // DECL_CONSTANT
#include "internal.h" // console_setup
#if CONFIG_MUX_SERIAL
#include "mux.h" // mux_set_uuid, MUX_UUID_LEN
#endif
#include "sched.h" // sched_main

DECL_CONSTANT_STR("MCU", "linux");


/****************************************************************
 * Real-time setup
 ****************************************************************/

static int
realtime_setup(void)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO) / 2;
    int ret = sched_setscheduler(0, SCHED_FIFO, &sp);
    if (ret < 0) {
        report_errno("sched_setscheduler", ret);
        return -1;
    }
    // Lock ourselves into memory
    ret = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (ret) {
        report_errno("mlockall", ret);
        return -1;
    }
    return 0;
}


/****************************************************************
 * Restart
 ****************************************************************/

static char **orig_argv;

void
command_config_reset(uint32_t *args)
{
    if (! sched_is_shutdown())
        shutdown("config_reset only available when shutdown");
    int ret = execv(orig_argv[0], orig_argv);
    report_errno("execv", ret);
}
DECL_COMMAND_FLAGS(command_config_reset, HF_IN_SHUTDOWN, "config_reset");


/****************************************************************
 * Startup
 ****************************************************************/

#if CONFIG_MUX_SERIAL
#define GETOPT_OPTS "wrI:U:"
#define USAGE_FORMAT "Usage: %s [-w] [-r] [-I path] [-U uuid]\n"
#else
#define GETOPT_OPTS "wrI:"
#define USAGE_FORMAT "Usage: %s [-w] [-r] [-I path]\n"
#endif

#if CONFIG_MUX_SERIAL
// One hex digit's value, or -1 when the character is not one.
static int
hex_digit(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

// Decode the twelve hex digits a caller names with `-U` into the device's
// six-byte uuid. No "0x" prefix, either case; -1 on anything else.
static int
parse_uuid(const char *hex, uint8_t *uuid)
{
    int i;
    for (i = 0; i < MUX_UUID_LEN; i++) {
        int hi = hex_digit(hex[i * 2]), lo = hex_digit(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0)
            return -1;
        uuid[i] = (hi << 4) | lo;
    }
    return hex[MUX_UUID_LEN * 2] ? -1 : 0;
}
#endif // CONFIG_MUX_SERIAL

int
main(int argc, char **argv)
{
    // Parse program args
    orig_argv = argv;
    int opt, watchdog = 0, realtime = 0;
    char *serial = "/tmp/klipper_host_mcu";
#if CONFIG_MUX_SERIAL
    uint8_t uuid[MUX_UUID_LEN];
    int have_uuid = 0;
#endif
    while ((opt = getopt(argc, argv, GETOPT_OPTS)) != -1) {
        switch (opt) {
        case 'w':
            watchdog = 1;
            break;
        case 'r':
            realtime = 1;
            break;
        case 'I':
            serial = optarg;
            break;
#if CONFIG_MUX_SERIAL
        case 'U':
            if (parse_uuid(optarg, uuid)) {
                fprintf(stderr, USAGE_FORMAT, argv[0]);
                return 1;
            }
            have_uuid = 1;
            break;
#endif
        default:
            fprintf(stderr, USAGE_FORMAT, argv[0]);
            return -1;
        }
    }

    // Initial setup
    if (realtime) {
        int ret = realtime_setup();
        if (ret)
            return ret;
    }
#if CONFIG_MUX_SERIAL
    // The identity has to be in place before the console comes up: it is what
    // this device answers a discovery scan with, and what an assignment names.
    // Without `-U` it stays all zero, so a device nobody named answers as one.
    if (have_uuid)
        mux_set_uuid(uuid);
#endif
    int ret = console_setup(serial);
    if (ret)
        return -1;
    if (watchdog) {
        int ret = watchdog_setup();
        if (ret)
            return ret;
    }

    // Main loop
    sched_main();
    return 0;
}
