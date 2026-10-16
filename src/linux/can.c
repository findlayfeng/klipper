// Serial over CAN for the linux target, on a SocketCAN interface
//
// Copyright (C) 2026  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <errno.h> // errno
#include <linux/can.h> // struct can_frame, CAN_EFF_MASK
#include <linux/can/raw.h> // CAN_RAW
#include <net/if.h> // if_nametoindex
#include <stdio.h> // fprintf
#include <string.h> // memcpy, memset
#include <sys/socket.h> // socket, bind
#include <unistd.h> // read, write, close
#include "generic/canbus.h" // canbus_process_data
#include "internal.h" // console_set_extra_fd
#include "sched.h" // DECL_TASK

static int can_fd = -1;
static struct task_wake can_rx_wake;

// The buffer command.c resolves a 64-bit host's pointer arguments against
// (command_decode_ptr). The generic/canserial.c byte stream is parsed out of
// its own buffer, so the offset a pointer argument carries was computed against
// a base this one is not: only the commands that pass a pointer (spicmds,
// i2ccmds, ds18b20) depend on it, and none of them works on this target. The
// buffer exists regardless because command.c asks for it in every build.
static uint8_t can_receive_buf[4096];

void *
console_receive_buffer(void)
{
    return can_receive_buf;
}

// Open the SocketCAN interface named on the command line.
//
// The interface has to exist already, and its bit timing is the interface's own
// - it is set with `ip link`, and a vcan interface has none at all. That is why
// no rate is configured here: the canbus frequency constant is only what this
// build reports to the host.
int
can_console_setup(const char *ifname)
{
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = if_nametoindex(ifname);
    if (!addr.can_ifindex) {
        fprintf(stderr, "Unknown CAN interface '%s'\n", ifname);
        return -1;
    }
    int fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        report_errno("socket can", fd);
        return -1;
    }
    int ret = set_non_blocking(fd);
    if (ret)
        goto fail;
    ret = set_close_on_exec(fd);
    if (ret)
        goto fail;
    ret = bind(fd, (struct sockaddr *)&addr, sizeof(addr));
    if (ret) {
        report_errno("bind can", ret);
        goto fail;
    }
    can_fd = fd;
    console_set_extra_fd(fd);
    return 0;

fail:
    close(fd);
    return -1;
}

// Transmit a packet
int
canhw_send(struct canbus_msg *msg)
{
    uint32_t len = CANMSG_DATA_LEN(msg);
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = msg->id & CAN_EFF_MASK;
    frame.can_dlc = len;
    memcpy(frame.data, msg->data, len);
    int ret = write(can_fd, &frame, sizeof(frame));
    if (ret < 0) {
        if (errno == EAGAIN)
            // The socket's queue is full: report "nothing sent" so the caller's
            // tx task keeps this frame and retries it on its next pass.
            return 0;
        report_errno("write can", ret);
        return -1;
    }
    return len;
}

// Setup the receive packet filter
//
// Nothing to do: the socket takes every frame on the interface, and
// canserial_process_data already reads only the ids that are this node's own.
// Narrowing the kernel's filter to an id instead would drop the assignment
// itself, which arrives on the admin channel while the node still has no id.
void
canhw_set_filter(uint32_t id)
{
}

// Report interface status
void
canhw_get_status(struct canbus_status *status)
{
    memset(status, 0, sizeof(*status));
    status->bus_state = CANBUS_STATE_ACTIVE;
}

// Wake the receive task. console_sleep() polls the socket and is the only
// source of CAN rx on this target: there is no interrupt to do it.
void
can_rx_notify(void)
{
    sched_wake_task(&can_rx_wake);
}

// Move the frames the socket holds into canserial's byte stream. Non-blocking:
// the loop drains the socket's receive queue and stops at EAGAIN, so one
// scheduler pass never waits on the bus.
void
can_rx_task(void)
{
    if (!sched_check_wake(&can_rx_wake))
        return;
    for (;;) {
        struct can_frame frame;
        int ret = read(can_fd, &frame, sizeof(frame));
        if (ret < 0) {
            if (errno == EAGAIN)
                // Nothing left in the queue this pass.
                return;
            report_errno("read can", ret);
            return;
        }
        if (ret < (int)sizeof(frame))
            // A datagram arrives all or nothing: anything shorter is not a
            // frame, and nothing says which id its bytes were for.
            return;
        struct canbus_msg msg;
        msg.id = frame.can_id & CAN_EFF_MASK;
        msg.dlc = frame.can_dlc;
        memcpy(msg.data, frame.data, CANMSG_DATA_LEN(&msg));
        canbus_process_data(&msg);
    }
}
DECL_TASK(can_rx_task);
