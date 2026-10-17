// Wrapper functions connecting canserial.c to low-level can hardware
//
// Copyright (C) 2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_BRIDGE_PORT_CAN, CONFIG_CANSERIAL
#include "canbus.h" // canhw_send
#include "command.h" // DECL_CONSTANT
#if CONFIG_BRIDGE_PORT_CAN
#include "bridge_can.h" // bridge_can_process_data, bridge_can_notify_tx
#endif
#if CONFIG_CANSERIAL
#include "canserial.h" // canserial_notify_tx
#endif

DECL_CONSTANT("CANBUS_FREQUENCY", CONFIG_CANBUS_FREQUENCY);

int
canbus_send(struct canbus_msg *msg)
{
    return canhw_send(msg);
}

void
canbus_set_filter(uint32_t id)
{
    canhw_set_filter(id);
}

void
canbus_notify_tx(void)
{
#if CONFIG_BRIDGE_PORT_CAN
    // The bridge's CAN downstream port queues its own blobs: a blob the
    // mailboxes could not take waits there until the transmitter frees one.
    bridge_can_notify_tx();
#endif
#if CONFIG_CANSERIAL
    canserial_notify_tx();
#endif
}

void
canbus_process_data(struct canbus_msg *msg)
{
#if CONFIG_BRIDGE_PORT_CAN
    // The frames of the devices behind the bridge's CAN port belong to the
    // router; it answers whether it took the frame. Everything else on the bus
    // is this node's own traffic (or nobody's) and falls through.
    if (bridge_can_process_data(msg))
        return;
#endif
#if CONFIG_CANSERIAL
    canserial_process_data(msg);
#endif
}
