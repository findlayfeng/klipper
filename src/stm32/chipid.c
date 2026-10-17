// Support for extracting the hardware chip id on stm32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "generic/canserial.h" // canserial_set_uuid
#include "generic/usb_cdc.h" // usb_fill_serial
#include "generic/usbstd.h" // usb_string_descriptor
#include "internal.h" // UID_BASE
#include "sched.h" // DECL_INIT
#if CONFIG_MUX_SERIAL
#include "generic/chipid_uuid.h" // chipid_uuid
#include "mux.h" // mux_set_uuid, MUX_UUID_LEN
#endif

#define CHIP_UID_LEN 12

static struct {
    struct usb_string_descriptor desc;
    uint16_t data[CHIP_UID_LEN * 2];
} cdc_chipid;

struct usb_string_descriptor *
usbserial_get_serialid(void)
{
   return &cdc_chipid.desc;
}

void
chipid_init(void)
{
    if (CONFIG_USB_SERIAL_NUMBER_CHIPID)
        usb_fill_serial(&cdc_chipid.desc, ARRAY_SIZE(cdc_chipid.data)
                        , (void*)UID_BASE);
    // The canbus uuid is the CAN node's identity (generic/canserial.c), so it is
    // set only in a build that has that node: a bridge's CAN downstream port is
    // a port of the router and not a node (bridge_can.c), so CONFIG_CANBUS stays
    // off there and the driver comes in with BRIDGE_PORT_CAN instead. USBCANBUS
    // carries the node over USB (generic/usb_canbus.c), which is the other build
    // CANSERIAL is not selected in.
    if (CONFIG_CANSERIAL || CONFIG_USBCANBUS)
        canserial_set_uuid((void*)UID_BASE, CHIP_UID_LEN);
#if CONFIG_MUX_SERIAL
    {
        uint8_t uuid[MUX_UUID_LEN];
        chipid_uuid(uuid, (void*)UID_BASE, CHIP_UID_LEN);
        mux_set_uuid(uuid);
    }
#endif
}
DECL_INIT(chipid_init);
