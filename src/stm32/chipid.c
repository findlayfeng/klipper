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
    if (CONFIG_CANBUS)
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
