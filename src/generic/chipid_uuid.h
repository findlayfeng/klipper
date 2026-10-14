#ifndef __CHIPID_UUID_H
#define __CHIPID_UUID_H

#include <stdint.h> // uint8_t, uint32_t
#include <string.h> // memcpy
#include "fasthash.h" // fasthash64

// The six-byte device uuid, derived from the raw hardware chip id: the low six
// bytes of fasthash64 over the id, with the seed CAN has always used to build
// its canbus_uuid (generic/canserial.c). This is the one derivation both the
// canbus uuid (canserial_set_uuid) and the serial mux identity (mux_set_uuid)
// are built from, so a board that speaks either wire answers with the same six
// bytes.
#define CHIPID_UUID_LEN 6
#define CHIPID_UUID_SEED 0xA16231A7

static inline void
chipid_uuid(uint8_t *uuid, const void *chip_id, uint32_t chip_id_len)
{
    uint64_t hash = fasthash64(chip_id, chip_id_len, CHIPID_UUID_SEED);
    memcpy(uuid, &hash, CHIPID_UUID_LEN);
}

#endif // chipid_uuid.h
