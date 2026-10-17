#ifndef __CANBUS_H__
#define __CANBUS_H__

#include <stdint.h> // uint32_t

struct canbus_msg {
    uint32_t id;
    uint32_t dlc;
    union {
        uint8_t data[8];
        uint32_t data32[2];
    };
};

#define CANMSG_ID_RTR (1<<30)
#define CANMSG_ID_EFF (1<<31)

#define CANMSG_DATA_LEN(msg) ((msg)->dlc > 8 ? 8 : (msg)->dlc)

struct canbus_status {
    uint32_t rx_error, tx_error, tx_retries;
    uint32_t bus_state;
};

enum {
    CANBUS_STATE_ACTIVE, CANBUS_STATE_WARN, CANBUS_STATE_PASSIVE,
    CANBUS_STATE_OFF,
};

// callbacks provided by board specific code
int canhw_send(struct canbus_msg *msg);
void canhw_set_filter(uint32_t id);
// Set the rate of the bus this controller is on, in bits per second. Called at
// power-up with the build's CAN rate, and again when a bridge CAN port is opened
// for the rate its host picked (generic/bridge_can.c). Returns 0 on success, -1
// when the driver cannot represent this rate for its clock.
int canhw_set_rate(uint32_t bitrate);
void canhw_get_status(struct canbus_status *status);
// Add one more entry to the hardware receive filter, on top of the ids
// canhw_set_filter() installs for this node.  arb and mask are an 11-bit
// standard arbitration id and its "must match" bits (a set mask bit means the
// corresponding arb bit must be equal).  Returns 0 on success, -1 if no filter
// is left or the driver cannot add filters.  Only called from a
// CONFIG_CANBUS_FILTER_MULTI build, and only implemented by a driver whose
// architecture selects that option.
int canhw_add_filter(uint32_t arb, uint32_t mask);

// canbus.c
int canbus_send(struct canbus_msg *msg);
void canbus_set_filter(uint32_t id);
void canbus_notify_tx(void);
void canbus_process_data(struct canbus_msg *msg);

#endif // canbus.h
