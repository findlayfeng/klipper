#ifndef __GENERIC_MISC_H
#define __GENERIC_MISC_H

#include <stdarg.h> // va_list
#include <stdint.h> // uint8_t

struct command_encoder;
void console_sendf(const struct command_encoder *ce, va_list args);
void *console_receive_buffer(void);

uint32_t timer_from_us(uint32_t us);
uint8_t timer_is_before(uint32_t time1, uint32_t time2);
uint32_t timer_read_time(void);
void timer_kick(void);

void *dynmem_start(void);
void *dynmem_end(void);

uint16_t crc16_ccitt(uint8_t *buf, uint_fast8_t len);

void bootloader_request(void);

// Restart this firmware. Every arch that defines command_reset defines this
// where it already did - generic/armcm_reset.c, atsam/main.c, ar100/main.c, and
// avr/watchdog.c (which exists only with CONFIG_AVR_WATCHDOG) - and linux's is
// the re-exec its command_config_reset already performed. The mux's
// RESTART_DEVICE command calls it (mux.c). The host library defines it as a
// no-op: the firmware it carries is the host process itself, which only the
// host can restart (host/host.c).
void mcu_restart(void);

#endif // misc.h
