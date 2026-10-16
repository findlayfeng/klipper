# Additional linux build rules

dirs-y += src/linux src/generic

src-y += linux/main.c linux/timer.c linux/console.c linux/watchdog.c
src-y += linux/pca9685.c linux/spidev.c linux/analog.c linux/hard_pwm.c
src-y += linux/i2c.c linux/gpio.c generic/crc16_ccitt.c generic/alloc.c
src-y += linux/sensor_ds18b20.c

# The CAN console: generic/canserial.c is the byte stream, generic/canbus.c the
# wrappers around the hardware calls, linux/can.c the SocketCAN backend. The
# fasthash is canserial's own: it derives the device uuid from the raw id the
# command line names (generic/chipid_uuid.h).
src-$(CONFIG_CANSERIAL) += generic/canserial.c generic/canbus.c linux/can.c
src-$(CONFIG_CANSERIAL) += ../lib/fast-hash/fasthash.c
dirs-$(CONFIG_CANSERIAL) += lib/fast-hash
ifeq ($(CONFIG_CANSERIAL),y)
CFLAGS += -Ilib/fast-hash
endif

CFLAGS_klipper.elf += -lutil -lrt -lpthread

flash: $(OUT)klipper.elf
	@echo "  Flashing"
	$(Q)sudo ./scripts/flash-linux.sh $(OUT)
