SERIAL_DRIVER = vendor

# Custom pointing driver auto-detects TPS43 (I2C) vs PS/2 TrackPoint (PIO) at
# boot. 'custom' skips QMK's automatic driver wiring, so pull in azoteq + I2C
# manually. SPLIT_POINTING (config.h) forwards the report to the USB master.
POINTING_DEVICE_DRIVER = custom
OPT_DEFS += -DMOUSE_EXTENDED_REPORT
I2C_DRIVER_REQUIRED = yes
SRC += drivers/sensors/azoteq_iqs5xx.c
SRC += pointing.c

# PS/2 host layer + RP2040 PIO transport for the TrackPoint. NOT PS2_MOUSE_ENABLE:
# pointing.c drives the PS/2 init/read itself and feeds the pointing device (so
# the data goes through split forwarding instead of ps2_mouse's direct HID send).
PS2_ENABLE = yes
PS2_DRIVER = vendor
