// Copyright 2024 yangjun
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

// Matrix rows: actively drive UNSELECTED rows HIGH instead of releasing them to
// input + pull-up (quantum/matrix.c unselect_row()).
//
// The default high-Z path relies on the RP2040's internal pull-up (~50-80kohm) to
// bring an unselected row back up. That is not enough on the RIGHT half: its first
// two row pins are GP27/GP26 = the MCU's ADC1/ADC0 pads, and on that PCB they carry
// extra analog circuitry (divider/filter) that holds them low once the MCU stops
// driving. Selecting the row still works (push-pull low), so keys register with the
// correct character — but the row never returns high, so every subsequent row window
// still reads that key's column as low. Symptom: press one key, the whole column
// fires. Only GP27/GP26 were affected; the plain digital rows GP22/GP20/GP23 were fine.
//
// Driving unselected rows high fixes it at the source: the pin sources current through
// ~50ohm instead of ~60kohm, and a pressed key then ties its column to a HIGH row, so
// the column reads not-pressed. SAFE ONLY BECAUSE EVERY SWITCH HAS A DIODE (confirmed
// on this PCB) — without diodes, two keys held in the same column on different rows
// would short a high-driven row to a low-driven one through the two switches.
#define MATRIX_UNSELECT_DRIVE_HIGH

// RGB power enable pin (active HIGH — assert before RGB matrix init)
#define RGB_EN_PIN GP11

// Split serial: full-duplex PIO UART, GP4=TX / GP5=RX on BOTH halves.
// The link cable is CROSSED (one half's GP4 goes to the other half's GP5), so
// master and slave use the same pin roles. Do NOT define SERIAL_USART_PIN_SWAP:
// on RP2040 it flips TX/RX on the master only (serial_vendor.c
// serial_transport_driver_master_init), which is for a straight-through
// GP4<->GP4 / GP5<->GP5 cable and would break this wiring.
#define SERIAL_USART_FULL_DUPLEX
#define SERIAL_USART_TX_PIN GP4
#define SERIAL_USART_RX_PIN GP5

// The side that enumerates USB first becomes the split master
#define SPLIT_USB_DETECT

// Recover from the cold-boot race in SPLIT_USB_DETECT.
//
// split_util.c decides master/slave ONCE, by waiting SPLIT_USB_TIMEOUT (2000ms)
// for usb_connected_state() — which means "enumeration finished", not "VBUS is
// present". On a PC cold boot the port is powered as soon as the PSU comes up,
// but the host controller does not enumerate until BIOS/OS is loaded, which is
// far longer than 2s. The plugged half times out, is_keyboard_master_impl()
// declares it a slave AND calls usb_disconnect(), and it never retries — the
// board stays invisible until a reset or a replug.
//
// The watchdog makes a slave-designated half mcu_reset() if no master talks to
// it within SPLIT_WATCHDOG_TIMEOUT (SPLIT_USB_TIMEOUT + 100 = 2100ms), so the
// detection re-runs every ~2s until the host is finally up. Once a master is
// established it pings the slave over the split link and the reboots stop.
#define SPLIT_WATCHDOG_ENABLE

// Handedness stored in EEPROM. Flash each half once with the matching target:
//   qmk flash -kb yj233_split67 -km vial -bl uf2-split-left
//   qmk flash -kb yj233_split67 -km vial -bl uf2-split-right
// (the two halves use different matrix pins — see split.matrix_pins.right in info.json)
#define EE_HANDS

// ---- Pointing module (auto-detected at boot by pointing.c) ----
// Both modules share GP0/GP1 and the GP2 reset line.

// Azoteq TPS43 touchpad over I2C0 (SDA=GP0, SCL=GP1), reset ACTIVE-LOW on GP2.
#define I2C_DRIVER I2CD0
#define I2C1_SDA_PIN GP0
#define I2C1_SCL_PIN GP1
#define AZOTEQ_IQS5XX_TPS43
// Sensor report period in ms. Was 4 (250 Hz), which is faster than the IQS572 can
// actually refresh — the chip then misses cycles (rr_missed) and reads come back
// erratic, which shows up as jitter. 10 ms (100 Hz) is the driver's own default and the
// community norm; POINTING_DEVICE_TASK_THROTTLE_MS below follows it. Lower cautiously.
#define AZOTEQ_IQS5XX_REPORT_RATE 10
#define AZOTEQ_RST_PIN GP2
// Zoom (two-finger pinch) disabled: the chip's gesture engine was reading a slightly
// uneven two-finger scroll as a pinch, so scrolling kept firing zoom by mistake. With
// zoom off, a two-finger gesture always resolves to scroll. (The BUTTON7/8 zoom handler
// in keymap.c was removed too, since the chip no longer emits those bits.)
#define AZOTEQ_IQS5XX_ZOOM_ENABLE false

// --- Tap-to-click sensitivity ---
// Stop a MOVING finger from being misread as a tap/click. tap_distance is the max
// travel (in sensor units) a touch may have and still count as a tap; driver default
// is 0x19 (25 ≈ 0.5 mm), loose enough that a small drag registers as a click. Tighten
// it so any real cursor motion cancels the tap. tap_time is the max touch duration for
// a tap (ms). Lower either further if you still get accidental clicks; if real taps stop
// registering, raise tap_distance back up a little.
// (If you don't want tap-to-click at all — you have physical MBtn1/MBtn2 keys — set
//  AZOTEQ_IQS5XX_TAP_ENABLE to false instead and delete these two lines.)
#define AZOTEQ_IQS5XX_TAP_DISTANCE 0x0A // 10 (was 25) — finger must stay put to click
#define AZOTEQ_IQS5XX_TAP_TIME 0x82     // 130 ms (was 150) — only quick touches count

// --- Two-finger scroll engagement distance ---
// How far (in sensor units, ~47.6/mm on the TPS43 X axis) the two fingers must travel
// before the CHIP declares a scroll gesture and starts emitting h/v. Driver default is
// 0x32 = 50 (~1.05 mm), which felt like "small slow moves don't scroll — you must swipe
// hard to trigger." 0x14 = 20 (~0.42 mm) engages a gentle scroll much sooner. This is a
// separate register from tap_distance (0x0A above), so lowering it does NOT steal a
// two-finger tap / right-click: a tap qualifies only while travel <= 10, which is below
// 20, so a real tap can never cross the scroll threshold. (Trade-off: a sloppy two-finger
// tap that drifts 20-50 units may now emit a tiny stray scroll instead of nothing.)
// Speed once engaged is handled separately in pointing.c (TPS43_SCROLL_*), not here.
#define AZOTEQ_IQS5XX_SCROLL_INITIAL_DISTANCE 0x14

// --- Touchpad orientation ---
// The TPS43 is mounted rotated 180 degrees from the sensor's native axes, so raw finger
// motion came out inverted on BOTH axes (up/down/left/right all reversed). ROTATION_180
// sets the chip's flip_x + flip_y bits (azoteq_iqs5xx.c set_xy_config), negating the
// RELATIVE X/Y delta registers. Those registers feed BOTH the cursor (report.x/y) and
// two-finger scroll (report.h/v), so cursor and scroll flip together and consistently --
// verified against the Azoteq IQS5xx-B000 datasheet sections 5.2.2 / 5.8.
#define AZOTEQ_IQS5XX_ROTATION_180

// Pointer speed and resting-jitter are deliberately NOT tuned here. Defining
// AZOTEQ_IQS5XX_TPS43 (above) already hard-#defines AZOTEQ_IQS5XX_RESOLUTION_X/_Y
// (2048x1792) inside the driver header, with a plain #define the preprocessor sees
// AFTER this file -- so re-#defining the resolution here only triggers a "macro
// redefined" warning and the preset value wins anyway. Instead the report is shaped in
// pointing.c (tps43_get_report): a small deadzone removes the +/-1-count resting tremble
// and an accumulate-and-divide slows the cursor/scroll while preserving slow-move
// precision. Keeping full 2048 resolution is on purpose -- large raw deltas let the
// deadzone cleanly separate real movement from the noise floor. Tunables live there.
// (Do NOT paste the driver's `#if defined(AZOTEQ_IQS5XX_TPS43) ... RESOLUTION ...` block
// into this file: it lives in the driver header, and a copy here both duplicates the
// defines and, if truncated, leaves an unterminated #if that breaks the whole config.)

// PS/2 TrackPoint over the RP2040 PIO (DATA=GP0, CLK=GP1), reset ACTIVE-HIGH on GP2.
// The PIO driver requires clock == data + 1 (GP0/GP1 satisfies it). PIO0 is taken
// by the WS2812 RGB driver and the split serial, so run PS/2 on PIO1.
#define PS2_DATA_PIN GP0
#define PS2_CLOCK_PIN GP1
#define PS2_PIO_USE_PIO1
#define PS2_MOUSE_INIT_DELAY 1000
#define TP_RST_PIN GP2 // same physical pin as AZOTEQ_RST_PIN, opposite polarity

// Split pointing: COMBINED means BOTH halves run the auto-detect probe and both
// can carry a module, so the pointing hardware is fully interchangeable between
// left and right. The slave's report is forwarded to the master and merged with
// the master's own (pointing_device_combine_reports), so the cursor works no
// matter which half holds the module and which half is plugged into USB.
// A half with no module detects nothing and contributes an all-zero report.
// If a module is ever mounted rotated on the right half, correct it with the
// POINTING_DEVICE_ROTATION_*_RIGHT / POINTING_DEVICE_INVERT_*_RIGHT defines.
#define SPLIT_POINTING_ENABLE
#define POINTING_DEVICE_COMBINED

// I2C bus speed. QMK's default for RP2040 is 100kHz; the IQS550 inside the TPS43
// supports 400kHz. Each poll costs 162 clock periods on the wire (a 10-byte base
// data read plus the end-comms write), so this takes a poll from ~1.6ms down to
// ~0.4ms. The transfer blocks the QMK main loop thread (the ChibiOS RP2040 I2C LLD
// suspends the caller), so that time comes straight out of the matrix scan budget.
// If the touchpad ever goes dead or erratic, this is the first thing to revert —
// 400kHz depends on the flex cable length and the bus pull-up values.
#define I2C1_CLOCK_SPEED 400000

// Poll the module no faster than it actually produces data. QMK defaults this to
// 1ms for split pointing, but the TPS43 only refreshes every
// AZOTEQ_IQS5XX_REPORT_RATE ms — polling faster just re-reads a cycle the sensor
// has not updated yet, and burns main-loop time doing it. Keep this tied to the
// report rate rather than to the I2C speed.
#define POINTING_DEVICE_TASK_THROTTLE_MS AZOTEQ_IQS5XX_REPORT_RATE

// NOTE: pointer speed / jitter are shaped on the report in pointing.c (tps43_get_report),
// NOT via a CPI/resolution define here — AZOTEQ_IQS5XX_TPS43 already fixes the sensor
// resolution at 2048x1792 (see the orientation block above). POINTING_DEVICE_DEFAULT_CPI
// is intentionally unused: QMK never reads it (nothing in quantum/ or drivers/ references it).

// RGB matrix brightness cap (25/255 ≈ 10%, keeps total current well under 500mA)
// lives in info.json as rgb_matrix.max_brightness / rgb_matrix.default.val —
// redefining it here only produces "config.h is overwriting info.json" warnings.

// RP2040 bootloader: double-tap the RESET button to enter UF2 mode
#define RP2040_BOOTLOADER_DOUBLE_TAP_RESET
#define RP2040_BOOTLOADER_DOUBLE_TAP_RESET_TIMEOUT 1000U
