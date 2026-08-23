// Copyright 2024 yangjun
// SPDX-License-Identifier: GPL-2.0-or-later

// ============================================================================
// Runtime auto-detect pointing driver: one of two modules shares GP0/GP1 + the
// GP2 reset line, and we pick whichever is present at boot.
//
//   - Azoteq TPS43 touchpad : I2C0,  reset ACTIVE-LOW  (low=reset, high=release)
//   - PS/2 TrackPoint        : PIO PS/2, reset ACTIVE-HIGH (high=reset, low=release)
//
// The two modules need OPPOSITE reset polarities on the shared GP2 line, so each
// probe drives its own polarity. We probe I2C first (a connected TrackPoint just
// NAKs); only if no touchpad answers do we switch GP0/GP1 to the PIO PS/2
// function and look for the TrackPoint.
//
// POINTING_DEVICE_DRIVER = custom + SPLIT_POINTING_ENABLE means the report is
// produced on the half that owns the module and forwarded to the USB master, so
// it works no matter which half is plugged in.
// ============================================================================

#include "quantum.h"
#include "pointing_device.h"
#include "i2c_master.h"
#include "sensors/azoteq_iqs5xx.h"
#include "ps2.h"
#include "ps2_mouse.h"

#ifndef AZOTEQ_IQS5XX_ADDRESS
#    define AZOTEQ_IQS5XX_ADDRESS (0x74 << 1)
#endif
#ifndef POINTING_DETECT_RETRIES
#    define POINTING_DETECT_RETRIES 3
#endif

// ---- TPS43 anti-jitter: enable the chip's built-in filters that QMK leaves off -----
// QMK's azoteq driver never touches these registers, so the chip runs with them at
// power-on defaults — which is why a resting finger trembles. We turn them on after
// init. Register addresses + bit layout are cross-verified: QMK's own driver and ZMK's
// (AYM1607/zmk-driver-azoteq-iqs5xx) agree on every register they share
// (0x058E/0x058F/0x0669/0x06B7/0xEEEE), which validates the ZMK-only ones on this chip.
// Refs: ZMK iqs5xx.h/.c, Azoteq IQS5xx-B000 datasheet / AZD087 setup guide.
#define TPS43_REG_FILTER_SETTINGS 0x0632   // MAV / IIR / ALP dynamic-filter enables
#define TPS43_REG_BOTTOM_BETA 0x0637       // low-speed IIR coefficient (steady-finger smoothing)
#define TPS43_REG_STATIONARY_THRESH 0x0672 // px a finger must move to count as "moving" (HW deadzone)
#define TPS43_REG_END_COMMS 0xEEEE         // close the comms window so the chip resumes
#define TPS43_FILTER_IIR 0x01
#define TPS43_FILTER_MAV 0x02
#define TPS43_FILTER_IIR_SELECT 0x04 // set = static IIR; clear (our choice) = dynamic IIR
#define TPS43_FILTER_ALP 0x08
#define TPS43_I2C_TIMEOUT_MS 10

// Tunables (override in config.h if needed):
#ifndef TPS43_STATIONARY_THRESHOLD
#    define TPS43_STATIONARY_THRESHOLD 7 // ZMK ships 5; higher = steadier at rest, but slow moves start later
#endif
#ifndef TPS43_BOTTOM_BETA
#    define TPS43_BOTTOM_BETA 5 // ZMK default; higher = more low-speed smoothing
#endif

typedef enum { MODULE_NONE = 0, MODULE_TPS43, MODULE_TRACKPOINT } pointing_module_t;
static pointing_module_t active_module = MODULE_NONE;

// TPS43 reset: ACTIVE-LOW (pulse low, release high).
static void tps43_reset(void) {
    gpio_set_pin_output(AZOTEQ_RST_PIN);
    gpio_write_pin_low(AZOTEQ_RST_PIN);
    wait_ms(10);
    gpio_write_pin_high(AZOTEQ_RST_PIN);
    wait_ms(100);
}

// TrackPoint reset: ACTIVE-HIGH (pulse high, release low).
static void trackpoint_reset(void) {
    gpio_set_pin_output(TP_RST_PIN);
    gpio_write_pin_high(TP_RST_PIN);
    wait_ms(20);
    gpio_write_pin_low(TP_RST_PIN);
}

// Write one 16-bit-addressed register: {addr_hi, addr_lo, data}, exactly the layout the
// Azoteq expects and that ZMK uses. Uses the low-level i2c_transmit (stable name across
// QMK versions) rather than the i2c_write_register16 helper, whose name differs between
// versions. Retries across a couple of comms windows in case the first attempt lands
// between the sensor's cycles and NAKs.
static void tps43_write_reg8(uint16_t reg, uint8_t val) {
    uint8_t packet[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val};
    for (uint8_t i = 0; i < 3; i++) {
        if (i2c_transmit(AZOTEQ_IQS5XX_ADDRESS, packet, sizeof(packet), TPS43_I2C_TIMEOUT_MS) == I2C_STATUS_SUCCESS) {
            return;
        }
        wait_ms(2);
    }
}

// Turn on the sensor's built-in smoothing. All writes share ONE comms window; the final
// END_COMMS write closes it so the chip resumes normal operation (same handshake QMK's
// driver and ZMK's use). Run once, right after azoteq_iqs5xx_init().
static void tps43_apply_smoothing(void) {
    tps43_write_reg8(TPS43_REG_FILTER_SETTINGS, TPS43_FILTER_IIR | TPS43_FILTER_MAV | TPS43_FILTER_ALP); // dynamic IIR (IIR_SELECT left clear)
    tps43_write_reg8(TPS43_REG_STATIONARY_THRESH, TPS43_STATIONARY_THRESHOLD);
    tps43_write_reg8(TPS43_REG_BOTTOM_BETA, TPS43_BOTTOM_BETA);
    uint8_t end_pkt[3] = {(uint8_t)(TPS43_REG_END_COMMS >> 8), (uint8_t)(TPS43_REG_END_COMMS & 0xFF), 0x00};
    i2c_transmit(AZOTEQ_IQS5XX_ADDRESS, end_pkt, sizeof(end_pkt), TPS43_I2C_TIMEOUT_MS);
}

void pointing_device_driver_init(void) {
    // --- Phase 1: Azoteq TPS43 over I2C ---
    tps43_reset();
    i2c_init();
    for (uint8_t i = 0; i < POINTING_DETECT_RETRIES; i++) {
        if (i2c_ping_address(AZOTEQ_IQS5XX_ADDRESS, 5) == I2C_STATUS_SUCCESS) {
            active_module = MODULE_TPS43;
            break;
        }
        wait_ms(20);
    }
    if (active_module == MODULE_TPS43) {
        azoteq_iqs5xx_init();
        tps43_apply_smoothing(); // enable the chip filters QMK's driver leaves off
        return;
    }

    // --- Phase 2: PS/2 TrackPoint over the RP2040 PIO ---
    trackpoint_reset();
    ps2_host_init();                 // re-mux GP0/GP1 from I2C onto the PIO
    wait_ms(PS2_MOUSE_INIT_DELAY);   // let the module finish POST/BAT

    for (uint8_t i = 0; i < POINTING_DETECT_RETRIES; i++) {
        // Flush the unsolicited power-on BAT (0xAA 0x00) so we read a clean ACK.
        while (pbuf_has_data()) {
            ps2_host_recv();
        }
        ps2_error = PS2_ERR_NONE;
        ps2_host_send(PS2_MOUSE_RESET); // 0xFF
        if (ps2_error == PS2_ERR_NONE) {
            active_module = MODULE_TRACKPOINT;
            break;
        }
        wait_ms(50);
    }

    if (active_module == MODULE_TRACKPOINT) {
        ps2_host_recv_response(); // BAT 0xAA
        ps2_host_recv_response(); // device id 0x00
        // Order matters: after 0xFF the device is in stream mode with reporting
        // DISABLED, which is the only safe window to send a mode command. QMK's own
        // ps2_mouse.c wraps mode changes in PS2_MOUSE_SEND_SAFE (disable reporting →
        // send → re-enable) precisely because a command issued while the device is
        // already streaming can collide with an in-flight movement packet.
        ps2_host_send(PS2_MOUSE_SET_STREAM_MODE);       // 0xEA
        ps2_host_send(PS2_MOUSE_ENABLE_DATA_REPORTING); // 0xF4 — must be last
    }
}

// ---- TrackPoint tunables ------------------------------------------------------------
// Pointer speed multiplier. The stick's raw per-report deltas are small; scale them up.
#ifndef TP_SPEED_MULT
#    define TP_SPEED_MULT 3 // pointer speed multiplier (keymap.c FN_SCROLL_DIVISOR tracks this: 8 * mult)
#endif
// Resting deadzone: drop |delta| <= this per poll. The stick emits a tiny ±1-count idle
// residual. On the half that is USB master it's applied once and is invisible; but when
// the TrackPoint half is the split SLAVE, its report is forwarded to the master and can
// be re-applied across polls, so that ±1 residual accumulates into a slow drift. Gating
// it to 0 removes the thing that gets amplified, so the cursor stays put no matter which
// half is master. Raise to 2 if a faint drift remains; lower to 0 to disable.
#ifndef TP_DEADZONE
#    define TP_DEADZONE 1
#endif

static report_mouse_t trackpoint_get_report(report_mouse_t mouse_report) {
    // The button state must be LATCHED here rather than taken from mouse_report.
    // PS/2 stream mode only emits a packet on movement or a button change, so a
    // held-but-motionless button produces no packets at all. On the split SLAVE
    // the driver is called with a zeroed report every poll
    // (transactions.c pointing_handlers_slave: get_report((report_mouse_t){0})),
    // so seeding from mouse_report.buttons would release a held button the moment
    // the stick stops moving — i.e. the TrackPoint buttons would only work on the
    // half that happens to be plugged into USB.
    static uint8_t latched_buttons = 0;
    static uint8_t pkt[3];      // partial PS/2 packet carried across polls
    static uint8_t pkt_len = 0; // bytes of pkt[] filled so far

    int16_t ax = 0;
    int16_t ay = 0;

    // Drain ONLY the bytes already in the ring buffer: every read is guarded by
    // pbuf_has_data(), so this never blocks waiting for the rest of a packet. That is the
    // fix for touchpad lag when the touchpad half is the USB master — this function runs on
    // the TrackPoint SLAVE inside the master's synchronous fetch transaction, so a blocking
    // mid-packet read here stalls the MASTER's whole pointing loop and its own device (the
    // touchpad) goes janky. A half-received packet is kept in pkt[] and finished next poll.
    // (The old code read the flags byte guarded but then read x and y unguarded.)
    while (pbuf_has_data()) {
        uint8_t b = ps2_host_recv_response();
        if (pkt_len == 0 && !(b & 0x08)) {
            continue; // byte0 must have bit3 set; drop stray bytes to resync
        }
        pkt[pkt_len++] = b;
        if (pkt_len < 3) {
            continue; // wait for the full 3-byte packet (possibly across polls)
        }
        pkt_len = 0;

        uint8_t flags = pkt[0];
        int16_t x     = pkt[1];
        int16_t y     = pkt[2];
        if (flags & (1 << PS2_MOUSE_X_SIGN)) x |= ~0xFF;
        if (flags & (1 << PS2_MOUSE_Y_SIGN)) y |= ~0xFF;
        ax += x;
        ay += y;
        latched_buttons = flags & PS2_MOUSE_BTN_MASK; // L/R/M -> button1/2/3
    }

    // Deadzone ONLY when this (the TrackPoint) half is the split SLAVE. In that case the
    // report is forwarded to the other half (the master), which can re-apply the tiny
    // resting ±residual across polls and turn it into slow drift — so we strip it here
    // before it leaves. When this half is itself the USB master the report is applied
    // directly with no such amplification, so we leave it fully untouched to keep maximum
    // precision. Net effect: the deadzone engages only when the TrackPoint-less half is
    // the master.
    if (!is_keyboard_master()) {
        if (ax <= TP_DEADZONE && ax >= -TP_DEADZONE) ax = 0;
        if (ay <= TP_DEADZONE && ay >= -TP_DEADZONE) ay = 0;
    }

    mouse_report.x       = ax * TP_SPEED_MULT;
    mouse_report.y       = -ay * TP_SPEED_MULT; // PS/2 Y is up-positive; USB HID is down-positive
    mouse_report.buttons = latched_buttons;
    return mouse_report;
}

// ---- TPS43 report shaping: tame sensitivity and resting jitter --------------------
// The bare Azoteq driver returns raw relative deltas with no smoothing, which gives two
// problems on this pad: (1) it is far too sensitive (a small finger move throws the
// cursor a long way); (2) a finger held STILL still emits +/-1-count noise, so the
// cursor trembles. We cannot fix (1) by lowering the sensor resolution, because
// AZOTEQ_IQS5XX_TPS43 hard-#defines the resolution in the driver header (see config.h),
// so we shape the report here instead — which also keeps the full 2048 resolution, so
// real slow movement produces large deltas that a small deadzone can tell apart from the
// noise floor.
//
// Tunables (override in config.h if needed):
#ifndef TPS43_CURSOR_DIVISOR
#    define TPS43_CURSOR_DIVISOR 2 // cursor speed = raw / N  (bigger = slower; 1 = full speed)
#endif
#ifndef TPS43_JITTER_DEADZONE
#    define TPS43_JITTER_DEADZONE 1 // drop resting |cursor delta| <= this (0 disables)
#endif

// ---- Scroll: constant-RATE limiter (not a divisor) ---------------------------------
// Why not a divisor: the driver's r.h/r.v is the RAW per-cycle finger delta during a
// chip-detected scroll gesture, i.e. proportional to finger velocity. A divisor r/N is a
// linear gearbox: it rescales but cannot bound peak speed, so a fast flick still flings
// (the "一下滑过去" overshoot). And it can't fix "small slow move never scrolls" — that
// is the CHIP's scroll_initial_distance gate (see AZOTEQ_IQS5XX_SCROLL_INITIAL_DISTANCE
// in config.h), upstream of us; until the two fingers travel that far, r.h=r.v=0.
//
// Instead we accumulate the raw delta and emit at most ±1 wheel notch per INTERVAL_MS,
// once STEP units have built up. Result: speed is ~constant (INTERVAL_MS sets it) no
// matter how fast the finger moves — a big swipe just scrolls for longer (more notches
// over time), a small swipe scrolls a little. STEP is the onset sensitivity once the
// chip has engaged scroll; CAP bounds the backlog so a lifted finger coasts at most
// ~CAP/STEP notches. NOTE: do NOT reset the accumulator on a single 0-delta cycle — the
// chip holds SCROLL=1 every cycle once engaged (there is no scroll-keep threshold) and a
// genuine slow scroll returns 0 on sub-unit cycles, so a per-cycle reset would wipe the
// backlog mid-scroll and re-break the slow case. Idle carry-over is cleared by a longer
// zero-run timeout in tps43_get_report instead.
#ifndef TPS43_SCROLL_STEP
#    define TPS43_SCROLL_STEP 8 // raw units per notch (onset ~STEP/47.6 mm; bigger = less sensitive/slower)
#endif
#ifndef TPS43_SCROLL_INTERVAL_MS
#    define TPS43_SCROLL_INTERVAL_MS 90 // min ms between notches = top speed (bigger = slower). ~7 notch/s (was 90; slowed 2x)
#endif
#ifndef TPS43_SCROLL_CAP
#    define TPS43_SCROLL_CAP (TPS43_SCROLL_STEP + TPS43_SCROLL_STEP / 2) // backlog clamp -> short coast only
#endif
#ifndef TPS43_SCROLL_IDLE_RESET_MS
#    define TPS43_SCROLL_IDLE_RESET_MS 100 // drop leftover backlog after this long with no scroll input
#endif

// Accumulate raw deltas and emit delta/divisor, carrying the remainder so slow movement
// is preserved instead of truncated to zero. This accumulate-and-divide is itself a
// zero-mean low-pass: oscillating jitter (+1,-1,+1,...) sums back toward 0 and emits
// nothing, while sustained motion accumulates and gets through — so it doubles as a
// jitter filter on top of the explicit deadzone.
static inline int16_t tps43_scale(int16_t delta, int16_t *accum, uint8_t divisor) {
    *accum += delta;
    int16_t out = *accum / divisor;
    *accum -= out * divisor;
    return out;
}

// Constant-rate scroll emitter (see comment block above the tunables). Accumulates raw
// scroll delta, clamps the backlog, and lets through at most ±1 notch per INTERVAL_MS.
// Deliberately does NOT special-case delta==0 (idle carry-over is handled by the
// zero-run timeout in the caller, not per cycle).
static inline int16_t tps43_scroll_rate(int16_t delta, int16_t *accum, uint32_t *last_ms) {
    *accum += delta;
    if (*accum > TPS43_SCROLL_CAP) {
        *accum = TPS43_SCROLL_CAP;
    } else if (*accum < -TPS43_SCROLL_CAP) {
        *accum = -TPS43_SCROLL_CAP;
    }
    if (timer_elapsed32(*last_ms) < TPS43_SCROLL_INTERVAL_MS) {
        return 0; // rate gate: too soon since the last notch
    }
    if (*accum >= TPS43_SCROLL_STEP) {
        *accum -= TPS43_SCROLL_STEP;
        *last_ms = timer_read32();
        return 1;
    }
    if (*accum <= -TPS43_SCROLL_STEP) {
        *accum += TPS43_SCROLL_STEP;
        *last_ms = timer_read32();
        return -1;
    }
    return 0;
}

static report_mouse_t tps43_get_report(report_mouse_t mouse_report) {
    static int16_t  acc_x = 0, acc_y = 0, acc_h = 0, acc_v = 0;
    static uint32_t scroll_notch_h = 0, scroll_notch_v = 0; // last-notch timestamps
    static uint32_t scroll_last_input = 0;                  // last cycle with scroll input

    report_mouse_t r = azoteq_iqs5xx_get_report(mouse_report);

    // Cursor: deadzone the resting noise floor first (so it never enters the
    // accumulator), then slow + smooth. x/y are only non-zero during single-finger
    // movement, so this never touches scroll/zoom.
    int16_t x = r.x;
    int16_t y = r.y;
    if (x <= TPS43_JITTER_DEADZONE && x >= -TPS43_JITTER_DEADZONE) x = 0;
    if (y <= TPS43_JITTER_DEADZONE && y >= -TPS43_JITTER_DEADZONE) y = 0;
    r.x = tps43_scale(x, &acc_x, TPS43_CURSOR_DIVISOR);
    r.y = tps43_scale(y, &acc_y, TPS43_CURSOR_DIVISOR);

    // Scroll: constant-rate limiter (see tps43_scroll_rate). We must NOT reset the
    // backlog on a single 0-delta cycle (the chip keeps SCROLL=1 through sub-unit cycles
    // of a slow scroll). Instead, only after a full zero-run of TPS43_SCROLL_IDLE_RESET_MS
    // — i.e. the gesture has really ended — do we drop leftover backlog so it can't leak
    // a stray notch into the next, possibly opposite-direction, scroll.
    if (r.h != 0 || r.v != 0) {
        scroll_last_input = timer_read32();
    } else if (timer_elapsed32(scroll_last_input) > TPS43_SCROLL_IDLE_RESET_MS) {
        acc_h = 0;
        acc_v = 0;
    }
    r.h = tps43_scroll_rate(r.h, &acc_h, &scroll_notch_h);
    r.v = tps43_scroll_rate(r.v, &acc_v, &scroll_notch_v);

    // buttons (tap / two-finger tap) pass through untouched. (Zoom is disabled in config.h.)
    return r;
}

report_mouse_t pointing_device_driver_get_report(report_mouse_t mouse_report) {
    switch (active_module) {
        case MODULE_TPS43:
            return tps43_get_report(mouse_report);
        case MODULE_TRACKPOINT:
            return trackpoint_get_report(mouse_report);
        default:
            return mouse_report;
    }
}
