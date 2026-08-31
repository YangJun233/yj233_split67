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

// Boot-time expansion kill switch, implemented in yj233_split67.c. True when THIS
// half came up with its kill key held (left = Delete, right = Backspace), meaning
// the module must stay dark for this boot. Declared here rather than in a shared
// header because this keyboard has no <keyboard>.h and adding one would change
// what QMK_KEYBOARD_H resolves to for the keymaps.
bool yj_expansion_killed(void);
void yj_expansion_kill_indicate(void);

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
    // Boot override: this half was reset with its kill key held, so do not touch
    // the module at all. Returning with active_module == MODULE_NONE is the same
    // state a half with no module soldered on ends up in, which is a path this
    // firmware already relies on -- GP0/GP1/GP2 stay unconfigured (high-Z), no I2C
    // or PS/2 traffic is ever generated, and pointing_device_driver_get_report()
    // hands the report straight back. Since pointing_device_send() re-zeroes the
    // report every cycle (quantum/pointing_device/pointing_device.c:218), this
    // half's contribution to the USB mouse report is identically zero -- so the
    // data is neither read from the module nor emitted to the host.
    if (yj_expansion_killed()) {
        active_module = MODULE_NONE;
        yj_expansion_kill_indicate(); // blink that key's led: "expansion off, you can let go"
        return;
    }

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
// Scaling the CURSOR here also scales what keymap.c's Fn one-finger scroll receives, since
// that gesture is built from the cursor delta -- FN_SCROLL_DIVISOR is 16 * this. Unrelated
// to the two-finger scroll, which is shaped by TPS43_SCROLL_* below and never touches x/y.
#    define TP_SPEED_MULT 3 // pointer speed multiplier
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

// ---- PS/2 packet decoding ------------------------------------------------------------
// The PIO driver signals a bad frame (start / parity / stop) by setting ps2_error and
// returning 0x00 -- at the return value a corrupted frame is indistinguishable from a
// genuine 0x00 data byte. Ignoring ps2_error therefore corrupts the stream in two ways:
// a rejected frame landing mid-packet is consumed as a real X or Y delta, and one landing
// at packet-start returns 0x00, fails the header test and is silently skipped -- which
// DROPS A BYTE, because the device really did transmit one in that slot. Every following
// byte is then read one position early and the packet phase stays shifted. So we check
// ps2_error on every read and abandon the packet when a frame is bad.
//
// If the pointer ever misbehaves again, trackpoint-diagnostics.md documents how to
// re-add the instrumentation that measures this (frame counters + raw packet log) and
// how to read the numbers. Diagnosed once on 2026-08-27; see that file for the case
// record.

// A partial packet is NORMAL: at a 10 ms poll interval the module's 3-byte burst is often
// split across two polls, so the timeout must comfortably exceed one poll period. It is
// only stale if nothing follows for far longer than that, meaning the byte stream stopped
// mid-packet -- drop it so leftover bytes cannot shift the phase of the next real packet.
#ifndef TP_PKT_IDLE_TIMEOUT_MS
#    define TP_PKT_IDLE_TIMEOUT_MS 30
#endif

static report_mouse_t trackpoint_get_report(report_mouse_t mouse_report) {
    // The button state must be LATCHED here rather than taken from mouse_report.
    // PS/2 stream mode only emits a packet on movement or a button change, so a
    // held-but-motionless button produces no packets at all. On the split SLAVE
    // the driver is called with a zeroed report every poll
    // (transactions.c pointing_handlers_slave: get_report((report_mouse_t){0})),
    // so seeding from mouse_report.buttons would release a held button the moment
    // the stick stops moving -- i.e. the TrackPoint buttons would only work on the
    // half that happens to be plugged into USB.
    static uint8_t  latched_buttons = 0;
    static uint8_t  pkt[3];           // partial PS/2 packet carried across polls
    static uint8_t  pkt_len      = 0; // bytes of pkt[] filled so far
    static uint32_t last_byte_ms = 0; // when the last good byte arrived (staleness check)

    int16_t ax = 0;
    int16_t ay = 0;

    // Discard a partial packet whose remaining bytes never arrived.
    if (pkt_len != 0 && timer_elapsed32(last_byte_ms) > TP_PKT_IDLE_TIMEOUT_MS) {
        pkt_len = 0;
    }

    // Drain ONLY the bytes already in the ring buffer: every read is guarded by
    // pbuf_has_data(), so this never blocks waiting for the rest of a packet. That is the
    // fix for touchpad lag when the touchpad half is the USB master -- this function runs on
    // the TrackPoint SLAVE inside the master's synchronous fetch transaction, so a blocking
    // mid-packet read here stalls the MASTER's whole pointing loop and its own device (the
    // touchpad) goes janky. A half-received packet is kept in pkt[] and finished next poll.
    while (pbuf_has_data()) {
        // Clear ps2_error first: the driver only ever SETS it and nothing clears it, so a
        // stale error left by an earlier call would make every later byte look corrupt.
        ps2_error = PS2_ERR_NONE;

        uint8_t b = ps2_host_recv_response();
        if (ps2_error != PS2_ERR_NONE) {
            // Corrupted frame. Its value (0x00) carries no information, so it must not be
            // fed into the packet -- and it must not be silently skipped either, because
            // the device really did transmit a byte in this slot. Abandoning the whole
            // packet keeps the phase honest: we deliberately resync on the next header.
            pkt_len = 0;
            continue;
        }
        last_byte_ms = timer_read32();

        if (pkt_len == 0) {
            // Packet-boundary hunt. bit3 is always 1 in a mouse byte0; bits 6/7 are the
            // X/Y overflow flags, which a TrackPoint essentially never sets. Testing all
            // three cuts the chance of a random data byte being mistaken for a header from
            // ~1/2 down to ~1/8, so a stream that lost a byte re-locks far sooner instead
            // of staying shifted indefinitely. An overflowed packet is dropped too -- its
            // deltas are meaningless by definition.
            if ((b & 0xC8) != 0x08) {
                continue;
            }
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
    // resting +/-residual across polls and turn it into slow drift -- so we strip it here
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
// Resting-jitter deadzone. NOW OFF BY DEFAULT, and it must stay off unless resting tremble
// actually comes back. The clip runs BEFORE the accumulator, so whatever it zeroes is gone for
// good -- it does not survive as a remainder. During slow movement the chip's per-cycle delta IS
// mostly +/-1 (a finger at 2 mm/s covers 0.02 mm per 10 ms cycle = 0.95 counts at 47.6 counts/mm),
// so a value of 1 threw away nearly the whole slow-motion signal: the cursor sat still, then
// jumped on the occasional cycle that happened to report 2. That is the "移动不均匀 / 不跟手"
// symptom this is set to 0 to fix.
//
// Resting tremble is still handled -- by the accumulate-and-divide in tps43_scale, which is a
// zero-mean low-pass: oscillating noise (+1,-1,+1,...) sums back toward zero and emits nothing,
// while sustained motion in one direction accumulates and gets through. That filter keeps the
// information; the deadzone destroyed it. If tremble does return, prefer raising
// TPS43_CURSOR_DIVISOR before setting this back to 1.
#ifndef TPS43_JITTER_DEADZONE
#    define TPS43_JITTER_DEADZONE 0 // drop resting |cursor delta| <= this (0 disables)
#endif

// ---- TWO-FINGER scroll: constant-RATE limiter (not a divisor) ----------------------
// SCOPE: this shapes the chip's two-finger scroll gesture ONLY, whose job on this board is
// small, precise scrolling. The Fn + one-finger gesture is a different feature for
// long-distance scrolling, lives in keymaps/vial/keymap.c, is built from x/y rather than
// h/v, and shares none of the constants below. Tune the two separately; do not try to make
// their speeds match.
//
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
//
// SPEED IS SET BY TWO PARAMETERS, EACH OWNING A DIFFERENT REGIME -- change both together
// or you only slow down half the range:
//   - slow finger (accumulator cannot outrun the gate): rate = finger_speed / STEP, so
//     scroll-per-millimetre-of-finger is 1/STEP. STEP alone sets this regime.
//   - fast finger (backlog pinned at CAP): rate = 1000 / INTERVAL_MS, a hard ceiling.
//     INTERVAL_MS alone sets this regime.
// Scaling both by the same factor moves the whole curve uniformly. History: original 8 / 90
// -> 12 / 135 (x1.5, every regime to 2/3 speed) -> now 6 / 68, i.e. both halved from 12 / 135
// so the whole curve is exactly 2x the previous speed (top speed 7.4 -> 14.7 notch/s, and
// scroll-per-millimetre doubled as well). To change speed again, keep multiplying BOTH by the
// same factor -- and re-check the IDLE_RESET_MS > INTERVAL_MS ordering noted below.
#ifndef TPS43_SCROLL_STEP
#    define TPS43_SCROLL_STEP 6 // raw units per notch (onset ~STEP/47.6 mm; bigger = less sensitive/slower)
#endif
#ifndef TPS43_SCROLL_INTERVAL_MS
#    define TPS43_SCROLL_INTERVAL_MS 68 // min ms between notches = top speed ceiling (bigger = slower). ~14.7 notch/s
#endif
#ifndef TPS43_SCROLL_CAP
#    define TPS43_SCROLL_CAP (TPS43_SCROLL_STEP + TPS43_SCROLL_STEP / 2) // backlog clamp -> short coast only (tracks STEP)
#endif
// MUST stay greater than TPS43_SCROLL_INTERVAL_MS. This timeout clears the backlog after a
// zero-run of raw input, while the rate gate withholds a notch for INTERVAL_MS. If the
// timeout were the shorter of the two, a backlog that is already >= STEP would be wiped
// before the gate ever let it out, silently swallowing the last notch of every flick. That
// was not a live bug at the original 90/100 (gate < timeout), but raising INTERVAL_MS to
// 135 crossed over it, so this moved to 150 to restore the ordering. At the current 68 the
// margin is wide again (150 > 68); leave it at 150 -- lowering it toward INTERVAL_MS would
// re-open the swallowed-last-notch bug.
#ifndef TPS43_SCROLL_IDLE_RESET_MS
#    define TPS43_SCROLL_IDLE_RESET_MS 150 // drop leftover backlog after this long with no scroll input
#endif

// ---- Click stretch: make the one-cycle tap pulse survive the whole pipeline ----------
// The chip reports single_tap / two_finger_tap as an EVENT that is set for exactly ONE
// sensor cycle, and azoteq_iqs5xx_get_report turns that into buttons != 0 for exactly one
// pointing cycle (~10 ms) and back to 0 on the next. press_and_hold is disabled (the
// driver defaults AZOTEQ_IQS5XX_PRESS_AND_HOLD_ENABLE to false and config.h does not
// override it), so a tap is a ~10 ms blip and nothing else -- there is no held state to
// fall back on. That blip has to survive two independent samplers:
//
//   1. Split forwarding. When the pad is on the SLAVE half, this function runs in the
//      slave's transaction handler at its own ~10 ms throttle, writing into split_shmem.
//      The MASTER copies that into shared_mouse_report but only CONSUMES it inside
//      pointing_device_task, which is itself throttled to POINTING_DEVICE_TASK_THROTTLE_MS
//      (pointing_device.c: the COMBINED branch reads shared_mouse_report once per task).
//      Two ~10 ms samplers running free of each other beat against one another: whenever
//      the phases line up badly the pressed report is overwritten by the next (released)
//      one before the master ever looks at it, and the click is dropped with no trace.
//      That is the "sometimes it just doesn't click" failure, and it is phase-dependent,
//      so it comes in runs rather than at random.
//   2. The USB host, which sees a press and a release ~10 ms apart.
//
// Holding the buttons for a minimum of TPS43_CLICK_MIN_MS makes the pressed state wider
// than either sampler's period, so neither can step over it. Cost: a click is
// TPS43_CLICK_MIN_MS long instead of ~10 ms -- invisible to the user, and far below any
// plausible double-tap interval (two finger LIFTS 50 ms apart is not physically
// reachable), so double-click still resolves as two clicks.
//
// This does NOT rescue a tap the chip never recognised in the first place -- that is what
// AZOTEQ_IQS5XX_TAP_TIME / _TAP_DISTANCE in config.h control. The two fixes are
// independent and address different halves of the same symptom.
#ifndef TPS43_CLICK_MIN_MS
#    define TPS43_CLICK_MIN_MS 50 // minimum click duration in ms (0 disables the stretch)
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

// Build the mouse report from ONE base-data read, keeping the finger count. This is what
// azoteq_iqs5xx_get_report() does, minus the part where it drops number_of_fingers before
// returning. The finger count is what the click stretch below uses to decide a new touch has
// started -- it cannot use the shaped x/y for that, because TPS43_JITTER_DEADZONE is 0 and
// those carry the resting +/-1 tremble (see the click-stretch comment). Taking both out of the
// same single I2C transaction also means the count and the gesture bits cannot disagree.
// Only the gestures this board actually enables are decoded: swipes
// (AZOTEQ_IQS5XX_SWIPE_X/Y_ENABLE) and zoom (AZOTEQ_IQS5XX_ZOOM_ENABLE) are false, and
// press_and_hold defaults false (verified in drivers/sensors/azoteq_iqs5xx.c), so those
// branches would be dead code here. The else-if ordering and the number_of_fingers == 1 guard
// on cursor motion are kept identical to the driver's, so nothing about the existing behaviour
// changes.
// Returns false if the cycle carried no usable data; on such a cycle the report and the finger
// count are both zeroed, which the click stretch treats as "keep stretching" -- the safe way
// round for a NAK.
static bool tps43_read_raw(report_mouse_t *out, uint8_t *fingers_out) {
    report_mouse_t            r    = {0};
    azoteq_iqs5xx_base_data_t base = {0};

    *fingers_out = 0;
    *out         = r;
    if (azoteq_iqs5xx_get_base_data(&base) != I2C_STATUS_SUCCESS) {
        return false; // NAK or bad read: no information this cycle, state must not advance
    }
    *fingers_out = base.number_of_fingers;

    int16_t raw_x = AZOTEQ_IQS5XX_COMBINE_H_L_BYTES(base.x.h, base.x.l);
    int16_t raw_y = AZOTEQ_IQS5XX_COMBINE_H_L_BYTES(base.y.h, base.y.l);

    if (base.gesture_events_0.single_tap) {
        r.buttons = pointing_device_handle_buttons(r.buttons, true, POINTING_DEVICE_BUTTON1);
    } else if (base.gesture_events_1.two_finger_tap) {
        r.buttons = pointing_device_handle_buttons(r.buttons, true, POINTING_DEVICE_BUTTON2);
    } else if (base.gesture_events_1.scroll) {
        r.h = CONSTRAIN_HID(raw_x);
        r.v = CONSTRAIN_HID(raw_y);
    }
    if (base.number_of_fingers == 1) {
        r.x = CONSTRAIN_HID_XY(raw_x);
        r.y = CONSTRAIN_HID_XY(raw_y);
    }
    *out = r;
    return true;
}

static report_mouse_t tps43_get_report(report_mouse_t mouse_report) {
    static int16_t  acc_x = 0, acc_y = 0, acc_h = 0, acc_v = 0;
    static uint32_t scroll_notch_h = 0, scroll_notch_v = 0; // last-notch timestamps
    static uint32_t scroll_last_input = 0;                  // last cycle with scroll input

    // mouse_report is unused: the chip's report is absolute-per-cycle, and the driver it
    // replaced ignored the incoming buttons too (it built its report from {0}).
    (void)mouse_report;

    uint8_t        fingers = 0;
    report_mouse_t r       = {0};
    // The return value distinguishes a failed read from a genuinely idle cycle. Nothing needs
    // that distinction any more: a failed read zeroes both the report and the finger count, and
    // the click stretch below treats fingers == 0 as "continue stretching", which is the safe
    // direction for a NAK. Discarded explicitly so it does not read as an oversight.
    (void)tps43_read_raw(&r, &fingers);

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

    // Buttons: stretch the chip's one-cycle tap event to at least TPS43_CLICK_MIN_MS (see
    // the comment block above the tunable). While the raw report still asserts a button we
    // keep re-arming the timer, so a genuinely held button (if press_and_hold is ever
    // enabled) is passed through and only its RELEASE is delayed by up to CLICK_MIN_MS.
    // State is static and touched only from the pointing task / split transaction handler
    // -- never from an ISR -- so no locking is needed.
#if TPS43_CLICK_MIN_MS > 0
    static uint8_t  held_buttons = 0;
    static uint32_t held_since   = 0;
    if (r.buttons != 0) {
        held_buttons |= r.buttons;
        held_since = timer_read32();
    } else if (held_buttons != 0) {
        // End the stretch as soon as a NEW finger is on the pad, so the click hands over
        // cleanly to whatever comes next (a drag, or a second click).
        //
        // This used to test the shaped deltas (x/y) instead, on the reasoning that the
        // deadzone had already removed the resting noise floor. TPS43_JITTER_DEADZONE is 0
        // now -- it was destroying slow movement -- so x/y carry the raw +/-1 tremble again
        // and that test would fire on noise, cutting every click back to one cycle and
        // re-opening the dropped-click problem CLICK_MIN_MS exists to fix. The finger count
        // is the signal actually wanted here and it does not depend on any shaping stage.
        // On a failed read fingers is 0, so the stretch simply continues -- the safe way round.
        if (fingers >= 1) {
            held_buttons = 0;
        } else if (timer_elapsed32(held_since) >= TPS43_CLICK_MIN_MS) {
            held_buttons = 0;
        }
    }
    r.buttons = held_buttons;
#endif
    // (Zoom is disabled in config.h, so BUTTON7/8 never appear here.)
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
