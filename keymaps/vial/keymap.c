// Copyright 2024 yangjun
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

// Hold Fn (the MO(1) layer) and move the pointing device to scroll instead of moving the
// cursor. Direction and speed follow the stick's force (its x/y motion). Runs on the USB
// master where the layer state is authoritative, so it works no matter which half owns
// the TrackPoint. Bigger FN_SCROLL_DIVISOR = slower (force still controls relative speed).
// Relates to pointing.c's TP_SPEED_MULT (currently 3): divisor = 16 * TP_SPEED_MULT gives
// scroll = raw/16 — i.e. half the previous raw/8 speed (slowed 2x per request). To keep a
// given scroll feel when TP_SPEED_MULT changes, scale this divisor by the same factor.
#define FN_LAYER          1
#define FN_SCROLL_DIVISOR 48

// Y direction of the Fn+move scroll. The two pointing modules want OPPOSITE signs here,
// because their "natural" scroll conventions genuinely differ:
//
//   TOUCHPAD (-1 is wrong, use +1): the chip's two-finger scroll feeds report.v straight
//     from the Y delta with no negation (azoteq_iqs5xx.c: temp_report.v = COMBINE(y)),
//     and the SAME Y delta also becomes report.y. In HID terms y>0 = cursor down and v>0 =
//     scroll up, so two-finger scrolling on this pad means "finger down -> scroll up".
//     Fn+one-finger has to match that or the two gestures fight each other.
//   TRACKPOINT (+1 is wrong, use -1): pointing.c already converts the stick to HID sense
//     (y>0 = down), so pushing the stick UP gives y<0. The classic stick convention is
//     "push up -> scroll up", which needs the negation.
//
// This is a compile-time choice, not runtime: the Fn scroll runs in
// pointing_device_task_user on the MASTER, over the already-combined report, and when the
// module sits on the slave half the master has no idea which module produced those deltas
// (pointing.c's active_module is per-half and is MODULE_NONE on a module-less master).
// Making it automatic would need a custom split transaction to publish the slave's module
// type. Currently set for the touchpad; flip to -1 if you swap the TrackPoint back in.
#define FN_SCROLL_Y_SIGN  1

// ---- Fn 滚动惯性 (inertial / momentum scroll) --------------------------------------
// Flick and lift: the wheel keeps turning after the finger leaves the pad and decays
// smoothly to a stop instead of cutting off dead. Shape:
//
//   launch speed = (finger speed at lift) * GAIN, clamped to MAX, dropped below MIN
//   decay        = exponential, v -= v*dt/TAU every cycle  ->  v(t) = v0 * e^(-t/TAU)
//   stop         = as soon as v falls under STOP
//
// so the ride lasts TAU * ln(v0/STOP): 5.0 s from a full-speed flick (10.00 -> 0.35
// notch/s, 13 notches) and 3.3 s from the gentlest flick that still qualifies. Speeds
// are held as milli-notches per second (mnps) in int32, so all of this is integer math
// and there is no float anywhere: 1000 mnps = one wheel notch per second.
//
// Why GAIN is 1/4: a real flick across this pad lifts at 30-50 notch/s of raw Fn-scroll
// rate (the pad is ~47.6 counts/mm, halved by TPS43_CURSOR_DIVISOR, then /FN_SCROLL_DIVISOR
// = about 0.5 notch per mm of finger travel). Coasting at that speed would fling the page;
// a quarter of it lands in the 4-10 notch/s band, which is around the touchpad's own
// two-finger top speed (TPS43_SCROLL_INTERVAL_MS = 68 ms -> 14.7 notch/s) and reads as
// "keeps going, doesn't run away". MIN then means "only a deliberate flick coasts": a drag
// lifting slower than MIN/GAIN = 12 notch/s (~24 mm/s) just stops where you left it.
//
// Velocity is measured from the raw per-cycle deltas, NOT from emitted notches: the wheel
// is quantised to whole notches, so counting notches over the last few cycles is far too
// coarse at these rates. Every cycle carrying motion contributes raw-units/ms through a
// 1/4 EMA (~40 ms of memory) instead, which is fine-grained and already smoothed.
//
// Finger-lift detection is "no cursor delta for FN_COAST_LIFT_MS". The pointing task runs
// once per AZOTEQ_IQS5XX_REPORT_RATE (10 ms), so 25 ms is 2-3 dead cycles. There is no
// finger-count bit to consult on this side -- with POINTING_DEVICE_COMBINED only
// x/y/h/v/buttons cross the split, and the touch count stays on whichever half owns the
// pad. It does not need one: a finger resting ON the pad is never perfectly still (
// pointing.c keeps the resting tremble on purpose, TPS43_JITTER_DEADZONE is 0), so
// "stop moving but keep touching" is self-correcting -- the EMA has already decayed under
// MIN by then, and any tremble cycle cancels a coast that did start.
#define FN_COAST_ENABLE                  // comment out to get the old hard-stop scroll back
#define FN_COAST_GAIN_NUM      1         // launch speed = lift speed * NUM/DEN ...
#define FN_COAST_GAIN_DEN      4         // ... 1/4 of the flick, see above
#define FN_COAST_MIN_MNPS      3000      // below 3.00 notch/s after gain: no coast at all
#define FN_COAST_MAX_MNPS      10000     // launch ceiling, 10.00 notch/s ("不要太快")
#define FN_COAST_TAU_MS        1400      // damping time constant; bigger = longer ride
#define FN_COAST_STOP_MNPS     350       // end the coast under 0.35 notch/s (~5 s from MAX)
#define FN_COAST_LIFT_MS       25        // dead cycles that count as "finger left the pad"
#define FN_COAST_VEL_WINDOW_MS 200       // older than this, the velocity estimate is stale

// Gesture post-processing only applies to the pointing-device path.
// During the PS/2 TrackPoint isolation test POINTING_DEVICE_ENABLE is off, so
// this is compiled out (the stock ps2_mouse driver sends reports directly).
#ifdef POINTING_DEVICE_ENABLE

#ifdef FN_COAST_ENABLE
// One axis of the Fn scroll: the drag-phase remainder, the lift-speed estimate and the
// coast state. h and v run through the same three helpers with independent state.
typedef struct {
    int16_t  accum;  // raw units not yet worth a whole notch (drag phase)
    int32_t  v_mnps; // EMA of finger speed, milli-notches/s, already in HID scroll sense
    int32_t  coast;  // current coast speed, mnps; 0 = not coasting
    int32_t  carry;  // coast fraction of a notch, milli-notches
} fn_axis_t;

// Set from process_record_user (main loop, not an ISR) and consumed by the pointing task.
static bool fn_coast_kill = false;

// Drag phase: accumulate-and-divide exactly as before, plus the velocity EMA. The
// instantaneous rate is measured over the gap since this axis LAST moved, not since the
// last cycle: during a slow drag most cycles are 0 and one unit turns up every third one,
// and dividing that by 10 ms would read it as three times the real speed -- which would
// let a slow drag clear the flick threshold.
static int16_t fn_scroll_drag(fn_axis_t *a, int16_t raw, uint32_t *moved_ms, uint32_t now) {
    a->accum += raw;
    int16_t notches = a->accum / FN_SCROLL_DIVISOR;
    a->accum -= notches * FN_SCROLL_DIVISOR;
    if (raw != 0) {
        uint32_t gap = now - *moved_ms;
        if (gap > FN_COAST_VEL_WINDOW_MS) {
            gap       = FN_COAST_VEL_WINDOW_MS;
            a->v_mnps = 0; // a fresh gesture must not inherit the previous flick's speed
        } else if (gap < 1) {
            gap = 1;
        }
        // Clamped so the *1000000 cannot overflow int32 even with MOUSE_EXTENDED_REPORT;
        // real per-cycle deltas here are well under 100.
        int32_t r = raw;
        if (r > 1000) r = 1000;
        if (r < -1000) r = -1000;
        int32_t inst = r * 1000000 / ((int32_t)FN_SCROLL_DIVISOR * (int32_t)gap);
        a->v_mnps += (inst - a->v_mnps) / 4; // ~40 ms EMA
        *moved_ms = now;
    }
    return notches;
}

// Finger left the pad: turn the last measured speed into a coast, or into nothing.
static void fn_scroll_launch(fn_axis_t *a) {
    int32_t v = a->v_mnps * FN_COAST_GAIN_NUM / FN_COAST_GAIN_DEN;
    a->v_mnps = 0;
    if (v > FN_COAST_MAX_MNPS) v = FN_COAST_MAX_MNPS;
    if (v < -FN_COAST_MAX_MNPS) v = -FN_COAST_MAX_MNPS;
    if (v < FN_COAST_MIN_MNPS && v > -FN_COAST_MIN_MNPS) return; // not a flick, just stop
    a->coast = v;
    a->carry = 0;
    a->accum = 0; // the drag remainder belongs to the finger, not to the coast
}

// Coast phase: integrate the speed into notches, then damp it. dt is the measured elapsed
// time, so a stalled pointing cycle cannot stretch the ride.
static int16_t fn_scroll_coast(fn_axis_t *a, uint32_t dt) {
    if (a->coast == 0) {
        return 0;
    }
    a->carry += a->coast * (int32_t)dt / 1000;
    int16_t notches = a->carry / 1000;
    a->carry -= (int32_t)notches * 1000;

    int32_t decay = a->coast * (int32_t)dt / FN_COAST_TAU_MS;
    if (decay == 0) {
        decay = (a->coast > 0) ? 1 : -1; // integer truncation must never stall the decay
    }
    a->coast -= decay;
    if (a->coast < FN_COAST_STOP_MNPS && a->coast > -FN_COAST_STOP_MNPS) {
        a->coast = 0;
        a->carry = 0;
    }
    return notches;
}

static inline void fn_scroll_stop(fn_axis_t *a) {
    a->coast = 0;
    a->carry = 0;
}

// Any keypress except Fn itself ends a running coast -- once you start typing (or click),
// the page has to stop moving. Fn is excluded so releasing it does not cut the ride short,
// which is the whole point of coasting after the finger is gone.
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    if (record->event.pressed && keycode != MO(FN_LAYER) && keycode != MO(2)) {
        fn_coast_kill = true;
    }
    return true;
}
#endif // FN_COAST_ENABLE

report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
    // Fn + move = scroll. While the Fn layer is held, turn cursor motion into wheel
    // scroll (up/down/left/right chosen by which way the finger goes) and suppress the
    // cursor. Accumulate + divide so a gentle move scrolls slowly and a fast one scrolls
    // fast; h/v are ADDED so a touchpad two-finger scroll that also produced h/v is
    // preserved. With FN_COAST_ENABLE the gesture additionally coasts after the lift.
#ifdef FN_COAST_ENABLE
    {
        static fn_axis_t ah = {0}, av = {0};
        static uint32_t  tick_ms = 0; // previous cycle, for the real dt
        static uint32_t  touch_ms = 0; // last cycle that carried finger motion
        static uint32_t  move_h_ms = 0, move_v_ms = 0; // per-axis, for the velocity gaps
        static bool      dragging = false; // a Fn drag is running and not yet cashed in

        uint32_t now = timer_read32();
        uint32_t dt  = now - tick_ms;
        tick_ms      = now;
        if (dt < 1) dt = 1;
        if (dt > 100) dt = 100; // first cycle or a long idle: never integrate a huge step

        bool moved = (mouse_report.x != 0 || mouse_report.y != 0);
        bool fn    = layer_state_is(FN_LAYER);

        // Anything new ends the ride: a touch (even the resting tremble of a finger put
        // back down), a two-finger scroll, a button, a keypress.
        if (moved || mouse_report.h != 0 || mouse_report.v != 0 || mouse_report.buttons != 0 || fn_coast_kill) {
            fn_scroll_stop(&ah);
            fn_scroll_stop(&av);
            fn_coast_kill = false;
            if (!fn) {
                dragging = false; // plain cursor use, not a flick waiting to be launched
            }
        }

        int16_t dh = 0, dv = 0;
        if (fn && moved) {
            dh       = fn_scroll_drag(&ah, mouse_report.x, &move_h_ms, now);
            dv       = fn_scroll_drag(&av, (int16_t)(FN_SCROLL_Y_SIGN * mouse_report.y), &move_v_ms, now);
            touch_ms = now;
            dragging = true;
        } else {
            // The launch test sits outside the `fn` check on purpose: letting go of Fn in
            // the same breath as lifting the finger must still start the coast.
            if (dragging && !moved && timer_elapsed32(touch_ms) >= FN_COAST_LIFT_MS) {
                dragging = false;
                fn_scroll_launch(&ah);
                fn_scroll_launch(&av);
            }
            dh = fn_scroll_coast(&ah, dt);
            dv = fn_scroll_coast(&av, dt);
        }

        mouse_report.h += dh; // move right -> scroll right (already matches two-finger)
        mouse_report.v += dv; // sign already applied per FN_SCROLL_Y_SIGN above
        if (fn) {
            mouse_report.x = 0;
            mouse_report.y = 0;
            return mouse_report;
        }
        // Not on the Fn layer: fall through so the rest of the hook still runs, carrying
        // whatever the coast just emitted.
    }
#else
    if (layer_state_is(FN_LAYER)) {
        static int16_t scroll_ax = 0, scroll_ay = 0;
        scroll_ax += mouse_report.x;
        scroll_ay += mouse_report.y;
        int16_t dh = scroll_ax / FN_SCROLL_DIVISOR;
        int16_t dv = scroll_ay / FN_SCROLL_DIVISOR;
        scroll_ax -= dh * FN_SCROLL_DIVISOR;
        scroll_ay -= dv * FN_SCROLL_DIVISOR;
        mouse_report.h += dh;                               // move/push right -> scroll right (already matches two-finger)
        mouse_report.v += (int16_t)(FN_SCROLL_Y_SIGN * dv); // sign per FN_SCROLL_Y_SIGN above
        mouse_report.x = 0;
        mouse_report.y = 0;
        return mouse_report;
    }
#endif // FN_COAST_ENABLE

    // Three-finger gesture dispatch — disabled (not removed). The stock vial-qmk azoteq
    // driver does not expose three-finger gestures (no AZOTEQ_IQS5XX_3F_* button-bit
    // encoding), so this block is wrapped in #if 0. Switch to #if 1 to re-enable it
    // once a custom driver that emits those bits is added back.
#if 0
    // The driver encodes gestures as button bits; intercept and clear them here,
    // then perform whatever action you want. DOWN must be checked before LEFT/RIGHT
    // because it uses both bits simultaneously.
    if ((mouse_report.buttons & AZOTEQ_IQS5XX_3F_DOWN_BITS) == AZOTEQ_IQS5XX_3F_DOWN_BITS) {
        mouse_report.buttons &= ~AZOTEQ_IQS5XX_3F_DOWN_BITS;
        // ← three-finger swipe down: customize action here
    } else if (mouse_report.buttons & AZOTEQ_IQS5XX_3F_UP_BIT) {
        mouse_report.buttons &= ~AZOTEQ_IQS5XX_3F_UP_BIT;
        // ← three-finger swipe up: customize action here
    } else if (mouse_report.buttons & AZOTEQ_IQS5XX_3F_LEFT_BIT) {
        mouse_report.buttons &= ~AZOTEQ_IQS5XX_3F_LEFT_BIT;
        tap_code16(KC_BTN4);  // three-finger swipe left: browser back
    } else if (mouse_report.buttons & AZOTEQ_IQS5XX_3F_RIGHT_BIT) {
        mouse_report.buttons &= ~AZOTEQ_IQS5XX_3F_RIGHT_BIT;
        tap_code16(KC_BTN5);  // three-finger swipe right: browser forward
    } else if (mouse_report.buttons & AZOTEQ_IQS5XX_3F_TAP_BIT) {
        mouse_report.buttons &= ~AZOTEQ_IQS5XX_3F_TAP_BIT;
        tap_code16(KC_BTN3);  // three-finger tap: middle click
    }
#endif

    // Zoom removed: AZOTEQ_IQS5XX_ZOOM_ENABLE is false in config.h, so the chip no longer
    // emits the BTN7/BTN8 pinch-zoom bits. The old handler that turned them into Ctrl+/-
    // has been deleted — two-finger gestures now always scroll.

    // Scroll shaping lives entirely in pointing.c (the TPS43_SCROLL_* constant-rate
    // limiter), so there is no scaling stage here.

    // Tap-to-drag ("选中锁定") removed: a tap then move no longer latches BUTTON1, so the
    // touchpad won't start an unintended selection/drag. Plain taps and physical mouse
    // buttons are unaffected.

    return mouse_report;
}
#endif // POINTING_DEVICE_ENABLE

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    // Layer 0: Base
    [0] = LAYOUT(
        KC_ESC,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,
        KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,
        KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_DEL,
        KC_LCTL, KC_LGUI, KC_LALT, MO(1),   KC_SPC,  KC_MS_BTN1,
        // right half
        KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,
        KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC,
        KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT, KC_BSLS,
        KC_BSPC, KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_UP,   KC_GRV,
        // [9,0] pairs with the left half's [4,6] MBtn1 — right click, not Enter.
        // Enter stays reachable as Fn + Space on either half (see Layer 1).
        KC_MS_BTN2, KC_SPC, MO(1),  KC_RCTL, KC_LEFT, KC_DOWN, KC_RGHT
    ),

    // Layer 1: Fn (right half's MO(1))
    // Esc/1-5 = ` / F1-F5
    // RGB: Tab=toggle  W/S=brightness+/-  E/D=hue+/-  R/F=sat+/-  T/G=mode/reverse
    // Fn + Space = Enter, Fn + Bksp = Insert
    // Fn + V (left) or Fn + M (right) = Win+Tab (Task View)
    // Fn + either thumb mouse-button key (left BTN1 / right BTN2 position) = middle click
    [1] = LAYOUT(
        KC_GRV,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,
        _______, RGB_TOG, RGB_VAI, RGB_HUI, RGB_SAI, RGB_MOD,
        _______, _______, RGB_VAD, RGB_HUD, RGB_SAD, RGB_RMOD,
        _______, _______, _______, _______, LGUI(KC_TAB), _______, _______,
        _______, _______, _______, _______, KC_ENT,  KC_MS_BTN3,
        // right half — F6-F12, nav cluster (Fn+arrows = Home/PgDn/End, Fn+Up = PgUp)
        KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,
        KC_VOLU, KC_WBAK, KC_UP,   KC_WFWD, _______, _______, KC_BRIU,
        KC_VOLD, KC_LEFT, KC_DOWN, KC_RGHT, _______, _______, KC_BRID,
        KC_INS,  KC_MPLY, LGUI(KC_TAB), _______, _______, _______, KC_PGUP, _______,
        KC_MS_BTN3, KC_ENT, _______, _______, KC_HOME, KC_PGDN, KC_END
    ),

    // Layer 2: left-half Fn (MO(2), the key labelled Fn).
    // Identical to layer 1 except the arrow cluster, which becomes media transport:
    //   Left = prev track, Down = play/pause, Right = next track.
    //   Up is left transparent, so it stays a plain Up arrow.
    [2] = LAYOUT(
        _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______,
        // right half
        _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______
    ),

    // Layer 3: reserved
    [3] = LAYOUT(
        _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______,
        // right half
        _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______
    ),
};
