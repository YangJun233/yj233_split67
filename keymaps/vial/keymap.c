// Copyright 2024 yangjun
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

// Hold Fn (the MO(1) layer) and move the pointing device to scroll instead of moving the
// cursor. Runs on the USB master where the layer state is authoritative, so it works no
// matter which half owns the module. Bigger FN_SCROLL_DIVISOR = slower.
//
// ---- THIS IS NOT THE TWO-FINGER SCROLL, AND THE TWO SHARE NO TUNING ------------------
// The pad has two scroll gestures with two different jobs, and they are independent all
// the way down. Do not tune one against the other; do not "match" their speeds.
//
//   Fn + ONE finger  (here, keymap.c)   -- long-distance scrolling. Drag speed is
//       FN_SCROLL_DIVISOR, and a flick coasts afterwards (FN_COAST_* below).
//   TWO fingers      (pointing.c)       -- small, precise scrolling. Speed and top-speed
//       cap are TPS43_SCROLL_STEP / _INTERVAL_MS / _CAP over there.
//
// They cannot even reach each other's inputs: tps43_read_raw() only fills x/y when the
// chip reports exactly ONE finger, and only fills h/v on a chip-detected (two-finger)
// scroll gesture. So one finger produces x/y and feeds this code; two fingers produce h/v
// and feed the limiter in pointing.c; neither path ever sees the other's numbers. The only
// two places they meet at runtime are deliberate and are not tuning: h/v passes through
// this hook untouched (`+=`, not `=`), and any incoming h/v cancels a running Fn coast --
// putting fingers down to stop a coasting page has to work.
//
// What this code DOES depend on is the CURSOR shaping, because its input is the cursor
// delta: TPS43_CURSOR_DIVISOR for the pad, TP_SPEED_MULT for the TrackPoint (currently 3;
// 48 = 16 * 3). Change either of those and the Fn scroll speed moves with it, so rescale
// FN_SCROLL_DIVISOR by the same factor to keep the feel. That is a cursor dependency, not
// a two-finger one.
#define FN_LAYER          1
#define FN_SCROLL_DIVISOR 48

// Y direction of the Fn+move scroll. The two pointing modules want OPPOSITE signs here,
// because their "natural" scroll conventions genuinely differ:
//
//   TOUCHPAD (-1 is wrong, use +1): the chip's two-finger scroll feeds report.v straight
//     from the Y delta with no negation (azoteq_iqs5xx.c: temp_report.v = COMBINE(y)),
//     and the SAME Y delta also becomes report.y. In HID terms y>0 = cursor down and v>0 =
//     scroll up, so two-finger scrolling on this pad means "finger down -> scroll up".
//     Fn+one-finger has to agree, or pushing the same way would scroll opposite ways.
//     DIRECTION is the only thing the two gestures have to share -- not speed, not
//     acceleration, not the coast.
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
// TAU is the reciprocal of the damping, so it moves the OPPOSITE way to it: to damp 30%
// harder, divide TAU by 1.3, do not multiply it. (1200 -> 920 was exactly that.)
//
// The ride therefore lasts TAU * ln(v0/STOP), which is deliberately NOT a fixed length:
// the damping is the constant, and how long a flick runs falls out of how hard it was
// thrown. Speeds are held as milli-notches per second (mnps) in int32, so all of this is
// integer math and there is no float anywhere: 1000 mnps = one wheel notch per second.
//
// WHY STOP IS SO HIGH (5.00 notch/s rather than nearly zero):
// The HID wheel is quantised to whole notches, and Windows turns one notch into 3 lines
// (~57 px). Deceleration can only be expressed as a widening gap between 57 px jumps, and
// once that gap passes roughly 200 ms the eye stops reading it as "slowing down" and
// starts reading it as stuttering. An exponential tail spends most of its life below that
// speed -- the first version of this aimed for a fixed 5 s ride and spent its last two
// seconds emitting isolated notches nearly a second apart, which is exactly the 一卡一卡
// it was reported as. Everything under STOP was never smooth deceleration, just a slow
// visible stop, so the coast is cut while it still looks like motion. STOP is therefore
// the one number to retune by eye: it IS the widest gap the coast will ever show, at
// 1000/STOP ms. 5.00 notch/s = 200 ms; 6.00 = 167 ms; 8.00 = 125 ms; 10.00 = 100 ms.
//
// MEASURED, not guessed: at STOP 5.00 the last five gaps of a hard flick came out 110,
// 130, 140, 160, 180 ms, and the three over ~140 ms were reported as visible stutter at
// the end of every coast. So the threshold on this pad and this screen sits somewhere
// under 140 ms, and STOP is set to 8.00 (worst gap 110 ms, whole tail at 90-110 ms). The
// cost of cutting there is small because the tail is the cheap part: it removes 4 notches
// out of 30 and 0.6 s out of 2.2 s. If a stutter ever reappears, raise STOP before
// touching anything else; if the coast now ends too eagerly, lower it a step at a time
// and watch that last gap rather than the total duration.
//
// Note that the STEP is not ours to shrink: how many lines a notch scrolls is a per-HOST
// setting shared with every other mouse on that machine, so it cannot be tuned for this
// keyboard alone and this code must assume the default 3 lines. That leaves the INTERVAL
// as the only lever, which is exactly what STOP is. (Shrinking the step per-device does
// exist -- high-resolution scrolling is a property of THIS device's HID descriptor and
// would not touch any other mouse -- but it is a descriptor change, parked unverified on
// the touchpad-hires-scroll branch.)
//
// With the values below: lift at 30 notch/s (the MAX ceiling) -> 1.2 s and 20 notches; at
// 20 -> 0.9 s and 11; at 15 -> 0.6 s and 7; at 12 (the MIN gate) -> 0.4 s and 3 (the ride
// is shorter than it reads, because TAU carries 30% more damping than it used to). MAX sits
// well above any real flick on purpose: clamping is itself a step at the handover, so the
// ceiling exists only to bound a freak reading, not to shape the feel. MIN is raised alongside
// STOP so the weakest qualifying flick still coasts a few notches rather than twitching
// once and stopping -- a coast shorter than that reads as a glitch, not as inertia.
//
// TUNING THE HANDOVER. Too fast: lengthen FN_COAST_PEAK_MS or raise FN_COAST_EMA_SHIFT.
// Too slow: shorten PEAK_MS or lower EMA_SHIFT. Move ONE of the two per test -- they push
// the same quantity from opposite ends and changing both at once overshoots (it did).
// GAIN is NOT the knob for either direction: scaling the speed at the handover is exactly
// what produces a step, which is the one thing this feature must not have. Use GAIN only
// if you actually want a deliberate step.
//
// The other way out -- keep the long tail and make each step small enough to stay smooth
// -- needs high-resolution scrolling, i.e. a USB HID descriptor change. That attempt is
// parked unverified on the touchpad-hires-scroll branch; this is the cheap fix.
//
// WHY GAIN IS 1/1: it must be. Anything less puts a STEP at the moment of lift -- the
// wheel is running at the finger's speed one cycle and at a fraction of it the next, which
// reads as "抬手之后速度明显降低", not as inertia. An earlier 1/4 was an attempt to keep
// the coast from feeling fast, and it bought that at the cost of the one property the
// feature exists for: continuity. Speed is bounded by MAX and by TAU instead, which act on
// the ride rather than on the handover. MAX is set high enough (30 notch/s) that a normal
// flick never touches it, because clamping is itself a step.
//
// MIN means "only a deliberate flick coasts": lifting slower than 12 notch/s (~24 mm/s of
// finger travel -- the pad is ~47.6 counts/mm, halved by TPS43_CURSOR_DIVISOR, then
// /FN_SCROLL_DIVISOR = about 0.5 notch per mm) just stops where you left it. MIN must stay
// ABOVE STOP, or a launch would be killed by the stop test on its very first cycle.
//
// Velocity is measured from the raw per-cycle deltas, NOT from emitted notches: the wheel
// is quantised to whole notches, so counting notches over the last few cycles is far too
// coarse at these rates. Every cycle carrying motion contributes raw-units/ms through a
// 1/4 EMA (~40 ms of memory) instead, which is fine-grained and already smoothed.
//
// Finger-lift detection is "no cursor delta for FN_COAST_LIFT_MS". The pointing task runs
// once per AZOTEQ_IQS5XX_REPORT_RATE (10 ms), so 20 ms is two dead cycles. One cycle was
// tried, to tighten the handover, and it is safe -- a stray NAK mid-flick launching an
// early coast is cancelled by the next cycle with motion, and a launch no longer clears
// the velocity estimate -- but it made the coast start sooner AND the estimate hotter at
// the same time, which was part of overshooting into "太快". Two cycles it is. There is no
// finger-count bit to consult on this side -- with POINTING_DEVICE_COMBINED only
// x/y/h/v/buttons cross the split, and the touch count stays on whichever half owns the
// pad. That absence is a real limitation, and this comment used to paper over it: it argued
// that a finger resting ON the pad is never perfectly still, so the tremble keeps reaching
// this hook and "stop moving but keep touching" is self-correcting. It is NOT, reliably.
// pointing.c does keep the tremble at the sensor (TPS43_JITTER_DEADZONE is 0), but the
// report passes through tps43_scale on the way here, and that accumulate-and-divide is a
// zero-mean low-pass: a strictly alternating +1,-1 tremble sums back to 0 and emits NOTHING.
// Only a run of same-signed cycles leaks a delta through. So "finger down but still" can
// look exactly like "finger lifted" from this side of the split.
//
// What actually keeps that benign is the MIN gate, not the tremble: coming to a stop decays
// the EMA under FN_COAST_MIN_MNPS, so a drag slowed to a halt launches nothing. The case
// genuinely not covered is stopping ABRUPTLY at speed without lifting -- that does coast,
// and from here it is indistinguishable from a flick-and-lift. Moving the finger again
// cancels it on the next cycle (real motion, not tremble), as does any keypress.
#define FN_COAST_ENABLE                  // comment out to get the old hard-stop scroll back
#define FN_COAST_GAIN_NUM      1         // launch speed = lift speed * NUM/DEN ...
#define FN_COAST_GAIN_DEN      1         // ... 1/1: hand the finger's own speed straight over
#define FN_COAST_MIN_MNPS      12000     // below 12.00 notch/s at lift: no coast at all
#define FN_COAST_MAX_MNPS      30000     // launch ceiling, 30.00 notch/s (rarely reached)
#define FN_COAST_TAU_MS        920       // 1/TAU IS the damping: SMALLER = more damping
#define FN_COAST_STOP_MNPS     8000      // stop at 8.00 notch/s = one notch per 125 ms
#define FN_COAST_PEAK_MS       120       // peak-hold lookback over the EMA
#define FN_COAST_EMA_SHIFT     2         // EMA weight 1/(1<<N); 2 = quarter, ~3 cycles of lag
#define FN_COAST_LIFT_MS       20        // dead cycles that count as "finger left the pad"
#define FN_COAST_VEL_WINDOW_MS 200       // older than this, the velocity estimate is stale

// Gesture post-processing only applies to the pointing-device path.
// During the PS/2 TrackPoint isolation test POINTING_DEVICE_ENABLE is off, so
// this is compiled out (the stock ps2_mouse driver sends reports directly).
#ifdef POINTING_DEVICE_ENABLE

#ifdef FN_COAST_ENABLE
// One axis of the Fn scroll: the drag-phase remainder, the lift-speed estimate and the
// coast state. h and v run through the same three helpers with independent state.
typedef struct {
    int16_t  accum;   // raw units not yet worth a whole notch (drag phase)
    int32_t  v_mnps;  // EMA of finger speed, milli-notches/s, already in HID scroll sense
    int32_t  v_peak;  // fastest recent EMA -- this is what a lift launches from
    uint32_t peak_ms; // when v_peak was last refreshed, for the hold timeout
    int32_t  coast;   // current coast speed, mnps; 0 = not coasting
    int32_t  carry;   // coast fraction of a notch, milli-notches
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
            a->v_peak = 0;
        } else if (gap < 1) {
            gap = 1;
        }
        // Clamped so the *1000000 cannot overflow int32 even with MOUSE_EXTENDED_REPORT;
        // real per-cycle deltas here are well under 100.
        int32_t r = raw;
        if (r > 1000) r = 1000;
        if (r < -1000) r = -1000;
        int32_t inst = r * 1000000 / ((int32_t)FN_SCROLL_DIVISOR * (int32_t)gap);
        a->v_mnps += (inst - a->v_mnps) / (1 << FN_COAST_EMA_SHIFT); // /, not >>: symmetric

        // Peak-hold on top of the EMA. Launching from the EMA as it stands at the instant
        // of lift starts the coast VISIBLY SLOWER than the finger was moving -- two effects
        // stack: the EMA lags the finger, and the last cycle or two before a lift droop as
        // the contact patch shrinks and the chip reports a partial move. So remember the
        // fastest EMA of the last FN_COAST_PEAK_MS and launch from that. The hold expires
        // rather than latching, so a finger that genuinely decelerates before lifting still
        // launches slow, or fails the MIN gate and does not coast at all.
        //
        // THE TWO SETTINGS THAT SET THE HANDOVER SPEED ARE EMA_SHIFT AND PEAK_MS, and both
        // were pushed in the fast direction once and pushed back after testing. Recording
        // both ends here so the next person does not walk the same loop:
        //
        //   Theory says these under-read. An EMA chasing a RAMP never reaches it, it settles
        //   ((1<<SHIFT)-1) cycles behind, and a flick is exactly a ramp -- the finger is
        //   still speeding up when it leaves the pad. At 1/4 that is a ~30 ms lag, worth
        //   several notch/s on a hard flick. Raising the weight to 1/2 (one cycle of lag) and
        //   shortening the hold to 50 ms was the principled fix for that.
        //
        //   Testing says otherwise, twice, in both directions. On this pad the 1/4 weight
        //   with a 120 ms hold lands where the flick actually feels like it continues; the
        //   1/2 weight overshoots it noticeably. Two things the theory leaves out: the light
        //   smoothing lets the raw delta's +/-1 quantisation (about +/-2 notch/s) straight
        //   through and the peak-hold then keeps the high side of it, and a 120 ms hold
        //   reaching back toward the middle of the stroke -- where a flick is fastest, since
        //   you ease off before running out of pad -- turns out to compensate for the lag
        //   rather than to add to it. The two errors cancel. Believe the hand, not the model.
        //
        // So: FN_COAST_EMA_SHIFT 2 and FN_COAST_PEAK_MS 120 are empirical, not derived. If
        // the handover ever needs to move, move ONE of them and re-test; do not "fix" both
        // toward the theory at once, which is how it overshot.
        int32_t mag  = a->v_mnps < 0 ? -a->v_mnps : a->v_mnps;
        int32_t pmag = a->v_peak < 0 ? -a->v_peak : a->v_peak;
        if (mag >= pmag || now - a->peak_ms > FN_COAST_PEAK_MS) {
            a->v_peak  = a->v_mnps;
            a->peak_ms = now;
        }
        *moved_ms = now;
    }
    return notches;
}

// Finger left the pad: turn the last measured speed into a coast, or into nothing.
static void fn_scroll_launch(fn_axis_t *a) {
    // Deliberately does NOT clear v_peak/v_mnps: staleness is already handled by the
    // FN_COAST_VEL_WINDOW_MS check in fn_scroll_drag, and keeping them means two flicks in
    // quick succession build on each other the way they do on a phone.
    int32_t v = a->v_peak * FN_COAST_GAIN_NUM / FN_COAST_GAIN_DEN;
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

        bool moved   = (mouse_report.x != 0 || mouse_report.y != 0);
        bool fn      = layer_state_is(FN_LAYER);
        bool killed  = fn_coast_kill; // latch it: the block below clears the flag

        // Anything new ends the ride: a touch (a finger put back down and moved), a
        // two-finger scroll, a button, a keypress.
        if (moved || mouse_report.h != 0 || mouse_report.v != 0 || mouse_report.buttons != 0 || killed) {
            fn_scroll_stop(&ah);
            fn_scroll_stop(&av);
            fn_coast_kill = false;
            // A keypress has to cancel the PENDING drag too, not just a coast already
            // running. Without the `|| killed` this block would consume the kill flag while
            // the coast has not launched yet -- during the FN_COAST_LIFT_MS window, when
            // fn_scroll_stop() has nothing to stop -- and the launch test below would then
            // fire anyway, one cycle after the user pressed a key to prevent exactly that.
            // Fn is held throughout that window, so `!fn` alone never clears it.
            if (!fn || killed) {
                dragging = false; // plain cursor use, or a keypress: not a flick to launch
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

    // Zoom removed: AZOTEQ_IQS5XX_ZOOM_ENABLE is false in config.h, so the chip no longer
    // emits the BTN7/BTN8 pinch-zoom bits. The old handler that turned them into Ctrl+/-
    // has been deleted — two-finger gestures now always scroll.

    // Nothing to do here for the TWO-FINGER scroll: its shaping is entirely pointing.c's
    // TPS43_SCROLL_* constant-rate limiter, and its h/v reach this hook already finished.
    // The Fn one-finger scroll above is a separate gesture with its own tuning; see the
    // block at the top of this file.

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
