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

// Gesture post-processing only applies to the pointing-device path.
// During the PS/2 TrackPoint isolation test POINTING_DEVICE_ENABLE is off, so
// this is compiled out (the stock ps2_mouse driver sends reports directly).
#ifdef POINTING_DEVICE_ENABLE
report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
    // Fn + move = scroll. While the Fn layer is held, turn cursor motion into wheel
    // scroll (up/down/left/right chosen by which way the stick is pushed) and suppress
    // the cursor. Accumulate + divide so a gentle push scrolls slowly and a hard push
    // scrolls fast (force sets the speed); h/v are ADDED so a touchpad two-finger scroll
    // that also produced h/v is preserved.
    if (layer_state_is(FN_LAYER)) {
        static int16_t scroll_ax = 0, scroll_ay = 0;
        scroll_ax += mouse_report.x;
        scroll_ay += mouse_report.y;
        int16_t dh = scroll_ax / FN_SCROLL_DIVISOR;
        int16_t dv = scroll_ay / FN_SCROLL_DIVISOR;
        scroll_ax -= dh * FN_SCROLL_DIVISOR;
        scroll_ay -= dv * FN_SCROLL_DIVISOR;
        mouse_report.h += dh;  // push right -> scroll right
        mouse_report.v += -dv; // push up (y<0) -> scroll up
        mouse_report.x = 0;
        mouse_report.y = 0;
        return mouse_report;
    }

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
