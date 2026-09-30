// Copyright 2024 yangjun
// SPDX-License-Identifier: GPL-2.0-or-later

// ============================================================================
// KEY_FADE: a pressed key's own led lights at full brightness and fades to
// black over KEY_FADE_MS (config.h), each key on its own clock.
// Everything else stays black, so an idle half renders all-black frames and
// yj233_split67.c cuts its WS2812 rail (RGB_EN_PIN).
//
// Colour  = rgb_matrix hue / saturation (Fn+E/D, Fn+R/F, or Vial's picker).
// Peak    = rgb_matrix val (Fn+W/S, Vial brightness), capped by max_brightness.
// Timing  = fixed KEY_FADE_MS. rgb_matrix speed is deliberately ignored: it is
//           one global value shared with every other effect, so tying the fade
//           to it would also slow the cycle effects down.
//
// Why a custom effect instead of QMK's SOLID_REACTIVE_SIMPLE: that one derives
// its fade time from the global speed (~65280 / (speed + 1) ms), and reads its
// hits from g_last_hit_tracker, which (a) holds only LED_HITS_TO_REMEMBER
// presses, so fast typing pushes a still-fading key out and it goes dark
// abruptly, and (b) has an upstream bug in rgb_task_timers(): when the OLDEST
// entry's tick is about to overflow (~65.5 s) it decrements count, which drops
// the NEWEST entry instead, and repeats every pass until the list is empty --
// cutting every fading key at once. Instead this effect keeps its own press
// time per led: no capacity limit, no dependency on that code.
// ============================================================================

RGB_MATRIX_EFFECT(KEY_FADE)

// ---- Vial ----
// VialRGB only exposes the QMK effects listed in quantum/vialrgb_effects.inc;
// a custom effect is invisible to it, and worse, while one is active Vial reads
// the mode back as "off" -- so touching the colour picker would send "off" and
// kill the lighting. Borrow the SOLID_REACTIVE_SIMPLE slot instead: that table
// entry is guarded by RGB_MATRIX_EFFECT_SOLID_REACTIVE_SIMPLE and names the
// enum value RGB_MATRIX_SOLID_REACTIVE_SIMPLE, so defining both here makes Vial
// list this effect as "Solid Reactive Simple" and select/report it correctly.
// This file is included several times (enum, implementations, dispatch
// switch), hence the guard.
#ifndef YJ_KEY_FADE_VIAL_ALIAS
#    define YJ_KEY_FADE_VIAL_ALIAS
#    ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_SIMPLE
#        error "KEY_FADE takes over Vial's SOLID_REACTIVE_SIMPLE slot -- do not also enable the built-in solid_reactive_simple animation"
#    endif
#    define RGB_MATRIX_EFFECT_SOLID_REACTIVE_SIMPLE
#    define RGB_MATRIX_SOLID_REACTIVE_SIMPLE RGB_MATRIX_CUSTOM_KEY_FADE
#endif

#ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS

#    ifndef RGB_MATRIX_KEYPRESSES
#        error "KEY_FADE needs RGB_MATRIX_KEYPRESSES (config.h): without it rgb_matrix never looks up a pressed key's led, so key_fade_start is never written (see yj233_split67.c)"
#    endif

// sync_timer time of each led's latest press, written by the press hook
// rgb_matrix_map_row_column_to_led_kb() in yj233_split67.c. (The hook cannot
// live here: this file is compiled inside rgb_matrix.c, the same translation
// unit as the weak default, so a definition here is a redefinition error.)
extern uint32_t key_fade_start[RGB_MATRIX_LED_COUNT];

// Idle leds are parked this far in the past: 2x the window, so a small
// backward clock step (split sync jitter on the slave) still leaves them dark
// instead of flashing a dim frame. Active fades just run that much longer.
#    define KEY_FADE_PARK_MS (2 * KEY_FADE_MS)

// The press times are absolute sync_timer values, so a jump of that clock
// would misplace them. On the slave sync_timer is overwritten by the master
// every 100 ms (quantum/sync_timer.c sync_timer_update): if only the master
// reboots (flashing it, QK_BOOT, EEPROM reset) the slave's clock jumps back by
// the master's whole uptime and every parked led would compute a huge
// negative age; if the slave reboots under a long-running master it jumps
// forward, past 2^31 ms after ~24.8 days, wrapping the other way. Either way
// the effect's init flag is not raised (mode/enable unchanged), so detect the
// jump here: the frame-to-frame step of g_rgb_timer is normally ~16 ms
// (RGB_MATRIX_LED_FLUSH_LIMIT); a step backwards by more than the park margin,
// or forwards by more than a whole window, re-parks every led like init does.
// The only thing lost is fades that were in progress across the jump.
static uint32_t key_fade_prev_frame;
static bool     key_fade_reset;

static bool KEY_FADE(effect_params_t *params) {
    RGB_MATRIX_USE_LIMITS(led_min, led_max);

    // Decided once per frame (iter 0 = first chunk), applied to every chunk.
    if (params->iter == 0) {
        int32_t step        = (int32_t)(g_rgb_timer - key_fade_prev_frame);
        key_fade_reset      = params->init || step < -(int32_t)KEY_FADE_MS || step > (int32_t)KEY_FADE_MS;
        key_fade_prev_frame = g_rgb_timer;
    }

    for (uint8_t i = led_min; i < led_max; i++) {
        // Before the flags test, so an led excluded by rgb_matrix flags is still
        // re-parked and cannot come back with a wrapped age.
        //
        // Reset covers: first frame after switching to this effect, enabling,
        // or waking (forget old presses); boot, where key_fade_start is 0 and
        // g_rgb_timer is still small; and a clock jump (above).
        if (key_fade_reset) {
            key_fade_start[i] = g_rgb_timer - KEY_FADE_PARK_MS;
        }

        RGB_MATRIX_TEST_LED_FLAGS();

        // Signed: a press recorded after this frame's g_rgb_timer snapshot
        // gives a small negative age -- it just happened, so full brightness.
        // Anything further negative cannot be a real press; treat as expired.
        int32_t age = (int32_t)(g_rgb_timer - key_fade_start[i]);
        if (age < 0) {
            age = (age > -(int32_t)KEY_FADE_MS) ? 0 : KEY_FADE_PARK_MS;
        }

        hsv_t hsv = rgb_matrix_config.hsv;
        if (age >= KEY_FADE_MS) {
            hsv.v = 0;
            // Keep it parked a fixed distance in the past, so the signed age
            // can never creep toward the 2^31 wrap (~24.8 days) while the key
            // sits unpressed.
            key_fade_start[i] = g_rgb_timer - KEY_FADE_PARK_MS;
        } else {
            // Linear in V, rounded up so the led stays lit for the whole window
            // and reaches 0 exactly at KEY_FADE_MS. hsv_to_rgb applies the
            // CIE1931 curve on top, so the fade is linear in perceived
            // lightness, not in PWM.
            hsv.v = ((uint32_t)hsv.v * (KEY_FADE_MS - age) + KEY_FADE_MS - 1) / KEY_FADE_MS;
        }

        rgb_t rgb = rgb_matrix_hsv_to_rgb(hsv);
        rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
    }
    return rgb_matrix_check_finished_leds(led_max);
}

#endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS
