// Copyright 2024 yangjun
// SPDX-License-Identifier: GPL-2.0-or-later

#include "quantum.h"

void keyboard_pre_init_kb(void) {
    // Power on the RGB LEDs before the RGB matrix driver initialises.
    // The pointing module reset (GP2) is handled per-module in pointing.c, since
    // the TPS43 and TrackPoint need opposite reset polarities.
    gpio_set_pin_output(RGB_EN_PIN);
    gpio_write_pin_high(RGB_EN_PIN);

    keyboard_pre_init_user();
}

#ifdef RGB_MATRIX_ENABLE
// Cut the WS2812 rail when the matrix is off.
//
// RGB_TOG only zeroes the effect: rgb_matrix_task() keeps rendering and
// flushing all-black frames (quantum/rgb_matrix/rgb_matrix.c, `effect = ... ? 0
// : mode`), so the LEDs stay powered and keep drawing their quiescent current.
// Gate RGB_EN_PIN on the enable/suspend state to actually remove that draw.
//
// Runs on both halves: rgb_matrix_config and the suspend flag are mirrored to
// the slave by the split RGB_MATRIX transaction, so each half gates its own rail.
static bool rgb_powered = true; // keyboard_pre_init_kb() left the rail on

static void rgb_power_set(bool on) {
    if (on == rgb_powered) return;

    rgb_powered = on;
    gpio_write_pin(RGB_EN_PIN, on);
    if (on) {
        // Let the rail come up before the driver clocks out the next frame,
        // otherwise the first bits arrive while the LEDs are still browning in
        // and the chain latches garbage for one frame.
        wait_ms(2);
    }
}

static void rgb_power_task(void) {
    rgb_power_set(rgb_matrix_is_enabled() && !rgb_matrix_get_suspend_state());
}
#endif

void housekeeping_task_kb(void) {
#ifdef RGB_MATRIX_ENABLE
    rgb_power_task();
#endif

    housekeeping_task_user();
}

// USB suspend: the host stopped the bus (PC asleep, lid closed, selective
// suspend). tmk_core/protocol/chibios/chibios.c parks in a while loop inside
// protocol_pre_task() for the whole suspend, so keyboard_task() — and with it
// housekeeping_task_kb() above — never runs. The rail has to be cut from these
// hooks instead of from the poll.
void suspend_power_down_kb(void) {
#ifdef RGB_MATRIX_ENABLE
    // Forced, not rgb_power_task(): quantum.c calls this hook before it sets
    // the matrix suspend flag, so the poll would still compute "keep it on".
    rgb_power_set(false);
#endif

    suspend_power_down_user();
}

void suspend_wakeup_init_kb(void) {
#ifdef RGB_MATRIX_ENABLE
    // Here the flag is already cleared, so the poll gives the right answer and
    // honours RGB_TOG — no flash of light on wake if the user had it off.
    rgb_power_task();
#endif

    suspend_wakeup_init_user();
}
