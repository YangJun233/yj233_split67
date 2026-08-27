// Copyright 2024 yangjun
// SPDX-License-Identifier: GPL-2.0-or-later

#include "quantum.h"

// ============================================================================
// Boot-time expansion-module kill switch
//
// Hold this half's designated key while THAT HALF resets, and its expansion
// module (the Azoteq TPS43 touchpad / PS-2 TrackPoint on GP0/GP1/GP2) is left
// completely uninitialised for this boot -- it is never probed, never polled,
// and contributes an all-zero mouse report, so nothing it does can reach USB.
//
//   LEFT  half: hold Delete    (matrix [3,6])
//   RIGHT half: hold Backspace (matrix [8,0])
//
// Purpose: a flaky touchpad/TrackPoint can flood the cursor with garbage and
// make the machine unusable. This is a keyboard-only escape hatch -- no host
// software, no re-flash.
//
// NOT persistent, on purpose: the next reset comes up with the module enabled
// again. Nothing is written to EEPROM, so the safe state (expansion ON) is the
// one you get by default and the override can never strand you.
//
// PHYSICAL matrix positions are used, not keycodes: at this point in boot the
// Vial dynamic keymap is not the authority on what is under the cap, and
// remapping Delete/Backspace in Vial must not move the escape hatch.
//
// Both halves run this code (POINTING_DEVICE_COMBINED -- the module can sit on
// either half), and each half only ever reads its own rows: split_common writes
// the local scan into `matrix + thisHand` (quantum/matrix.c:343), and thisHand
// is 0 on the left / ROWS_PER_HAND on the right, which is exactly the row
// numbering used by LAYOUT. So the constants below are the LAYOUT coordinates
// verbatim, and the check works regardless of which half is USB master.
// ============================================================================

#include "bootmagic.h" // BOOTMAGIC_ROW / BOOTMAGIC_COLUMN

#if !defined(BOOTMAGIC_ENABLE)
#    error "yj233_split67: the expansion kill switch samples the boot matrix from bootmagic_should_reset(), which only runs when BOOTMAGIC_ENABLE is on. Turning bootmagic off would silently delete the feature (dead code), so it is a hard error instead."
#endif

#define EXPANSION_KILL_ROW_LEFT 3 // left half: Delete    -> LAYOUT [3,6]
#define EXPANSION_KILL_COL_LEFT 6
#define EXPANSION_KILL_ROW_RIGHT 8 // right half: Backspace -> LAYOUT [8,0]
#define EXPANSION_KILL_COL_RIGHT 0

// g_led_config indices of those same two keys, i.e. the position of their
// entries in info.json rgb_matrix.layout (that list IS the WS2812 chain order).
// Only ONE led is ever lit -- a full-matrix flash would pull more current than
// this board's rail is comfortable with.
#define EXPANSION_KILL_LED_LEFT 24  // matrix [3,6]
#define EXPANSION_KILL_LED_RIGHT 59 // matrix [8,0]

#define EXPANSION_KILL_BLINKS 3
#define EXPANSION_KILL_BLINK_MS 100
#define EXPANSION_KILL_BLINK_R 40 // red, dim: one led at ~40/255 is a few mA

static bool expansion_killed = false;

// Read by pointing.c (pointing_device_driver_init) to decide whether to probe.
bool yj_expansion_killed(void) {
    return expansion_killed;
}

// Sample the kill key and then run bootmagic's own stock reset check.
//
// Why here rather than in a dedicated hook: quantum_init() calls bootmagic() ->
// bootmagic_scan(), which does `matrix_scan(); wait_ms(BOOTMAGIC_DEBOUNCE);
// matrix_scan();` and only then calls this function. That makes this the first
// and only point in boot where matrix[] holds a settled, debounced snapshot of
// the keys held at power-on -- reached without adding a single extra scan, extra
// delay, or extra early split transaction. Overriding bootmagic_scan() instead
// would mean copying its bootloader_jump()/EEPROM-wipe body, i.e. duplicating
// the destructive path just to get one read; sampling from inside this predicate
// keeps that path untouched.
//
// The return expression below is bootmagic.c's stock implementation, reproduced
// unchanged so the normal "hold Esc at plug-in to wipe EEPROM and enter the
// bootloader" behaviour is preserved exactly.
bool bootmagic_should_reset(void) {
    uint8_t kill_row = is_keyboard_left() ? EXPANSION_KILL_ROW_LEFT : EXPANSION_KILL_ROW_RIGHT;
    uint8_t kill_col = is_keyboard_left() ? EXPANSION_KILL_COL_LEFT : EXPANSION_KILL_COL_RIGHT;

    expansion_killed = (matrix_get_row(kill_row) & (MATRIX_ROW_SHIFTER << kill_col)) != 0;

    // ---- stock bootmagic behaviour from here on ----
    uint8_t row = BOOTMAGIC_ROW;
    uint8_t col = BOOTMAGIC_COLUMN;

#if defined(SPLIT_KEYBOARD) && defined(BOOTMAGIC_ROW_RIGHT) && defined(BOOTMAGIC_COLUMN_RIGHT)
    if (!is_keyboard_left()) {
        row = BOOTMAGIC_ROW_RIGHT;
        col = BOOTMAGIC_COLUMN_RIGHT;
    }
#endif

    return matrix_get_row(row) & (MATRIX_ROW_SHIFTER << col);
}

// Blink the kill key's own led so you can see the override took, and know when
// it is safe to let go of the key.
//
// Called from pointing_device_driver_init(), which is deliberate: rgb_matrix_init()
// runs at quantum/keyboard.c:497 and pointing_device_init() at :545, so the WS2812
// driver is up by then, while bootmagic_should_reset() (where the key is sampled)
// runs far too early to light anything.
//
// rgb_matrix_set_color() does the split index translation itself
// (quantum/rgb_matrix/rgb_matrix.c:148-159): on the right half a global index >=
// split_count[0] is rebased onto the local chain, so passing the g_led_config
// index works unmodified on both halves. Every other led is still at the driver's
// zeroed power-on buffer, so the flush lights this one led and nothing else.
//
// Blocking waits are fine here: this path skips module probing entirely (which
// costs up to ~1.2s on the TrackPoint branch), so a killed half reaches its main
// loop sooner than a normal one even with the blink -- comfortably inside
// SPLIT_WATCHDOG_TIMEOUT (2100ms) if this half came up as the slave.
void yj_expansion_kill_indicate(void) {
#ifdef RGB_MATRIX_ENABLE
    uint8_t led = is_keyboard_left() ? EXPANSION_KILL_LED_LEFT : EXPANSION_KILL_LED_RIGHT;

    for (uint8_t i = 0; i < EXPANSION_KILL_BLINKS; i++) {
        rgb_matrix_set_color(led, EXPANSION_KILL_BLINK_R, 0, 0);
        rgb_matrix_update_pwm_buffers();
        wait_ms(EXPANSION_KILL_BLINK_MS);
        rgb_matrix_set_color(led, 0, 0, 0);
        rgb_matrix_update_pwm_buffers();
        wait_ms(EXPANSION_KILL_BLINK_MS);
    }
#endif
}

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
