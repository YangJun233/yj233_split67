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
#include "usb_device_state.h"

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
// Cut the WS2812 rail when the matrix is off, the host is suspended, or the
// effect has been showing nothing at all for a moment.
//
// RGB_TOG only zeroes the effect: rgb_matrix_task() keeps rendering and
// flushing all-black frames (quantum/rgb_matrix/rgb_matrix.c, `effect = ... ? 0
// : mode`), so the LEDs stay powered and keep drawing their quiescent current.
// Gate RGB_EN_PIN on the enable/suspend state to actually remove that draw.
//
// Runs on both halves: rgb_matrix_config and (with rgb_matrix.sleep in
// info.json, i.e. RGB_MATRIX_SLEEP) the matrix suspend flag are mirrored to the
// slave by the split RGB_MATRIX transaction, so each half gates its own rail.
// Without RGB_MATRIX_SLEEP rgb_matrix_set_suspend_state() is a no-op and the
// flag reads false forever -- that is why info.json enables it.
static bool rgb_powered = true; // keyboard_pre_init_kb() left the rail on

// Master only: set by the USB suspend hooks below. The matrix suspend flag is
// set by quantum.c only AFTER suspend_power_down_kb() and an rgb_matrix_task()
// pass (suspend_power_down_quantum), so on its own it leaves a window where a
// lit frame's indicator hook would switch the rail back on.
static bool rgb_usb_suspended = false;

// Last time a rendered frame had any led lit on this half (see
// rgb_matrix_indicators_kb). The rail is cut once frames have stayed black for
// RGB_RAIL_OFF_DELAY_MS -- e.g. KEY_FADE with no key inside its fade window.
// Effect-agnostic on purpose: it reads the actual frame rather than knowing
// which effect is on. The delay stops effects that only touch black for an
// instant (breathing at the bottom of its cycle) from toggling the rail -- and
// paying a power-up inrush plus the 2 ms stall -- on every cycle.
#define RGB_RAIL_OFF_DELAY_MS 200
static uint32_t rgb_last_lit = 0;

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
    bool dark = timer_elapsed32(rgb_last_lit) >= RGB_RAIL_OFF_DELAY_MS;
    rgb_power_set(rgb_matrix_is_enabled() && !rgb_matrix_get_suspend_state() && !rgb_usb_suspended && !dark);
}

// ---- KEY_FADE press timestamps (effect in rgb_matrix_kb.inc) ----
//
// sync_timer time of each led's latest press. Only this half's entries are
// ever rendered; on the master the other half's presses land here too and are
// simply never read.
uint32_t key_fade_start[RGB_MATRIX_LED_COUNT];

// Press hook. rgb_matrix_handle_key_event() (quantum/rgb_matrix/rgb_matrix.c)
// calls rgb_matrix_map_row_column_to_led() -> this _kb hook for every key PRESS
// (RGB_MATRIX_KEYPRESSES, config.h), from switch_events() on BOTH halves
// (RGB_MATRIX_SPLIT) -- so each half sees its own keys from its own scan and no
// SPLIT_TRANSPORT_MIRROR is needed. It is the only per-press hook that runs on
// the slave too (process_record_* are master only). Its real job, adding extra
// leds for a key, is unused here: returning 0 keeps the stock one-led mapping.
// Runs whatever effect is active; the timestamps are only read by KEY_FADE.
uint8_t rgb_matrix_map_row_column_to_led_kb(uint8_t row, uint8_t column, uint8_t *led_i) {
    uint8_t led = g_led_config.matrix_co[row][column];
    if (led != NO_LED) {
        key_fade_start[led] = sync_timer_read32();
    }
    return 0;
}

// ---- Per-half current limiter + lit-frame detection ----
//
// Hook point: rgb_matrix_task() calls rgb_matrix_indicators() once per frame,
// right after the effect finished rendering the whole frame and before the
// FLUSHING state sends it (quantum/rgb_matrix/rgb_matrix.c, RENDERING case). So
// the buffer read here is the complete frame that is about to go out, and
// anything changed here is what the leds get. It is only called while an effect
// is running (effect != 0); the off / suspended cases are handled by
// rgb_power_task() from housekeeping and the suspend hooks.
//
// The values read are final PWM values: rgb_matrix's hsv_to_rgb has already
// applied the CIE1931 curve (forced on with RGB_MATRIX_ENABLE, quantum/color.c),
// so this is what actually sets the led current.
//
// The buffer is the RP2040 vendor driver's ws2812_leds[] (ws2812_vendor.c). It
// has no header declaration, hence the extern; a different WS2812 driver without
// that symbol fails at link time rather than silently skipping the limiter.
//
// Assumes every effect rewrites every led every frame (true for all effects built
// here). An effect that drew once and relied on the buffer persisting would be
// scaled again every frame and fade out.
extern ws2812_led_t ws2812_leds[WS2812_LED_COUNT];

// Budget in "sum of r+g+b" units: one channel at 255 ~= WS2812_CHANNEL_FULL_MA.
#define RGB_CURRENT_BUDGET_UNITS ((uint32_t)RGB_CURRENT_BUDGET_MA * 255 / WS2812_CHANNEL_FULL_MA)

bool rgb_matrix_indicators_kb(void) {
    // The limiter is a safety cap, so it runs even if the user hook asks to skip
    // keyboard-level indicators; its return value is passed through untouched.
    bool res = rgb_matrix_indicators_user();

    // This half's leds occupy the driver buffer from local index 0
    // (rgb_matrix_led_index() rebases the right half). Entries past them are
    // never on this chain, so they are not counted.
#    ifdef RGB_MATRIX_SPLIT
    const uint8_t split[2] = RGB_MATRIX_SPLIT;
    const uint8_t count    = is_keyboard_left() ? split[0] : split[1];
#    else
    const uint8_t count = RGB_MATRIX_LED_COUNT;
#    endif

    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) {
        sum += ws2812_leds[i].r + ws2812_leds[i].g + ws2812_leds[i].b;
    }

    if (sum > RGB_CURRENT_BUDGET_UNITS) {
        // Same factor for every led keeps the hues; rounding down keeps the
        // result at or under the budget.
        for (uint8_t i = 0; i < count; i++) {
            ws2812_leds[i].r = (uint32_t)ws2812_leds[i].r * RGB_CURRENT_BUDGET_UNITS / sum;
            ws2812_leds[i].g = (uint32_t)ws2812_leds[i].g * RGB_CURRENT_BUDGET_UNITS / sum;
            ws2812_leds[i].b = (uint32_t)ws2812_leds[i].b * RGB_CURRENT_BUDGET_UNITS / sum;
        }
    }

    // Power the rail up HERE, before this frame is flushed, rather than waiting
    // for housekeeping: otherwise the first lit frame after a key press would be
    // sent into an unpowered chain and lost, adding a whole frame of latency.
    if (sum != 0) {
        rgb_last_lit = timer_read32();
    }
    rgb_power_task();

    return res;
}
#endif

// ============================================================================
// Stuck modifier after plug-in: resync the host once USB is up
//
// Symptom: right after plugging in, Alt is held on the host -- every key comes out as
// Alt+key -- until Alt is pressed and released once. Pressing it clears it, so the key
// and the matrix are fine; what is stuck is a PRESS that reached some state (the host's,
// or QMK's own) without its matching RELEASE. Where that press comes from is not yet
// established -- candidates are a boot-time transient on the matrix, the split handshake,
// a report sent while USB was half-enumerated, or a Vial combo/macro emitting Alt.
//
// The standard cure, independent of the cause: once the host has configured the
// device and had a moment to settle, and nothing is physically held, send one
// all-released report (clear_keyboard also clears QMK's own mods/keys, so a phantom
// on the firmware side is cleared too). It re-runs on every re-enumeration -- plug-in,
// host reboot, KVM switch -- since each is a new chance for the same desync.
//
// NOT a fix for the root cause, and it cannot help if the phantom press arrives later
// than RESYNC_DELAY_MS, or comes from the host side (RDP, a VM, another keyboard).
// USB_WAIT_FOR_ENUMERATION is not an option instead: with SPLIT_USB_DETECT the slave
// half never enumerates and would hang in that wait.
// ============================================================================
#ifndef YJ_USB_RESYNC_DELAY_MS
#    define YJ_USB_RESYNC_DELAY_MS 500
#endif

static bool yj_any_key_held(void) {
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        if (matrix_get_row(row)) return true;
    }
    return false;
}

static void yj_usb_resync_task(void) {
    static bool     was_configured = false;
    static bool     pending        = false;
    static uint32_t configured_ms  = 0;

    if (!is_keyboard_master()) return;

    // Polled here rather than from notify_usb_device_state_change_kb(), which runs from
    // the USB event callback -- not a place to be sending reports from.
    const bool configured = usb_device_state_get_configure_state() == USB_DEVICE_STATE_CONFIGURED;
    if (configured && !was_configured) {
        pending       = true;
        configured_ms = timer_read32();
    }
    was_configured = configured;

    if (pending && configured && timer_elapsed32(configured_ms) >= YJ_USB_RESYNC_DELAY_MS) {
        // A key held right now is genuine: leave it alone and do not retry, so this can
        // never release something the user is actually pressing.
        pending = false;
        if (!yj_any_key_held()) {
            clear_keyboard();
        }
    }
}

void housekeeping_task_kb(void) {
#ifdef RGB_MATRIX_ENABLE
    rgb_power_task();
#endif
    yj_usb_resync_task();

    housekeeping_task_user();
}

// USB suspend: the host stopped the bus (PC asleep, lid closed, selective
// suspend). tmk_core/protocol/chibios/chibios.c parks in a while loop inside
// protocol_pre_task() for the whole suspend, so keyboard_task() — and with it
// housekeeping_task_kb() above — never runs. The rail has to be cut from these
// hooks instead of from the poll.
void suspend_power_down_kb(void) {
#ifdef RGB_MATRIX_ENABLE
    // quantum.c calls this hook before it sets the matrix suspend flag, and then
    // runs rgb_matrix_task() in the same pass -- whose indicator hook would turn
    // a lit frame's rail back on. The local flag closes that window.
    rgb_usb_suspended = true;
    rgb_power_task();
#endif

    suspend_power_down_user();
}

void suspend_wakeup_init_kb(void) {
#ifdef RGB_MATRIX_ENABLE
    // Here the matrix flag is already cleared, so the poll gives the right
    // answer and honours RGB_TOG — no flash of light on wake if the user had it
    // off. A lit effect then gets its rail back from its first frame.
    rgb_usb_suspended = false;
    rgb_power_task();
#endif

    suspend_wakeup_init_user();
}
