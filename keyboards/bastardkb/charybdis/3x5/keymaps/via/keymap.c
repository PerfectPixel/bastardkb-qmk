/**
 * Copyright 2021 Charly Delay <charly@codesink.dev> (@0xcharly)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include QMK_KEYBOARD_H

#ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
#    include "timer.h"
#endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

enum custom_keycodes {
  SP_SCRL = SAFE_RANGE
};

enum {
    TAB_MCTL,
};

tap_dance_action_t tap_dance_actions[] = {
    [TAB_MCTL] = ACTION_TAP_DANCE_DOUBLE(KC_TAB, KC_MCTL),
};

/*        34 KEY MATRIX / LAYOUT MAPPING

  ╭─────────────────────┬─────────────────────╮
  │ LT4 LT3 LT2 LT1 LT0 │ RT0 RT1 RT2 RT3 RT4 │
  │ LM4 LM3 LM2 LM1 LM0 │ RM0 RM1 RM2 RM3 RM4 │
  │ LB4 LB3 LB2 LB1 LB0 │ RB0 RB1 RB2 RB3 RB4 │
  ╰───────╮ LH2 LH1 LH0 │ RH0 RH1 ╭───────────╯
          ╰─────────────┴─────────╯             */

enum combo_events {
  L_BRACKET_COMBO,  // LT3 and LM3 => [
  R_BRACKET_COMBO,  // RT3 and RM3 => ]
  L_PAREN_COMBO,    // LT2 and LM2 => (
  R_PAREN_COMBO,    // RT2 and RM2 => )
  L_BRACE_COMBO,    // LT1 and LM1 => {
  R_BRACE_COMBO,    // RT1 and RM1 => }
  L_ABK_COMBO,      // LT0 and LM0 => <
  R_ABK_COMBO,      // RT0 and RM0 => >
  L_MOUSE1_COMBO,   // LB1 AND LB2 => Mouse 1
  L_MOUSE2_COMBO,   // LB2 AND LB3 => Mouse 2
  L_MOUSE3_COMBO,   // LB1 AND LB3 => Mouse 3
  R_MOUSE1_COMBO,   // RB1 AND RB2 => Mouse 1
  R_MOUSE2_COMBO,   // RB2 AND RB3 => Mouse 2
  R_MOUSE3_COMBO,   // RB1 AND RB3 => Mouse 3
};

const uint16_t l_bracket_combo[] PROGMEM = {KC_W, LALT_T(KC_R), COMBO_END};
const uint16_t r_bracket_combo[] PROGMEM = {KC_Y, LALT_T(KC_I), COMBO_END};
const uint16_t l_paren_combo[] PROGMEM = {KC_F, LGUI_T(KC_S), COMBO_END};
const uint16_t r_paren_combo[] PROGMEM = {KC_U, RGUI_T(KC_E), COMBO_END};
const uint16_t l_brace_combo[] PROGMEM = {KC_P, LSFT_T(KC_T), COMBO_END};
const uint16_t r_brace_combo[] PROGMEM = {KC_L, RSFT_T(KC_N), COMBO_END};
const uint16_t l_abk_combo[] PROGMEM = {KC_B, KC_G, COMBO_END};
const uint16_t r_abk_combo[] PROGMEM = {KC_J, KC_M, COMBO_END};
const uint16_t l_mouse1_combo[] PROGMEM = {KC_C, KC_D, COMBO_END};
const uint16_t l_mouse2_combo[] PROGMEM = {KC_X, KC_C, COMBO_END};
const uint16_t l_mouse3_combo[] PROGMEM = {KC_X, KC_D, COMBO_END};
const uint16_t r_mouse1_combo[] PROGMEM = {KC_H, KC_COMM, COMBO_END};
const uint16_t r_mouse2_combo[] PROGMEM = {KC_COMM, KC_DOT, COMBO_END};
const uint16_t r_mouse3_combo[] PROGMEM = {KC_H, KC_DOT, COMBO_END};

combo_t key_combos[] = {
    [L_BRACKET_COMBO] = COMBO(l_bracket_combo, KC_LBRC),
    [R_BRACKET_COMBO] = COMBO(r_bracket_combo, KC_RBRC),
    [L_PAREN_COMBO] = COMBO(l_paren_combo, KC_LPRN),
    [R_PAREN_COMBO] = COMBO(r_paren_combo, KC_RPRN),
    [L_BRACE_COMBO] = COMBO(l_brace_combo, KC_LCBR),
    [R_BRACE_COMBO] = COMBO(r_brace_combo, KC_RCBR),
    [L_ABK_COMBO] = COMBO(l_abk_combo, KC_LABK),
    [R_ABK_COMBO] = COMBO(r_abk_combo, KC_RABK),
    [L_MOUSE1_COMBO] = COMBO(l_mouse1_combo, KC_BTN1),
    [L_MOUSE2_COMBO] = COMBO(l_mouse2_combo, KC_BTN2),
    [L_MOUSE3_COMBO] = COMBO(l_mouse3_combo, KC_BTN3),
    [R_MOUSE1_COMBO] = COMBO(r_mouse1_combo, KC_BTN1),
    [R_MOUSE2_COMBO] = COMBO(r_mouse2_combo, KC_BTN2),
    [R_MOUSE3_COMBO] = COMBO(r_mouse3_combo, KC_BTN3),
};

// key overrides
const key_override_t dot_key_override =
    ko_make_basic(MOD_MASK_SHIFT, KC_DOT, KC_COLN);  // Shift . is :
const key_override_t comm_key_override =
    ko_make_basic(MOD_MASK_SHIFT, KC_COMM, KC_SCLN); // Shift , is ;

const key_override_t** key_overrides = (const key_override_t*[]){
    &dot_key_override,
    &comm_key_override,
    NULL
};

enum charybdis_keymap_layers {
    LAYER_BASE_COLEMAK = 0,
    LAYER_BASE_QWERTY,
    LAYER_MEDIA,
    LAYER_POINTER,
    LAYER_NUMERAL,
    LAYER_SYMBOLS,
};

// Automatically enable sniping-mode on the pointer layer.
#define CHARYBDIS_AUTO_SNIPING_ON_LAYER LAYER_POINTER

#ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
static uint16_t auto_pointer_layer_timer = 0;

#    ifndef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS
#        define CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS 1000
#    endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS

#    ifndef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD
#        define CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD 8
#    endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD
#endif     // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

#define ESC_MED LT(LAYER_MEDIA, KC_ESC)
#define ENT_SYM LT(LAYER_SYMBOLS, KC_ENT)
#define BSP_NUM LT(LAYER_NUMERAL, KC_BSPC)
#define _L_PTR(KC) LT(LAYER_POINTER, KC)
#define QWERTY DF(LAYER_BASE_QWERTY)
#define COLEMAK DF(LAYER_BASE_COLEMAK)
#define TD_TABM TD(TAB_MCTL)

#ifndef POINTING_DEVICE_ENABLE
#    define DRGSCRL KC_NO
#    define DPI_MOD KC_NO
#    define S_D_MOD KC_NO
#    define SNIPING KC_NO
#endif // !POINTING_DEVICE_ENABLE

// clang-format off
/** \brief QWERTY layout (3 rows, 10 columns). */
#define LAYOUT_LAYER_BASE_QWERTY                                                              \
       KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P, \
       KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L, KC_QUOT, \
       KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M, KC_COMM,  KC_DOT, KC_SLSH, \
                      ESC_MED, SP_SCRL, TD_TABM, ENT_SYM, BSP_NUM

/** \brief COLEMAK Mod-DH layout (3 rows, 10 columns). */
#define LAYOUT_LAYER_BASE_COLEMAK                                                             \
       KC_Q,    KC_W,    KC_F,    KC_P,    KC_B,    KC_J,    KC_L,    KC_U,    KC_Y, KC_QUOT, \
       KC_A,    KC_R,    KC_S,    KC_T,    KC_G,    KC_M,    KC_N,    KC_E,    KC_I,    KC_O, \
       KC_Z,    KC_X,    KC_C,    KC_D,    KC_V,    KC_K,    KC_H, KC_COMM,  KC_DOT, KC_SLSH, \
                      ESC_MED, SP_SCRL, TD_TABM, ENT_SYM, BSP_NUM

/** Convenience row shorthands. */
#define _______________DEAD_HALF_ROW_______________ XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
#define ______________HOME_ROW_CAGS_L______________ KC_LCTL, KC_LALT, KC_LGUI, KC_LSFT, XXXXXXX
#define ______________HOME_ROW_CAGS_R______________ XXXXXXX, KC_LSFT, KC_LGUI, KC_LALT, KC_LCTL

/*
 * Layers used on the Charybdis Nano.
 *
 * These layers started off heavily inspired by the Miryoku layout, but trimmed
 * down and tailored for a stock experience that is meant to be fundation for
 * further personalization.
 *
 * See https://github.com/manna-harbour/miryoku for the original layout.
 */

/**
 * \brief Media layer.
 *
 * Tertiary left- and right-hand layer is media and RGB control.  This layer is
 * symmetrical to accomodate the left- and right-hand trackball.
 */
#define LAYOUT_LAYER_MEDIA                                                                    \
    XXXXXXX,RGB_RMOD, RGB_TOG, RGB_MOD, XXXXXXX, XXXXXXX,RGB_RMOD, RGB_TOG, RGB_MOD, XXXXXXX, \
    KC_MPRV, KC_VOLD, KC_MUTE, KC_VOLU, KC_MNXT, KC_MPRV, KC_VOLD, KC_MUTE, KC_VOLU, KC_MNXT, \
    XXXXXXX, XXXXXXX, XXXXXXX,  EE_CLR, QK_BOOT, QK_BOOT,  EE_CLR, XXXXXXX,  QWERTY, COLEMAK, \
                      _______, KC_MPLY, KC_MSTP, KC_MSTP, KC_MPLY

/** \brief Mouse emulation and pointer functions. */
#define LAYOUT_LAYER_POINTER                                                                  \
    XXXXXXX, XXXXXXX, XXXXXXX, DPI_MOD, S_D_MOD,  KC_INS, XXXXXXX,  KC_UP,  XXXXXXX, XXXXXXX, \
    ______________HOME_ROW_CAGS_L______________, KC_CAPS, KC_LEFT, KC_DOWN, KC_RGHT, XXXXXXX, \
    _______, DRGSCRL, SNIPING, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, SNIPING, DRGSCRL, _______, \
                      KC_BTN2, KC_BTN1, KC_BTN3,  KC_ENT, KC_BSPC

/**
 * \brief Numeral layout.
 *
 * Primary left-hand layer (right home thumb) is numerals and symbols. Numerals
 * are in the standard numpad locations with symbols in the remaining positions.
 * `KC_DOT` is duplicated from the base layer.
 */
#define LAYOUT_LAYER_NUMERAL                                                                  \
    KC_LBRC,    KC_7,    KC_8,    KC_9, KC_RBRC, _______________DEAD_HALF_ROW_______________, \
    KC_SCLN,    KC_4,    KC_5,    KC_6,  KC_EQL, ______________HOME_ROW_CAGS_R______________, \
     KC_GRV,    KC_1,    KC_2,    KC_3, KC_BSLS, _______________DEAD_HALF_ROW_______________, \
                       KC_DOT,    KC_0, KC_MINS, XXXXXXX, _______

/**
 * \brief Symbols layer.
 *
 * Secondary left-hand layer has shifted symbols in the same locations to reduce
 * chording when using mods with shifted symbols. `KC_LPRN` is duplicated next to
 * `KC_RPRN`.
 */
#define LAYOUT_LAYER_SYMBOLS                                                                  \
    KC_LCBR, KC_AMPR, KC_ASTR, KC_LPRN, KC_RCBR, _______________DEAD_HALF_ROW_______________, \
    KC_COLN,  KC_DLR, KC_PERC, KC_CIRC, KC_PLUS, ______________HOME_ROW_CAGS_R______________, \
    KC_TILD, KC_EXLM,   KC_AT, KC_HASH, KC_PIPE, _______________DEAD_HALF_ROW_______________, \
                      KC_LPRN, KC_RPRN, KC_UNDS, _______, XXXXXXX

/**
 * \brief Add Home Row mod to a layout.
 *
 * Expects a 10-key per row layout.  Adds support for GACS (Gui, Alt, Ctl, Shift)
 * home row.  The layout passed in parameter must contain at least 20 keycodes.
 *
 * This is meant to be used with `LAYER_ALPHAS_QWERTY` defined above, eg.:
 *
 *     HOME_ROW_MOD_GACS(LAYER_ALPHAS_QWERTY)
 */
#define _HOME_ROW_MOD_GACS(                                            \
    L00, L01, L02, L03, L04, R05, R06, R07, R08, R09,                  \
    L10, L11, L12, L13, L14, R15, R16, R17, R18, R19,                  \
    ...)                                                               \
             L00,         L01,         L02,         L03,         L04,  \
             R05,         R06,         R07,         R08,         R09,  \
      LCTL_T(L10), LALT_T(L11), LGUI_T(L12), LSFT_T(L13),        L14,  \
             R15,  RSFT_T(R16), RGUI_T(R17), LALT_T(R18), RCTL_T(R19), \
      __VA_ARGS__
#define HOME_ROW_MOD_GACS(...) _HOME_ROW_MOD_GACS(__VA_ARGS__)

/**
 * \brief Add pointer layer keys to a layout.
 *
 * Expects a 10-key per row layout.  The layout passed in parameter must contain
 * at least 30 keycodes.
 *
 * This is meant to be used with `LAYER_ALPHAS_QWERTY` defined above, eg.:
 *
 *     POINTER_MOD(LAYER_ALPHAS_QWERTY)
 */
#define _POINTER_MOD(                                                  \
    L00, L01, L02, L03, L04, R05, R06, R07, R08, R09,                  \
    L10, L11, L12, L13, L14, R15, R16, R17, R18, R19,                  \
    L20, L21, L22, L23, L24, R25, R26, R27, R28, R29,                  \
    ...)                                                               \
             L00,         L01,         L02,         L03,         L04,  \
             R05,         R06,         R07,         R08,         R09,  \
             L10,         L11,         L12,         L13,         L14,  \
             R15,         R16,         R17,         R18,         R19,  \
      _L_PTR(L20),        L21,         L22,         L23,         L24,  \
             R25,         R26,         R27,         R28,  _L_PTR(R29), \
      __VA_ARGS__
#define POINTER_MOD(...) _POINTER_MOD(__VA_ARGS__)

#define LAYOUT_wrapper(...) LAYOUT(__VA_ARGS__)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [LAYER_BASE_QWERTY] = LAYOUT_wrapper(
    POINTER_MOD(HOME_ROW_MOD_GACS(LAYOUT_LAYER_BASE_QWERTY))
  ),
  [LAYER_BASE_COLEMAK] = LAYOUT_wrapper(
    POINTER_MOD(HOME_ROW_MOD_GACS(LAYOUT_LAYER_BASE_COLEMAK))
  ),
  [LAYER_MEDIA] = LAYOUT_wrapper(LAYOUT_LAYER_MEDIA),
  [LAYER_NUMERAL] = LAYOUT_wrapper(LAYOUT_LAYER_NUMERAL),
  [LAYER_POINTER] = LAYOUT_wrapper(LAYOUT_LAYER_POINTER),
  [LAYER_SYMBOLS] = LAYOUT_wrapper(LAYOUT_LAYER_SYMBOLS),
};
// clang-format on

#ifdef POINTING_DEVICE_ENABLE
#    ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
    if (abs(mouse_report.x) > CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD || abs(mouse_report.y) > CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD) {
        if (auto_pointer_layer_timer == 0) {
            layer_on(LAYER_POINTER);
#        ifdef RGB_MATRIX_ENABLE
            rgb_matrix_mode_noeeprom(RGB_MATRIX_NONE);
            rgb_matrix_sethsv_noeeprom(HSV_GREEN);
#        endif // RGB_MATRIX_ENABLE
        }
        auto_pointer_layer_timer = timer_read();
    }
    return mouse_report;
}

void matrix_scan_user(void) {
    if (auto_pointer_layer_timer != 0 && TIMER_DIFF_16(timer_read(), auto_pointer_layer_timer) >= CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS) {
        auto_pointer_layer_timer = 0;
        layer_off(LAYER_POINTER);
#        ifdef RGB_MATRIX_ENABLE
        rgb_matrix_mode_noeeprom(RGB_MATRIX_DEFAULT_MODE);
#        endif // RGB_MATRIX_ENABLE
    }
}
#    endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

#    ifdef CHARYBDIS_AUTO_SNIPING_ON_LAYER
layer_state_t layer_state_set_user(layer_state_t state) {
    charybdis_set_pointer_sniping_enabled(layer_state_cmp(state, CHARYBDIS_AUTO_SNIPING_ON_LAYER));

    #ifdef RGB_MATRIX_ENABLE
    switch(get_highest_layer(state)) {
        case LAYER_MEDIA:
            // HSV_YELLOW
            rgb_matrix_sethsv_noeeprom(43, 255, 64);
            break;

        case LAYER_NUMERAL:
            // HSV_PURPLE
            rgb_matrix_sethsv_noeeprom(191, 255, 64);
            break;

        case LAYER_SYMBOLS:
            // HSV_MAGENTA
            rgb_matrix_sethsv_noeeprom(213, 255, 64);
            break;

        case LAYER_POINTER:
            // HSV_BLUE
            rgb_matrix_sethsv_noeeprom(170, 255, 64);
            break;

        case LAYER_BASE_COLEMAK:
        case LAYER_BASE_QWERTY:
        default:
            // HSV_GREEN
            rgb_matrix_sethsv_noeeprom(85, 255, 64);
            break;
    }
    #endif

    return state;
}
#    endif // CHARYBDIS_AUTO_SNIPING_ON_LAYER
#endif     // POINTING_DEVICE_ENABLE

#ifdef RGB_MATRIX_ENABLE
// Forward-declare this helper function since it is defined in
// rgb_matrix.c.
void rgb_matrix_update_pwm_buffers(void);

// Set breathing on boot
void keyboard_post_init_user(void) {
    rgb_matrix_mode_noeeprom(RGB_MATRIX_BREATHING);
    // HSV_GREEN
    rgb_matrix_sethsv_noeeprom(85, 255, 64);
}
#endif

// see: https://github.com/qmk/qmk_firmware/blob/master/docs/mod_tap.md
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    static uint16_t sp_scrl_timer;

    switch (keycode) {
        case SP_SCRL:
            if(record->event.pressed) {
                sp_scrl_timer = timer_read();
                charybdis_set_pointer_dragscroll_enabled(true);
                // HSV_BLUE
                rgb_matrix_sethsv_noeeprom(170, 255, 64);
            } else {
                charybdis_set_pointer_dragscroll_enabled(false);
                // HSV_GREEN
                rgb_matrix_sethsv_noeeprom(85, 255, 64);
                if (timer_elapsed(sp_scrl_timer) < TAPPING_TERM) {
                    tap_code16(KC_SPC);
                }
            }
            return false; // We handled this keypress
    }
    return true;
}

bool get_custom_auto_shifted_key(uint16_t keycode, keyrecord_t *record) {
    switch(keycode) {
        case LCTL_T(KC_A):
        case LALT_T(KC_R):
        case LGUI_T(KC_S):
        case LSFT_T(KC_T):
        case RSFT_T(KC_N):
        case RGUI_T(KC_E):
        case LALT_T(KC_I):
        case RCTL_T(KC_O):
        case _L_PTR(KC_Z):
        case _L_PTR(KC_SLSH):
            return true;
        default:
            return false;
    }
}

void autoshift_press_user(uint16_t keycode, bool shifted, keyrecord_t *record) {
    switch(keycode) {
        case KC_COMM:
            register_code16((!shifted) ? KC_COMM : KC_SCLN);
            break;
        case KC_DOT:
            register_code16((!shifted) ? KC_DOT : KC_COLN);
            break;
        default:
            if (shifted) {
                add_weak_mods(MOD_BIT(KC_LSFT));
            }
            // & 0xFF gets the Tap key for Tap Holds, required when using Retro Shift
            register_code16((IS_RETRO(keycode)) ? keycode & 0xFF : keycode);
    }
}

void autoshift_release_user(uint16_t keycode, bool shifted, keyrecord_t *record) {
    switch(keycode) {
        case KC_COMM:
            unregister_code16((!shifted) ? KC_COMM : KC_SCLN);
            break;
        case KC_DOT:
            unregister_code16((!shifted) ? KC_DOT : KC_COLN);
            break;
        default:
            // & 0xFF gets the Tap key for Tap Holds, required when using Retro Shift
            // The IS_RETRO check isn't really necessary here, always using
            // keycode & 0xFF would be fine.
            unregister_code16((IS_RETRO(keycode)) ? keycode & 0xFF : keycode);
    }
}
