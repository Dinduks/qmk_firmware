/* Copyright 2020 Samy Dindane <samy@dindane.com>
 * Based upon the default Kyria configuration by Thomas Baart
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
#include "hypefury_logos.c"

enum {
  TD_BSLS_GRV = 0,
  TD_LBRC_RBRC,
  TD_LCBR_RCBR,
  TD_LPRN_RPRN,
  TD_SCLN_COLN,
  TD_COMM_CCEDILLE,
  TD_DOT_ELLIPSIS,
  TD_UNDS_MINS,
  TD_PLUS_EQL,
  TD_HASH_LCBR,
  TD_DLR_LPRN,
  TD_PERC_LBRC,
  TD_CIRC_RBRC,
  TD_AMPR_RPRN,
  TD_ASTR_RCBR,
};

qk_tap_dance_action_t tap_dance_actions[] = {
  [TD_BSLS_GRV]  = ACTION_TAP_DANCE_DOUBLE(KC_BSLS, KC_GRV),
  [TD_LBRC_RBRC]  = ACTION_TAP_DANCE_DOUBLE(KC_LBRC, KC_RBRC),
  [TD_LCBR_RCBR]  = ACTION_TAP_DANCE_DOUBLE(KC_LCBR, KC_RCBR),
  [TD_LPRN_RPRN]  = ACTION_TAP_DANCE_DOUBLE(KC_LPRN, KC_RPRN),
  [TD_SCLN_COLN]  = ACTION_TAP_DANCE_DOUBLE(KC_SCLN, KC_COLN),
  [TD_COMM_CCEDILLE]  = ACTION_TAP_DANCE_DOUBLE(KC_COMM, RALT(KC_COMM)),
  [TD_DOT_ELLIPSIS]  = ACTION_TAP_DANCE_DOUBLE(KC_DOT, UC(0x2026)),
  [TD_UNDS_MINS] = ACTION_TAP_DANCE_DOUBLE(KC_UNDS, KC_MINS),
  [TD_PLUS_EQL] = ACTION_TAP_DANCE_DOUBLE(KC_PLUS, KC_EQL),
  [TD_HASH_LCBR] = ACTION_TAP_DANCE_DOUBLE(KC_HASH, KC_LCBR),
  [TD_DLR_LPRN] = ACTION_TAP_DANCE_DOUBLE(KC_DLR, KC_LPRN),
  [TD_PERC_LBRC] = ACTION_TAP_DANCE_DOUBLE(KC_PERC, KC_LBRC),
  [TD_CIRC_RBRC] = ACTION_TAP_DANCE_DOUBLE(KC_CIRC, KC_RBRC),
  [TD_AMPR_RPRN] = ACTION_TAP_DANCE_DOUBLE(KC_AMPR, KC_RPRN),
  [TD_ASTR_RCBR] = ACTION_TAP_DANCE_DOUBLE(KC_ASTR, KC_RCBR),
};

void matrix_init_user(void) {
  set_unicode_input_mode(UC_LNX);
}

void led_set_user(uint8_t usb_led) {
  if (!IS_LED_ON(host_keyboard_leds(), USB_LED_NUM_LOCK)) {
    tap_code(KC_NUMLOCK);
  }
}

enum layers {
  _QWERTY = 0,
  _GAMING,
  _LOWER,
  _RAISE,
};

// Combos
enum combo_events {
  TOGGLE_GAMING_MODE,
};

const uint16_t PROGMEM toggle_gaming_mode_combo[] = {KC_A, KC_V, COMBO_END};
combo_t key_combos[COMBO_COUNT] = {
  [TOGGLE_GAMING_MODE] = COMBO_ACTION(toggle_gaming_mode_combo),
};

void process_combo_event(uint8_t combo_index, bool pressed) {
  switch(combo_index) {
    case TOGGLE_GAMING_MODE:
      if (pressed) {
        layer_invert(_GAMING);
      }
      break;
  }
}
// Combos end

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
/*
 * Base Layer: QWERTY
 *
 * ,-------------------------------------------.                              ,-------------------------------------------.
 * |  Tab   |   Q  |   W  |   E  |   R  |   T  |                              |   Y  |   U  |   I  |   O  |   P  |  | \   |
 * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
 * |Ctrl/ESC|   A  |   S  |  D   |   F  |   G  |                              |   H  |   J  |   K  |   L  | ;  : |  ' "   |
 * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
 * | LShift |   Z  |   X  |   C  |   V  |   B  | Space| Frag |  |      |      |   N  |   M  | ,  < | . >  | /  ? | RShift |
 * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
 *                        | Alt  | GUI  | Enter|LShift| Ctrl |  | Ctrl |LShift| Space| Bksp | AltGr|
 *                        |      |      | Lower|      |      |  |      |      | Raise| Symb |      |
 *                        `----------------------------------'  `----------------------------------'
 */
  [_QWERTY] = LAYOUT(
    KC_TAB,                  KC_Q,   KC_W,   KC_E,   KC_R,   KC_T,                                     KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    TD(TD_BSLS_GRV),
    MT(MOD_LCTL,KC_ESC),     KC_A,   KC_S,   KC_D,   KC_F,   KC_G,                                     KC_H,    KC_J,    KC_K,    KC_L,    TD(TD_SCLN_COLN), KC_QUOT,
    KC_LSFT,                 KC_Z,   KC_X,   KC_C,   KC_V,   KC_B, KC_SPC, TO(_GAMING), _______, _______, KC_N,    KC_M,    TD(TD_COMM_CCEDILLE), KC_DOT,  KC_SLSH, MT(MOD_RSFT,KC_CAPS),
             KC_LALT, KC_LGUI, LT(_LOWER,KC_ENT), KC_LSFT, KC_LCTL, KC_LCTL, KC_LSFT, LT(_RAISE,KC_SPC), KC_BSPC, KC_RALT
  ),

/*
 * Base Layer: GAMING
 *
 * ,-------------------------------------------.                              ,-------------------------------------------.
 * |  Tab   |   Q  |   W  |   E  |   R  |   T  |                              |   Y  |   U  |   I  |   O  |   P  |  `     |
 * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
 * |  Ctrl  |   A  |   S  |  D   |   F  |   G  |                              |   H  |   J  |   K  |   L  | ;  : |  ' "   |
 * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
 * |  Esc   |   Z  |   X  |   C  |   V  |   B  |Alt+F9| Qwe  |  |      |      |   N  |   M  | ,  < | . >  | /  ? | RShift |
 * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
 *                        | Num1 | Num2 | Enter|LShift| Lower|  |      |LShift| Space| Bksp | Num4 |
 *                        |      |      |      |      | Num3 |  |      |      | Raise| Symb |      |
 *                        `----------------------------------'  `----------------------------------'
 */
  [_GAMING] = LAYOUT(
    KC_TAB,  KC_Q,   KC_W,   KC_E,   KC_R,   KC_T,                                     KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_GRV,
    KC_LCTRL,KC_A,   KC_S,   KC_D,   KC_F,   KC_G,                                     KC_H,    KC_J,    KC_K,    KC_L,    TD(TD_SCLN_COLN), KC_QUOT,
    KC_ESC, KC_Z,   KC_X,   KC_C,   KC_V,   KC_B, A(KC_F9), TO(_QWERTY), _______, _______, KC_N,    KC_M,    TD(TD_COMM_CCEDILLE), KC_DOT,  KC_SLSH, KC_KP_4,
             KC_KP_1, KC_KP_2, KC_ENT, KC_LSFT, LT(_LOWER,KC_KP_3), _______, KC_LSFT, LT(_RAISE,KC_SPC), KC_BSPC, KC_RALT

  ),

/*
 * Lower layer
 *
 * ,-------------------------------------------.                              ,-------------------------------------------.
 * |        |      |      |      |      |      |                              | Calc | PgUp | Home |PgDown|PrtScr| `      |
 * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
 * |   ▽    | TOG  | HUI  | SAI  | VAI  | MOD  |                              | Left | Down | Up   | Right|      |        |
 * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
 * |   ▽    |      | HUD  | SAD  | VAD  | RMOD |      |      |  |      |      | End  | Menu |      |      |      |  ▽     |
 * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
 *                        |      |      |      |      |  ▽   |  |      |      |      | Del  |      |
 *                        |      |      |      |      |      |  |      |      |      |      |      |
 *                        `----------------------------------'  `----------------------------------'
 */
  [_LOWER] = LAYOUT(
    _______, _______, _______, _______, _______, _______,                                     KC_CALC, KC_PGUP, KC_HOME, KC_PGDN, KC_PSCR, KC_GRV,
    KC_TRNS, RGB_TOG, RGB_HUI, RGB_SAI, RGB_VAI, RGB_MOD,                                     KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT, _______, _______,
    KC_TRNS, _______, RGB_HUD, RGB_SAD, RGB_VAD, RGB_RMOD,_______, _______, _______, _______, KC_END,  KC_APP,  _______, _______, _______, KC_TRNS,
                               KC_TRNS, KC_TRNS, _______, _______, _______, _______, _______, _______, KC_DEL,  _______
  ),

/*
 * Raise Layer: Number keys, special chars and function keys
 *
 * ,-------------------------------------------.                              ,-------------------------------------------.
 * |        |  !   |  @   |  # { |  $ ( |  % [ |                              |  ^ ] |  & ) |  * } |  _ - |  + = |  F12   |
 * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
 * |   ▽    |  1   |  2   |  3   |  4   |  5   |                              |  6   |  7   |  8   |  9   |  0   |  F11   |
 * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
 * |   ▽    |  F1  |  F2  | F3   | F4   | F5   |                              | F6   | F7   |  F8  | F9   | F10  |  ▽     |
 * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
 *                        |      |  ▽   |      |      |  ▽   |  |      |      |      |      |      |
 *                        |      |      |      |      |      |  |      |      |      |      |      |
 *                        `----------------------------------'  `----------------------------------'
 */
  [_RAISE] = LAYOUT(
    _______, KC_EXLM, KC_AT,   TD(TD_HASH_LCBR), TD(TD_DLR_LPRN), TD(TD_PERC_LBRC),           TD(TD_CIRC_RBRC), TD(TD_AMPR_RPRN), TD(TD_ASTR_RCBR), TD(TD_UNDS_MINS), TD(TD_PLUS_EQL), KC_F12,
    KC_TRNS, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,                                        KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_F11,
    KC_TRNS, KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   _______, _______, _______, _______, KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_TRNS,
                               KC_TRNS, KC_TRNS, _______, _______, _______, _______, _______, _______, _______, _______
  ),

///*
// * Layer template
// *
// * ,-------------------------------------------.                              ,-------------------------------------------.
// * |        |      |      |      |      |      |                              |      |      |      |      |      |        |
// * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
// * |        |      |      |      |      |      |                              |      |      |      |      |      |        |
// * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
// * |   ▽    |      |      |      |      |      |      |      |  |      |      |      |      |      |      |      |    ▽   |
// * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
// *                        |      |  ▽   |      |      |  ▽   |  |      |      |      |      |      |
// *                        |      |      |      |      |      |  |      |      |      |      |      |
// *                        `----------------------------------'  `----------------------------------'
// */
//    [_LAYERINDEX] = LAYOUT(
//      _______, _______, _______, _______, _______, _______,                                     _______, _______, _______, _______, _______, _______,
//      KC_TRNS, _______, _______, _______, _______, _______,                                     _______, _______, _______, _______, _______, _______,
//      KC_TRNS, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_TRNS,
//                                 _______, KC_TRNS, _______, _______, KC_TRNS, _______, _______, _______, _______, _______
//    ),
};

#ifdef OLED_DRIVER_ENABLE
oled_rotation_t oled_init_user(oled_rotation_t rotation) {
  return OLED_ROTATION_180;
}

static void render_status(void) {
  oled_write_P(PSTR("\n   H Y P E F U R Y\n"), false);

  oled_write_P(PSTR("\nKyria rev1.0\n\n"), false);

  oled_write_P(PSTR("Layer: "), false);
  switch (get_highest_layer(layer_state)) {
    case _QWERTY:
      oled_write_P(PSTR("Default\n"), false);
      break;
    case _GAMING:
      oled_write_P(PSTR("FR4G M0D3\n"), false);
      break;
    case _LOWER:
      oled_write_P(PSTR("Lower\n"), false);
      break;
    case _RAISE:
      oled_write_P(PSTR("Raise\n"), false);
      break;
    default:
      oled_write_P(PSTR("Undefined\n"), false);
  }

  uint8_t led_usb_state = host_keyboard_leds();
  oled_write_P(IS_LED_ON(led_usb_state, USB_LED_CAPS_LOCK) ? PSTR("CAPLCK ") : PSTR("       "), false);
  oled_write_P(IS_LED_ON(led_usb_state, USB_LED_SCROLL_LOCK) ? PSTR("SCRLCK ") : PSTR("       "), false);
}

void oled_task_user(void) {
  if (is_keyboard_master()) {
    render_status();
  } else {
    render_hypefury_logo();
  }
}
#endif

#ifdef ENCODER_ENABLE
void encoder_update_user(uint8_t index, bool clockwise) {
  if (index == 0) {
    if (clockwise) {
      tap_code(KC_VOLU);
    } else {
      tap_code(KC_VOLD);
    }
  } else if (index == 1) {
    if (clockwise) {
      tap_code(KC_PGDN);
    } else {
      tap_code(KC_PGUP);
    }
  }
}
#endif
