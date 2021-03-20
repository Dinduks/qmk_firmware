/* Copyright 2015-2017 Jack Humbert
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
#include "muse.h"

enum planck_layers {
  _QWERTY,
  _LOWER,
  _RAISE,
  _ADJUST
};

enum planck_keycodes {
  QWERTY = SAFE_RANGE,
  BACKLIT,
};

enum {
  TD_BSLS_GRV = 0,
  TD_LBRC_RBRC,
  TD_LCBR_RCBR,
  TD_LPRN_RPRN,
  TD_SCLN_COLN,
  TD_COMM_CCEDILLE,
//   TD_DOT_ELLIPSIS,
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
//   [TD_DOT_ELLIPSIS]  = ACTION_TAP_DANCE_DOUBLE(KC_DOT, UC(0x2026)),
  [TD_UNDS_MINS] = ACTION_TAP_DANCE_DOUBLE(KC_UNDS, KC_MINS),
  [TD_PLUS_EQL] = ACTION_TAP_DANCE_DOUBLE(KC_PLUS, KC_EQL),
  [TD_HASH_LCBR] = ACTION_TAP_DANCE_DOUBLE(KC_HASH, KC_LCBR),
  [TD_DLR_LPRN] = ACTION_TAP_DANCE_DOUBLE(KC_DLR, KC_LPRN),
  [TD_PERC_LBRC] = ACTION_TAP_DANCE_DOUBLE(KC_PERC, KC_LBRC),
  [TD_CIRC_RBRC] = ACTION_TAP_DANCE_DOUBLE(KC_CIRC, KC_RBRC),
  [TD_AMPR_RPRN] = ACTION_TAP_DANCE_DOUBLE(KC_AMPR, KC_RPRN),
  [TD_ASTR_RCBR] = ACTION_TAP_DANCE_DOUBLE(KC_ASTR, KC_RCBR),
};

#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

/* Qwerty
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |   Q  |   W  |   E  |   R  |   T  |   Y  |   U  |   I  |   O  |   P  | | \  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |Ct/ESC|   A  |   S  |   D  |   F  |   G  |   H  |   J  |   K  |   L  | ; :  | ' "  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |LShift|   Z  |   X  |   C  |   V  |   B  |   N  |   M  |   ,  |   .  |   /  |RShift|
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |Vol + |Vol - | Alt  | GUI  |Lower |LShift|LShift|Space | Bksp | Alt  |  Up  |Down  |
 * |      |      |      |      |Enter |      |      |Raise |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_QWERTY] = LAYOUT_planck_grid(
    KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    TD(TD_BSLS_GRV),
    MT(MOD_LCTL,KC_ESC),  KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    TD(TD_SCLN_COLN), KC_QUOT,
    KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    TD(TD_COMM_CCEDILLE), KC_DOT,  KC_SLSH, MT(MOD_RSFT,KC_CAPS) ,
    KC_VOLD, KC_VOLU, KC_LALT, KC_LGUI, LT(_LOWER,KC_ENT),   KC_LSFT,  KC_LSFT,  LT(_RAISE,KC_SPC),   KC_BSPC, KC_RALT, KC_UP,  KC_DOWN
),

/* Lower
 * ,-----------------------------------------------------------------------------------.
 * |      |      |      |      |      |      | Calc | PgUp | Home |PgDown|PrtScr| `    |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | TOG  | HUI  | SAI  | VAI  | MOD  | Left | Down | Up   | Right|      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      | HUD  | SAD  | VAD  | RMOD | End  | Menu |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             | Del  |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_LOWER] = LAYOUT_planck_grid(
    _______, _______, _______, _______, _______, _______,  KC_CALC, KC_PGUP, KC_HOME, KC_PGDN, KC_PSCR, KC_GRV,
    KC_TRNS, RGB_TOG, RGB_HUI, RGB_SAI, RGB_VAI, RGB_MOD,  KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT, _______, _______,
    KC_TRNS, _______, RGB_HUD, RGB_SAD, RGB_VAD, RGB_RMOD, KC_END,  KC_APP,  _______, _______, _______, KC_TRNS,
    _______, _______, _______, _______, _______, _______, _______, _______,  KC_DEL,  _______, _______, _______
),

/* Raise
 * ,-----------------------------------------------------------------------------------.
 * |      |  !   |  @   | # {  | $ (  | % [  | ^ ]  | & )  | * }  | _ -  | + =  | F12  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |  1   |  2   |  3   |  4   |  5   |  6   |  7   |  8   |  9   |  0   | F11  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |  F1  |  F2  | F3   | F4   | F5   | F6   | F7   |  F8  | F9   | F10  |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Play | Next |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_RAISE] = LAYOUT_planck_grid(
    _______, KC_EXLM, KC_AT,   TD(TD_HASH_LCBR), TD(TD_DLR_LPRN), TD(TD_PERC_LBRC), TD(TD_CIRC_RBRC), TD(TD_AMPR_RPRN), TD(TD_ASTR_RCBR), TD(TD_UNDS_MINS), TD(TD_PLUS_EQL), KC_F12,
    KC_TRNS, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_F11,
    KC_TRNS, KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_TRNS,
    KC_MPLY, KC_MNXT, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
),

/* Adjust (Lower + Raise)
 *                      v------------------------RGB CONTROL--------------------v
 * ,-----------------------------------------------------------------------------------.
 * |      | Reset|Debug |      |      |      |      |      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |MUSmod|Aud on|Audoff|AGnorm|AGswap|      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |Voice-|Voice+|Mus on|Musoff|MIDIon|MIDIof|TermOn|TermOf|      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_ADJUST] = LAYOUT_planck_grid(
    _______, RESET,   DEBUG, _______,  _______, _______, _______, _______, _______,  _______, _______, _______,
    _______, _______, MU_MOD,  AU_ON,   AU_OFF, AG_NORM, AG_SWAP, _______, _______,  _______, _______, _______,
    _______, MUV_DE,  MUV_IN,  MU_ON,   MU_OFF, MI_ON,   MI_OFF,  TERM_ON, TERM_OFF, _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______
)

};

layer_state_t layer_state_set_user(layer_state_t state) {
  return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case QWERTY:
      if (record->event.pressed) {
        print("mode just switched to qwerty and this is a huge string\n");
        set_single_persistent_default_layer(_QWERTY);
      }
      return false;
      break;
    case BACKLIT:
      if (record->event.pressed) {
        register_code(KC_RSFT);
        #ifdef BACKLIGHT_ENABLE
          backlight_step();
        #endif
        #ifdef KEYBOARD_planck_rev5
          writePinLow(E6);
        #endif
      } else {
        unregister_code(KC_RSFT);
        #ifdef KEYBOARD_planck_rev5
          writePinHigh(E6);
        #endif
      }
      return false;
      break;
  }
  return true;
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_DOWN);
      #else
        tap_code(KC_PGDN);
      #endif
    } else {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_UP);
      #else
        tap_code(KC_PGUP);
      #endif
    }
  }
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    } else {
        if (muse_counter) {
            stop_all_notes();
            muse_counter = 0;
        }
    }
#endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
