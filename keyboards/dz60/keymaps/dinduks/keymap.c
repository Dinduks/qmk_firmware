#include QMK_KEYBOARD_H

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

enum layers {
  _QWERTY = 0,
  _LOWER,
  _RAISE,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_QWERTY] = LAYOUT(
      KC_BSLS, KC_1, KC_2, KC_3, KC_4, KC_5, KC_6, KC_7, KC_8, KC_9, KC_0, KC_MINS, KC_EQL, KC_NO, KC_GRV,
      KC_TAB, KC_Q, KC_W, KC_E, KC_R, KC_T, KC_Y, KC_U, KC_I, KC_O, KC_P, TD(TD_BSLS_GRV), KC_RBRC, KC_BSPC,
      MT(MOD_LCTL,KC_ESC), KC_A, KC_S, KC_D, KC_F, KC_G, KC_H, KC_J, KC_K, KC_L, TD(TD_SCLN_COLN), KC_QUOT, KC_ENT,
      KC_LSFT, KC_NO, KC_Z, KC_X, KC_C, KC_V, KC_B, KC_N, KC_M, TD(TD_COMM_CCEDILLE), KC_DOT, KC_SLSH, MT(MOD_RSFT,KC_CAPS), KC_NO,
      KC_NO, KC_LGUI, KC_LALT, LT(_LOWER,KC_ENT), MT(MOD_LSFT, KC_BSPC), LT(_RAISE,KC_SPC), KC_RALT, MO(2), KC_NO, MO(1), KC_RCTL),

  [_LOWER] = LAYOUT(
      KC_GRV, KC_F1, KC_F2, KC_F3, KC_F4, KC_F5, KC_F6, KC_F7, KC_F8, KC_F9, KC_F10, KC_F11, KC_F12, KC_TRNS, RESET,
      KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_CALC, KC_PGUP, KC_HOME, KC_PGDN, KC_PSCR, KC_GRV, KC_TRNS, KC_DEL,
      KC_TRNS, RGB_TOG, RGB_HUI, RGB_SAI, RGB_VAI, RGB_MOD, KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT, KC_TRNS, KC_TRNS, KC_TRNS,
      KC_TRNS, KC_TRNS, RGB_HUD, RGB_SAD, RGB_VAD, RGB_RMOD, KC_TRNS, KC_END,  KC_APP, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
      KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

  [_RAISE] = LAYOUT(
      KC_TRNS, KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11, KC_F12, KC_TRNS, KC_TRNS,
      KC_TRNS, KC_EXLM, KC_AT, TD(TD_HASH_LCBR), TD(TD_DLR_LPRN), TD(TD_PERC_LBRC), TD(TD_CIRC_RBRC), TD(TD_AMPR_RPRN), TD(TD_ASTR_RCBR), TD(TD_UNDS_MINS), TD(TD_PLUS_EQL), KC_TRNS, KC_TRNS, KC_TRNS,
      KC_TRNS, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_TRNS, KC_TRNS,
      KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
      KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),
};
