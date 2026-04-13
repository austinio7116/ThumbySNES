/*
 * ThumbySNES — Thumby Color physical button reader.
 *
 * GPIO map (active low, internal pull-ups):
 *   GP0  LEFT     GP1  UP       GP2  RIGHT     GP3  DOWN
 *   GP21 A        GP25 B        GP6  LB        GP22 RB
 *   GP26 MENU
 *
 * Snapshot + previous-snapshot model — poll once per frame to update
 * edge-detection state, then is_pressed / just_pressed without further
 * GPIO reads.
 */
#include "snes_buttons.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define BTN_LEFT_GP   0
#define BTN_UP_GP     1
#define BTN_RIGHT_GP  2
#define BTN_DOWN_GP   3
#define BTN_LB_GP     6
#define BTN_A_GP     21
#define BTN_RB_GP    22
#define BTN_B_GP     25
#define BTN_MENU_GP  26

static const uint8_t s_pins[SNES_BTN_COUNT] = {
    [SNES_BTN_LEFT]  = BTN_LEFT_GP,
    [SNES_BTN_RIGHT] = BTN_RIGHT_GP,
    [SNES_BTN_UP]    = BTN_UP_GP,
    [SNES_BTN_DOWN]  = BTN_DOWN_GP,
    [SNES_BTN_A]     = BTN_A_GP,
    [SNES_BTN_B]     = BTN_B_GP,
    [SNES_BTN_LB]    = BTN_LB_GP,
    [SNES_BTN_RB]    = BTN_RB_GP,
    [SNES_BTN_MENU]  = BTN_MENU_GP,
};

static uint16_t s_cur = 0;
static uint16_t s_prev = 0;

void snes_buttons_init(void) {
    for (int i = 0; i < SNES_BTN_COUNT; i++) {
        gpio_init(s_pins[i]);
        gpio_set_dir(s_pins[i], GPIO_IN);
        gpio_pull_up(s_pins[i]);
    }
    s_cur = s_prev = 0;
}

void snes_buttons_poll(void) {
    s_prev = s_cur;
    uint16_t b = 0;
    for (int i = 0; i < SNES_BTN_COUNT; i++) {
        if (!gpio_get(s_pins[i])) b |= (uint16_t)(1u << i);
    }
    s_cur = b;
}

int snes_buttons_is_pressed(int btn) {
    return (btn >= 0 && btn < SNES_BTN_COUNT) && (s_cur  & (1u << btn));
}

int snes_buttons_just_pressed(int btn) {
    return (btn >= 0 && btn < SNES_BTN_COUNT)
        && (s_cur & (1u << btn))
        && !(s_prev & (1u << btn));
}
