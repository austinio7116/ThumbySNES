/*
 * ThumbySNES — Thumby Color physical button reader.
 *
 * Exposes per-button held/just-pressed queries. Call snes_buttons_poll()
 * once per frame; it latches current + previous states for edge detection.
 */
#ifndef THUMBYSNES_BUTTONS_H
#define THUMBYSNES_BUTTONS_H

#include <stdint.h>

/* Logical button IDs — match the PLAN.md §7 mapping. */
enum {
    SNES_BTN_LEFT = 0,
    SNES_BTN_RIGHT,
    SNES_BTN_UP,
    SNES_BTN_DOWN,
    SNES_BTN_A,
    SNES_BTN_B,
    SNES_BTN_LB,
    SNES_BTN_RB,
    SNES_BTN_MENU,
    SNES_BTN_COUNT
};

void snes_buttons_init(void);
void snes_buttons_poll(void);
int  snes_buttons_is_pressed(int btn);
int  snes_buttons_just_pressed(int btn);

#endif
