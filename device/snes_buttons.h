/*
 * ThumbySNES — Thumby Color physical button reader.
 *
 * Exposes per-button held/just-pressed queries. Call snes_buttons_poll()
 * once per frame; it latches current + previous states for edge detection.
 */
#ifndef THUMBYSNES_BUTTONS_H
#define THUMBYSNES_BUTTONS_H

#include <stdint.h>

/* Physical Thumby-Color button IDs (indices into a GPIO table).
 *
 * IMPORTANT: these were previously named SNES_BTN_*, which silently
 * collided with the SNES pad bitmasks defined as preprocessor macros
 * in src/snes_core.h — the macros won, so every call like
 * `snes_buttons_is_pressed(SNES_BTN_A)` passed a value like 128
 * instead of 4, hit the out-of-range guard, and silently returned 0.
 * That made A/B/X/Y/L/R + D-pad all appear dead whenever snes_core.h
 * was included in the same TU. Renamed to TBY_BTN_* to make the
 * physical-button namespace unambiguous. */
enum {
    TBY_BTN_LEFT = 0,
    TBY_BTN_RIGHT,
    TBY_BTN_UP,
    TBY_BTN_DOWN,
    TBY_BTN_A,
    TBY_BTN_B,
    TBY_BTN_LB,
    TBY_BTN_RB,
    TBY_BTN_MENU,
    TBY_BTN_COUNT
};

void snes_buttons_init(void);
void snes_buttons_poll(void);
int  snes_buttons_is_pressed(int btn);
int  snes_buttons_just_pressed(int btn);

#endif
