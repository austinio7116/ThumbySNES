/*
 * ThumbySNES — snes_run.c (Phase 3 stub).
 *
 * Shows a "loading: <rom>" splash for a brief moment so the picker
 * can be exercised end-to-end: pick → stub runs → back to picker.
 * Phase 4 wires in snes_core + the per-scanline LCD render path.
 */
#include "snes_run.h"
#include "snes_font.h"
#include "snes_lcd_gc9107.h"
#include "snes_buttons.h"

#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"

#define FB_W 128
#define FB_H 128

int snes_run_rom(const snes_rom_entry *rom, uint16_t *fb) {
    memset(fb, 0, FB_W * FB_H * sizeof(uint16_t));
    snes_font_draw(fb, "loading...",    24, 44, 0xFFFF);
    char buf[24];
    size_t L = strlen(rom->name);
    if (L >= 22) { memcpy(buf, rom->name, 21); buf[21] = 0; }
    else         { memcpy(buf, rom->name, L + 1); }
    snes_font_draw(fb, buf, 2, 60, 0xCE59);
    snes_font_draw(fb, "Phase 3 stub — no core yet",  0, 80, 0x6B4D);
    snes_font_draw(fb, "MENU to return",             18, 100, 0xCE59);
    snes_lcd_present(fb);

    /* Wait for MENU press (or 3 s timeout). */
    absolute_time_t deadline = make_timeout_time_ms(3000);
    while (!time_reached(deadline)) {
        snes_buttons_poll();
        if (snes_buttons_just_pressed(SNES_BTN_MENU)) break;
        sleep_ms(16);
    }
    return 0;
}

int snes_run_clock_override(const char *rom_name) {
    (void)rom_name;
    return 0;
}
