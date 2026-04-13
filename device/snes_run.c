/*
 * ThumbySNES — snes_run.c (Phase 4 device emulator driver).
 *
 * Picker hands us the chosen ROM. We:
 *   1. mmap it from flash XIP (zero copy),
 *   2. call snes_load_xip so snes9x2002 uses the XIP pointer directly
 *      (no 6 MB ROM buffer in SRAM),
 *   3. per frame: poll buttons, step one frame, 2x2 RGB565 downscale
 *      GFX.Screen (256x224 stride-640) into the 128x128 LCD framebuffer
 *      with FILL cropping (same math as sneshost), present.
 *   4. exit on MENU.
 *
 * Audio is stubbed for Phase 4. Phase 5 wires the PWM path to snes_get_audio.
 *
 * Falls back to snes_load (heap copy) if the ROM isn't contiguous in
 * flash — happens rarely but the picker doesn't defrag in Phase 3.
 */
#include "snes_run.h"
#include "snes_font.h"
#include "snes_lcd_gc9107.h"
#include "snes_buttons.h"

#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#if THUMBYSNES_HAVE_CORE
#include "snes_core.h"
#include "snes_picker.h"
/* Pull snes9x2002's GFX struct so we can sample the framebuffer after
 * each frame. GFX.Screen is a uint8* whose rows are GFX_PITCH (640 B)
 * bytes apart; each pixel is a uint16 RGB565. */
#include "gfx.h"

#define FB_W 128
#define FB_H 128
#define SNES_W 256
#define SNES_H 224

/* FILL mode — 224x224 crop from 256x224 (16 px each side) → 128x128
 * via 7:4 stride with 2x2 RGB565 blend. Identical to sneshost's
 * downscale_fill, just reading from snes9x2002's GFX.Screen with
 * its 640-byte row pitch. */
#define FILL_X_CROP 16
#define FILL_SRC    224

static inline uint16_t rgb565_avg2x2(uint16_t a, uint16_t b,
                                     uint16_t c, uint16_t d) {
    uint32_t rsum = ((a >> 11) & 0x1F) + ((b >> 11) & 0x1F)
                  + ((c >> 11) & 0x1F) + ((d >> 11) & 0x1F);
    uint32_t gsum = ((a >>  5) & 0x3F) + ((b >>  5) & 0x3F)
                  + ((c >>  5) & 0x3F) + ((d >>  5) & 0x3F);
    uint32_t bsum = (a & 0x1F) + (b & 0x1F) + (c & 0x1F) + (d & 0x1F);
    return (uint16_t)(((rsum >> 2) << 11) | ((gsum >> 2) << 5) | (bsum >> 2));
}

/* GFX.Screen row stride is 640 B = 320 uint16. We only sample the
 * first 256 pixels of each row (SNES visible width). */
#define GFX_ROW_PX 320

static void __attribute__((section(".time_critical.snes_blit")))
    blit_fill_to_lcd(uint16_t *lcd_fb) {
    const uint16_t *src0 = (const uint16_t *)GFX.Screen;
    for (int oy = 0; oy < FB_H; oy++) {
        int sy  = (oy * 7) >> 2;                  /* 0..223 */
        int sy2 = sy + 1; if (sy2 > FILL_SRC - 1) sy2 = FILL_SRC - 1;
        const uint16_t *r0 = src0 + sy  * GFX_ROW_PX + FILL_X_CROP;
        const uint16_t *r1 = src0 + sy2 * GFX_ROW_PX + FILL_X_CROP;
        uint16_t *out = lcd_fb + oy * FB_W;
        for (int ox = 0; ox < FB_W; ox++) {
            int sx  = (ox * 7) >> 2;              /* 0..223 */
            int sx2 = sx + 1; if (sx2 > FILL_SRC - 1) sx2 = FILL_SRC - 1;
            out[ox] = rgb565_avg2x2(r0[sx], r0[sx2], r1[sx], r1[sx2]);
        }
    }
}

/* Map Thumby physical buttons to the SNES pad bitmask snes_core expects.
 * YX profile (see PLAN.md §7): A→SNES A, B→SNES B, LB→SNES Y, RB→SNES X. */
static uint16_t snes_read_pad(void) {
    uint16_t p = 0;
    if (snes_buttons_is_pressed(SNES_BTN_UP))    p |= SNES_BTN_UP   | 0; /* see core pad bits */
    /* NB: SNES_BTN_* in our button enum conflict with the SNES pad
     * bitmask macros defined in snes_core.h. Use the core macros
     * explicitly here. */
    (void)p;
    extern int snes_buttons_is_pressed(int); /* silence */
    uint16_t bits = 0;
    #define CORE_UP     (1u << 11)
    #define CORE_DOWN   (1u << 10)
    #define CORE_LEFT   (1u <<  9)
    #define CORE_RIGHT  (1u <<  8)
    #define CORE_A      (1u <<  7)
    #define CORE_X      (1u <<  6)
    #define CORE_L      (1u <<  5)
    #define CORE_R      (1u <<  4)
    #define CORE_START  (1u << 12)
    #define CORE_SELECT (1u << 13)
    #define CORE_Y      (1u << 14)
    #define CORE_B      (1u << 15)
    if (snes_buttons_is_pressed(SNES_BTN_UP))    bits |= CORE_UP;
    if (snes_buttons_is_pressed(SNES_BTN_DOWN))  bits |= CORE_DOWN;
    if (snes_buttons_is_pressed(SNES_BTN_LEFT))  bits |= CORE_LEFT;
    if (snes_buttons_is_pressed(SNES_BTN_RIGHT)) bits |= CORE_RIGHT;
    if (snes_buttons_is_pressed(SNES_BTN_A))     bits |= CORE_A;
    if (snes_buttons_is_pressed(SNES_BTN_B))     bits |= CORE_B;
    if (snes_buttons_is_pressed(SNES_BTN_LB))    bits |= CORE_Y;
    if (snes_buttons_is_pressed(SNES_BTN_RB))    bits |= CORE_X;
    /* MENU short = SNES Start; MENU handled separately to also exit. */
    if (snes_buttons_is_pressed(SNES_BTN_MENU))  bits |= CORE_START;
    return bits;
}

static void draw_loading(uint16_t *fb, const char *rom_name) {
    memset(fb, 0, FB_W * FB_H * sizeof(uint16_t));
    snes_font_draw(fb, "loading...", 24, 44, 0xFFFF);
    char buf[24];
    size_t L = strlen(rom_name);
    if (L >= 22) { memcpy(buf, rom_name, 21); buf[21] = 0; }
    else         { memcpy(buf, rom_name, L + 1); }
    snes_font_draw(fb, buf, 2, 60, 0xCE59);
    snes_lcd_present(fb);
}

static void draw_error(uint16_t *fb, const char *msg) {
    memset(fb, 0, FB_W * FB_H * sizeof(uint16_t));
    snes_font_draw(fb, "error", 45, 44, 0xF800);
    snes_font_draw(fb, msg,    2, 60, 0xCE59);
    snes_font_draw(fb, "press MENU to return", 1, 100, 0xCE59);
    snes_lcd_present(fb);
    while (!snes_buttons_just_pressed(SNES_BTN_MENU)) {
        snes_buttons_poll();
        sleep_ms(16);
    }
}

/* Tiny progress splash — if the core hard-faults during init the last
 * message on-screen identifies where. Keep the messages short so the
 * 5×7 font fits inside a 128-pixel row. */
static void draw_progress(uint16_t *fb, const char *msg) {
    /* Clear only the bottom line — leaves the "loading: rom name" above. */
    for (int y = 76; y < 128; y++)
        for (int x = 0; x < 128; x++) fb[y * 128 + x] = 0;
    snes_font_draw(fb, msg, 2, 82, 0x7BEF);
    snes_lcd_present(fb);
}

int snes_run_rom(const snes_rom_entry *rom, uint16_t *fb) {
    draw_loading(fb, rom->name);

    /* Zero-copy XIP path first — snes9x2002 reads Memory.ROM from flash
     * directly. Falls back to a heap copy if the FAT cluster chain is
     * fragmented (snes_picker_mmap_rom returns non-zero). */
    draw_progress(fb, "mmap rom...");
    const uint8_t *rom_data = NULL;
    size_t         rom_len  = 0;
    int xip_ok = snes_picker_mmap_rom(rom->name, &rom_data, &rom_len);
    snes_result_t r;
    uint8_t *heap_rom = NULL;
    if (xip_ok == 0) {
        draw_progress(fb, "core init (xip)...");
        r = snes_load_xip(rom_data, rom_len);
    } else {
        draw_progress(fb, "slurping rom...");
        heap_rom = snes_picker_load_rom(rom->name, &rom_len);
        if (!heap_rom) { draw_error(fb, "open failed"); return -1; }
        draw_progress(fb, "core init (heap)...");
        r = snes_load(heap_rom, rom_len);
    }
    if (r != SNES_OK) {
        if (heap_rom) free(heap_rom);
        draw_error(fb, r == SNES_ERR_OOM ? "out of memory" : "bad ROM");
        return -2;
    }
    draw_progress(fb, "running...");

    /* Main emulator loop — no frame-rate pacing yet (device drives as
     * fast as it can; we'll add 60 Hz pacing in Phase 5 once we know
     * whether we're CPU-bound). */
    int exit_pressed = 0;
    while (!exit_pressed) {
        snes_buttons_poll();
        /* MENU long-press to exit — Phase 3 stub used short press. We
         * treat any MENU release as exit for now. */
        if (snes_buttons_just_pressed(SNES_BTN_MENU)) exit_pressed = 1;

        snes_set_pad(snes_read_pad());
        snes_run_frame();
        blit_fill_to_lcd(fb);
        snes_lcd_present(fb);
    }

    snes_unload();
    if (heap_rom) free(heap_rom);
    return 0;
}

int snes_run_clock_override(const char *rom_name) {
    (void)rom_name;
    return 0;
}

#else /* !THUMBYSNES_HAVE_CORE — Phase 3 stub (unchanged) */

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

#endif
