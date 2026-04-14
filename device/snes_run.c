/*
 * ThumbySNES — snes_run.c (Phase 4 device emulator driver, LakeSnes core).
 *
 * Picker hands us the chosen ROM. We:
 *   1. mmap it from flash XIP (zero copy),
 *   2. snes_load_xip — LakeSnes Cart->rom points at flash directly,
 *   3. install a per-scanline callback that downscales each line into
 *      the 128x128 LCD framebuffer (FILL mode 7:4 + 2x2 RGB565 blend),
 *   4. per frame: poll buttons, snes_run_frame (callback fires 224x),
 *      LCD present, repeat,
 *   5. exit on MENU.
 *
 * Audio is stubbed for Phase 4. Phase 5 wires the PWM path.
 */
#include "snes_run.h"
#include "snes_font.h"
#include "snes_lcd_gc9107.h"
#include "snes_buttons.h"
#include "snes_audio_pwm.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#if THUMBYSNES_HAVE_CORE
#include "snes_core.h"
#include "snes_picker.h"

#define FB_W 128
#define FB_H 128

/* FILL: 224 of the 256-pixel SNES line maps to 128 device columns,
 * sampling stride 7:4 with a 2x2 RGB565 blend. Vertical mapping is
 * symmetric — but it's done per-line using the callback's `line` arg.
 *
 * Note: the native-LCD fast path (snes_set_lcd_mode in snes_core) is
 * available but produces a nearest-neighbour 7:4 downscale which is
 * too aliased for text-heavy SNES content. We keep the 2x2 blend
 * callback as the default to preserve readability. */
#define FILL_X_CROP 16
#define FILL_SRC    224

static inline uint16_t rgb565_avg2(uint16_t a, uint16_t b) {
    uint32_t rsum = ((a >> 11) & 0x1F) + ((b >> 11) & 0x1F);
    uint32_t gsum = ((a >>  5) & 0x3F) + ((b >>  5) & 0x3F);
    uint32_t bsum = (a & 0x1F) + (b & 0x1F);
    return (uint16_t)(((rsum >> 1) << 11) | ((gsum >> 1) << 5) | (bsum >> 1));
}

static uint16_t *s_lcd_target = NULL;
static uint16_t  s_blend_a[FB_W];
static int       s_blend_a_dev_y = -1;

/* RGB565 scanline callback. Input is a 256-pixel fully composited
 * line from the PPU's per-line compositor (cgramRgb565 with brightness
 * baked in — no channel math here). We FILL-crop 16 px off each side,
 * 7:4-stride down to 128 px, horizontal-blend pairs, then vertical-
 * blend the pair of SNES lines that map to each device row. */
static void __attribute__((section(".time_critical.snes_blit")))
    on_scanline_565(void *user, int line, const uint16_t *line565)
{
    (void)user;
    if (line < 1 || line > 224) return;
    int sy = line - 1;
    int dev_y = (sy * 4) / 7;
    if (dev_y >= FB_H) return;

    uint16_t row_blended[FB_W];
    for (int ox = 0; ox < FB_W; ox++) {
        int sx  = ((ox * 7) >> 2) + FILL_X_CROP;
        int sx2 = sx + 1; if (sx2 > FILL_X_CROP + FILL_SRC - 1) sx2 = FILL_X_CROP + FILL_SRC - 1;
        row_blended[ox] = rgb565_avg2(line565[sx], line565[sx2]);
    }

    if (s_blend_a_dev_y == dev_y) {
        uint16_t *out = s_lcd_target + dev_y * FB_W;
        for (int x = 0; x < FB_W; x++) out[x] = rgb565_avg2(s_blend_a[x], row_blended[x]);
        s_blend_a_dev_y = -1;
    } else {
        if (s_blend_a_dev_y >= 0 && s_blend_a_dev_y < FB_H) {
            uint16_t *out = s_lcd_target + s_blend_a_dev_y * FB_W;
            memcpy(out, s_blend_a, sizeof(s_blend_a));
        }
        memcpy(s_blend_a, row_blended, sizeof(row_blended));
        s_blend_a_dev_y = dev_y;
    }
}

/* Map Thumby physical buttons to the SNES pad bitmask snes_core expects.
 * YX profile (see PLAN.md §7): A→SNES A, B→SNES B, LB→SNES Y, RB→SNES X.
 * MENU is *not* mapped here — it's handled separately by the run-loop:
 * long-press exits, short-press passes through as SNES Start.
 *
 * LB+RB chord is an alternate Start + exit, for units with a broken
 * MENU button. While the chord is active Y and X are suppressed so
 * the pad cleanly shows only Start (no spurious X/Y input to the
 * game during the chord press). */
static uint16_t snes_read_pad(bool send_start, bool lr_chord) {
    uint16_t bits = 0;
    if (snes_buttons_is_pressed(TBY_BTN_UP))    bits |= (1u << 11);
    if (snes_buttons_is_pressed(TBY_BTN_DOWN))  bits |= (1u << 10);
    if (snes_buttons_is_pressed(TBY_BTN_LEFT))  bits |= (1u <<  9);
    if (snes_buttons_is_pressed(TBY_BTN_RIGHT)) bits |= (1u <<  8);
    if (snes_buttons_is_pressed(TBY_BTN_A))     bits |= (1u <<  7);
    if (snes_buttons_is_pressed(TBY_BTN_B))     bits |= (1u << 15);
    if (!lr_chord) {
        if (snes_buttons_is_pressed(TBY_BTN_LB)) bits |= (1u << 14); /* Y */
        if (snes_buttons_is_pressed(TBY_BTN_RB)) bits |= (1u <<  6); /* X */
    }
    if (send_start)                              bits |= (1u << 12); /* Start */
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
    snes_font_draw(fb, "error",        45, 44, 0xF800);
    snes_font_draw(fb, msg,             2, 60, 0xCE59);
    snes_font_draw(fb, "MENU or L+R",  24, 96, 0xCE59);
    snes_font_draw(fb, "to return",    32,108, 0xCE59);
    snes_lcd_present(fb);
    for (;;) {
        snes_buttons_poll();
        if (snes_buttons_just_pressed(TBY_BTN_MENU)) break;
        if (snes_buttons_is_pressed(TBY_BTN_LB) &&
            snes_buttons_is_pressed(TBY_BTN_RB)) break;
        sleep_ms(16);
    }
}

int snes_run_rom(const snes_rom_entry *rom, uint16_t *fb) {
    draw_loading(fb, rom->name);

    /* Zero-copy XIP first — Cart->rom points at flash. Fallback to a
     * heap copy if the FAT cluster chain isn't contiguous. */
    const uint8_t *rom_data = NULL;
    size_t         rom_len  = 0;
    int xip_ok = snes_picker_mmap_rom(rom->name, &rom_data, &rom_len);
    snes_result_t r;
    uint8_t *heap_rom = NULL;
    if (xip_ok == 0) {
        r = snes_load_xip(rom_data, rom_len);
    } else {
        heap_rom = snes_picker_load_rom(rom->name, &rom_len);
        if (!heap_rom) { draw_error(fb, "open failed"); return -1; }
        r = snes_load(heap_rom, rom_len);
    }
    if (r != SNES_OK) {
        if (heap_rom) free(heap_rom);
        draw_error(fb, r == SNES_ERR_OOM ? "out of memory" : "bad ROM");
        return -2;
    }

    s_lcd_target = fb;
    s_blend_a_dev_y = -1;
    /* RGB565 fast-path callback — PPU composites whole lines via
     * cgramRgb565 with brightness baked in, so this path does zero
     * per-pixel channel math (just 2×2 blends + FILL 7:4 subsample). */
    snes_set_scanline_cb_rgb565(on_scanline_565, NULL);
    /* Performance: skip subscreen / colour-math fetch. Loses HUD fade
     * effects + transparency but cuts per-pixel work nearly in half. */
    snes_set_skip_color_math(1);
    /* Half-vertical: skip ~96 "blend partner" lines per frame. Keeps
     * horizontal 2-sample blend for text readability; loses vertical
     * smoothing. ~43% fewer PPU line renders. */
    snes_set_half_vertical(1);
    /* Audio: pull stereo samples from DSP (runs on core 1) and push
     * mono to the PWM ring buffer. dsp_getSamples resamples from
     * 32040 Hz to our requested count. We ask for 367 samples/frame
     * (22050 / 60) — at sub-60 fps emulation the audio plays slower
     * than real-time (pitch drops) but that's inherent to running
     * below full speed. */
    #define AUDIO_SAMPLES_PER_FRAME 367
    /* FILL mode crops 16 px off each horizontal edge. The per-line
     * compositor still renders all 256 columns (its inner loops are
     * cheaper per-x than the per-pixel walk was, so skipping the 32
     * cropped cols isn't worth losing the straight-line memory access
     * pattern). The scanline blend samples only x=16..239. */

    /* MENU button: long-press (>= 800 ms) exits to picker. A short
     * press is forwarded as SNES Start. We send Start for the whole
     * time MENU is held until the long-press threshold elapses, so
     * games that need Start to be held briefly (most title screens)
     * see it.
     *
     * LB+RB chord: alternate Start / exit for units with a broken
     * MENU button. Same thresholds: while both shoulders are held,
     * SNES Start is sent continuously (with Y/X suppressed so the
     * game sees a clean Start press); ≥ 800 ms of chord-hold exits
     * to the picker. */
    #define MENU_HOLD_EXIT_MS 800
    absolute_time_t menu_press_t  = nil_time;
    absolute_time_t chord_press_t = nil_time;
    bool menu_was_down  = false;
    bool chord_was_down = false;

    int exit_pressed = 0;
    int frame = 0;
    /* Tiny FPS counter — top-left corner. Updated once per second. */
    char fps_str[8] = {0};
    int  fps_frames = 0;
    absolute_time_t fps_window_start = get_absolute_time();
    while (!exit_pressed) {
        snes_buttons_poll();

        bool menu_now  = snes_buttons_is_pressed(TBY_BTN_MENU);
        bool chord_now = snes_buttons_is_pressed(TBY_BTN_LB)
                      && snes_buttons_is_pressed(TBY_BTN_RB);
        bool send_start = false;

        if (menu_now) {
            if (!menu_was_down) menu_press_t = get_absolute_time();
            int held_ms = (int)(absolute_time_diff_us(menu_press_t,
                                  get_absolute_time()) / 1000);
            if (held_ms >= MENU_HOLD_EXIT_MS) exit_pressed = 1;
            else                              send_start  = true;
        }
        menu_was_down = menu_now;

        if (chord_now) {
            if (!chord_was_down) chord_press_t = get_absolute_time();
            int held_ms = (int)(absolute_time_diff_us(chord_press_t,
                                  get_absolute_time()) / 1000);
            if (held_ms >= MENU_HOLD_EXIT_MS) exit_pressed = 1;
            else                              send_start  = true;
        }
        chord_was_down = chord_now;

        snes_set_pad(snes_read_pad(send_start, chord_now));
        s_blend_a_dev_y = -1;
        snes_run_frame();

        /* Audio disabled — at 7-20 fps the DSP produces 1/3 to 1/8 of
         * the samples the PWM consumes, resulting in badly stretched
         * audio that's worse than silence. The plumbing works (see
         * git history) and can be re-enabled when fps is higher. */

        if (s_blend_a_dev_y >= 0 && s_blend_a_dev_y < FB_H) {
            memcpy(s_lcd_target + s_blend_a_dev_y * FB_W, s_blend_a, sizeof(s_blend_a));
            s_blend_a_dev_y = -1;
        }
        /* FPS overlay in top-left — small, ~24×8 px so it covers as
         * little of the picture as possible. Refreshes once per second. */
        fps_frames++;
        absolute_time_t now = get_absolute_time();
        int64_t window_us = absolute_time_diff_us(fps_window_start, now);
        if (window_us >= 1000000) {
            int fps10 = (int)((int64_t)fps_frames * 10000000 / window_us);
            snprintf(fps_str, sizeof(fps_str), "%d.%d", fps10 / 10, fps10 % 10);
            fps_frames = 0;
            fps_window_start = now;
        }
        if (fps_str[0]) {
            for (int y = 0; y < 8; y++)
                for (int x = 0; x < 24; x++) fb[y * FB_W + x] = 0x0000;
            snes_font_draw(fb, fps_str, 1, 0, 0xFFE0);
        }

        snes_lcd_present(fb);
        frame++;
    }

    snes_unload();
    if (heap_rom) free(heap_rom);
    return 0;
}

int snes_run_clock_override(const char *rom_name) {
    (void)rom_name;
    return 0;
}

#else /* !THUMBYSNES_HAVE_CORE — Phase 3 stub */

#define FB_W 128
#define FB_H 128

int snes_run_rom(const snes_rom_entry *rom, uint16_t *fb) {
    memset(fb, 0, FB_W * FB_H * sizeof(uint16_t));
    snes_font_draw(fb, "loading...", 24, 44, 0xFFFF);
    snes_font_draw(fb, rom->name,     2, 60, 0xCE59);
    snes_font_draw(fb, "no core",    36, 80, 0x6B4D);
    snes_font_draw(fb, "MENU to return", 18, 100, 0xCE59);
    snes_lcd_present(fb);
    absolute_time_t deadline = make_timeout_time_ms(3000);
    while (!time_reached(deadline)) {
        snes_buttons_poll();
        if (snes_buttons_just_pressed(TBY_BTN_MENU)) break;
        sleep_ms(16);
    }
    return 0;
}

int snes_run_clock_override(const char *rom_name) {
    (void)rom_name;
    return 0;
}

#endif
