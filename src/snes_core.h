/*
 * ThumbySNES — thin wrapper around the vendored snes9x2002 core.
 *
 * Mirrors the shape of ThumbyNES's nes_core.[ch] / sms_core.[ch] / gb_core.[ch]
 * so the device-side driver in device/snes_run.c can look the same as the
 * other emulator drivers.
 *
 * Phase 0 status: all functions are stubs returning SNES_ERR_NOT_IMPLEMENTED
 * until vendor/snes9x2002/ is populated. See PLAN.md §10.
 */
#ifndef THUMBYSNES_SNES_CORE_H
#define THUMBYSNES_SNES_CORE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SNES_OK                   = 0,
    SNES_ERR_NO_ROM           = -1,
    SNES_ERR_BAD_ROM          = -2,
    SNES_ERR_UNSUPPORTED_CHIP = -3,  /* SuperFX / SA-1 / DSP-n / etc. */
    SNES_ERR_OOM              = -4,
    SNES_ERR_NOT_IMPLEMENTED  = -99,
} snes_result_t;

/*
 * Load a ROM image from a contiguous memory region. The buffer MUST remain
 * live for the lifetime of the emulator session. This path allocates a 6 MB
 * internal ROM buffer inside snes9x2002 and memcpy's the ROM into it —
 * fine on host; don't use on device.
 */
snes_result_t snes_load(const uint8_t *rom, size_t rom_len);

/*
 * XIP / zero-copy load: the core's Memory.ROM pointer is set directly to
 * `rom` (no 6 MB malloc, no copy). Caller MUST guarantee:
 *   - `rom` is readable for at least `rom_len` bytes, and remains valid
 *     for the lifetime of the emulator session (typically XIP flash on
 *     the Thumby Color).
 *   - The ROM has no 512-byte header (*.sfc or headerless *.smc).
 *   - The ROM is not one of the special cases that triggers a ROM-rewrite
 *     inside LoadROM (DAIKAIJYUMONOGATARI2, WWF SUPER WRESTLEMANIA,
 *     interleaved ROMs like Tales/Star Ocean, ExHiROM > 4 MB). These will
 *     fault when the core tries to memmove into read-only flash.
 */
snes_result_t snes_load_xip(const uint8_t *rom, size_t rom_len);

/* Run one SNES frame (~4.17M master cycles). */
snes_result_t snes_run_frame(void);

/* Release all state. Safe to call without a prior load. */
void snes_unload(void);

/* Performance toggle — when on, the PPU skips colour math (subscreen
 * fetch + add/sub blending). Loses SNES transparency / fade effects
 * but cuts per-pixel work nearly in half. Off by default. */
void snes_set_skip_color_math(int enable);

/* Per-line PPU render window. SNES renders 256 pixels per scanline
 * by default; FILL display mode crops to 224 (16 px each side), so
 * setting (16, 240) skips ~12.5% of the per-pixel work. Default is
 * the full SNES width (0, 256). */
void snes_set_render_x_range(int x_start, int x_end);

/* Native-LCD fast path. When enabled, the PPU composites RGB565 directly
 * into the caller's 128×128 framebuffer at device resolution — no BGRX
 * intermediate, no 224-wide line buffer, and lines that don't map to a
 * device row are skipped entirely for pixel compositing. Cuts the per-
 * pixel cost ~3× versus the scanline-callback path.
 *
 * Call with `fb = NULL` to restore classic full-width rendering (used by
 * the SDL host for the "FULL" debug mode).
 *
 * The caller is responsible for keeping `fb` live for the lifetime of
 * the emulator session (or until another snes_set_lcd_mode call). */
void snes_set_lcd_mode(uint16_t *fb, int w, int h);

/* Render 1 in every (skip+1) frames. CPU + APU still advance normally
 * on every frame (game state stays correct); the PPU's per-pixel
 * compositing is suppressed on the other frames. `skip=0` = render
 * every frame (default); `skip=1` = render every other frame; etc.
 *
 * Only valid in LCD mode (snes_set_lcd_mode) — classic path ignores. */
void snes_set_frameskip(int skip);

/* Install a per-scanline output callback (LakeSnes core hooks this
 * via ppu_setScanlineCallback). `cb(user, line, line_buffer)` fires
 * after each visible scanline; `line_buffer` is 256 px × 8 B in BGRX
 * format. The default callback assembles a 256x224 RGB565 frame for
 * snes_get_framebuffer; install your own for direct LCD-blit paths. */
void snes_set_scanline_cb(void (*cb)(void *user, int line, const uint8_t *line_buffer),
                          void *user);

/* Diagnostic peeks into the live core. Safe to call any time after
 * snes_load*. Returns 0 if no core loaded. */
uint16_t snes_dbg_pc(void);          /* CPU program counter (16-bit, in current PB) */
uint8_t  snes_dbg_pb(void);          /* CPU program bank */
uint16_t snes_dbg_a(void);
uint8_t  snes_dbg_brightness(void);  /* PPU brightness $2100 low nibble (0..15) */
uint8_t  snes_dbg_forced_blank(void);/* 1 if PPU is in forced blank */
uint8_t  snes_dbg_wram(uint32_t addr); /* WRAM byte at addr (& 0x1FFFF) */

/* APU outPorts[0..3] — the SPC700 → CPU mailbox bytes the game reads
 * at $2140-$2143. After IPL boot they should be 0xAA, 0xBB, 0x??, 0x??.
 * If they stay 0, SPC isn't running. */
uint8_t  snes_dbg_apu_out(int idx);

/* APU cycles run since reset — proves SPC is actually executing. */
uint32_t snes_dbg_apu_cycles(void);

/* Peek into the cart ROM buffer as the LoROM mapper sees it.
 * For PC=00:8090 this returns the byte at ROM offset 0x90. */
uint8_t  snes_dbg_rom_byte_lorom(uint32_t bank, uint16_t addr);

/* SPC700 program counter + in-port byte 0. Tells us whether the
 * SPC is actually executing and what value CPU last sent it. */
uint16_t snes_dbg_spc_pc(void);
uint8_t  snes_dbg_apu_in(int idx);

/*
 * Copy the current 256x224 PPU output as RGB565 into `dst`.
 * `dst` must hold 256*224 uint16_t = 112 KiB. On the device we skip this
 * and render downscaled straight into the LCD framebuffer from the PPU
 * line hook — see PLAN.md §4.1 and §5.3.
 */
void snes_get_framebuffer(uint16_t *dst);

/*
 * Pull N stereo 16-bit samples at 22050 Hz from the DSP mixer into `dst`.
 * Returns number of samples actually written. `dst` length is in samples
 * (L+R pairs), not bytes.
 */
size_t snes_get_audio(int16_t *dst, size_t frames);

/*
 * Controller port 1 bitfield. Bit layout matches the SNES bus order used
 * by snes9x2002 so the input path is a single mov on the critical path.
 */
#define SNES_BTN_R       (1u << 4)
#define SNES_BTN_L       (1u << 5)
#define SNES_BTN_X       (1u << 6)
#define SNES_BTN_A       (1u << 7)
#define SNES_BTN_RIGHT   (1u << 8)
#define SNES_BTN_LEFT    (1u << 9)
#define SNES_BTN_DOWN    (1u << 10)
#define SNES_BTN_UP      (1u << 11)
#define SNES_BTN_START   (1u << 12)
#define SNES_BTN_SELECT  (1u << 13)
#define SNES_BTN_Y       (1u << 14)
#define SNES_BTN_B       (1u << 15)

void snes_set_pad(uint16_t pad_bits);

#ifdef __cplusplus
}
#endif

#endif /* THUMBYSNES_SNES_CORE_H */
