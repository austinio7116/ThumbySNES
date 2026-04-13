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

/* Software reset — keeps loaded ROM. */
void snes_reset(void);

/* Release all state. Safe to call without a prior load. */
void snes_unload(void);

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
