/*
 * ThumbySNES — snes_core.c
 *
 * Thin wrapper around the vendored LakeSnes core. Same public API as
 * the previous snes9x2002-backed wrapper so device/snes_run.c and
 * src/sneshost / snesbench keep working unchanged.
 *
 * Two load paths:
 *   snes_load     — heap copy via LakeSnes's snes_loadRom (host).
 *   snes_load_xip — zero-copy: caller's pointer is held by Cart->rom
 *                    for the lifetime of the session. On device the
 *                    pointer points at XIP-mapped flash.
 *
 * Output:
 *   - The PPU ships per-scanline output via ppu_setScanlineCallback.
 *     Our callback assembles each line into a 256x224 RGB565
 *     framebuffer that snes_get_framebuffer hands to the host.
 *   - Device builds will install a different callback that downscales
 *     to 128x128 directly and writes to LCD framebuffer. See
 *     snes_set_scanline_cb.
 */
#include "snes_core.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#if THUMBYSNES_HAVE_CORE
#include "snes.h"
#include "ppu.h"
#include "cart.h"

extern uint8_t thumbysnes_cart_xip;

static Snes      *s_snes = NULL;
static int        s_loaded = 0;
static uint16_t   s_pad1 = 0;

/* Default scanline sink: assemble into a 256x224 RGB565 frame buffer
 * that the (host) frontend can read out via snes_get_framebuffer. The
 * device installs its own callback (downscale + LCD blit) and never
 * touches this buffer. */
static uint16_t   s_default_fb[256 * 224];

/* SNES button bit layout LakeSnes expects (from input.c). 0..11. */
enum {
    LK_BTN_R     = 4,
    LK_BTN_L     = 5,
    LK_BTN_X     = 6,
    LK_BTN_A     = 7,
    LK_BTN_RIGHT = 8,
    LK_BTN_LEFT  = 9,
    LK_BTN_DOWN  = 10,
    LK_BTN_UP    = 11,
    LK_BTN_START = 12,
    LK_BTN_SELECT= 13,
    LK_BTN_Y     = 14,
    LK_BTN_B     = 15,
};

/* Convert one BGR_BGR_ scanline (the format ppu_handlePixel writes —
 * 8 bytes per pixel for a future hires path; we read the first 3 of
 * each 8-byte slot) into 256 RGB565 pixels at row Y of the host fb. */
static void default_scanline_cb(void *user, int line, const uint8_t *src)
{
    (void)user;
    if (line < 1 || line > 224) return;
    uint16_t *dst = s_default_fb + (line - 1) * 256;
    /* pixelOutputFormat = BGRX (default): byte 0 = B, 1 = G, 2 = R. */
    for (int x = 0; x < 256; x++) {
        const uint8_t *p = src + x * 8;
        uint8_t b = p[0], g = p[1], r = p[2];
        dst[x] = ((uint16_t)(r >> 3) << 11)
               | ((uint16_t)(g >> 2) <<  5)
               |  (uint16_t)(b >> 3);
    }
}

static int snes_emu_init(void)
{
    if (s_snes) return 1;
    s_snes = snes_init();
    if (!s_snes) return 0;
    /* Force XBGR layout (B=byte0, G=byte1, R=byte2, X=byte3 within each
     * 4-byte half-pixel slot) so default_scanline_cb's p[0/1/2] reads
     * line up. ppu_init defaults to BGRX which shifts everything by 1. */
    ppu_setPixelOutputFormat(s_snes->ppu, ppu_pixelOutputFormatXBGR);
    ppu_setScanlineCallback(s_snes->ppu, default_scanline_cb, NULL);
    return 1;
}

static snes_result_t snes_load_common(const uint8_t *rom, size_t rom_len, int xip)
{
    if (!rom || rom_len < 0x8000) return SNES_ERR_BAD_ROM;
    thumbysnes_cart_xip = (uint8_t)(xip ? 1 : 0);
    if (!snes_emu_init()) return SNES_ERR_OOM;
    if (!snes_loadRom(s_snes, rom, (int)rom_len)) {
        return SNES_ERR_BAD_ROM;
    }
    s_loaded = 1;
    return SNES_OK;
}

snes_result_t snes_load(const uint8_t *rom, size_t rom_len)
{
    return snes_load_common(rom, rom_len, /*xip=*/0);
}

snes_result_t snes_load_xip(const uint8_t *rom, size_t rom_len)
{
    return snes_load_common(rom, rom_len, /*xip=*/1);
}

snes_result_t snes_run_frame(void)
{
    if (!s_loaded) return SNES_ERR_NO_ROM;
    snes_runFrame(s_snes);
    return SNES_OK;
}

void snes_unload(void)
{
    if (s_snes) {
        snes_free(s_snes);
        s_snes = NULL;
    }
    s_loaded = 0;
    thumbysnes_cart_xip = 0;
}

void snes_get_framebuffer(uint16_t *dst)
{
    if (!dst) return;
    memcpy(dst, s_default_fb, sizeof(s_default_fb));
}

size_t snes_get_audio(int16_t *dst, size_t frames)
{
    if (!dst || frames == 0) return 0;
    /* TODO Phase 5: route LakeSnes audio. snes_setSamples(snes, buf, n)
     * lets the core write samples into a buffer at end-of-frame, but
     * the timing is per-frame not per-host-buffer. For now, silence. */
    memset(dst, 0, frames * 2 * sizeof(int16_t));
    return frames;
}

void snes_set_pad(uint16_t pad_bits)
{
    if (s_pad1 == pad_bits) return;  /* avoid redundant set */
    /* Diff the bits and only push changes. snes_setButtonState takes
     * (player, button-index, pressed). */
    static const struct { uint16_t mask; int btn; } map[] = {
        { (1u << 11), LK_BTN_UP    },
        { (1u << 10), LK_BTN_DOWN  },
        { (1u <<  9), LK_BTN_LEFT  },
        { (1u <<  8), LK_BTN_RIGHT },
        { (1u <<  7), LK_BTN_A     },
        { (1u <<  6), LK_BTN_X     },
        { (1u <<  5), LK_BTN_L     },
        { (1u <<  4), LK_BTN_R     },
        { (1u << 12), LK_BTN_START },
        { (1u << 13), LK_BTN_SELECT},
        { (1u << 14), LK_BTN_Y     },
        { (1u << 15), LK_BTN_B     },
    };
    uint16_t changed = pad_bits ^ s_pad1;
    for (size_t i = 0; i < sizeof(map) / sizeof(*map); i++) {
        if (changed & map[i].mask) {
            snes_setButtonState(s_snes, 1, map[i].btn, (pad_bits & map[i].mask) != 0);
        }
    }
    s_pad1 = pad_bits;
}

/* Optional: install a custom per-scanline callback (device build uses
 * this for direct LCD blit + downscale). */
void snes_set_scanline_cb(void (*cb)(void*, int, const uint8_t*), void *user)
{
    if (s_snes) ppu_setScanlineCallback(s_snes->ppu, cb, user);
}

void snes_set_skip_color_math(int enable)
{
    if (s_snes) s_snes->ppu->skipColorMath = enable ? true : false;
}

void snes_set_render_x_range(int x_start, int x_end)
{
    if (!s_snes) return;
    if (x_start < 0)   x_start = 0;
    if (x_end   > 256) x_end   = 256;
    if (x_end <= x_start) { x_start = 0; x_end = 256; }
    s_snes->ppu->renderXStart = x_start;
    s_snes->ppu->renderXEnd   = x_end;
}

#include "cpu.h"
#include "spc.h"
#include "apu.h"

uint16_t snes_dbg_pc(void)         { return s_snes ? s_snes->cpu->pc : 0; }
uint8_t  snes_dbg_pb(void)         { return s_snes ? s_snes->cpu->k  : 0; }
uint16_t snes_dbg_a(void)          { return s_snes ? s_snes->cpu->a  : 0; }
uint8_t  snes_dbg_brightness(void) { return s_snes ? s_snes->ppu->brightness : 0; }
uint8_t  snes_dbg_forced_blank(void){ return (s_snes && s_snes->ppu->forcedBlank) ? 1 : 0; }
uint8_t  snes_dbg_wram(uint32_t addr) {
    return s_snes ? s_snes->ram[addr & 0x1FFFF] : 0;
}
uint8_t  snes_dbg_apu_out(int idx) {
    if (!s_snes || idx < 0 || idx > 3) return 0;
    return s_snes->apu->outPorts[idx];
}
uint32_t snes_dbg_apu_cycles(void) {
    return s_snes ? s_snes->apu->cycles : 0;
}
uint16_t snes_dbg_spc_pc(void) { return s_snes ? s_snes->apu->spc->pc : 0; }
uint8_t  snes_dbg_apu_in(int idx) {
    if (!s_snes || idx < 0 || idx > 3) return 0;
    return s_snes->apu->inPorts[idx];
}
uint8_t snes_dbg_rom_byte_lorom(uint32_t bank, uint16_t addr) {
    if (!s_snes || !s_snes->cart || !s_snes->cart->rom) return 0;
    uint32_t off = ((bank & 0x7f) << 15) | (addr & 0x7fff);
    off &= (s_snes->cart->romSize - 1);
    return s_snes->cart->rom[off];
}

#else /* !THUMBYSNES_HAVE_CORE */

snes_result_t snes_load(const uint8_t *rom, size_t rom_len)       { (void)rom; (void)rom_len; return SNES_ERR_NOT_IMPLEMENTED; }
snes_result_t snes_load_xip(const uint8_t *rom, size_t rom_len)   { (void)rom; (void)rom_len; return SNES_ERR_NOT_IMPLEMENTED; }
snes_result_t snes_run_frame(void)                                { return SNES_ERR_NOT_IMPLEMENTED; }
void          snes_unload(void)                                   { }
void          snes_get_framebuffer(uint16_t *dst)                 { if (dst) memset(dst, 0, 256 * 224 * 2); }
size_t        snes_get_audio(int16_t *dst, size_t frames)         { if (dst) memset(dst, 0, frames * 4); return frames; }
void          snes_set_pad(uint16_t pad_bits)                     { (void)pad_bits; }
void          snes_set_scanline_cb(void (*cb)(void*, int, const uint8_t*), void *user) { (void)cb; (void)user; }
void          snes_set_skip_color_math(int enable){ (void)enable; }
void          snes_set_render_x_range(int s, int e){ (void)s; (void)e; }
uint16_t      snes_dbg_pc(void)          { return 0; }
uint8_t       snes_dbg_pb(void)          { return 0; }
uint16_t      snes_dbg_a(void)           { return 0; }
uint8_t       snes_dbg_brightness(void)  { return 0; }
uint8_t       snes_dbg_forced_blank(void){ return 0; }
uint8_t       snes_dbg_wram(uint32_t addr){ (void)addr; return 0; }
uint8_t       snes_dbg_apu_out(int idx){ (void)idx; return 0; }
uint32_t      snes_dbg_apu_cycles(void){ return 0; }
uint8_t       snes_dbg_rom_byte_lorom(uint32_t b, uint16_t a){ (void)b; (void)a; return 0; }
uint16_t      snes_dbg_spc_pc(void){ return 0; }
uint8_t       snes_dbg_apu_in(int idx){ (void)idx; return 0; }

#endif
