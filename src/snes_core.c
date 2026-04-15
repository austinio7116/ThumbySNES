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

/* Dual-core APU: core 1 reads this pointer in a tight loop and runs
 * SPC opcodes when it's non-NULL. Updated by snes_load / snes_unload
 * on core 0. `volatile` because both cores touch it — byte pointers
 * are atomic on M33 but the compiler must not hoist the load out of
 * the core 1 loop. */
static Snes * volatile s_apu_core1_snes = NULL; /* unused on host */

/* PPU-CPU pipeline: core 0 dispatches ppu_runLine to core 1 via these
 * flags. Core 1 checks ppu_pipeline_line; if > 0, it runs ppu_runLine
 * for that line, clears it, and sets ppu_pipeline_done = 1. Core 0
 * reads ppu_pipeline_done to know if it's safe to dispatch the next
 * line (or falls back to core-0 render if core 1 is still busy).
 *
 * All fields are word-aligned volatile — single-word writes are atomic
 * on M33, and `volatile` prevents the compiler from caching reads
 * across the dispatch boundary. */
volatile int s_ppu_pipeline_line = 0;  /* line to render, 0 = none */
volatile int s_ppu_pipeline_done = 1;  /* 1 = core 1 idle */

/* Native-LCD mode state. `s_lcd_fb` holds the client's framebuffer so
 * we can re-install it after a frameskipped frame, and so snes_unload
 * can clear it cleanly. */
static uint16_t  *s_lcd_fb = NULL;
static int        s_lcd_w  = 0;
static int        s_lcd_h  = 0;
static uint8_t    s_lcd_src_x[128];
static int8_t     s_lcd_dev_y[239];
static int        s_frameskip = 0;      /* 0 = no skip, 1 = skip every other, … */
static int        s_frame_ctr  = 0;

/* Host-only default framebuffer — 256×224 RGB565 = 112 KB.
 * Device never uses this (installs its own scanline callback).
 * Gated on THUMBYSNES_DUAL_CORE to save 112 KB of BSS on device. */
#if !(defined(THUMBYSNES_DUAL_CORE) && THUMBYSNES_DUAL_CORE)
static uint16_t   s_default_fb[256 * 224];
#endif

/* SNES controller serial protocol button order.
 *
 * input_read() in LakeSnes shifts currentState right, sending bit 0
 * first. snes_doAutoJoypad maps bit 0 → portAutoRead[15] (B), bit 1 →
 * portAutoRead[14] (Y), etc. So the button index for snes_setButtonState
 * must match the SERIAL position, not the portAutoRead bit position.
 *
 * Previous code used portAutoRead bit positions (LK_BTN_A=7 etc.) which
 * caused every button to map to the wrong SNES button — e.g. Thumby A
 * registered as D-pad RIGHT. */
enum {
    LK_BTN_B      = 0,
    LK_BTN_Y      = 1,
    LK_BTN_SELECT = 2,
    LK_BTN_START  = 3,
    LK_BTN_UP     = 4,
    LK_BTN_DOWN   = 5,
    LK_BTN_LEFT   = 6,
    LK_BTN_RIGHT  = 7,
    LK_BTN_A      = 8,
    LK_BTN_X      = 9,
    LK_BTN_L      = 10,
    LK_BTN_R      = 11,
};

/* Host-only: 256×224 RGB565 framebuffer for snes_get_framebuffer.
 * On device this is NEVER used (device installs its own scanline
 * callback) — gated to save 112 KB of BSS. */
#if !(defined(THUMBYSNES_DUAL_CORE) && THUMBYSNES_DUAL_CORE)
static void default_scanline_cb_rgb565(void *user, int line, const uint16_t *src)
{
    (void)user;
    if (line < 1 || line > 224) return;
    memcpy(s_default_fb + (line - 1) * 256, src, 256 * sizeof(uint16_t));
}
#endif

static int snes_emu_init(void)
{
    if (s_snes) return 1;
    s_snes = snes_init();
    if (!s_snes) return 0;
    /* Install the RGB565 fast-path callback as the default. On device
     * the run loop overrides this with its own blend callback, so the
     * host-only default_scanline_cb_rgb565 is never called. */
#if !(defined(THUMBYSNES_DUAL_CORE) && THUMBYSNES_DUAL_CORE)
    ppu_setScanlineCbRgb565(s_snes->ppu, default_scanline_cb_rgb565, NULL);
#endif
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
    /* Release the shared snes pointer to core 1. From this point core 1
     * will begin executing spc_runOpcode in a tight loop. */
    s_apu_core1_snes = s_snes;
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
    /* Frameskip only applies when we're in LCD mode — classic path still
     * relies on the scanline callback firing every frame for the host
     * framebuffer to stay up to date. */
    int skip_this = 0;
    if (s_lcd_fb && s_frameskip > 0) {
        if ((s_frame_ctr % (s_frameskip + 1)) != 0) skip_this = 1;
        s_frame_ctr++;
    }
    s_snes->ppu->skipRender = (uint8_t)(skip_this ? 1 : 0);
    snes_runFrame(s_snes);
    s_snes->ppu->skipRender = 0;
    /* Drain PPU pipeline — wait for core 1 to finish the last line
     * before we return (the caller will LCD-present the framebuffer
     * which must be fully composited). */
    while (!s_ppu_pipeline_done) { /* spin — at most one line's render */ }
    return SNES_OK;
}

void snes_unload(void)
{
    /* Drain PPU pipeline + park core 1 before freeing snes. */
    while (!s_ppu_pipeline_done) {}
    s_ppu_pipeline_line = 0;
    s_apu_core1_snes = NULL;
    if (s_snes) {
        snes_free(s_snes);
        s_snes = NULL;
    }
    s_loaded = 0;
    thumbysnes_cart_xip = 0;
    s_lcd_fb = NULL;
    s_lcd_w = s_lcd_h = 0;
    s_frameskip = 0;
    s_frame_ctr = 0;
}

void snes_get_framebuffer(uint16_t *dst)
{
    if (!dst) return;
#if !(defined(THUMBYSNES_DUAL_CORE) && THUMBYSNES_DUAL_CORE)
    memcpy(dst, s_default_fb, sizeof(s_default_fb));
#else
    memset(dst, 0, 256 * 224 * sizeof(uint16_t));
#endif
}

size_t snes_get_audio(int16_t *dst, size_t frames)
{
    if (!dst || frames == 0) return 0;
    if (!s_snes) {
        memset(dst, 0, frames * 2 * sizeof(int16_t));
        return frames;
    }
    /* Pull stereo samples from DSP's ring buffer. snes_setSamples
     * resamples from 32040 Hz to `frames` samples per call. On
     * dual-core builds the DSP runs on core 1 continuously, so the
     * sample buffer has fresh data. Race on sampleBuffer/sampleOffset
     * is benign — worst case is one slightly stale sample. */
    snes_setSamples(s_snes, dst, (int)frames);
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

void snes_set_scanline_cb_rgb565(void (*cb)(void*, int, const uint16_t*), void *user)
{
    if (s_snes) ppu_setScanlineCbRgb565(s_snes->ppu, cb, user);
}

/* Dual-core APU entry point. Core 1 calls this once at boot on the
 * device build; on host it's a no-op (host keeps the classic
 * snes_catchupApu path). Runs SPC opcodes at the M33's natural rate —
 * that's roughly real-time for the SPC's 1.024 MHz — while core 0
 * handles CPU + PPU + LCD. CPU-side snes_catchupApu is compiled to a
 * no-op on device so the bus hot path loses the per-access SPC
 * catchup overhead. */
void snes_apu_core1_loop(void)
{
#if defined(THUMBYSNES_DUAL_CORE) && THUMBYSNES_DUAL_CORE
    for (;;) {
        Snes *s = s_apu_core1_snes;
        if (!s) {
            __asm__ volatile ("nop");
            continue;
        }
        /* PPU-CPU pipeline: if core 0 dispatched a PPU line, render
         * it here. This is the pipeline overlap — core 0 is running
         * CPU for the next line while we render this one. */
        int line = s_ppu_pipeline_line;
        if (line > 0) {
            ppu_runLine(s->ppu, line);
            s_ppu_pipeline_line = 0;
            s_ppu_pipeline_done = 1;
            /* __dmb() not needed — volatile writes are ordered on M33
             * and ppu_runLine's last store (scanline callback writing
             * to the LCD framebuffer) is sequenced before these flag
             * writes by program order. */
        }
        /* SPC opcode: run one between PPU dispatches. Core 1 time-
         * shares between PPU and SPC this way — SPC gets ~60-80% of
         * core 1's budget (all the time between PPU renders). */
        spc_runOpcode(s->apu->spc);
    }
#else
    return;
#endif
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

/* Precompute FILL-mode source-x / device-row tables. The device does
 * 7:4 stride from 224 source pixels (cropped 16 each side) to 128
 * device pixels; to render at native resolution we pick one source
 * sample per device pixel instead of averaging pairs. Near-identical
 * visual result on-device at a fraction of the cost. */
static void snes_lcd_build_fill_tables(void)
{
    /* Horizontal: device col ox ∈ [0, 128). Source x = ox*7/4 + 16,
     * giving a 128-long list of SNES x's (16..239). */
    for (int ox = 0; ox < 128; ox++) {
        int sx = ((ox * 7) >> 2) + 16;
        if (sx > 239) sx = 239;
        s_lcd_src_x[ox] = (uint8_t)sx;
    }
    /* Vertical: for each SNES line sy ∈ [0, 224), device row dy =
     * sy*4/7. Multiple sys map to the same dy; we mark only the
     * first one (-1 for the rest, which skips pixel compositing
     * on that line). */
    int prev_dy = -1;
    for (int sy = 0; sy < 239; sy++) {
        if (sy < 224) {
            int dy = (sy * 4) / 7;
            s_lcd_dev_y[sy] = (dy != prev_dy) ? (int8_t)dy : (int8_t)-1;
            prev_dy = dy;
        } else {
            s_lcd_dev_y[sy] = -1;
        }
    }
}

void snes_set_lcd_mode(uint16_t *fb, int w, int h)
{
    s_lcd_fb = fb;
    s_lcd_w  = w;
    s_lcd_h  = h;
    if (!s_snes) return;
    if (!fb) {
        ppu_setLcdMode(s_snes->ppu, NULL, 0, 0, NULL, NULL);
        return;
    }
    snes_lcd_build_fill_tables();
    ppu_setLcdMode(s_snes->ppu, fb, w, h, s_lcd_src_x, s_lcd_dev_y);
}

void snes_set_frameskip(int skip)
{
    if (skip < 0) skip = 0;
    if (skip > 8) skip = 8;
    s_frameskip = skip;
    s_frame_ctr = 0;
}

void snes_set_half_vertical(int enable)
{
    if (s_snes) s_snes->ppu->halfVertical = enable ? 1 : 0;
}

#include "cpu.h"
#include "spc.h"
#include "apu.h"

uint16_t snes_dbg_pc(void)         { return s_snes ? s_snes->cpu->pc : 0; }
uint8_t  snes_dbg_pb(void)         { return s_snes ? s_snes->cpu->k  : 0; }
uint16_t snes_dbg_a(void)          { return s_snes ? s_snes->cpu->a  : 0; }
uint8_t  snes_dbg_last_op(void)    { return s_snes ? s_snes->cpu->lastOpcode : 0; }
uint8_t  snes_dbg_last_pb(void)    { return s_snes ? s_snes->cpu->lastPb     : 0; }
uint16_t snes_dbg_last_pc(void)    { return s_snes ? s_snes->cpu->lastPc     : 0; }
uint32_t snes_dbg_op_count(void)   { return s_snes ? s_snes->cpu->opcodeCount: 0; }
uint32_t snes_dbg_frames(void)     { return s_snes ? s_snes->frames           : 0; }
uint16_t snes_dbg_vpos(void)       { return s_snes ? s_snes->vPos             : 0; }
uint8_t  snes_dbg_in_vblank(void)  { return (s_snes && s_snes->inVblank) ? 1 : 0; }
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
void          snes_set_scanline_cb_rgb565(void (*cb)(void*, int, const uint16_t*), void *user) { (void)cb; (void)user; }
void          snes_apu_core1_loop(void) { }
void          snes_set_skip_color_math(int enable){ (void)enable; }
void          snes_set_render_x_range(int s, int e){ (void)s; (void)e; }
void          snes_set_lcd_mode(uint16_t *fb, int w, int h){ (void)fb; (void)w; (void)h; }
void          snes_set_frameskip(int skip){ (void)skip; }
void          snes_set_half_vertical(int enable){ (void)enable; }
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
uint8_t       snes_dbg_last_op(void) { return 0; }
uint8_t       snes_dbg_last_pb(void) { return 0; }
uint16_t      snes_dbg_last_pc(void) { return 0; }
uint32_t      snes_dbg_op_count(void){ return 0; }
uint32_t      snes_dbg_frames(void)  { return 0; }
uint16_t      snes_dbg_vpos(void)    { return 0; }
uint8_t       snes_dbg_in_vblank(void){ return 0; }

#endif
