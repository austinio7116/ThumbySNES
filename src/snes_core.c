/*
 * ThumbySNES — snes_core.c
 *
 * Thin wrapper around snes9x2002. Mirrors the init sequence in
 * vendor/snes9x2002/libretro/libretro.c (snes_init / retro_load_game /
 * retro_run / retro_deinit) without dragging in the libretro frontend.
 *
 * Host build: GFX buffers live in heap (4 MB + 2 MB zbuffers — fine on
 * desktop). Device build: Phase 2 audit will shrink these drastically,
 * probably by streaming scanlines out of GFX.Screen into the 128x128
 * LCD framebuffer on the fly.
 */
#include "snes_core.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#if THUMBYSNES_HAVE_CORE

#include "snes9x.h"
#include "memmap.h"
#include "cpuexec.h"
#include "apu.h"
#include "ppu.h"
#include "soundux.h"
#include "gfx.h"
#include "display.h"
#include <streams/memory_stream.h>

#define AUDIO_SAMPLE_RATE 32040

static uint8_t *s_gfx_screen_buffer = NULL;
static uint8_t *s_gfx_zbuffer_buffer = NULL;
static uint8_t *s_gfx_subzbuffer_buffer = NULL;

static uint16_t s_pad1 = 0;
static int s_loaded = 0;
static int s_inited = 0;

/* Overclock knobs declared in snes9x.h — normally provided by libretro.c.
 * We keep them at defaults (disabled), which makes ONE_CYCLE/SLOW_ONE_CYCLE/
 * TWO_CYCLES fall through to the stock 6 / 8 / 12 master-cycle constants. */
bool  overclock_cycles = false;
int   one_c = 6, slow_one_c = 8, two_c = 12;
bool8 ROMAPUEnabled = 0;

static void snes_init_settings(void)
{
    memset(&Settings, 0, sizeof(Settings));
    Settings.JoystickEnabled     = FALSE;
    Settings.SoundPlaybackRate   = AUDIO_SAMPLE_RATE;
    Settings.Stereo              = TRUE;
    Settings.SoundBufferSize     = 0;
    Settings.CyclesPercentage    = 100;
    Settings.DisableSoundEcho    = FALSE;
    Settings.APUEnabled          = FALSE;
    Settings.H_Max               = SNES_CYCLES_PER_SCANLINE;
    Settings.SkipFrames          = AUTO_FRAMERATE;
    Settings.Shutdown            = TRUE;
    Settings.ShutdownMaster      = TRUE;
    Settings.FrameTimePAL        = 20000;
    Settings.FrameTimeNTSC       = 16667;
    Settings.FrameTime           = Settings.FrameTimeNTSC;
    Settings.DisableSampleCaching = FALSE;
    Settings.DisableMasterVolume = FALSE;
    Settings.Mouse               = FALSE;
    Settings.SuperScope          = FALSE;
    Settings.MultiPlayer5        = FALSE;
    Settings.ControllerOption    = 0;

    Settings.ForceTransparency   = FALSE;
    Settings.Transparency        = TRUE;
    Settings.SixteenBit          = TRUE;

    Settings.SupportHiRes        = FALSE;
    Settings.AutoSaveDelay       = 30;
    Settings.ApplyCheats         = TRUE;
    Settings.TurboMode           = FALSE;
    Settings.TurboSkipFrames     = 15;
    Settings.SoundSync           = FALSE;
    Settings.SpeedHacks          = TRUE;

    Settings.HBlankStart         = (256 * Settings.H_Max) / SNES_HCOUNTER_MAX;
    Settings.InterpolatedSound   = TRUE;

    CPU.Flags = 0;
}

static void snes_init_gfx_buffers(void)
{
    const int safety = 128;
    GFX.Pitch = 2048;

    s_gfx_screen_buffer = (uint8_t *)calloc(1, 2048 * 512 * 2 * 2 + safety);
    GFX.Screen_buffer = s_gfx_screen_buffer;
    GFX.Screen = s_gfx_screen_buffer + safety;
    GFX.SubScreen = GFX.Screen + 2048 * 512 * 2;

    s_gfx_zbuffer_buffer = (uint8_t *)calloc(1, GFX.Pitch * 512 * sizeof(uint16) + safety);
    GFX.ZBuffer_buffer = s_gfx_zbuffer_buffer;
    GFX.ZBuffer = s_gfx_zbuffer_buffer + safety;

    s_gfx_subzbuffer_buffer = (uint8_t *)calloc(1, GFX.Pitch * 512 * sizeof(uint16) + safety);
    GFX.SubZBuffer_buffer = s_gfx_subzbuffer_buffer;
    GFX.SubZBuffer = s_gfx_subzbuffer_buffer + safety;

    GFX.Delta = 1048576; /* (SubScreen - Screen) >> 1 */
}

static int snes_emu_init(void)
{
    if (s_inited) return 1;

    snes_init_settings();

    if (!MemoryInit() || !S9xInitAPU()) {
        MemoryDeinit();
        S9xDeinitAPU();
        return 0;
    }
    if (!S9xInitSound() || !S9xGraphicsInit()) {
        return 0;
    }
    snes_init_gfx_buffers();
    s_inited = 1;
    return 1;
}

snes_result_t snes_load(const uint8_t *rom, size_t rom_len)
{
    if (!rom || rom_len < 0x8000) {
        return SNES_ERR_BAD_ROM;
    }
    if (!snes_emu_init()) {
        return SNES_ERR_OOM;
    }

    /* Core's memstream-backed LoadROM() reads from this buffer instead of fopen. */
    memstream_set_buffer((uint8_t *)rom, rom_len);

    if (!LoadROM()) {
        return SNES_ERR_BAD_ROM;
    }

    S9xReset();
    CPU.APU_APUExecuting = Settings.APUEnabled = 1;
    Settings.SixteenBitSound = TRUE;
    so.stereo = Settings.Stereo;
    S9xSetPlaybackRate(Settings.SoundPlaybackRate);
    S9xSetSoundMute(FALSE);

    s_loaded = 1;
    return SNES_OK;
}

snes_result_t snes_run_frame(void)
{
    if (!s_loaded) return SNES_ERR_NO_ROM;
    IPPU.RenderThisFrame = TRUE;
    S9xMainLoop();
    return SNES_OK;
}

void snes_reset(void)
{
    if (s_loaded) S9xReset();
}

void snes_unload(void)
{
    if (s_inited) {
        S9xDeinitAPU();
        MemoryDeinit();
        S9xGraphicsDeinit();
    }
    free(s_gfx_screen_buffer);     s_gfx_screen_buffer = NULL;
    free(s_gfx_zbuffer_buffer);    s_gfx_zbuffer_buffer = NULL;
    free(s_gfx_subzbuffer_buffer); s_gfx_subzbuffer_buffer = NULL;
    GFX.Screen_buffer = GFX.Screen = GFX.SubScreen = NULL;
    GFX.ZBuffer_buffer = GFX.ZBuffer = NULL;
    GFX.SubZBuffer_buffer = GFX.SubZBuffer = NULL;
    s_inited = 0;
    s_loaded = 0;
}

void snes_get_framebuffer(uint16_t *dst)
{
    if (!dst) return;
    if (!s_loaded) { memset(dst, 0, 256 * 224 * 2); return; }
    int h = (IPPU.RenderedScreenHeight > 0) ? IPPU.RenderedScreenHeight : 224;
    int w = (IPPU.RenderedScreenWidth  > 0) ? IPPU.RenderedScreenWidth  : 256;
    if (h > 224) h = 224;
    if (w > 256) w = 256;
    /* Old raster path writes GFX.Screen with stride GFX_PITCH (= 640 B = 320 px). */
    for (int y = 0; y < 224; y++) {
        if (y < h) {
            const uint16_t *src = (const uint16_t *)(GFX.Screen + y * GFX_PITCH);
            memcpy(dst + y * 256, src, w * 2);
            if (w < 256) memset(dst + y * 256 + w, 0, (256 - w) * 2);
        } else {
            memset(dst + y * 256, 0, 256 * 2);
        }
    }
}

size_t snes_get_audio(int16_t *dst, size_t frames)
{
    if (!dst || frames == 0) return 0;
    if (!s_loaded) { memset(dst, 0, frames * 4); return frames; }
    /* S9xMixSamples' internal MixBuffer/EchoBuffer are sized SOUND_BUFFER_SIZE
     * (= 2*44100/50 = 1764 samples). Chunk so each call stays under that —
     * otherwise a single large request overflows the static buffers and
     * glibc's FORTIFY_SOURCE aborts us on the SDL audio thread. */
    const size_t MAX_SAMPLES_PER_CALL = 1764;
    size_t remaining = frames * 2; /* total samples: L+R pairs */
    int16_t *out = dst;
    while (remaining > 0) {
        size_t chunk = remaining > MAX_SAMPLES_PER_CALL
                     ? (MAX_SAMPLES_PER_CALL & ~1u) /* keep stereo pair-aligned */
                     : remaining;
        S9xMixSamples(out, (int)chunk);
        out       += chunk;
        remaining -= chunk;
    }
    return frames;
}

void snes_set_pad(uint16_t pad_bits)
{
    s_pad1 = pad_bits;
}

/* ------------------------------------------------------------------
 * Host-glue stubs the core expects to be provided by the frontend.
 * Mirrors the "Dummy functions" block at the bottom of libretro.c.
 * ------------------------------------------------------------------ */
uint32 S9xReadJoypad(int which1)
{
    if (which1 == 0) return 0x80000000u | (uint32)s_pad1;
    return 0;
}

bool8_32 S9xDeinitUpdate(int width, int height)
{
    /* Core finished a frame's worth of rendering into GFX.Screen.
     * Nothing to do here — the host pulls the framebuffer via
     * snes_get_framebuffer() after snes_run_frame() returns. */
    (void)width; (void)height;
    return TRUE;
}

bool8_32 S9xContinueUpdate(int width, int height) { (void)width; (void)height; return TRUE; }
void     S9xLoadSDD1Data(void) { }
bool8_32 S9xReadMousePosition(int which1, int *x, int *y, uint32 *buttons) { (void)which1; (void)x; (void)y; (void)buttons; return FALSE; }
bool8_32 S9xReadSuperScopePosition(int *x, int *y, uint32 *buttons) { (void)x; (void)y; (void)buttons; return FALSE; }
void     S9xToggleSoundChannel(int channel) { (void)channel; }
void     S9xSyncSpeed(void) { /* No throttling — host runs as fast as it can. */ }
void     S9xExit(void) { /* Core asked us to bail. Swallow — caller drives the lifecycle. */ }
void     S9xMessage(int type, int number, const char *message)
{
    (void)type; (void)number;
    if (message) fprintf(stderr, "snes9x: %s\n", message);
}

#else /* !THUMBYSNES_HAVE_CORE */

snes_result_t snes_load(const uint8_t *rom, size_t rom_len)       { (void)rom; (void)rom_len; return SNES_ERR_NOT_IMPLEMENTED; }
snes_result_t snes_run_frame(void)                                { return SNES_ERR_NOT_IMPLEMENTED; }
void          snes_reset(void)                                    { }
void          snes_unload(void)                                   { }
void          snes_get_framebuffer(uint16_t *dst)                 { if (dst) memset(dst, 0, 256 * 224 * 2); }
size_t        snes_get_audio(int16_t *dst, size_t frames)         { if (dst) memset(dst, 0, frames * 4); return frames; }
void          snes_set_pad(uint16_t pad_bits)                     { (void)pad_bits; }

#endif /* THUMBYSNES_HAVE_CORE */
