/*
 * ThumbySNES — Phase 1 SDL2 host runner.
 *
 * Usage:
 *   sneshost <rom.smc>
 *
 * Window modes (toggle with TAB at runtime):
 *   "full"   — 256x224 SNES output, scaled 2x for visibility (512x448 window).
 *   "thumby" — 128x128 downscaled preview (what the device will actually show),
 *              scaled 4x for visibility (512x512 window). Launch with `--thumby`
 *              to start in this mode, or press TAB once running.
 *
 * Keyboard mirrors the Thumby's YX profile (see PLAN.md §7) plus a few extras
 * for testing: A/B/LB/RB → Z/X/Q/W plus SNES L/R on the same keys via the LR
 * profile Thumby will eventually swap into.
 */
#include "snes_core.h"

#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define SNES_W 256
#define SNES_H 224
#define THUMBY_W 128
#define THUMBY_H 128
#define FULL_SCALE 2
#define THUMBY_SCALE 4

/* FIT mode: 112-line 2:1-downscaled SNES image centered in 128 lines,
 * 8px letterbox top+bottom. Horizontal is an even 2:1 drop. */
#define FIT_Y_OFFSET 8          /* (128 - 112) / 2 */
#define FIT_SRC_ROWS 112        /* 224 / 2 */

/* FILL mode: 224x224 square crop from the 256x224 SNES frame (16px off each
 * horizontal edge), downscaled 7:4 (224 → 128) both axes — fills the full
 * 128x128 screen at the cost of losing 16 columns of gameplay each side. */
#define FILL_X_CROP 16          /* (256 - 224) / 2 */
#define FILL_SRC    224

static uint16_t s_fb_full[SNES_W * SNES_H];
static uint16_t s_fb_thumby[THUMBY_W * THUMBY_H];

/* 2x2 box-average in RGB565 channel space (R 5b, G 6b, B 5b summed across
 * 4 source pixels, divided by 4, repacked). Small helper shared by the
 * two downscale paths. */
static inline uint16_t rgb565_avg2x2(uint16_t a, uint16_t b, uint16_t c, uint16_t d)
{
    uint32_t rsum = ((a >> 11) & 0x1F) + ((b >> 11) & 0x1F)
                  + ((c >> 11) & 0x1F) + ((d >> 11) & 0x1F);
    uint32_t gsum = ((a >>  5) & 0x3F) + ((b >>  5) & 0x3F)
                  + ((c >>  5) & 0x3F) + ((d >>  5) & 0x3F);
    uint32_t bsum = (a & 0x1F) + (b & 0x1F) + (c & 0x1F) + (d & 0x1F);
    return (uint16_t)(((rsum >> 2) << 11) | ((gsum >> 2) << 5) | (bsum >> 2));
}

/* FIT + blend: 256x224 → 128x112 letterboxed, 2:1 drop both axes with 2x2
 * blend. SNES version of ThumbyNES's blit_blend (nes_run.c:139). */
static void downscale_fit(const uint16_t *src, uint16_t *dst)
{
    memset(dst, 0, THUMBY_W * THUMBY_H * sizeof(uint16_t));
    for (int dy = 0; dy < FIT_SRC_ROWS; dy++) {
        const uint16_t *r0 = src + (dy * 2)     * SNES_W;
        const uint16_t *r1 = src + (dy * 2 + 1) * SNES_W;
        uint16_t       *out = dst + (FIT_Y_OFFSET + dy) * THUMBY_W;
        for (int dx = 0; dx < THUMBY_W; dx++) {
            int sx = dx * 2;
            out[dx] = rgb565_avg2x2(r0[sx], r0[sx + 1], r1[sx], r1[sx + 1]);
        }
    }
}

/* FILL + blend: 224x224 square → 128x128, 7:4 nearest-stride + 2x2 blend
 * at each output pixel's source anchor. Not bilinear (expensive) — instead
 * a 2x2 box average sampled at stride-1.75 positions. Source x within the
 * cropped 224-wide window; offset by FILL_X_CROP to land inside the 256-wide
 * SNES buffer.
 *
 * Resulting pattern: every 4-output block consumes 7 source pixels
 * (strides 2,2,2,1 per output — some pixels overlap adjacent boxes), so the
 * aliasing is uneven but the image fills the full 128x128 screen instead of
 * letterboxing to 112 rows. */
static void downscale_fill(const uint16_t *src, uint16_t *dst)
{
    for (int oy = 0; oy < THUMBY_H; oy++) {
        int sy  = (oy * 7) >> 2;                 /* floor(oy * 1.75), 0..223 */
        int sy2 = sy + 1; if (sy2 > FILL_SRC - 1) sy2 = FILL_SRC - 1;
        const uint16_t *r0 = src + sy  * SNES_W + FILL_X_CROP;
        const uint16_t *r1 = src + sy2 * SNES_W + FILL_X_CROP;
        uint16_t       *out = dst + oy * THUMBY_W;
        for (int ox = 0; ox < THUMBY_W; ox++) {
            int sx  = (ox * 7) >> 2;             /* 0..223 */
            int sx2 = sx + 1; if (sx2 > FILL_SRC - 1) sx2 = FILL_SRC - 1;
            out[ox] = rgb565_avg2x2(r0[sx], r0[sx2], r1[sx], r1[sx2]);
        }
    }
}

static uint16_t keys_to_pad(void)
{
    const Uint8 *k = SDL_GetKeyboardState(NULL);
    uint16_t p = 0;
    if (k[SDL_SCANCODE_UP])     p |= SNES_BTN_UP;
    if (k[SDL_SCANCODE_DOWN])   p |= SNES_BTN_DOWN;
    if (k[SDL_SCANCODE_LEFT])   p |= SNES_BTN_LEFT;
    if (k[SDL_SCANCODE_RIGHT])  p |= SNES_BTN_RIGHT;
    if (k[SDL_SCANCODE_X])      p |= SNES_BTN_A;
    if (k[SDL_SCANCODE_Z])      p |= SNES_BTN_B;
    if (k[SDL_SCANCODE_S])      p |= SNES_BTN_X;
    if (k[SDL_SCANCODE_A])      p |= SNES_BTN_Y;
    if (k[SDL_SCANCODE_Q])      p |= SNES_BTN_L;
    if (k[SDL_SCANCODE_W])      p |= SNES_BTN_R;
    if (k[SDL_SCANCODE_RETURN]) p |= SNES_BTN_START;
    if (k[SDL_SCANCODE_RSHIFT] || k[SDL_SCANCODE_LSHIFT]) p |= SNES_BTN_SELECT;
    return p;
}

static void audio_cb(void *user, Uint8 *stream, int len)
{
    (void)user;
    size_t frames = len / 4; /* stereo s16 */
    snes_get_audio((int16_t *)stream, frames);
}

/* Display modes cycled with TAB:
 *   FILL — 224x224 crop → 128x128 (new default, fills the screen)
 *   FIT  — 256x224 → 128x112 letterboxed (matches NES blit_blend shape)
 *   FULL — 256x224 raw, 2x upscale (for comparing what blend loses) */
enum { MODE_FILL = 0, MODE_FIT = 1, MODE_FULL = 2, MODE_COUNT = 3 };

typedef struct {
    SDL_Window *win;
    SDL_Renderer *ren;
    SDL_Texture *tex;
    int tex_w, tex_h;
    int mode;
} display_t;

static void display_recreate(display_t *d, int mode)
{
    if (d->tex) SDL_DestroyTexture(d->tex);
    if (d->ren) SDL_DestroyRenderer(d->ren);
    if (d->win) SDL_DestroyWindow(d->win);

    int win_w, win_h;
    const char *title;
    switch (mode) {
    case MODE_FILL:
        d->tex_w = THUMBY_W; d->tex_h = THUMBY_H;
        win_w = THUMBY_W * THUMBY_SCALE; win_h = THUMBY_H * THUMBY_SCALE;
        title = "ThumbySNES — 128x128 FILL (224²→128², 7:4 blend)  [TAB to cycle]";
        break;
    case MODE_FIT:
        d->tex_w = THUMBY_W; d->tex_h = THUMBY_H;
        win_w = THUMBY_W * THUMBY_SCALE; win_h = THUMBY_H * THUMBY_SCALE;
        title = "ThumbySNES — 128x128 FIT (256x224→128x112 + letterbox)  [TAB to cycle]";
        break;
    default: /* MODE_FULL */
        d->tex_w = SNES_W; d->tex_h = SNES_H;
        win_w = SNES_W * FULL_SCALE; win_h = SNES_H * FULL_SCALE;
        title = "ThumbySNES — 256x224 FULL (native)  [TAB to cycle]";
        break;
    }
    d->mode = mode;
    d->win = SDL_CreateWindow(title,
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, win_w, win_h, 0);
    d->ren = SDL_CreateRenderer(d->win, -1, SDL_RENDERER_ACCELERATED);
    d->tex = SDL_CreateTexture(d->ren, SDL_PIXELFORMAT_RGB565,
        SDL_TEXTUREACCESS_STREAMING, d->tex_w, d->tex_h);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr,
            "usage: %s <rom.smc> [--fill | --fit | --full]\n"
            "  TAB at runtime cycles through FILL → FIT → FULL.\n"
            "  FILL: 224² crop → 128x128 (default, fills the device screen).\n"
            "  FIT:  256x224 → 128x112 letterboxed (more side room, 16px black bars).\n"
            "  FULL: 256x224 native 2x — desktop-only, for reference.\n",
            argv[0]);
        return 1;
    }
    int start_mode = MODE_FILL;
    int use_xip    = 0;
    for (int i = 2; i < argc; i++) {
        if      (!strcmp(argv[i], "--fill")) start_mode = MODE_FILL;
        else if (!strcmp(argv[i], "--fit"))  start_mode = MODE_FIT;
        else if (!strcmp(argv[i], "--full")) start_mode = MODE_FULL;
        /* legacy alias */
        else if (!strcmp(argv[i], "--thumby")) start_mode = MODE_FIT;
        else if (!strcmp(argv[i], "--xip"))    use_xip = 1;
    }

    int fd = open(argv[1], O_RDONLY);
    if (fd < 0) { perror("open"); return 1; }
    struct stat st;
    fstat(fd, &st);
    const uint8_t *rom = mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (rom == MAP_FAILED) { perror("mmap"); close(fd); return 1; }

    snes_result_t r = use_xip ? snes_load_xip(rom, st.st_size)
                              : snes_load    (rom, st.st_size);
    if (r != SNES_OK) {
        fprintf(stderr, "snes_load: %d\n", r);
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    display_t d = {0};
    display_recreate(&d, start_mode);

    SDL_AudioSpec want = {0}, got = {0};
    want.freq = 32040;
    want.format = AUDIO_S16SYS;
    want.channels = 2;
    want.samples = 1024;
    want.callback = audio_cb;
    SDL_AudioDeviceID adev = SDL_OpenAudioDevice(NULL, 0, &want, &got, 0);
    if (adev) SDL_PauseAudioDevice(adev, 0);

    int running = 1;
    Uint64 freq = SDL_GetPerformanceFrequency();
    Uint64 next = SDL_GetPerformanceCounter();

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = 0;
            if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_ESCAPE) running = 0;
                if (e.key.keysym.sym == SDLK_TAB) {
                    display_recreate(&d, (d.mode + 1) % MODE_COUNT);
                }
            }
        }
        snes_set_pad(keys_to_pad());
        snes_run_frame();
        snes_get_framebuffer(s_fb_full);

        const uint16_t *src;
        int pitch_bytes;
        if (d.mode == MODE_FILL) {
            downscale_fill(s_fb_full, s_fb_thumby);
            src = s_fb_thumby;
            pitch_bytes = THUMBY_W * sizeof(uint16_t);
        } else if (d.mode == MODE_FIT) {
            downscale_fit(s_fb_full, s_fb_thumby);
            src = s_fb_thumby;
            pitch_bytes = THUMBY_W * sizeof(uint16_t);
        } else {
            src = s_fb_full;
            pitch_bytes = SNES_W * sizeof(uint16_t);
        }
        SDL_UpdateTexture(d.tex, NULL, src, pitch_bytes);
        SDL_RenderClear(d.ren);
        SDL_RenderCopy(d.ren, d.tex, NULL, NULL);
        SDL_RenderPresent(d.ren);

        next += freq / 60;
        Uint64 nowc = SDL_GetPerformanceCounter();
        if (nowc < next) {
            double ms = (double)(next - nowc) * 1000.0 / (double)freq;
            if (ms > 0.0 && ms < 32.0) SDL_Delay((Uint32)ms);
        } else {
            next = nowc;
        }
    }

    if (adev) SDL_CloseAudioDevice(adev);
    if (d.tex) SDL_DestroyTexture(d.tex);
    if (d.ren) SDL_DestroyRenderer(d.ren);
    if (d.win) SDL_DestroyWindow(d.win);
    SDL_Quit();

    snes_unload();
    munmap((void *)rom, st.st_size);
    close(fd);
    return 0;
}
