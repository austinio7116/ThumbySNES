/*
 * ppu_emit_asm.h — hand-rolled Thumb-2 inner loops for the PPU tile-row
 * emit path.
 *
 * Replaces the 8-iteration per-pixel loops in
 * ppu_emitBgSlotRgb565's cache-hit branch. Net effect: ~8 ARM
 * instructions per pixel compiled → ~3 instr per non-zero pixel
 * and ~2 per zero, via:
 *   - Two aligned LDR's of 8 cached source bytes
 *   - UBFX to extract each byte from the 32-bit packed word
 *   - CBZ to skip zero pixels with no condition flag work
 *   - LDRH [rCG, rP, LSL #1] for the palette lookup (1 instr)
 *   - STRH with immediate offset for each output pixel
 *
 * Two entry points:
 *   ppu_emit8_nonwin   — no window mask, all 8 pixels candidate-writable
 *   ppu_emit8_windowed — winMask[0..7] non-zero cells are skipped
 *
 * Both REQUIRE the caller to have clipped to the visible range
 * (outX >= 0 && outX + 8 <= 256). Edge tiles stay on the per-pixel C
 * clipped fallback.
 *
 * Host builds fall back to pure C loops.
 *
 * Note on labels: GCC's `%=` inline-asm directive emits a unique number
 * so labels don't collide across multiple inlinings of the template.
 * Numeric local labels (`1:`, `1f`) don't compose with `%=`, so we use
 * named labels like `.Lpi1_%=`.
 */
#ifndef THUMBYSNES_PPU_EMIT_ASM_H
#define THUMBYSNES_PPU_EMIT_ASM_H

#include <stdint.h>

#if defined(__arm__) || defined(__thumb__)

static inline void __attribute__((always_inline))
ppu_emit8_nonwin(uint16_t *out, const uint8_t *src, const uint16_t *cg)
{
    uint32_t s0, s1, p, v;
    __asm__ volatile (
        "ldr   %[s0], [%[src], #0]            \n"
        "ldr   %[s1], [%[src], #4]            \n"
        "uxtb  %[p], %[s0]                    \n"
        "cbz   %[p], .Lpi1_%=                 \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #0]             \n"
        ".Lpi1_%=: ubfx %[p], %[s0], #8, #8   \n"
        "cbz   %[p], .Lpi2_%=                 \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #2]             \n"
        ".Lpi2_%=: ubfx %[p], %[s0], #16, #8  \n"
        "cbz   %[p], .Lpi3_%=                 \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #4]             \n"
        ".Lpi3_%=: ubfx %[p], %[s0], #24, #8  \n"
        "cbz   %[p], .Lpi4_%=                 \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #6]             \n"
        ".Lpi4_%=: uxtb %[p], %[s1]           \n"
        "cbz   %[p], .Lpi5_%=                 \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #8]             \n"
        ".Lpi5_%=: ubfx %[p], %[s1], #8, #8   \n"
        "cbz   %[p], .Lpi6_%=                 \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #10]            \n"
        ".Lpi6_%=: ubfx %[p], %[s1], #16, #8  \n"
        "cbz   %[p], .Lpi7_%=                 \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #12]            \n"
        ".Lpi7_%=: ubfx %[p], %[s1], #24, #8  \n"
        "cbz   %[p], .Lpi8_%=                 \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #14]            \n"
        ".Lpi8_%=:                            \n"
        : [s0] "=&r"(s0), [s1] "=&r"(s1),
          [p]  "=&r"(p),  [v]  "=&r"(v)
        : [src] "r"(src), [cg] "r"(cg), [out] "r"(out)
        : "cc", "memory"
    );
}

static inline void __attribute__((always_inline))
ppu_emit8_windowed(uint16_t *out, const uint8_t *src,
                   const uint16_t *cg, const uint8_t *winMask)
{
    uint32_t s0, s1, w0, w1, p, v;
    __asm__ volatile (
        "ldr   %[s0], [%[src], #0]            \n"
        "ldr   %[s1], [%[src], #4]            \n"
        "ldr   %[w0], [%[wm],  #0]            \n"
        "ldr   %[w1], [%[wm],  #4]            \n"

        "uxtb  %[p], %[s0]                    \n"
        "cbz   %[p], .Lwpi1_%=                \n"
        "tst   %[w0], #0xff                   \n"
        "bne   .Lwpi1_%=                      \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #0]             \n"

        ".Lwpi1_%=: ubfx %[p], %[s0], #8, #8  \n"
        "cbz   %[p], .Lwpi2_%=                \n"
        "tst   %[w0], #0xff00                 \n"
        "bne   .Lwpi2_%=                      \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #2]             \n"

        ".Lwpi2_%=: ubfx %[p], %[s0], #16, #8 \n"
        "cbz   %[p], .Lwpi3_%=                \n"
        "tst   %[w0], #0xff0000               \n"
        "bne   .Lwpi3_%=                      \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #4]             \n"

        ".Lwpi3_%=: ubfx %[p], %[s0], #24, #8 \n"
        "cbz   %[p], .Lwpi4_%=                \n"
        "tst   %[w0], #0xff000000             \n"
        "bne   .Lwpi4_%=                      \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #6]             \n"

        ".Lwpi4_%=: uxtb %[p], %[s1]          \n"
        "cbz   %[p], .Lwpi5_%=                \n"
        "tst   %[w1], #0xff                   \n"
        "bne   .Lwpi5_%=                      \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #8]             \n"

        ".Lwpi5_%=: ubfx %[p], %[s1], #8, #8  \n"
        "cbz   %[p], .Lwpi6_%=                \n"
        "tst   %[w1], #0xff00                 \n"
        "bne   .Lwpi6_%=                      \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #10]            \n"

        ".Lwpi6_%=: ubfx %[p], %[s1], #16, #8 \n"
        "cbz   %[p], .Lwpi7_%=                \n"
        "tst   %[w1], #0xff0000               \n"
        "bne   .Lwpi7_%=                      \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #12]            \n"

        ".Lwpi7_%=: ubfx %[p], %[s1], #24, #8 \n"
        "cbz   %[p], .Lwpi8_%=                \n"
        "tst   %[w1], #0xff000000             \n"
        "bne   .Lwpi8_%=                      \n"
        "ldrh  %[v], [%[cg], %[p], lsl #1]    \n"
        "strh  %[v], [%[out], #14]            \n"
        ".Lwpi8_%=:                           \n"
        : [s0] "=&r"(s0), [s1] "=&r"(s1),
          [w0] "=&r"(w0), [w1] "=&r"(w1),
          [p]  "=&r"(p),  [v]  "=&r"(v)
        : [src] "r"(src), [cg] "r"(cg),
          [out] "r"(out), [wm] "r"(winMask)
        : "cc", "memory"
    );
}

#else /* non-ARM: C fallback */

static inline void
ppu_emit8_nonwin(uint16_t *out, const uint8_t *src, const uint16_t *cg)
{
    for (int pi = 0; pi < 8; pi++) {
        uint8_t p = src[pi];
        if (p) out[pi] = cg[p];
    }
}

static inline void
ppu_emit8_windowed(uint16_t *out, const uint8_t *src,
                   const uint16_t *cg, const uint8_t *winMask)
{
    for (int pi = 0; pi < 8; pi++) {
        uint8_t p = src[pi];
        if (p && !winMask[pi]) out[pi] = cg[p];
    }
}

#endif

#endif /* THUMBYSNES_PPU_EMIT_ASM_H */
