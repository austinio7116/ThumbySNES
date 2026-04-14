
#ifndef PPU_H
#define PPU_H

#include <stdint.h>
#include <stdbool.h>

typedef struct Ppu Ppu;

#include "snes.h"
#include "statehandler.h"

typedef struct BgLayer {
  uint16_t hScroll;
  uint16_t vScroll;
  bool tilemapWider;
  bool tilemapHigher;
  uint16_t tilemapAdr;
  uint16_t tileAdr;
  bool bigTiles;
  bool mosaicEnabled;
} BgLayer;

typedef struct Layer {
  bool mainScreenEnabled;
  bool subScreenEnabled;
  bool mainScreenWindowed;
  bool subScreenWindowed;
} Layer;

typedef struct WindowLayer {
  bool window1enabled;
  bool window2enabled;
  bool window1inversed;
  bool window2inversed;
  uint8_t maskLogic;
} WindowLayer;

struct Ppu {
  Snes* snes;
  // vram access
  uint16_t vram[0x8000];
  uint16_t vramPointer;
  bool vramIncrementOnHigh;
  uint16_t vramIncrement;
  uint8_t vramRemapMode;
  uint16_t vramReadBuffer;
  // cgram access
  uint16_t cgram[0x100];
  uint8_t cgramPointer;
  bool cgramSecondWrite;
  uint8_t cgramBuffer;
  // oam access
  uint16_t oam[0x100];
  uint8_t highOam[0x20];
  uint8_t oamAdr;
  uint8_t oamAdrWritten;
  bool oamInHigh;
  bool oamInHighWritten;
  bool oamSecondWrite;
  uint8_t oamBuffer;
  // object/sprites
  bool objPriority;
  uint16_t objTileAdr1;
  uint16_t objTileAdr2;
  uint8_t objSize;
  uint8_t objPixelBuffer[256]; // line buffers
  uint8_t objPriorityBuffer[256];
  bool timeOver;
  bool rangeOver;
  bool objInterlace;
  // background layers
  BgLayer bgLayer[4];
  uint8_t scrollPrev;
  uint8_t scrollPrev2;
  uint8_t mosaicSize;
  uint8_t mosaicStartLine;
  // layers
  Layer layer[5];
  // mode 7
  int16_t m7matrix[8]; // a, b, c, d, x, y, h, v
  uint8_t m7prev;
  bool m7largeField;
  bool m7charFill;
  bool m7xFlip;
  bool m7yFlip;
  bool m7extBg;
  // mode 7 internal
  int32_t m7startX;
  int32_t m7startY;
  // windows
  WindowLayer windowLayer[6];
  uint8_t window1left;
  uint8_t window1right;
  uint8_t window2left;
  uint8_t window2right;
  // color math
  uint8_t clipMode;
  uint8_t preventMathMode;
  bool addSubscreen;
  bool subtractColor;
  bool halfColor;
  bool mathEnabled[6];
  uint8_t fixedColorR;
  uint8_t fixedColorG;
  uint8_t fixedColorB;
  // settings
  bool forcedBlank;
  uint8_t brightness;
  uint8_t mode;
  bool bg3priority;
  bool evenFrame;
  bool pseudoHires;
  bool overscan;
  bool frameOverscan; // if we are overscanning this frame (determined at 0,225)
  bool interlace;
  bool frameInterlace; // if we are interlacing this frame (determined at start vblank)
  bool directColor;
  // latching
  uint16_t hCount;
  uint16_t vCount;
  bool hCountSecond;
  bool vCountSecond;
  bool countersLatched;
  uint8_t ppu1openBus;
  uint8_t ppu2openBus;
  /* ThumbySNES per-line render. Upstream LakeSnes stored a full XBGR
   * frame buffer (`512 * 4 * 239 * 2 = 978 KB`) for the SDL frontend
   * to pull whole frames out of. We can't afford that on a 520 KB
   * MCU — instead the buffer is exactly one scanline (256 px × 8 B)
   * and the frontend installs `scanlineCb`, called by ppu_runLine
   * after every visible line with the row-0 contents. The frontend
   * then downscales / blits / discards before the next line writes.
   *
   * Loses the upstream interlace double-buffering (we render every
   * frame, ignore `evenFrame`). Trivial for non-interlaced games. */
  uint8_t pixelBuffer[2048];
  uint8_t pixelOutputFormat;
  void  (*scanlineCb)(void* user, int line, const uint8_t* lineBuffer);
  void   *scanlineUser;

  /* ThumbySNES per-line RGB565 scanline output.
   *
   * Modern fast path (preferred over the BGRX pixelBuffer flow above).
   * ppu_runLine builds a fully composited 256-pixel RGB565 line via
   * tight per-slot passes (bottom-up over layerPerMode, overwriting
   * non-zero pixels), then hands it to `scanlineCbRgb565`. The
   * frontend consumes RGB565 directly — no BGRX-byte conversion, no
   * per-pixel channel math, no subscreen duplication.
   *
   * Compared to the per-pixel ppu_handlePixel path:
   *   - 8× less scanline memory traffic (2 bytes/pixel vs 16),
   *   - per-pixel channel-scale + brightness math collapsed to a
   *     single cgramRgb565[] lookup,
   *   - layer iteration collapses from O(slots × pixels) with
   *     function-call-per-pixel window checks to O(slots + pixels)
   *     with per-slot window-mask precompute. */
  uint16_t lineRgb565[256];
  void  (*scanlineCbRgb565)(void* user, int line, const uint16_t* line565);
  void   *scanlineUserRgb565;

  /* ThumbySNES per-line BG render cache.
   *
   * Upstream's ppu_getPixel is called once per pixel per layer slot
   * (~70 KB calls per frame for SMW). Each call re-fetches the
   * tilemap word + 1-4 plane words from VRAM and decodes the pixel.
   * The same tile is fetched 8 times in a row for 8 consecutive
   * pixels — pure waste.
   *
   * `bgLine[layer][priority][x]` caches the resolved palette index
   * for each of 256 output pixels per BG layer per priority slot,
   * computed once at the start of ppu_runLine. ppu_getPixel then
   * just reads the array.
   *
   * `bgLineCacheValid[layer]` = 1 only when the simple-case path was
   * used for this line (no mosaic, no Mode 7, no hires modes 5/6,
   * no OPT modes 2/4/6). Otherwise ppu_getPixel falls back to the
   * old per-pixel ppu_getPixelForBgLayer path. */
  uint8_t bgLine[4][2][256];
  uint8_t bgLineCacheValid[4];

  /* Per-priority "contains any non-zero pixel" flag set by
   * ppu_renderBgLine. Lets the compositor skip slots that are fully
   * transparent for this line (common — e.g. BG3 water tint / BG2
   * during screens where only BG1 + sprites are in use). Saves a
   * full 256-wide pass per skipped slot. */
  uint8_t bgLineNonEmpty[4][2];

  /* ThumbySNES: when set, ppu_handlePixel forces mathEnabled = false
   * for every pixel — skips the recursive subscreen fetch and the
   * add/sub colour math. Loses SNES transparency effects (HUD fades,
   * lantern glow, water tint) but cuts per-pixel work nearly in half
   * for games that use colour math heavily. */
  bool skipColorMath;

  /* ThumbySNES: render only x ∈ [renderXStart, renderXEnd) per line.
   * Default 0/256 (full SNES width). For FILL mode on a 128x128 LCD
   * we crop 16 px off each horizontal edge — set to 16/240 to skip
   * 32 invisible pixels per line (~12.5% fewer ppu_handlePixel calls). */
  int renderXStart;
  int renderXEnd;

  /* ThumbySNES native-LCD render mode.
   *
   * When `lcdFb` is non-NULL the PPU bypasses the 256-wide `pixelBuffer`
   * + BGRX-bytes pipeline entirely and composites RGB565 straight into
   * the caller's LCD framebuffer at native device resolution. Huge win
   * on a 128×128 LCD: we collapse 224×224 source pixels (≈50K calls to
   * ppu_handlePixel per frame) to 128×128 (≈16K) — a 3× cut on the
   * dominant cost in the profile.
   *
   *   lcdFb          = output framebuffer, RGB565, row-major, row 0 at top.
   *   lcdFbW, lcdFbH = output dimensions (expected 128×128).
   *   lcdSrcX[ox]    = source SNES x (0..255) sampled for device column ox.
   *   lcdDevY[sy]    = device row this SNES line (sy=0..223) maps onto,
   *                    or -1 if that line should be skipped entirely for
   *                    pixel compositing (its sy maps to a device row
   *                    that a lower sy already filled).
   *
   * Sprite eval + BG line cache still run on every visible SNES line so
   * OAM range/time-over state stays correct; the only work skipped is
   * per-pixel compositing for lines that don't survive downsampling.
   *
   * When `lcdFb` is NULL the PPU runs its classic full-width path — host
   * build and unit tests use that. */
  uint16_t *lcdFb;
  int       lcdFbW;
  int       lcdFbH;
  uint8_t   lcdSrcX[128];
  int8_t    lcdDevY[239];  /* 224 visible lines (+slack for overscan). */

  /* ThumbySNES CGRAM → RGB565 cache with brightness baked in.
   *
   * ppu_handlePixel's classic path re-derives r/g/b/brightness math
   * from cgram per pixel. In LCD mode we collapse that to a single
   * uint16_t array read — the table is rebuilt lazily when CGRAM or
   * brightness changes. `cgramDirty` is set by writes to $2121/$2122
   * and to $2100 (brightness), and cleared on rebuild. */
  uint16_t  cgramRgb565[256];
  uint8_t   cgramDirty;

  /* ThumbySNES frameskip hint — when non-zero, ppu_runLine skips all
   * work (sprite eval, BG cache, compositing). snes_core toggles this
   * for frameskipped frames. */
  uint8_t   skipRender;

  /* ThumbySNES half-vertical mode. When set, ppu_runLine skips "blend
   * partner" lines — the second SNES line mapping to the same device
   * row under 7:4 vertical stride. ~96 of 224 lines are skipped,
   * cutting PPU per-line work by ~43%. The device scanline callback
   * handles missing partners gracefully (writes the first-hit line
   * as-is without vertical averaging). Horizontal 2-sample blend is
   * preserved; only vertical smoothing is lost. */
  uint8_t   halfVertical;
};

enum { ppu_pixelOutputFormatXBGR = 0, ppu_pixelOutputFormatBGRX = 1 };

Ppu* ppu_init(Snes* snes);
void ppu_free(Ppu* ppu);
void ppu_reset(Ppu* ppu);
void ppu_handleState(Ppu* ppu, StateHandler* sh);
bool ppu_checkOverscan(Ppu* ppu);
void ppu_handleVblank(Ppu* ppu);
void ppu_handleFrameStart(Ppu* ppu);
void ppu_runLine(Ppu* ppu, int line);
uint8_t ppu_read(Ppu* ppu, uint8_t adr);
void ppu_write(Ppu* ppu, uint8_t adr, uint8_t val);
void ppu_putPixels(Ppu* ppu, uint8_t* pixels);   /* legacy stub — see ppu.c */
void ppu_setPixelOutputFormat(Ppu* ppu, int pixelOutputFormat);
void ppu_setScanlineCallback(Ppu* ppu,
                             void (*cb)(void*, int, const uint8_t*),
                             void *user);

/* ThumbySNES: install an RGB565 per-line callback. When non-NULL,
 * ppu_runLine uses the per-line full-composite fast path and fires
 * this cb with the 256-pixel RGB565 line. Mutually exclusive with
 * the classic scanlineCb (the RGB565 path takes precedence — setting
 * both is a no-op for the BGRX callback). */
void ppu_setScanlineCbRgb565(Ppu* ppu,
                             void (*cb)(void*, int, const uint16_t*),
                             void *user);

/* ThumbySNES: enable native-LCD rendering. `lcdFb` is a W×H RGB565
 * framebuffer; the PPU writes composited output directly into it with
 * no intermediate pixelBuffer or scanline callback. Passing lcdFb=NULL
 * returns the PPU to its classic full-width path. */
void ppu_setLcdMode(Ppu* ppu, uint16_t *lcdFb, int lcdFbW, int lcdFbH,
                    const uint8_t *lcdSrcX, const int8_t *lcdDevY);

#endif
