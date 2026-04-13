
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

#endif
