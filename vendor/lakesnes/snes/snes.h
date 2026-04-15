
#ifndef SNES_H
#define SNES_H

#include <stdint.h>
#include <stdbool.h>

typedef struct Snes Snes;

#include "cpu.h"
#include "apu.h"
#include "dma.h"
#include "ppu.h"
#include "cart.h"
#include "input.h"
#include "statehandler.h"

struct Snes {
  Cpu* cpu;
  Apu* apu;
  Ppu* ppu;
  Dma* dma;
  Cart* cart;
  bool palTiming;
  // input
  Input* input1;
  Input* input2;
  // ram
  uint8_t ram[0x20000];
  uint32_t ramAdr;
  // frame timing
  uint16_t hPos;
  uint16_t vPos;
  uint32_t frames;
  uint64_t cycles;
  uint64_t syncCycle;
  // cpu handling — float not double, see ratio constants in snes.c
  float apuCatchupCycles;
  // nmi / irq
  bool hIrqEnabled;
  bool vIrqEnabled;
  bool nmiEnabled;
  uint16_t hTimer;
  uint16_t vTimer;
  bool inNmi;
  bool irqCondition;
  bool inIrq;
  bool inVblank;
  // joypad handling
  uint16_t portAutoRead[4]; // as read by auto-joypad read
  bool autoJoyRead;
  uint16_t autoJoyTimer; // times how long until reading is done
  bool ppuLatch;
  // multiplication/division
  uint8_t multiplyA;
  uint16_t multiplyResult;
  uint16_t divideA;
  uint16_t divideResult;
  // misc
  bool fastMem;
  uint8_t openBus;

  /* ThumbySNES block-based memory map (snes9x2002 architecture).
   *
   * 24-bit SNES address space divided into 4 KB blocks (4096 entries).
   * Each entry is either a direct pointer (for ROM/WRAM — dereference
   * with block-local offset for ~95% of reads in 3 instructions) or
   * a sentinel < SNES_MAP_LAST (fall through to slow path for PPU,
   * APU, registers, DMA).
   *
   * Replaces the 6-8 cascading if-chains in snes_rread. With 1.8M
   * reads/frame, this saves ~10M branch instructions per frame.
   *
   * readMapSpeed[block] = access cycle cost (6, 8, or 12) for the
   * per-opcode cycle accumulator. */
#define SNES_MAP_SHIFT     12
#define SNES_MAP_BLOCK     (1 << SNES_MAP_SHIFT)  /* 4096 */
#define SNES_MAP_MASK      (SNES_MAP_BLOCK - 1)    /* 0xFFF */
#define SNES_MAP_BLOCKS    4096

#define SNES_MAP_SPECIAL   ((uint8_t*)1)  /* PPU, APU, input, regs, DMA */
#define SNES_MAP_LAST      ((uint8_t*)16) /* anything >= is a real pointer */

  uint8_t *readMap[SNES_MAP_BLOCKS];
  uint8_t  readMapSpeed[SNES_MAP_BLOCKS];

  /* Per-opcode cycle accumulator. The block-map fast path adds cycles
   * here instead of calling snes_runCycles per access. Flushed once
   * per opcode in cpu_runOpcode via snes_flushCycles. Reduces
   * snes_runCycles calls from ~1.8M/frame to ~600K/frame. */
  int pendingCycles;
};

Snes* snes_init(void);
void snes_free(Snes* snes);
void snes_reset(Snes* snes, bool hard);
void snes_handleState(Snes* snes, StateHandler* sh);
void snes_runFrame(Snes* snes);
// used by dma, cpu
void snes_runCycles(Snes* snes, int cycles);

/* Flush accumulated cycles from the block-map fast path. Called once
 * per opcode from cpu_runOpcode. Declared in header for inlining in
 * cpu.c; also emitted as a real symbol in snes.c for cpu_asm.c. */
static inline void snes_flushCycles(Snes* snes) {
  if (snes->pendingCycles > 0) {
    snes_runCycles(snes, snes->pendingCycles);
    snes->pendingCycles = 0;
  }
}
/* Force a non-inline copy for external callers (cpu_asm.c) */
#ifndef SNES_FLUSH_EXTERN_DEFINED
#define SNES_FLUSH_EXTERN_DEFINED
#endif
void snes_syncCycles(Snes* snes, bool start, int syncCycles);
uint8_t snes_readBBus(Snes* snes, uint8_t adr);
void snes_writeBBus(Snes* snes, uint8_t adr, uint8_t val);
uint8_t snes_read(Snes* snes, uint32_t adr);
void snes_write(Snes* snes, uint32_t adr, uint8_t val);
void snes_cpuIdle(void* mem, bool waiting);
uint8_t snes_cpuRead(void* mem, uint32_t adr);
void snes_cpuWrite(void* mem, uint32_t adr, uint8_t val);
// debugging
void snes_runCpuCycle(Snes* snes);
void snes_runSpcCycle(Snes* snes);

// snes_other.c functions:

enum { pixelFormatXRGB = 0, pixelFormatRGBX = 1 };

bool snes_loadRom(Snes* snes, const uint8_t* data, int length);
void snes_setButtonState(Snes* snes, int player, int button, bool pressed);
void snes_setPixelFormat(Snes* snes, int pixelFormat);
void snes_setPixels(Snes* snes, uint8_t* pixelData);
void snes_setSamples(Snes* snes, int16_t* sampleData, int samplesPerFrame);
int snes_saveBattery(Snes* snes, uint8_t* data);
bool snes_loadBattery(Snes* snes, uint8_t* data, int size);
int snes_saveState(Snes* snes, uint8_t* data);
bool snes_loadState(Snes* snes, uint8_t* data, int size);

#endif
