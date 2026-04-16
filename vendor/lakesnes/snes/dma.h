
#ifndef DMA_H
#define DMA_H

#include <stdint.h>
#include <stdbool.h>

typedef struct Dma Dma;

#include "snes.h"
#include "statehandler.h"

typedef struct DmaChannel {
  uint8_t bAdr;
  uint16_t aAdr;
  uint8_t aBank;
  uint16_t size; // also indirect hdma adr
  uint8_t indBank; // hdma
  uint16_t tableAdr; // hdma
  uint8_t repCount; // hdma
  uint8_t unusedByte;
  bool dmaActive;
  bool hdmaActive;
  uint8_t mode;
  bool fixed;
  bool decrement;
  bool indirect; // hdma
  bool fromB;
  bool unusedBit;
  bool doTransfer; // hdma
  bool terminated; // hdma
} DmaChannel;

struct Dma {
  Snes* snes;
  DmaChannel channel[8];
  uint8_t dmaState;
  bool hdmaInitRequested;
  bool hdmaRunRequested;
};

Dma* dma_init(Snes* snes);
void dma_free(Dma* dma);
void dma_reset(Dma* dma);
void dma_handleState(Dma* dma, StateHandler* sh);
uint8_t dma_read(Dma* dma, uint16_t adr); // 43x0-43xf
void dma_write(Dma* dma, uint16_t adr, uint8_t val); // 43x0-43xf
void dma_handleDmaSlow(Dma* dma, int cpuCycles);
void dma_startDma(Dma* dma, uint8_t val, bool hdma);

/* ThumbySNES fast-exit guard. dma_handleDma is called from every
 * cpu read/write/idle (40-48M times/frame). For the overwhelming
 * majority, nothing is pending and the function returns immediately
 * after three flag checks. Inlining the fast-exit here kills the
 * ~10% of core-0 frame time spent on the call+return overhead. */
static inline void dma_handleDma(Dma* dma, int cpuCycles) {
  if (dma->dmaState == 0
      && !dma->hdmaInitRequested
      && !dma->hdmaRunRequested) return;
  dma_handleDmaSlow(dma, cpuCycles);
}

#endif
