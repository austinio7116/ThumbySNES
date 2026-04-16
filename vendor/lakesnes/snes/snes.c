
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "snes.h"
#include "cpu.h"
#include "apu.h"
#include "spc.h"
#include "dma.h"
#include "perf.h"
#include "ppu.h"
#include "cart.h"
#include "input.h"
#include "statehandler.h"

/* ThumbySNES: float not double. M33 has a single-precision FPU; doubles
 * go through softfloat at ~50 cycles per op. snes_runCycle hits this
 * 178K times per frame — switching to float saves a measurable chunk
 * of per-cycle overhead. */
static const float apuCyclesPerMaster = (32040.0f * 32.0f) / (1364.0f * 262.0f * 60.0f);
static const float apuCyclesPerMasterPal = (32040.0f * 32.0f) / (1364.0f * 312.0f * 50.0f);

static void snes_runCycle(Snes* snes);
static void snes_catchupApu(Snes* snes);
static void snes_doAutoJoypad(Snes* snes);
static uint8_t snes_readReg(Snes* snes, uint16_t adr);
static void snes_writeReg(Snes* snes, uint16_t adr, uint8_t val);
static uint8_t snes_rread(Snes* snes, uint32_t adr); // wrapped by read, to set open bus
static int snes_getAccessTime(Snes* snes, uint32_t adr);

Snes* snes_init(void) {
  Snes* snes = malloc(sizeof(Snes));
  snes->cpu = cpu_init(snes, snes_cpuRead, snes_cpuWrite, snes_cpuIdle);
  snes->apu = apu_init(snes);
  snes->dma = dma_init(snes);
  snes->ppu = ppu_init(snes);
  snes->cart = cart_init(snes);
  snes->input1 = input_init(snes);
  snes->input2 = input_init(snes);
  snes->palTiming = false;
  return snes;
}

void snes_free(Snes* snes) {
  cpu_free(snes->cpu);
  apu_free(snes->apu);
  dma_free(snes->dma);
  ppu_free(snes->ppu);
  cart_free(snes->cart);
  input_free(snes->input1);
  input_free(snes->input2);
  free(snes);
}

/* ThumbySNES: populate the block-based read map after ROM load / reset.
 * Each 4 KB block gets either a direct pointer (ROM, WRAM) or a
 * sentinel (SNES_MAP_SPECIAL) for register regions. */
static void snes_buildReadMap(Snes* snes) {
  Cart *cart = snes->cart;
  for (int block = 0; block < SNES_MAP_BLOCKS; block++) {
    uint8_t bank = block >> 4;          /* high 8 bits */
    uint16_t adr_base = (block & 0xf) << 12;  /* low 16 bits, aligned to 4K */
    uint8_t speed = 8;  /* default */

    /* Banks 7E-7F: WRAM (128 KB, direct map) */
    if (bank == 0x7e || bank == 0x7f) {
      snes->readMap[block] = snes->ram + ((bank & 1) << 16) + adr_base;
      snes->readMapSpeed[block] = 8;
      continue;
    }

    /* Banks 00-3F and 80-BF: system area in low half, cart in high */
    if (bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
      if (adr_base < 0x2000) {
        /* WRAM mirror */
        snes->readMap[block] = snes->ram + adr_base;
        speed = 8;
      } else if (adr_base >= 0x2000 && adr_base < 0x5000) {
        /* PPU ($2100), CPU regs ($4200), DMA ($4300), input ($4016) */
        snes->readMap[block] = SNES_MAP_SPECIAL;
        speed = (adr_base < 0x4000) ? 6 : (adr_base < 0x4200 ? 12 : 6);
      } else if (adr_base >= 0x6000 && adr_base < 0x8000) {
        /* Cart SRAM region (LoROM) or open bus — let slow path handle */
        snes->readMap[block] = SNES_MAP_SPECIAL;
        speed = 8;
      } else if (adr_base >= 0x8000) {
        /* ROM */
        if (cart && cart->rom && cart->romSize > 0) {
          if (cart->type == 1) { /* LoROM */
            uint32_t off = (((bank & 0x7f) << 15) | (adr_base & 0x7fff)) & (cart->romSize - 1);
            snes->readMap[block] = cart->rom + off;
          } else if (cart->type == 2 || cart->type == 3) { /* HiROM / ExHiROM */
            uint32_t off = (((bank & 0x3f) << 16) | adr_base) & (cart->romSize - 1);
            snes->readMap[block] = cart->rom + off;
          } else {
            snes->readMap[block] = SNES_MAP_SPECIAL;
          }
        } else {
          snes->readMap[block] = SNES_MAP_SPECIAL;
        }
        speed = (snes->fastMem && bank >= 0x80) ? 6 : 8;
      } else {
        /* 0x5000-0x5FFF: open bus area */
        snes->readMap[block] = SNES_MAP_SPECIAL;
        speed = 6;
      }
      snes->readMapSpeed[block] = speed;
      continue;
    }

    /* Banks 40-7D, C0-FF: all ROM */
    if (cart && cart->rom && cart->romSize > 0) {
      if (cart->type == 1) { /* LoROM */
        uint32_t off = (((bank & 0x7f) << 15) | (adr_base & 0x7fff)) & (cart->romSize - 1);
        snes->readMap[block] = cart->rom + off;
      } else if (cart->type == 2 || cart->type == 3) { /* HiROM */
        uint32_t off = (((bank & 0x3f) << 16) | adr_base) & (cart->romSize - 1);
        snes->readMap[block] = cart->rom + off;
      } else {
        snes->readMap[block] = SNES_MAP_SPECIAL;
      }
    } else {
      snes->readMap[block] = SNES_MAP_SPECIAL;
    }
    snes->readMapSpeed[block] = (snes->fastMem && bank >= 0x80) ? 6 : 8;
  }
}

void snes_reset(Snes* snes, bool hard) {
  cpu_reset(snes->cpu, hard);
  apu_reset(snes->apu);
  dma_reset(snes->dma);
  ppu_reset(snes->ppu);
  input_reset(snes->input1);
  input_reset(snes->input2);
  cart_reset(snes->cart);
  if(hard) memset(snes->ram, 0, sizeof(snes->ram));
  snes->ramAdr = 0;
  snes->hPos = 0;
  snes->vPos = 0;
  snes->frames = 0;
  snes->cycles = 0;
  snes->syncCycle = 0;
  snes->apuCatchupCycles = 0.0;
  snes->hIrqEnabled = false;
  snes->vIrqEnabled = false;
  snes->nmiEnabled = false;
  snes->hTimer = 0x1ff;
  snes->vTimer = 0x1ff;
  snes->inNmi = false;
  snes->irqCondition = false;
  snes->inIrq = false;
  snes->inVblank = false;
  memset(snes->portAutoRead, 0, sizeof(snes->portAutoRead));
  snes->autoJoyRead = false;
  snes->autoJoyTimer = 0;
  snes->ppuLatch = false;
  snes->multiplyA = 0xff;
  snes->multiplyResult = 0xfe01;
  snes->divideA = 0xffff;
  snes->divideResult = 0x101;
  snes->fastMem = false;
  snes->openBus = 0;
  snes->pendingCycles = 0;
  snes_buildReadMap(snes);
}

void snes_handleState(Snes* snes, StateHandler* sh) {
  sh_handleBools(sh,
    &snes->palTiming, &snes->hIrqEnabled, &snes->vIrqEnabled, &snes->nmiEnabled, &snes->inNmi, &snes->irqCondition,
    &snes->inIrq, &snes->inVblank, &snes->autoJoyRead, &snes->ppuLatch, &snes->fastMem, NULL
  );
  sh_handleBytes(sh, &snes->multiplyA, &snes->openBus, NULL);
  sh_handleWords(sh,
    &snes->hPos, &snes->vPos, &snes->hTimer, &snes->vTimer,
    &snes->portAutoRead[0], &snes->portAutoRead[1], &snes->portAutoRead[2], &snes->portAutoRead[3],
    &snes->autoJoyTimer, &snes->multiplyResult, &snes->divideA, &snes->divideResult, NULL
  );
  sh_handleInts(sh, &snes->ramAdr, &snes->frames, NULL);
  sh_handleLongLongs(sh, &snes->cycles, &snes->syncCycle, NULL);
  sh_handleDoubles(sh, &snes->apuCatchupCycles, NULL);
  sh_handleByteArray(sh, snes->ram, 0x20000);
  // components
  cpu_handleState(snes->cpu, sh);
  dma_handleState(snes->dma, sh);
  ppu_handleState(snes->ppu, sh);
  apu_handleState(snes->apu, sh);
  input_handleState(snes->input1, sh);
  input_handleState(snes->input2, sh);
  cart_handleState(snes->cart, sh);
}

#if __has_include("cpu_asm.h")
#include "cpu_asm.h"
#define HAS_CPU_ASM 1
#else
#define HAS_CPU_ASM 0
#endif

void snes_runFrame(Snes* snes) {
  // run until we are starting a new frame (leaving vblank)
  while(snes->inVblank) {
    cpu_runOpcode(snes->cpu);
  }
  // then run until we are at vblank, or we end up at next frame
  uint32_t frame = snes->frames;
#if HAS_CPU_ASM && defined(THUMBYSNES_DUAL_CORE) && THUMBYSNES_DUAL_CORE
  while(!snes->inVblank && frame == snes->frames) {
    cpu_runBatchAsm(snes->cpu, 64);
  }
#else
  while(!snes->inVblank && frame == snes->frames) {
    for (int _b = 0; _b < 8; _b++) {
      cpu_runOpcode(snes->cpu);
      if (snes->inVblank || snes->frames != frame) break;
    }
  }
#endif
  snes_catchupApu(snes);
}

/* ThumbySNES batched cycle scheduler.
 *
 * snes_runCycle needs to run every cycle at exactly the hPos values where
 * events fire — hPos=0 (line start), 16 (HDMA init), 512 (ppu_runLine),
 * 1104 (HDMA run), hTimer*4 (if hIrq enabled), and at the wrap point.
 * Between those, the per-cycle work is just: IRQ condition (constant
 * when hPos is between trigger points), autoJoyTimer-=2, hPos+=2.
 *
 * At a typical content-frame budget we see 1-2M per-cycle calls on core
 * 0. Batching the "boring" stretches by jumping straight to the next
 * event cuts the slow-path call count by ~200× (from ~700K to ~3K per
 * frame) at the cost of one "find next event" check per batch. */
LAKESNES_HOT void snes_runCycles(Snes* snes, int cycles) {
  if(snes->hPos + cycles >= 536 && snes->hPos < 536) {
    // if we go past 536, add 40 cycles for dram refersh
    cycles += 40;
  }
  while (cycles > 0) {
    int hp = snes->hPos;

    /* If hp is AT an event (or in wrap territory) run the per-cycle
     * slow path so its event handlers and post-increment wrap check
     * fire normally. hp==0 triggers vblank/NMI transitions, hp==16
     * HDMA init, hp==512 ppu_runLine, hp==1104 HDMA run, and the
     * last stretch before the line wrap (1356..1362) needs per-cycle
     * treatment because the wrap check uses the post-increment hPos
     * value (1360 NTSC special / 1364 NTSC / 1368 PAL). */
    bool atEvent =
      (hp == 0 || hp == 16 || hp == 512 || hp == 1104 || hp >= 1356 ||
       (snes->hIrqEnabled && hp == snes->hTimer * 4));
    if (atEvent) {
      snes_runCycle(snes);
      cycles -= 2;
      continue;
    }

    /* Find the next event hPos after `hp`. Batch stops exactly there
     * so the next loop iteration takes the slow path. */
    int next;
    if      (hp <   16) next = 16;
    else if (hp <  512) next = 512;
    else if (hp < 1104) next = 1104;
    else                next = 1356;
    if (snes->hIrqEnabled) {
      int irqHp = snes->hTimer * 4;
      if (irqHp > hp && irqHp < next) next = irqHp;
    }

    int step = next - hp;
    if (step < 2) {
      snes_runCycle(snes);
      cycles -= 2;
      continue;
    }
    if (step > cycles) step = cycles;
    /* step is always even (hp and next are both even) and >= 2. */

    /* Batch-advance counters. Same arithmetic as `step / 2` snes_runCycle
     * calls, but we do it once. */
    snes->cycles += step;
    snes->hPos   += step;
    cycles       -= step;

#if !(defined(THUMBYSNES_DUAL_CORE) && THUMBYSNES_DUAL_CORE)
    /* Single-core: keep APU catchup accumulator in sync. */
    snes->apuCatchupCycles +=
      (snes->palTiming ? apuCyclesPerMasterPal : apuCyclesPerMaster) * (float)step;
#endif

    /* autoJoyTimer batches by `step`. */
    if (snes->autoJoyTimer > 0) {
      snes->autoJoyTimer = (snes->autoJoyTimer > step)
                         ? (uint16_t)(snes->autoJoyTimer - step)
                         : 0;
    }

    /* IRQ condition is a function of (hPos, vPos, timers, enable bits).
     * During a batch only hPos changes — and we stopped the batch before
     * reaching hTimer*4 (if hIrq enabled), so the condition's hPos-
     * dependence can't transition mid-batch. Re-evaluate at batch end
     * and apply the false→true edge check. */
    bool condition = (
      (snes->vIrqEnabled || snes->hIrqEnabled) &&
      (snes->vPos == snes->vTimer || !snes->vIrqEnabled) &&
      (snes->hPos == snes->hTimer * 4 || !snes->hIrqEnabled)
    );
    if (!snes->irqCondition && condition) {
      snes->inIrq = true;
      cpu_setIrq(snes->cpu, true);
    }
    snes->irqCondition = condition;
  }
}

/* Non-inline version for external callers (cpu_asm.c).
 * The static inline in snes.h is used by cpu.c. */
void snes_flushCycles_extern(Snes* snes) {
  snes_flushCycles(snes);
}

void snes_syncCycles(Snes* snes, bool start, int syncCycles) {
  if(start) {
    snes->syncCycle = snes->cycles;
    int count = syncCycles - (snes->cycles % syncCycles);
    snes_runCycles(snes, count);
  } else {
    int count = syncCycles - ((snes->cycles - snes->syncCycle) % syncCycles);
    snes_runCycles(snes, count);
  }
}

LAKESNES_HOT static void snes_runCycle(Snes* snes) {
#if !(defined(THUMBYSNES_DUAL_CORE) && THUMBYSNES_DUAL_CORE)
  /* Dual-core build: APU runs free on core 1, no per-master-cycle
   * accumulator update needed. Save the FPU multiply + add per tick
   * (this runs millions of times per frame). */
  snes->apuCatchupCycles += (snes->palTiming ? apuCyclesPerMasterPal : apuCyclesPerMaster) * 2.0f;
#endif
  snes->cycles += 2;
  // check for h/v timer irq's
  bool condition = (
    (snes->vIrqEnabled || snes->hIrqEnabled) &&
    (snes->vPos == snes->vTimer || !snes->vIrqEnabled) &&
    (snes->hPos == snes->hTimer * 4 || !snes->hIrqEnabled)
  );
  if(!snes->irqCondition && condition) {
    snes->inIrq = true;
    cpu_setIrq(snes->cpu, true);
  }
  snes->irqCondition = condition;
  // handle positional stuff
  if(snes->hPos == 0) {
    // end of hblank, do most vPos-tests
    bool startingVblank = false;
    if(snes->vPos == 0) {
      // end of vblank
      snes->inVblank = false;
      snes->inNmi = false;
      ppu_handleFrameStart(snes->ppu);
    } else if(snes->vPos == 225) {
      // ask the ppu if we start vblank now or at vPos 240 (overscan)
      startingVblank = !ppu_checkOverscan(snes->ppu);
    } else if(snes->vPos == 240){
      // if we are not yet in vblank, we had an overscan frame, set startingVblank
      if(!snes->inVblank) startingVblank = true;
    }
    if(startingVblank) {
      // if we are starting vblank
      ppu_handleVblank(snes->ppu);
      snes->inVblank = true;
      snes->inNmi = true;
      if(snes->autoJoyRead) {
        // TODO: this starts a little after start of vblank
        snes->autoJoyTimer = 4224;
        snes_doAutoJoypad(snes);
      }
      if(snes->nmiEnabled) {
        cpu_nmi(snes->cpu);
      }
    }
  } else if(snes->hPos == 16) {
    if(snes->vPos == 0) snes->dma->hdmaInitRequested = true;
  } else if(snes->hPos == 512) {
    // render the line halfway of the screen for better compatibility
    if(!snes->inVblank && snes->vPos > 0) {
#if defined(THUMBYSNES_DUAL_CORE) && THUMBYSNES_DUAL_CORE
      /* PPU-CPU pipeline: wait for core 1 to finish the previous line,
       * then dispatch this line. Core 0 ran CPU opcodes between
       * hPos=512 of the previous line and now — that's the pipeline
       * overlap window. The wait here is for the "tail" of the
       * previous render; it's short when core 1 keeps up, and on
       * heavy lines the wait is the price we pay.
       *
       * We ALWAYS dispatch to core 1 (never fall back to core 0
       * render) because the scanline blend callback uses shared state
       * (s_blend_a) — calling it from both cores races. */
      extern volatile int s_ppu_pipeline_line;
      extern volatile int s_ppu_pipeline_done;
      while (!s_ppu_pipeline_done) { /* spin — previous line finishing */ }
      s_ppu_pipeline_done = 0;
      s_ppu_pipeline_line = snes->vPos;
#else
      ppu_runLine(snes->ppu, snes->vPos);
#endif
    }
  } else if(snes->hPos == 1104) {
    if(!snes->inVblank) snes->dma->hdmaRunRequested = true;
  }
  // handle autoJoyRead-timer
  if(snes->autoJoyTimer > 0) snes->autoJoyTimer -= 2;
  // increment position
  snes->hPos += 2;
  if(!snes->palTiming) {
    // line 240 of odd frame with no interlace is 4 cycles shorter
    if((snes->hPos == 1360 && snes->vPos == 240 && !snes->ppu->evenFrame && !snes->ppu->frameInterlace) || snes->hPos == 1364) {
      snes->hPos = 0;
      snes->vPos++;
      // even interlace frame is 263 lines
      if((snes->vPos == 262 && (!snes->ppu->frameInterlace || !snes->ppu->evenFrame)) || snes->vPos == 263) {
        snes->vPos = 0;
        snes->frames++;
      }
    }
  } else {
    // line 311 of odd frame with interlace is 4 cycles longer
    if((snes->hPos == 1364 && (snes->vPos != 311 || snes->ppu->evenFrame || !snes->ppu->frameInterlace)) || snes->hPos == 1368) {
      snes->hPos = 0;
      snes->vPos++;
      // even interlace frame is 313 lines
      if((snes->vPos == 312 && (!snes->ppu->frameInterlace || !snes->ppu->evenFrame)) || snes->vPos == 313) {
        snes->vPos = 0;
        snes->frames++;
      }
    }
  }
}

static void snes_catchupApu(Snes* snes) {
#if defined(THUMBYSNES_DUAL_CORE) && THUMBYSNES_DUAL_CORE
  /* Dual-core build: SPC700 + DSP run continuously on core 1. CPU-side
   * APU port accesses only need to read the shared inPorts/outPorts
   * bytes, which are atomic on M33. No explicit catchup needed — the
   * SPC has been running all along. The accumulator field is unused
   * in this configuration. */
  (void)snes;
  return;
#else
  /* ThumbySNES: run SPC opcodes while the accumulator has any work in
   * it, not just the integer portion. Upstream casts the accumulator to
   * int before passing to apu_runCycles — on tight CPU polling loops
   * the fractional cycles get perpetually lost, leaving the SPC
   * chronically one step behind the CPU. This manifests as SMW's sound
   * driver acknowledging one byte behind what the CPU just sent, so
   * the game hangs in its handshake loop forever.
   *
   * Running while accumulator > 0 keeps the long-term ratio identical
   * to upstream but pays out the fractional debt on every call instead
   * of accumulating it indefinitely. */
  while (snes->apuCatchupCycles > 0.0f) {
    uint32_t before = snes->apu->cycles;
    spc_runOpcode(snes->apu->spc);
    uint32_t ran = snes->apu->cycles - before;
    if (ran == 0) break; /* safety — should never happen */
    snes->apuCatchupCycles -= (float) ran;
  }
#endif
}

static void snes_doAutoJoypad(Snes* snes) {
  memset(snes->portAutoRead, 0, sizeof(snes->portAutoRead));
  // latch controllers
  input_latch(snes->input1, true);
  input_latch(snes->input2, true);
  input_latch(snes->input1, false);
  input_latch(snes->input2, false);
  for(int i = 0; i < 16; i++) {
    uint8_t val = input_read(snes->input1);
    snes->portAutoRead[0] |= ((val & 1) << (15 - i));
    snes->portAutoRead[2] |= (((val >> 1) & 1) << (15 - i));
    val = input_read(snes->input2);
    snes->portAutoRead[1] |= ((val & 1) << (15 - i));
    snes->portAutoRead[3] |= (((val >> 1) & 1) << (15 - i));
  }
}

LAKESNES_HOT uint8_t snes_readBBus(Snes* snes, uint8_t adr) {
  if(adr < 0x40) {
    return ppu_read(snes->ppu, adr);
  }
  if(adr < 0x80) {
    snes_catchupApu(snes); // catch up the apu before reading
    return snes->apu->outPorts[adr & 0x3];
  }
  if(adr == 0x80) {
    uint8_t ret = snes->ram[snes->ramAdr++];
    snes->ramAdr &= 0x1ffff;
    return ret;
  }
  return snes->openBus;
}

LAKESNES_HOT void snes_writeBBus(Snes* snes, uint8_t adr, uint8_t val) {
  if(adr < 0x40) {
    ppu_write(snes->ppu, adr, val);
    return;
  }
  if(adr < 0x80) {
    snes_catchupApu(snes); // catch up the apu before writing
    snes->apu->inPorts[adr & 0x3] = val;
    return;
  }
  switch(adr) {
    case 0x80: {
      snes->ram[snes->ramAdr++] = val;
      snes->ramAdr &= 0x1ffff;
      break;
    }
    case 0x81: {
      snes->ramAdr = (snes->ramAdr & 0x1ff00) | val;
      break;
    }
    case 0x82: {
      snes->ramAdr = (snes->ramAdr & 0x100ff) | (val << 8);
      break;
    }
    case 0x83: {
      snes->ramAdr = (snes->ramAdr & 0x0ffff) | ((val & 1) << 16);
      break;
    }
  }
}

static uint8_t snes_readReg(Snes* snes, uint16_t adr) {
  switch(adr) {
    case 0x4210: {
      uint8_t val = 0x2; // CPU version (4 bit)
      val |= snes->inNmi << 7;
      snes->inNmi = false;
      return val | (snes->openBus & 0x70);
    }
    case 0x4211: {
      uint8_t val = snes->inIrq << 7;
      snes->inIrq = false;
      cpu_setIrq(snes->cpu, false);
      return val | (snes->openBus & 0x7f);
    }
    case 0x4212: {
      uint8_t val = (snes->autoJoyTimer > 0);
      val |= (snes->hPos < 4 || snes->hPos >= 1096) << 6;
      val |= snes->inVblank << 7;
      return val | (snes->openBus & 0x3e);
    }
    case 0x4213: {
      return snes->ppuLatch << 7; // IO-port
    }
    case 0x4214: {
      return snes->divideResult & 0xff;
    }
    case 0x4215: {
      return snes->divideResult >> 8;
    }
    case 0x4216: {
      return snes->multiplyResult & 0xff;
    }
    case 0x4217: {
      return snes->multiplyResult >> 8;
    }
    case 0x4218:
    case 0x421a:
    case 0x421c:
    case 0x421e: {
      return snes->portAutoRead[(adr - 0x4218) / 2] & 0xff;
    }
    case 0x4219:
    case 0x421b:
    case 0x421d:
    case 0x421f: {
      return snes->portAutoRead[(adr - 0x4219) / 2] >> 8;
    }
    default: {
      return snes->openBus;
    }
  }
}

static void snes_writeReg(Snes* snes, uint16_t adr, uint8_t val) {
  switch(adr) {
    case 0x4200: {
      snes->autoJoyRead = val & 0x1;
      if(!snes->autoJoyRead) snes->autoJoyTimer = 0;
      snes->hIrqEnabled = val & 0x10;
      snes->vIrqEnabled = val & 0x20;
      if(!snes->hIrqEnabled && !snes->vIrqEnabled) {
        snes->inIrq = false;
        cpu_setIrq(snes->cpu, false);
      }
      // if nmi is enabled while inNmi is still set, immediately generate nmi
      if(!snes->nmiEnabled && (val & 0x80) && snes->inNmi) {
        cpu_nmi(snes->cpu);
      }
      snes->nmiEnabled = val & 0x80;
      break;
    }
    case 0x4201: {
      if(!(val & 0x80) && snes->ppuLatch) {
        // latch the ppu
        ppu_read(snes->ppu, 0x37);
      }
      snes->ppuLatch = val & 0x80;
      break;
    }
    case 0x4202: {
      snes->multiplyA = val;
      break;
    }
    case 0x4203: {
      snes->multiplyResult = snes->multiplyA * val;
      break;
    }
    case 0x4204: {
      snes->divideA = (snes->divideA & 0xff00) | val;
      break;
    }
    case 0x4205: {
      snes->divideA = (snes->divideA & 0x00ff) | (val << 8);
      break;
    }
    case 0x4206: {
      if(val == 0) {
        snes->divideResult = 0xffff;
        snes->multiplyResult = snes->divideA;
      } else {
        snes->divideResult = snes->divideA / val;
        snes->multiplyResult = snes->divideA % val;
      }
      break;
    }
    case 0x4207: {
      snes->hTimer = (snes->hTimer & 0x100) | val;
      break;
    }
    case 0x4208: {
      snes->hTimer = (snes->hTimer & 0x0ff) | ((val & 1) << 8);
      break;
    }
    case 0x4209: {
      snes->vTimer = (snes->vTimer & 0x100) | val;
      break;
    }
    case 0x420a: {
      snes->vTimer = (snes->vTimer & 0x0ff) | ((val & 1) << 8);
      break;
    }
    case 0x420b: {
      dma_startDma(snes->dma, val, false);
      break;
    }
    case 0x420c: {
      dma_startDma(snes->dma, val, true);
      break;
    }
    case 0x420d: {
      bool newFast = val & 0x1;
      if (newFast != snes->fastMem) {
        snes->fastMem = newFast;
        snes_buildReadMap(snes); /* speed entries change for banks 80+ */
      }
      break;
    }
    default: {
      break;
    }
  }
}

static uint8_t snes_rread(Snes* snes, uint32_t adr) {
  uint8_t bank = adr >> 16;
  adr &= 0xffff;
  if(bank == 0x7e || bank == 0x7f) {
    return snes->ram[((bank & 1) << 16) | adr]; // ram
  }
  if(bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
    if(adr < 0x2000) {
      return snes->ram[adr]; // ram mirror
    }
    if(adr >= 0x2100 && adr < 0x2200) {
      return snes_readBBus(snes, adr & 0xff); // B-bus
    }
    if(adr == 0x4016) {
      return input_read(snes->input1) | (snes->openBus & 0xfc);
    }
    if(adr == 0x4017) {
      return input_read(snes->input2) | (snes->openBus & 0xe0) | 0x1c;
    }
    if(adr >= 0x4200 && adr < 0x4220) {
      return snes_readReg(snes, adr); // internal registers
    }
    if(adr >= 0x4300 && adr < 0x4380) {
      return dma_read(snes->dma, adr); // dma registers
    }
  }
  // read from cart
  return cart_read(snes->cart, bank, adr);
}

LAKESNES_HOT void snes_write(Snes* snes, uint32_t adr, uint8_t val) {
  snes->openBus = val;
  uint8_t bank = adr >> 16;
  adr &= 0xffff;
  if(bank == 0x7e || bank == 0x7f) {
    snes->ram[((bank & 1) << 16) | adr] = val; // ram
  }
  if(bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) {
    if(adr < 0x2000) {
      snes->ram[adr] = val; // ram mirror
    }
    if(adr >= 0x2100 && adr < 0x2200) {
      snes_writeBBus(snes, adr & 0xff, val); // B-bus
    }
    if(adr == 0x4016) {
      input_latch(snes->input1, val & 1); // input latch
      input_latch(snes->input2, val & 1);
    }
    if(adr >= 0x4200 && adr < 0x4220) {
      snes_writeReg(snes, adr, val); // internal registers
    }
    if(adr >= 0x4300 && adr < 0x4380) {
      dma_write(snes->dma, adr, val); // dma registers
    }
  }
  // write to cart
  cart_write(snes->cart, bank, adr, val);
}

static int snes_getAccessTime(Snes* snes, uint32_t adr) {
  uint8_t bank = adr >> 16;
  adr &= 0xffff;
  if((bank < 0x40 || (bank >= 0x80 && bank < 0xc0)) && adr < 0x8000) {
    // 00-3f,80-bf:0-7fff
    if(adr < 0x2000 || adr >= 0x6000) return 8; // 0-1fff, 6000-7fff
    if(adr < 0x4000 || adr >= 0x4200) return 6; // 2000-3fff, 4200-5fff
    return 12; // 4000-41ff
  }
  // 40-7f,co-ff:0000-ffff, 00-3f,80-bf:8000-ffff
  return (snes->fastMem && bank >= 0x80) ? 6 : 8; // depends on setting in banks 80+
}

LAKESNES_HOT uint8_t snes_read(Snes* snes, uint32_t adr) {
  uint8_t val = snes_rread(snes, adr);
  snes->openBus = val;
  return val;
}

void snes_cpuIdle(void* mem, bool waiting) {
  Snes* snes = (Snes*) mem;
  dma_handleDma(snes->dma, 6);
  snes_runCyclesFast(snes, 6);
}

LAKESNES_HOT uint8_t snes_cpuRead(void* mem, uint32_t adr) {
  Snes* snes = (Snes*) mem;
#if defined(THUMBYSNES_DIRECT_CPU_CALLS) && THUMBYSNES_DIRECT_CPU_CALLS
  /* Block-map fast path (snes9x2002 architecture). One array lookup
   * replaces the 6-8 if-chain in snes_rread + snes_getAccessTime.
   * Covers ~95% of reads (ROM + WRAM) in 4 instructions.
   * Cycles accumulated — flushed once per opcode in cpu_runOpcode.
   * DMA check kept per-access (no-op 99% of the time). */
  {
    uint32_t block = (adr >> SNES_MAP_SHIFT) & (SNES_MAP_BLOCKS - 1);
    uint8_t *ptr = snes->readMap[block];
    if (ptr >= SNES_MAP_LAST) {
      int speed = snes->readMapSpeed[block];
      dma_handleDma(snes->dma, speed);
      snes->pendingCycles += speed;
      snes->openBus = ptr[adr & SNES_MAP_MASK];
      return snes->openBus;
    }
  }
#endif
  /* Slow path — PPU, APU, input, registers, DMA, unmapped regions. */
  int cycles = snes_getAccessTime(snes, adr);
  dma_handleDma(snes->dma, cycles);
  snes_runCyclesFast(snes, cycles);
  return snes_read(snes, adr);
}

LAKESNES_HOT void snes_cpuWrite(void* mem, uint32_t adr, uint8_t val) {
  Snes* snes = (Snes*) mem;
#if defined(THUMBYSNES_DIRECT_CPU_CALLS) && THUMBYSNES_DIRECT_CPU_CALLS
  /* Block-map fast path for WRAM writes. ROM writes are no-ops (read-
   * only), so we only fast-path if the block points into snes->ram.
   * We detect this by checking if ptr falls within the ram array. */
  {
    uint32_t block = (adr >> SNES_MAP_SHIFT) & (SNES_MAP_BLOCKS - 1);
    uint8_t *ptr = snes->readMap[block];
    if (ptr >= SNES_MAP_LAST) {
      /* Check if this block is WRAM (pointer falls within snes->ram).
       * ROM blocks also pass ptr >= SNES_MAP_LAST but are read-only. */
      ptrdiff_t off = ptr - snes->ram;
      if (off >= 0 && off < 0x20000) {
        int speed = snes->readMapSpeed[block];
        dma_handleDma(snes->dma, speed);
        snes->pendingCycles += speed;
        snes->openBus = val;
        ptr[adr & SNES_MAP_MASK] = val;
        return;
      }
    }
  }
#endif
  /* Slow path — PPU writes, cart SRAM, register writes, etc. */
  int cycles = snes_getAccessTime(snes, adr);
  dma_handleDma(snes->dma, cycles);
  snes_runCyclesFast(snes, cycles);
  snes_write(snes, adr, val);
}

// debugging

void snes_runCpuCycle(Snes* snes) {
  cpu_runOpcode(snes->cpu);
}

void snes_runSpcCycle(Snes* snes) {
  // TODO: apu catchup is not aware of this, SPC runs extra cycle(s)
  spc_runOpcode(snes->apu->spc);
}
