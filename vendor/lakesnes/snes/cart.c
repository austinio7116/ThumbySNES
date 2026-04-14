
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "cart.h"
#include "snes.h"
#include "statehandler.h"

static uint8_t cart_readLorom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeLorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);
static uint8_t cart_readHirom(Cart* cart, uint8_t bank, uint16_t adr);
static uint8_t cart_readExHirom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeHirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);

Cart* cart_init(Snes* snes) {
  Cart* cart = malloc(sizeof(Cart));
  cart->snes = snes;
  cart->type = 0;
  cart->_romIsExternal = 0;
  cart->rom = NULL;
  cart->romSize = 0;
  cart->ram = NULL;
  cart->ramSize = 0;
  return cart;
}

void cart_free(Cart* cart) {
  /* External (XIP) ROM lives in flash — never free it. */
  if(cart->rom != NULL && !cart->_romIsExternal) free(cart->rom);
  if(cart->ram != NULL) free(cart->ram);
  free(cart);
}

void cart_reset(Cart* cart) {
  // do not reset ram, assumed to be battery backed
}

bool cart_handleTypeState(Cart* cart, StateHandler* sh) {
  // when loading, return if values match
  if(sh->saving) {
    sh_handleBytes(sh, &cart->type, NULL);
    sh_handleInts(sh, &cart->romSize, &cart->ramSize, NULL);
    return true;
  } else {
    uint8_t type = 0;
    uint32_t romSize = 0;
    uint32_t ramSize = 0;
    sh_handleBytes(sh, &type, NULL);
    sh_handleInts(sh, &romSize, &ramSize, NULL);
    return !(type != cart->type || romSize != cart->romSize || ramSize != cart->ramSize);
  }
}

void cart_handleState(Cart* cart, StateHandler* sh) {
  if(cart->ram != NULL) sh_handleByteArray(sh, cart->ram, cart->ramSize);
}

/* ThumbySNES: when nonzero, the ROM pointer passed to cart_load is
 * already mapped (XIP flash on device, mmap on host) — skip the
 * malloc + memcpy and point cart->rom at it directly. The lifetime
 * is the caller's problem (must outlive the emulator session). */
uint8_t thumbysnes_cart_xip = 0;

void cart_load(Cart* cart, int type, uint8_t* rom, int romSize, int ramSize) {
  cart->type = type;
  if(cart->rom != NULL && !cart->_romIsExternal) free(cart->rom);
  if(cart->ram != NULL) free(cart->ram);
  if(thumbysnes_cart_xip) {
    cart->rom = rom;             /* zero-copy */
    cart->_romIsExternal = 1;
  } else {
    cart->rom = malloc(romSize);
    cart->_romIsExternal = 0;
    memcpy(cart->rom, rom, romSize);
  }
  cart->romSize = romSize;
  if(ramSize > 0) {
    cart->ram = malloc(ramSize);
    memset(cart->ram, 0, ramSize);
  } else {
    cart->ram = NULL;
  }
  cart->ramSize = ramSize;

#if CART_ROM_CACHE_SIZE > 0
  /* Cache the first CART_ROM_CACHE_SIZE bytes of ROM in SRAM. For
   * LoROM this is bank 00 adr 0x8000-0xBFFF (main game code). For
   * HiROM this is the first 16 KB of the mapped ROM. The exact hot
   * region varies per game, but bank-0 code is almost always the
   * highest-frequency access pattern. */
  {
    int cacheLen = romSize < CART_ROM_CACHE_SIZE ? romSize : CART_ROM_CACHE_SIZE;
    memcpy(cart->romCache, cart->rom, cacheLen);
    if (cacheLen < CART_ROM_CACHE_SIZE)
      memset(cart->romCache + cacheLen, 0, CART_ROM_CACHE_SIZE - cacheLen);
    cart->romCacheOffset = 0;  /* cache maps ROM[0..CART_ROM_CACHE_SIZE) */
  }
#endif
}

bool cart_handleBattery(Cart* cart, bool save, uint8_t* data, int* size) {
  if(save) {
    *size = cart->ramSize;
    if(data == NULL) return true;
    // assumes data is correct size
    if(cart->ram != NULL) memcpy(data, cart->ram, cart->ramSize);
    return true;
  } else {
    if(*size != cart->ramSize) return false;
    if(cart->ram != NULL) memcpy(cart->ram, data, cart->ramSize);
    return true;
  }
}

uint8_t cart_read(Cart* cart, uint8_t bank, uint16_t adr) {
  switch(cart->type) {
    case 0: return cart->snes->openBus;
    case 1: return cart_readLorom(cart, bank, adr);
    case 2: return cart_readHirom(cart, bank, adr);
    case 3: return cart_readExHirom(cart, bank, adr);
  }
  return cart->snes->openBus;
}

void cart_write(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  switch(cart->type) {
    case 0: break;
    case 1: cart_writeLorom(cart, bank, adr, val); break;
    case 2: cart_writeHirom(cart, bank, adr, val); break;
    case 3: cart_writeHirom(cart, bank, adr, val); break;
  }
}

static uint8_t cart_readLorom(Cart* cart, uint8_t bank, uint16_t adr) {
  if(((bank >= 0x70 && bank < 0x7e) || bank >= 0xf0) && adr < 0x8000 && cart->ramSize > 0) {
    return cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)];
  }
  bank &= 0x7f;
  if(adr >= 0x8000 || bank >= 0x40) {
    uint32_t offset = ((bank << 15) | (adr & 0x7fff)) & (cart->romSize - 1);
#if CART_ROM_CACHE_SIZE > 0
    uint32_t cacheIdx = offset - cart->romCacheOffset;
    if (cacheIdx < CART_ROM_CACHE_SIZE) return cart->romCache[cacheIdx];
#endif
    return cart->rom[offset];
  }
  return cart->snes->openBus;
}

static void cart_writeLorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  if(((bank >= 0x70 && bank < 0x7e) || bank > 0xf0) && adr < 0x8000 && cart->ramSize > 0) {
    // banks 70-7e and f0-ff, adr 0000-7fff
    cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)] = val;
  }
}

static uint8_t cart_readHirom(Cart* cart, uint8_t bank, uint16_t adr) {
  bank &= 0x7f;
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    return cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)];
  }
  if(adr >= 0x8000 || bank >= 0x40) {
    uint32_t offset = (((bank & 0x3f) << 16) | adr) & (cart->romSize - 1);
#if CART_ROM_CACHE_SIZE > 0
    uint32_t cacheIdx = offset - cart->romCacheOffset;
    if (cacheIdx < CART_ROM_CACHE_SIZE) return cart->romCache[cacheIdx];
#endif
    return cart->rom[offset];
  }
  return cart->snes->openBus;
}

static uint8_t cart_readExHirom(Cart* cart, uint8_t bank, uint16_t adr) {
  if((bank & 0x7f) < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    return cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)];
  }
  bool secondHalf = bank < 0x80;
  bank &= 0x7f;
  if(adr >= 0x8000 || bank >= 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[(((bank & 0x3f) << 16) | (secondHalf ? 0x400000 : 0) | adr) & (cart->romSize - 1)];
  }
  return cart->snes->openBus;
}

static void cart_writeHirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  bank &= 0x7f;
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)] = val;
  }
}
