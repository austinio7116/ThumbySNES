
#ifndef CART_H
#define CART_H

#include <stdint.h>
#include <stdbool.h>

typedef struct Cart Cart;

#include "snes.h"
#include "statehandler.h"

/* ThumbySNES hot-ROM SRAM cache size. First N bytes of ROM are copied
 * into SRAM at load time so the hottest code/data reads hit SRAM
 * instead of flash XIP. Immune to core-1 SPC/DSP XIP cache thrashing.
 * 16 KB fits within the ~30 KB SRAM headroom (STATUS.md). Set to 0
 * to disable. */
#define CART_ROM_CACHE_SIZE 16384

struct Cart {
  Snes* snes;
  uint8_t type;
  uint8_t _romIsExternal; /* ThumbySNES: 1 = rom points at flash XIP, no free */

  uint8_t* rom;
  uint32_t romSize;
  uint8_t* ram;
  uint32_t ramSize;

#if CART_ROM_CACHE_SIZE > 0
  uint8_t *romCache;        /* points at static buffer, NOT heap */
  uint32_t romCacheOffset;  /* ROM offset that romCache[0] maps to */
#endif
};

// TODO: how to handle reset & load?

Cart* cart_init(Snes* snes);
void cart_free(Cart* cart);
void cart_reset(Cart* cart); // will reset special chips etc, general reading is set up in load
bool cart_handleTypeState(Cart* cart, StateHandler* sh);
void cart_handleState(Cart* cart, StateHandler* sh);
void cart_load(Cart* cart, int type, uint8_t* rom, int romSize, int ramSize); // loads rom, sets up ram buffer
bool cart_handleBattery(Cart* cart, bool save, uint8_t* data, int* size); // saves/loads ram
uint8_t cart_read(Cart* cart, uint8_t bank, uint16_t adr);
void cart_write(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);

#endif
