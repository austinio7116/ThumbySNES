# ThumbySNES — Phase 2 memory audit (2026-04-13)

## Question

Can we fit snes9x2002's runtime footprint inside the ~390 KB of SRAM the
stock Thumby Color has available after firmware + display framebuffer +
drivers? (See `PLAN.md §4.1` for the budget derivation.)

## Headline finding

**No — not without surgery.** As-shipped the core needs roughly **16 MB of
heap + 744 KB of static BSS**. That's ~40× the heap budget and ~2× the
static budget. But — most of the bulk is in oversized **compile-time
buffers** and **optional features** (cheats, SA-1, DSP1 LUTs) that can be
removed or replaced with flash-resident alternatives. Realistic device
footprint after the shrinks below: **~360-400 KB — tight but viable.**

## How it was measured

`src/snes_memaudit_main.c` intercepts `malloc` / `calloc` / `realloc` /
`free` via GNU ld `--wrap` and tracks live bytes + peak watermark across
boot → `snes_load` → N frames → `snes_unload` for each ROM. Build target
`memaudit` (host-only, Phase 2-only).

Static BSS/data was read from `size` and `nm --size-sort` on
`build/libsnes9x2002.a`.

Host is x86-64 (64-bit pointers), so pointer-heavy structs (`Memory.Map`,
`Memory.WriteMap`) cost 2× what they will on the 32-bit ARM device.

## Per-ROM heap peaks

Every ROM in the collection hit the **same 15,783 KB** peak (SMW 512 KB,
Chrono Trigger 4 MB, Super Metroid 4 MB, Mario Kart + DSP1, Star Fox +
SuperFX — all identical). snes9x2002 sizes buffers at compile-time maxima
regardless of actual ROM size, so the audit collapses to one number.

## Live heap breakdown (at steady-state, any ROM)

| Source | Bytes | KB | Device fate |
|---|---:|---:|---|
| `Memory.ROM` — `MAX_ROM_SIZE + 0x200 + 0x8000` in `memmap.c:177` | 6,324,736 | **6,180** | **Drop alloc. Point `Memory.ROM` at XIP flash mmap of the FatFs file.** Same trick as `ThumbyNES/device/nes_picker.c`. |
| `GFX.Screen_buffer` — `2048 × 512 × 2 × 2 + safety` (our init) | 4,194,432 | **4,096** | Shrink to ~64 KB scanline window OR render direct into 128×128 LCD framebuffer (32 KB) + tiny one-line bounce. |
| `GFX.ZBuffer_buffer` | 2,097,280 | **2,048** | Shrink to 256×224 = 56 KB, or per-scanline 256 B. |
| `GFX.SubZBuffer_buffer` | 2,097,280 | **2,048** | Same as ZBuffer. |
| `IPPU.TileCache[TILE_2BIT]` — `MAX_2BIT_TILES=4096 × 128` | 524,288 | 512 | Shrink `MAX_2BIT_TILES` to 512 → 64 KB. Rarely saturated. |
| `IPPU.TileCache[TILE_4BIT]` — `MAX_4BIT_TILES=2048 × 128` | 262,144 | 256 | Shrink to 512 → 64 KB. |
| `IPPU.TileCache[TILE_8BIT]` — `MAX_8BIT_TILES=1024 × 128` — Mode 7 only | 131,072 | 128 | Shrink to 256 → 32 KB, or disable for non-M7 games (per-ROM config). |
| `Memory.RAM` (SNES WRAM) | 131,072 | 128 | **Keep as-is — mandatory.** |
| `Memory.SRAM` (cart save RAM, oversized) | 131,072 | 128 | Cap at 32 KB (0x8000). Largest real SNES cart SRAM is 64 KB; snes9x2002 allocates 128 KB unconditionally. |
| `GFX.X2` — 16-bit × 65536 color-math LUT | 131,072 | 128 | Move to flash const. |
| `GFX.ZERO_OR_X2` — ditto | 131,072 | 128 | Move to flash const. |
| `GFX.ZERO` — ditto | 131,072 | 128 | Move to flash const. |
| `Memory.VRAM` | 65,536 | 64 | **Keep as-is — mandatory.** |
| `IAPU.RAM` (SNES ARAM) | 65,536 | 64 | **Keep as-is — mandatory.** |
| 3× `IPPU.TileCached[...]` (validity bitmaps) | 7,168 | 7 | Shrink with tile caches. |
| DSP1 `Cos/SinTable2Fix`, `Cos/SinTable2` (runtime `malloc` copies) | ~32,000 | 32 | Already duplicated as static BSS — drop runtime allocs. |
| `misc` (small snes9x bookkeeping + memory_stream tmps) | ~138,000 | ~135 | Leave alone; most is freed after load. |
| **Total** | **~15,784 KB** | | Target: ~300 KB after shrinks (see below). |

## Static BSS breakdown (library-wide: 744 KB)

| Symbol | File | Size | Device fate |
|---|---|---:|---|
| `Cheat` | cheats.c | **309,632 B (302 KB)** | **Remove. Compile out cheats.c / cheats2.c.** Not referenced from cpu.c / memmap.c / cpuexec.c — fully isolated. Saves 302 KB BSS + 12 KB text. |
| `Echo` — `int[24000]` delay buffer | globals.c:171 | 96,000 B (94 KB) | Switch to `int16` → 48 KB, or shrink to 8000 samples → 16 KB. Some audio quality loss. |
| `Memory` struct — includes `Map[0x1000]`, `WriteMap[0x1000]` | globals.c | 90,264 B (88 KB) | On 32-bit ARM the two map tables halve to 16 KB each; real device size ~56 KB. Keep. |
| `buffer` — DMA staging | dma.c:59 | 65,536 B (64 KB) | DMA transfer staging. Shrink to 16 KB — real-world DMAs rarely exceed that in one burst. |
| `SA1_Map` | globals.c | 32,768 B (32 KB) | **Remove. Compile out sa1.c / sa1cpu.c**, `Settings.ForceNoSA1=TRUE`. Saves 64 KB BSS + ~80 KB text. |
| `SA1_WriteMap` | globals.c | 32,768 B (32 KB) | Same. |
| `bytes0x2000` | ? | 8,192 B | Investigate. |
| `SinTable2Fix` | DSP1 | 8,192 B | Move to flash const (fixed-point lookup table). |
| `DirectColourMaps` | globals.c | 8,192 B | Move to flash const. |
| `SinTable2` | DSP1 | 8,192 B | Move to flash const. |
| `CosTable2Fix` | DSP1 | 8,192 B | Move to flash const. |
| `CosTable2` | DSP1 | 8,192 B | Move to flash const. |
| `wave`, `MixBuffer`, `EchoBuffer`, `DummyEchoBuffer` | soundux | 4× 7,056 B = 28 KB | Combined with SOUND_BUFFER_SIZE = 2×44100/50. If we drop core sample rate to 32040 Hz and 60 fps: 2×32040/60=1068 samples. Saves ~22 KB. |
| `IPPU` struct | ppu.h | 4,888 B | Keep. |
| `LineData` | gfx | 3,840 B | Keep. |
| `GSU` | fxemu | 3,288 B | **Remove with SuperFX** (Settings.ForceNoSuperFX). Star Fox + Yoshi's Island won't run — acceptable per §11 non-goals. |
| `LineMatrixData` | gfx | 2,880 B | Keep. |
| (rest) | various | ~80 KB | Keep. |
| **Total** | | **~744 KB** | Target after shrinks: **~300 KB** |

## Device path — surgeries, expected savings

Priority order. Each step stands alone; cumulative savings shown.

| # | Surgery | Effort | Saves (static + heap) | Cumulative |
|--:|---|---|--:|--:|
| 1 | **Skip ROM copy — point `Memory.ROM` at XIP** | Patch `LoadROM()` in memmap.c — read header from memstream, then set `Memory.ROM = (uint8*)XIP_BASE + offset` | — +6,180 KB heap | -6,180 KB |
| 2 | **Render straight into 128×128 LCD fb — drop `GFX.Screen_buffer` to scanline-only** | Replace the 4 MB `Screen_buffer` alloc with a tiny ~16 KB line-bounce buffer, add inline blit\_fill / blit\_fit hook in the per-scanline path (Phase 4 work anyway) | — +4,064 KB heap | -10,244 KB |
| 3 | **Shrink ZBuffer + SubZBuffer to 256×224** | 2 × 56 KB instead of 2 × 2 MB | — +3,984 KB heap | -14,228 KB |
| 4 | **Move `GFX.X2` / `ZERO_OR_X2` / `ZERO` LUTs to flash const** | Patch `S9xGraphicsInit` in gfx.c — drop the malloc, point at flash array, fill at build time via generator under `tools/` | — +384 KB heap | -14,612 KB |
| 5 | **Shrink tile caches** — `MAX_2BIT_TILES 4096→512, 4BIT 2048→512, 8BIT 1024→256` | One-line ppu.h changes, stress-test to confirm no misses during gameplay | — +832 KB heap | -15,444 KB |
| 6 | **Cap `Memory.SRAM` at 32 KB** | Change malloc in memmap.c:175 | — +96 KB heap | -15,540 KB |
| 7 | **Compile out cheats.c / cheats2.c** | CMake source-list pruning + `Settings.ApplyCheats=FALSE` | -302 KB static | -15,540 h + -302 s |
| 8 | **Compile out sa1.c / sa1cpu.c** | CMake source-list pruning + `Settings.ForceNoSA1=TRUE`. Breaks Super Mario RPG, Kirby Super Star — noted in PLAN.md §11 | -64 KB static -80 KB text | -15,540 h + -366 s |
| 9 | **Move DSP1 tables to flash const + drop runtime mallocs** | dsp1emu_yo.c:77-81 | -32 KB static +~32 KB heap | -15,572 h + -398 s |
| 10 | **Drop `Cheat` runtime malloc path + shrink Echo to int16[8000]** | Small patch to soundux / globals | -80 KB static | -15,572 h + -478 s |

## Post-surgery device budget (estimated)

| Region | Size | Notes |
|---|---:|---|
| Keep (WRAM / VRAM / ARAM / Memory struct on 32-bit / IPPU etc) | ~360 KB | Non-negotiable SNES state + core bookkeeping |
| Post-shrink GFX buffers (Z + SubZ + scanline, no Screen_buffer) | ~130 KB | 256×224 Z+SubZ = 112 KB + ~16 KB line bounce |
| Post-shrink SRAM + tile caches + DMA buffer | ~150 KB | 32 + 96 + 16 + LUTs on flash |
| **Total** | **~640 KB** | **Still 250 KB over 390 KB budget** |

**Gap analysis**: the ZBuffer + SubZBuffer at 2 × 56 KB = 112 KB is still
large. They exist for BG priority and color-math depth testing. Device
option: run per-scanline, 256 bytes × 2 = 0.5 KB total. Requires confirming
snes9x2002's renderer doesn't cross-reference rows across scanlines
(probably does — color math looks at the subscreen from the main scanline).

Second option: drop Settings.Transparency / color math entirely — saves
SubZBuffer + SubScreen. Visually significant loss (many SNES games use
transparency for status bars, menus, HUD fades) but buys back ~80 KB.

Third option: the `Memory.Map` tables at 32 KB × 2 (16 KB × 2 on 32-bit)
could be replaced with a smaller hash or direct-mapped scheme — saves
~24 KB but changes the core's hot path.

## Next steps for Phase 2

1. **Prove the surgeries one at a time**, confirming the host bench still
   plays SMW between each step:
   - [ ] Patch `LoadROM` to accept a pre-mapped pointer (skip copy).
   - [ ] Drop `Memory.SRAM` to 32 KB.
   - [ ] Remove cheats.c / cheats2.c from CMake; confirm no missing symbols.
   - [ ] Remove sa1.c / sa1cpu.c from CMake.
   - [ ] Shrink tile cache maxima; run all 17 ROMs.
   - [ ] Shrink `GFX.Screen_buffer` + Z buffers to 256×224 rectangles.

2. **Re-run memaudit** after each surgery; record new watermark in this doc.

3. **Decide on transparency / color-math cut** based on the ZBuffer gap.
   This is the last knob before the per-scanline render work in Phase 4.

4. **If still over 390 KB**: the remaining gap is probably the GFX Screen +
   ZBuffer. Push to Phase 4's scanline-renderer surgery — those buffers
   become per-line local state and stop being allocated at all.

Exit criterion for Phase 2: memaudit reports heap ≤ 350 KB after all
surgeries on host build. Any fixed-cost BSS the core can't let go of we
book as a debt and test against the real device budget in Phase 3.

---

## Phase 2 results (2026-04-13)

| # | Surgery | Status | Actual saving |
|--:|---|---|--:|
| 1 | Remove cheats (gc-sections + stub `S9xInitCheatData`/`S9xApplyCheats`) | ✅ | **-186 KB BSS** |
| 2 | Compile out sa1.c / sa1cpu.c, stub 5 entry points | ✅ | **-104 KB text** (flash-only benefit; SA-1 struct stays in globals.c because memmap.c references `SA1_Map`) |
| 3 | Cap `Memory.SRAM` 128 KB → 64 KB (largest real cart SRAM = Star Fox) | ✅ | **-64 KB heap** |
| 4 | Move `GFX.ZERO` LUT to flash const | **Deferred** — USE\_OLD\_COLOUR\_OPS is off, only one 128 KB LUT allocated (not three); move is device-specific work (CMake-time generator). Skip for host phase. | (0 on host / -128 KB on device) |
| 5 | Shrink tile caches (MAX\_2BIT 512, MAX\_4BIT 512, MAX\_8BIT 256) | **REVERTED** | See note below |
| 6 | Shrink DMA `buffer` 64 KB → 16 KB (disables SDD1 games — non-goal) | ✅ | **-48 KB BSS** |
| 7 | Shrink `Echo` `int[24000]` → `int[16384]` (still covers SNES HW max 240 ms echo at 32 kHz) | ✅ | **-30 KB BSS** |
| 8 | XIP ROM (skip LoadROM copy) | **Deferred to device port** — host mmap already doesn't hit this path | (0 on host / -6,180 KB on device) |

### Failed surgery: tile cache shrink

Attempted `MAX_2BIT_TILES=512`, `MAX_4BIT_TILES=512`, `MAX_8BIT_TILES=256`.
Result: `free(): invalid pointer` inside `MemoryDeinit` — heap corruption.

**Why**: `tile.c:561-566` computes `TileNumber = (TileAddr & 0xffff) >>
BG.TileShift` with no bounds check against `MAX_*BIT_TILES`. For 4 bpp,
`TileShift=5`, so `TileNumber` ranges 0..2047. A 512-slot cache would be
indexed out-of-bounds by up to 4×. Upstream's values (4096 / 2048 / 1024)
are the **minimum correct sizes** for each bit-depth, dictated by VRAM.

**Consequence**: the 896 KB of tile-cache heap (128 + 256 + 512 KB) stays
baked in on device unless we add an eviction path to `tile.c` — turn the
indexed cache into a hash/LRU. That's a Phase 4+ surgery, not Phase 2.

### Headline numbers (host, SMW, steady state)

|  | Before | After Phase 2 | Δ |
|---|---:|---:|---:|
| Heap peak | 15,783 KB | **15,591 KB** | -192 KB (primarily SRAM cap + Echo) |
| Static BSS (libsnes9x2002.a linked into memaudit) | 744 KB | **479 KB** | **-265 KB** (cheats + DMA + Echo) |
| Static text (same) | 688 KB | **588 KB** | -100 KB (SA-1) |

### Regression check

All 17 ROMs boot + run 120-frame bench cleanly after Phase 2 surgeries
(range: Star Fox 50× to Super Metroid 277× realtime on host). Super
Mario RPG parses + runs despite SA-1 stubs — game will hang at title when
SA-1 computations are expected, acceptable per §9 non-goals.

### Device projection with all surgeries (including device-only)

| Bucket | SRAM KB |
|---|---:|
| WRAM + VRAM + ARAM + capped SRAM | 320 |
| Tile caches (can't shrink without eviction path) | 896 |
| Memory struct (64-bit host → halves on 32-bit) | ~44 |
| DMA buffer (shrunk) | 16 |
| Echo (shrunk) + other sound | ~30 |
| Other core BSS (IPPU, GSU, LineData, Settings, etc.) | ~70 |
| GFX.Screen_buffer → per-scanline bounce (Phase 4) | ~16 |
| GFX.ZBuffer + SubZBuffer → per-scanline locals (Phase 4) | ~1 |
| GFX.ZERO LUT → flash const | 0 |
| Memory.ROM → XIP | 0 |
| **Subtotal emulator** | **~1,393 KB** |

Against a 390 KB budget → **still 1 MB over**. The tile caches alone (896 KB) eat
2.3× the budget. **Phase 2 alone cannot close the gap.**

### Path to fit (what Phase 4+ must do)

1. **Replace tile cache with LRU / hashed** — 896 KB → ~16 KB (4-8 KB per
   bit-depth, evict on collision). Accepts cache-miss re-decode cost.
2. Phase 4's scanline renderer surgery (already planned).
3. Static BSS review — Memory struct map tables halve on 32-bit; smaller
   state like `Settings` struct is already small.

With (1) applied: subtotal drops from 1,393 → ~513 KB, then minus device
pointer halving (Memory struct saves ~22 KB) + scanline wins already
counted → **~491 KB**. Still ~100 KB over, and we haven't counted the
firmware overhead yet.

Further cuts if needed:
- Drop Settings.Transparency / color math subsystem (SubScreen + depth) —
  costly visually but saves another ~30 KB of stateful buffers.
- Reduce Echo further (shrink to 8192 → -32 KB with audible artifacts).
- Move Echo + DSP tables + other large const LUTs explicitly to flash.

**Initial Phase 2 verdict**: fitting in 390 KB requires both Phase 2
surgeries (done) AND Phase 4+ tile-cache eviction AND probably one more
knob. Phase 4 measurements below show the gap is wider than that.

---

## Phase 4 groundwork — tile cache eviction (2026-04-13)

Implemented a direct-mapped hash cache for tile decoding, gated by
`THUMBYSNES_TILE_EVICTION` in `ppu.h`. See the vendor patch list in
`vendor/VENDORING.md`.

### Design

- `MAX_2BIT_TILES`, `MAX_4BIT_TILES`, `MAX_8BIT_TILES` become **slot counts**
  (power of 2), not upstream's "one slot per possible TileNumber":
  - 2bpp: 256 (was 4096) → 32 KB cache + 512 B bitmap + 512 B tags
  - 4bpp: 256 (was 2048) → 32 KB cache + 512 B bitmap + 512 B tags
  - 8bpp:  64 (was 1024) →  8 KB cache + 128 B bitmap + 128 B tags
- New `uint16_t* IPPU.TileTag[3]` array: tag[slot] = TileNumber currently in
  that slot. Set alongside `Buffered[slot]` on decode.
- `TILE_PREAMBLE` macro rewritten: `slot = TileNumber & (SlotCount - 1)`;
  cache hit iff `Buffered[slot] && Tag[slot] == TileNumber`; else re-decode.
- Invalidation sites in `ppu.h` (VRAM register writes) mask indices with
  `& (MAX_*BIT_TILES - 1)` so they target the slot, not a raw TileNumber.
- `gfx.c` setup of `BG.Buffer`/`BG.Buffered` extended to also set `BG.Tag`
  and `BG.BufferSlots`.

### Measured impact (host, SMW, 120 frames)

| | Before (Phase 2) | After (Phase 4 prep) | Δ |
|---|---:|---:|---:|
| Heap peak | 15,591 KB | **14,890 KB** | **-701 KB** |
| Tile cache heap (bitmap+cache+tags) | 896 KB + 7 KB | **72 KB + 1 KB** | **-830 KB** |
| SMW fps (idle title) | 108× realtime | 61×–139× realtime | within noise |

Heap drop slightly less than the cache surgery alone would predict because
some small allocations (~100 KB) drifted up; the net is still ≈-700 KB.

### Regression test

All 17 ROMs boot + bench 120 frames cleanly, fps in same range as before
the eviction. Visual spot-check on SMW rendering pending.

### Updated device projection

With all surgeries (Phase 2 + tile eviction + Phase 4 scanline):

| Region | KB |
|---|---:|
| WRAM + VRAM + ARAM + SRAM (capped 64) | 320 |
| Memory struct (32-bit) | ~60 |
| Tile caches (hash-evicted, 256/256/64 slots) | **~72** |
| DMA buffer (shrunk) + Echo (shrunk) + sound | ~36 |
| Other core BSS (IPPU, GSU, LineData, Settings, etc.) | ~70 |
| GFX.Screen_buffer (per-scanline, Phase 4) | ~16 |
| GFX.ZBuffer + SubZBuffer (per-scanline, Phase 4) | ~1 |
| GFX.ZERO LUT → flash const (Phase 4 device build) | 0 |
| Memory.ROM → XIP (Phase 4) | 0 |
| **Subtotal emulator** | **~575 KB** |

Against a ~390 KB emulator-state budget (520 total - 130 firmware overhead):
**~185 KB still over**. Mostly from:
- WRAM 128 + VRAM 64 + ARAM 64 = 256 KB, non-negotiable
- Memory struct 60 KB, can't shrink without map-table redesign
- Other core BSS ~70 KB

Possible additional cuts to close the gap:
- Cap or disable SubScreen/color-math path (Settings.Transparency=FALSE) —
  saves the color-math-specific state, maybe 20-30 KB.
- Shrink Memory map tables (Memory.Map / WriteMap / MemorySpeed) from
  4096-entry to 2048-entry (LoROM/HiROM both map with 24-bit addresses
  but only 8 MB effective, so 2048 might suffice) — saves ~24 KB.
- Move Memory.VRAM to... wait, VRAM is mandatory writable, stays in SRAM.
- Drop Settings / cheats-related struct fields entirely.

Net: **fitting in 390 KB is still possible but requires ~185 KB more**,
which means closing transparency and/or shrinking memory map. Both are
surgical changes that can happen in Phase 4 alongside the scanline
render. Still feasible, still tight, no comfortable margin.

---

## Phase 4 device-build measurements (2026-04-13)

Built `firmware/snesrun_device.uf2` with `THUMBYSNES_LINK_CORE=ON`,
all Phase 2 surgeries + tile eviction + new device-only flags:
`THUMBYSNES_FLASH_GFX_ZERO=1` (LUT to flash) and
`THUMBYSNES_DISABLE_TRANSPARENCY=1` (alias SubScreen onto Screen,
SubZBuffer onto ZBuffer, force `Settings.Transparency = FALSE`).

`arm-none-eabi-size build_device/snesrun_device.elf`:

| Section | Bytes | Note |
|---|---:|---|
| text   | 742,000 | Flash-resident — fine, 16 MB available |
| data   |       0 | |
| **bss** | **333,628** | **64% of 520 KB SRAM total** |

The big BSS contributors (from `nm --size-sort`):

| Symbol | KB | Note |
|---|---:|---|
| Echo | 64 | snes9x SPC echo delay buffer |
| Memory (struct) | 56 | Memory map tables — already halved on 32-bit |
| cache (snes_flash_disk) | 16 | Already shrunk 8→4 blocks |
| g_fb (LCD framebuffer) | 32 | Mandatory for SPI flush |
| buffer (DMA) | 16 | Already shrunk |
| SA1_Map + SA1_WriteMap | 32 | Dead code — still BSS-resident |
| DSP1 sin/cos tables | 32 | 4× 8 KB float arrays |
| DirectColourMaps | 8 | |
| Sound buffers (Mix/Echo/Dummy) | 21 | SOUND_BUFFER_SIZE-derived |
| IPPU + GSU + LineData + LineMatrixData | ~13 | Core PPU state |
| g_roms (picker) | 6.5 | Per-ROM metadata cache |
| favs_buf (picker) | 4 | Favorites store |
| (rest) | ~33 | small stuff |

SRAM math: 520 KB total - 333 KB BSS - ~50 KB stack/firmware overhead =
**~137 KB heap available**.

### Heap demand vs supply

The mandatory SNES state alone exceeds the heap budget on its own:

| Heap alloc | KB | Status |
|---|---:|---|
| Memory.RAM (WRAM) | 128 | mandatory |
| Memory.SRAM (capped) | 64 | mandatory |
| Memory.VRAM | 64 | mandatory |
| IAPU.RAM (ARAM) | 64 | mandatory |
| **Mandatory subtotal** | **320** | |
| FillRAM (XIP I/O shadow) | 32 | XIP-only |
| Tile caches (hash-evicted) | 72 | shrunk Phase 4 prep |
| GFX.Screen (Sub aliased) | 150 | SubScreen no longer separate |
| GFX.ZBuffer (SubZ aliased) | 75 | SubZBuffer no longer separate |
| (other small core allocs) | ~30 | |
| **Total demand** | **~679 KB** | |

Heap supply: **~137 KB**. Demand: **~679 KB**. **Gap: ~540 KB.**

### Honest conclusion

**A standard snes9x2002 emulator does not fit in stock Thumby Color SRAM
(520 KB).** Even after every shrink applied (cheats out, SA-1 out, SDD1
shrunk, Echo at 16 K samples, tile cache hash-evicted to 72 KB, GFX
buffers shrunk 11 MB → 225 KB, GFX.ZERO LUT to flash, SubScreen+SubZ
aliased), the mandatory SNES state (128+64+64+64 KB = 320 KB of WRAM /
VRAM / ARAM / SRAM) **alone exceeds the available heap (~137 KB) by
~180 KB**.

The remaining gap (540 KB) breaks down as:
- ~180 KB of mandatory SNES state can't fit in heap because BSS already
  consumed most of SRAM.
- ~225 KB of GFX buffers (Screen + ZBuffer) — needs per-scanline render
  surgery to shrink to ~3 KB.
- ~135 KB of other heap (FillRAM, tile caches, misc).

### What would close the gap?

1. **Convert mandatory SNES state from heap to fixed BSS.** Same total
   bytes, but moving Memory.RAM/.SRAM/.VRAM/IAPU.RAM out of malloc and
   into static arrays would cancel the 320 KB heap demand at the cost of
   320 KB more BSS — net zero, but means we're not blocked by heap
   exhaustion. Linker would catch the over-budget condition at build time
   instead of at runtime.

2. **Per-scanline GFX rendering.** Patch `ppu_.c` so it writes one line
   at a time into a small bounce buffer, immediately blitted to the LCD
   framebuffer + reused for the next line. Saves ~225 KB. Significant
   surgery — the existing `RenderScreen` accumulates many lines in a
   single call across HDMA clip ranges.

3. **Drop more BSS to flash.** DSP1 tables (32 KB) + DirectColourMaps
   (8 KB if PPU.Brightness is held fixed) → flash-const generators.
   Echo `int[16384]` → `int16[16384]` (-32 KB). SA1_Map/WriteMap remove
   (requires patching memmap.c to stop indexing into them in mapper
   setup, ~64 KB save).

Even ALL of (1)+(2)+(3) leaves us ~50 KB tight against 520 KB. Workable
but no margin.

### Strategic options

The fundamental constraint is `320 KB mandatory SNES state` + `333 KB
BSS` = 653 KB > 520 KB SRAM **before counting any GFX buffers, stack,
or firmware overhead**. The Thumby Color hardware as shipped is at the
ragged edge of feasibility for a snes9x-class emulator.

Three paths forward:

| Path | Effort | Outcome |
|---|---|---|
| **A. Push through with all surgeries** | High (per-scanline renderer + BSS-to-flash for DSP1 + heap-to-BSS for SNES state + remove SA-1 references in memmap.c) | Tight fit, no margin. Some games will OOM under heavy scenes. CPU perf untested. |
| **B. PSRAM-equipped board** (Pico Plus 2 = 8 MB PSRAM, ~$10) | Low (CMake board override + a bit of memory-mapping for WRAM/SRAM/ARAM in PSRAM). Stays code-compatible with stock Thumby Color where it fits. | Memory solved cleanly. Still need CPU work. NOT stock Thumby Color. |
| **C. Smaller SNES core** (search for a scaled-down core) | Medium (vendor + integrate a different emulator). Risk: smaller cores tend to be incomplete. | Probably possible to fit, lower compatibility ceiling. |

User decision needed: **proceed with A**, **pivot to B**, or **investigate
C**? Phase 4 work to date is committed; whichever path we take, Phases
0-3 + the cache eviction + GFX shrink groundwork remain useful.

