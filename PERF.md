# ThumbySNES — catalog of performance attempts

Every optimization currently active in the firmware, grouped by category.
Mix of things already present in the upstream LakeSnes core when we
adopted it, stock Pico SDK / toolchain levers, and targeted changes this
project has made on top.

Use this as a reference when anyone asks "have you tried X?" — if it's
listed here, it's in place; if not, it's either discarded (see
"Optimizations tried + reverted" in `STATUS.md`) or unexplored.

---

## Baseline choice

1. **LakeSnes** (pure C, ~6500 LOC, per-scanline rendering native to
   the core). Chosen over snes9x2002 because the latter's 4 MB GFX
   buffer architecture couldn't fit in 520 KB SRAM. Everything below
   stacks on top of this choice.

## System / build-level

2. **300 MHz overclock** (`set_sys_clock_khz(300000)` in
   `device/snes_device_main.c`). RP2350 spec is 150 MHz; +20% on
   raw CPU for free.
3. **`LAKESNES_HOT` → `.time_critical.snes` SRAM placement.** Hot
   functions get copied from flash XIP into SRAM at boot, dodging
   flash cache misses on the inner loop. Currently marked:
   `cpu_runOpcode`, `spc_runOpcode`, `ppu_runLine`, `ppu_renderBgLine`,
   `snes_runCycle`/`runCycles`, `snes_readBBus`/`writeBBus`,
   `snes_read`/`write`/`cpuRead`/`cpuWrite`, `dma_handleDma`,
   `apu_runCycles`, `apu_spcRead`/`Write`/`Idle`, `dsp_cycle`.
4. **`double` → `float`** for APU catch-up math. M33 has a
   single-precision FPU; doubles went through softfloat at ~50
   cycles per op.
5. **`-O3 -fomit-frame-pointer -fstrict-aliasing -ffunction-sections
   -fdata-sections`** on the core library; **`-Wl,--gc-sections`** on
   link. Dead-strips unused code.
6. **SPC fractional-cycle catch-up fix.** Upstream cast the cycle
   accumulator to `int` before running SPC opcodes, permanently
   losing fractional cycles on tight CPU polls — this is what the
   SMW hang likely is. Runs while accumulator > 0.0f. (Still in
   single-core fallback; no longer in the hot path on device
   thanks to the dual-core split below.)

## ROM / memory layout

7. **XIP zero-copy ROM.** Patched `cart_load` to take the caller's
   pointer when `thumbysnes_cart_xip=1`; `snes_loadRom` skips its
   power-of-2 mirror malloc on the XIP path. ROM lives in flash,
   accessed directly via address arithmetic in `cart_read`
   (LoROM/HiROM mapper). No bank copying into SRAM — SRAM is
   ~fully consumed by WRAM 128 + VRAM 64 + ARAM 64 + SNES state +
   hot code / driver state, ~30 KB headroom. XIP cache handles
   hot-bank locality.
8. **Per-scanline pixelBuffer.** Upstream LakeSnes carried a 978 KB
   XBGR frame buffer for the SDL frontend; patched to 2 KB (one
   line). Scanline callback fires after each visible line so the
   frontend can consume + discard. The only reason this emulator
   fits in SRAM at all.

## PPU performance knobs

9. **`skipColorMath`.** Drops subscreen fetch + add/sub blending.
   Loses transparency / fade effects; cuts per-pixel work nearly
   in half.
10. **`renderXStart`/`renderXEnd`.** Skips the ~32 invisible edge
    columns under FILL-mode crop. ~12% fewer per-pixel calls.
11. **Per-line BG cache.** `bgLine[layer][priority][x]` decoded once
    per scanline with tile-major inner loop; compositor reads the
    array instead of decoding tiles per pixel.
12. **BG empty-slot skip.** `bgLineNonEmpty[L][P]` flag lets the
    compositor skip full 256-wide passes for fully-transparent
    (layer, priority) slots — common on scenes using only BG1 +
    sprites.
13. **Per-line RGB565 full composite.** Walks the mode's layer-slot
    list once per line bottom-up, writing RGB565 directly via
    `cgramRgb565`. Replaces the per-pixel `ppu_handlePixel` walk
    (which did 8-12 slot iterations per pixel with
    `ppu_getWindowState` function calls inside the hot loop).
14. **CGRAM → RGB565 LUT with brightness baked in.** `cgramRgb565[256]`,
    lazy rebuild on $2100/$2121/$2122 writes via a `cgramDirty`
    flag. Collapses 3-channel brightness math + byte-pack into
    one `uint16_t` read.
15. **Tile-decode bit-lane LUT.** 256-entry byte→8-lane expansion
    table used by `ppu_renderBgLine` AND `ppu_evaluateSprites`.
    Replaces 8 × {shift + AND + OR} inner loops with pointer
    derefs + array reads. ~2× faster per 4bpp tile.
16. **Native-LCD mode** (opt-in via `snes_set_lcd_mode`, NOT
    default). PPU composites RGB565 directly into the 128×128
    device framebuffer at native resolution (128 of 256 cols
    sampled, ~43% of lines skipped). Nearest-neighbour — text
    unreadable, so held back from default. Kept as per-ROM opt-in.
17. **Frameskip** (opt-in via `snes_set_frameskip(n)`, NOT enabled).
    Sets a `skipRender` flag that bails at the top of `ppu_runLine`
    before sprite eval + BG cache + composite. Halves PPU cost at
    the price of visible-fps halving.

## CPU / bus / scheduler

18. **Batched cycle scheduler.** `snes_runCycles` fast-forwards hPos
    between events (0 / 16 / 512 / 1104 / hTimer×4 / wrap territory)
    instead of looping `snes_runCycle` per 2 master cycles.
    Advances `snes->cycles`, `snes->hPos`, `autoJoyTimer` in bulk;
    re-evaluates IRQ condition at batch end with the false→true
    edge detector intact. ~300× fewer slow-path calls per frame.

## Dual-core split

19. **`THUMBYSNES_DUAL_CORE=1`** compile-time flag on device build.
20. **Core 0**: 65816 CPU + PPU + LCD SPI DMA + bus + DMA + input +
    USB. **Core 1**: SPC700 + DSP only (audio DSP, not output).
21. **Core 1 entry** (`snes_apu_core1_loop`) parks reading a shared
    `Snes*` pointer; launched once at boot via
    `multicore_launch_core1` and starts running `spc_runOpcode` in a
    tight loop whenever a ROM is loaded.
22. **`snes_catchupApu` compiles to no-op** on dual-core builds.
23. **Per-master-cycle `apuCatchupCycles` accumulator update skipped.**
    Saves an FPU multiply + add per ~1–2M `snes_runCycle` calls per
    frame.
24. **Byte-atomic `inPorts` / `outPorts`.** No mailbox or spinlock;
    byte writes/reads are atomic on M33, and games' polling loops
    handle cross-core latency naturally. Side benefit: SMW's SPC
    handshake hang may resolve as the SPC is no longer starved of
    cycles during tight CPU polls.

## CPU dispatcher (already active; was "known gap")

25. **ARM Thumb-2 asm dispatcher** (`src/cpu_asm.c`). 256-handler
    naked-function state machine. Keeps the 65816 registers in
    ARM callee-saved regs across a batch of opcodes; bus-access
    helpers (`.Lrd`, `.Lwr`, `.Lidl`, `.Lfetch`) preserve r4-r11
    across BLX back into C. `cpu_runBatchAsm(cpu, 64)` runs 64
    opcodes per call from `snes_runFrame`. Replaces the giant
    switch in `cpu_doOpcode`. See "ASM dispatcher debugging
    history" in `README.md` — the merge uncovered two latent bugs
    (r0 clobber in `.Lidl/.Lwr`, missing Thumb bit on internal
    function pointers) that took a ROM-based bisection harness
    to track down.

## Per-session additions (2026-04-16)

26. **DMA block-copy fast path.** `dma_transferByte`
    (`vendor/lakesnes/snes/dma.c:340`) detects the common
    DMA pattern — WRAM/ROM A-bus source, PPU VRAM data port
    B-bus dest (`$2118`/`$2119`), `vramRemapMode == 0` — and
    bypasses `snes_read` + `snes_writeBBus` + `ppu_write`
    dispatch (~25 instructions). Goes straight through the
    `snes->readMap` block pointer and writes directly into
    `ppu->vram[]`, preserving openBus and vramPointer
    increment semantics exactly. Invalidates the tile cache
    (#27) via `tileCacheEpoch++`. Skipped for remap modes 1-3
    (rare); those fall through to the canonical slow path.

27. **Persistent tile-row decode cache** (`ppu.c:314` in
    `ppu_renderBgLine`). 256-entry direct-mapped cache of
    decoded 8-pixel tile rows, keyed on a 32-bit pack of
    `{tile & 0x5fff, row, bitDepth, tileAdrBase>>12}`. Each
    slot stores post-palette-offset palette-index bytes.
    Epoch counter (`tileCacheEpoch`) bumped on every VRAM byte
    write (`ppu_write` cases `$18`/`$19`; also by the DMA fast
    path #26). Entries valid iff tag matches AND stamped
    epoch equals current — zero-cost invalidation, no
    invalidation passes. Key construction hoisted out of the
    per-tile loop where possible (`tcLayerBits`). Miss path
    **fuses** the cache store into the original per-pixel
    decode loop so miss is ~zero extra cost; hit path skips
    the 8-iteration decode body entirely, saving ~50 cycles
    per hit. Storage: 4 KB in the Ppu struct (heap).
    Net host effect: Zelda +19%, SMW +5%, FFII +4%, Metroid
    +1%, Chrono Trigger -2% (noise). Device benefit expected
    larger since the decode loop's LUT derefs hit flash XIP
    (see #29) on miss.

28. **Per-slot window-mask precompute** (`ppu.c:565` in
    `ppu_composeLineRgb565`, new helper `ppu_buildWindowMask`).
    When a layer slot is windowed, build a 256-byte mask
    array once from the `windowLayer[L]` registers + window
    left/right bytes + combine logic, then use `winMask[x]`
    in the per-pixel composite loop. Replaces 256 calls to
    `ppu_getWindowState` (each an ~20-cycle branch cascade)
    with one hoisted pre-pass + 256 byte-loads. Helps games
    that use windows (lantern in Zelda, transitions, spell
    effects). No effect on the vast majority of games that
    don't window.

29. **Bit-lane LUT in SRAM** (`ppu.c:71`, `ppu_bitExpandLut`
    section `.time_critical.snes_lut`). The 2 KB byte→8-lane
    expansion table used by `ppu_renderBgLine` and
    `ppu_evaluateSprites` previously sat in flash `.rodata`.
    Tile decoders deref it 4-8 times per tile — any flash XIP
    cache miss costs ~100 cycles. Pico SDK copies
    `.time_critical.*` sections into SRAM at boot. Device-only
    (gated on `LAKESNES_HOT_SRAM=1`); host leaves it in
    `.rodata` where the x86 L1 already handles it.

30. **Packed-32 RGB565 blend** (`device/snes_run.c:46`,
    `rgb565_avg2` + `rgb565_avg2_x2`). Rewrote the 2-sample
    RGB565 average using the classic bit-mask-parallel-add
    trick: `((a ^ b) & 0xF7DE) >> 1 + (a & b)` — ~4 ARM ops
    vs ~12 for the extract-per-channel form. The 32-bit
    variant applies the same trick to two RGB565 pairs
    packed in a word, used by the vertical blend inner loop
    (128 pixels → 64 iterations, pair-aligned). `s_blend_a`
    and `row_blended` are 4-byte aligned. Cuts the scanline
    callback's blend cost by ~50-60%.

---

## Known gaps (NOT yet attempted — ordered by expected impact)

These aren't active optimizations, listed here for honesty:

- **Cross-core pipelining beyond APU.** Currently core 1 runs
  SPC+DSP + PPU line composite (via `s_ppu_pipeline_line`). Sprite
  eval could be shipped earlier as a producer ahead of the
  compositor. Tricky timing and cross-core memory traffic; may
  or may not win.
- **Inline WRAM fast path in top-4 ASM opcodes.** LDA/STA
  imm+abs+dp hit `.Lrd`/`.Lwr` via BLX even when the bus
  access is a plain WRAM/ROM readMap hit. Inlining that check
  saves ~6-8 cycles per opcode on ~25% of executed ops.
  Estimated +1-2 fps. Careful ASM register management required.
- **M33 DSP intrinsics on the composite path** (`__UADD8`,
  `__SEL`, `__PKHBT`). Not currently used — the scanline blend
  uses the bitmask trick (#30) which is already tight, but
  UADD8 + SEL could halve the CGRAM brightness math on cache
  rebuilds or accelerate tile-plane merges.
- **Function alignment audit.** Pull the map file, check whether
  the hot ASM dispatcher and `ppu_composeLineRgb565` straddle
  16-byte boundaries poorly. M33 icache is 4-way / small;
  `__attribute__((aligned(16)))` on the hot top-5 can stop
  conflict misses. Cheap to try.
- **Half-resolution internal render.** PPU composites directly
  into 128×128 at native res, skipping every-other odd column
  and averaging at the tile-decode layer (not post-blend).
  Different from the existing Native-LCD opt-in mode (#16)
  because it would keep palette fidelity. Potentially 2×. Weeks
  of work.
