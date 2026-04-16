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

## Post-re-profile additions (2026-04-16 late)

Profile taken with device-equivalent defines
(`THUMBYSNES_DIRECT_CPU_CALLS=1`) on FFII + Zelda showed
`ppu_runLine` at 46% and core-0 overhead (DMA dispatch + cycle
scheduler) at ~20%. Three matched levers landed:

31. **`dma_handleDma` fast-exit inline** (`dma.h`, `dma.c`).
    Renamed the body to `dma_handleDmaSlow` and added a static
    inline wrapper in the header that early-returns when
    `dmaState == 0 && !hdmaInitRequested && !hdmaRunRequested`.
    Called ~40M times per frame from every cpu read/write/idle;
    the overwhelming majority are no-ops but previously ate
    function-call overhead. Now the no-op case is ~3 loads + 3
    compares, fully inlined at the call site.

32. **`snes_runCyclesFast` inline fast path** (`snes.h`).
    Static inline that short-circuits the common
    "cycles advance hp within a single no-event region and no
    IRQ edge in window" case, bypassing the while-loop inside
    `snes_runCycles`. Falls through to the slow path when:
    - hp is currently at an event (0, 16, 512, 1104) or wrap
      territory (≥ 1356)
    - hp equals hTimer*4 with hIrqEnabled
    - hp+cycles crosses the next event boundary
    - hp+cycles crosses the 536 DRAM-refresh boundary
    Applied to `snes_cpuRead/Write/Idle` slow paths and
    `snes_flushCycles` (the per-opcode accumulator flush).

    **Gotcha:** on single-core (host) builds, the fast body
    still needs to update `snes->apuCatchupCycles` — skipping
    that starved the SPC of cycles and caused every ROM to
    render black on host. Compiled out on
    `THUMBYSNES_DUAL_CORE=1` (device) where the SPC runs free
    on core 1.

33. **Tile-major RGB565 composite** (`ppu.c:`
    `ppu_emitBgSlotRgb565`, replaces the bgLine reader inside
    `ppu_composeLineRgb565`). For each (layer, priority) slot,
    walks the tilemap and emits RGB565 pixels **directly** into
    `lineRgb565[]` — skipping the 2 KB `bgLine[L][P]`
    intermediate that the old flow wrote then read back.
    Walks the tilemap twice per BG layer (once per priority
    slot); the extra walk is cheap thanks to the tile-row
    decode cache (#27) — cache hits elide both the VRAM reads
    and the bit-lane expansion. `ppu_runLine` skips the
    `ppu_renderBgLine` pre-pass when the RGB565 callback is
    active; the legacy BGRX path still populates `bgLine[L][P]`
    via `ppu_renderBgLine` for backward compatibility.

## Audio quality fixes (not perf, but related)

These don't change fps; they fix audio quality issues caused by
the mismatch between emulated fps and wall time. Listed here
because they're part of the same dual-core architecture and the
symptoms look like perf bugs (audio pitch drifting).

34. **DSP read-cursor tracking** (`dsp.h` + `dsp.c`). Upstream
    `dsp_getSamples` always reads the LAST 534 NTSC (641 PAL)
    samples from the 1024-slot ring, assuming one pull per
    emulated 60 Hz frame. At 7-25 fps emulation the caller pulls
    every 40-140 ms wall — between pulls the DSP writes
    1500-4500 samples but only 534 get read, rest overwritten.
    Each pull's resample ratio depends on the variable pwm_room
    (wall-time-dependent) / fixed 534 input → pitch wobbles with
    fps jitter. Fix: new `uint16_t lastReadOffset` tracks where
    the consumer left off; `dsp_getSamples` resamples the range
    actually produced `[lastReadOffset, sampleOffset]` into the
    caller's requested output count. Clamps to most-recent 1020
    samples if the producer has lapped us, holds last sample on
    empty (no click). Device push (`device/snes_run.c`) raised
    from 1024-sample cap to 3072 so the PWM ring (4096 slots,
    ~185 ms) stays primed between pulls.

35. **Wall-clock throttle SPC on core 1** (`src/snes_core.c`,
    `snes_apu_core1_loop`). L34 alone still wobbled: the producer
    (SPC on core 1) ran at core 1's native rate, roughly 20-30×
    real-time when PPU had slack, lapping the DSP ring many
    times per pull. The clamp in L34 then always read "most
    recent 1020 samples" = fixed input window, variable output
    window → ratio still wobbled.
    Fix: pace `apu->cycles` growth against `time_us_64()` so
    SPC executes at the real 1,025,280 APU cycles/sec. When
    `apu->cycles > cycles_ref + wall_elapsed_us * 1025280 / 1e6`,
    the loop idles; otherwise `spc_runOpcode` runs.
    Reference captured on ROM load (`s_apu_core1_snes` going
    null → non-null) and reset on unload. When PPU starves SPC
    (heavy scenes), the throttle simply doesn't fire and SPC
    runs flat-out — audio tempo slows gracefully but pitch
    stays correct because the resample ratio tracks actual
    production rate consistently. Requires `pico_time` headers
    exposed to `lakesnes_dev` via
    `INTERFACE_INCLUDE_DIRECTORIES` (linking the library would
    pull in its .c files which won't compile without
    `pico_sdk_init`).

### Post-session numbers (host, 900 frames, 3-run mean, --xip)

| Scene (ROM, content)               | Pre-31-33 | Post-31-33 | Δ     |
|------------------------------------|----------:|-----------:|------:|
| FFII (title/map, light)            |  555 fps  |    893 fps | **+61%** |
| Zelda ALTTP (medium)               |  522 fps  |    708 fps | **+36%** |
| Chrono Trigger (animated title)    |  664 fps  |    800 fps | **+20%** |
| SMW (heavy overworld)              |  344 fps  |    435 fps | **+26%** |
| Super Metroid (heavy atmospheric)  |  527 fps  |    679 fps | **+29%** |

**The improvement is asymmetric** — light scenes gain 50-60%,
heavy scenes only 20-30%. Why: the three levers hit per-opcode
overhead (L1, L2) and per-line compositor work (L3). Heavy
scenes spend proportionally more time on things these levers
don't touch:

- **More sprites per line** → more OAM walks and tile decodes in
  `ppu_evaluateSprites` (unchanged)
- **More active BG layers** → L3's "2 tilemap walks per BG layer"
  overhead multiplies (more walks than the old one-walk-
  two-priorities pattern), partly cancelling the bgLine-skip win
- **More unique tiles per line** → lower tile-cache hit rate (#27),
  more decode work, which our extra walks amplify
- **More VRAM traffic** from HDMA / mid-frame DMAs

The next-big-lever list below is now ordered with heavy-scene
impact in mind.

---

## Known gaps (NOT yet attempted — ordered for heavy-scene impact)

Post-L33 the bottleneck profile has NOT been re-taken. The items
below are guesses; whichever ones actually matter will become
clear only from a fresh profile on a heavy-content scene (SMW
overworld, Zelda castle interior with rain effects, FFII combat
with full sprite set).

- **Sprite eval rewrite.** `ppu_evaluateSprites` walks all 128
  OAM entries per scanline, re-deriving sprite range, tile
  index, row offset, and priority from scratch. On heavy-
  sprite scenes it's the per-line cost that scales fastest with
  complexity. Candidates: share a per-line "sprites-in-range"
  index across priority slots, sprite-tile cache paralleling
  the BG tile cache (#27), or move sprite eval to core 1 ahead
  of the composite.
- **Single-walk tile-major composite.** L33 walks the tilemap
  twice per BG layer (once per priority slot). A single-walk
  variant would emit to two output "strata" buffers in one pass,
  or use a mask + top-down walk with early-exit when the mask
  fills. Complexity is real; gains should show up most on
  layer-dense scenes where the extra walk currently costs.
- **Accuracy drops (speed-over-fidelity project policy).**
  Mode 7 rendered as flat BG, mosaic disabled, hires modes 5/6
  collapsed to mode 1, BGRX legacy path stubbed out. Zero-risk
  toggles; moderate win on specific games.
- **Inline WRAM fast path in top-4 ASM opcodes.** LDA/STA
  imm+abs+dp hit `.Lrd`/`.Lwr` via BLX even when the bus
  access is a plain WRAM/ROM readMap hit. Inlining that check
  saves ~6-8 cycles per opcode on ~25% of executed ops.
  Estimated +1-2 fps. Careful ASM register management required.
- **Cross-core pipelining beyond APU.** Currently core 1 runs
  SPC+DSP + PPU line composite (via `s_ppu_pipeline_line`). Sprite
  eval could be shipped earlier as a producer ahead of the
  compositor. Tricky timing and cross-core memory traffic; may
  or may not win.
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
