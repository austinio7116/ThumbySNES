# ThumbySNES

A bare-metal SNES emulator firmware for the [Thumby Color](https://thumby.us/).
Flashes as a single `.uf2`, exposes the device as a USB drive for dragging
`.smc` / `.sfc` ROMs onto it, and runs them with a ROM picker and in-game
menu — the same shape as [ThumbyNES](../ThumbyNES).

**Status**: scaffold only. See [PLAN.md](PLAN.md) for the full design and
phased build order.

## Build (host, Phase 0 / 1)

```bash
cmake -B build -S .
cmake --build build -j8
./build/snesbench path/to/rom.smc 600     # Phase 0: headless N-frame bench
./build/sneshost  path/to/rom.smc         # Phase 1: SDL2 runner (requires libsdl2-dev)
```

## Build (device, Phase 3+)

```bash
cmake -B build_device -S device \
      -DPICO_SDK_PATH=/home/maustin/mp-thumby/lib/pico-sdk
cmake --build build_device -j8
# Output: build_device/snesrun_device.uf2
```

## License

ThumbySNES glue: see LICENSE (matches snes9x's non-commercial terms for
redistribution compatibility).

Vendored `snes9x2002` is covered by the snes9x license — see
`vendor/snes9x2002/LICENSE` once vendored.
