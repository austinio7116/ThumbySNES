#
# ThumbySNES — device firmware build.
#
# Build:
#   cd ThumbySNES
#   cmake -B build_device -S device \
#         -DPICO_SDK_PATH=/home/maustin/mp-thumby/lib/pico-sdk
#   cmake --build build_device -j8
#
# Output: build_device/snesrun_device.uf2
# Flash:  hold DOWN dpad while powering on to enter BOOTSEL,
#         then drag the .uf2 onto the RPI-RP2350 drive.
#
cmake_minimum_required(VERSION 3.16)

set(PICO_BOARD tinycircuits_thumby_color_rp2350)
set(PICO_PLATFORM rp2350)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(thumbysnes_device C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

set(SNES_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

# ----------------------------------------------------------------------
# Vendored snes9x2002 — only linked in Phase 4+. Phase 3 firmware is
# emulator-less (snes_run.c is a stub splash) so we skip compiling the
# ~60-file core on every device build until we actually call into it.
# Set THUMBYSNES_LINK_CORE=ON at CMake time to include it:
#   cmake -B build_device -S device -DTHUMBYSNES_LINK_CORE=ON ...
# ----------------------------------------------------------------------
option(THUMBYSNES_LINK_CORE "Link the snes9x2002 core into the firmware" OFF)

if(THUMBYSNES_LINK_CORE)
    set(S9X_DIR    ${SNES_ROOT}/vendor/snes9x2002)
    set(S9X_SRC    ${S9X_DIR}/src)
    set(S9X_COMMON ${S9X_DIR}/libretro/libretro-common)

    set(SNES9X2002_CORE_SRC
        ${S9X_SRC}/apu.c          ${S9X_SRC}/apuaux.c
        ${S9X_SRC}/c4.c           ${S9X_SRC}/c4emu.c
        ${S9X_SRC}/clip.c         ${S9X_SRC}/data.c
        ${S9X_SRC}/dsp1.c         ${S9X_SRC}/fxemu.c
        ${S9X_SRC}/fxinst.c       ${S9X_SRC}/globals.c
        ${S9X_SRC}/dma.c          ${S9X_SRC}/memmap.c
        ${S9X_SRC}/cpu.c          ${S9X_SRC}/cpuexec.c
        ${S9X_SRC}/cpuops.c       ${S9X_SRC}/sdd1.c
        ${S9X_SRC}/sdd1emu.c      ${S9X_SRC}/snapshot.c
        ${S9X_SRC}/soundux.c      ${S9X_SRC}/spc700.c
        ${S9X_SRC}/srtc.c
        ${S9X_SRC}/ppu_.c         ${S9X_SRC}/gfx.c
        ${S9X_SRC}/tile.c
        ${S9X_COMMON}/streams/memory_stream.c
        ${SNES_ROOT}/src/snes_core.c
    )

    add_library(snes9x2002_dev STATIC ${SNES9X2002_CORE_SRC})
    target_include_directories(snes9x2002_dev PUBLIC
        ${S9X_SRC}
        ${S9X_DIR}/libretro
        ${S9X_COMMON}/include
        ${SNES_ROOT}/src
    )
    target_compile_definitions(snes9x2002_dev PUBLIC
        __LIBRETRO__
        HAVE_STDINT_H
        HAVE_INTTYPES_H
        HAVE_STRINGS_H
        USE_SA1
        __OLD_RASTER_FX__
        THUMBYSNES_HAVE_CORE=1
        # Place hot loops in SRAM for flash-XIP cache-miss avoidance.
        # Device-only — on host builds this expands to nothing.
        "IRAM_ATTR=__attribute__((section(\".time_critical.snes\")))"
    )
    target_compile_options(snes9x2002_dev PRIVATE
        -O3 -fomit-frame-pointer -ffast-math -fstrict-aliasing
        -ffunction-sections -fdata-sections
        -Wno-array-bounds -Wno-format -Wno-unused-parameter
        -Wno-unused-function -Wno-unused-variable
        -Wno-unused-but-set-variable -Wno-sign-compare
        -Wno-pointer-sign -Wno-implicit-function-declaration
        -Wno-parentheses
    )
endif()

# ----------------------------------------------------------------------
# snesrun_device — Phase 3 firmware: lobby + picker + stub emulator.
# ----------------------------------------------------------------------
add_executable(snesrun_device
    snes_device_main.c
    snes_lcd_gc9107.c
    snes_buttons.c
    snes_audio_pwm.c
    snes_flash_disk.c
    snes_msc.c
    snes_picker.c
    snes_font.c
    snes_battery.c
    snes_run.c
    usb_descriptors.c
    fatfs/ff.c
    fatfs/ffsystem.c
    fatfs/ffunicode.c
    fatfs/diskio.c
)
target_include_directories(snesrun_device PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/fatfs
)
target_compile_options(snesrun_device PRIVATE
    -O2 -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable
    -Wno-unused-but-set-variable
    -ffunction-sections -fdata-sections
)
target_link_options(snesrun_device PRIVATE -Wl,--gc-sections)
target_link_libraries(snesrun_device PRIVATE
    pico_stdlib
    pico_unique_id
    hardware_clocks
    hardware_spi
    hardware_dma
    hardware_gpio
    hardware_pwm
    hardware_irq
    hardware_flash
    hardware_sync
    hardware_adc
    tinyusb_device
    tinyusb_board
)
if(THUMBYSNES_LINK_CORE)
    target_link_libraries(snesrun_device PRIVATE snes9x2002_dev)
endif()
pico_enable_stdio_usb(snesrun_device 0)
pico_enable_stdio_uart(snesrun_device 0)
pico_add_extra_outputs(snesrun_device)
