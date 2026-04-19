#
# ThumbySNES — device firmware build (LakeSnes core).
#
# Build:
#   cd ThumbySNES
#   cmake -B build_device -S device \
#         -DPICO_SDK_PATH=/home/maustin/mp-thumby/lib/pico-sdk \
#         -DTHUMBYSNES_LINK_CORE=ON
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
# Vendored LakeSnes (MIT — see vendor/lakesnes/LICENSE.txt + VENDORING.md)
# Phase 4 default OFF so picker-only firmware iterations don't recompile
# the ~6500 LOC core. Pass -DTHUMBYSNES_LINK_CORE=ON for the playable
# firmware.
# ----------------------------------------------------------------------
option(THUMBYSNES_LINK_CORE "Link the LakeSnes core into the firmware" OFF)
# Diagnostic: force the per-opcode C dispatcher (cpu_runOpcode) instead of
# the Thumb-2 ASM dispatcher (cpu_runBatchAsm) on device builds. Used to
# bisect device-only bugs — if a game works on this build but not the
# normal one, the bug is in cpu_asm.c. OFF for production.
option(THUMBYSNES_FORCE_C_DISPATCH "Force C opcode dispatcher on device (diagnostic)" OFF)

if(THUMBYSNES_LINK_CORE)
    set(LK_SRC ${SNES_ROOT}/vendor/lakesnes/snes)
    set(LAKESNES_CORE_SRC
        ${LK_SRC}/apu.c          ${LK_SRC}/cart.c
        ${LK_SRC}/cpu.c          ${LK_SRC}/dma.c
        ${LK_SRC}/dsp.c          ${LK_SRC}/input.c
        ${LK_SRC}/ppu.c          ${LK_SRC}/snes.c
        ${LK_SRC}/snes_other.c   ${LK_SRC}/spc.c
        ${LK_SRC}/statehandler.c
        ${SNES_ROOT}/src/snes_core.c
        ${SNES_ROOT}/src/cpu_asm.c
        # cpu_fast.c is #included from cpu.c (same TU for inlining)
    )

    add_library(lakesnes_dev STATIC ${LAKESNES_CORE_SRC})
    # snes_apu_core1_loop needs time_us_64 for the wall-clock SPC
    # throttle (Stage 2 audio fix). Include pico_time headers (and
    # its transitive dependency headers) without linking the library
    # — snesrun_device brings in the real implementation via
    # pico_stdlib below.
    target_include_directories(lakesnes_dev PUBLIC
        ${LK_SRC} ${SNES_ROOT}/src
        $<TARGET_PROPERTY:pico_time,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:hardware_timer,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:hardware_base,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:pico_base_headers,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:hardware_sync,INTERFACE_INCLUDE_DIRECTORIES>
    )
    target_compile_definitions(lakesnes_dev PUBLIC
        THUMBYSNES_HAVE_CORE=1
        # LAKESNES_HOT macro → puts marked functions in .time_critical.snes
        # which the Pico SDK linker copies to SRAM at boot. Avoids flash
        # XIP cache misses on the inner emulator loop.
        LAKESNES_HOT_SRAM=1
        # Dual-core: SPC700 + DSP run on core 1, CPU + PPU on core 0.
        # snes_catchupApu compiles to a no-op; snes_apu_core1_loop is
        # the core 1 entry point.
        THUMBYSNES_DUAL_CORE=1
        # Devirtualize CPU read/write/idle: direct BL to snes_cpuRead
        # instead of indirect BLX through function pointer. Saves ~3-5
        # cycles per call across ~1.8M calls/frame.
        THUMBYSNES_DIRECT_CPU_CALLS=1
    )
    if(THUMBYSNES_FORCE_C_DISPATCH)
        target_compile_definitions(lakesnes_dev PUBLIC
            THUMBYSNES_FORCE_C_DISPATCH=1
        )
        message(STATUS "ThumbySNES: forcing C opcode dispatcher (diagnostic build)")
    endif()
    target_compile_options(lakesnes_dev PRIVATE
        -O3 -fomit-frame-pointer -fstrict-aliasing
        -ffunction-sections -fdata-sections
        -Wno-unused-parameter -Wno-unused-but-set-variable
    )
endif()

# ----------------------------------------------------------------------
# snesrun_device firmware
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
    pico_multicore
)
if(THUMBYSNES_LINK_CORE)
    target_link_libraries(snesrun_device PRIVATE lakesnes_dev)
endif()
pico_enable_stdio_usb(snesrun_device 0)
pico_enable_stdio_uart(snesrun_device 0)
pico_add_extra_outputs(snesrun_device)
