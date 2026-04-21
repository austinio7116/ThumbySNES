/*
 * XIP mode introspection.
 *
 * The QMI block on RP2350 has per-chip-select read-format registers
 * (qmi_hw->m[0].rfmt for flash on CS0). The RFMT field layout encodes
 * the command prefix / suffix widths, dummy-cycle count, and data
 * width. Fast-read-quad (command 0xEB) sets:
 *     DATA_WIDTH   = 2 (Q)    — 4 bits per cycle on data phase
 *     ADDR_WIDTH   = 2 (Q)
 *     SUFFIX_WIDTH = 2 (Q)    — M-phase continuation-mode byte
 *     DUMMY_WIDTH  = 2 (Q)
 *     DUMMY_LEN    = 4        — W25Q needs 4 dummy cycles @ quad width
 *
 * Default post-boot 0x03 single-read:
 *     DATA_WIDTH = ADDR_WIDTH = DUMMY_WIDTH = 0 (S)
 *     DUMMY_LEN  = 0
 *
 * Reading the register is cheap and safe from any thread.
 */

#include "snes_xip.h"

#include <string.h>
#include "hardware/structs/qmi.h"
#include "hardware/regs/qmi.h"
#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"
#include "pico/bootrom_constants.h"

uint32_t snes_xip_read_rfmt(void)
{
    return qmi_hw->m[0].rfmt;
}

int snes_xip_data_width(void)
{
    return (int)((qmi_hw->m[0].rfmt & QMI_M0_RFMT_DATA_WIDTH_BITS)
                 >> QMI_M0_RFMT_DATA_WIDTH_LSB);
}

const char *snes_xip_mode_desc(void)
{
    switch (snes_xip_data_width()) {
        case 0: return "single (slow)";
        case 1: return "dual";
        case 2: return "quad (fast)";
        default: return "reserved";
    }
}

/* Fast QPI XIP setup — ported from ThumbyOne's thumbyone_xip_fast_setup.
 *
 * Matches boot2_w25q080.S's INIT values (W25Q080 / W25Q16JV / AT25SF081
 * compatible). The critical extra step (vs naively writing RFMT) is
 * the W25Q 66h/99h flash reset FIRST, so flash comes out of whatever
 * state it's in (continuous read, mid-transaction, …) and enters from
 * a clean power-on state where the 0xEB prefix byte is correctly
 * parsed as a command rather than as address/mode bits.
 *
 * Runs from RAM (__not_in_flash_func, noinline) since it reconfigures
 * the very QMI block the CPU is fetching instructions through. */
/* Matches ThumbyOne / boot2_w25q080 defaults. Constructor runs at
 * sys_clk = 150 MHz → 75 MHz flash clock. After main's clock bump
 * to 300 MHz, flash clock becomes 150 MHz (above W25Q080's 104 MHz
 * quad spec) — but typical silicon runs fine at that, same as
 * ThumbyOne at 250 MHz (125 MHz flash). */
#define TBY_FLASH_SPI_CLKDIV   2
#define TBY_FLASH_SPI_RXDELAY  2
#define TBY_CMD_READ_FAST_QPI  0xEB
#define TBY_MODE_CONT_READ     0xA0
#define TBY_WAIT_CYCLES        4

#define TBY_M0_TIMING (                                        \
    (1u                     << QMI_M0_TIMING_COOLDOWN_LSB) |   \
    (TBY_FLASH_SPI_RXDELAY  << QMI_M0_TIMING_RXDELAY_LSB)  |   \
    (TBY_FLASH_SPI_CLKDIV   << QMI_M0_TIMING_CLKDIV_LSB))

#define TBY_M0_RCMD (                                          \
    (TBY_CMD_READ_FAST_QPI  << QMI_M0_RCMD_PREFIX_LSB) |       \
    (TBY_MODE_CONT_READ     << QMI_M0_RCMD_SUFFIX_LSB))

#define TBY_M0_RFMT_WITH_PREFIX (                                              \
    (QMI_M0_RFMT_PREFIX_WIDTH_VALUE_S << QMI_M0_RFMT_PREFIX_WIDTH_LSB) |       \
    (QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q   << QMI_M0_RFMT_ADDR_WIDTH_LSB)   |       \
    (QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB) |       \
    (QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q  << QMI_M0_RFMT_DUMMY_WIDTH_LSB)  |       \
    (QMI_M0_RFMT_DATA_WIDTH_VALUE_Q   << QMI_M0_RFMT_DATA_WIDTH_LSB)   |       \
    (QMI_M0_RFMT_PREFIX_LEN_VALUE_8   << QMI_M0_RFMT_PREFIX_LEN_LSB)   |       \
    (QMI_M0_RFMT_SUFFIX_LEN_VALUE_8   << QMI_M0_RFMT_SUFFIX_LEN_LSB)   |       \
    (TBY_WAIT_CYCLES                  << QMI_M0_RFMT_DUMMY_LEN_LSB))

uint32_t __not_in_flash_func(snes_xip_rerun_boot2)(void)
    __attribute__((noinline));

uint32_t __not_in_flash_func(snes_xip_rerun_boot2)(void) {
    uint32_t ints = save_and_disable_interrupts();

    /* Step 1: reset flash out of whatever state it's in. Enable QMI
     * direct mode with a conservative clock (CLKDIV=30 ~= 5 MHz @
     * 150 MHz clk_sys). */
    qmi_hw->direct_csr = (30u << QMI_DIRECT_CSR_CLKDIV_LSB)
                       | QMI_DIRECT_CSR_EN_BITS
                       | QMI_DIRECT_CSR_AUTO_CS0N_BITS;
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {
        __asm__ volatile("nop");
    }

    /* 66h (enable-reset), then 99h (reset). These are recognised at
     * any bus width so this works whether flash was in dual/quad. */
    qmi_hw->direct_tx = 0x66u;
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {
        __asm__ volatile("nop");
    }
    (void)qmi_hw->direct_rx;
    qmi_hw->direct_tx = 0x99u;
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {
        __asm__ volatile("nop");
    }
    (void)qmi_hw->direct_rx;

    /* W25Q datasheet: ~30 µs tRST after the reset command. Burn
     * 100 µs to be safe. */
    for (volatile int i = 0; i < 3000; ++i) { __asm__ volatile("nop"); }

    qmi_hw->direct_csr = 0;

    /* Step 2: configure fast QPI XIP on the now-clean flash. */
    qmi_hw->m[0].timing = TBY_M0_TIMING;
    qmi_hw->m[0].rcmd   = TBY_M0_RCMD;
    qmi_hw->m[0].rfmt   = TBY_M0_RFMT_WITH_PREFIX;

    /* Dummy read puts flash into continuous-read mode with the A0h
     * mode bits latched. */
    volatile uint32_t dummy = *(volatile uint32_t *)XIP_NOCACHE_NOALLOC_BASE;
    (void)dummy;

    /* Drop the command prefix — flash now expects continuous-read
     * transactions without the 0xEB byte. */
    qmi_hw->m[0].rfmt = TBY_M0_RFMT_WITH_PREFIX
                       & ~QMI_M0_RFMT_PREFIX_LEN_BITS;
    __asm__ volatile("dsb" ::: "memory");

    restore_interrupts(ints);
    return snes_xip_read_rfmt();
}
