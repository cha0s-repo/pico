/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pio_sdio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// This program instantiates a PIO SPI with each of the four possible
// CPOL/CPHA combinations, with the serial input and output pin mapped to the
// same GPIO. Any data written into the state machine's TX FIFO should then be
// serialised, deserialised, and reappear in the state machine's RX FIFO.
// #define TEST_SDIO   

#define PIN_CMD 11
#define PIN_SCK 12
#define PIN_DAT1 13 

#define PIN_CMD_CAP     6
#define PIN_SCK_CAP     7
#define PIN_DAT1_CAP    8 

#define DMA_CHAN        0

#define BUF_SIZE        6
#define WRAP_BIT        8
#define CAP_SIZE        (1<<WRAP_BIT)
#define SDIO_SIZE       1024

static uint8_t cap_buf[CAP_SIZE];

static uint32_t dma_start;
static uint32_t dma_fit_len = CAP_SIZE;
static uint32_t dma_read_p ;
static bool     dma_need_consumed = false;

static uint32_t test_cnt;

static pio_sdio_inst_t sdio_cmd = {
        .pio = pio0,
        .sm = 0
};

static pio_sdio_inst_t sdio_cmd_cap = {
        .pio = pio0,
        .sm = 1
};

static dma_channel_config *dma_cfg;

static void send_data(const pio_sdio_inst_t *sdio) {
    static uint8_t txbuf[BUF_SIZE];

    for (int i = 0; i < BUF_SIZE; i++) {
        txbuf[i] = test_cnt++;
    }

    pio_sm_clear_fifos(sdio_cmd.pio, sdio_cmd.sm);
    pio_sm_restart(sdio_cmd.pio, sdio_cmd.sm);

    pio_spi_write8_blocking(sdio, txbuf, BUF_SIZE);
}

static void reconfig_dma(uint32_t start, uint32_t len) {

}

static void dma_handler() {
        // Clear the interrupt request.
    dma_hw->ints0 = 1u << DMA_CHAN;

    uint32_t start = dma_start + dma_fit_len;
    int len =  CAP_SIZE - (start - dma_read_p);

    if (len <= 0 ) {
        // wait for data consumed
        dma_need_consumed = true;
        
        return;
    }

    // Give the channel a new wave table entry to read from, and re-trigger it
    dma_channel_set_write_addr(DMA_CHAN, &cap_buf[start%CAP_SIZE], true);
    dma_channel_set_trans_count(DMA_CHAN, len, true);
    dma_start = start;
    dma_fit_len = len;
}

static void dma_init() {
    *dma_cfg = dma_channel_get_default_config(DMA_CHAN);
    channel_config_set_read_increment(dma_cfg, false);
    channel_config_set_write_increment(dma_cfg, true);
    channel_config_set_dreq(dma_cfg, pio_get_dreq(sdio_cmd_cap.pio, sdio_cmd_cap.sm, false));
    channel_config_set_transfer_data_size(dma_cfg, DMA_SIZE_8);
    channel_config_set_ring(dma_cfg, true, WRAP_BIT);

    dma_channel_configure(DMA_CHAN, dma_cfg,
        cap_buf,        // Destination pointer
        &(sdio_cmd_cap.pio)->rxf[sdio_cmd_cap.sm],      // Source pointer
        CAP_SIZE, // Number of transfers
        true                // Start immediately
    );
    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(DMA_CHAN, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    pio_sm_set_enabled(sdio_cmd_cap.pio, sdio_cmd_cap.sm, true);
}

static int dma_rx_remains() {
    return dma_channel_get_trans_count(DMA_CHAN);
}

static void print_buf() {
    int consume = 6;
    int dma_rem = dma_rx_remains();
    int start_bit = 0;
    int dir = 0;
    int cmd_no = 0;
    int cmd_data = 0;

    uint32_t csm_end = dma_start + (dma_fit_len - dma_rem);
    if ((dma_read_p + consume ) > csm_end) {
        return;
    }
    uint32_t raw = dma_read_p % CAP_SIZE;

    start_bit = cap_buf[raw] >> 7 & 1;
    dir = cap_buf[raw] >> 6 & 1;
    cmd_no = cap_buf[raw] & (0x3f);

    cmd_data = (cap_buf[(raw+1)% CAP_SIZE] << 8) + cap_buf[(raw+2)% CAP_SIZE];

    dma_read_p += consume;
    printf("sdio cmd: %d, dir %d, cmd code %d, data %d\n", start_bit, dir, cmd_no, cmd_data);

    if (dma_need_consumed) {
        
        dma_need_consumed = false;
        dma_handler();
    }
}

int main() {
    stdio_init_all();
    gpio_set_dir(PIN_CMD_CAP, false);
    gpio_set_dir(PIN_SCK_CAP, false);
    gpio_pull_up(PIN_CMD_CAP);
    gpio_pull_up(PIN_SCK_CAP);
    sleep_ms(5000);
    printf("start....\n");

#ifdef TEST_SDIO
    float clkdiv = 31.25f;  // 1 MHz @ 125 clk_sys
    uint sdio_cmd_offs = pio_add_program(sdio_cmd.pio, &sdio_cmd_program);
    uint cpha = 1;          // clock stay hi when idle
    uint cpol = 0;          // down edge enable

    printf("CPHA = %d, CPOL = %d\n", cpha, cpol);
    pio_sdio_cmd_init(sdio_cmd.pio, sdio_cmd.sm,
                    sdio_cmd_offs,
                    8,       // 8 bits per OSR
                    clkdiv,
                    cpha,
                    cpol,
                    PIN_SCK,
                    PIN_CMD
    );
#endif
    uint sdio_cmd_cap_offs = pio_add_program(sdio_cmd_cap.pio, &sdio_cmd_cap_program);

    pio_sdio_cmd_cap_init(sdio_cmd_cap.pio, sdio_cmd_cap.sm, sdio_cmd_cap_offs, PIN_CMD_CAP, 1.0f);
 
    dma_init();
    while (1) {
#ifdef TEST_SDIO
        send_data(&sdio_cmd);
#endif
        sleep_ms(500);
        
        print_buf();
        
    }
}
