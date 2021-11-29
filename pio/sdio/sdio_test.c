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

#define PIN_CMD 11
#define PIN_SCK 12
#define PIN_DAT1 13 

#define PIN_CMD_CAP     6
#define PIN_SCK_CAP     7
#define PIN_DAT1_CAP    8 

#define DMA_CHAN        0

#define BUF_SIZE        6
#define WRAP_BIT        4
#define CAP_SIZE        (1<<WRAP_BIT)
#define SDIO_SIZE       1024

static uint8_t cap_buf[CAP_SIZE];
static uint8_t sdio_buf[SDIO_SIZE];
static uint32_t sdio_w_ptr, sdio_cnt;
static uint32_t dma_cnt;
static int32_t dma_last_cnt = -1;
static uint32_t dma_r_ptr;

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
    txbuf[0] = 0x55;
    txbuf[1] = 0x12;
    txbuf[2] = 0x34;
    txbuf[3] = 0x56;
    txbuf[4] = 0x78;
    txbuf[5] = 0x99;

    pio_sm_clear_fifos(sdio_cmd.pio, sdio_cmd.sm);
    pio_sm_restart(sdio_cmd.pio, sdio_cmd.sm);

    pio_spi_write8_blocking(sdio, txbuf, BUF_SIZE);
}

static int sdio_buf_push(const uint8_t *start, uint32_t len) {
    int i = 0;
    uint32_t fit_size = (sdio_cnt + len) > SDIO_SIZE ? (SDIO_SIZE - sdio_cnt) : len;
    for (i = 0; i < fit_size; i++) {
        if (sdio_w_ptr >= SDIO_SIZE) sdio_w_ptr = 0;
        sdio_buf[sdio_w_ptr++] = start[i];
    }
    sdio_cnt += fit_size;
    return i;
}

static void dma_handler() {

    // Clear the interrupt request.
    dma_hw->ints0 = 1u << DMA_CHAN;
    // Give the channel a new wave table entry to read from, and re-trigger it
    dma_channel_set_write_addr(DMA_CHAN, cap_buf, true);
    dma_channel_set_trans_count(DMA_CHAN, CAP_SIZE, true);
    
    dma_cnt++;
    dma_last_cnt = -1;
    sdio_buf_push(&cap_buf[dma_r_ptr], CAP_SIZE - dma_r_ptr);
    dma_r_ptr = 0;
}

static void dma_init() {
    *dma_cfg = dma_channel_get_default_config(DMA_CHAN);
    channel_config_set_read_increment(dma_cfg, false);
    channel_config_set_write_increment(dma_cfg, true);
    channel_config_set_dreq(dma_cfg, pio_get_dreq(sdio_cmd_cap.pio, sdio_cmd_cap.sm, false));
    channel_config_set_transfer_data_size(dma_cfg, DMA_SIZE_8);

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
    uint32_t c = sdio_cnt;
    sdio_cnt = 0;
    int start = sdio_w_ptr - c;
    if (start < 0) {
        start += SDIO_SIZE;
    }
    for (int i = 0; i < c; i++) {
        if (start >= SDIO_SIZE) start = 0;
        printf(" %02x", sdio_buf[start++]);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(5000);
    printf("start....\n");

    float clkdiv = 31.25f;  // 1 MHz @ 125 clk_sys
    uint sdio_cmd_offs = pio_add_program(sdio_cmd.pio, &sdio_cmd_program);
    uint sdio_cmd_cap_offs = pio_add_program(sdio_cmd_cap.pio, &sdio_cmd_cap_program);
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

    pio_sdio_cmd_cap_init(sdio_cmd_cap.pio, sdio_cmd_cap.sm, sdio_cmd_cap_offs, PIN_CMD_CAP, 1.0f);
 
    dma_init();
    while (1) {
        send_data(&sdio_cmd);
        sleep_ms(500);

        if (dma_last_cnt == dma_cnt) {
            // in this period dma did not fetch data, read remains data out
            uint32_t ldata = CAP_SIZE - dma_rx_remains();
            if (ldata > dma_r_ptr) {
                sdio_buf_push(&cap_buf[dma_r_ptr], ldata - dma_r_ptr);
                dma_r_ptr = ldata;
            }
        } 
        dma_last_cnt = dma_cnt;
        
        print_buf();
        
    }
}
