/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pio_sdio.h"

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

#define BUF_SIZE 6

void test(const pio_sdio_inst_t *spi) {
    static uint8_t txbuf[BUF_SIZE];
    txbuf[0] = 0x55;
    txbuf[1] = 0x12;
    txbuf[2] = 0x34;
    txbuf[3] = 0x56;
    txbuf[4] = 0x78;
    txbuf[5] = 0x99;

    pio_spi_write8_blocking(spi, txbuf, BUF_SIZE);

}

int main() {
    static uint8_t rxbuf[BUF_SIZE];
    stdio_init_all();
    sleep_ms(5000);
    printf("start....\n");
    pio_sdio_inst_t sdio_cmd = {
            .pio = pio0,
            .sm = 0
    };

    pio_sdio_inst_t sdio_cmd_cap = {
            .pio = pio0,
            .sm = 1
    };
    float clkdiv = 125.0f;  // 1 MHz @ 125 clk_sys
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
 
    while (1) {
        //test(&sdio_cmd);
        for (int i = 0; i < BUF_SIZE; i++) {
            rxbuf[i] = pio_sm_get_blocking(sdio_cmd_cap.pio, sdio_cmd_cap.sm);
            printf("%02x ", rxbuf[i]);
        }
        //printf("\ndone\n");
        //sleep_ms(1000);
    }
}
