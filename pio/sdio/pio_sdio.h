/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _PIO_SDIO_H
#define _PIO_SDIO_H

#include "hardware/pio.h"
#include "sdio.pio.h"

typedef struct pio_sdio_inst {
    PIO pio;
    uint sm;
    uint cs_pin;
} pio_sdio_inst_t;

void pio_spi_write8_blocking(const pio_sdio_inst_t *spi, const uint8_t *src, size_t len);

void pio_spi_read8_blocking(const pio_sdio_inst_t *spi, uint8_t *dst, size_t len);

void pio_spi_write8_read8_blocking(const pio_sdio_inst_t *spi, uint8_t *src, uint8_t *dst, size_t len);

#endif