#ifndef SPI_HANDLER_H
#define SPI_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../config.h"

// Function prototypes - Basic SPI
void spi_init(void);
uint8_t spi_read_byte(void);
void spi_write_byte(uint8_t data);
void spi_transfer(uint8_t *tx_buffer, uint8_t *rx_buffer, size_t length);

// High-speed capture functions
bool spi_dma_init(void);
bool spi_start_capture(void (*callback)(uint8_t*, size_t));
bool spi_stop_capture(void);
size_t spi_get_buffer_usage(void);

// External clock control
bool spi_sync_to_external_clock(void);
uint32_t spi_get_actual_sample_rate(void);

#endif // SPI_HANDLER_H