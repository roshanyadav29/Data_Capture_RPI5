#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// Target sampling frequency
#define TARGET_SAMPLE_RATE 8192000 // 8.192 MHz

// SPI configuration parameters
#define SPI_SPEED 10000000  // 10 MHz (higher than required 8.192 MHz for headroom)
#define SPI_MODE 0          // SPI mode 0
#define SPI_CS_PIN 8        // Chip select pin
#define SPI_BITS_PER_WORD 8 // Bits per SPI word
#define SPI_DEVICE "/dev/spidev0.0"  // SPI device path

// DMA buffer configuration
#define DMA_BUFFER_SIZE (1024 * 1024)  // 1MB chunks
#define NUM_DMA_BUFFERS 8              // Total buffering of 8MB before flush

// External clock configuration
#define USE_EXTERNAL_CLOCK true
#define CLOCK_PIN 18  // GPIO pin for external clock input

// Thread sleep times
#define THREAD_SLEEP_US 1000  // 1ms sleep in worker thread

// Storage configuration
#define SD_CARD_PATH "/mnt/sdcard/data.txt"

// Buffer management
#define BUFFER_SIZE 1024  // Size for general purpose buffers

#endif // CONFIG_H