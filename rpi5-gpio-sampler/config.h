#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// Target sampling frequency
#define TARGET_SAMPLE_RATE 8192000 // 8.192 MHz

// GPIO configuration parameters
#define GPIO_DATA_PIN 17     // GPIO pin for data input
#define GPIO_CLOCK_PIN 18    // GPIO pin for external clock input
#define GPIO_BASE 0xFE200000 // Raspberry Pi 5's GPIO base address

// DMA buffer configuration
#define DMA_BUFFER_SIZE (1024 * 1024)  // 1MB chunks
#define NUM_DMA_BUFFERS 8              // Total buffering of 8MB before flush

// Thread sleep times
#define THREAD_SLEEP_US 1000  // 1ms sleep in worker thread

// Storage configuration
#define SD_CARD_PATH "/home/pi/gpio_data"  // Directory to store data files

// Buffer management
#define BUFFER_SIZE 1024  // Size for general purpose buffers

// PCM registers base address (for PCM clock option)
#define PCM_BASE 0xFE203000

#endif // CONFIG_H