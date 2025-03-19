#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// Target sampling frequency
#define TARGET_SAMPLE_RATE 8192000 // 8.192 MHz

// Calculate buffer size for 1 minute of data
// (8.192 MHz / 8 bits per byte) * 60 seconds = ~61.44 MB
#define BYTES_PER_SECOND (TARGET_SAMPLE_RATE / 8)
#define CAPTURE_SECONDS 300
#define TOTAL_BUFFER_SIZE (BYTES_PER_SECOND * CAPTURE_SECONDS)

// Add 10% safety margin
#define RAM_BUFFER_SIZE (TOTAL_BUFFER_SIZE + (TOTAL_BUFFER_SIZE / 10))

// GPIO configuration parameters
#define GPIO_DATA_PIN 17     // GPIO pin for data input
#define GPIO_CLOCK_PIN 18    // GPIO pin for external clock input
#define GPIO_BASE 0xFE200000 // Raspberry Pi 5's GPIO base address

// Storage configuration
#define SD_CARD_PATH "/home/pi/gpio_data"  // Directory to store data files

// Thread sleep times
#define THREAD_SLEEP_US 1000  // 1ms sleep in worker thread

// General buffer size for temporary operations
#define BUFFER_SIZE 4096  // Size for general purpose buffers

#endif // CONFIG_H