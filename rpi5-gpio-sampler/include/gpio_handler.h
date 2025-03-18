#ifndef GPIO_HANDLER_H
#define GPIO_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../config.h"

typedef enum {
    CLOCK_SOURCE_EXTERNAL, // Use external clock on GPIO pin
    CLOCK_SOURCE_PCM       // Use internal PCM hardware clock
} ClockSource;

// Function prototypes - GPIO setup
void gpio_init(ClockSource clock_source);
void gpio_cleanup(void);

// High-speed capture functions
bool gpio_dma_init(void);
bool gpio_start_capture(void (*callback)(uint8_t*, size_t));
bool gpio_stop_capture(void);
size_t gpio_get_buffer_usage(void);
uint32_t gpio_get_actual_sample_rate(void);

#endif // GPIO_HANDLER_H