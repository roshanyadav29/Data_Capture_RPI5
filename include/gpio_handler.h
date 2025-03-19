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

// RAM buffer for 1-minute capture
bool gpio_allocate_ram_buffer(void);
void gpio_free_ram_buffer(void);

// High-speed capture functions
bool gpio_start_capture(void);
bool gpio_stop_capture(void);
uint8_t* gpio_get_buffer(size_t *size);
uint32_t gpio_get_actual_sample_rate(void);

bool gpio_capture_is_running(void);

#endif // GPIO_HANDLER_H