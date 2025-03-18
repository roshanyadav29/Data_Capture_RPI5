#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "../include/gpio_handler.h"
#include "../include/buffer_manager.h"
#include "../include/data_storage.h"
#include "../include/utils.h"
#include "../config.h"

// Global flag to indicate capture is running
static bool running = true;

// Signal handler for graceful termination
void handle_signal(int sig) {
    printf("Received signal %d, stopping capture...\n", sig);
    running = false;
}

// Callback function for processing captured data
void process_data(uint8_t* data, size_t size) {
    write_data_to_sd(data, size);
}

void print_usage(const char* program_name) {
    printf("Usage: %s [OPTIONS]\n", program_name);
    printf("Options:\n");
    printf("  --external    Use external clock on GPIO %d (default)\n", GPIO_CLOCK_PIN);
    printf("  --pcm         Use internal PCM hardware clock\n");
    printf("  --help        Display this help message\n");
}

int main(int argc, char *argv[]) {
    // Default to external clock
    ClockSource clock_source = CLOCK_SOURCE_EXTERNAL;
    
    // Simple command-line argument parsing
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--pcm") == 0) {
            clock_source = CLOCK_SOURCE_PCM;
            printf("Using PCM hardware clock\n");
        } else if (strcmp(argv[i], "--external") == 0) {
            clock_source = CLOCK_SOURCE_EXTERNAL;
            printf("Using external clock on GPIO %d\n", GPIO_CLOCK_PIN);
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return EXIT_SUCCESS;
        } else {
            printf("Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }
    
    printf("Starting GPIO-based data capture at 8.192 MHz\n");
    printf("Data pin: GPIO %d\n", GPIO_DATA_PIN);
    printf("Clock: %s\n", (clock_source == CLOCK_SOURCE_EXTERNAL) ? 
           "External (GPIO pin)" : "Internal PCM");
    
    // Register signal handlers
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    
    // Initialize subsystems
    gpio_init(clock_source);
    if (!gpio_dma_init()) {
        log_error("Failed to initialize DMA buffers");
        return EXIT_FAILURE;
    }
    
    data_storage_init();
    
    // Start capture with callback
    if (!gpio_start_capture(process_data)) {
        log_error("Failed to start capture");
        gpio_cleanup();
        return EXIT_FAILURE;
    }
    
    log_info("Capture started. Press Ctrl+C to stop.");
    
    // Main loop - wait for termination signal
    while (running) {
        // Print current buffer usage periodically
        printf("Buffer usage: %zu bytes\n", gpio_get_buffer_usage());
        sleep(1);  // Update every second
    }
    
    // Cleanup
    printf("Stopping capture...\n");
    gpio_stop_capture();
    gpio_cleanup();
    data_storage_finalize();
    
    log_info("Capture completed successfully");
    return EXIT_SUCCESS;
}