#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "../include/gpio_handler.h"
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

void print_usage(const char* program_name) {
    printf("Usage: %s [OPTIONS]\n", program_name);
    printf("Options:\n");
    printf("  --help        Display this help message\n");
}

int main(int argc, char *argv[]) {
    // Simple command-line argument parsing
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return EXIT_SUCCESS;
        } else {
            printf("Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }
    
    log_info("Starting GPIO-based data capture system with external clock");
    printf("Data pin: GPIO %d\n", GPIO_DATA_PIN);
    printf("Clock pin: GPIO %d (external)\n", GPIO_CLOCK_PIN);
    
    // Register signal handlers
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    
    // Phase 1 - Initialize GPIO
    log_info("Phase 1: Initializing hardware");
    gpio_init();
    
    // Allocate RAM buffer for capture
    if (!gpio_allocate_ram_buffer()) {
        fprintf(stderr, "Failed to allocate %.2f MB RAM buffer. Check available memory.\n", 
            (float)RAM_BUFFER_SIZE / (1024 * 1024));
        gpio_cleanup();
        return EXIT_FAILURE;
    }
    
    // Phase 2 - RAM Capture
    log_info("Phase 2: Starting RAM capture with external clock");
    printf("Capturing data at %.3f MHz (external clock)...\n", TARGET_SAMPLE_RATE / 1000000.0);
    
    if (!gpio_start_capture()) {
        log_error("Failed to start capture");
        gpio_free_ram_buffer();
        gpio_cleanup();
        return EXIT_FAILURE;
    }
    
    // Wait for capture to finish or be interrupted
    while (running) {
        // Just check if we're still capturing
        if (!gpio_capture_is_running()) {
            break;
        }
        sleep(1);
    }

    // Now stop the capture properly
    gpio_stop_capture();
    
    // Phase 3 - Storage to SD Card
    log_info("Phase 3: Writing data to SD card");
    data_storage_init();
    
    // Get captured buffer
    size_t captured_size = 0;
    uint8_t* captured_data = gpio_get_buffer(&captured_size);
    
    if (captured_data && captured_size > 0) {
        printf("Writing %.2f MB of captured data to SD card...\n",
               (float)captured_size / (1024 * 1024));
        
        // Write data to SD card
        write_data_to_sd(captured_data, captured_size);
        
        printf("Data successfully stored to SD card.\n");
    } else {
        log_error("No data was captured");
    }
    
    // Cleanup
    data_storage_finalize();
    gpio_free_ram_buffer();
    gpio_cleanup();
    
    log_info("Capture completed successfully");
    return EXIT_SUCCESS;
}