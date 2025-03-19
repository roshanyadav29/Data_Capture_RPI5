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
    printf("  --external    Use external clock on GPIO %d (default)\n", GPIO_CLOCK_PIN);
    printf("  --pcm         Use internal PCM hardware clock\n");
    printf("  --dry-run     Run in dry run mode (no hardware access)\n");
    printf("  --help        Display this help message\n");
}

int main(int argc, char *argv[]) {
    // Default to external clock
    ClockSource clock_source = CLOCK_SOURCE_EXTERNAL;
    bool dry_run_mode = false;
    
    // Simple command-line argument parsing
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--pcm") == 0) {
            clock_source = CLOCK_SOURCE_PCM;
            printf("Using PCM hardware clock\n");
        } else if (strcmp(argv[i], "--external") == 0) {
            clock_source = CLOCK_SOURCE_EXTERNAL;
            printf("Using external clock on GPIO %d\n", GPIO_CLOCK_PIN);
        } else if (strcmp(argv[i], "--dry-run") == 0) {
            dry_run_mode = true;
            log_info("Running in dry run mode - no hardware access");
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return EXIT_SUCCESS;
        } else {
            printf("Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }
    
    log_info("Starting GPIO-based data capture system");
    printf("Data pin: GPIO %d\n", GPIO_DATA_PIN);
    printf("Clock: %s\n", (clock_source == CLOCK_SOURCE_EXTERNAL) ? 
           "External (GPIO pin)" : "Internal PCM");
    
    // Register signal handlers
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    
    // Phase 1 - Initialize GPIO
    log_info("Phase 1: Initializing hardware");
    if (!dry_run_mode) {
        gpio_init(clock_source);
        
        // Allocate RAM buffer for capture
        if (!gpio_allocate_ram_buffer()) {
            fprintf(stderr, "Failed to allocate %.2f MB RAM buffer. Check available memory.\n", 
                (float)RAM_BUFFER_SIZE / (1024 * 1024));
            gpio_cleanup();
            return EXIT_FAILURE;
        }
    } else {
        // Simulate data for testing
        log_info("Simulating data for dry run mode");
    }
    
    // Phase 2 - RAM Capture
    log_info("Phase 2: Starting RAM capture");
    printf("Capturing data at 8.192 MHz...\n");
    
    if (!dry_run_mode) {
        if (!gpio_start_capture()) {
            log_error("Failed to start capture");
            gpio_free_ram_buffer();
            gpio_cleanup();
            return EXIT_FAILURE;
        }
    }
    
    // Wait for capture to finish or be interrupted
    while (running) {
        // Just check if we're still capturing
        if (!dry_run_mode && !gpio_capture_is_running()) {
            break;
        }
        sleep(1);
    }

    // Now stop the capture properly
    if (!dry_run_mode) {
        gpio_stop_capture();
    }
    
    // Phase 3 - Storage to SD Card
    log_info("Phase 3: Writing data to SD card");
    data_storage_init();
    
    // Get captured buffer
    size_t captured_size = 0;
    uint8_t* captured_data = NULL;
    
    if (!dry_run_mode) {
        captured_data = gpio_get_buffer(&captured_size);
    } else {
        // Simulate captured data for dry run mode
        captured_size = 1024 * 1024; // 1 MB of simulated data
        captured_data = malloc(captured_size);
        memset(captured_data, 0xAA, captured_size); // Fill with dummy data
    }
    
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
    if (!dry_run_mode) {
        gpio_free_ram_buffer();
        gpio_cleanup();
    } else {
        free(captured_data);
    }
    
    log_info("Capture completed successfully");
    return EXIT_SUCCESS;
}