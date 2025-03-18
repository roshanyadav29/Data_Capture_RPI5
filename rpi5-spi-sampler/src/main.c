#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <stdbool.h>
#include "../include/spi_handler.h"
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

int main() {
    // Register signal handlers
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    
    // Initialize subsystems
    spi_init();
    if (!spi_dma_init()) {
        log_error("Failed to initialize DMA buffers");
        return EXIT_FAILURE;
    }
    
    data_storage_init();
    
    // Start capture with callback
    if (!spi_start_capture(process_data)) {
        log_error("Failed to start capture");
        return EXIT_FAILURE;
    }
    
    log_info("Capture started. Press Ctrl+C to stop.");
    
    // Main loop - wait for termination signal
    while (running) {
        // Print current buffer usage periodically
        printf("Buffer usage: %zu bytes\n", spi_get_buffer_usage());
        delay_ms(1000);  // Update every second
    }
    
    // Cleanup
    spi_stop_capture();
    data_storage_finalize();
    
    log_info("Capture completed successfully");
    return EXIT_SUCCESS;
}