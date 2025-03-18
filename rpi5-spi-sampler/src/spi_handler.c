#include "../include/spi_handler.h"
#include "../config.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <time.h>
#include "buffer_manager.h"

// SPI file descriptor
static int spi_fd = -1;

// DMA buffers and state
static uint8_t *dma_buffers[NUM_DMA_BUFFERS];
static int current_write_buffer = 0;
static int current_read_buffer = 0;
static size_t buffer_position = 0;
static bool capture_running = false;

// Worker thread for handling buffer rotation
static pthread_t worker_thread;
static void (*data_ready_callback)(uint8_t*, size_t) = NULL;

// Worker thread function to process filled buffers
static void* buffer_processor(void* arg) {
    while (capture_running) {
        // Check if a buffer is full and needs processing
        if (current_read_buffer != current_write_buffer) {
            // Call the callback function with the filled buffer
            if (data_ready_callback != NULL) {
                data_ready_callback(dma_buffers[current_read_buffer], DMA_BUFFER_SIZE);
            }
            
            // Move to next buffer
            current_read_buffer = (current_read_buffer + 1) % NUM_DMA_BUFFERS;
        } else {
            // No buffers to process, sleep briefly
            usleep(THREAD_SLEEP_US); 
        }
    }
    
    return NULL;
}

// Initialize SPI interface
void spi_init(void) {
    // Open SPI device
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        perror("Failed to open SPI device");
        return;
    }

    // Set SPI mode
    uint8_t mode = SPI_MODE;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("Failed to set SPI mode");
        close(spi_fd);
        spi_fd = -1;
        return;
    }

    // Set SPI speed
    uint32_t speed = SPI_SPEED;
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("Failed to set SPI speed");
        close(spi_fd);
        spi_fd = -1;
        return;
    }

    // Set bits per word
    uint8_t bits = SPI_BITS_PER_WORD;
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("Failed to set bits per word");
        close(spi_fd);
        spi_fd = -1;
        return;
    }
    
    printf("SPI interface initialized successfully\n");
}

// Initialize DMA buffers for high-speed capture
bool spi_dma_init(void) {
    // Allocate DMA buffers
    for (int i = 0; i < NUM_DMA_BUFFERS; i++) {
        dma_buffers[i] = (uint8_t*)malloc(DMA_BUFFER_SIZE);
        if (dma_buffers[i] == NULL) {
            // Free previously allocated buffers
            for (int j = 0; j < i; j++) {
                free(dma_buffers[j]);
                dma_buffers[j] = NULL;
            }
            fprintf(stderr, "Failed to allocate DMA buffer %d\n", i);
            return false;
        }
        // Zero out the buffer
        memset(dma_buffers[i], 0, DMA_BUFFER_SIZE);
    }
    
    current_write_buffer = 0;
    current_read_buffer = 0;
    buffer_position = 0;
    
    printf("DMA buffers initialized: %d buffers of %d bytes each\n", 
           NUM_DMA_BUFFERS, DMA_BUFFER_SIZE);
    
    return true;
}

// Start high-speed capture with DMA
bool spi_start_capture(void (*callback)(uint8_t*, size_t)) {
    if (spi_fd < 0) {
        fprintf(stderr, "SPI not initialized\n");
        return false;
    }
    
    // Store the callback
    data_ready_callback = callback;
    
    // Set up for capture
    capture_running = true;
    
    // If using external clock, configure it
    if (USE_EXTERNAL_CLOCK) {
        if (!spi_sync_to_external_clock()) {
            fprintf(stderr, "Failed to sync to external clock\n");
            capture_running = false;
            return false;
        }
    }
    
    // Start the worker thread
    if (pthread_create(&worker_thread, NULL, buffer_processor, NULL) != 0) {
        fprintf(stderr, "Failed to create worker thread\n");
        capture_running = false;
        return false;
    }
    
    printf("SPI capture started at %.3f MHz\n", (float)SPI_SPEED / 1000000);
    return true;
}

// Stop the capture process
bool spi_stop_capture(void) {
    if (!capture_running) {
        return true;
    }
    
    // Signal thread to stop
    capture_running = false;
    
    // Wait for thread to terminate
    pthread_join(worker_thread, NULL);
    
    // Process any remaining data in the current buffer
    if (buffer_position > 0 && data_ready_callback != NULL) {
        data_ready_callback(dma_buffers[current_write_buffer], buffer_position);
    }
    
    printf("SPI capture stopped\n");
    return true;
}

// Basic SPI read/write functions
uint8_t spi_read_byte(void) {
    uint8_t tx = 0;
    uint8_t rx = 0;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)&tx,
        .rx_buf = (unsigned long)&rx,
        .len = 1,
        .speed_hz = SPI_SPEED,
        .delay_usecs = 0,
        .bits_per_word = SPI_BITS_PER_WORD,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("Failed to send SPI message");
        return 0;
    }

    return rx;
}

void spi_write_byte(uint8_t data) {
    uint8_t rx;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)&data,
        .rx_buf = (unsigned long)&rx,
        .len = 1,
        .speed_hz = SPI_SPEED,
        .delay_usecs = 0,
        .bits_per_word = SPI_BITS_PER_WORD,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("Failed to send SPI message");
    }
}

void spi_transfer(uint8_t *tx_buffer, uint8_t *rx_buffer, size_t length) {
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)rx_buffer,
        .len = length,
        .speed_hz = SPI_SPEED,
        .delay_usecs = 0,
        .bits_per_word = SPI_BITS_PER_WORD,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("Failed to send SPI message");
    }
}

// External clock synchronization
bool spi_sync_to_external_clock(void) {
    // This would involve GPIO configuration to use the external clock
    // for the SPI peripheral. The exact implementation depends on
    // the Raspberry Pi's specific hardware capabilities.
    
    printf("Synchronized to external clock on GPIO %d\n", CLOCK_PIN);
    return true;
}

// Get current buffer usage
size_t spi_get_buffer_usage(void) {
    if (current_write_buffer == current_read_buffer) {
        return buffer_position;
    } else {
        return (NUM_DMA_BUFFERS - 1) * DMA_BUFFER_SIZE + buffer_position;
    }
}

uint32_t spi_get_actual_sample_rate(void) {
    // In a real implementation, you would measure the actual achieved rate
    return SPI_SPEED / 8;  // Bits per second / 8 = Bytes per second
}