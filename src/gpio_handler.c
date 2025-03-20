#include "../include/gpio_handler.h"
#include "../config.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
#include <poll.h>
#include <time.h>
#include "../include/buffer_manager.h"
#include "../include/utils.h"

// Memory mapped GPIO
static volatile uint32_t *gpio_map = NULL;
static int mem_fd = -1;

// RAM buffer for data capture
static BufferChunk* current_chunk = NULL;
static bool capture_running = false;
static size_t total_bytes_captured = 0;

// Capture thread
static pthread_t capture_thread;

// Performance monitoring
static struct timespec capture_start_time;
static struct timespec capture_end_time;

// GPIO register offsets
#define GPFSEL   0x00  // Function select registers
#define GPSET    0x1C  // Pin output set registers
#define GPCLR    0x28  // Pin output clear registers
#define GPLEV    0x34  // Pin level registers
#define GPEDS    0x40  // Event detect status registers
#define GPREN    0x4C  // Rising edge detect enable registers
#define GPFEN    0x58  // Falling edge detect enable registers

// For sysfs GPIO edge detection
static char gpio_edge_path[64];
static int gpio_fd = -1;

// Initialize GPIO for direct memory access
void gpio_init(void) {
    // Open /dev/mem to access physical memory
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("Failed to open /dev/mem");
        return;
    }

    // Map GPIO memory to our address space
    gpio_map = (volatile uint32_t *)mmap(
        NULL,
        0x1000,  // Map 4KB
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        GPIO_BASE
    );

    if (gpio_map == MAP_FAILED) {
        perror("GPIO mmap failed");
        close(mem_fd);
        mem_fd = -1;
        return;
    }

    // Configure the GPIO data pin as input
    int data_reg = GPIO_DATA_PIN / 10;
    int data_shift = (GPIO_DATA_PIN % 10) * 3;
    
    // Clear the bits (set as input)
    gpio_map[GPFSEL/4 + data_reg] &= ~(7 << data_shift);
    
    log_info("GPIO %d configured as data input", GPIO_DATA_PIN);

    // Configure the GPIO clock pin as input
    int clock_reg = GPIO_CLOCK_PIN / 10;
    int clock_shift = (GPIO_CLOCK_PIN % 10) * 3;
    
    // Clear the bits (set as input)
    gpio_map[GPFSEL/4 + clock_reg] &= ~(7 << clock_shift);
    
    // Set up rising edge detection on the clock pin
    int edge_reg = GPIO_CLOCK_PIN / 32;
    int edge_bit = 1 << (GPIO_CLOCK_PIN % 32);
    
    // Clear any pending events
    gpio_map[GPEDS/4 + edge_reg] = edge_bit;
    
    // Enable rising edge detection
    gpio_map[GPREN/4 + edge_reg] |= edge_bit;
    
    log_info("GPIO %d configured as external clock input with rising edge detection", GPIO_CLOCK_PIN);
    
    // Alternative: Set up edge detection using sysfs for more reliable interrupt handling
    int export_fd = open("/sys/class/gpio/export", O_WRONLY);
    if (export_fd >= 0) {
        char gpio_str[4];
        sprintf(gpio_str, "%d", GPIO_CLOCK_PIN);
        write(export_fd, gpio_str, strlen(gpio_str));
        close(export_fd);
        
        // Set direction to input
        sprintf(gpio_edge_path, "/sys/class/gpio/gpio%d/direction", GPIO_CLOCK_PIN);
        int direction_fd = open(gpio_edge_path, O_WRONLY);
        if (direction_fd >= 0) {
            write(direction_fd, "in", 2);
            close(direction_fd);
        }
        
        // Set edge to rising
        sprintf(gpio_edge_path, "/sys/class/gpio/gpio%d/edge", GPIO_CLOCK_PIN);
        int edge_fd = open(gpio_edge_path, O_WRONLY);
        if (edge_fd >= 0) {
            write(edge_fd, "rising", 6);
            close(edge_fd);
        }
        
        // Open the value file for polling
        sprintf(gpio_edge_path, "/sys/class/gpio/gpio%d/value", GPIO_CLOCK_PIN);
        gpio_fd = open(gpio_edge_path, O_RDONLY);
        if (gpio_fd >= 0) {
            // Read initial value to clear any pending events
            char value[2];
            read(gpio_fd, value, 1);
            log_info("Successfully set up sysfs GPIO interface for clock pin");
        }
    }
}

// Allocate RAM buffer for capture
bool gpio_allocate_ram_buffer(void) {
    printf("Initializing buffer manager for %.2f MB of data...\n", 
           (float)RAM_BUFFER_SIZE / (1024 * 1024));
    
    // Calculate number of chunks needed (1MB per chunk)
    int num_chunks = (RAM_BUFFER_SIZE / (1024 * 1024)) + 1;
    return buffer_init(1024 * 1024, num_chunks);
}

// Free RAM buffer
void gpio_free_ram_buffer(void) {
    if (current_chunk) {
        buffer_release_chunk(current_chunk);
        current_chunk = NULL;
    }
    buffer_cleanup();
}

// External clock capture thread
static void* external_clock_capture_thread(void* arg) {
    (void)arg;  // Suppress unused parameter warning
    struct pollfd pfd;
    char value[2];
    int bit_count = 0;
    uint8_t byte_value = 0;
    unsigned long sample_counter = 0;
    struct timespec last_report_time;
    int time_check_counter = 0;
    
    // Calculate which register and bit to read
    int reg_offset = GPLEV/4;          // GPLEV register offset (divided by 4 for 32-bit access)
    int data_bit = 1 << (GPIO_DATA_PIN % 32);  // Bit mask for our data pin
    int clock_bit = 1 << (GPIO_CLOCK_PIN % 32); // Bit mask for clock pin
    int clock_reg = GPEDS/4 + (GPIO_CLOCK_PIN / 32);
    
    // Set up polling if using sysfs approach
    if (gpio_fd >= 0) {
        pfd.fd = gpio_fd;
        pfd.events = POLLPRI | POLLERR;
    }

    // Get start time
    clock_gettime(CLOCK_MONOTONIC, &capture_start_time);
    last_report_time = capture_start_time;
    
    printf("Starting data capture at %.2f MHz (external clock)...\n", TARGET_SAMPLE_RATE / 1000000.0);
    
    // Get the first chunk ready
    if (!current_chunk) {
        current_chunk = buffer_get_next_chunk();
        if (!current_chunk) {
            log_error("Failed to get buffer chunk");
            return NULL;
        }
    }
    
    while (capture_running && total_bytes_captured < RAM_BUFFER_SIZE) {
        // Check if current buffer chunk is full
        if (current_chunk->used >= current_chunk->size) {
            // Release the current chunk for storage processing
            buffer_release_chunk(current_chunk);
            
            // Get a new chunk
            current_chunk = buffer_get_next_chunk();
            if (!current_chunk) {
                log_warning("Buffer full, stopping capture");
                break;
            }
        }
        
        // Wait for rising edge on the clock pin
        if (gpio_fd >= 0) {
            // Use sysfs polling approach (more reliable but slower)
            if (poll(&pfd, 1, 100) > 0) {
                // Rising edge detected, read current value
                lseek(gpio_fd, 0, SEEK_SET);
                read(gpio_fd, value, 1);
                
                // Read data pin state
                uint32_t level = gpio_map[reg_offset];
                uint8_t data_value = (level & data_bit) ? 1 : 0;
                
                // Add bit to our byte (LSB first)
                byte_value |= (data_value << bit_count);
                bit_count++;
                
                if (bit_count >= 8) {
                    // Store the complete byte
                    current_chunk->data[current_chunk->used++] = byte_value;
                    total_bytes_captured++;
                    
                    // Reset for next byte
                    bit_count = 0;
                    byte_value = 0;
                    
                    sample_counter++;
                }
            }
        } else {
            // Direct memory-mapped approach (faster but may miss edges)
            // Check for rising edge event on clock pin
            if (gpio_map[clock_reg] & clock_bit) {
                // Clear the event
                gpio_map[clock_reg] = clock_bit;
                
                // Read data pin state
                uint32_t level = gpio_map[reg_offset];
                uint8_t data_value = (level & data_bit) ? 1 : 0;
                
                // Add bit to our byte (LSB first)
                byte_value |= (data_value << bit_count);
                bit_count++;
                
                if (bit_count >= 8) {
                    // Store the complete byte
                    current_chunk->data[current_chunk->used++] = byte_value;
                    total_bytes_captured++;
                    
                    // Reset for next byte
                    bit_count = 0;
                    byte_value = 0;
                    
                    sample_counter++;
                }
            }
        }
        
        // Periodically report progress
        time_check_counter++;
        if (time_check_counter >= 100000) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            
            double elapsed = (now.tv_sec - last_report_time.tv_sec) + 
                            (now.tv_nsec - last_report_time.tv_nsec) / 1000000000.0;
                            
            if (elapsed >= 5.0) {
                double total_elapsed = (now.tv_sec - capture_start_time.tv_sec) + 
                                      (now.tv_nsec - capture_start_time.tv_nsec) / 1000000000.0;
                
                double rate = total_bytes_captured * 8 / total_elapsed;
                
                printf("Captured: %.2f MB (%.2f%%), Rate: %.2f MHz\n", 
                       (float)total_bytes_captured / (1024 * 1024),
                       100.0 * total_bytes_captured / RAM_BUFFER_SIZE,
                       rate / 1000000.0);
                       
                last_report_time = now;
            }
            
            time_check_counter = 0;
        }
    }
    
    // Record end time
    clock_gettime(CLOCK_MONOTONIC, &capture_end_time);
    
    // If we exited due to full buffer, report it
    if (total_bytes_captured >= RAM_BUFFER_SIZE) {
        log_info("Capture complete - RAM buffer full");
    }
    
    // Save any incomplete byte
    if (bit_count > 0 && current_chunk && current_chunk->used < current_chunk->size) {
        current_chunk->data[current_chunk->used++] = byte_value;
        total_bytes_captured++;
    }
    
    // Release the final chunk if it has data
    if (current_chunk && current_chunk->used > 0) {
        buffer_release_chunk(current_chunk);
        current_chunk = NULL;
    }
    
    return NULL;
}

// Start high-speed capture
bool gpio_start_capture(void) {
    if (gpio_map == NULL) {
        log_error("GPIO not initialized");
        return false;
    }
    
    if (buffer_get_total_size() == 0) {
        log_error("Buffer not initialized");
        return false;
    }
    
    // Set up for capture
    capture_running = true;
    total_bytes_captured = 0;
    
    // Start the capture thread
    if (pthread_create(&capture_thread, NULL, external_clock_capture_thread, NULL) != 0) {
        log_error("Failed to create capture thread");
        return false;
    }
    
    log_info("GPIO capture started with external clock on GPIO %d", GPIO_CLOCK_PIN);
    
    return true;
}

// Stop the capture process
bool gpio_stop_capture(void) {
    if (!capture_running) {
        return true;  // Already stopped
    }
    
    // Signal thread to stop
    capture_running = false;
    
    // Wait for thread to terminate
    pthread_join(capture_thread, NULL);
    
    // Calculate actual sample rate
    double elapsed = (capture_end_time.tv_sec - capture_start_time.tv_sec) + 
                    (capture_end_time.tv_nsec - capture_start_time.tv_nsec) / 1000000000.0;
    double bytes_per_second = total_bytes_captured / elapsed;
    double bits_per_second = bytes_per_second * 8;
    
    printf("Capture completed:\n");
    printf("  Bytes captured: %zu (%.2f MB)\n", 
           total_bytes_captured, (float)total_bytes_captured / (1024 * 1024));
    printf("  Duration: %.2f seconds\n", elapsed);
    printf("  Average rate: %.2f MHz (%.1f%% of target)\n", 
           bits_per_second / 1000000.0, 100.0 * bits_per_second / TARGET_SAMPLE_RATE);
    
    return true;
}

bool gpio_capture_is_running(void) {
    return capture_running;
}

// This function copies data from all chunks
uint8_t* gpio_get_buffer(size_t *size) {
    // WARNING: This function returns a pointer to a static buffer that will be freed
    // on subsequent calls. The caller should process or copy the data before calling
    // this function again.
    static uint8_t* output_buffer = NULL;
    
    // Free any previously allocated buffer
    if (output_buffer != NULL) {
        free(output_buffer);
        output_buffer = NULL;
    }
    
    size_t total_size = buffer_get_used_size();
    if (size) {
        *size = total_size;
    }
    
    if (total_size == 0) {
        log_warning("No data captured");
        return NULL;
    }
    
    // Allocate new buffer
    output_buffer = (uint8_t*)malloc(total_size);
    if (!output_buffer) {
        log_error("Failed to allocate memory for output buffer");
        return NULL;
    }
    
    if (!buffer_copy_all_data(output_buffer, total_size)) {
        log_error("Failed to copy data from buffer chunks");
        free(output_buffer);
        output_buffer = NULL;
        return NULL;
    }
    
    return output_buffer;
}

// Clean up GPIO resources
void gpio_cleanup(void) {
    if (gpio_map != NULL) {
        // Unmap GPIO memory
        munmap((void*)gpio_map, 0x1000);
        gpio_map = NULL;
    }
    
    if (mem_fd >= 0) {
        close(mem_fd);
        mem_fd = -1;
    }
    
    if (gpio_fd >= 0) {
        close(gpio_fd);
        gpio_fd = -1;
        
        // Unexport the GPIO pin
        int unexport_fd = open("/sys/class/gpio/unexport", O_WRONLY);
        if (unexport_fd >= 0) {
            char gpio_str[4];
            sprintf(gpio_str, "%d", GPIO_CLOCK_PIN);
            write(unexport_fd, gpio_str, strlen(gpio_str));
            close(unexport_fd);
        }
    }
}

// Get actual sample rate
uint32_t gpio_get_actual_sample_rate(void) {
    double elapsed = (capture_end_time.tv_sec - capture_start_time.tv_sec) + 
                    (capture_end_time.tv_nsec - capture_start_time.tv_nsec) / 1000000000.0;
    return (uint32_t)((total_bytes_captured * 8) / elapsed);
}