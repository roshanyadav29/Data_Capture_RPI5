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
#include <sched.h>      // For sched_yield() and CPU affinity
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
static uint64_t sample_counter = 0;
static uint32_t time_check_counter = 0;
static struct timespec last_report_time;

// GPIO register offsets
#define GPFSEL   0x00  // Function select registers
#define GPSET    0x1C  // Pin output set registers
#define GPCLR    0x28  // Pin output clear registers
#define GPLEV    0x34  // Pin level registers
#define GPEDS    0x40  // Event detect status registers
#define GPREN    0x4C  // Rising edge detect enable registers
#define GPFEN    0x58  // Falling edge detect enable registers

// For sysfs GPIO edge detection
static int gpio_fd = -1;

// RPi 4 peripheral base address - defined here since not in config.h
#define BCM2711_PERI_BASE        0xFE000000  // This is the correct base for RPi 4

// GPIO setup macros - RPi 4 specific
#define INP_GPIO(g) *(gpio_map+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio_map+((g)/10)) |= (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio_map+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio_map+7)  // sets   bits which are 1, ignores bits which are 0
#define GPIO_CLR *(gpio_map+10) // clears bits which are 1, ignores bits which are 0

#define GPIO_READ(g) (*(gpio_map+13)&(1<<(g))) // 0 if LOW, (1<<g) if HIGH

// GPIO Event Detect Status
#define GPEDS               0x40
#define GPREN               0x4C  // Rising edge detect enable
#define GPFEN               0x58  // Falling edge detect enable

// GPIO Pull-up/down - different mechanism on RPi 4
#define GPPUD               0x94

void gpio_init(void) {
    log_info("Initializing GPIO for Raspberry Pi 4...");
    
    // Open /dev/mem
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        log_error("Can't open /dev/mem. Try checking permissions.");
        return;
    }
    
    // mmap GPIO
    void *gpio_map_raw = mmap(
        NULL,                   // Any address in our space will do
        4*1024,                 // Map length
        PROT_READ | PROT_WRITE, // Enable read/write
        MAP_SHARED,             // Shared with other processes
        mem_fd,                 // File to map
        GPIO_BASE               // Offset to GPIO peripheral
    );
    
    close(mem_fd); // No need to keep mem_fd open after mmap
    mem_fd = -1;
    
    if (gpio_map_raw == MAP_FAILED) {
        log_error("mmap error %d\n", (int)gpio_map_raw);
        return;
    }
    
    gpio_map = (volatile uint32_t *)gpio_map_raw;
    
    // Setup the Data and Clock pins as inputs
    log_info("Setting up GPIO pins...");
    INP_GPIO(GPIO_DATA_PIN);  // Set as input
    INP_GPIO(GPIO_CLOCK_PIN); // Set as input
    
    // Enable rising edge detection on the clock pin
    int edge_reg = GPIO_CLOCK_PIN / 32;
    int edge_bit = 1 << (GPIO_CLOCK_PIN % 32);
    
    // Enable rising edge detection - crucial for capturing edges
    gpio_map[GPREN/4 + edge_reg] |= edge_bit;
    
    // Clear any pending events before we start
    gpio_map[GPEDS/4 + edge_reg] = edge_bit;
    
    // Enable pull-down on data pin for clean signals
    // RPi 4 requires a different sequence than RPi 5
    gpio_map[GPPUD/4] = 1;     // 1 = pull-down, 0 = disable, 2 = pull-up
    
    // Wait 150 cycles – this provides the required setup time for the control signal
    for (volatile int i = 0; i < 150; i++);
    
    // Clock the control signal into the GPIO pads
    gpio_map[GPPUD/4 + 1] = (1 << GPIO_DATA_PIN);
    
    // Wait 150 cycles – this provides the required hold time for the control signal
    for (volatile int i = 0; i < 150; i++);
    
    // Remove the control signal
    gpio_map[GPPUD/4] = 0;
    gpio_map[GPPUD/4 + 1] = 0;
    
    log_info("GPIO initialized successfully");
    
    // Setup data structures for capture
    total_bytes_captured = 0;
    sample_counter = 0;
    time_check_counter = 0;
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

// Edge detection using mmap approach - efficient and stable

static void* external_clock_capture_thread(void* arg) {
    (void)arg; // Suppress unused parameter warning
    
    // Set thread to real-time priority but not maximum
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO) / 2;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    
    // Registers for direct memory access - optimized for edge detection
    int level_reg = GPLEV/4;      // GPIO level register (for reading pin values)
    int event_reg = GPEDS/4 + (GPIO_CLOCK_PIN / 32);  // Event detect status register
    int clock_bit = 1 << (GPIO_CLOCK_PIN % 32);       // Bit mask for clock pin
    int data_bit = 1 << GPIO_DATA_PIN;                // Bit mask for data pin
    
    // Bit accumulation
    uint8_t byte_value = 0;
    uint8_t bit_count = 0;
    
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
    
    // Variables for edge detection tracking and diagnostics
    uint64_t edges_detected = 0;
    struct timespec last_edge_time;
    clock_gettime(CLOCK_MONOTONIC, &last_edge_time);
    uint32_t missed_edges_counter = 0;
    
    // For periodic OS yield to prevent freezing
    struct timespec last_yield_time = last_edge_time;
    
    // Main capture loop
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
        
        // Check for clock edge using event detect register (hardware edge detection)
        // This is the key to reliable edge detection without software polling
        if (gpio_map[event_reg] & clock_bit) {
            // Clear the event immediately to catch the next edge
            gpio_map[event_reg] = clock_bit;
            
            // Hardware detected a rising edge
            // Read data pin immediately
            uint8_t data_value = (gpio_map[level_reg] & data_bit) ? 1 : 0;
            
            // Add bit to our byte (LSB first)
            byte_value |= (data_value << bit_count);
            bit_count++;
            
            if (bit_count == 8) {
                // Store the complete byte
                current_chunk->data[current_chunk->used++] = byte_value;
                total_bytes_captured++;
                
                // Reset for next byte
                bit_count = 0;
                byte_value = 0;
                
                sample_counter++;
            }
            
            // Track edge timing for diagnostics
            edges_detected++;
            
            struct timespec current_edge_time;
            clock_gettime(CLOCK_MONOTONIC, &current_edge_time);
            
            // Only calculate timing every 1000 edges to reduce overhead
            if (edges_detected % 1000 == 0) {
                double edge_interval = (current_edge_time.tv_sec - last_edge_time.tv_sec) + 
                                      (current_edge_time.tv_nsec - last_edge_time.tv_nsec) / 1000000000.0;
                double instantaneous_rate = 1000.0 / edge_interval;  // 1000 edges / time = rate
                
                if (edge_interval > 0.001) { // More than 1ms for 1000 edges is slow
                    missed_edges_counter++;
                    if (missed_edges_counter % 10 == 0) {
                        printf("Clock rate: %.2f MHz\n", instantaneous_rate / 1000000.0);
                    }
                }
                
                last_edge_time = current_edge_time;
            }
        }
        
        // Periodic yield to prevent system freeze - check time since last yield
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        
        double time_since_yield = (now.tv_sec - last_yield_time.tv_sec) + 
                               (now.tv_nsec - last_yield_time.tv_nsec) / 1000000000.0;
        
        // Yield to the OS every 20ms - this prevents system freeze
        // without significantly affecting capture rate (missing at most a few edges)
        if (time_since_yield > 0.02) {  // 20ms
            sched_yield();
            last_yield_time = now;
        }
        
        // Do periodic progress reporting
        time_check_counter++;
        if (time_check_counter >= 10000000) {  // Less frequent reporting
            double elapsed = (now.tv_sec - capture_start_time.tv_sec) + 
                            (now.tv_nsec - capture_start_time.tv_nsec) / 1000000000.0;
                            
            double rate = sample_counter * 8 / elapsed;  // bits per second
            
            printf("Captured: %.2f MB (%.2f%%), Rate: %.2f MHz, Edges: %llu\n", 
                   (float)total_bytes_captured / (1024 * 1024),
                   100.0 * total_bytes_captured / RAM_BUFFER_SIZE,
                   rate / 1000000.0,
                   (unsigned long long)edges_detected);
                   
            // If we've captured enough data or reached the time limit
            if (total_bytes_captured >= RAM_BUFFER_SIZE || 
                elapsed >= CAPTURE_SECONDS) {
                log_info("Capture complete");
                capture_running = false;
                break;
            }
            
            time_check_counter = 0;
        }
    }
    
    // Calculate final statistics
    clock_gettime(CLOCK_MONOTONIC, &capture_end_time);
    
    double elapsed = (capture_end_time.tv_sec - capture_start_time.tv_sec) + 
              (capture_end_time.tv_nsec - capture_start_time.tv_nsec) / 1000000000.0;
              
    // Calculate rate
    double bytes_per_second = total_bytes_captured / elapsed;
    double bits_per_second = bytes_per_second * 8;
    
    printf("Capture completed:\n");
    printf("  Bytes captured: %zu (%.2f MB)\n", 
           total_bytes_captured, (float)total_bytes_captured / (1024 * 1024));
    printf("  Duration: %.2f seconds\n", elapsed);
    printf("  Average rate: %.2f MHz (%.1f%% of target)\n", 
           bits_per_second / 1000000.0, 
           100.0 * bits_per_second / (TARGET_SAMPLE_RATE));
    printf("  Total edges detected: %llu\n", (unsigned long long)edges_detected);
    
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
    sample_counter = 0;
    time_check_counter = 0;
    
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
    clock_gettime(CLOCK_MONOTONIC, &capture_end_time);
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

// Function to help diagnose clock issues
bool gpio_debug_clock_signal(int seconds) {
    printf("Testing clock signal for %d seconds...\n", seconds);
    
    // Register for edge detection
    int event_reg = GPEDS/4 + (GPIO_CLOCK_PIN / 32);
    int clock_bit = 1 << (GPIO_CLOCK_PIN % 32);
    
    uint64_t edge_count = 0;
    struct timespec start_time, current_time, last_yield_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    last_yield_time = start_time;
    
    // Clear any pending events before we start
    gpio_map[event_reg] = clock_bit;
    
    // Report frequency every quarter second
    double last_report_time = 0;
    uint64_t last_report_count = 0;
    
    while (1) {
        // Check for rising edge using hardware edge detection
        if (gpio_map[event_reg] & clock_bit) {
            // Clear the event
            gpio_map[event_reg] = clock_bit;
            edge_count++;
        }
        
        // Check if we've run for the specified duration
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double elapsed = (current_time.tv_sec - start_time.tv_sec) + 
                        (current_time.tv_nsec - start_time.tv_nsec) / 1000000000.0;
        
        if (elapsed >= seconds) {
            break;
        }
        
        // Print status every 0.25 second with instantaneous rate
        if (elapsed - last_report_time >= 0.25) {
            double interval = elapsed - last_report_time;
            uint64_t edges = edge_count - last_report_count;
            double rate = edges / interval;
            
            printf("Clock rate: %.2f MHz (%.0f edges in %.2fs)\n", 
                  rate / 1000000.0, (double)edges, interval);
            
            last_report_time = elapsed;
            last_report_count = edge_count;
        }
        
        // Yield periodically to prevent system freeze
        double time_since_yield = (current_time.tv_sec - last_yield_time.tv_sec) + 
                                 (current_time.tv_nsec - last_yield_time.tv_nsec) / 1000000000.0;
        
        if (time_since_yield > 0.02) {  // 20ms
            sched_yield();
            last_yield_time = current_time;
        }
    }
    
    double total_elapsed = (current_time.tv_sec - start_time.tv_sec) + 
                          (current_time.tv_nsec - start_time.tv_nsec) / 1000000000.0;
    double avg_rate = edge_count / total_elapsed;
    
    printf("Clock signal test complete:\n");
    printf("  Edges detected: %llu\n", (unsigned long long)edge_count);
    printf("  Duration: %.2f seconds\n", total_elapsed);
    printf("  Average rate: %.2f MHz\n", avg_rate / 1000000.0);
    printf("  Stability: %.1f%% of target\n", 100.0 * avg_rate / TARGET_SAMPLE_RATE);
    
    return avg_rate >= TARGET_SAMPLE_RATE * 0.9; // Return true if at least 90% of target
}