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

// Memory mapped GPIO
static volatile uint32_t *gpio_map = NULL;
static volatile uint32_t *pcm_map = NULL;
static int mem_fd = -1;

// RAM buffer for data capture
static BufferChunk* current_chunk = NULL;
static bool capture_running = false;
static size_t total_bytes_captured = 0;

// Capture thread
static pthread_t capture_thread;

// Clock source setting
static ClockSource current_clock_source;

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

// PCM registers base address
#define PCM_BASE 0xFE203000

// PCM register offsets
#define PCM_CS     0x00  // Control and status
#define PCM_FIFO   0x04  // FIFO data
#define PCM_MODE   0x08  // Mode

// For sysfs GPIO edge detection
static char gpio_edge_path[64];
static int gpio_fd = -1;

// Initialize GPIO for direct memory access
void gpio_init(ClockSource clock_source) {
    current_clock_source = clock_source;
    
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

    // If using PCM clock, map PCM registers too
    if (clock_source == CLOCK_SOURCE_PCM) {
        pcm_map = (volatile uint32_t *)mmap(
            NULL,
            0x1000,  // Map 4KB
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            mem_fd,
            PCM_BASE
        );

        if (pcm_map == MAP_FAILED) {
            perror("PCM mmap failed");
            munmap((void*)gpio_map, 0x1000);
            close(mem_fd);
            mem_fd = -1;
            return;
        }
    }

    // Configure the GPIO data pin as input
    int data_reg = GPIO_DATA_PIN / 10;
    int data_shift = (GPIO_DATA_PIN % 10) * 3;
    
    // Clear the bits (set as input)
    gpio_map[GPFSEL/4 + data_reg] &= ~(7 << data_shift);
    
    printf("GPIO %d configured as data input\n", GPIO_DATA_PIN);

    if (clock_source == CLOCK_SOURCE_EXTERNAL) {
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
        
        printf("GPIO %d configured as external clock input with rising edge detection\n", GPIO_CLOCK_PIN);
        
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
            
            printf("External clock configured via sysfs\n");
        }
    } else if (clock_source == CLOCK_SOURCE_PCM) {
        // Configure PCM clock for 8.192 MHz
        // Disable PCM
        pcm_map[PCM_CS/4] = 0;
        usleep(100);
        
        // Set up PCM clock - assuming 500MHz core clock
        pcm_map[PCM_MODE/4] = (61 << 10); // 500MHz / 61 ≈ 8.196MHz
        
        // Enable PCM with appropriate settings
        pcm_map[PCM_CS/4] = 1; // Enable PCM
        
        printf("PCM clock configured for approximately 8.192 MHz\n");
    }
}

// Allocate RAM buffer for 1-minute capture
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
}

// External clock capture thread
static void* external_clock_capture_thread(void* arg) {
    (void)arg;  // Add this line to suppress the warning
    struct pollfd pfd;
    char value[2];
    int bit_count = 0;
    uint8_t byte_value = 0;
    unsigned long sample_counter = 0;
    struct timespec last_report_time;
    int time_check_counter = 0; // Add this line
    
    // Calculate which register and bit to read
    int reg_offset = GPLEV/4;          // GPLEV register offset (divided by 4 for 32-bit access)
    int data_bit = 1 << (GPIO_DATA_PIN % 32);  // Bit mask for our data pin
    int clock_bit = 1 << (GPIO_CLOCK_PIN % 32); // Bit mask for clock pin
    int clock_reg = GPEDS/4 + (GPIO_CLOCK_PIN / 32);
    
    // Set up polling if using sysfs approach
    if (gpio_fd >= 0) {
        pfd.fd = gpio_fd;
        pfd.events = POLLPRI | POLLERR;
        
        // Initial read to clear any pending events
        lseek(gpio_fd, 0, SEEK_SET);
        read(gpio_fd, value, 1);
    }

    // Get start time
    clock_gettime(CLOCK_MONOTONIC, &capture_start_time);
    last_report_time = capture_start_time;
    
    printf("Starting 1-minute data capture...\n");
    
    while (capture_running && total_bytes_captured < RAM_BUFFER_SIZE) {
        bool edge_detected = false;
        
        if (gpio_fd >= 0) {
            // Use sysfs polling approach
            if (poll(&pfd, 1, 10) > 0) {  // 10ms timeout to reduce potential missed samples
                if (pfd.revents & POLLPRI) {
                    // Clock edge detected
                    lseek(gpio_fd, 0, SEEK_SET);
                    read(gpio_fd, value, 1);
                    edge_detected = true;
                }
            }
        } else {
            // Use memory-mapped register approach
            if (gpio_map[clock_reg] & clock_bit) {
                // Clear the event
                gpio_map[clock_reg] = clock_bit;
                edge_detected = true;
            }
        }
        
        if (edge_detected) {
            // Read data pin on clock edge
            uint32_t level = (gpio_map[reg_offset] & data_bit) ? 1 : 0;
            
            // Pack 8 bits into a byte (MSB first)
            byte_value = (byte_value << 1) | level;
            bit_count++;
            
            if (bit_count == 8) {
                // Got a complete byte, check if current chunk is full
                if (!current_chunk || current_chunk->used >= current_chunk->size) {
                    // Release current chunk if it exists
                    if (current_chunk) {
                        current_chunk->ready_for_writing = true;
                        buffer_release_chunk(current_chunk);
                    }
                    
                    // Get next chunk
                    current_chunk = buffer_get_next_chunk();
                    if (!current_chunk) {
                        capture_running = false;
                        printf("Buffer full, stopping capture\n");
                        break;
                    }
                }
                
                // Store byte in current chunk
                current_chunk->data[current_chunk->used++] = byte_value;
                total_bytes_captured++;
                bit_count = 0;
                byte_value = 0;
                
                // Increment sample counter
                sample_counter++;
            }
            
            // Only check time periodically to improve performance
            if (++time_check_counter >= 1000) {
                time_check_counter = 0;
                
                struct timespec current_time; // This declaration is missing
                clock_gettime(CLOCK_MONOTONIC, &current_time);
                double elapsed = (current_time.tv_sec - last_report_time.tv_sec) + 
                                (current_time.tv_nsec - last_report_time.tv_nsec) / 1000000000.0;
                
                if (elapsed >= 5.0) {
                    double total_elapsed = (current_time.tv_sec - capture_start_time.tv_sec) + 
                                          (current_time.tv_nsec - capture_start_time.tv_nsec) / 1000000000.0;
                    
                    double bytes_per_second = sample_counter / elapsed;
                    double expected_rate = BYTES_PER_SECOND;
                    double efficiency = (bytes_per_second / expected_rate) * 100.0;
                    
                    printf("Capture progress: %.1f seconds (%.1f%%) - %.2f KB/sec (%.1f%% of target)\n", 
                           total_elapsed, (total_elapsed / CAPTURE_SECONDS) * 100,
                           bytes_per_second / 1024, efficiency);
                    
                    // Reset counter for next interval
                    sample_counter = 0;
                    last_report_time = current_time;
                    
                    // Check if we've reached 1 minute or buffer is full
                    if (total_elapsed >= CAPTURE_SECONDS || total_bytes_captured >= RAM_BUFFER_SIZE) {
                        capture_running = false;
                        break;
                    }
                }
            }
        }
    }
    
    // Record end time
    clock_gettime(CLOCK_MONOTONIC, &capture_end_time);
    
    // If we exited due to full buffer, report it
    if (total_bytes_captured >= RAM_BUFFER_SIZE) {
        printf("RAM buffer capacity reached (%.2f MB). Stopping capture.\n",
               (float)total_bytes_captured / (1024 * 1024));
    }
    
    if (current_chunk && current_chunk->used > 0) {
        current_chunk->ready_for_writing = true;
        // Don't release it here, just mark it for writing
    }
    
    return NULL;
}

// PCM clock capture thread (similar modifications for PCM clock)
static void* pcm_clock_capture_thread(void* arg) {
    (void)arg;  // Add this line to suppress the warning
    int bit_count = 0;
    uint8_t byte_value = 0;
    unsigned long sample_counter = 0;
    struct timespec last_report_time;
    
    // Calculate which register and bit to read
    int reg_offset = GPLEV/4;  // GPLEV register offset
    int data_bit = 1 << (GPIO_DATA_PIN % 32);  // Bit mask for our data pin
    
    // Get start time
    clock_gettime(CLOCK_MONOTONIC, &capture_start_time);
    last_report_time = capture_start_time;
    
    printf("Starting 1-minute data capture with PCM clock...\n");
    
    while (capture_running && total_bytes_captured < RAM_BUFFER_SIZE) {
        // Wait for PCM clock cycle
        if (pcm_map[PCM_CS/4] & (1 << 5)) { // Check PCM_CS.RXREADY flag
            // Read data pin on clock edge
            uint32_t level = (gpio_map[reg_offset] & data_bit) ? 1 : 0;
            
            // Pack 8 bits into a byte (MSB first)
            byte_value = (byte_value << 1) | level;
            bit_count++;
            
            if (bit_count == 8) {
                // Got a complete byte, check if current chunk is full
                if (!current_chunk || current_chunk->used >= current_chunk->size) {
                    // Release current chunk if it exists
                    if (current_chunk) {
                        current_chunk->ready_for_writing = true;
                        buffer_release_chunk(current_chunk);
                    }
                    
                    // Get next chunk
                    current_chunk = buffer_get_next_chunk();
                    if (!current_chunk) {
                        capture_running = false;
                        printf("Buffer full, stopping capture\n");
                        break;
                    }
                }
                
                // Store byte in current chunk
                current_chunk->data[current_chunk->used++] = byte_value;
                total_bytes_captured++;
                bit_count = 0;
                byte_value = 0;
                
                // Increment sample counter
                sample_counter++;
            }
            
            // Report progress every 5 seconds
            struct timespec current_time;
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            double elapsed = (current_time.tv_sec - last_report_time.tv_sec) + 
                            (current_time.tv_nsec - last_report_time.tv_nsec) / 1000000000.0;
            
            if (elapsed >= 5.0) {
                double total_elapsed = (current_time.tv_sec - capture_start_time.tv_sec) + 
                                      (current_time.tv_nsec - capture_start_time.tv_nsec) / 1000000000.0;
                
                double bytes_per_second = sample_counter / elapsed;
                double expected_rate = BYTES_PER_SECOND;
                double efficiency = (bytes_per_second / expected_rate) * 100.0;
                
                printf("Capture progress: %.1f seconds (%.1f%%) - %.2f KB/sec (%.1f%% of target)\n", 
                       total_elapsed, (total_elapsed / CAPTURE_SECONDS) * 100,
                       bytes_per_second / 1024, efficiency);
                
                // Reset counter for next interval
                sample_counter = 0;
                last_report_time = current_time;
                
                // Check if we've reached 1 minute or buffer is full
                if (total_elapsed >= CAPTURE_SECONDS || total_bytes_captured >= RAM_BUFFER_SIZE) {
                    capture_running = false;
                    break;
                }
            }
        }
    }
    
    // Record end time
    clock_gettime(CLOCK_MONOTONIC, &capture_end_time);
    
    if (current_chunk && current_chunk->used > 0) {
        current_chunk->ready_for_writing = true;
        // Don't release it here, just mark it for writing
    }
    
    return NULL;
}

// Start high-speed capture
bool gpio_start_capture(void) {
    if (gpio_map == NULL) {
        fprintf(stderr, "GPIO not initialized\n");
        return false;
    }
    
    if (buffer_get_total_size() == 0) {
        fprintf(stderr, "RAM buffer not allocated\n");
        return false;
    }
    
    // Set up for capture
    capture_running = true;
    total_bytes_captured = 0;
    
    // Start the appropriate capture thread based on clock source
    if (current_clock_source == CLOCK_SOURCE_EXTERNAL) {
        if (pthread_create(&capture_thread, NULL, external_clock_capture_thread, NULL) != 0) {
            fprintf(stderr, "Failed to create capture thread\n");
            capture_running = false;
            return false;
        }
        printf("GPIO capture started with external clock on GPIO %d\n", GPIO_CLOCK_PIN);
    } else {
        if (pthread_create(&capture_thread, NULL, pcm_clock_capture_thread, NULL) != 0) {
            fprintf(stderr, "Failed to create capture thread\n");
            capture_running = false;
            return false;
        }
        printf("GPIO capture started with PCM hardware clock\n");
    }
    
    return true;
}

// Stop the capture process
bool gpio_stop_capture(void) {
    if (!capture_running) {
        return true;
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
           bits_per_second / 1000000, 
           (bits_per_second / TARGET_SAMPLE_RATE) * 100);
    
    return true;
}

bool gpio_capture_is_running(void) {
    return capture_running;
}

// This function needs complete rewriting to copy data from all chunks
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
        return NULL;
    }
    
    // Allocate new buffer
    output_buffer = (uint8_t*)malloc(total_size);
    if (!output_buffer) {
        fprintf(stderr, "Failed to allocate memory for output buffer\n");
        return NULL;
    }
    
    if (!buffer_copy_all_data(output_buffer, total_size)) {
        fprintf(stderr, "Failed to copy data from buffer chunks\n");
        free(output_buffer);
        return NULL;
    }
    
    return output_buffer;
}

// Clean up GPIO resources
void gpio_cleanup(void) {
    if (gpio_map != NULL) {
        munmap((void*)gpio_map, 0x1000);
        gpio_map = NULL;
    }
    
    if (pcm_map != NULL) {
        munmap((void*)pcm_map, 0x1000);
        pcm_map = NULL;
    }
    
    if (mem_fd >= 0) {
        close(mem_fd);
        mem_fd = -1;
    }
    
    if (gpio_fd >= 0) {
        close(gpio_fd);
        gpio_fd = -1;
        
        // Unexport the GPIO
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

#ifdef DRY_RUN
// Mock functions that simulate hardware behavior
int mock_gpio_read() {
    static int counter = 0;
    counter++;
    // Generate test pattern
    return counter % 256;
}

bool setup_gpio_mapping() {
    // Skip real hardware access
    log_info("Mock GPIO setup completed");
    return true;
}

// Other mocked functions...
#endif