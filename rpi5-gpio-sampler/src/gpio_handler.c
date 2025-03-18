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

// Memory mapped GPIO
static volatile uint32_t *gpio_map = NULL;
static volatile uint32_t *pcm_map = NULL;
static int mem_fd = -1;

// DMA buffers and state
static uint8_t *dma_buffers[NUM_DMA_BUFFERS];
static int current_write_buffer = 0;
static int current_read_buffer = 0;
static size_t buffer_position = 0;
static bool capture_running = false;

// Worker thread for handling buffer rotation
static pthread_t worker_thread;
static pthread_t capture_thread;
static void (*data_ready_callback)(uint8_t*, size_t) = NULL;

// Clock source setting
static ClockSource current_clock_source;

// GPIO register offsets
#define GPFSEL   0x00  // Function select registers
#define GPSET    0x1C  // Pin output set registers
#define GPCLR    0x28  // Pin output clear registers
#define GPLEV    0x34  // Pin level registers
#define GPEDS    0x40  // Event detect status registers
#define GPREN    0x4C  // Rising edge detect enable registers
#define GPFEN    0x58  // Falling edge detect enable registers
#define GPHEN    0x64  // High detect enable registers
#define GPLEN    0x70  // Low detect enable registers
#define GPAREN   0x7C  // Async rising edge detect registers
#define GPAFEN   0x88  // Async falling edge detect registers

// PCM register offsets
#define PCM_CS     0x00  // Control and status
#define PCM_FIFO   0x04  // FIFO data
#define PCM_MODE   0x08  // Mode
#define PCM_RXC    0x0C  // Receive config
#define PCM_TXC    0x10  // Transmit config
#define PCM_DREQ   0x14  // DMA request level
#define PCM_INTEN  0x18  // Interrupt enables
#define PCM_INTSTC 0x1C  // Interrupt status & clear
#define PCM_GRAY   0x20  // Gray mode control

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
        // Export the GPIO
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
        
        // Set up PCM clock - this requires understanding the exact clock divider calculation
        // For example (assuming 500MHz core clock):
        // 500MHz / 61 = 8.196MHz (close to 8.192MHz)
        pcm_map[PCM_MODE/4] = (61 << 10); // Set clock divider
        
        // Enable PCM with appropriate settings
        pcm_map[PCM_CS/4] = 1; // Enable PCM
        
        printf("PCM clock configured for approximately 8.192 MHz\n");
    }
}

// External clock capture thread
static void* external_clock_capture_thread(void* arg) {
    struct pollfd pfd;
    char value[2];
    uint8_t* current_buffer = dma_buffers[current_write_buffer];
    int bit_count = 0;
    uint8_t byte_value = 0;
    
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
    
    // Add these variables for performance monitoring
    unsigned long sample_counter = 0;
    time_t last_report_time = time(NULL);
    
    while (capture_running) {
        bool edge_detected = false;
        
        if (gpio_fd >= 0) {
            // Use sysfs polling approach
            if (poll(&pfd, 1, 100) > 0) {  // 100ms timeout
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
                // Got a complete byte, store it
                current_buffer[buffer_position++] = byte_value;
                bit_count = 0;
                byte_value = 0;
                
                // Check if buffer is full
                if (buffer_position >= DMA_BUFFER_SIZE) {
                    // Advance to next buffer
                    current_write_buffer = (current_write_buffer + 1) % NUM_DMA_BUFFERS;
                    
                    // Check if we're about to overwrite unprocessed data
                    if (current_write_buffer == current_read_buffer) {
                        fprintf(stderr, "Buffer overflow - data is being captured faster than it can be processed\n");
                        // Skip one buffer
                        current_read_buffer = (current_read_buffer + 1) % NUM_DMA_BUFFERS;
                    }
                    
                    current_buffer = dma_buffers[current_write_buffer];
                    buffer_position = 0;
                }
            }
            
            // Increment sample counter
            sample_counter++;
            
            // Report performance every 10 seconds
            time_t current_time = time(NULL);
            if (current_time - last_report_time >= 10) {
                float samples_per_second = sample_counter / 10.0f;
                float expected_rate = TARGET_SAMPLE_RATE / 8.0f; // Bytes per second
                float efficiency = (samples_per_second / expected_rate) * 100.0f;
                
                printf("Performance: %.2f bytes/sec (%.1f%% of target)\n", 
                       samples_per_second, efficiency);
                
                // Reset counter
                sample_counter = 0;
                last_report_time = current_time;
            }
        }
    }
    
    return NULL;
}

// PCM clock capture thread
static void* pcm_clock_capture_thread(void* arg) {
    uint8_t* current_buffer = dma_buffers[current_write_buffer];
    int bit_count = 0;
    uint8_t byte_value = 0;
    
    // Calculate which register and bit to read
    int reg_offset = GPLEV/4;  // GPLEV register offset
    int data_bit = 1 << (GPIO_DATA_PIN % 32);  // Bit mask for our data pin
    
    while (capture_running) {
        // Wait for PCM clock cycle - this would be synchronized with the PCM hardware
        // In practice, we would need to use interrupts or DMA with the PCM clock
        // For now, we just check PCM status
        if (pcm_map[PCM_CS/4] & (1 << 5)) { // Check PCM_CS.RXREADY flag
            // Read data pin on clock edge
            uint32_t level = (gpio_map[reg_offset] & data_bit) ? 1 : 0;
            
            // Pack 8 bits into a byte (MSB first)
            byte_value = (byte_value << 1) | level;
            bit_count++;
            
            if (bit_count == 8) {
                // Got a complete byte, store it
                current_buffer[buffer_position++] = byte_value;
                bit_count = 0;
                byte_value = 0;
                
                // Check if buffer is full
                if (buffer_position >= DMA_BUFFER_SIZE) {
                    // Advance to next buffer
                    current_write_buffer = (current_write_buffer + 1) % NUM_DMA_BUFFERS;
                    
                    // Check if we're about to overwrite unprocessed data
                    if (current_write_buffer == current_read_buffer) {
                        fprintf(stderr, "Buffer overflow - data is being captured faster than it can be processed\n");
                        // Skip one buffer
                        current_read_buffer = (current_read_buffer + 1) % NUM_DMA_BUFFERS;
                    }
                    
                    current_buffer = dma_buffers[current_write_buffer];
                    buffer_position = 0;
                }
            }
        }
    }
    
    return NULL;
}

// Initialize DMA buffers
bool gpio_dma_init(void) {
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

// Start high-speed capture
bool gpio_start_capture(void (*callback)(uint8_t*, size_t)) {
    if (gpio_map == NULL) {
        fprintf(stderr, "GPIO not initialized\n");
        return false;
    }
    
    // Store the callback
    data_ready_callback = callback;
    
    // Set up for capture
    capture_running = true;
    
    // Start the worker thread for processing buffers
    if (pthread_create(&worker_thread, NULL, buffer_processor, NULL) != 0) {
        fprintf(stderr, "Failed to create worker thread\n");
        capture_running = false;
        return false;
    }
    
    // Start the appropriate capture thread based on clock source
    if (current_clock_source == CLOCK_SOURCE_EXTERNAL) {
        if (pthread_create(&capture_thread, NULL, external_clock_capture_thread, NULL) != 0) {
            fprintf(stderr, "Failed to create capture thread\n");
            capture_running = false;
            pthread_cancel(worker_thread);
            pthread_join(worker_thread, NULL);
            return false;
        }
        printf("GPIO capture started with external clock on GPIO %d\n", GPIO_CLOCK_PIN);
    } else {
        if (pthread_create(&capture_thread, NULL, pcm_clock_capture_thread, NULL) != 0) {
            fprintf(stderr, "Failed to create capture thread\n");
            capture_running = false;
            pthread_cancel(worker_thread);
            pthread_join(worker_thread, NULL);
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
    
    // Signal threads to stop
    capture_running = false;
    
    // Wait for threads to terminate
    pthread_join(capture_thread, NULL);
    pthread_join(worker_thread, NULL);
    
    // Process any remaining data in the current buffer
    if (buffer_position > 0 && data_ready_callback != NULL) {
        data_ready_callback(dma_buffers[current_write_buffer], buffer_position);
    }
    
    printf("GPIO capture stopped\n");
    return true;
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

// Get current buffer usage
size_t gpio_get_buffer_usage(void) {
    if (current_write_buffer == current_read_buffer) {
        return buffer_position;
    } else {
        return (NUM_DMA_BUFFERS - 1) * DMA_BUFFER_SIZE + buffer_position;
    }
}

// Get actual sample rate
uint32_t gpio_get_actual_sample_rate(void) {
    // In a real implementation, you would measure the actual achieved rate
    return TARGET_SAMPLE_RATE;
}