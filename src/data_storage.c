#include "../include/data_storage.h"
#include "../config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>

static FILE *data_file = NULL;
static char filepath[256];

void data_storage_init(void) {
    // Create directory if it doesn't exist
    char mkdir_cmd[256];
    sprintf(mkdir_cmd, "mkdir -p %s", SD_CARD_PATH);
    system(mkdir_cmd);
    
    // Generate a timestamp for the filename
    time_t now;
    struct tm *tm_info;
    char timestamp_str[64];
    
    time(&now);
    tm_info = localtime(&now);
    strftime(timestamp_str, 64, "%Y%m%d_%H%M%S", tm_info);
    
    sprintf(filepath, "%s/gpio_capture_%s.bin", SD_CARD_PATH, timestamp_str);
    
    // Create the data file
    data_file = fopen(filepath, "wb");
    if (data_file == NULL) {
        perror("Failed to open data file for writing");
        return;
    }
    
    printf("Data will be stored to: %s\n", filepath);
}

void write_data_to_sd(uint8_t *buffer, size_t size) {
    if (data_file == NULL || buffer == NULL || size == 0) {
        return;
    }
    
    // Write in chunks to show progress
    const size_t chunk_size = 1024 * 1024; // 1MB chunks
    size_t remaining = size;
    size_t offset = 0;
    size_t total_written = 0;
    
    while (remaining > 0) {
        size_t to_write = (remaining > chunk_size) ? chunk_size : remaining;
        
        size_t written = fwrite(buffer + offset, 1, to_write, data_file);
        if (written != to_write) {
            perror("Failed to write all data to file");
            break;
        }
        
        // Update progress
        offset += written;
        remaining -= written;
        total_written += written;
        
        // Show progress every 5MB
        if (total_written % (5 * 1024 * 1024) == 0) {
            printf("Written %zu bytes\n", total_written);
        }
    }
    
    // Flush the data to ensure it's written to disk
    fflush(data_file);
}

void data_storage_finalize(void) {
    if (data_file != NULL) {
        fclose(data_file);
        data_file = NULL;
        printf("Data file closed successfully\n");
    }
}