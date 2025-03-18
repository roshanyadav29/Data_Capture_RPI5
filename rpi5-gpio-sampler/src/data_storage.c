#include "../include/data_storage.h"
#include "../config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static FILE *data_file = NULL;
static char timestamp_str[64];

void data_storage_init(void) {
    // Create the directory if it doesn't exist
    char mkdir_cmd[256];
    sprintf(mkdir_cmd, "mkdir -p %s", SD_CARD_PATH);
    system(mkdir_cmd);
    
    // Generate a timestamp for the filename
    time_t now;
    struct tm *tm_info;
    time(&now);
    tm_info = localtime(&now);
    strftime(timestamp_str, 64, "%Y%m%d_%H%M%S", tm_info);
    
    char filepath[256];
    sprintf(filepath, "%s/capture_%s.bin", SD_CARD_PATH, timestamp_str);
    
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
    
    size_t written = fwrite(buffer, 1, size, data_file);
    if (written != size) {
        perror("Failed to write all data to file");
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