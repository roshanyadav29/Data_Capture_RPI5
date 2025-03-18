#include <stdio.h>
#include <stdlib.h>
#include "../include/data_storage.h"
#include "../include/buffer_manager.h"
#include "../config.h"

#define SD_CARD_PATH "/mnt/sdcard/data.txt"

void data_storage_init(void) {
    // Initialize data storage - create file if it doesn't exist
    FILE *file = fopen(SD_CARD_PATH, "w");
    if (file == NULL) {
        perror("Failed to initialize data storage");
        return;
    }
    fclose(file);
}

void write_data_to_sd(uint8_t *buffer, size_t size) {
    if (buffer == NULL || size == 0) {
        return;
    }

    FILE *file = fopen(SD_CARD_PATH, "a");
    if (file == NULL) {
        perror("Failed to open SD card for writing");
        return;
    }

    // Write binary data to file
    size_t written = fwrite(buffer, 1, size, file);
    if (written != size) {
        perror("Failed to write all data to SD card");
    }

    fclose(file);
}

void data_storage_finalize(void) {
    // Any cleanup operations
    printf("Data storage finalized\n");
}

void write_data_to_sd_card(Buffer *buffer) {
    FILE *file = fopen(SD_CARD_PATH, "a");
    if (file == NULL) {
        perror("Failed to open SD card for writing");
        return;
    }

    for (size_t i = 0; i < buffer->write_index; i++) {
        fprintf(file, "%d\n", buffer->data[i]);
    }

    fclose(file);
}

void flush_buffer_to_sd_card(Buffer *buffer) {
    if (buffer->write_index > 0) {
        write_data_to_sd_card(buffer);
        buffer->write_index = 0; // Reset the write index after flushing
    }
}