#ifndef DATA_STORAGE_H
#define DATA_STORAGE_H

#include <stdint.h>

// Function to initialize the data storage system
void data_storage_init(void);

// Function to write data from the buffer to the SD card
void write_data_to_sd(uint8_t *buffer, size_t size);

// Function to finalize the data storage process
void data_storage_finalize(void);

#endif // DATA_STORAGE_H