#ifndef BUFFER_MANAGER_H
#define BUFFER_MANAGER_H

#include <stdint.h>
#include <stddef.h>
#include "../config.h"

typedef struct {
    uint8_t data[BUFFER_SIZE];  // Buffer to hold the sampled data
    size_t write_index;          // Index for writing to the buffer
    size_t read_index;           // Index for reading from the buffer
    size_t size;                 // Current size of the buffer
} Buffer;

// Buffer management functions
void buffer_init(Buffer *buffer);
void buffer_write(Buffer *buffer, const uint8_t *data, size_t length);
size_t buffer_read(Buffer *buffer, uint8_t *data, size_t length);
int buffer_is_full(Buffer *buffer);
int buffer_is_empty(Buffer *buffer);

// Raw buffer functions
int init_buffer(size_t size);
int write_to_buffer(const uint8_t *data, size_t length);
size_t read_from_buffer(uint8_t *out_data, size_t length);
void free_buffer(void);

#endif // BUFFER_MANAGER_H