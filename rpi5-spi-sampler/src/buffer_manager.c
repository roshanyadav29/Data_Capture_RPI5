#include "../include/buffer_manager.h"
#include "../config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Static buffer for raw buffer functions
static uint8_t *raw_buffer = NULL;
static size_t raw_buffer_size = 0;
static size_t write_index = 0;

// Buffer struct functions
void buffer_init(Buffer *buffer) {
    if (buffer == NULL) {
        return;
    }
    
    buffer->write_index = 0;
    buffer->read_index = 0;
    buffer->size = 0;
    memset(buffer->data, 0, BUFFER_SIZE);
}

void buffer_write(Buffer *buffer, const uint8_t *data, size_t length) {
    if (buffer == NULL || data == NULL) {
        return;
    }
    
    // Only write what fits in the buffer
    size_t available = BUFFER_SIZE - buffer->size;
    size_t to_write = (length > available) ? available : length;
    
    for (size_t i = 0; i < to_write; i++) {
        buffer->data[(buffer->write_index + i) % BUFFER_SIZE] = data[i];
    }
    
    buffer->write_index = (buffer->write_index + to_write) % BUFFER_SIZE;
    buffer->size += to_write;
}

size_t buffer_read(Buffer *buffer, uint8_t *data, size_t length) {
    if (buffer == NULL || data == NULL || buffer->size == 0) {
        return 0;
    }
    
    size_t to_read = (length > buffer->size) ? buffer->size : length;
    
    for (size_t i = 0; i < to_read; i++) {
        data[i] = buffer->data[(buffer->read_index + i) % BUFFER_SIZE];
    }
    
    buffer->read_index = (buffer->read_index + to_read) % BUFFER_SIZE;
    buffer->size -= to_read;
    
    return to_read;
}

int buffer_is_full(Buffer *buffer) {
    return (buffer != NULL && buffer->size >= BUFFER_SIZE);
}

int buffer_is_empty(Buffer *buffer) {
    return (buffer == NULL || buffer->size == 0);
}

// Raw buffer functions
int init_buffer(size_t size) {
    if (raw_buffer != NULL) {
        free(raw_buffer);
    }
    raw_buffer = (uint8_t *)malloc(size);
    if (raw_buffer == NULL) {
        return -1; // Memory allocation failed
    }
    raw_buffer_size = size;
    write_index = 0;
    return 0; // Success
}

int write_to_buffer(const uint8_t *data, size_t length) {
    if (write_index + length > raw_buffer_size) {
        return -1; // Not enough space in buffer
    }
    memcpy(raw_buffer + write_index, data, length);
    write_index += length;
    return 0; // Success
}

size_t read_from_buffer(uint8_t *out_data, size_t length) {
    size_t read_length = (length > write_index) ? write_index : length;
    memcpy(out_data, raw_buffer, read_length);
    memmove(raw_buffer, raw_buffer + read_length, write_index - read_length);
    write_index -= read_length;
    return read_length; // Number of bytes read
}

void free_buffer() {
    if (raw_buffer != NULL) {
        free(raw_buffer);
        raw_buffer = NULL;
        raw_buffer_size = 0;
        write_index = 0;
    }
}