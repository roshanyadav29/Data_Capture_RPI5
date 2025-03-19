#ifndef BUFFER_MANAGER_H
#define BUFFER_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Buffer chunk structure
typedef struct {
    uint8_t* data;
    size_t size;
    size_t used;
    bool ready_for_writing;
} BufferChunk;

// Buffer manager initialization/cleanup
bool buffer_init(size_t chunk_size, int num_chunks);
void buffer_cleanup(void);

// Buffer management functions
BufferChunk* buffer_get_next_chunk(void);
void buffer_release_chunk(BufferChunk* chunk);
void buffer_mark_chunk_processed(BufferChunk* chunk);
bool buffer_is_full(void);

// Status functions
size_t buffer_get_total_size(void);
size_t buffer_get_used_size(void);
int buffer_get_available_chunks(void);

bool buffer_copy_all_data(uint8_t* dest_buffer, size_t buffer_size);

#endif // BUFFER_MANAGER_H