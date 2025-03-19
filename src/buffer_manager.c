#include "../include/buffer_manager.h"
#include "../include/utils.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Buffer management variables
static BufferChunk* buffer_chunks = NULL;
static int total_chunks = 0;
static int available_chunks = 0;
static int current_write_chunk = 0;

bool buffer_init(size_t chunk_size, int num_chunks) {
    // Free any existing buffers
    buffer_cleanup();
    
    // Allocate the array of chunks
    buffer_chunks = (BufferChunk*)calloc(num_chunks, sizeof(BufferChunk));
    if (!buffer_chunks) {
        log_error("Failed to allocate buffer chunk array");
        return false;
    }
    
    // Allocate memory for each chunk
    for (int i = 0; i < num_chunks; i++) {
        buffer_chunks[i].data = (uint8_t*)malloc(chunk_size);
        if (!buffer_chunks[i].data) {
            log_error("Failed to allocate buffer chunk %d", i);
            buffer_cleanup();
            return false;
        }
        
        buffer_chunks[i].size = chunk_size;
        buffer_chunks[i].used = 0;
        buffer_chunks[i].ready_for_writing = false;
    }
    
    total_chunks = num_chunks;
    available_chunks = num_chunks;
    current_write_chunk = 0;
    
    log_info("Buffer manager initialized with %d chunks of %zu bytes each", 
             num_chunks, chunk_size);
    return true;
}

void buffer_cleanup(void) {
    if (buffer_chunks) {
        for (int i = 0; i < total_chunks; i++) {
            if (buffer_chunks[i].data) {
                free(buffer_chunks[i].data);
                buffer_chunks[i].data = NULL;
            }
        }
        
        free(buffer_chunks);
        buffer_chunks = NULL;
    }
    
    total_chunks = 0;
    available_chunks = 0;
    current_write_chunk = 0;
}

BufferChunk* buffer_get_next_chunk(void) {
    if (!buffer_chunks || available_chunks <= 0) {
        return NULL;
    }
    
    // Find the next available chunk for writing
    int start_idx = current_write_chunk;
    do {
        if (!buffer_chunks[current_write_chunk].ready_for_writing) {
            // Reset the chunk for reuse
            buffer_chunks[current_write_chunk].used = 0;
            return &buffer_chunks[current_write_chunk];
        }
        
        // Move to next chunk
        current_write_chunk = (current_write_chunk + 1) % total_chunks;
    } while (current_write_chunk != start_idx);
    
    // If we got here, all chunks are marked as ready for writing
    return NULL;
}

void buffer_release_chunk(BufferChunk* chunk) {
    if (!chunk || !buffer_chunks) {
        return;
    }
    
    // Find the chunk in our array
    for (int i = 0; i < total_chunks; i++) {
        if (&buffer_chunks[i] == chunk) {
            buffer_chunks[i].ready_for_writing = true;
            available_chunks--; // Add this line to update available chunks
            break;
        }
    }
}

void buffer_mark_chunk_processed(BufferChunk* chunk) {
    if (!chunk || !buffer_chunks) {
        return;
    }
    
    for (int i = 0; i < total_chunks; i++) {
        if (&buffer_chunks[i] == chunk) {
            buffer_chunks[i].ready_for_writing = false;
            available_chunks++;
            break;
        }
    }
}

bool buffer_is_full(void) {
    return available_chunks <= 0;
}

size_t buffer_get_total_size(void) {
    if (!buffer_chunks) {
        return 0;
    }
    
    return total_chunks * buffer_chunks[0].size;
}

size_t buffer_get_used_size(void) {
    if (!buffer_chunks) {
        return 0;
    }
    
    size_t used = 0;
    for (int i = 0; i < total_chunks; i++) {
        if (buffer_chunks[i].ready_for_writing) {
            used += buffer_chunks[i].used;
        }
    }
    
    return used;
}

int buffer_get_available_chunks(void) {
    return available_chunks;
}

bool buffer_copy_all_data(uint8_t* dest_buffer, size_t buffer_size) {
    if (!dest_buffer || !buffer_chunks) {
        return false;
    }
    
    size_t total_used = buffer_get_used_size();
    if (buffer_size < total_used) {
        return false;
    }
    
    // Copy data from all chunks marked as ready for writing
    size_t offset = 0;
    for (int i = 0; i < total_chunks; i++) {
        if (buffer_chunks[i].ready_for_writing) {
            memcpy(dest_buffer + offset, buffer_chunks[i].data, buffer_chunks[i].used);
            offset += buffer_chunks[i].used;
            
            // Mark chunk as processed to recycle it
            buffer_mark_chunk_processed(&buffer_chunks[i]);
        }
    }
    
    return true;
}