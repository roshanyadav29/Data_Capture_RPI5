#ifndef DATA_STORAGE_H
#define DATA_STORAGE_H

#include <stdint.h>
#include <stddef.h>

void data_storage_init(void);
void write_data_to_sd(uint8_t *buffer, size_t size);
void data_storage_finalize(void);

#endif // DATA_STORAGE_H