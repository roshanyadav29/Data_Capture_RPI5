#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>

// Function prototypes
void log_error(const char *message);
void log_info(const char *message);
void delay_ms(unsigned int milliseconds);
void *allocate_memory(size_t size);
void free_memory(void *ptr);

#endif // UTILS_H