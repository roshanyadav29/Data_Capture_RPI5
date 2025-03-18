#include "utils.h"
#include <stdio.h>
#include <stdlib.h>

void log_error(const char *message) {
    FILE *log_file = fopen("error.log", "a");
    if (log_file != NULL) {
        fprintf(log_file, "ERROR: %s\n", message);
        fclose(log_file);
    } else {
        perror("Failed to open log file");
    }
}

void log_info(const char *message) {
    FILE *log_file = fopen("info.log", "a");
    if (log_file != NULL) {
        fprintf(log_file, "INFO: %s\n", message);
        fclose(log_file);
    } else {
        perror("Failed to open log file");
    }
}

void *allocate_memory(size_t size) {
    void *ptr = malloc(size);
    if (ptr == NULL) {
        log_error("Memory allocation failed");
    }
    return ptr;
}

void free_memory(void *ptr) {
    if (ptr != NULL) {
        free(ptr);
    }
}