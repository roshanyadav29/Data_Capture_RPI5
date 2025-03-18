#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "buffer_manager.h"

#define BUFFER_SIZE 1024

void test_buffer_allocation() {
    char *buffer = allocate_buffer(BUFFER_SIZE);
    if (buffer == NULL) {
        printf("Buffer allocation failed\n");
        return;
    }
    printf("Buffer allocated successfully\n");
    free(buffer);
}

void test_buffer_write_read() {
    char *buffer = allocate_buffer(BUFFER_SIZE);
    if (buffer == NULL) {
        printf("Buffer allocation failed\n");
        return;
    }

    const char *test_data = "Hello, SPI!";
    write_to_buffer(buffer, test_data, strlen(test_data) + 1);
    
    char read_data[BUFFER_SIZE];
    read_from_buffer(buffer, read_data, BUFFER_SIZE);
    
    if (strcmp(test_data, read_data) == 0) {
        printf("Buffer write/read test passed\n");
    } else {
        printf("Buffer write/read test failed\n");
    }

    free(buffer);
}

void test_buffer_overflow() {
    char *buffer = allocate_buffer(BUFFER_SIZE);
    if (buffer == NULL) {
        printf("Buffer allocation failed\n");
        return;
    }

    const char *overflow_data = "This data is too long for the buffer and should cause an overflow!";
    write_to_buffer(buffer, overflow_data, strlen(overflow_data) + 1);
    
    char read_data[BUFFER_SIZE];
    read_from_buffer(buffer, read_data, BUFFER_SIZE);
    
    printf("Buffer overflow test completed. Read data: %s\n", read_data);
    
    free(buffer);
}

int main() {
    test_buffer_allocation();
    test_buffer_write_read();
    test_buffer_overflow();
    return 0;
}