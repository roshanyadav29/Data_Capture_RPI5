#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "spi_handler.h"
#include "buffer_manager.h"

#define TEST_BUFFER_SIZE 1024

void test_spi_initialization() {
    int result = spi_init();
    if (result != 0) {
        printf("SPI Initialization failed\n");
    } else {
        printf("SPI Initialization succeeded\n");
    }
}

void test_spi_read() {
    uint8_t data[10];
    int result = spi_read(data, sizeof(data));
    if (result < 0) {
        printf("SPI Read failed\n");
    } else {
        printf("SPI Read succeeded: ");
        for (int i = 0; i < result; i++) {
            printf("%02X ", data[i]);
        }
        printf("\n");
    }
}

void test_buffer_management() {
    Buffer *buffer = buffer_create(TEST_BUFFER_SIZE);
    if (buffer == NULL) {
        printf("Buffer creation failed\n");
        return;
    }

    for (int i = 0; i < TEST_BUFFER_SIZE; i++) {
        buffer_write(buffer, (uint8_t)i);
    }

    printf("Buffer write succeeded\n");

    for (int i = 0; i < TEST_BUFFER_SIZE; i++) {
        uint8_t value = buffer_read(buffer);
        if (value != (uint8_t)i) {
            printf("Buffer read failed at index %d: expected %d, got %d\n", i, i, value);
            break;
        }
    }

    printf("Buffer read succeeded\n");
    buffer_destroy(buffer);
}

int main() {
    test_spi_initialization();
    test_spi_read();
    test_buffer_management();
    return 0;
}