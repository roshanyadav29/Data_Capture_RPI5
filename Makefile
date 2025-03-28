# Raspberry Pi 4 GPIO Sampler Makefile
CC = gcc
CFLAGS = -Wall -Wextra -O2 -I./include -D_GNU_SOURCE
LDFLAGS = -pthread

SRC = src/main.c src/gpio_handler.c src/buffer_manager.c src/data_storage.c src/utils.c
OBJ = $(SRC:.c=.o)
TARGET = gpio_sampler

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJ)

run: $(TARGET)
	sudo ./$(TARGET)

.PHONY: all clean run
