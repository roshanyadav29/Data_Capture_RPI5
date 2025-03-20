# Raspberry Pi 5 GPIO Sampler Makefile
CC = gcc
CFLAGS = -Wall -Wextra -O2 -I./include
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
	rm -f $(OBJ) $(TARGET)

run: $(TARGET)
	sudo ./$(TARGET)

test:
	$(CC) $(CFLAGS) -DDRY_RUN -o gpio_sampler_test $(SRC_FILES) $(LIBS)

.PHONY: all clean run test
