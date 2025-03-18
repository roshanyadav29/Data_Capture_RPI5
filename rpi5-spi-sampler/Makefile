CC = gcc
CFLAGS = -Iinclude -Wall -Wextra -O2
SRC = src/main.c src/spi_handler.c src/buffer_manager.c src/data_storage.c src/utils.c
OBJ = $(SRC:.c=.o)
TARGET = spi_sampler

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean