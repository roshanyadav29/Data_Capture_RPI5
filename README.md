# Raspberry Pi 5 GPIO Sampler

## Overview
This project implements high-speed digital data capture (8.192 MHz) from Raspberry Pi 5 GPIO pins. It can use either an external clock for precise timing or the internal PCM hardware clock.

## Features
- High-speed data sampling at 8.192 MHz
- Two clock options:
  - External clock for perfect synchronization
  - Internal PCM clock for standalone operation
- Efficient RAM buffering (8MB in 1MB chunks)
- SD card storage for captured data

## Hardware Requirements
- Raspberry Pi 5
- External clock source (for external clock mode)
- Data source connected to GPIO pin 17
- Clock source connected to GPIO pin 18 (for external clock mode)

## Building the Project
```bash
make
```

## Running the Project
The application requires root privileges to access GPIO hardware:
```bash
sudo ./gpio_sampler
# Or use the make run target
make run
```

## Configuration
All configuration parameters are defined in [config.h](config.h):

- Sampling rate: 8.192 MHz by default
- GPIO pins: Data pin (17) and Clock pin (18)
- Buffer size: 1MB chunks, 8 buffers total (8MB RAM buffer)
- Storage location: `/home/pi/gpio_data` by default

To modify these settings, edit the config.h file before building.

## Project Structure
- `src/`: Source code files
  - `main.c`: Application entry point and main loop
  - `gpio_handler.c`: GPIO initialization and sampling functions
  - `buffer_manager.c`: Memory buffer management
  - `data_storage.c`: SD card data storage operations
  - `utils.c`: Utility functions
- `include/`: Header files
  - Corresponding headers for each source file
- `Makefile`: Build configuration

## Performance Considerations
- The system can maintain 8.192 MHz sampling for extended periods
- Captured data is buffered in RAM to prevent SD card write bottlenecks
- Consider using a high-quality SD card with good write performance

## Troubleshooting
- Ensure GPIO pins are correctly connected
- Verify that the external clock signal is stable (if using external clock)
- Check that the application has sufficient permissions to access hardware
- Ensure adequate free space on the SD card