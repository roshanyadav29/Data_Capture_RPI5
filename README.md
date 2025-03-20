# Raspberry Pi 5 GPIO Sampler

## Overview
This project implements high-speed digital data capture (8.192 MHz) from Raspberry Pi 5 GPIO pins using an external clock for precise timing.

## Features
- High-speed data sampling at 8.192 MHz
- External clock for perfect synchronization
- Efficient RAM buffering (8MB in 1MB chunks)
- SD card storage for captured data

## Hardware Requirements
- Raspberry Pi 5
- External clock source at 8.192 MHz
- Data source connected to GPIO pin 17
- Clock source connected to GPIO pin 18

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

- Sampling rate: 8.192 MHz (defined by external clock)
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
- Verify that the external clock signal is stable and at the correct frequency
- Check that the application has sufficient permissions to access hardware
- Ensure adequate free space on the SD card