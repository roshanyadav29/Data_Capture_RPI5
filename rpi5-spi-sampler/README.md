# rpi5-spi-sampler

## Overview
The rpi5-spi-sampler project is designed to facilitate high-speed data sampling from SPI devices using the Raspberry Pi 5. The project captures data quickly into a RAM buffer and later writes it to an SD card, allowing for efficient data handling and storage.

## Project Structure
```
rpi5-spi-sampler
├── src
│   ├── main.c            # Entry point of the application
│   ├── spi_handler.c     # SPI communication handling
│   ├── buffer_manager.c   # RAM buffer management
│   ├── data_storage.c     # Data storage to SD card
│   └── utils.c           # Utility functions
├── include
│   ├── spi_handler.h     # Header for SPI functions
│   ├── buffer_manager.h   # Header for buffer management
│   ├── data_storage.h     # Header for data storage functions
│   └── utils.h           # Header for utility functions
├── test
│   ├── test_spi.c        # Unit tests for SPI functions
│   └── test_buffer.c     # Unit tests for buffer management
├── Makefile              # Build instructions
├── config.h              # Configuration settings
└── README.md             # Project documentation
```

## Features
- High-speed data sampling from SPI devices.
- Efficient RAM buffer management for quick data capture.
- Controlled data storage to SD card to prevent data loss.
- Modular design with separate source files for different functionalities.

## Setup Instructions
1. Clone the repository:
   ```
   git clone <repository-url>
   cd rpi5-spi-sampler
   ```

2. Install necessary dependencies (if any).

3. Build the project using the Makefile:
   ```
   make
   ```

4. Run the application:
   ```
   ./rpi5-spi-sampler
   ```

## Usage
- Configure the SPI parameters in `config.h`.
- Adjust buffer size and other settings as needed.
- Use the provided test files to verify functionality.

## Dependencies
- Raspberry Pi libraries for SPI communication.
- Standard C libraries.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.