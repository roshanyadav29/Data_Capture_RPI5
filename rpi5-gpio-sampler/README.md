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