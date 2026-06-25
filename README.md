# High-Speed Embedded Data Acquisition & DSP Pipeline

![Language](https://img.shields.io/badge/Language-C-blue.svg)
![MCU](https://img.shields.io/badge/MCU-STM32-red.svg)
![DSP](https://img.shields.io/badge/Library-CMSIS--DSP-green.svg)

## Overview
This repository contains C firmware for a high-speed, interrupt-driven data acquisition and Digital Signal Processing (DSP) pipeline. Designed for industrial motor monitoring and mechanical diagnostics, the system continuously fuses vibration data (SPI) and electrical current draw (ADC) into a dense 21-feature array.

By leveraging the ARM Cortex-M4's hardware Floating-Point Unit (FPU) and the CMSIS-DSP library, the system extracts critical time-domain and frequency-domain metrics in real-time. It outputs clean, normalized feature vectors perfectly structured for data logging or as the ingestion layer for downstream edge machine learning models.

## Key Engineering Features
* **Deterministic Acquisition:** 800 Hz hardware-triggered interrupts (EXTI) guarantee zero drift in sample timing.
* **Race-Condition Prevention:** A Ping-Pong (double buffering) memory architecture isolates the high-speed Interrupt Service Routine (ISR) from the main execution loop, preventing data corruption.
* **Hardware-Accelerated DSP:** Utilizes ARM CMSIS-DSP (`arm_rfft_fast_f32`, `arm_rms_f32`) for real-time zero-centering, Fast Fourier Transforms (FFT), and spectral energy binning without blocking the CPU.
* **Synchronous Sensor Fusion:** Merges asynchronous 3-axis SPI accelerometer data with precisely polled 12-bit ADC current readings within a 1.25 ms execution window.

## Hardware Architecture

### Components Used
* **MCU:** STM32 Nucleo (ARM Cortex-M4 with FPU)
* **Vibration Sensor:** ADXL345 (Configured for Full-Resolution, ±16g, SPI)
* **Current Sensor:** ACS712 (Analog Output)

### Pin Configuration
| Component | STM32 Pin | Peripheral / Function | Notes |
| :--- | :--- | :--- | :--- |
| **ADXL345 CS** | `PA4` | GPIO Output | Manual Chip Select for SPI framing |
| **ADXL345 SCK**| `PA5` | SPI1_SCK | Serial Clock |
| **ADXL345 SDO**| `PA6` | SPI1_MISO | Master In Slave Out |
| **ADXL345 SDA**| `PA7` | SPI1_MOSI | Master Out Slave In |
| **ADXL345 INT1**| `PB0` | EXTI0 | 800 Hz Hardware Interrupt |
| **ACS712 OUT** | `PA0` | ADC1_IN0 | 12-bit Polled ADC (56-cycle sample time) |
| **Debug Tx** | `PA2` | USART2_TX | 115200 Baud (printf retargeted) |

> **⚠️ Hardware Warning for ACS712:** The ACS712 is a 5V sensor with a 2.5V quiescent output. While safe for idle/low-current prototyping on the STM32's 3.3V ADC, a voltage divider or op-amp buffer is required for production deployments to prevent ADC overvoltage during heavy motor load spikes.

## Software Pipeline

1. **Acquisition (ISR):** The ADXL345 fires an 800 Hz interrupt. The ISR triggers a fast burst SPI read, polls the ADC, scales the data to physical units, and safely writes to the active buffer.
2. **Buffer Swap:** Once 256 samples are collected (every 0.32 seconds), a volatile pointer instantly swaps the ISR to the secondary buffer, releasing the primary buffer to the main loop lock-free.
3. **Dimensionality Reduction:** The raw 1,024-point buffer (256 samples × 4 channels) is mathematically compressed into a 21-feature vector:
   * **Vibration (X, Y, Z Axes):** RMS, Peak-to-Peak, Crest Factor, Low/Mid/High FFT Energy Bins.
   * **Electrical (Current):** Mean Voltage (Load), RMS Voltage (Power), Peak-to-Peak Ripple.
4. **Output:** The 21-feature array is printed via UART, allowing for rapid dataset generation for external analysis.

## Build Instructions (STM32CubeIDE)

1. Clone this repository:
   ```bash
   git clone [https://github.com/](https://github.com/)[YourUsername]/Motor-DAQ-DSP-Pipeline.git
