# 📦 CAN Accelerometer Logger

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-STM32-blue.svg)]()
[![Interface](https://img.shields.io/badge/interface-CAN%2FUART-green.svg)]()
[![Status](https://img.shields.io/badge/status-Development-yellow.svg)]()

A compact STM32-based motion sensor module using an ADXL345 digital accelerometer.  
This device logs 3-axis acceleration data at 1 kHz for 1 second (1000 samples) and then transmits the data via **CAN** and **UART**.

---

## 🚀 Features

- ✅ SPI communication with ADXL345
- ✅ Buffered sampling at **1 kHz** for 1 second
- ✅ Data output via:
  - UART (CSV format)
  - CAN bus (fragmented transfer with optional header and CRC)
- ✅ Trigger via GPIO (e.g. rising edge or external button)
- ✅ Visual feedback via onboard LED

---

## 🧠 Technical Overview

| Feature         | Value                    |
|----------------|--------------------------|
| MCU            | STM32G4xx (e.g. G431)    |
| Sensor         | ADXL345 (±8g range)      |
| Sampling Rate  | 1000 Hz (1 kHz)          |
| Buffer Size    | 1000 samples (x,y,z)     |
| Interface      | SPI (ADXL), UART, CAN    |
| CAN Speed      | 500 kbit/s               |
| CRC            | CRC8 for data integrity  |

---

## ⚙️ Trigger Modes

| Mode       | Description                          |
|------------|--------------------------------------|
| GPIO       | Starts on rising edge                |
| Button     | Optional fallback (User button)      |

---

## 🖧 CAN Communication

- The device sends a header frame with total data length.
- Followed by multiple frames (8 bytes each).
- Designed to work with another STM32 as a CAN receiver & I2C bridge.

---

## 📡 UART Output

- Data is streamed as comma-separated values (CSV):
  ```
  124,0,-1032
  123,1,-1033
  ...
  ```

---

## 🛠️ Build Requirements

- STM32CubeIDE or Makefile toolchain
- STM32G4 family MCU (e.g. Nucleo-G431RB)
- ADXL345 accelerometer
- Proper CAN transceivers (e.g. TJA1050)

---

## 📎 License

This project is licensed under the MIT License.
