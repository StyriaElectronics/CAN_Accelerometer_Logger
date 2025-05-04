# 📦 CAN Accelerometer Logger
 is a small STM32-based project that reads 3-axis acceleration data from an ADXL345 sensor, buffers it for 1 second at 1 kHz, and then transmits it via UART and CAN bus. Perfect for capturing impact or vibration events in embedded systems.

[![Platform](https://img.shields.io/badge/platform-STM32-blue.svg)]()
[![Interface](https://img.shields.io/badge/interface-CAN%2FUART-green.svg)]()
[![Status](https://img.shields.io/badge/status-Stable-brightgreen.svg)]()
[![License](https://img.shields.io/badge/license-CC0--1.0-lightgrey.svg)]()
[![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)]()
[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()

## 🚀 Features

- 🧭 3-axis accelerometer data from ADXL345 via SPI  
- ⏱️ 1000 samples per second (1 kHz), total 1000 samples in buffer  
- 🟢 Trigger signal via GPIO (rising edge)  
- 💾 Data is buffered in RAM, sent after acquisition  
- 📤 Data output via UART and optionally via CAN  
- 📦 CAN data transmission in chunks with custom protocol  
- 🧪 Simple and compact C code, ideal for lab testing or automotive debug setups  

## ⚙️ Requirements

STM32 microcontroller with:

- SPI (ADXL345)
- UART (for output)
- GPIO (for trigger input)
- CAN (for data transmission)
- ADXL345 accelerometer (3.3 V logic)
- STM32CubeMX / STM32CubeIDE (for initial setup)

## 🔌 Wiring

| Signal       | STM32 Pin  | ADXL345       |
|--------------|------------|----------------|
| SPI MOSI     | e.g. PA7   | SDI            |
| SPI MISO     | e.g. PA6   | SDO            |
| SPI SCK      | e.g. PA5   | SCL            |
| SPI CS       | e.g. PB0   | CS             |
| Trigger Input| e.g. PA8   | (external source) |
| UART TX      | e.g. PA9   | (debug terminal) |
| CAN TX / RX  | any FDCAN-capable pins |

## 🧪 Example Workflow

1. **Boot**  
2. **Wait** for trigger (PA8 ↑)  
3. **Acquire** 1000 XYZ samples (1 ms cadence)  
4. **Transmit**  
   - UART: CSV line per sample  
   - CAN FD: Header + chunked payload → CRC-8 terminator  

---

## 💬 CAN Protocol Overview

| Frame | CAN ID | DLC | Payload |
|-------|--------|-----|---------|
| **Request**  | `0x123` | 1 | `0xAB` (host → logger) |
| **Header**   | `0x321` | 3 | `LEN_H`, `LEN_L`, `CHUNK_CNT` |
| **Payload**  | `0x322` … `0x32F` | up to 64 | Raw data bytes (XYZ buffer) |
| **CRC**      | `0x330` | 1 | CRC-8 (XOR of `LEN` + payload) |

- **LEN** = 2000 bytes (1000 samples × 2 bytes per axis)  
- **CHUNK_CNT** = ceil(LEN / 64)  
- CRC is calculated after the final payload frame and sent in its own frame.

> *Default bitrate:* 500 kbit/s nominal.


## 📂 Folder Structure

```
CAN_Accelerometer_Logger/
├── Core/
│   ├── Src/
│   └── Inc/
├── Drivers/
├── .gitignore
├── README.md
└── LICENSE (CC0-1.0)
```

## 📄 License

This project is licensed under **CC0 1.0 Universal (Public Domain Dedication)**. You can use it freely without attribution – though a ⭐ on GitHub is always appreciated :)

⚠️ **Disclaimer**:  
This project is provided "as is" without warranty of any kind, either expressed or implied.  
In no event shall the author(s) be liable for any claim, damages, or other liability arising from, out of, or in connection with the use or other dealings in this project.

## ✨ Credits

Made with ❤️ by **Styria Electronics** for high-performance embedded data acquisition over CAN bus.

🛠️ Contributions welcome!
