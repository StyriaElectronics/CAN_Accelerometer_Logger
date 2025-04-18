# 📦 CAN Accelerometer Logger
 is a small STM32-based project that reads 3-axis acceleration data from an ADXL345 sensor, buffers it for 1 second at 1 kHz, and then transmits it via UART and CAN bus. Perfect for capturing impact or vibration events in embedded systems.

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-STM32-blue.svg)]()
[![Interface](https://img.shields.io/badge/interface-CAN%2FUART-green.svg)]()
[![Status](https://img.shields.io/badge/status-Development-yellow.svg)]()

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

1. Startup  
2. Wait for trigger input (PA8 HIGH)  
3. Sample 1000 accelerometer values (XYZ)  
4. After sampling:  
   - Send data over UART (CSV format)  
   - Send header + chunked payload via CAN  

## 💬 CAN Protocol Overview

- **Request:** ID 0x123, single byte 0xAB to request data  
- **Header:** ID 0x321, first frame = 2 bytes length (e.g. 0x27 0x11 = 10001)  
- **Payload:** subsequent frames with raw buffer data  

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

This project is licensed under [CC0 1.0 Universal](https://creativecommons.org/publicdomain/zero/1.0/) (Public Domain Dedication). You can use it without attribution – though a ⭐ on GitHub is always appreciated :)

## ✨ Credits

Made with ❤️ by **Styria Electronics** for high-performance embedded data acquisition over CAN bus.

🛠️ Contributions welcome!
