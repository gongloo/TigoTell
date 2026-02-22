# TigoTell

*Tigo CCA/TAP sniffer built atop Waveshare ESP32-S3-RS485-CAN.*
Non-intrusively monitors communication between Tigo Cloud Connect Advanced (CCA) and Tigo Access Point (TAP) or solar modules to extract real-time telemetry data.

## 📸 Overview
<!--TODO: [TigoTell Screenshot](screenshot.png) -->

**TigoTell** passively listens to the RS485 communication lines of your Tigo solar installation. By sniffing the packets exchanged between the CCA and the TAP, it decodes module-level power statistics, discovering nodes automatically without interfering with the proprietary network. This allows you to log fine-grained solar data into databases like InfluxDB while the official hardware remains unaware.

## ✨ Features
- **Passive Sniffing:** Safe, receive-only RS485 monitoring without disrupting existing communications.
- **Data Parsing:** Decodes Tigo `PowerData`, `Announce`, and `NodeTable` frames (Voltage, Current, Temperature, Duty Cycle, RSSI, etc.).
- **InfluxDB Integration:** Streams parsed telemetry directly to an InfluxDB instance over UDP line-protocol.
- **REST APIs:** Live JSON endpoints for power data, system discovery statistics, and parser health (`/json` and `/version`).
- **WebSerial Debugging:** Built-in web-based serial console.
- **OTA Updates:** Fast and secure Over-The-Air firmware updates using ElegantOTA.
- **WiFi Management:** AP-based initial setup for seamless network provisioning utilizing NetWizard.

## 📋 Requirements
### Hardware
- **Base Board**: Waveshare ESP32-S3-RS485-CAN (or a comparable ESP32-S3 board with an RS485 transceiver).
- **Wiring**: Connection to the RS485 A and B lines between the Tigo CCA and TAP. 

### Software / Dependencies
- [pioarduino](https://github.com/pioarduino/platform-espressif32) Core (CLI) or VSCode IDE extension.
- *(Optional)* An **InfluxDB** instance (configured to receive UDP datagrams) if you wish to record time-series metrics.

## 🚀 Getting Started

### 1. Configuration
Clone the repository and prepare the required configuration files:
```bash
git clone https://github.com/gongloo/TigoTell.git
cd TigoTell
```

Copy the example configuration files and edit them to match your setup:
```bash
cp src/config.example.h src/config.h
cp platformio_upload.example.ini platformio_upload.ini
```

- In `src/config.h`, uncomment and define your InfluxDB hostname/IP, port, and the exact pins your RS485 transceiver uses (defaults are typically TX=17, RX=18, EN=21).
- In `platformio_upload.ini`, configure your OTA credentials (`custom_username` and `custom_password`).

### 2. Build and Flash
Use pioarduino to compile the firmware and flash it via USB:
```bash
pio run -e esp32-s3-devkitm-1 -t upload
```
*Important Note for First Upload: For the initial flash over USB, Over-The-Air (OTA) updates will not be available yet. You must comment out the `upload_protocol = custom` line in `platformio.ini` to force a USB upload instead of attempting an OTA update. You can uncomment it again for future OTA flashes.*

*Note: Make sure to also run the `uploadfs` target if LittleFS requires a fresh image, although normal firmware updates bundle necessary files.*

### 3. Network Provisioning
1. Once flashed, the ESP32 will broadcast a WiFi Access Point named `TigoTell`.
2. Connect to this network on your phone or computer.
3. A captive portal should appear (or navigate to `192.168.4.1`).
4. Enter your local WiFi credentials to connect the device to your network.

### 4. Usage
Once connected to your local network, you can access the device in your browser via mdns (`http://TigoTell.local`) or its assigned IP address.
- **`/json`**: Returns a snapshot of all discovered nodes and their latest parsed data, including system stats.
- **`/version`**: Returns the current firmware version and build timestamp.
- **`/update`**: Access the ElegantOTA firmware update portal.
- **`/webserial`**: View live serial log output.

## 🙏 Credits & Acknowledgments
This project stands on the shoulders of the incredible reverse-engineering work done by the community. A massive thank you to the following projects which provided the foundation for decoding the Tigo protocols:

- **[willglynn/taptap](https://github.com/willglynn/taptap)**: Excellent documentation and implementation of the Tigo wireless protocol.
- **[tictactom/tigo_server](https://github.com/tictactom/tigo_server)**: Fundamental research and parsing logic for the Tigo CCA/TAP communications.
- **Google Gemini**: For AI pair-programming assistance in building and structuring this project!
