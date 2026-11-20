# Third-party notices

TigoTell's own source code is MIT-licensed (see [LICENSE](LICENSE)). The firmware is built with the following libraries, which PlatformIO downloads at build time and which keep their own licenses. Firmware binaries (release assets) contain them.

| Library | License |
| --- | --- |
| [ElegantOTA](https://github.com/ayushsharma82/ElegantOTA) (via [gongloo's fork](https://github.com/gongloo/ElegantOTA)) | AGPL-3.0 |
| [NetWizard](https://github.com/ayushsharma82/NetWizard) | AGPL-3.0 |
| [WebSerial](https://github.com/ayushsharma82/WebSerial) | AGPL-3.0 |
| [ESPAsyncWebServer](https://github.com/ESP32Async/ESPAsyncWebServer) | LGPL-3.0 |
| [AsyncTCP](https://github.com/ESP32Async/AsyncTCP) | LGPL-3.0 |
| [ArduinoJson](https://github.com/bblanchon/ArduinoJson) | MIT |
| [PubSubClient](https://github.com/knolleary/pubsubclient) | MIT |
| [ESP_Git_OTA](https://github.com/Hsun1031/ESP_Git_OTA) | MIT |
| [Arduino core for ESP32](https://github.com/espressif/arduino-esp32) / ESP-IDF | LGPL-2.1 / Apache-2.0 |
| [Material Design Icons](https://fonts.google.com/icons) (dashboard) | Apache-2.0 |

Because the firmware includes AGPL-3.0 libraries, anyone distributing firmware binaries should make the complete corresponding source available, as this repository does. Protocol knowledge, the barcode check-digit algorithm and node-table details follow the documentation of [taptap](https://github.com/willglynn/taptap) (MIT, © Will Glynn).
