# Building and flashing

## Toolchain

[PlatformIO](https://platformio.org/) with the `espressif32` platform and the Arduino
framework, targeting `seeed_xiao_esp32s3`. Everything is declared in
[platformio.ini](../platformio.ini); no manual SDK setup is needed.

Libraries are resolved automatically: ezTime, PubSubClient, WebSerial, and ESP Async WebServer.

> **`lib_ignore = AsyncTCP_RP2040W` is required.** Without it the dependency finder pulls the
> RP2040 TCP library into an ESP32-S3 build and the link fails. Do not remove it.

## Credentials

`include/credentials.h` is not in the repository. Create it before the first build:

```c
#ifndef CREDENTIALS_H
#define CREDENTIALS_H

#define WIFI_SSID      "your-ssid"
#define WIFI_PASSWORD  "your-password"
#define MQTT_SERVER    "192.168.1.10"     // broker IP or hostname
#define MQTT_USER_NAME "mqtt-user"
#define MQTT_PASSWORD  "mqtt-password"

#endif
```

Those five symbols are all the firmware requires.

## Build environments

Four are defined. `release_wCOM` is the default.

| Environment | Upload via | Use it for |
|---|---|---|
| `release_wCOM` | USB serial (`esptool`) | First flash, and any time the device is on the bench |
| `release_wOTA` | Network (`espota`) | Normal updates once the device is installed |
| `debug` | Built-in USB JTAG | Stepping through code with a debugger |
| `i2c-scan` | USB serial | Standalone I²C bus diagnostic — see below |

```bash
pio run -e release_wCOM -t upload      # build + flash over USB
pio run -e release_wOTA -t upload      # build + flash over the network
pio run -e release_wCOM                # build only, no flash
```

Set `upload_port` / `monitor_port` in [platformio.ini](../platformio.ini) to match your
system, and `upload_port` in the OTA environment to your device's hostname or IP.

## First flash must be over USB

There is no bootloader-level network recovery. Flash `release_wCOM` over USB once, confirm the
device joins WiFi and reaches the broker, and only then switch to OTA.

If an OTA update ever leaves the device unresponsive, it goes back on a USB cable.

## Reading the log

Once installed, the device is in an irrigation box and will not see a USB cable again. Two
remote options:

- **WebSerial** at `http://<device-ip>/webserial` — the same stream as the serial console.
  Sending any character prints a one-line status summary including a live pressure and
  temperature reading, which is the only way to poll the sensor when no water is flowing.
- **MQTT** — the idle heartbeat topics carry pressure, temperature, WiFi SSID and RSSI every
  30 minutes.

Log output produced *before* WiFi comes up is buffered and replayed the moment a WebSerial
client connects, so boot messages are not lost.

## The `i2c-scan` environment

A standalone diagnostic sketch, [test/i2c_scan.cpp](../test/i2c_scan.cpp), that replaces the
main firmware entirely (`build_src_filter` excludes `src/`). It scans every address and reports
bus state, which distinguishes "sensor not responding" from "bus is wedged."

```bash
pio run -e i2c-scan -t upload
pio device monitor -e i2c-scan
```

**Open the serial monitor before resetting the board** — the sketch blocks on `while (!Serial)`
so that all of its steps are captured.

## COM port gotcha

The XIAO ESP32S3 presents **two different USB identities**:

| Mode | VID:PID |
|---|---|
| Running the sketch | `2886:0059` (Seeed) |
| ROM bootloader | `303A:1001` (Espressif) |

Windows can assign these different COM numbers, so the port that worked for the serial monitor
is not necessarily the port that works for upload — and the number can change after a failed
flash. If upload fails with a port error, check Device Manager for the current number rather
than assuming.

Two related traps:

- **"Could not open COM*n*"** most often means something else already holds the port — a serial
  monitor in another window, for instance. Close it and retry before suspecting the board.
- **`board_build.mcu` cannot rescue a wrong `board` setting.** Overriding the MCU affects only
  the compiler; the OpenOCD target, linker script, partition table and pin variant all still
  come from `board`. Setting the correct board is the only fix.
