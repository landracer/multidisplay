# MultiDisplay ↔ OpenDash Integration Guide

> How MultiDisplay sensor data flows into the OpenDash dashboard system.
> Written March 2026. This document is the authoritative reference for
> anyone (developer or LLM) working on the integration between these
> two projects.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [What MultiDisplay Provides](#what-multidisplay-provides)
3. [Bluetooth Link (HC-06 → HC-05)](#bluetooth-link)
4. [Binary Serial Protocol](#binary-serial-protocol)
5. [OpenDash Receiver Architecture](#opendash-receiver-architecture)
6. [What Was Changed in Each Project](#what-was-changed)
7. [Current Status & Known Issues](#current-status)
8. [Future: OBDII/ELM327 Expansion](#future-obdii)
9. [File Reference](#file-reference)

---

## 1. System Overview <a name="system-overview"></a>

```
  ┌───────────────────────────────────────┐
  │        MultiDisplay v2 (MD)           │
  │   ATmega2560 / Seeeduino Mega         │
  │   Reads: 8×EGT, 8×VDO, RPM, boost,   │
  │          throttle, lambda, LMM, etc.   │
  │   Serial2 → HC-06 slave ("mdv2")      │
  │   115200 baud, binary 95-byte frames  │
  └────────────────┬──────────────────────┘
                   │ Bluetooth SPP (auto-paired)
                   ▼
  ┌───────────────────────────────────────┐
  │        HC-05 Master Module            │
  │   Pre-configured: ROLE=1, BIND=mdv2   │
  │   Wired to Waveshare ESP32-S3-LCD-2.8C│
  │   HC-05 TX → GPIO20 (J9 DP / USB D+) │
  └────────────────┬──────────────────────┘
                   │ UART1 @ 115200
                   ▼
  ┌───────────────────────────────────────┐
  │   OpenDash LEFT Gauge Pod             │
  │   opendash_uart.c parses binary frames│
  │   Forwards to CENTER via ESP-NOW      │
  └────────────────┬──────────────────────┘
                   │ ESP-NOW (DATA_RESPONSE)
                   ▼
  ┌───────────────────────────────────────┐
  │   OpenDash CENTER Display             │
  │   Displays all values, logs to SD     │
  │   Forwards to RIGHT via ESP-NOW       │
  └───────────────────────────────────────┘
```

---

## 2. What MultiDisplay Provides <a name="what-multidisplay-provides"></a>

MD is the **sensor acquisition package**. It reads automotive sensors through
the Seeeduino Mega's analog/digital pins and streams everything over serial.

### Sensor Inputs

| Sensor | Count | Interface | MD Source Code |
|---|---|---|---|
| Type K EGT | 8 channels | SPI (MAX31855) | `SensorData.cpp` |
| VDO Pressure | 3 channels | Analog | `SensorData.cpp` |
| VDO Temperature | 3 channels | Analog | `SensorData.cpp` |
| RPM | 1 | Digital interrupt | `SensorData.cpp` |
| Boost / MAP | 1 | Analog | `SensorData.cpp` |
| Throttle Position | 1 | Analog | `SensorData.cpp` |
| Lambda / O2 | 1 | Analog | `SensorData.cpp` |
| Mass Air Flow (LMM) | 1 | Analog | `SensorData.cpp` |
| Battery Voltage | 1 | Analog | `SensorData.cpp` |
| Case Temperature | 1 | Analog | `SensorData.cpp` |
| Vehicle Speed | 1 | Computed/ECU | `SensorData.cpp` |
| Gear Position | 1 | Computed (RPM÷speed) | `SensorData.cpp` |
| N75 Boost Duty | 1 | PWM output feedback | `RPMBoostController.cpp` |
| Requested Boost | 1 | PID target | `RPMBoostController.cpp` |
| EFR Turbo Speed | 1 | Digital (BorgWarner) | `SensorData.cpp` |
| Knock Sensor | 1 | Analog | `SensorData.cpp` |

### Build Configurations

MD firmware has two major build variants controlled by `MultidisplayDefines.h`:

| Define | Vehicle | EGT Slots | Extra Fields |
|---|---|---|---|
| `VR6_MOTRONIC` | VR6 with Bosch Motronic | 3 active, 8 in frame | 35 bytes Digifant/ECU data in padding |
| `DIGIFANT` (default) | Digifant-1 ECU | configurable via `NUMBER_OF_ATTACHED_TYPE_K` | 35 bytes zeros in padding |

> **Important:** The binary frame **always contains 8 EGT slots** regardless
> of how many physical thermocouples are installed. Unused slots read 0.

---

## 3. Bluetooth Link <a name="bluetooth-link"></a>

### Hardware

| Role | Module | Address | Name | Baud |
|---|---|---|---|---|
| **Slave** (on MD) | HC-06 | `2017:01:117753` | "mdv2" | 115200 |
| **Master** (on OpenDash) | HC-05 | — | — | 115200 |

### Pairing

The HC-05 and HC-06 are pre-paired using AT commands. Once powered on,
the HC-05 master automatically connects to the HC-06 slave and begins
forwarding the serial data stream.

See `BLUETOOTH_PAIRING.md` in the OpenDash repo for the full AT command
sequence.

### Wiring (OpenDash Side)

HC-05 is powered from the Waveshare ESP32-S3-LCD-2.8C board:

| HC-05 Pin | Connects To | Notes |
|---|---|---|
| VCC | 5V | From USB or external supply |
| GND | GND | Common ground |
| TXD | GPIO20 (J9 DP) | HC-05 output → ESP32-S3 UART1 RX |
| RXD | GPIO19 (J9 DN) | Not currently used (TX disabled) |
| KEY/EN | Not connected | AT mode not used at runtime |

> **GPIO20 (USB D+) reclaim:** The ESP32-S3 USB-Serial/JTAG peripheral
> normally owns GPIO19/20. OpenDash releases them at boot via
> `usb_serial_jtag_ll_phy_enable_pad(false)`. The ROM bootloader
> re-enables USB automatically when flashing.

---

## 4. Binary Serial Protocol <a name="binary-serial-protocol"></a>

MD uses `SERIALOUT_BINARY` mode (the default for Bluetooth output).

### Frame Structure

```
┌─────┬──────┬───────────────────┬─────┐
│ STX │ TAG  │   Payload (93B)   │ ETX │
│0x02 │0x5F  │                   │0x03 │
└─────┴──────┴───────────────────┴─────┘
  1B    1B         93 bytes        1B    = 95 bytes total
```

- **STX** = 0x02 (start of frame)
- **TAG** = 0x5F (95 decimal = `SERIALOUT_BINARY_TAG` = total frame size)
- **Payload** = 93 bytes of sensor data (TAG byte is first byte of payload)
- **ETX** = 0x03 (end of frame)
- **Endianness:** Little-endian (AVR native)
- **No byte-stuffing** — 0x02 and 0x03 CAN appear in payload data
- **No checksum/CRC** — frame integrity relies on STX+TAG+ETX framing

### Complete Byte Map

> **Authoritative reference:** See `SERIAL_PROTOCOL.md` in this repository
> for the full 500-line byte map with parsing pseudocode and scaling tables.

Quick summary of payload offsets (relative to TAG byte at offset 0):

| Offset | Size | Field | Scaling |
|---|---|---|---|
| 0 | 1 | TAG (0x5F) | — |
| 1-2 | u16 | RPM | raw |
| 3-4 | s16 | Boost | ÷100 → kPa |
| 5-6 | u16 | Throttle | raw % |
| 7-8 | s16 | Lambda | ÷100 |
| 9-10 | s16 | LMM | ÷100 → g/s |
| 11-12 | s16 | Case Temp | ÷100 → °C |
| 13-14 | s16 | Battery | ÷100 → V |
| 15-30 | 8×s16 | EGT[0..7] | raw °C |
| 31-36 | 3×s16 | VDO Pres[0..2] | ÷10 → kPa |
| 37-42 | 3×s16 | VDO Temp[0..2] | ÷10 → °C |
| 43-44 | s16 | Speed | ÷100 → km/h |
| 45 | u8 | Gear | raw |
| 46 | u8 | N75 Duty | raw % |
| 47-48 | s16 | Req Boost | ÷100 → kPa |
| 49-52 | u32 | EFR Speed | raw RPM |
| 53-54 | u16 | Knock | raw |
| 55-89 | 35B | VR6/Digifant data or zeros | build-dependent |
| 90-92 | 3B | reserved/padding | — |

---

## 5. OpenDash Receiver Architecture <a name="opendash-receiver-architecture"></a>

The receiver lives entirely in the `common/` shared component so any
OpenDash node could potentially receive MD data.

### Key Files

| File | Purpose |
|---|---|
| `common/include/opendash_uart.h` | Pin defines, data struct (`opendash_md_data_t`), debug flag |
| `common/src/opendash_uart.c` | UART driver, binary parser, connection state machine |
| `common/include/opendash_data_model.h` | Data point ID definitions (EGT1-EGT8, RPM, etc.) |
| `left/main/main.c` | Calls `opendash_uart_init()`, reads data, forwards to Center |

### Data Struct

```c
typedef struct {
    float    rpm, boost, throttle, lambda, lmm, bat_volt;
    float    egt[8];
    float    case_temp;
    float    vdo_pres1, vdo_pres2, vdo_pres3;
    float    vdo_temp1, vdo_temp2, vdo_temp3;
    float    speed;
    uint8_t  gear;
    float    n75_duty, req_boost, efr_speed, knock;
    uint32_t frame_count;
} opendash_md_data_t;
```

All values are **already scaled to real units** (the parser applies ÷100,
÷10 etc. during extraction). Consumers just read the struct.

### Parser Flow

```
uart_rx_task() loop:
  1. Scan byte-by-byte for STX (0x02)
  2. Read next byte — must be TAG (0x5F)
  3. Bulk-read 93 bytes (payload + ETX)
  4. Verify last byte = ETX (0x03)
  5. Call parse_binary_frame() → fills opendash_md_data_t
  6. main.c polls opendash_uart_get_data() every 50ms
  7. Pushes values to local LVGL UI
  8. Forwards to Center via ESP-NOW DATA_RESPONSE messages
```

### Connection State Machine

```
WAITING ──[first valid frame]──▶ RECEIVING ──[5s timeout]──▶ TIMEOUT
   ▲                                                            │
   └──────────[HC-05 reconnect attempt]─────────────────────────┘
```

### Debug Toggle

In `opendash_uart.h`:
```c
#define OPENDASH_UART_DEBUG  0   // 0 = quiet, 1 = verbose per-frame logging
```

---

## 6. What Was Changed in Each Project <a name="what-was-changed"></a>

### Changes in MultiDisplay Repository

| File | Change | Purpose |
|---|---|---|
| `SERIAL_PROTOCOL.md` | **Created** (513 lines) | Canonical binary protocol specification |
| `OPENDASH_INTEGRATION.md` | **Created** (this file) | Integration guide |
| `UART_SETUP.md` | Existing (outdated) | References ASCII format; binary protocol in SERIAL_PROTOCOL.md supersedes |
| `PROJECT_DOCUMENTATION.md` | Existing | General MD project docs |
| `CODE_ANNOTATIONS.md` | Existing | Source code annotation guide |

**No source code changes were made** to the MultiDisplay firmware itself.
The firmware's `SERIALOUT_BINARY` output is used as-is.

### Changes in OpenDash Repository

| File | Change | Purpose |
|---|---|---|
| `common/include/opendash_uart.h` | **Created** | UART pin config, data struct, debug flag |
| `common/src/opendash_uart.c` | **Created** | Full binary parser + UART driver |
| `common/include/opendash_data_model.h` | **Modified** | Added EGT5-EGT8 (0x0118-0x011B) |
| `common/CMakeLists.txt` | **Modified** | Added `esp_hal_usb` to PRIV_REQUIRES |
| `left/main/main.c` | **Modified** | UART init, data forwarding, UI update |
| `UART_CONNECTION.md` | **Rewritten** | Full protocol reference for OpenDash side |
| `FEATURES_AND_SENSORS.md` | **Created** | Sensor capability matrix |
| `BLUETOOTH_PAIRING.md` | **Created** | HC-05/HC-06 end-user guide |
| `TODO.md` | **Updated** | Checked off MD items, added new sections |
| `CHANGELOG.md` | **Updated** | Beta release entry with full MD history |

---

## 7. Current Status & Known Issues <a name="current-status"></a>

### Working (March 2026)

- Binary frame parsing: all fields extracted correctly
- All 8 EGT channels received and forwarded
- RPM, boost, throttle, lambda, LMM, battery voltage display
- VDO oil pressure and temperature
- Left gauge displays live data
- Center receives relayed data from Left via ESP-NOW
- ~80-85% frame success rate (good frames per second)

### Known Issues

| Issue | Impact | Cause | Status |
|---|---|---|---|
| **Bad ETX ~15-20%** | Frame loss | False STX+TAG (0x02 0x5F) in VR6 zero padding | Workaround: parser resyncs automatically |
| **HC-05 KEY/TX not wired** | No AT-mode reconnect | HC-05 KEY and RXD pins not connected | Modules pre-paired; auto-reconnect not needed |
| **VDO Pres2/3, Temp2/3** | Not forwarded | No data point IDs assigned yet | Add when sensors are installed |
| **EFR speed, knock** | Parsed but not forwarded | No data point IDs assigned | Add when sensors are installed |

### Performance

- Frame rate: ~80-85 valid frames/second (out of ~100 sent by MD)
- Latency: < 1 frame (10ms parse + 50ms main loop poll = ~60ms end-to-end)
- CPU impact: minimal with debug logging off

---

## 8. Future: OBDII/ELM327 Expansion <a name="future-obdii"></a>

The MultiDisplay firmware roadmap includes adding ELM327/OBDII support.
When that arrives:

1. **MD firmware** will read OBD2 PIDs and include them in the serial stream
2. **Binary frame** may expand beyond 95 bytes (new TAG value) or use
   additional frame types
3. **OpenDash parser** will need to handle the new frame format — the
   TAG byte already serves as a frame-size indicator, so a new TAG value
   signals a different payload layout
4. OpenDash already has OBD2 data point IDs defined (coolant temp, intake
   temp, engine load, timing advance, fuel level, etc.)

Alternatively, Center can read OBD2 directly via its onboard CAN bus,
bypassing MD entirely.

---

## 9. File Reference <a name="file-reference"></a>

### MultiDisplay Source Files (read to build this integration)

| File | What it contains |
|---|---|
| `MultidisplayController.cpp` L1289-1570 | `serialSend()` — the binary frame serialization |
| `MultidisplayDefines.h` | `SERIALOUT_BINARY_TAG`, `MAX_ATTACHED_TYPE_K`, build flags |
| `SensorData.h` | `SensorData` struct with all sensor fields |
| `util.cpp` | `float2fixedintb100()` — ×100 float-to-int conversion |

### OpenDash Files

| File | What it contains |
|---|---|
| `common/src/opendash_uart.c` | Binary parser, UART driver, state machine |
| `common/include/opendash_uart.h` | Pin defines, `opendash_md_data_t` struct |
| `common/include/opendash_data_model.h` | All data point IDs |
| `left/main/main.c` | UART init, `forward_md_data_to_center()`, UI updates |
| `UART_CONNECTION.md` | OpenDash-side protocol reference |
| `SERIAL_PROTOCOL.md` (MD repo) | Canonical protocol spec |

---

*This document was created during the OpenDash Beta release (March 2026)
to ensure any developer or AI assistant can understand the full
MultiDisplay ↔ OpenDash data pipeline without reading source code.*
