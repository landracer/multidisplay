# MultiDisplay OBD-II Implementation Guide

> **Comprehensive reference for the OBD-II via ELM327 subsystem.**
> Source of truth: `obd2_implementation/` directory and `MultidisplayDefinesOBD2.h`.
> This document covers architecture, configuration, protocols, serial frame format,
> standards compliance, troubleshooting, and future roadmap.
>
> **Standards Compliance:** SAE J1979-2, ISO 15031-5, ISO 9141-2

---

## Table of Contents

1. [Overview](#1-overview)
2. [Standards Reference](#2-standards-reference)
3. [Architecture](#3-architecture)
4. [Hardware Wiring](#4-hardware-wiring)
5. [File Layout](#5-file-layout)
6. [ELM327 AT Protocol](#6-elm327-at-protocol)
7. [PID Configuration](#7-pid-configuration)
8. [Tier-Based Poll Scheduler](#8-tier-based-poll-scheduler)
9. [PID Decode Formulas (SAE J1979)](#9-pid-decode-formulas)
10. [Binary Serial Frame Format](#10-binary-serial-frame-format)
11. [Standalone OBD2 Frame (TAG 0x4F)](#11-standalone-obd2-frame)
12. [Integration with MultidisplayController](#12-integration-with-multidisplaycontroller)
13. [Bluetooth Mirroring (Serial2)](#13-bluetooth-mirroring)
14. [Build & Flash](#14-build--flash)
15. [Troubleshooting](#15-troubleshooting)
16. [Future Improvements](#16-future-improvements)

---

## 1. Overview

The OBD-II subsystem adds standardized vehicle diagnostics to MultiDisplay via an
**ELM327DSL** interpreter IC wired to UART1 on the ATmega2560. It polls the vehicle
ECU for Mode 01 PIDs (live data) and Mode 03 DTCs (diagnostic trouble codes),
caches decoded values, and embeds a 35-byte summary into the standard 95-byte
binary serial frame at bytes 58–92.

**Key capabilities:**
- Up to 32 simultaneously tracked PIDs with 3-tier priority scheduling
- 40+ SAE J1979 decode formulas stored in PROGMEM
- DTC read (Service 03) and clear (Service 04)
- Automatic supported-PID discovery via PID 0x00
- Zero-overhead when disabled (`#undef OBD2_ELM327`)
- Full data mirrored to Bluetooth Serial2 via `MD_SERIAL_WRITE` macro
- Standalone OBD2 frames (TAG 0x4F) with per-PID detail

---

## 2. Standards Reference

All PID numbers, decode formulas, scaling factors, and DTC formats in this
implementation conform to the following industry standards:

### SAE J1979-2 (formerly SAE J1979)
- **Title:** E/E Diagnostic Test Modes — OBD II
- **Governs:** Mode 01 (live data PIDs), Mode 02 (freeze frame), Mode 03
  (stored DTCs), Mode 04 (clear DTCs), Mode 09 (vehicle information)
- **Our usage:** All Mode 01 PID request/response parsing (service byte 0x01,
  response prefix 0x41), Mode 03/04 for DTC management
- **PID formulas:** Section 6 — Standard PID Data Definitions. Every decode
  formula in `OBD2Data.cpp` is taken directly from the J1979 formula tables.

### ISO 15031-5
- **Title:** Road vehicles — Communication between vehicle and external
  equipment for emissions-related diagnostics — Part 5: Emissions-related
  diagnostic services
- **Governs:** Identical PID definitions to SAE J1979 (ISO adoption of SAE
  standard). The formula table, PID numbering, and data byte assignments
  are identical.
- **DTC Format:** 2-byte DTC encoding: first nibble = category (P/C/B/U),
  remaining 12 bits = fault code (e.g., 0x0300 → "P0300").

### ISO 9141-2
- **Title:** Road vehicles — Diagnostic systems — Part 2: CARB requirements
  for interchange of digital information (K-line)
- **Governs:** The physical/data-link layer used when `OBD2_ELM_PROTOCOL=3`.
  5-baud initialization, slow-init handshake, 10.4 kbaud data rate.
- **Our usage:** The ELM327 handles ISO 9141-2 transparently. We configure
  it via `ATSP3` and the ELM327 manages all init/timing.

### ELM327DSL Datasheet
- **Source:** elmelectronics.com — ELM327DSL.pdf
- **Governs:** AT command syntax, response format, error string handling,
  baud rate configuration, protocol selection.

> **Compliance note:** When adding new PIDs, ALWAYS use the SAE J1979-2 /
> ISO 15031-5 formula tables. Do NOT invent custom scaling. If a PID is not
> in the SAE standard, it is a manufacturer-specific PID and must be clearly
> documented as non-standard.

---

## 3. Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    MultidisplayController                           │
│   myconstructor()  →  obd2.begin()                                 │
│   mainLoop()       →  obd2.poll()                                  │
│   serialSend()     →  obd2.serialSendOBD2()                        │
│                       + 35-byte OBD2 summary in SERIALOUT_BINARY    │
└──────────┬──────────────────────────────────────────────────────────┘
           │
           ▼
┌───────────────────────────────────────────────────────────────────┐
│                     OBD2Manager (obd2_implementation/)             │
│   ┌──────────┐   ┌──────────┐   ┌──────────────────────┐         │
│   │ ELM327   │   │ OBD2Data │   │ Poll Scheduler       │         │
│   │ .h / .cpp│   │ .h / .cpp│   │ (in OBD2Manager.cpp) │         │
│   │          │   │          │   │                      │         │
│   │ AT cmds  │   │ 40+ PID  │   │ 3-tier round-robin  │         │
│   │ PID req  │   │ formulas │   │ Tier1: every cycle   │         │
│   │ DTC r/w  │   │ in PROGMEM│   │ Tier2: every 2nd    │         │
│   │ Supported│   │ Value    │   │ Tier3: every 5th     │         │
│   │ PID query│   │ cache    │   │                      │         │
│   └────┬─────┘   └────┬─────┘   └──────────────────────┘         │
│        │              │                                            │
│        ▼              ▼                                            │
│   Serial1 (UART1)   RAM cache                                     │
│   38400 baud         [OBD2_MAX_TRACKED_PIDS]                      │
└────────┬──────────────────────────────────────────────────────────┘
         │
         ▼
   ┌─────────────┐        ┌─────────────────────────────────────┐
   │ ELM327DSL   │ ◄──── │ Vehicle OBD-II Port (DLC connector) │
   │   IC chip   │  K-line│ ISO 9141-2 / CAN (depending on     │
   │ on MD board │  or CAN│ ATSP protocol setting)              │
   └─────────────┘        └─────────────────────────────────────┘
```

### Data Flow (per main loop iteration)

1. `MultidisplayController::mainLoop()` calls `obd2.poll()`
2. `OBD2Manager::poll()` checks if the next PID in the schedule is due
3. If due: sends `"01XX\r"` to ELM327 via Serial1
4. ELM327 returns `"41 XX AA BB..."` or error string
5. `ELM327::requestPID()` parses the hex response into `OBD2PIDResponse`
6. `OBD2Data::decodePID()` applies the SAE J1979 formula → float value
7. `OBD2Data::updateCache()` stores the value with timestamp
8. When `serialSend()` runs:
   - The 35-byte OBD2 summary is embedded at bytes 58–92 of the binary frame
   - `serialSendOBD2()` optionally sends the standalone 0x4F frame

---

## 4. Hardware Wiring

### ELM327DSL ↔ ATmega2560

| Signal      | ELM327DSL Pin | ATmega2560 Pin | Notes                        |
|-------------|---------------|----------------|------------------------------|
| TX (out)    | Pin 4 (TXD)   | RX1 (Pin 19)   | ELM327 transmit → ATmega receive |
| RX (in)     | Pin 5 (RXD)   | TX1 (Pin 18)   | ATmega transmit → ELM327 receive |
| VCC         | Pin 2          | 5V             | 5V supply (3.3V–5V tolerant) |
| GND         | Pin 1          | GND            | Common ground                |

### ELM327DSL ↔ Vehicle OBD-II Connector (DLC)

| Signal | ELM327DSL Pin | OBD-II DLC Pin | Protocol         |
|--------|---------------|----------------|------------------|
| K-line | Pin 12 (K)    | Pin 7          | ISO 9141-2       |
| CAN-H  | Pin 6         | Pin 6          | ISO 15765-4 CAN  |
| CAN-L  | Pin 14        | Pin 14         | ISO 15765-4 CAN  |
| Battery| Pin 16        | Pin 16         | +12V (always on) |
| Ground | Pin 4/5       | Pin 4/5        | Chassis/signal GND |

> **Note:** Only one physical layer is active at a time — controlled by `ATSP`.
> For K-line (ATSP3), only the K-line pin is needed. For CAN (ATSP6), only
> CAN-H/CAN-L are needed.

---

## 5. File Layout

```
multidisplay/
├── MultidisplayDefinesOBD2.h          ← ELM327 config, PID enable/disable, tier defines
├── MultidisplayDefines.h              ← ECU=OBD2_ELM327_ECU, BT enable, MD_SERIAL_WRITE macro
├── MultidisplayController.h           ← #include guard for OBD2Manager.h
├── MultidisplayController.cpp         ← begin/poll/serialSend integration + 35-byte summary
└── obd2_implementation/
    ├── ELM327.h                       ← ELM327 AT command driver (class declaration)
    ├── ELM327.cpp                     ← AT init, PID request, DTC read/clear, response parsing
    ├── OBD2Data.h                     ← PID decoder + cache (class declaration, PID #defines)
    ├── OBD2Data.cpp                   ← 40+ SAE J1979 decode formulas in PROGMEM, cache ops
    ├── OBD2Manager.h                  ← Top-level controller (class declaration, poll schedule)
    └── OBD2Manager.cpp                ← Scheduler logic, serialSendOBD2(), global `obd2` instance
```

---

## 6. ELM327 AT Protocol

### Initialization Sequence

When `obd2.begin()` is called, the following AT commands are sent:

| Step | Command  | Purpose                              | Expected Response |
|------|----------|--------------------------------------|-------------------|
| 1    | `ATZ\r`  | Reset ELM327 to defaults             | `"ELM327 v..."`   |
| 2    | `ATE0\r` | Disable echo                         | `"OK"`             |
| 3    | `ATL0\r` | Disable linefeeds                    | `"OK"`             |
| 4    | `ATS0\r` | Disable spaces in responses          | `"OK"`             |
| 5    | `ATSP3\r`| Set protocol to ISO 9141-2           | `"OK"`             |

> **ATSP value** is configured by `OBD2_ELM_PROTOCOL` in `MultidisplayDefinesOBD2.h`.
> Default is `3` (ISO 9141-2). Change to `0` for automatic detection or `6` for
> CAN (ISO 15765-4, 11-bit ID, 500 kbaud).

### PID Request / Response

| Action  | Format Sent                   | Response Format                      |
|---------|-------------------------------|--------------------------------------|
| Mode 01 | `"010C\r"` (RPM example)     | `"410C1AF8"` or `"410C 1A F8"` (with spaces) |
| Mode 03 | `"03\r"` (read DTCs)         | `"430300430201"` (DTC list)          |
| Mode 04 | `"04\r"` (clear DTCs)        | `"44"` or `"OK"`                     |

### Error Strings

The ELM327 returns ASCII error strings on failure:

| Error String      | Meaning                         | ELM327Status Code |
|-------------------|---------------------------------|-------------------|
| `"NO DATA"`       | ECU did not respond to PID      | `ELM_NO_DATA`     |
| `"ERROR"`         | General communication error     | `ELM_ERROR`       |
| `"UNABLE TO CONNECT"` | Cannot reach ECU           | `ELM_ERROR`       |
| `"BUS INIT: ...OK"` | First-request init (normal)  | `ELM_BUS_INIT`    |
| *(timeout)*       | No response within timeout      | `ELM_TIMEOUT`     |

---

## 7. PID Configuration

### Enabling / Disabling PIDs

Edit `MultidisplayDefinesOBD2.h` to enable or disable individual PIDs:

```cpp
// Enable:
#define OBD2_ENABLE_PID_RPM             // 0x0C Engine RPM

// Disable (comment out):
// #define OBD2_ENABLE_PID_ETHANOL_PCT  // 0x52 Ethanol fuel %
```

### Available PIDs (Mode 01)

| PID  | Hex   | Define                         | Tier | SAE J1979 Name                |
|------|-------|--------------------------------|------|-------------------------------|
| 0x04 | `04`  | `OBD2_ENABLE_PID_ENGINE_LOAD`  | 1    | Calculated engine load        |
| 0x05 | `05`  | `OBD2_ENABLE_PID_COOLANT_TEMP` | 1    | Engine coolant temperature    |
| 0x06 | `06`  | `OBD2_ENABLE_PID_STFT_B1`      | 2    | Short term fuel trim bank 1   |
| 0x07 | `07`  | `OBD2_ENABLE_PID_LTFT_B1`      | 2    | Long term fuel trim bank 1    |
| 0x0A | `0A`  | `OBD2_ENABLE_PID_FUEL_PRESSURE`| 2    | Fuel pressure (gauge)         |
| 0x0B | `0B`  | `OBD2_ENABLE_PID_INTAKE_MAP`   | 1    | Intake manifold pressure      |
| 0x0C | `0C`  | `OBD2_ENABLE_PID_RPM`          | 1    | Engine RPM                    |
| 0x0D | `0D`  | `OBD2_ENABLE_PID_SPEED`        | 1    | Vehicle speed                 |
| 0x0E | `0E`  | `OBD2_ENABLE_PID_TIMING_ADV`   | 2    | Timing advance                |
| 0x0F | `0F`  | `OBD2_ENABLE_PID_INTAKE_TEMP`  | 2    | Intake air temperature        |
| 0x10 | `10`  | `OBD2_ENABLE_PID_MAF_RATE`     | 2    | Mass air flow rate            |
| 0x11 | `11`  | `OBD2_ENABLE_PID_THROTTLE_POS` | 1    | Throttle position             |
| 0x1F | `1F`  | `OBD2_ENABLE_PID_RUNTIME`      | 3    | Run time since start          |
| 0x2F | `2F`  | `OBD2_ENABLE_PID_FUEL_LEVEL`   | 3    | Fuel tank level input         |
| 0x33 | `33`  | `OBD2_ENABLE_PID_BARO_PRESSURE`| 2    | Barometric pressure           |
| 0x42 | `42`  | `OBD2_ENABLE_PID_CTRL_VOLTAGE` | 3    | Control module voltage        |
| 0x46 | `46`  | `OBD2_ENABLE_PID_AMBIENT_TEMP` | 3    | Ambient air temperature       |
| 0x5C | `5C`  | `OBD2_ENABLE_PID_OIL_TEMP`     | 3    | Engine oil temperature        |

### Disabled by Default (uncomment to enable)

| PID  | Hex   | Define                         | SAE J1979 Name                |
|------|-------|--------------------------------|-------------------------------|
| 0x52 | `52`  | `OBD2_ENABLE_PID_ETHANOL_PCT`  | Ethanol fuel percentage       |
| 0x70 | `70`  | `OBD2_ENABLE_PID_BOOST_PRESS`  | Boost pressure control        |
| 0x74 | `74`  | `OBD2_ENABLE_PID_TURBO_RPM`    | Turbocharger RPM              |
| 0x78 | `78`  | `OBD2_ENABLE_PID_EGT_B1`       | Exhaust gas temp bank 1       |
| 0xA6 | `A6`  | `OBD2_ENABLE_PID_ODOMETER`     | Odometer                      |

---

## 8. Tier-Based Poll Scheduler

The scheduler uses a round-robin approach with tier-based frequency multipliers:

| Tier | Frequency         | Use Case              | Default PIDs                    |
|------|-------------------|-----------------------|---------------------------------|
| 1    | Every cycle       | Fast-changing values  | RPM, Speed, Coolant, Load, MAP, Throttle |
| 2    | Every 2nd cycle   | Moderate-change values| IAT, MAF, Timing, STFT, LTFT, Fuel Press, Baro |
| 3    | Every 5th cycle   | Slow-changing values  | Runtime, Fuel Level, Ambient Temp, Oil Temp, Ctrl Voltage |

### How It Works

1. `buildPollSchedule()` iterates all `OBD2_ENABLE_PID_xxx` defines and
   creates an `OBD2PollEntry` for each enabled PID
2. Each entry stores: `pid`, `tier`, `lastPoll` timestamp, `supported` flag
3. `poll()` walks the round-robin index, checking if enough time has passed
   since the last poll for that PID's tier
4. Tier intervals are calculated from the main loop execution rate:
   - Tier 1: immediate (every poll call)
   - Tier 2: skip 1 cycle
   - Tier 3: skip 4 cycles
5. After each successful poll, the decoded value is cached in `OBD2Data`

### Timing Considerations

- **ELM327 response time:** 50–200 ms per PID (ISO 9141-2 is slow)
- **Effective poll rate:** With 6 Tier-1 PIDs, each PID updates ~1.5–3 Hz
- **Main loop impact:** `poll()` blocks for one PID response per call.
  The rest of the main loop (sensor reads, display, etc.) runs between polls.

---

## 9. PID Decode Formulas (SAE J1979)

All formulas below are from **SAE J1979-2 Section 6** / **ISO 15031-5 Annex B**.
The decode functions are stored in PROGMEM in `OBD2Data.cpp`.

| PID  | Formula                        | Unit    | Range              |
|------|--------------------------------|---------|--------------------|
| 0x04 | `A × 100 / 255`               | %       | 0 – 100            |
| 0x05 | `A − 40`                      | °C      | −40 – 215          |
| 0x06 | `(A − 128) × 100 / 128`       | %       | −100 – 99.2        |
| 0x07 | `(A − 128) × 100 / 128`       | %       | −100 – 99.2        |
| 0x0A | `A × 3`                       | kPa     | 0 – 765            |
| 0x0B | `A`                            | kPa     | 0 – 255            |
| 0x0C | `((A × 256) + B) / 4`         | RPM     | 0 – 16383.75       |
| 0x0D | `A`                            | km/h    | 0 – 255            |
| 0x0E | `(A / 2) − 64`                | ° BTDC  | −64 – 63.5         |
| 0x0F | `A − 40`                      | °C      | −40 – 215          |
| 0x10 | `((A × 256) + B) / 100`       | g/s     | 0 – 655.35         |
| 0x11 | `A × 100 / 255`               | %       | 0 – 100            |
| 0x1F | `(A × 256) + B`               | seconds | 0 – 65535          |
| 0x2F | `A × 100 / 255`               | %       | 0 – 100            |
| 0x33 | `A`                            | kPa     | 0 – 255            |
| 0x42 | `((A × 256) + B) / 1000`      | V       | 0 – 65.535         |
| 0x46 | `A − 40`                      | °C      | −40 – 215          |
| 0x5C | `A − 40`                      | °C      | −40 – 215          |

> **Important:** The `A`, `B`, `C`, `D` notation refers to data bytes in the
> OBD-II response (after the service + PID header bytes). For example, for
> PID 0x0C (RPM), the response is `"41 0C AA BB"` where A=AA and B=BB.

---

## 10. Binary Serial Frame Format

### OBD2 Summary in Main Frame (Bytes 58–92)

The OBD2 data occupies the **variant data** region of the standard 95-byte
binary frame. This region was previously used for Digifant K-line data or
filled with zeros (VR6 build).

| Offset | Size | Type       | Scaling  | Content                    |
|--------|------|------------|----------|----------------------------|
| 58     | 1    | `uint8_t`  | bitfield | OBD2 flags (see below)     |
| 59     | 2    | `int16_t`  | ×100     | OBD2 RPM                   |
| 61     | 2    | `int16_t`  | ×100     | OBD2 Vehicle speed (km/h)  |
| 63     | 2    | `int16_t`  | ×100     | OBD2 Coolant temp (°C)     |
| 65     | 2    | `int16_t`  | ×100     | OBD2 Engine load (%)       |
| 67     | 2    | `int16_t`  | ×100     | OBD2 Intake MAP (kPa)      |
| 69     | 2    | `int16_t`  | ×100     | OBD2 Throttle position (%) |
| 71     | 2    | `int16_t`  | ×100     | OBD2 Intake air temp (°C)  |
| 73     | 2    | `int16_t`  | ×100     | OBD2 MAF rate (g/s)        |
| 75     | 2    | `int16_t`  | ×100     | OBD2 Timing advance (°)    |
| 77     | 2    | `int16_t`  | ×100     | OBD2 STFT bank 1 (%)       |
| 79     | 2    | `int16_t`  | ×100     | OBD2 LTFT bank 1 (%)       |
| 81     | 2    | `int16_t`  | ×100     | OBD2 Fuel pressure (kPa)   |
| 83     | 2    | `int16_t`  | ×100     | OBD2 Baro pressure (kPa)   |
| 85     | 2    | `int16_t`  | ×100     | OBD2 Oil temp (°C)         |
| 87     | 2    | `int16_t`  | ×100     | OBD2 Control voltage (V)   |
| 89     | 2    | `int16_t`  | ×100     | OBD2 Fuel level (%)        |
| 91     | 2    | —          | —        | Reserved (zero-padded)     |

**Total OBD2 region: 35 bytes** (1 flag + 16 PIDs × 2 bytes + 2 reserved)

### OBD2 Flags Byte (Offset 58)

| Bit | Meaning                                      |
|-----|----------------------------------------------|
| 0   | ELM327 initialized and ready (1 = yes)       |
| 1   | DTCs present (1 = at least one stored DTC)    |
| 2–7 | Reserved (zero)                              |

### Value Encoding

All PID values are encoded as **signed 16-bit integers** with **×100 fixed-point**
scaling (same convention as the MD core fields like boost and lambda):

```
Encoded value = (int16_t)(float_value × 100.0)
Decoded value = (float)encoded_value / 100.0
```

**Byte order:** Little-endian (AVR native). Low byte first, high byte second.

### Receiver Pseudocode

```c
uint8_t flags  = payload[58];
int16_t rpm    = payload[59] | (payload[60] << 8);   // LE
float   obd_rpm = (float)rpm / 100.0f;

bool elm_ready = (flags & 0x01) != 0;
bool has_dtcs  = (flags & 0x02) != 0;
bool obd_present = (flags != 0);
```

---

## 11. Standalone OBD2 Frame (TAG 0x4F)

In addition to the 35-byte summary embedded in the main frame,
`serialSendOBD2()` sends a separate per-PID detail frame:

```
┌─────┬──────┬─────────────────────────────────────┬─────┐
│ STX │ TAG  │              payload                 │ ETX │
│0x02 │ 0x4F │ time[4] count[1] {pid val_h val_l}×N│0x03 │
└─────┴──────┴─────────────────────────────────────┴─────┘
```

| Field     | Offset | Size  | Description                              |
|-----------|--------|-------|------------------------------------------|
| STX       | 0      | 1     | 0x02                                     |
| TAG       | 1      | 1     | 0x4F (79 decimal) — OBD2 frame marker    |
| time      | 2      | 4     | `millis()` uint32_t LE                   |
| pid_count | 6      | 1     | Number of PID entries following           |
| pid[N]    | 7+3*i  | 1     | PID number                               |
| value_hi  | 8+3*i  | 1     | Value high byte (×100 fixed-point)       |
| value_lo  | 9+3*i  | 1     | Value low byte (×100 fixed-point)        |
| ETX       | last   | 1     | 0x03                                     |

> **Note:** The standalone frame is variable-length. Receivers that expect
> fixed 95-byte frames (TAG=0x5F) should gracefully skip TAG=0x4F frames.
> OpenDash does this by draining bytes until ETX when it sees TAG=0x4F.

---

## 12. Integration with MultidisplayController

### Initialization (`myconstructor()` / `setup()`)

```cpp
#ifdef OBD2_ELM327
    if (!obd2.begin()) {
        // ELM327 not responding — OBD2 disabled for this session
    }
#endif
```

### Main Loop (`mainLoop()`)

```cpp
#ifdef OBD2_ELM327
    obd2.poll();    // Non-blocking if no PID is due; blocks for one response if due
#endif
```

### Serial Output (`serialSend()`)

In the `SERIALOUT_BINARY` case, after the standard 58 bytes:

```cpp
#ifdef OBD2_ELM327
    // 35-byte OBD2 summary at bytes 58-92
    uint8_t obd2_flags = 0;
    if (obd2.isReady()) obd2_flags |= 0x01;
    if (obd2.getDTCCount() > 0) obd2_flags |= 0x02;
    MD_SERIAL_WRITE(obd2_flags);

    // 16 PID values as int16 ×100 LE
    int16_t v;
    v = (int16_t)(obd2.getValue(PID_RPM) * 100.0f);
    MD_SERIAL_WRITE(lowByte(v)); MD_SERIAL_WRITE(highByte(v));
    // ... (repeat for each PID)

    // 2 reserved bytes
    MD_SERIAL_WRITE(0); MD_SERIAL_WRITE(0);
#else
    // Non-OBD2 builds: 35 zeros or Digifant K-line data
#endif
```

After the main frame, the standalone frame is also sent:
```cpp
#ifdef OBD2_ELM327
    obd2.serialSendOBD2();   // Sends the TAG=0x4F frame
#endif
```

---

## 13. Bluetooth Mirroring (Serial2)

When `BLUETOOTH_ON_SERIAL2` is defined, the `MD_SERIAL_WRITE` macro writes
every byte to **both** `Serial` (USB) and `Serial2` (HC-06 Bluetooth):

```cpp
#ifdef BLUETOOTH_ON_SERIAL2
  #define MD_SERIAL_WRITE(b) do { Serial.write(b); Serial2.write(b); } while(0)
#else
  #define MD_SERIAL_WRITE(b) Serial.write(b)
#endif
```

This ensures OpenDash receives the complete 95-byte frame (including OBD2 data)
over the Bluetooth SPP link. Previously, only STX and ETX were sent on Serial2.

---

## 14. Build & Flash

### Prerequisites

- [PlatformIO](https://platformio.org/) installed
- ATmega2560-based board (Seeeduino Mega or Arduino Mega 2560)

### Build

```bash
cd /path/to/multidisplay
pio run
```

### Flash

```bash
pio run --target upload
```

### Monitor Serial Output

```bash
pio device monitor --baud 115200
```

### Build Configuration

The `platformio.ini` in the multidisplay root configures:

| Setting          | Value                  |
|------------------|------------------------|
| Platform         | `atmelavr`             |
| Board            | `megaatmega2560`       |
| Framework        | `arduino`              |
| Upload baud      | `115200`               |
| Monitor baud     | `115200`               |
| Extra flags      | `-Wno-narrowing`       |

### Resource Usage (with OBD2 enabled)

| Resource | Usage         | Total     | Percent |
|----------|---------------|-----------|---------|
| RAM      | ~2634 bytes   | 8192      | 32.2%   |
| Flash    | ~39234 bytes  | 253952    | 15.4%   |

## 15. Troubleshooting

### ELM327 Not Responding at Startup

| Symptom                              | Cause                         | Fix                            |
|--------------------------------------|-------------------------------|--------------------------------|
| `obd2.begin()` returns `false`       | Wrong baud rate               | Verify `OBD2_ELM327_BAUD` matches ELM327 config |
| No "ELM327" banner after ATZ         | Wiring issue (TX1↔RX, RX1↔TX)| Check TX/RX crossover          |
| ELM327 responds but `UNABLE TO CONNECT` | Vehicle not running / no ECU | Start engine, check DLC wiring |
| `BUS INIT: ...ERROR`                | ISO 9141-2 init failed        | Try `ATSP0` (auto-detect) instead of `ATSP3` |

### No OBD2 Data in Serial Frame

| Symptom                              | Cause                         | Fix                            |
|--------------------------------------|-------------------------------|--------------------------------|
| Bytes 58–92 all zero                 | OBD2 build not enabled        | Verify `#define OBD2_ELM327` uncommented |
| Flags byte = 0x00                    | ELM327 init failed            | Check ELM327 power and wiring  |
| Flags byte = 0x01 but PIDs all zero  | No PIDs enabled or ECU unsupported | Check `OBD2_ENABLE_PID_xxx` defines |

### Bluetooth Not Receiving OBD2 Data

| Symptom                              | Cause                         | Fix                            |
|--------------------------------------|-------------------------------|--------------------------------|
| 2-byte frames on BT (STX + ETX only)| `BLUETOOTH_ON_SERIAL2` not defined | Enable in `MultidisplayDefines.h` |
| Frame size < 95 bytes on BT         | Partial write                 | Check `MD_SERIAL_WRITE` macro  |

### PID Returns NO DATA

| Symptom                              | Cause                         | Fix                            |
|--------------------------------------|-------------------------------|--------------------------------|
| Specific PID returns `ELM_NO_DATA`  | ECU doesn't support that PID  | Auto-disabled after 3 failures; check PID 0x00 bitmap |
| All PIDs return `ELM_NO_DATA`       | Wrong OBD-II protocol         | Try `OBD2_ELM_PROTOCOL 0` (auto) |

### Baud Rate Issues

| Symptom                              | Cause                         | Fix                            |
|--------------------------------------|-------------------------------|--------------------------------|
| `[ELM] baud stays at 9600`          | AT BRD handshake failed       | Check BRD divisor: hh=round(4M/baud). 38400→0x68, NOT 0x06. See ELM327DSL datasheet §"Using Higher RS232 Baud Rates". |
| `[ELM] BRD no ID at new baud`       | Wrong divisor or timing       | Verify: ELM sends "OK\r" (no `>`), then ID at new baud after BRT wait. Host must switch baud before reading ID. |
| `[ELM] BRD not accepted`            | Chip doesn't support AT BRD   | ELM327 v1.0/v1.1 — stay at 9600 |
| `[ELM] PP 0C not supported`         | Chip doesn't support PP cmds  | Some v1.4 chips lack PP. Use AT BRD only. |

---

## 16. Future Improvements

- [x] **Per-PID NO_DATA auto-disable:** PIDs returning NO DATA are
      automatically disabled after 3 consecutive failures
- [ ] **Supported PID auto-discovery:** Query PID 0x00/0x20/0x40/0x60 at
      startup and auto-disable unsupported PIDs
- [ ] **CAN protocol support:** Test with ATSP6 (ISO 15765-4 CAN) for
      modern vehicles — faster response times (~5 ms vs ~200 ms)
- [ ] **Mode 02 freeze frame:** Read freeze frame data for advanced diagnostics
- [ ] **Mode 09 VIN:** Read vehicle identification number at startup
- [ ] **DTC display on LCD:** Show DTC count and codes on the MultiDisplay screen
- [ ] **DTC clear button:** Physical button or serial command to clear DTCs
- [x] **Aggressive adaptive timing:** AT AT2 + ATST 32 (200ms) for fast polling
- [ ] **Adaptive tier timing:** Auto-adjust poll intervals based on ECU response time
- [ ] **Additional PIDs:** Catalyst temp, EGR %, turbo boost, exhaust pressure
- [ ] **Manufacturer-specific PIDs:** Mode 22 enhanced diagnostics (non-standard)
- [ ] **ELM327 v2.x features:** Long messages, CAN filter/mask configuration
- [x] **ELM327 reconnect:** Auto-reinitialize every 30s if communication fails
- [ ] **Watchdog for ELM327:** Detect and recover from mid-session stalls
- [ ] **Configurable OBD2 summary fields:** Allow remapping which 16 PIDs appear
      in the 35-byte summary region

---

*Last updated: July 2026*
*Standards: SAE J1979-2, ISO 15031-5, ISO 9141-2, ELM327DSL Datasheet*
