# Changelog

All notable changes to the MultiDisplay firmware will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

## [v2.2.0-rAtTrax] - 2026-03-XX

### Added — rAtTrax Shield Peripherals
- **DS3231 Real-Time Clock** (`RTCDriver.h/.cpp`) — I2C driver (addr 0x68):
  - BCD register read/write for seconds through year
  - `readTime()` called once per second from mainLoop
  - `setFromGPS()` for UTC time sync from GPS NMEA data
  - `formatTimestamp()` / `formatTime()` / `formatDate()` for human-readable output
  - `RTC_ENABLE` feature define in MultidisplayDefines.h

- **I2C SD Card Logger** (`SDLogger.h/.cpp`) — Qwiic OpenLog driver (addr 0x2A):
  - Auto-creates sequential log files `MDnnnn.CSV` per power cycle
  - EEPROM-persisted file sequence counter (address 210-211)
  - CSV header: timestamp,rpm,boost,egt1-8,lambda,throttle,speed,gear,efr_speed,bat
  - RTC-timestamped records via `writeTimestamped()`
  - Chunked I2C writes (30-byte Wire library limit on AVR)
  - Periodic sync/flush every 5s (`SDLOGGER_SYNC_INTERVAL`)
  - GPS lat/lon/speed appended when fix is available
  - `SDLOGGER_ENABLE` feature define in MultidisplayDefines.h

- **GY-NEO6MV2 GPS Reader** (`GPSReader.h/.cpp`) — on Serial3 (RX3=14, TX3=15):
  - Non-blocking NMEA sentence parser (processes Serial3 buffer each loop)
  - $GPRMC parsing: lat, lon, speed (knots + km/h), course, UTC time/date
  - $GPGGA parsing: satellite count
  - NMEA checksum verification
  - `hasFix()` with 3-second staleness check
  - Position stored as degrees × 10^6 (integer fixed-point)
  - `serialWriteGPS()` packs 12 bytes (lat4 + lon4 + speed2 + course2)
  - `GPS_ENABLE` feature define in MultidisplayDefines.h

### Added — Documentation & Build System
- `RTCDriver.h/.cpp` — DS3231 I2C RTC driver (2 new files)
- `SDLogger.h/.cpp` — I2C SD card logger driver (2 new files)
- `GPSReader.h/.cpp` — NMEA GPS parser for Serial3 (2 new files)
- Updated `CHANGELOG.md` with comprehensive change history

### Changed — Project Infrastructure
- **Migrated from Eclipse to PlatformIO** — `platformio.ini` added for
  ATmega2560 build/upload/monitor via `pio run` commands
- **OpenDash integration** — Binary serial frames flow to OpenDash via
  Bluetooth SPP (Serial2) and USB (Serial), protocol documented in
  `SERIAL_PROTOCOL.md` and `OPENDASH_INTEGRATION.md`
- **rAtTrax ecosystem unification** — MultiDisplay, OpenDash, and rAtTrax BMS
  now share a common serial frame protocol and Bluetooth transport layer
    (opendash will be with the realm shortly, hang tight, it'll be awesome!!!)
### Changed — Author Credits
- Added **uknowmelast** and **Axiom (rAtTrax project)** as contributors to
  all modified source files (26 files total)
- Original authors Stephan Martin and Dominik Gummel preserved in all headers
- New OBD2/RTC/SD/GPS files carry `rAtTrax / MultiDisplay Project` copyright

### Changed — MultidisplayController Integration
- `myconstructor()`: initializes RTC → SD Logger → GPS after Wire.begin()
- `mainLoop()`: GPS polled every cycle, RTC read every 1s, SD record per
  serial frame, SD sync every 5s
- Debug heartbeat extended: GPS fix/sat, RTC time, SD status

### Changed — UART Allocation (ATmega2560)
| UART    | Use              | Baud   |
|---------|------------------|--------|
| Serial  | USB / PC         | 115200 |
| Serial1 | ELM327 OBD-II    | 38400  |
| Serial2 | Bluetooth HC-06  | 115200 |
| Serial3 | GPS GY-NEO6MV2   | 9600   |

### Changed — I2C Bus Devices
| Address | Device          | Use                     |
|---------|-----------------|-------------------------|
| 0x20/27 | PCF8574P        | IO Expander (existing)  |
| 0x68    | DS3231          | Real-Time Clock (new)   |
| 0x2A    | Qwiic OpenLog   | SD Card Logger (new)    |

### Build Metrics (with OBD2 + RTC + SD + GPS)
- RAM: 2953 / 8192 bytes (36.0%)
- Flash: 45330 / 253952 bytes (17.8%)

---

## [v2.1.0-obd2] - 2026-07-XX

### Added — OBD-II via ELM327
- **Complete OBD-II subsystem** — 6 new source files in `obd2_implementation/`:
  - `ELM327.h/.cpp` — AT command driver (ATZ, ATE0, ATL0, ATS0, ATSP),
    PID request/response parsing, DTC read (Mode 03) / clear (Mode 04),
    supported PID bitmap query (PID 0x00)
  - `OBD2Data.h/.cpp` — 40+ SAE J1979 decode formulas stored in PROGMEM,
    PID value cache with timestamps, unit/name lookup
  - `OBD2Manager.h/.cpp` — 3-tier priority poll scheduler (Tier 1 every cycle,
    Tier 2 every 2nd, Tier 3 every 5th), standalone serial frame builder
    (TAG 0x4F), global `obd2` instance
- `MultidisplayDefinesOBD2.h` — ELM327 configuration header:
  - 38400 baud default (ELM327DSL datasheet)
  - ATSP3 (ISO 9141-2) protocol selection
  - Per-PID enable/disable via `#define OBD2_ENABLE_PID_xxx`
  - 18 PIDs enabled across 3 tiers, 5 additional PIDs available (commented)
  - `OBD2_MAX_TRACKED_PIDS = 32` configurable limit
- **35-byte OBD2 summary** embedded in standard binary frame at bytes 58–92:
  - 1 flags byte (bit0=ELM ready, bit1=DTCs present)
  - 16 PID values as `int16_t × 100` little-endian
  - 2 reserved bytes
- **Standalone OBD2 frame** (TAG 0x4F): variable-length per-PID detail frame
  for PC logging software
- `platformio.ini` — PlatformIO build system for ATmega2560

### Added — Documentation
- `OBD2_IMPLEMENTATION.md` — Comprehensive OBD-II guide: architecture, wiring,
  AT protocol, PID configuration, tier scheduler, SAE J1979 formulas,
  binary frame format, BT mirroring, build/flash, troubleshooting, roadmap
- `CHANGELOG.md` — This file
- Updated `SERIAL_PROTOCOL.md` §3.3 — OBD2 variant data byte map added
  (preceding Digifant K-line and VR6 sections)
- Updated `.github/agents/multidisplay.agent.md` — OBD-II standards compliance
  section, coding rules, build commands, editing checklist

### Changed
- `MultidisplayDefines.h`:
  - `ECU` define set to `OBD2_ELM327_ECU`
  - `BLUETOOTH_ON_SERIAL2 = 1` enabled for BT mirroring
  - Added `MD_SERIAL_WRITE` macro: writes to both `Serial` and `Serial2`
    when `BLUETOOTH_ON_SERIAL2` is defined
- `MultidisplayController.h` — Added `#include "obd2_implementation/OBD2Manager.h"`
  behind `#ifdef OBD2_ELM327` guard
- `MultidisplayController.cpp`:
  - `myconstructor()`: calls `obd2.begin()` for ELM327 initialization
  - `mainLoop()`: calls `obd2.poll()` for PID scheduling
  - `serialSend()`: calls `obd2.serialSendOBD2()` after main frame
  - `SERIALOUT_BINARY` case: 35-byte OBD2 summary written at bytes 58–92

### Fixed
- **Bluetooth Serial2 payload mirroring** — Previously only STX and ETX
  were sent on Serial2; the 93-byte payload body was sent only on Serial.
  ALL `Serial.write()` calls in the `SERIALOUT_BINARY` case replaced with
  `MD_SERIAL_WRITE()` macro, ensuring complete 95-byte frames on Bluetooth.

### Standards
- SAE J1979-2 — All PID numbers and decode formulas
- ISO 15031-5 — DTC encoding format
- ISO 9141-2 — K-line protocol (via ELM327 ATSP3)
- ELM327DSL Datasheet — AT command syntax

### Build Metrics
- RAM: 2670 / 8192 bytes (32.6%)
- Flash: 40600 / 253952 bytes (16.0%)

### Improved — OBD-II Reliability & Performance
- **Echo-tolerant PID parser** — `parseOBDResponse()` now handles ELM327
  echo prefix (ATE0 not always effective on v1.4 chips). Uses offset-based
  parsing at positions [0, 4, 5] with full-scan fallback for BUS INIT text.
  New `tryParseAt()` helper for modular hex parsing.
- **Per-PID NO_DATA auto-disable** — PIDs returning NO DATA are disabled
  after 3 consecutive failures, preventing repeated polling of unsupported
  PIDs. Logged as `[OBD2] PID 0xXX disabled (NO DATA)`.
- **Aggressive adaptive timing** — Changed from AT AT1 to AT AT2 for faster
  OBD bus timeout adaptation after first successful response.
- **Reduced OBD timeout** — ATST lowered from FF (1020ms) to 32 (200ms).
  Unsupported PIDs fail in 200ms instead of 1s, dramatically improving
  effective poll rate. AT AT2 adapts down further for responsive ECUs.
- **MCU-side timeout reduced** — ELM_TIMEOUT_MS lowered from 2000ms to
  1000ms, matching the faster ATST and reducing worst-case stall time.
- **Baud rate upgrade to 38400** — AT BRD handshake working at 38400 baud:
  - Correct BRD divisor: `hh = round(4M / baud)` → 0x68 for 38400
  - Correct protocol: ELM sends "OK\r" (no `>`) → switch baud → read ID
    at new baud → confirm with CR → ELM responds "OK\r>" (per datasheet
    §"Using Higher RS232 Baud Rates")
  - Strategy B (PP 0C/PP 00 permanent baud) as fallback
  - Auto-detect scan prioritizes target baud as 2nd scan for post-PP boots
  - Frame rate: ~5.0 Hz (ISO 9141-2 bus speed is bottleneck, not serial)
- **ATE0 verification** — Echo Off command now verified with ATI probe;
  retried once if echo is still present.
- **Cleaner init** — Startup delay reduced from 8s to 2s. Diagnostic
  output streamlined for production use.

### Added — ELM327 Driver Enhancements
- `ELM_PARSE_FAIL` status code for responses that pass checkResponse()
  but fail hex parsing (aids debugging)
- `tryParseAt()` — modular OBD hex parser starting at arbitrary buffer offset
- `_activeBaud` member + `activeBaud()` accessor for runtime baud reporting
- Heartbeat log: `[MD] t=<ms> OBD2=<READY|FAIL> PIDs=<n> baud=<rate>`

---

## [v2.0.0] - Pre-OBD2 Baseline

### Features (existing before OBD-II)
- 95-byte binary serial frame (SERIALOUT_BINARY) at 100 Hz
- 8× EGT channels (Type K thermocouple)
- 8× VDO sensors (3 pressure, 3 temperature)
- RPM, boost (absolute), throttle, lambda, LMM, battery voltage
- Vehicle speed + gear computation
- N75 boost control with PID (gear + RPM dependent)
- BorgWarner EFR turbo speed sensor
- Digifant K-line ECU interface
- VR6 Motronic ECU interface
- 20×4 LCD display with multiple screens
- Bluetooth SPP via HC-06 on Serial2
- Data streaming to PC (binary + text + TunerPro ADX modes)
- Event logging (max RPM, etc.) with EEPROM persistence
