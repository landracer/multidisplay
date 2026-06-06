# MDLogger ↔ MultiDisplay I2C Protocol

**Status:** Implemented. Zero-warning builds on both sides. Ready for hardware bring-up.
**Slave:** ATmega328P running [`MDLogger`](../../multidisplay-firmware/MDLogger/) at I2C address `0x2A`.
**Master:** ATmega2560 running [`MultiDisplay`](../../multidisplay-firmware/multidisplay/).

---

## 1. Why this exists

The MultiDisplay (Mega2560) collects sensor data at ~100 Hz and streams it to a host
PC / OpenDash over UART. We also want **persistent SD-card logging** at full rate
**including GPS** without burdening the Mega's already-busy main loop.

Solution: a small ATmega328P co-processor — the **MDLogger** — owns the SD card
and a high-rate u-blox GPS, and listens on I2C for sensor frames pushed by the Mega.

```
   ┌────────────────────┐         ┌──────────────────────┐         ┌────────┐
   │  MultiDisplay      │   I2C   │  MDLogger            │  SPI    │   SD   │
   │  (Mega2560,master) │ ──────► │  (ATmega328P,slave)  │ ──────► │  card  │
   │                    │  0x2A   │                      │         └────────┘
   │  100 Hz sensors    │         │  GPS @ 10–15 Hz      │
   └────────────────────┘         └──────────────────────┘
```

---

## 2. Wire format (raw — no STX/LEN/ETX framing)

I2C already provides `START`/`STOP` delimiting and per-byte `ACK`/`NACK`.
We do **not** add a redundant framing layer.

### 2.1 Master → Slave

| Opcode | Name              | Payload                              | Total bytes |
|-------:|-------------------|--------------------------------------|------------:|
| `0x01` | `DATA_FRAME`      | 74-byte `MDLoggerFrame` struct       | 75          |
| `0x02` | `LOG_REQUEST`     | (none — request manual SD flush)     | 1           |
| `0x03` | `STATUS_REQUEST`  | (none — sets up read response)       | 1           |
| `0x04` | `CONFIG`          | `[type][valLo][valHi]`                | 4           |

`CONFIG` types:

| `type` | Meaning            | Value (16-bit)                |
|-------:|--------------------|-------------------------------|
| `0x01` | Set PID mask       | bitmask of `MDLOGGER_PID_*`   |
| `0x02` | Set GPS poll rate  | Hz (10..20)                   |
| `0x03` | Set log interval   | ms (TODO — not yet wired)     |

### 2.2 Slave → Master (read after `STATUS_REQUEST`)

5 bytes, no framing:

| Byte | Field            | Meaning                                       |
|-----:|------------------|-----------------------------------------------|
| 0    | `statusFlags`    | bit-packed `MDLoggerStatus.asByte`            |
| 1    | `satellites`     | GPS satellite count                           |
| 2    | `fix`            | `0` = no fix, `1` = fix                       |
| 3    | `gpsConfigured`  | `1` after UBX init succeeds                    |
| 4    | `dataFieldsValid`| `1` once GPRMC/GGA seen                       |

`statusFlags` bit layout (from `MDLoggerStatus` union):

```
bit 0: sdReady        — SD card initialized & log file open
bit 1: bufferFull     — frame queue dropped a frame (back-pressure flag)
bit 2: loggingActive  — actively writing records
bit 3: gpsHasFix      — duplicate of byte 2 for convenience
bit 4–7: reserved
```

### 2.3 Slave → Master (read after non-`STATUS_REQUEST`)

| Opcode that triggered it | Response                  |
|--------------------------|---------------------------|
| `DATA_FRAME`             | none (master doesn't read) |
| `LOG_REQUEST`            | `[0x80]` (ACK, 1 byte)     |
| `CONFIG`                 | `[0x80]` (ACK, 1 byte)     |
| unknown opcode           | `[0xFF][offendingCmd]` (2 bytes) |

The master currently only consumes `STATUS_REQUEST` responses. ACK/ERROR are
held in the slave's `_i2cRespBuffer` for inspection.

---

## 3. The 74-byte `MDLoggerFrame` struct

Defined in [`MDLoggerProtocol.h`](../../multidisplay-firmware/multidisplay/MDLoggerProtocol.h)
and shared by both projects. Packed (`__attribute__((packed))`).

| Offset | Size | Field          | Type      | Scaling           | Notes                                       |
|-------:|-----:|----------------|-----------|-------------------|---------------------------------------------|
| 0      | 4    | `timestamp`    | uint32 LE | raw               | Mega's `millis()`                           |
| 4      | 57   | `mdData[57]`   | byte[]    | per-field         | Sensor payload (see §3.1)                   |
| 61     | 4    | `latitude`     | int32 LE  | × 1 000 000       | Degrees                                     |
| 65     | 4    | `longitude`    | int32 LE  | × 1 000 000       | Degrees                                     |
| 69     | 2    | `speedKmh100`  | uint16 LE | × 100             | km/h                                        |
| 71     | 2    | `course100`    | uint16 LE | × 100             | Degrees                                     |
| 73     | 1    | `satellites`   | uint8     | raw               | GPS satellite count                          |
| 74-... | -    | -              | -         | -                 | (struct ends — total 74 bytes incl. `fix`)  |

> The struct is `sizeof(MDLoggerFrame) == 74` exactly because `fix` is the last byte.

### 3.1 `mdData[57]` byte map

Mirrors the SERIALOUT_BINARY layout, **without** the leading `STX`/`TAG`/`time`
fields (those are either implicit or carried in the `timestamp` field above).
Bytes 53-56 are reserved and zeroed by the master's packer.

| `mdData` offset | Size | Field            | Scaling | Source                            |
|----------------:|-----:|------------------|---------|-----------------------------------|
| 0..1            | 2    | calRPM           | raw     | `data.calRPM`                     |
| 2..3            | 2    | calAbsoluteBoost | × 100   | `data.calAbsoluteBoost`           |
| 4               | 1    | calThrottle      | raw     | `data.calThrottle`                |
| 5..6            | 2    | calLambdaF       | × 100   | `data.calLambdaF`                 |
| 7..8            | 2    | calLMM           | × 100   | `data.calLMM`                     |
| 9..10           | 2    | calCaseTemp      | × 100   | `data.calCaseTemp`                |
| 11..26          | 16   | calEgt[0..7]     | raw     | `data.calEgt[i]`                  |
| 27..28          | 2    | batVolt          | × 100   | `data.batVolt`                    |
| 29..30          | 2    | VDOPres1         | × 10    | `data.VDOPres1`                   |
| 31..32          | 2    | VDOPres2         | × 10    | `data.VDOPres2`                   |
| 33..34          | 2    | VDOPres3         | × 10    | `data.VDOPres3`                   |
| 35..36          | 2    | VDOTemp1         | raw     | `data.VDOTemp1`                   |
| 37..38          | 2    | VDOTemp2         | raw     | `data.VDOTemp2`                   |
| 39..40          | 2    | VDOTemp3         | raw     | `data.VDOTemp3`                   |
| 41..42          | 2    | speedF           | × 100   | `data.speedF`                     |
| 43              | 1    | gear             | raw     | `data.gear`                       |
| 44              | 1    | n75_duty         | raw     | `boostController.boostOutput`     |
| 45..46          | 2    | req_Boost        | × 100   | `boostController.req_Boost`       |
| 47              | 1    | req_Boost_PWM    | raw     | `boostController.req_Boost_PWM`   |
| 48              | 1    | flags            | bits    | bit 0: PID enabled, bit 1: aggressive |
| 49..50          | 2    | efr_speed_reading| raw     | `data.efr_speed_reading`          |
| 51..52          | 2    | knock            | raw     | `data.knock`                      |
| 53..56          | 4    | reserved         | -       | zeroed                            |

---

## 4. Buffer sizing (critical)

The default Arduino Wire library hard-codes a **32-byte** I2C buffer.
A `DATA_FRAME` is **75 bytes** (1 opcode + 74-byte struct). To make this work
we bumped the buffer on both sides:

| Side    | File                                    | Change                                          |
|---------|-----------------------------------------|-------------------------------------------------|
| Slave   | [`MDLogger/platformio.ini`](../../multidisplay-firmware/MDLogger/platformio.ini) | Added `-D TWI_BUFFER_SIZE=96`                   |
| Master  | [`multidisplay/libs/Wire.h`](../../multidisplay-firmware/multidisplay/libs/Wire.h) | `#define BUFFER_LENGTH 96` (was 32)             |
| Master  | [`multidisplay/libs/twi.h`](../../multidisplay-firmware/multidisplay/libs/twi.h) | `#define TWI_BUFFER_LENGTH 96` (was 32)         |

**RAM cost:**

| Side          | Before     | After      | Δ        |
|---------------|-----------:|-----------:|---------:|
| Slave (328P)  | 1682 / 2048 (82.1 %) | 1838 / 2048 (89.7 %) | +156 B |
| Master (Mega) | 3014 / 8192 (36.8 %) | 3014 / 8192 (36.8 %) | 0 B *   |

*The master's `_i2cBuffer` was eliminated when we removed the framing layer,
which paid for the TWI buffer growth.

---

## 5. ISR-safety pattern (slave side)

I2C events fire from inside the AVR TWI ISR. The slave is careful to keep ISR
work short:

```
                   ┌──────────────┐
   I2C onReceive ──► read CMD     │
                   │ if DATA_FRAME│
                   │   memcpy 74B │ ──► queue (2 slots)
                   │ if STATUS_RQ │
                   │   build 5B   │ ──► _i2cRespBuffer
                   └──────────────┘
                          │
                          ▼  (NO SD I/O HERE)
                   ┌──────────────┐
   loop()/poll() ──► drainI2CQueue│ ──► writeLogRecord ──► SD.write
                   │ flushSD (5s) │ ──► SD.flush
                   │ pollGPS      │ ──► SoftwareSerial drain
                   └──────────────┘
```

Rules:
- `onReceive`/`onRequest` MUST NOT call `Serial.print` or SD operations.
- All inbound `DATA_FRAME` payloads land in a 2-slot ring buffer.
- The main loop drains the queue, writes to SD, and periodically flushes.

---

## 6. Reconnect / link-health (master side)

The slave may boot later than the master (typical when both share VBAT).
The master tolerates this:

1. `MDLoggerClient::begin()` does one best-effort `pollStatus()` and returns.
2. The main loop calls `pollStatus()` **unconditionally** every 5 s.
3. A successful `pollStatus()` sets `_ready = true` (link up).
4. Any failed `Wire.endTransmission()` clears `_ready` (link down).
5. `sendData()` is a no-op while `_ready == false` — no bus traffic wasted.

This means the system self-heals from:
- Late slave boot
- Brief I2C bus glitches (loose wire, brown-out)
- Slave watchdog reset

---

## 7. Build & flash

### 7.1 Slave (ATmega328P) — AVR-ISP-MK2

The prototype board uses the ISP header, not USB. Flash with avrdude:

```bash
cd multidisplay-firmware/MDLogger
pio run                          # build → .pio/build/MDLogger/firmware.hex
avrdude -c avrispmkii -p m328p -P usb -U flash:w:.pio/build/MDLogger/firmware.hex:i
```

Expected boot output (via serial monitor on the board's USB-serial):

```
[MDLogger] Starting...
[MDLogger] SD initialized
[MDLogger] Log file: MD0001.CSV
[MDLogger] GPS configured for high-rate output
[MDLogger] Ready (I2C addr 0x2A)
```

### 7.2 Master (ATmega2560) — USB or AVR-ISP-MK2

**USB (Arduino bootloader):**
```bash
cd multidisplay-firmware/multidisplay
pio run -e mega2560              # build
pio run -e mega2560 -t upload    # flash via USB bootloader
pio device monitor               # 115200 baud
```

**AVR-ISP-MK2 (direct):**
```bash
cd multidisplay-firmware/multidisplay
pio run -e mega2560              # build → .pio/build/mega2560/firmware.hex
avrdude -c avrispmkii -p m2560 -P usb -U flash:w:.pio/build/mega2560/firmware.hex:i
```

Expected debug heartbeat (every 5 s):

```
[MD] t=12345 OBD2=READY PIDs=18 baud=38400 GPS=FIX sat=8 RTC=14:23:01 SD=OK
                                                                       ^^^^
                                                  MD=OK = MDLogger STATUS poll succeeded
```

---

## 8. Bring-up checklist

| # | Step                                                              | Pass criteria                                  |
|---|-------------------------------------------------------------------|------------------------------------------------|
| 1 | Flash slave standalone (no Mega connected, no I2C)                | SD CSV file created, "Ready" message          |
| 2 | Flash master standalone (no slave connected)                      | Heartbeat shows `SD=FAIL` (correct)            |
| 3 | Connect I2C SDA/SCL with pull-ups (both rails powered)            | Heartbeat changes to `SD=OK` within 5 s        |
| 4 | Spin engine to log real data                                      | CSV rows accumulate at ~100 Hz on slave SD     |
| 5 | Pull GPS antenna outdoors                                         | Heartbeat shows `GPS=FIX sat=N`, lat/lon non-0 |
| 6 | Yank slave power, restore                                         | `SD=FAIL` then `SD=OK` within 5 s              |

---

## 9. File map

### Slave (`multidisplay-firmware/MDLogger/`)
| File                           | Role                                            |
|--------------------------------|-------------------------------------------------|
| `platformio.ini`               | Build config; `TWI_BUFFER_SIZE=96`              |
| `include/MDLogger.h`           | Class decl, defines, `MDLoggerFrame` struct     |
| `include/GPS_CONFIG.h`         | UBX helpers, `configureGPS()`                   |
| `src/MDLogger.cpp`             | Full implementation (ISR handlers, SD, GPS)    |
| `src/main.cpp`                 | Arduino `setup()`/`loop()` shim                 |

### Master (`multidisplay-firmware/multidisplay/`)
| File                           | Role                                            |
|--------------------------------|-------------------------------------------------|
| `MDLoggerProtocol.h`           | **Shared** wire-format types + opcodes          |
| `MDLoggerClient.h`             | Master-side driver class declaration            |
| `MDLoggerClient.cpp`           | `packFrame`/`sendData`/`pollStatus`             |
| `MultidisplayController.cpp`   | Calls `mdLoggerClient.*`; `sdLogWriteRecord()` packer |
| `libs/Wire.h`/`libs/twi.h`     | Custom Wire library, `BUFFER_LENGTH=96`         |

---

## 10. Known limitations / future work

- **`CONFIG` opcode `0x03` (log interval)** is parsed but not applied to `_nextLogTime`. Stub.
- **`LOG_REQUEST` (0x02)** triggers `flushSD()` from inside the I2C ISR. Low risk because it's manual, but if abused at high rate it could stall the bus briefly.
- **GPS course (`course100`)** is plumbed end-to-end but the slave's NMEA parser doesn't currently extract it — always 0.
- **Bidirectional handshake**: there's no version byte. If we revise the frame layout, both sides must be re-flashed simultaneously. Add a `STATUS` byte for protocol version if this becomes a problem.
- **Hardware**: I2C lines need 4.7 kΩ pull-ups to 3.3 V (or 5 V if both devices are 5 V tolerant). Bus length should stay under ~50 cm without active buffers.

---

*Document version 1.0 — generated 2026-04-26.*
