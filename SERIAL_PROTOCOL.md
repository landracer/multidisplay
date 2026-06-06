# Multidisplay Serial Protocol Reference

> **Canonical specification for the multidisplay serial data stream.**
> Source of truth: `MultidisplayController::serialSend()` in `MultidisplayController.cpp`.
> This document is intended as a standalone reference — any LLM or developer should
> be able to implement a parser from this file alone.

---

## 1. Overview

The **multidisplay** is an AVR ATmega-based automotive data acquisition system that reads
engine sensors (RPM, boost, EGT, lambda, etc.) and streams the data over serial.

| Parameter         | Value                                    |
|-------------------|------------------------------------------|
| Baud rate         | **115 200** (8N1)                        |
| Byte order        | **Little-endian** (AVR ATmega)           |
| `sizeof(int)`     | **2 bytes** (AVR)                        |
| `sizeof(unsigned long)` | **4 bytes** (AVR)                  |
| Default send rate | Every **10 ms** (100 Hz), configurable   |
| Default mode      | `SERIALOUT_BINARY` (mode 1)              |

### Serial Ports

| Port      | Arduino Pin | Usage                                               |
|-----------|-------------|-----------------------------------------------------|
| `Serial`  | USB / Pin 0-1 | Primary output — full frames sent here            |
| `Serial2` | Pin 16-17   | Bluetooth (HC-06 slave, name `mdv2`). Only sends full frames if `BLUETOOTH_ON_SERIAL2` is enabled in the build. |

> **Note:** The stock source code only sends STX and ETX on `Serial2`, not the payload.
> Custom builds (e.g. rAtTrax) may patch `serialSend()` to send the full payload on
> `Serial2` as well. If you are receiving via Bluetooth and only get 2-byte frames
> (0x02 0x03), the payload writes need to be duplicated to `Serial2`.

### Serial Modes

| Mode | Value | Name                     | Description                              |
|------|-------|--------------------------|------------------------------------------|
| 0    | `SERIALOUT_DISABLED`     | Off — no data sent                        |
| 1    | `SERIALOUT_BINARY`       | Binary frames (default) — fixed 95-byte   |
| 2    | `SERIALOUT_ENABLED`      | Human-readable text (semicolon-separated) |
| 3    | `SERIALOUT_TUNERPRO_ADX` | TunerPro ADX format (Digifant specific)   |

The mode cycles with `ChangeSerOut()`: Disabled → Binary → Text → Disabled.

---

## 2. Frame Envelope

Every frame (all modes) is wrapped:

```
┌─────┬────────────────────────────────────────────┬─────┐
│ STX │                 payload                    │ ETX │
│0x02 │           (mode-specific data)             │0x03 │
└─────┴────────────────────────────────────────────┴─────┘
```

- **STX** = `0x02` (ASCII Start of Text)
- **ETX** = `0x03` (ASCII End of Text)
- There is **no byte-stuffing** — if a payload byte happens to be 0x02 or 0x03,
  it is sent as-is. Receivers must use the fixed frame size (binary mode) or
  newline/timeout (text mode) to detect frame boundaries.

---

## 3. Binary Frame Format (SERIALOUT_BINARY, mode 1)

### 3.1 Frame Structure

| # | Field            | Offset | Size | C Type         | Scaling          | Unit / Notes                    |
|---|------------------|--------|------|----------------|------------------|---------------------------------|
| — | STX              | 0      | 1    | `uint8_t`      | literal `0x02`   | Frame start                     |
| 1 | TAG              | 1      | 1    | `uint8_t`      | literal `95`     | = `SERIALOUT_BINARY_TAG` = total frame size |
| 2 | time             | 2      | 4    | `uint32_t` LE  | raw              | `millis()` — ms since boot      |
| 3 | calRPM           | 6      | 2    | `int16_t` LE   | raw              | Engine RPM                       |
| 4 | calAbsoluteBoost | 8      | 2    | `uint16_t` LE  | ÷100.0           | Absolute boost in **Bar**        |
| 5 | calThrottle      | 10     | 1    | `uint8_t`      | raw              | Throttle position 0–100 %       |
| 6 | calLambdaF       | 11     | 2    | `uint16_t` LE  | ÷100.0           | Lambda ratio (0.80 = rich, 1.00 = stoich) |
| 7 | calLMM           | 13     | 2    | `uint16_t` LE  | ÷100.0           | Air-flow meter voltage (0–5 V)  |
| 8 | calCaseTemp      | 15     | 2    | `uint16_t` LE  | ÷100.0           | Case / ambient temp in **°C**   |
| 9 | calEgt[0]        | 17     | 2    | `uint16_t` LE  | raw              | EGT channel 0 in **°C**         |
| 10| calEgt[1]        | 19     | 2    | `uint16_t` LE  | raw              | EGT channel 1 in **°C**         |
| 11| calEgt[2]        | 21     | 2    | `uint16_t` LE  | raw              | EGT channel 2 in **°C**         |
| 12| calEgt[3]        | 23     | 2    | `uint16_t` LE  | raw              | EGT channel 3 in **°C**         |
| 13| calEgt[4]        | 25     | 2    | `uint16_t` LE  | raw              | EGT channel 4 in **°C**         |
| 14| calEgt[5]        | 27     | 2    | `uint16_t` LE  | raw              | EGT channel 5 in **°C**         |
| 15| calEgt[6]        | 29     | 2    | `uint16_t` LE  | raw              | EGT channel 6 in **°C**         |
| 16| calEgt[7]        | 31     | 2    | `uint16_t` LE  | raw              | EGT channel 7 in **°C**         |
| 17| batVolt          | 33     | 2    | `uint16_t` LE  | ÷100.0           | Battery voltage in **V**         |
| 18| VDOPres1         | 35     | 2    | `int16_t` LE   | ÷10.0            | VDO pressure 1 in **Bar** (oil) |
| 19| VDOPres2         | 37     | 2    | `int16_t` LE   | ÷10.0            | VDO pressure 2 in **Bar**       |
| 20| VDOPres3         | 39     | 2    | `int16_t` LE   | ÷10.0            | VDO pressure 3 in **Bar**       |
| 21| VDOTemp1         | 41     | 2    | `int16_t` LE   | raw              | VDO temperature 1 in **°C**     |
| 22| VDOTemp2         | 43     | 2    | `int16_t` LE   | raw              | VDO temperature 2 in **°C**     |
| 23| VDOTemp3         | 45     | 2    | `int16_t` LE   | raw              | VDO temperature 3 in **°C**     |
| 24| speedF           | 47     | 2    | `uint16_t` LE  | ÷100.0           | Vehicle speed in **km/h**        |
| 25| gear             | 49     | 1    | `uint8_t`      | raw              | Current gear (1–6, 0=neutral)    |
| 26| n75_duty         | 50     | 1    | `uint8_t`      | raw              | N75 solenoid duty cycle (0–255)  |
| 27| req_Boost        | 51     | 2    | `uint16_t` LE  | ÷100.0           | Requested boost setpoint in **Bar** |
| 28| req_Boost_PWM    | 53     | 1    | `uint8_t`      | raw              | N75 map raw PWM value            |
| 29| flags            | 54     | 1    | `uint8_t`      | bitfield         | bit 0: PID enabled, bit 1: aggressive PID |
| 30| efr_speed_reading| 55     | 2    | `uint16_t` LE  | see §3.4         | EFR turbo speed timer ticks      |
| 31| knock            | 57     | 2    | `uint16_t` LE  | raw              | Knock sensor value               |
| — | *variant data*   | 59     | 35   | `uint8_t[35]`  | varies           | See §3.3 below                   |
| — | ETX              | 94     | 1    | `uint8_t`      | literal `0x03`   | Frame end                        |

**Total: 95 bytes per frame** (1 STX + 1 TAG + 57 MD2 data + 35 variant data + 1 ETX)

### 3.2 Byte Offset Quick Reference (payload only, 0-based from TAG)

For parser implementations, offsets relative to STX (byte 0):

```
Offset 0:  STX   (0x02)
Offset 1:  TAG   (0x5F = 95 decimal)
Offset 2:  time  [4 bytes, uint32 LE]    → millis()
Offset 6:  RPM   [2 bytes, int16 LE]     → raw RPM
Offset 8:  boost [2 bytes, uint16 LE]    → ÷100 for Bar (absolute)
Offset 10: thr   [1 byte,  uint8]        → throttle %
Offset 11: lambda[2 bytes, uint16 LE]    → ÷100 for ratio
Offset 13: LMM   [2 bytes, uint16 LE]    → ÷100 for Volts
Offset 15: case  [2 bytes, uint16 LE]    → ÷100 for °C
Offset 17: egt0  [2 bytes, uint16 LE]    → °C
Offset 19: egt1  [2 bytes, uint16 LE]    → °C
Offset 21: egt2  [2 bytes, uint16 LE]    → °C
Offset 23: egt3  [2 bytes, uint16 LE]    → °C
Offset 25: egt4  [2 bytes, uint16 LE]    → °C
Offset 27: egt5  [2 bytes, uint16 LE]    → °C
Offset 29: egt6  [2 bytes, uint16 LE]    → °C
Offset 31: egt7  [2 bytes, uint16 LE]    → °C
Offset 33: bat   [2 bytes, uint16 LE]    → ÷100 for Volts
Offset 35: vdoP1 [2 bytes, int16 LE]     → ÷10 for Bar
Offset 37: vdoP2 [2 bytes, int16 LE]     → ÷10 for Bar
Offset 39: vdoP3 [2 bytes, int16 LE]     → ÷10 for Bar
Offset 41: vdoT1 [2 bytes, int16 LE]     → °C raw
Offset 43: vdoT2 [2 bytes, int16 LE]     → °C raw
Offset 45: vdoT3 [2 bytes, int16 LE]     → °C raw
Offset 47: speed [2 bytes, uint16 LE]    → ÷100 for km/h
Offset 49: gear  [1 byte,  uint8]        → 1-6
Offset 50: n75   [1 byte,  uint8]        → duty 0-255
Offset 51: rqBst [2 bytes, uint16 LE]    → ÷100 for Bar
Offset 53: rqPWM [1 byte,  uint8]        → raw PWM
Offset 54: flags [1 byte,  uint8]        → bitfield
Offset 55: efr   [2 bytes, uint16 LE]    → timer ticks (see §3.4)
Offset 57: knock [2 bytes, uint16 LE]    → raw
Offset 59: variant data [35 bytes]       → see §3.3
Offset 94: ETX   (0x03)
```

### 3.3 Variant-Specific Trailing Data (Bytes 59–93)

#### VR6 Motronic Build (`#define VR6_MOTRONIC`)

35 bytes of **zeros** (padding). No useful data.

#### Digifant K-Line Build (`#define DIGIFANT_KLINE`)

| Offset | Size | Description                                     |
|--------|------|-------------------------------------------------|
| 59     | 32   | Digifant K-line frame (bytes[1..32] of raw frame, excluding STX/ETX) |
| 91     | 2    | `df_kline_discarded_frames` (uint16 LE) — debug counter |
| 93     | 1    | `df_kline_last_frame_completely_received` — frame index (255 = none) |

If no Digifant K-line frame has been received, 32 zero bytes + 2 zero bytes + 0xFF.

### 3.4 EFR Turbo Speed Conversion

The `efr_speed_reading` field is a raw timer count from the EFR compressor speed sensor.
Convert to RPM:

```
if efr_speed_reading == 0 or efr_speed_reading == 0xFFFF:
    efr_rpm = 0
else:
    efr_rpm = BW_EFR_TIME_2_RPM / efr_speed_reading
```

Where `BW_EFR_TIME_2_RPM = 40000000` (defined in `MultidisplayDefines.h`).

### 3.5 Float-to-Fixed Conversion (`float2fixedintb100`)

All float values are converted to `uint16_t` via:

```c
uint16_t float2fixedintb100(float in) {
    return (uint16_t)(in * 100.0);
}
```

To recover the original float on the receiver:

```c
float value = (float)read_uint16_le() / 100.0;
```

Fields using ×100 scaling: `calAbsoluteBoost`, `calLambdaF`, `calLMM`, `calCaseTemp`,
`batVolt`, `speedF`, `req_Boost`.

VDO pressures use ×10 scaling (fixed-int base 10).

### 3.6 Boost: Relative vs Absolute

- **Binary mode** sends `calAbsoluteBoost` (absolute pressure in Bar, always positive).
  Typical sea-level idle ≈ 1.00 Bar. Full boost ≈ 2.50 Bar.
- **Text mode** sends `calBoost` (relative to atmosphere). Vacuum is negative, boost is positive.

To convert absolute → relative: `relative = absolute - boostAmbientPressureBar`
(ambient is calibrated at startup, typically ~1.01 Bar at sea level).

---

## 4. Text Frame Format (SERIALOUT_ENABLED, mode 2)

Semicolon-separated ASCII values, wrapped in STX/ETX. No trailing newline inside frame.

```
\x02 2:TIME;RPM;BOOST;THROTTLE;LAMBDA;LMM;CASETEMP;EGT0;EGT1;BATVOLT;VDOP1;VDOP2;VDOP3;VDOT1;VDOT2;VDOT3 \x03
```

| Field # | Name         | Arduino Type | Format        | Unit                  |
|---------|-------------|-------------|---------------|-----------------------|
| prefix  | `2:`        | —           | mode tag      | Always "2:"           |
| 1       | time        | unsigned long | decimal      | ms since boot         |
| 2       | calRPM      | int          | decimal       | RPM                   |
| 3       | calBoost    | float        | decimal       | **Relative** Bar      |
| 4       | calThrottle | int          | decimal       | 0–100 %               |
| 5       | calLambdaF  | float        | decimal       | Lambda ratio          |
| 6       | calLMM      | float        | decimal       | Volts (0–5)           |
| 7       | calCaseTemp | float        | decimal       | °C                    |
| 8       | calEgt[0]   | uint16       | decimal       | °C                    |
| 9       | calEgt[1]   | uint16       | decimal       | °C                    |
| 10      | batVolt     | float        | decimal       | Volts                 |
| 11      | VDOPres1    | int          | decimal       | ×10 (÷10 for Bar)     |
| 12      | VDOPres2    | int          | decimal       | ×10 (÷10 for Bar)     |
| 13      | VDOPres3    | int          | decimal       | ×10 (÷10 for Bar)     |
| 14      | VDOTemp1    | int          | decimal       | °C                    |
| 15      | VDOTemp2    | int          | decimal       | °C                    |
| 16      | VDOTemp3    | int          | decimal       | °C                    |

**Key difference from binary:** Text mode only sends **2 EGT channels** (0 and 1),
sends **relative** boost (not absolute), and does NOT send speed, gear, N75,
EFR speed, knock, or variant data.

Example text frame:
```
\x02 2:123456;3200;0.85;75;0.92;3.21;45.30;620;590;13.80;32;0;0;95;0;0 \x03
```

---

## 5. EGT (Exhaust Gas Temperature) Details

### 5.1 Hardware

Type K thermocouple channels are read via MAX31855 or AD8495 ICs connected through
a multiplexer. The multidisplay cycles through channels using a state machine:

```c
#define MAX_ATTACHED_TYPE_K  8    // array size (always 8 slots in binary frame)
#define NUMBER_OF_ATTACHED_TYPE_K  6   // actually connected (VR6 build)
                                       // = 5 for Digifant build
```

### 5.2 Reading Cycle

The TypK state machine (`FetchTypK` / `fetchTypK2` / `fetchTypK3_fast`) cycles:
1. Select multiplexer channel N
2. Wait for ADC conversion (~100ms)
3. Read channel N → store in `data.calEgt[N]`
4. Increment N; wrap at `NUMBER_OF_ATTACHED_TYPE_K`

At each wrap (all channels read), `checkAndSaveMaxEgt()` updates the hottest channel.

### 5.3 Binary vs Text EGT

| Mode   | Channels Sent | Slots in Frame |
|--------|---------------|----------------|
| Binary | ALL 8 (`MAX_ATTACHED_TYPE_K`) | `calEgt[0]` through `calEgt[7]` |
| Text   | Only 2        | `calEgt[0]` and `calEgt[1]`     |

**Channels beyond `NUMBER_OF_ATTACHED_TYPE_K` will read 0 or contain stale data.**
Values ≥ 1200 °C indicate an open (disconnected) thermocouple (`MAX_TYPE_K = 1200`).

### 5.4 For VR6 with 6 EGTs

- `calEgt[0]` through `calEgt[5]` → valid (cylinders 1–6)
- `calEgt[6]` and `calEgt[7]` → not connected, will be 0 or stale

### 5.5 For Digifant with 5 EGTs

- `calEgt[0]` through `calEgt[4]` → valid
- `calEgt[5]` through `calEgt[7]` → not connected

---

## 6. Receive Protocol (PC → Multidisplay)

The multidisplay can receive commands via `serialReceive()`. Commands are framed with
STX/ETX and use the `srData` union (48 bytes max payload).

### 6.1 Command Format

```
\x02  CMD_BYTE  [PAYLOAD...]  \x03
```

The first byte after STX is checked against TAG values. If it does not match any
known tag, it is treated as a command byte with srData payload following.

### 6.2 Binary Response Tags

| TAG Value | Name                             | Description                       |
|-----------|----------------------------------|-----------------------------------|
| 95        | `SERIALOUT_BINARY_TAG`           | Standard data frame (MD → PC)     |
| 4         | `SERIALOUT_BINARY_TAG_ACK`       | Acknowledgment response           |
| 22        | `SERIALOUT_BINARY_TAG_N75_DUTY_MAP` | N75 duty map data              |
| 38        | `SERIALOUT_BINARY_TAG_N75_SETPOINT_MAP` | N75 setpoint map data     |
| 23        | `SERIALOUT_BINARY_TAG_N75_PARAMS` | N75 PID parameters               |
| 17        | `SERIALOUT_BINARY_TAG_GEAR_RATIO_6G` | Gear ratio map (6 gears)     |

### 6.3 Command Bytes

| Command | Usage                                                              |
|---------|--------------------------------------------------------------------|
| 0 or 1  | PID library control (25 bytes of PID config)                       |
| 2       | Button simulation: payload=1(A press), 2(A hold), 3(B press), 4(B hold) |
| 3       | Change serial mode: payload=0(off), 2(text), 3(tunerpro), 4(binary)|
| 4       | EEPROM ops: save=0, calibrate=1, read=2, brightness=3, N75 duty=4 |
| 5       | Debug: RPM override=0, N75 override=3, BT setup=5                 |
| 6       | N75 V2 config: maps, params, serial freq, gear ratios             |

---

## 7. Build Configuration Summary

### Active Defines (VR6 rAtTrax Build)

```c
#define MULTIDISPLAY_V2              // V2 hardware (ATmega2560, expanded I/O)
#define VR6_MOTRONIC                 // VR6 Motronic ECU variant
#define VR6_MOTRONIC_M381            // Motronic M3.8.1 sub-variant
#define GEAR_RECOGNITION             // Speed/gear computation enabled
#define BOOSTN75                     // N75 electronic boost control enabled
#define BW_EFR_SPEEDSENSOR           // BorgWarner EFR turbo speed sensor
#define NUMBER_OF_ATTACHED_TYPE_K 6  // 6 EGT channels wired
#define MAX_ATTACHED_TYPE_K 8        // 8 slots in binary frame
#define SERIALOUT_BINARY_TAG 95      // binary frame = 95 bytes total
#define SERIALFREQ 10                // 10ms → 100 Hz serial rate
#define LAMBDA_WIDEBAND              // Wideband O2 sensor
#define BW_EFR_TIME_2_RPM 40000000   // EFR speed conversion constant
// #define BLUETOOTH_ON_SERIAL2      // COMMENTED OUT in stock code
```

### Bluetooth Configuration

The multidisplay's **HC-06 slave** module is named `mdv2` with PIN `1234`, configured
via AT commands sent on `Serial2` at 115200 baud.

An external **HC-05 master** module (pre-configured with `AT+ROLE=1`, `AT+BIND=<mdv2_addr>`,
`AT+UART=115200,0,0`) auto-connects to the HC-06 and transparently bridges the serial
data to its UART output pins.

---

## 8. Example: Parsing a Binary Frame (Pseudocode)

```python
def parse_multidisplay_binary(raw_bytes):
    """Parse a 95-byte multidisplay binary frame.
    
    Args:
        raw_bytes: 95-byte buffer starting with STX (0x02)
    
    Returns:
        dict with all sensor values in engineering units
    """
    assert len(raw_bytes) == 95
    assert raw_bytes[0] == 0x02   # STX
    assert raw_bytes[1] == 95     # TAG (0x5F)
    assert raw_bytes[94] == 0x03  # ETX
    
    def u16(offset):
        """Read uint16 little-endian at offset."""
        return raw_bytes[offset] | (raw_bytes[offset + 1] << 8)
    
    def s16(offset):
        """Read int16 little-endian at offset."""
        v = u16(offset)
        return v - 0x10000 if v >= 0x8000 else v
    
    def u32(offset):
        """Read uint32 little-endian at offset."""
        return (raw_bytes[offset]
              | (raw_bytes[offset + 1] << 8)
              | (raw_bytes[offset + 2] << 16)
              | (raw_bytes[offset + 3] << 24))
    
    result = {
        'time_ms':       u32(2),
        'rpm':           s16(6),
        'boost_bar':     u16(8)  / 100.0,   # absolute
        'throttle_pct':  raw_bytes[10],
        'lambda':        u16(11) / 100.0,
        'lmm_volts':     u16(13) / 100.0,
        'casetemp_c':    u16(15) / 100.0,
        'egt': [u16(17 + i*2) for i in range(8)],  # 8 channels, °C
        'battery_v':     u16(33) / 100.0,
        'vdo_pres1_bar': s16(35) / 10.0,
        'vdo_pres2_bar': s16(37) / 10.0,
        'vdo_pres3_bar': s16(39) / 10.0,
        'vdo_temp1_c':   s16(41),
        'vdo_temp2_c':   s16(43),
        'vdo_temp3_c':   s16(45),
        'speed_kmh':     u16(47) / 100.0,
        'gear':          raw_bytes[49],
        'n75_duty':      raw_bytes[50],
        'req_boost_bar': u16(51) / 100.0,
        'req_boost_pwm': raw_bytes[53],
        'flags':         raw_bytes[54],
        'pid_enabled':   bool(raw_bytes[54] & 1),
        'pid_aggressive': bool(raw_bytes[54] & 2),
        'efr_reading':   u16(55),
        'knock':         u16(57),
    }
    
    # EFR speed conversion
    if result['efr_reading'] == 0 or result['efr_reading'] == 0xFFFF:
        result['efr_rpm'] = 0
    else:
        result['efr_rpm'] = 40000000 // result['efr_reading']
    
    return result
```

### Frame Receiver State Machine

```python
FRAME_SIZE = 95
TAG = 0x5F  # 95 decimal

def receive_frames(serial_port):
    """Yield complete binary frames from a serial stream."""
    buf = bytearray()
    
    while True:
        byte = serial_port.read(1)
        if not byte:
            continue
        
        b = byte[0]
        
        if len(buf) == 0:
            if b == 0x02:  # STX
                buf.append(b)
            continue
        
        buf.append(b)
        
        if len(buf) == 2:
            if buf[1] != TAG:
                buf.clear()  # TAG mismatch — resync
                continue
        
        if len(buf) == FRAME_SIZE:
            if buf[FRAME_SIZE - 1] == 0x03:  # ETX
                yield bytes(buf)
            buf.clear()
```

---

## 9. Sensor Value Ranges (Typical)

| Sensor          | Min     | Typical     | Max         | Notes                         |
|-----------------|---------|-------------|-------------|-------------------------------|
| RPM             | 0       | 800–7000    | 8500        | Idle ~800, redline varies     |
| Boost (abs.)    | 0.20    | 1.00        | 3.00 Bar    | 1.0 = atmospheric             |
| Throttle        | 0       | 0–100       | 100 %       |                               |
| Lambda          | 0.60    | 0.85–1.05   | 1.50        | 1.0 = stoichiometric          |
| LMM             | 0.0     | 1.0–4.5     | 5.0 V       |                               |
| EGT             | 0       | 200–850     | 1100 °C     | ≥1200 = open/disconnected     |
| Battery         | 10.0    | 13.8        | 15.0 V      |                               |
| VDO Pressure    | 0.0     | 0.0–5.0     | 10.0 Bar    | Oil, fuel, etc.               |
| VDO Temperature | -40     | 80–120      | 150 °C      | Oil, water, etc.              |
| Speed           | 0       | 0–250       | 300 km/h    |                               |
| Gear            | 0       | 1–6         | 6           | 0 = neutral/unknown           |
| EFR Speed       | 0       | 0–120000    | 154000 RPM  | Redline ~148000               |
| Knock           | 0       | 0           | 65535       | Higher = more knock           |

---

## 10. Source Code References

| File                           | Contains                                              |
|--------------------------------|-------------------------------------------------------|
| `MultidisplayController.cpp`   | `serialSend()`, `serialReceive()`, `HeaderPrint()`    |
| `MultidisplayController.h`     | Controller class, `srData` union, TypK state machine  |
| `MultidisplayDefines.h`        | All `SERIALOUT_*` defines, `MAX_ATTACHED_TYPE_K`, build config |
| `MultidisplayDefinesVR6.h`     | VR6-specific: `NUMBER_OF_ATTACHED_TYPE_K=6`, sensors  |
| `MultidisplayDefinesDigifant.h`| Digifant-specific: `NUMBER_OF_ATTACHED_TYPE_K=5`      |
| `SensorData.h`                 | `SensorData` class with all sensor fields             |
| `util.cpp`                     | `float2fixedintb100()` / `fixedintb1002float()`       |
| `RPMBoostController.cpp`       | N75 boost control, `req_Boost`, `boostOutput`         |

---

## 11. Changelog

| Date       | Change                                                 |
|------------|--------------------------------------------------------|
| 2026-03-17 | Initial comprehensive documentation from source code analysis |
