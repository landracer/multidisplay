/*
    OBD2Manager.cpp — Top-level OBD-II Controller Implementation

    This is the main orchestrator. It:
    1. Initializes the ELM327 on Serial1
    2. Builds a poll schedule from the enabled PID defines
    3. Round-robin polls PIDs from the ELM327 each main loop cycle
    4. Decodes responses and stores them in the data cache
    5. Sends OBD-II binary frames over serial/BT for OpenDash

    Copyright 2026 rAtTrax / MultiDisplay Project
    Contributors: uknowmelast, Axiom
    Licensed under GPL v3
*/

#include "OBD2Manager.h"
#include "../MultidisplayDefines.h"

#ifdef OBD2_ELM327

/* ── Global Instance ──────────────────────────────────────────────────── */
OBD2Manager obd2;

/* ── Constructor ──────────────────────────────────────────────────────── */

OBD2Manager::OBD2Manager()
    : _pollCount(0)
    , _pollIndex(0)
    , _dtcCount(0)
    , _milOn(false)
    , _ecuDtcCount(0)
    , _lastMILCheck(0)
    , _lastDTCFrameSend(0)
    , _consecutiveFails(0)
    , _lastPollAttempt(0)
    , _lastInitAttempt(0)
{
    memset(_polls, 0, sizeof(_polls));
    memset(_dtcs, 0, sizeof(_dtcs));
    memset(&_vin, 0, sizeof(_vin));
}

/* ── begin() ──────────────────────────────────────────────────────────── */

bool OBD2Manager::begin()
{
    // Initialize ELM327 on the configured serial port
    bool ok = _elm.begin(OBD2_SERIAL, OBD2_ELM327_BAUD, OBD2_ELM_PROTOCOL);

    if (ok) {
        buildPollSchedule();

        // Optionally check which PIDs the ECU actually supports
        uint32_t supported = 0;
        if (_elm.getSupportedPIDs_00_20(&supported)) {
            // Mark unsupported PIDs
            for (uint8_t i = 0; i < _pollCount; i++) {
                uint8_t pid = _polls[i].pid;
                if (pid >= 0x01 && pid <= 0x20) {
                    // Bit 31 = PID 0x01, bit 0 = PID 0x20
                    uint8_t bitPos = 0x20 - pid;
                    _polls[i].supported = (supported >> bitPos) & 1;
                } else {
                    // PIDs > 0x20: assume supported (could query 0x20, 0x40, etc.)
                    _polls[i].supported = true;
                }
            }
        } else {
            // Can't query support — assume all enabled PIDs are supported
            for (uint8_t i = 0; i < _pollCount; i++) {
                _polls[i].supported = true;
            }
        }

        Serial.print(F("OBD2: ELM327 ready, "));
        Serial.print(_pollCount);
        Serial.println(F(" PIDs scheduled"));
    } else {
        Serial.println(F("OBD2: ELM327 init FAILED"));
    }

    return ok;
}

/* ── poll() ───────────────────────────────────────────────────────────── */

void OBD2Manager::poll()
{
    // If ELM327 init failed, try to re-initialize every 30 seconds
    if (!_elm.isReady()) {
        uint32_t now = millis();
        if ((now - _lastInitAttempt) >= 30000UL) {
            Serial.println(F("[OBD2] Re-attempting ELM327 init..."));
            if (begin()) {
                Serial.println(F("[OBD2] Re-init succeeded!"));
            }
            _lastInitAttempt = millis();  // Set AFTER begin() completes
        }
        return;
    }

    if (_pollCount == 0) return;

    // MIL/DTC status is now handled as a regular data point (PID 0x01)
    // in the poll schedule — see buildPollSchedule() tier 3.
    // NO separate periodic getMILStatus() call. (Rule 7: no polling)

    // Back off when ELM327 fails repeatedly — don't block main loop
    // After 3 consecutive failures, wait 5 seconds between attempts
    // After 10 failures, wait 30 seconds
    if (_consecutiveFails >= 3) {
        uint32_t backoff = (_consecutiveFails >= 10) ? 30000UL : 5000UL;
        if ((millis() - _lastPollAttempt) < backoff) return;
    }

    // Find the next PID that is due for polling
    for (uint8_t attempts = 0; attempts < _pollCount; attempts++) {
        _pollIndex = (_pollIndex + 1) % _pollCount;
        OBD2PollEntry &entry = _polls[_pollIndex];

        if (!entry.supported) continue;

        // Calculate poll interval based on tier
        uint32_t interval = OBD2_POLL_BASE_MS;
        switch (entry.tier) {
            case 1: interval *= OBD2_TIER1_MULT; break;
            case 2: interval *= OBD2_TIER2_MULT; break;
            case 3: interval *= OBD2_TIER3_MULT; break;
        }

        uint32_t now = millis();
        if ((now - entry.lastPoll) < interval) continue;

        // This PID is due — send the request
        _lastPollAttempt = now;
        OBD2PIDResponse resp;
        if (_elm.requestPID(entry.pid, &resp)) {
            // PID 0x01 (Monitor Status): extract MIL + DTC count from byte A
            // Byte A: bit7 = MIL on/off, bits 6-0 = DTC count
            if (entry.pid == PID_MONITOR_STATUS && resp.dataLen >= 1) {
                _milOn = (resp.data[0] & 0x80) != 0;
                _ecuDtcCount = resp.data[0] & 0x7F;
                if (_ecuDtcCount > 0 && _dtcCount == 0) {
                    refreshDTCs();
                }
            } else {
                _data.decodePID(&resp);
            }
            entry.lastPoll = now;
            entry.noDataCount = 0;
            _consecutiveFails = 0;
        } else {
            _consecutiveFails++;
            // Track per-PID NO_DATA — auto-disable after 3
            if (_elm.lastStatus() == ELM_NO_DATA) {
                entry.noDataCount++;
                if (entry.noDataCount >= 3) {
                    entry.supported = false;
                    Serial.print(F("[OBD2] PID 0x"));
                    Serial.print(entry.pid, HEX);
                    Serial.println(F(" disabled (NO DATA)"));
                }
            }
        }

        // Only poll ONE PID per call to keep the main loop responsive
        return;
    }
}

/* ── serialSendOBD2() ─────────────────────────────────────────────────── */

void OBD2Manager::serialSendOBD2()
{
    if (_pollCount == 0) return;

    // Count how many valid PIDs we have
    uint8_t validCount = 0;
    for (uint8_t i = 0; i < _pollCount; i++) {
        if (_data.isValid(_polls[i].pid)) validCount++;
    }
    if (validCount == 0) return;

    // Build binary frame: STX TAG time[4] pid_count[1] {pid[1] val_hi[1] val_lo[1]}×N ETX
    uint8_t buf[OBD2_MAX_FRAME_SIZE];
    uint8_t idx = 0;

    // STX
    buf[idx++] = 0x02;

    // TAG
    buf[idx++] = OBD2_BINARY_TAG;

    // Time (4 bytes, little-endian)
    unsigned long t = millis();
    buf[idx++] = (t) & 0xFF;
    buf[idx++] = (t >> 8) & 0xFF;
    buf[idx++] = (t >> 16) & 0xFF;
    buf[idx++] = (t >> 24) & 0xFF;

    // PID count
    buf[idx++] = validCount;

    // PID data entries
    for (uint8_t i = 0; i < _pollCount && idx + 3 < OBD2_MAX_FRAME_SIZE - 1; i++) {
        uint8_t pid = _polls[i].pid;
        if (!_data.isValid(pid)) continue;

        float val = _data.getValue(pid);

        // Per-PID scaling to avoid int16 overflow.
        // RPM max 16383 × 100 = overflow, use ×1.
        // MAF max 655.35 × 100 = overflow, use ×10.
        // Fuel Pressure max 765 × 100 = overflow, use ×10.
        float scale = 100.0f;
        if (pid == 0x0C)      scale = 1.0f;   // PID_RPM
        else if (pid == 0x10) scale = 10.0f;   // PID_MAF_RATE
        else if (pid == 0x0A) scale = 10.0f;   // PID_FUEL_PRESSURE
        int16_t fixedVal = (int16_t)(val * scale);

        buf[idx++] = pid;
        buf[idx++] = (fixedVal) & 0xFF;        // low byte
        buf[idx++] = (fixedVal >> 8) & 0xFF;    // high byte
    }

    // ETX
    buf[idx++] = 0x03;

    // Send on Serial (USB) and Serial2 (Bluetooth)
    Serial.write(buf, idx);
#if defined(MULTIDISPLAY_V2) && defined(BLUETOOTH_ON_SERIAL2)
    Serial2.write(buf, idx);
#endif
}

/* ── refreshDTCs() ────────────────────────────────────────────────────── */

void OBD2Manager::refreshDTCs()
{
    if (!_elm.isReady()) return;
    _dtcCount = _elm.readDTCs(_dtcs, 16);
}

/* ── clearDTCs() ──────────────────────────────────────────────────────── */

bool OBD2Manager::clearDTCs()
{
    if (!_elm.isReady()) return false;
    bool ok = _elm.clearDTCs();
    if (ok) {
        _dtcCount = 0;
        _milOn = false;
        _ecuDtcCount = 0;
    }
    return ok;
}

/* ── refreshVIN() ─────────────────────────────────────────────────────── */

bool OBD2Manager::refreshVIN()
{
    if (!_elm.isReady()) return false;
    return _elm.getVIN(&_vin);
}

/* ── sendDTCFrame() — TAG 0x44 ────────────────────────────────────────── */

void OBD2Manager::sendDTCFrame()
{
    // Frame: STX TAG count {code[5]}×N ETX
    // Max: 3 + 16*5 + 1 = 84 bytes
    uint8_t buf[96];
    uint8_t idx = 0;

    buf[idx++] = 0x02;     // STX
    buf[idx++] = 0x44;     // TAG = 'D' for DTC
    buf[idx++] = _dtcCount;

    for (uint8_t i = 0; i < _dtcCount && i < 16; i++) {
        memcpy(&buf[idx], _dtcs[i].code, 5);
        idx += 5;
    }

    buf[idx++] = 0x03;     // ETX

    MD_SERIAL_WRITE(buf, idx);
}

/* ── sendVINFrame() — TAG 0x56 ────────────────────────────────────────── */

void OBD2Manager::sendVINFrame()
{
    // Read VIN if not cached
    if (!_vin.valid) {
        refreshVIN();
    }

    // Frame: STX TAG vin[17] valid ETX
    uint8_t buf[22];
    uint8_t idx = 0;

    buf[idx++] = 0x02;     // STX
    buf[idx++] = 0x56;     // TAG = 'V' for VIN
    memcpy(&buf[idx], _vin.vin, 17);
    idx += 17;
    buf[idx++] = _vin.valid ? 1 : 0;
    buf[idx++] = 0x03;     // ETX

    MD_SERIAL_WRITE(buf, idx);
}

/* ── sendMILFrame() — TAG 0x4D ────────────────────────────────────────── */

void OBD2Manager::sendMILFrame()
{
    // Refresh MIL from ECU
    bool mil = false;
    int8_t cnt = _elm.getMILStatus(&mil);
    if (cnt >= 0) {
        _milOn = mil;
        _ecuDtcCount = (uint8_t)cnt;
    }

    // Frame: STX TAG mil_on dtc_count ETX
    uint8_t buf[5];
    buf[0] = 0x02;         // STX
    buf[1] = 0x4D;         // TAG = 'M' for MIL
    buf[2] = _milOn ? 1 : 0;
    buf[3] = _ecuDtcCount;
    buf[4] = 0x03;         // ETX

    MD_SERIAL_WRITE(buf, 5);
}

/* ── receiveOpenDashCommand() ─────────────────────────────────────────── */

void OBD2Manager::receiveOpenDashCommand()
{
#if defined(MULTIDISPLAY_V2) && defined(BLUETOOTH_ON_SERIAL2)
    if (!Serial2.available()) return;

    // Look for STX
    if (Serial2.peek() != 0x02) {
        Serial2.read();  // discard non-STX byte
        return;
    }
    Serial2.read();  // consume STX

    // Read command byte with short timeout
    unsigned long timeout = millis() + 200;
    while (!Serial2.available() && millis() < timeout) {}
    if (!Serial2.available()) return;

    uint8_t cmd = Serial2.read();

    // Drain remaining bytes until ETX
    timeout = millis() + 100;
    while (millis() < timeout) {
        if (Serial2.available()) {
            if (Serial2.read() == 0x03) break;
        }
    }

    // Process command
    switch (cmd) {
        case 0x44:  // DTC read request
            Serial.println(F("[OD] DTC read request"));
            refreshDTCs();
            sendDTCFrame();
            break;
        case 0x43:  // DTC clear request
            Serial.println(F("[OD] DTC clear request"));
            if (clearDTCs()) {
                sendDTCFrame();  // Confirm with empty DTC frame
            }
            break;
        case 0x56:  // VIN read request
            Serial.println(F("[OD] VIN read request"));
            sendVINFrame();
            break;
        case 0x4D:  // MIL status request
            Serial.println(F("[OD] MIL status request"));
            sendMILFrame();
            break;
    }
#endif
}

/* ══════════════════════════════════════════════════════════════════════════
 * Private: Build Poll Schedule
 * ══════════════════════════════════════════════════════════════════════════ */

void OBD2Manager::addPID(uint8_t pid, uint8_t tier)
{
    if (_pollCount >= OBD2_MAX_TRACKED_PIDS) return;
    _polls[_pollCount].pid = pid;
    _polls[_pollCount].tier = tier;
    _polls[_pollCount].lastPoll = 0;
    _polls[_pollCount].supported = true; // Assume until proven otherwise
    _pollCount++;
}

void OBD2Manager::buildPollSchedule()
{
    _pollCount = 0;

    /* ── Tier 1: Essential (fastest polling) ───────────────────────────── */
#ifdef OBD2_ENABLE_PID_RPM
    addPID(PID_RPM, 1);
#endif
#ifdef OBD2_ENABLE_PID_SPEED
    addPID(PID_SPEED, 1);
#endif
#ifdef OBD2_ENABLE_PID_COOLANT_TEMP
    addPID(PID_COOLANT_TEMP, 1);
#endif
#ifdef OBD2_ENABLE_PID_ENGINE_LOAD
    addPID(PID_ENGINE_LOAD, 1);
#endif
#ifdef OBD2_ENABLE_PID_INTAKE_MAP
    addPID(PID_INTAKE_MAP, 1);
#endif
#ifdef OBD2_ENABLE_PID_THROTTLE_POS
    addPID(PID_THROTTLE_POS, 1);
#endif

    /* ── Tier 2: Standard (medium polling) ─────────────────────────────── */
#ifdef OBD2_ENABLE_PID_INTAKE_TEMP
    addPID(PID_INTAKE_TEMP, 2);
#endif
#ifdef OBD2_ENABLE_PID_MAF_RATE
    addPID(PID_MAF_RATE, 2);
#endif
#ifdef OBD2_ENABLE_PID_TIMING_ADV
    addPID(PID_TIMING_ADVANCE, 2);
#endif
#ifdef OBD2_ENABLE_PID_STFT_B1
    addPID(PID_STFT_BANK1, 2);
#endif
#ifdef OBD2_ENABLE_PID_LTFT_B1
    addPID(PID_LTFT_BANK1, 2);
#endif
#ifdef OBD2_ENABLE_PID_FUEL_PRESSURE
    addPID(PID_FUEL_PRESSURE, 2);
#endif
#ifdef OBD2_ENABLE_PID_BARO_PRESSURE
    addPID(PID_BARO_PRESSURE, 2);
#endif

    /* ── Tier 3: Extended (slow polling) ───────────────────────────────── */
#ifdef OBD2_ENABLE_PID_MONITOR_STATUS
    addPID(PID_MONITOR_STATUS, 3); // MIL + DTC count — data point, not separate poll
#endif
#ifdef OBD2_ENABLE_PID_RUNTIME
    addPID(PID_RUNTIME, 3);
#endif
#ifdef OBD2_ENABLE_PID_FUEL_LEVEL
    addPID(PID_FUEL_LEVEL, 3);
#endif
#ifdef OBD2_ENABLE_PID_AMBIENT_TEMP
    addPID(PID_AMBIENT_TEMP, 3);
#endif
#ifdef OBD2_ENABLE_PID_OIL_TEMP
    addPID(PID_OIL_TEMP, 3);
#endif
#ifdef OBD2_ENABLE_PID_CTRL_VOLTAGE
    addPID(PID_CTRL_MODULE_VOLTAGE, 3);
#endif
#ifdef OBD2_ENABLE_PID_ETHANOL_PCT
    addPID(PID_ETHANOL_PCT, 3);
#endif
#ifdef OBD2_ENABLE_PID_TURBO_RPM
    addPID(PID_TURBO_RPM, 3);
#endif
#ifdef OBD2_ENABLE_PID_BOOST_PRESS
    addPID(PID_BOOST_PRESSURE, 3);
#endif
#ifdef OBD2_ENABLE_PID_EGT_B1
    addPID(PID_EGT_BANK1, 3);
#endif
#ifdef OBD2_ENABLE_PID_ODOMETER
    addPID(PID_ODOMETER, 3);
#endif
}

#endif /* OBD2_ELM327 */
