/*
    MDLoggerClient.cpp — I2C master driver for MDLogger (328P) slave

    See MDLoggerClient.h for the wire protocol description.
    See MDLOGGER_PROTOCOL.md for the full pairing/bring-up guide.

    NOTE: This file uses the multidisplay project's CUSTOM Wire library
    (multidisplay/libs/Wire.{h,cpp}), which exposes send()/receive()
    instead of the modern write()/read() names.

    Copyright 2026 rAtTrax / MultiDisplay Project
    Licensed under GPL v3
*/

#include "MDLoggerClient.h"

/* ═════════════════════════════════════════════════════════════════════════════
 * Construction
 * ═════════════════════════════════════════════════════════════════════════════ */

MDLoggerClient::MDLoggerClient() : _ready(false) {
    memset(&_status, 0, sizeof(_status));
    memset(&_frame, 0, sizeof(_frame));
}

/* ═════════════════════════════════════════════════════════════════════════════
 * begin() — Initialize Wire as master and probe the slave once.
 *
 * If the slave isn't powered yet (slave boots later), this returns with
 * _ready=false. The next pollStatus() call (driven by the main loop)
 * will re-arm the link automatically.
 * ═════════════════════════════════════════════════════════════════════════════ */

void MDLoggerClient::begin() {
    Wire.begin();   // master mode
    _ready = false;
    pollStatus();   // best-effort initial probe; ignore result
}

/* ═════════════════════════════════════════════════════════════════════════════
 * packFrame() — Stage the next outbound DATA_FRAME.
 *
 * Inputs are pre-scaled to match the SERIALOUT_BINARY layout (mdData) and
 * the GPS unit conventions (lat/lon × 1e6, speed × 100, course × 100).
 * Cheap memcpy — safe to call every serial cycle.
 * ═════════════════════════════════════════════════════════════════════════════ */

void MDLoggerClient::packFrame(uint32_t ts, const uint8_t *mdData,
                               int32_t lat, int32_t lon, uint16_t speed,
                               uint16_t course, uint8_t sats, uint8_t fix) {
    _frame.timestamp   = ts;
    memcpy(_frame.mdData, mdData, MDLOGGER_MD_DATA_SIZE);
    _frame.latitude    = lat;
    _frame.longitude   = lon;
    _frame.speedKmh100 = speed;
    _frame.course100   = course;
    _frame.satellites  = sats;
    _frame.fix         = fix;
}

/* ═════════════════════════════════════════════════════════════════════════════
 * sendData() — Transmit the staged frame.
 *
 * Wire format on the bus:
 *   [START][addr|W][0x01][74-byte frame][STOP]   — 75 payload bytes
 *
 * Requires TWI_BUFFER_LENGTH ≥ 76 in the custom Wire library
 * (we bumped it to 96 in libs/twi.h and libs/Wire.h).
 *
 * On NACK we clear _ready so the next pollStatus() can re-detect the slave.
 * ═════════════════════════════════════════════════════════════════════════════ */

bool MDLoggerClient::sendData() {
    if (!_ready) return false;

    Wire.beginTransmission(MDLOGGER_I2C_ADDR);
    Wire.send(MDLOGGER_CMD_DATA_FRAME);
    Wire.send((uint8_t*)&_frame, sizeof(_frame));
    uint8_t result = Wire.endTransmission();

    if (result != 0) {
        // Bus error / NACK — slave fell off the bus. Force a reconnect on
        // the next pollStatus() cycle.
        _ready = false;
        return false;
    }
    return true;
}

/* ═════════════════════════════════════════════════════════════════════════════
 * pollStatus() — Send STATUS_REQUEST, read 5-byte response.
 *
 * Wire sequence:
 *   1. [START][addr|W][0x03][STOP]                 — set up the request
 *   2. [START][addr|R][b0][b1][b2][b3][b4][STOP]   — read 5-byte response
 *
 * Response bytes:
 *   b0 = MDLoggerStatus.asByte (sdReady|bufferFull|loggingActive|gpsHasFix|...)
 *   b1 = GPS satellite count
 *   b2 = GPS fix flag (0/1)
 *   b3 = GPS configured flag (0/1)
 *   b4 = data fields valid flag (0/1)
 *
 * Side effect: sets _ready=true on success (this is the reconnect path).
 * ═════════════════════════════════════════════════════════════════════════════ */

bool MDLoggerClient::pollStatus() {
    Wire.beginTransmission(MDLOGGER_I2C_ADDR);
    Wire.send(MDLOGGER_CMD_STATUS_REQUEST);
    if (Wire.endTransmission() != 0) {
        _ready = false;  // slave not on bus
        return false;
    }

    Wire.requestFrom((uint8_t)MDLOGGER_I2C_ADDR, (uint8_t)MDLOGGER_STATUS_SIZE);
    if (Wire.available() < MDLOGGER_STATUS_SIZE) {
        _ready = false;  // partial / no response
        return false;
    }

    uint8_t raw[MDLOGGER_STATUS_SIZE];
    for (uint8_t i = 0; i < MDLOGGER_STATUS_SIZE; i++) {
        raw[i] = Wire.receive();
    }

    _status.asByte = raw[0];   // bit-packed status flags
    // raw[1..4] available via getStatus() bits if we extend the union later.
    _ready = true;             // reconnect / link-up
    return true;
}

/* Global instance referenced by MultidisplayController. */
MDLoggerClient mdLoggerClient;
