/*
    ELM327.h — ELM327 IC Communication Driver
    Handles AT commands, PID requests, and DTC operations
    over a HardwareSerial UART connection.

    Reference: ELM327DSL Datasheet
    https://www.elmelectronics.com/wp-content/uploads/2020/05/ELM327DSL.pdf

    Copyright 2026 rAtTrax / MultiDisplay Project
    Contributors: uknowmelast, Axiom
    Licensed under GPL v3
*/

#ifndef ELM327_H_
#define ELM327_H_

#include <Arduino.h>
#include <stdint.h>

/* ── Response buffer size ─────────────────────────────────────────────── */
#define ELM_RESP_BUF_SIZE   128

/* ── ELM327 status codes ─────────────────────────────────────────────── */
enum ELM327Status : uint8_t {
    ELM_OK          = 0,
    ELM_NO_DATA     = 1,
    ELM_ERROR       = 2,
    ELM_TIMEOUT     = 3,
    ELM_BUS_INIT    = 4,    // "BUS INIT: ...OK" during first request
    ELM_NOT_INIT    = 5,
    ELM_PARSE_FAIL  = 6,    // ELM responded OK but couldn't parse hex data
};

/* ── OBD-II PID response structure ────────────────────────────────────── */
struct OBD2PIDResponse {
    uint8_t  service;       // Response service (0x41 for mode 01)
    uint8_t  pid;           // PID number
    uint8_t  data[4];       // Up to 4 data bytes (A, B, C, D)
    uint8_t  dataLen;       // Number of valid data bytes
    float    value;         // Decoded value (after formula applied)
};

/* ── DTC structure ────────────────────────────────────────────────────── */
struct OBD2DTC {
    char code[6];           // e.g., "P0300" null-terminated
};

/* ── VIN structure ─────────────────────────────────────────────────────── */
struct OBD2VIN {
    char vin[18];           // 17 chars + null-terminator
    bool valid;             // true if VIN was successfully read
};

/* ════════════════════════════════════════════════════════════════════════
 * ELM327 Class
 * ════════════════════════════════════════════════════════════════════════ */

class ELM327 {
public:
    ELM327();

    /**
     * Initialize the ELM327 on the given serial port.
     * Sends ATZ, ATE0, ATL0, ATS0, ATSP<protocol>.
     * Returns true if ELM327 responds to ATZ with "ELM327" banner.
     */
    bool begin(HardwareSerial &serial, uint32_t baud, uint8_t protocol);

    /** Shut down — flushes serial */
    void end();

    /** True after successful begin() */
    bool isReady() const { return _ready; }

    /** Last ELM327 status */
    ELM327Status lastStatus() const { return _lastStatus; }

    /** Actual baud rate used (after auto-detection) */
    uint32_t activeBaud() const { return _activeBaud; }

    /* ── Single PID request ───────────────────────────────────────────── */

    /**
     * Request a Mode 01 PID. Sends "01XX\r" and parses "41 XX AA BB...".
     * Raw data bytes are stored in resp->data[], count in resp->dataLen.
     * resp->value is NOT decoded here — caller uses the formula table.
     * Returns true on successful response.
     */
    bool requestPID(uint8_t pid, OBD2PIDResponse *resp);

    /**
     * Request a PID from any service (mode). Sends "SSPP\r".
     */
    bool requestServicePID(uint8_t service, uint8_t pid, OBD2PIDResponse *resp);

    /* ── DTC operations ───────────────────────────────────────────────── */

    /**
     * Read stored DTCs (Service 03). Returns count stored in dtcs[].
     * max_count limits how many are read.
     */
    uint8_t readDTCs(OBD2DTC *dtcs, uint8_t maxCount);

    /**
     * Clear DTCs and freeze frames (Service 04).
     */
    bool clearDTCs();

    /* ── VIN query ─────────────────────────────────────────────────────── */

    /**
     * Read Vehicle Identification Number (Service 09, PID 02).
     * Returns true if VIN was successfully read and stored in vin->vin[].
     */
    bool getVIN(OBD2VIN *vin);

    /**
     * Read MIL status and DTC count (Service 01, PID 01).
     * Returns the DTC count from the ECU, or -1 on error.
     * mil_on is set to true if the MIL lamp is commanded on.
     */
    int8_t getMILStatus(bool *mil_on);

    /* ── Supported PID query ──────────────────────────────────────────── */

    /**
     * Query supported PIDs 0x00-0x20 and store the 32-bit bitmap.
     * Each bit = 1 PID (bit 31 = PID 0x01, bit 0 = PID 0x20).
     */
    bool getSupportedPIDs_00_20(uint32_t *bitmap);

    /* ── Raw AT command ───────────────────────────────────────────────── */

    /**
     * Send an AT command and read response into buf.
     * Returns number of bytes read, 0 on timeout.
     */
    uint8_t sendATCommand(const char *cmd, char *buf, uint8_t bufLen);

private:
    HardwareSerial *_serial;
    bool            _ready;
    ELM327Status    _lastStatus;
    uint32_t        _timeout;
    uint8_t         _retries;
    uint32_t        _activeBaud;

    /* Send a command string (with \r terminator) and read the response.
     * Strips echo, whitespace. Returns bytes read into buf.              */
    uint8_t sendCommand(const char *cmd, char *buf, uint8_t bufLen);

    /* Parse hex response "41 0C 1A F8" into OBD2PIDResponse */
    bool parseOBDResponse(const char *buf, uint8_t bufLen, OBD2PIDResponse *resp);

    /* Try parsing OBD response starting at pointer p */
    bool tryParseAt(const char *p, const char *end, OBD2PIDResponse *resp);

    /* Convert two hex ASCII chars to a byte: "1A" -> 0x1A */
    static uint8_t hexToByte(char hi, char lo);

    /* Check if response contains error strings */
    ELM327Status checkResponse(const char *buf);
};

#endif /* ELM327_H_ */
