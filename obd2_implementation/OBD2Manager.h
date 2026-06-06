/*
    OBD2Manager.h — Top-level OBD-II Controller

    Orchestrates the ELM327 driver, PID scheduler, data cache, and
    serial output. This is the single entry point that the MD core
    (MultidisplayController) calls from its main loop.

    Copyright 2026 rAtTrax / MultiDisplay Project
    Contributors: uknowmelast, Axiom
    Licensed under GPL v3
*/

#ifndef OBD2MANAGER_H_
#define OBD2MANAGER_H_

#include <Arduino.h>
#include "ELM327.h"
#include "OBD2Data.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * Poll Schedule Entry
 * ═══════════════════════════════════════════════════════════════════════════ */

struct OBD2PollEntry {
    uint8_t  pid;           // PID to poll
    uint8_t  tier;          // 1, 2, or 3 (controls poll frequency)
    uint32_t lastPoll;      // millis() of last poll
    bool     supported;     // ECU reports this PID as supported
    uint8_t  noDataCount;   // Consecutive NO_DATA responses (auto-disable at 3)
};

/* ═══════════════════════════════════════════════════════════════════════════
 * OBD2Manager Class
 * ═══════════════════════════════════════════════════════════════════════════ */

class OBD2Manager {
public:
    OBD2Manager();

    /**
     * Initialize ELM327 and build the poll schedule from enabled PIDs.
     * Call from MultidisplayController::myconstructor().
     * Returns true if ELM327 is responding.
     */
    bool begin();

    /**
     * Poll the next scheduled PID. Call from the main loop.
     * This is non-blocking if no PID is due — returns immediately.
     * If a PID IS due, it sends the request to ELM327 and blocks
     * until the response (or timeout).
     */
    void poll();

    /**
     * Build and send an OBD-II binary frame over serial (and BT).
     * Frame format:
     *   STX(0x02) TAG(OBD2_BINARY_TAG=0x4F) payload ETX(0x03)
     *
     * Payload:
     *   time[4]  pid_count[1]  { pid[1] value_hi[1] value_lo[1] }×N
     *
     * value is fixed-point × 100 (same convention as MD core).
     */
    void serialSendOBD2();

    /* ── Data access for display / integration ────────────────────────── */

    /** Get a decoded PID value. Returns 0 if not available. */
    float getValue(uint8_t pid) const { return _data.getValue(pid); }

    /** Check if a PID has valid data */
    bool isValid(uint8_t pid) const { return _data.isValid(pid); }

    /** Get the data manager (for direct cache access) */
    OBD2Data &getData() { return _data; }

    /** Get the ELM327 driver (for DTC operations, etc.) */
    ELM327 &getELM() { return _elm; }

    /** Is ELM327 initialized and communicating? */
    bool isReady() const { return _elm.isReady(); }

    /** Number of PIDs in the active poll schedule */
    uint8_t getActivePIDCount() const { return _pollCount; }

    /** Number of DTCs currently stored */
    uint8_t getDTCCount() const { return _dtcCount; }

    /** Get DTC at index */
    const OBD2DTC *getDTC(uint8_t idx) const {
        return (idx < _dtcCount) ? &_dtcs[idx] : NULL;
    }

    /** Read DTCs from ECU (call periodically or on demand) */
    void refreshDTCs();

    /** Clear DTCs from ECU */
    bool clearDTCs();

    /* ── DTC / VIN / MIL frame senders for OpenDash ───────────────────── */

    /**
     * Send a DTC frame (TAG 0x44) containing all stored DTC codes.
     * Frame: STX(0x02) TAG(0x44) count(1) {code[5]}×N ETX(0x03)
     * Call after refreshDTCs() or periodically when DTCs are present.
     */
    void sendDTCFrame();

    /**
     * Send a VIN frame (TAG 0x56) containing the vehicle identification number.
     * Frame: STX(0x02) TAG(0x56) vin[17] ETX(0x03)
     * Call once on connect or on demand.
     */
    void sendVINFrame();

    /**
     * Send a MIL status frame (TAG 0x4D) with MIL lamp state and DTC count.
     * Frame: STX(0x02) TAG(0x4D) mil_on(1) dtc_count(1) ETX(0x03)
     */
    void sendMILFrame();

    /**
     * Process incoming commands from OpenDash via Serial2 (Bluetooth).
     * Command format: STX(0x02) cmd_byte(1) ETX(0x03)
     *   0x44 = DTC read request
     *   0x43 = DTC clear request
     *   0x56 = VIN read request
     *   0x4D = MIL status request
     * Call from mainLoop() to check for pending commands.
     */
    void receiveOpenDashCommand();

    /** Get a copy of the VIN (empty if not yet read) */
    const OBD2VIN &getVIN() const { return _vin; }

    /** Read VIN from ECU and cache it */
    bool refreshVIN();

    /** Get last known MIL state */
    bool isMILOn() const { return _milOn; }

    /** Get last known ECU DTC count (from PID 0101) */
    uint8_t getECUDTCCount() const { return _ecuDtcCount; }

private:
    ELM327     _elm;
    OBD2Data   _data;

    /* Poll schedule */
    OBD2PollEntry _polls[OBD2_MAX_TRACKED_PIDS];
    uint8_t       _pollCount;
    uint8_t       _pollIndex;       // Round-robin index

    /* DTC storage */
    OBD2DTC _dtcs[16];
    uint8_t _dtcCount;

    /* VIN cache */
    OBD2VIN  _vin;

    /* MIL status cache */
    bool     _milOn;
    uint8_t  _ecuDtcCount;
    uint32_t _lastMILCheck;

    /* DTC auto-send timer */
    uint32_t _lastDTCFrameSend;

    /* Failure tracking — back off when ELM327 stops responding */
    uint8_t  _consecutiveFails;
    uint32_t _lastPollAttempt;
    uint32_t _lastInitAttempt;

    /* Build the poll schedule from enabled PID defines */
    void buildPollSchedule();

    /* Add a PID to the poll schedule */
    void addPID(uint8_t pid, uint8_t tier);
};

/* Global instance — declared in OBD2Manager.cpp */
extern OBD2Manager obd2;

#endif /* OBD2MANAGER_H_ */
