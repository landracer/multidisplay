/*
    OBD2Data.h — OBD-II PID Decoder and Data Manager

    Decodes raw OBD-II PID response bytes into float values using the
    standard SAE J1979 formulas. Manages a cache of the latest PID
    values with timestamps for freshness checking.

    Copyright 2026 rAtTrax / MultiDisplay Project
    Contributors: uknowmelast, Axiom
    Licensed under GPL v3
*/

#ifndef OBD2DATA_H_
#define OBD2DATA_H_

#include <Arduino.h>
#include <stdint.h>
#include "../MultidisplayDefines.h"
#include "ELM327.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * Standard OBD-II PID Definitions (Mode 01)
 * ═══════════════════════════════════════════════════════════════════════════ */

// PID hex values
#define PID_SUPPORTED_00_20         0x00
#define PID_MONITOR_STATUS          0x01  // Monitor Status Since DTCs Cleared (MIL + DTC count)
#define PID_ENGINE_LOAD             0x04
#define PID_COOLANT_TEMP            0x05
#define PID_STFT_BANK1              0x06
#define PID_LTFT_BANK1              0x07
#define PID_STFT_BANK2              0x08
#define PID_LTFT_BANK2              0x09
#define PID_FUEL_PRESSURE           0x0A
#define PID_INTAKE_MAP              0x0B
#define PID_RPM                     0x0C
#define PID_SPEED                   0x0D
#define PID_TIMING_ADVANCE          0x0E
#define PID_INTAKE_TEMP             0x0F
#define PID_MAF_RATE                0x10
#define PID_THROTTLE_POS            0x11
#define PID_O2_VOLTAGE_B1S1         0x14
#define PID_OBD_STANDARD            0x1C
#define PID_RUNTIME                 0x1F
#define PID_SUPPORTED_20_40         0x20
#define PID_FUEL_RAIL_PRESSURE      0x23
#define PID_EGR_COMMANDED           0x2C
#define PID_FUEL_LEVEL              0x2F
#define PID_BARO_PRESSURE           0x33
#define PID_CATALYST_TEMP_B1S1      0x3C
#define PID_SUPPORTED_40_60         0x40
#define PID_CTRL_MODULE_VOLTAGE     0x42
#define PID_AMBIENT_TEMP            0x46
#define PID_THROTTLE_POS_B          0x47
#define PID_ACCEL_PEDAL_D           0x49
#define PID_ACCEL_PEDAL_E           0x4A
#define PID_FUEL_TYPE               0x51
#define PID_ETHANOL_PCT             0x52
#define PID_OIL_TEMP                0x5C
#define PID_FUEL_INJECTION_TIMING   0x5D
#define PID_FUEL_RATE               0x5E
#define PID_SUPPORTED_60_80         0x60
#define PID_DRIVER_TORQUE           0x61
#define PID_ENGINE_TORQUE           0x62
#define PID_REFERENCE_TORQUE        0x63
#define PID_TURBO_INLET_PRESS       0x6F
#define PID_BOOST_PRESSURE          0x70
#define PID_TURBO_RPM               0x74
#define PID_EGT_BANK1               0x78
#define PID_ODOMETER                0xA6

/* ═══════════════════════════════════════════════════════════════════════════
 * PID Formula Entry
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Decode function pointer: takes raw data bytes, returns float value */
typedef float (*pid_decode_fn)(const uint8_t *data);

/** PID descriptor stored in PROGMEM */
struct PIDDescriptor {
    uint8_t      pid;        // PID number
    uint8_t      dataLen;    // Expected data bytes in response
    pid_decode_fn decode;    // Formula function
    const char   *unit;      // Unit string (PROGMEM)
    const char   *name;      // Human-readable name (PROGMEM)
};

/* ═══════════════════════════════════════════════════════════════════════════
 * PID Cache Entry (runtime data store)
 * ═══════════════════════════════════════════════════════════════════════════ */

struct PIDCacheEntry {
    uint8_t  pid;
    float    value;
    uint32_t timestamp;     // millis() when last updated
    bool     valid;         // Has been received at least once
};

/* ═══════════════════════════════════════════════════════════════════════════
 * OBD2Data Class — PID decoder + data cache
 * ═══════════════════════════════════════════════════════════════════════════ */

class OBD2Data {
public:
    OBD2Data();

    /** Decode a raw PID response into a float value.
     *  Looks up the PID in the formula table and applies the conversion.
     *  Returns true if PID was found and decoded. */
    bool decodePID(OBD2PIDResponse *resp);

    /** Store a decoded value in the cache */
    void updateCache(uint8_t pid, float value);

    /** Get the cached value for a PID. Returns 0 and valid=false if not cached. */
    float getValue(uint8_t pid) const;

    /** Check if a PID has a valid cached value */
    bool isValid(uint8_t pid) const;

    /** Get the timestamp of the last update for a PID */
    uint32_t getTimestamp(uint8_t pid) const;

    /** Get the unit string for a PID (from PROGMEM) */
    const char *getUnit(uint8_t pid) const;

    /** Get the human-readable name for a PID */
    const char *getName(uint8_t pid) const;

    /** Get the PID descriptor (formula table entry) */
    static const PIDDescriptor *getDescriptor(uint8_t pid);

    /** Get the total number of known PID formulas */
    static uint8_t getFormulaCount();

private:
    PIDCacheEntry _cache[OBD2_MAX_TRACKED_PIDS];
    uint8_t       _cacheCount;

    int8_t findCacheIndex(uint8_t pid) const;
    int8_t allocCacheEntry(uint8_t pid);
};

/* ═══════════════════════════════════════════════════════════════════════════
 * Convenience: get a PID value by name (for display code / OpenDash)
 *
 * Usage in app code:
 *    float rpm = obd2Data.getValue(PID_RPM);
 *    float coolant = obd2Data.getValue(PID_COOLANT_TEMP);
 * ═══════════════════════════════════════════════════════════════════════════ */

#endif /* OBD2DATA_H_ */
