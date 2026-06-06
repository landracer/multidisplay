/*
    OBD2Data.cpp — PID Decoder Implementation with SAE J1979 Formulas

    Every standard OBD-II PID has a specific formula to convert raw bytes
    into engineering values. This file contains all the decode functions
    and the lookup table that maps PIDs to their formulas.

    Copyright 2026 rAtTrax / MultiDisplay Project
    Contributors: uknowmelast, Axiom
    Licensed under GPL v3
*/

#include "OBD2Data.h"

#ifdef OBD2_ELM327

/* ═══════════════════════════════════════════════════════════════════════════
 * PID Decode Functions (SAE J1979 standard formulas)
 *
 * Each function takes a pointer to the raw data bytes (A, B, C, D)
 * and returns the decoded float value.
 * ═══════════════════════════════════════════════════════════════════════════ */

// PID 0x04: Engine load = A × 100 / 255 (%)
static float dec_engine_load(const uint8_t *d) { return d[0] * 100.0f / 255.0f; }

// PID 0x05: Coolant temp = A - 40 (°C)
static float dec_coolant_temp(const uint8_t *d) { return (float)d[0] - 40.0f; }

// PID 0x06-0x09: Fuel trim = (A - 128) × 100 / 128 (%)
static float dec_fuel_trim(const uint8_t *d) { return ((float)d[0] - 128.0f) * 100.0f / 128.0f; }

// PID 0x0A: Fuel pressure = A × 3 (kPa)
static float dec_fuel_pressure(const uint8_t *d) { return d[0] * 3.0f; }

// PID 0x0B: Intake MAP = A (kPa)
static float dec_intake_map(const uint8_t *d) { return (float)d[0]; }

// PID 0x0C: RPM = (A × 256 + B) / 4 (rpm)
static float dec_rpm(const uint8_t *d) { return ((uint16_t)d[0] * 256 + d[1]) / 4.0f; }

// PID 0x0D: Speed = A (km/h)
static float dec_speed(const uint8_t *d) { return (float)d[0]; }

// PID 0x0E: Timing advance = A / 2 - 64 (degrees BTDC)
static float dec_timing_advance(const uint8_t *d) { return d[0] / 2.0f - 64.0f; }

// PID 0x0F: Intake temp = A - 40 (°C)
static float dec_intake_temp(const uint8_t *d) { return (float)d[0] - 40.0f; }

// PID 0x10: MAF rate = (A × 256 + B) / 100 (g/s)
static float dec_maf_rate(const uint8_t *d) { return ((uint16_t)d[0] * 256 + d[1]) / 100.0f; }

// PID 0x11: Throttle = A × 100 / 255 (%)
static float dec_throttle(const uint8_t *d) { return d[0] * 100.0f / 255.0f; }

// PID 0x14: O2 voltage B1S1 = A / 200 (V)
static float dec_o2_voltage(const uint8_t *d) { return d[0] / 200.0f; }

// PID 0x1C: OBD standard = A (enum)
static float dec_obd_standard(const uint8_t *d) { return (float)d[0]; }

// PID 0x1F: Runtime = A × 256 + B (seconds)
static float dec_runtime(const uint8_t *d) { return (float)((uint16_t)d[0] * 256 + d[1]); }

// PID 0x23: Fuel rail gauge pressure = (A × 256 + B) × 10 (kPa)
static float dec_fuel_rail_pressure(const uint8_t *d) { return ((uint16_t)d[0] * 256 + d[1]) * 10.0f; }

// PID 0x2C: Commanded EGR = A × 100 / 255 (%)
static float dec_egr(const uint8_t *d) { return d[0] * 100.0f / 255.0f; }

// PID 0x2F: Fuel level = A × 100 / 255 (%)
static float dec_fuel_level(const uint8_t *d) { return d[0] * 100.0f / 255.0f; }

// PID 0x33: Barometric pressure = A (kPa)
static float dec_baro(const uint8_t *d) { return (float)d[0]; }

// PID 0x3C: Catalyst temp B1S1 = (A × 256 + B) / 10 - 40 (°C)
static float dec_catalyst_temp(const uint8_t *d) {
    return ((uint16_t)d[0] * 256 + d[1]) / 10.0f - 40.0f;
}

// PID 0x42: Control module voltage = (A × 256 + B) / 1000 (V)
static float dec_ctrl_voltage(const uint8_t *d) {
    return ((uint16_t)d[0] * 256 + d[1]) / 1000.0f;
}

// PID 0x46: Ambient temp = A - 40 (°C)
static float dec_ambient_temp(const uint8_t *d) { return (float)d[0] - 40.0f; }

// PID 0x47: Throttle B = A × 100 / 255 (%)
static float dec_throttle_b(const uint8_t *d) { return d[0] * 100.0f / 255.0f; }

// PID 0x49: Accel pedal D = A × 100 / 255 (%)
static float dec_accel_d(const uint8_t *d) { return d[0] * 100.0f / 255.0f; }

// PID 0x4A: Accel pedal E = A × 100 / 255 (%)
static float dec_accel_e(const uint8_t *d) { return d[0] * 100.0f / 255.0f; }

// PID 0x51: Fuel type = A (enum)
static float dec_fuel_type(const uint8_t *d) { return (float)d[0]; }

// PID 0x52: Ethanol % = A × 100 / 255 (%)
static float dec_ethanol(const uint8_t *d) { return d[0] * 100.0f / 255.0f; }

// PID 0x5C: Oil temp = A - 40 (°C)
static float dec_oil_temp(const uint8_t *d) { return (float)d[0] - 40.0f; }

// PID 0x5D: Fuel injection timing = ((A × 256 + B) - 26880) / 128 (degrees)
static float dec_injection_timing(const uint8_t *d) {
    return ((float)((uint16_t)d[0] * 256 + d[1]) - 26880.0f) / 128.0f;
}

// PID 0x5E: Fuel rate = (A × 256 + B) / 20 (L/h)
static float dec_fuel_rate(const uint8_t *d) {
    return ((uint16_t)d[0] * 256 + d[1]) / 20.0f;
}

// PID 0x61: Driver demand torque = A - 125 (%)
static float dec_driver_torque(const uint8_t *d) { return (float)d[0] - 125.0f; }

// PID 0x62: Actual engine torque = A - 125 (%)
static float dec_engine_torque(const uint8_t *d) { return (float)d[0] - 125.0f; }

// PID 0x63: Reference torque = A × 256 + B (N·m)
static float dec_ref_torque(const uint8_t *d) { return (float)((uint16_t)d[0] * 256 + d[1]); }

// PID 0x6F: Turbo inlet pressure = (A × 256 + B) × 0.03125 - 1 (kPa)
static float dec_turbo_inlet(const uint8_t *d) {
    return ((uint16_t)d[0] * 256 + d[1]) * 0.03125f - 1.0f;
}

// PID 0x70: Boost pressure = (A × 256 + B) × 0.03125 - 1 (kPa)
static float dec_boost_press(const uint8_t *d) {
    return ((uint16_t)d[0] * 256 + d[1]) * 0.03125f - 1.0f;
}

// PID 0x74: Turbo RPM = (A × 256 + B) × 10 (rpm)
static float dec_turbo_rpm(const uint8_t *d) { return ((uint16_t)d[0] * 256 + d[1]) * 10.0f; }

// PID 0x78: EGT Bank 1 sensor 1 = (A × 256 + B) / 10 - 40 (°C)
static float dec_egt_b1(const uint8_t *d) {
    // Skip first byte (supported sensors bitmap), use bytes 2-3 for sensor 1
    return ((uint16_t)d[1] * 256 + d[2]) / 10.0f - 40.0f;
}

// PID 0xA6: Odometer = (A×2^24 + B×2^16 + C×2^8 + D) / 10 (km)
static float dec_odometer(const uint8_t *d) {
    uint32_t raw = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
                   ((uint32_t)d[2] << 8) | d[3];
    return raw / 10.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Unit and Name Strings (stored in PROGMEM to save RAM)
 * ═══════════════════════════════════════════════════════════════════════════ */

static const char PROGMEM u_pct[]   = "%";
static const char PROGMEM u_degc[]  = "\xB0""C";
static const char PROGMEM u_kpa[]   = "kPa";
static const char PROGMEM u_rpm[]   = "rpm";
static const char PROGMEM u_kmh[]   = "km/h";
static const char PROGMEM u_deg[]   = "\xB0";
static const char PROGMEM u_gs[]    = "g/s";
static const char PROGMEM u_v[]     = "V";
static const char PROGMEM u_sec[]   = "s";
static const char PROGMEM u_lh[]    = "L/h";
static const char PROGMEM u_nm[]    = "N\xB7m";
static const char PROGMEM u_km[]    = "km";
static const char PROGMEM u_none[]  = "";

static const char PROGMEM n_load[]    = "Engine Load";
static const char PROGMEM n_coolant[] = "Coolant Temp";
static const char PROGMEM n_stft1[]   = "STFT Bank1";
static const char PROGMEM n_ltft1[]   = "LTFT Bank1";
static const char PROGMEM n_stft2[]   = "STFT Bank2";
static const char PROGMEM n_ltft2[]   = "LTFT Bank2";
static const char PROGMEM n_fuelp[]   = "Fuel Pressure";
static const char PROGMEM n_map[]     = "Intake MAP";
static const char PROGMEM n_rpm[]     = "RPM";
static const char PROGMEM n_speed[]   = "Speed";
static const char PROGMEM n_timing[]  = "Timing Adv";
static const char PROGMEM n_iat[]     = "Intake Temp";
static const char PROGMEM n_maf[]     = "MAF Rate";
static const char PROGMEM n_tps[]     = "Throttle";
static const char PROGMEM n_o2v[]     = "O2 B1S1";
static const char PROGMEM n_obdstd[]  = "OBD Std";
static const char PROGMEM n_runtime[] = "Runtime";
static const char PROGMEM n_frp[]     = "Fuel Rail P";
static const char PROGMEM n_egr[]     = "EGR";
static const char PROGMEM n_fuel[]    = "Fuel Level";
static const char PROGMEM n_baro[]    = "Baro Press";
static const char PROGMEM n_cat[]     = "Catalyst Temp";
static const char PROGMEM n_ctrlv[]   = "ECU Voltage";
static const char PROGMEM n_amb[]     = "Ambient Temp";
static const char PROGMEM n_tpsb[]    = "Throttle B";
static const char PROGMEM n_accd[]    = "Accel Ped D";
static const char PROGMEM n_acce[]    = "Accel Ped E";
static const char PROGMEM n_ftype[]   = "Fuel Type";
static const char PROGMEM n_eth[]     = "Ethanol %";
static const char PROGMEM n_oilt[]    = "Oil Temp";
static const char PROGMEM n_injt[]    = "Inj Timing";
static const char PROGMEM n_frate[]   = "Fuel Rate";
static const char PROGMEM n_dtorq[]   = "Driver Torq";
static const char PROGMEM n_etorq[]   = "Engine Torq";
static const char PROGMEM n_rtorq[]   = "Ref Torque";
static const char PROGMEM n_tinlet[]  = "Turbo Inlet";
static const char PROGMEM n_boost[]   = "Boost Press";
static const char PROGMEM n_trpm[]    = "Turbo RPM";
static const char PROGMEM n_egt[]     = "EGT Bank1";
static const char PROGMEM n_odo[]     = "Odometer";

/* ═══════════════════════════════════════════════════════════════════════════
 * PID Formula Table (stored in PROGMEM)
 * ═══════════════════════════════════════════════════════════════════════════ */

static const PIDDescriptor PROGMEM pidTable[] = {
//  PID                     len  decode              unit     name
    {PID_ENGINE_LOAD,        1, dec_engine_load,     u_pct,   n_load},
    {PID_COOLANT_TEMP,       1, dec_coolant_temp,    u_degc,  n_coolant},
    {PID_STFT_BANK1,         1, dec_fuel_trim,       u_pct,   n_stft1},
    {PID_LTFT_BANK1,         1, dec_fuel_trim,       u_pct,   n_ltft1},
    {PID_STFT_BANK2,         1, dec_fuel_trim,       u_pct,   n_stft2},
    {PID_LTFT_BANK2,         1, dec_fuel_trim,       u_pct,   n_ltft2},
    {PID_FUEL_PRESSURE,      1, dec_fuel_pressure,   u_kpa,   n_fuelp},
    {PID_INTAKE_MAP,         1, dec_intake_map,      u_kpa,   n_map},
    {PID_RPM,                2, dec_rpm,             u_rpm,   n_rpm},
    {PID_SPEED,              1, dec_speed,           u_kmh,   n_speed},
    {PID_TIMING_ADVANCE,     1, dec_timing_advance,  u_deg,   n_timing},
    {PID_INTAKE_TEMP,        1, dec_intake_temp,     u_degc,  n_iat},
    {PID_MAF_RATE,           2, dec_maf_rate,        u_gs,    n_maf},
    {PID_THROTTLE_POS,       1, dec_throttle,        u_pct,   n_tps},
    {PID_O2_VOLTAGE_B1S1,    1, dec_o2_voltage,      u_v,     n_o2v},
    {PID_OBD_STANDARD,       1, dec_obd_standard,    u_none,  n_obdstd},
    {PID_RUNTIME,            2, dec_runtime,         u_sec,   n_runtime},
    {PID_FUEL_RAIL_PRESSURE, 2, dec_fuel_rail_pressure, u_kpa, n_frp},
    {PID_EGR_COMMANDED,      1, dec_egr,             u_pct,   n_egr},
    {PID_FUEL_LEVEL,         1, dec_fuel_level,      u_pct,   n_fuel},
    {PID_BARO_PRESSURE,      1, dec_baro,            u_kpa,   n_baro},
    {PID_CATALYST_TEMP_B1S1, 2, dec_catalyst_temp,   u_degc,  n_cat},
    {PID_CTRL_MODULE_VOLTAGE,2, dec_ctrl_voltage,    u_v,     n_ctrlv},
    {PID_AMBIENT_TEMP,       1, dec_ambient_temp,    u_degc,  n_amb},
    {PID_THROTTLE_POS_B,     1, dec_throttle_b,      u_pct,   n_tpsb},
    {PID_ACCEL_PEDAL_D,      1, dec_accel_d,         u_pct,   n_accd},
    {PID_ACCEL_PEDAL_E,      1, dec_accel_e,         u_pct,   n_acce},
    {PID_FUEL_TYPE,          1, dec_fuel_type,       u_none,  n_ftype},
    {PID_ETHANOL_PCT,        1, dec_ethanol,         u_pct,   n_eth},
    {PID_OIL_TEMP,           1, dec_oil_temp,        u_degc,  n_oilt},
    {PID_FUEL_INJECTION_TIMING, 2, dec_injection_timing, u_deg, n_injt},
    {PID_FUEL_RATE,          2, dec_fuel_rate,       u_lh,    n_frate},
    {PID_DRIVER_TORQUE,      1, dec_driver_torque,   u_pct,   n_dtorq},
    {PID_ENGINE_TORQUE,      1, dec_engine_torque,   u_pct,   n_etorq},
    {PID_REFERENCE_TORQUE,   2, dec_ref_torque,      u_nm,    n_rtorq},
    {PID_TURBO_INLET_PRESS,  2, dec_turbo_inlet,     u_kpa,   n_tinlet},
    {PID_BOOST_PRESSURE,     2, dec_boost_press,     u_kpa,   n_boost},
    {PID_TURBO_RPM,          2, dec_turbo_rpm,       u_rpm,   n_trpm},
    {PID_EGT_BANK1,          3, dec_egt_b1,          u_degc,  n_egt},
    {PID_ODOMETER,           4, dec_odometer,        u_km,    n_odo},
};

#define PID_TABLE_COUNT (sizeof(pidTable) / sizeof(PIDDescriptor))

/* ═══════════════════════════════════════════════════════════════════════════
 * OBD2Data Implementation
 * ═══════════════════════════════════════════════════════════════════════════ */

OBD2Data::OBD2Data() : _cacheCount(0)
{
    memset(_cache, 0, sizeof(_cache));
}

/* ── getDescriptor() — static PROGMEM lookup ──────────────────────────── */

const PIDDescriptor *OBD2Data::getDescriptor(uint8_t pid)
{
    for (uint8_t i = 0; i < PID_TABLE_COUNT; i++) {
        uint8_t tpid = pgm_read_byte(&pidTable[i].pid);
        if (tpid == pid) {
            return &pidTable[i];
        }
    }
    return NULL;
}

uint8_t OBD2Data::getFormulaCount()
{
    return PID_TABLE_COUNT;
}

/* ── decodePID() ──────────────────────────────────────────────────────── */

bool OBD2Data::decodePID(OBD2PIDResponse *resp)
{
    if (!resp) return false;

    const PIDDescriptor *desc = getDescriptor(resp->pid);
    if (!desc) return false;

    // Read function pointer from PROGMEM
    pid_decode_fn fn = (pid_decode_fn)pgm_read_ptr(&desc->decode);
    if (!fn) return false;

    resp->value = fn(resp->data);
    updateCache(resp->pid, resp->value);
    return true;
}

/* ── updateCache() ────────────────────────────────────────────────────── */

void OBD2Data::updateCache(uint8_t pid, float value)
{
    int8_t idx = findCacheIndex(pid);
    if (idx < 0) {
        idx = allocCacheEntry(pid);
        if (idx < 0) return; // Cache full
    }

    _cache[idx].value = value;
    _cache[idx].timestamp = millis();
    _cache[idx].valid = true;
}

/* ── getValue() ───────────────────────────────────────────────────────── */

float OBD2Data::getValue(uint8_t pid) const
{
    int8_t idx = findCacheIndex(pid);
    if (idx >= 0 && _cache[idx].valid) {
        return _cache[idx].value;
    }
    return 0.0f;
}

/* ── isValid() ────────────────────────────────────────────────────────── */

bool OBD2Data::isValid(uint8_t pid) const
{
    int8_t idx = findCacheIndex(pid);
    return (idx >= 0 && _cache[idx].valid);
}

/* ── getTimestamp() ───────────────────────────────────────────────────── */

uint32_t OBD2Data::getTimestamp(uint8_t pid) const
{
    int8_t idx = findCacheIndex(pid);
    if (idx >= 0) return _cache[idx].timestamp;
    return 0;
}

/* ── getUnit() ────────────────────────────────────────────────────────── */

const char *OBD2Data::getUnit(uint8_t pid) const
{
    const PIDDescriptor *desc = getDescriptor(pid);
    if (desc) return (const char *)pgm_read_ptr(&desc->unit);
    return u_none;
}

/* ── getName() ────────────────────────────────────────────────────────── */

const char *OBD2Data::getName(uint8_t pid) const
{
    const PIDDescriptor *desc = getDescriptor(pid);
    if (desc) return (const char *)pgm_read_ptr(&desc->name);
    return u_none;
}

/* ── findCacheIndex() ─────────────────────────────────────────────────── */

int8_t OBD2Data::findCacheIndex(uint8_t pid) const
{
    for (uint8_t i = 0; i < _cacheCount; i++) {
        if (_cache[i].pid == pid) return (int8_t)i;
    }
    return -1;
}

/* ── allocCacheEntry() ────────────────────────────────────────────────── */

int8_t OBD2Data::allocCacheEntry(uint8_t pid)
{
    if (_cacheCount >= OBD2_MAX_TRACKED_PIDS) return -1;

    uint8_t idx = _cacheCount++;
    _cache[idx].pid = pid;
    _cache[idx].value = 0.0f;
    _cache[idx].timestamp = 0;
    _cache[idx].valid = false;
    return (int8_t)idx;
}

#endif /* OBD2_ELM327 */
