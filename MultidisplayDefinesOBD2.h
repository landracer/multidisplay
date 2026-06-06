/*
    MultidisplayDefinesOBD2.h
    ECU Configuration for OBD-II via ELM327DSL IC on Serial1

    Reference: ELM327DSL Datasheet (ELM Electronics)
    https://www.elmelectronics.com/wp-content/uploads/2020/05/ELM327DSL.pdf

    The ELM327DSL is wired directly to ATmega2560 UART1 (RX1/TX1).
    Pin 6 on the ELM327DSL sets the power-on baud rate (9600 on this HW).
    After ATZ, we use AT BRD to upgrade to the target baud rate for
    faster PID polling throughput.

    Copyright 2026 rAtTrax / MultiDisplay Project
    Contributors: uknowmelast, Axiom
    Licensed under GPL v3 — see gpl.txt
*/

#ifndef MULTIDISPLAYDEFINESOBD2_H_
#define MULTIDISPLAYDEFINESOBD2_H_

/* ═══════════════════════════════════════════════════════════════════════════
 * ELM327 Hardware Configuration
 * ═══════════════════════════════════════════════════════════════════════════ */

#define OBD2_ELM327                     // Master enable for OBD-II via ELM327

/* Power-on baud rate — determined by ELM327DSL Pin 6 hardware level.
 * The auto-detect in ELM327::begin() tries this first, then scans
 * other common rates if it fails.                                        */
#define OBD2_ELM327_BAUD        9600

/* Target operational baud rate — after ATZ at the power-on rate,
 * ELM327::begin() uses AT BRD to upgrade to this speed.
 * Divisor = 4,000,000 / target_baud  (ELM327DSL datasheet §"AT BRD")
 *   38400 → divisor 0x68 (104)   — standard, reliable
 *   57600 → divisor 0x45 (69)    — fast, good on ATmega2560 @ 16MHz
 *  115200 → divisor 0x23 (35)    — fastest, 2.1% UART error at 16MHz
 * Set to 0 to skip baud upgrade and stay at the power-on rate.           */
#define OBD2_ELM327_TARGET_BAUD 38400

/* Serial port used for ELM327 communication.
 * Serial1 = UART1 on ATmega2560 (RX1=pin19, TX1=pin18)                 */
#define OBD2_SERIAL               Serial1

/* ELM327 response timeout in milliseconds.
 * With ATST 32 (200ms OBD timeout) and AT AT2 (aggressive adaptive),
 * responses typically arrive within 50-200ms. 1s gives plenty of margin
 * while avoiding long stalls on unsupported PIDs.                       */
#define OBD2_ELM_TIMEOUT_MS     1000

/* Number of retries for failed ELM327 commands.
 * 1 = two attempts total.  Needed for ISO 9141-2 BUS INIT:
 * the first request may trigger slow-init ("BUS INIT: ...OK"),
 * the second attempt gets actual data.  With the backoff in
 * OBD2Manager::poll(), this limits blocking to ~4s worst case.          */
#define OBD2_ELM_RETRIES        1

/* ═══════════════════════════════════════════════════════════════════════════
 * OBD-II Protocol Selection
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ELM327 protocol number sent via ATSP command.
 * 0 = Automatic (recommended — ELM327 will detect ISO 9141-2)
 * 3 = ISO 9141-2 (force if auto-detect fails)
 * 6 = ISO 15765-4 CAN (11-bit, 500kbaud)
 * See ELM327DSL.pdf section "AT SP" for full list.                       */
#define OBD2_ELM_PROTOCOL       3       // Force ISO 9141-2 for OBD simulator

/* ═══════════════════════════════════════════════════════════════════════════
 * PID Selection — Enable/Disable individual PIDs
 *
 * Comment out any PID you do NOT want polled. The scheduler only
 * polls enabled PIDs. For the header/display, enabled PIDs are
 * available via obd2_get_value(OBD2_PID_xxx).
 *
 * Tier 1: Essential — fast poll (every cycle)
 * Tier 2: Standard  — slower poll (every 2nd cycle)
 * Tier 3: Extended  — slow poll (every 5th cycle)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── Tier 1: Essential (poll every cycle) ─────────────────────────────── */
#define OBD2_ENABLE_PID_RPM             // 0x0C Engine RPM
#define OBD2_ENABLE_PID_SPEED           // 0x0D Vehicle speed
#define OBD2_ENABLE_PID_COOLANT_TEMP    // 0x05 Coolant temperature
#define OBD2_ENABLE_PID_ENGINE_LOAD     // 0x04 Calculated engine load
#define OBD2_ENABLE_PID_INTAKE_MAP      // 0x0B Intake manifold pressure
#define OBD2_ENABLE_PID_THROTTLE_POS    // 0x11 Throttle position

/* ── Tier 2: Standard (poll every 2nd cycle) ──────────────────────────── */
#define OBD2_ENABLE_PID_INTAKE_TEMP     // 0x0F Intake air temperature
#define OBD2_ENABLE_PID_MAF_RATE        // 0x10 MAF air flow rate
#define OBD2_ENABLE_PID_TIMING_ADV      // 0x0E Ignition timing advance
#define OBD2_ENABLE_PID_STFT_B1         // 0x06 Short term fuel trim bank 1
#define OBD2_ENABLE_PID_LTFT_B1         // 0x07 Long term fuel trim bank 1
#define OBD2_ENABLE_PID_FUEL_PRESSURE   // 0x0A Fuel pressure
#define OBD2_ENABLE_PID_BARO_PRESSURE   // 0x33 Barometric pressure

/* ── Tier 3: Extended (poll every 5th cycle) ──────────────────────────── */
#define OBD2_ENABLE_PID_MONITOR_STATUS  // 0x01 MIL lamp + DTC count (data point, NOT polled separately)
#define OBD2_ENABLE_PID_RUNTIME         // 0x1F Run time since start
#define OBD2_ENABLE_PID_FUEL_LEVEL      // 0x2F Fuel tank level
#define OBD2_ENABLE_PID_AMBIENT_TEMP    // 0x46 Ambient air temperature
#define OBD2_ENABLE_PID_OIL_TEMP        // 0x5C Engine oil temperature
#define OBD2_ENABLE_PID_CTRL_VOLTAGE    // 0x42 Control module voltage
// #define OBD2_ENABLE_PID_ETHANOL_PCT  // 0x52 Ethanol fuel %
// #define OBD2_ENABLE_PID_TURBO_RPM    // 0x74 Turbocharger RPM
// #define OBD2_ENABLE_PID_BOOST_PRESS  // 0x70 Boost pressure control
// #define OBD2_ENABLE_PID_EGT_B1       // 0x78 Exhaust gas temp bank 1
// #define OBD2_ENABLE_PID_ODOMETER     // 0xA6 Odometer

/* ═══════════════════════════════════════════════════════════════════════════
 * Scheduler Configuration
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Maximum number of PIDs the scheduler can track simultaneously.
 * Each PID entry uses ~20 bytes of RAM.  32 PIDs = ~640 bytes.           */
#define OBD2_MAX_TRACKED_PIDS   32

/* Base poll interval in milliseconds. Tier multipliers apply on top.     */
#define OBD2_POLL_BASE_MS       50

/* Tier poll multipliers (Tier 1 = every cycle, Tier 2 = 2x, Tier 3 = 5x) */
#define OBD2_TIER1_MULT         1
#define OBD2_TIER2_MULT         2
#define OBD2_TIER3_MULT         5

/* ═══════════════════════════════════════════════════════════════════════════
 * Serial Output / Bluetooth Configuration
 *
 * The existing MD binary frame (TAG=0x5F, 95 bytes) is preserved for the
 * core MD sensor data. OBD-II data is sent in a SEPARATE frame with its
 * own tag so OpenDash can parse both independently.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Tag byte for OBD-II binary frame (must not collide with 0x5F=95)       */
#define OBD2_BINARY_TAG         0x4F    // 'O' for OBD

/* Maximum OBD-II frame payload (after STX+TAG, before ETX)
 * Each PID entry = 1 byte PID + 2 bytes value = 3 bytes
 * With 20 active PIDs: header(6) + 20*3 + ETX = 67 bytes                */
#define OBD2_MAX_FRAME_SIZE     128

/* ═══════════════════════════════════════════════════════════════════════════
 * RPM and Speed source selection
 *
 * When OBD2 is enabled, RPM and Speed can come from OBD-II instead of
 * the analog/frequency inputs. Enable these to USE OBD-II values for
 * gear computation and boost control.
 * ═══════════════════════════════════════════════════════════════════════════ */

// #define USE_OBD2_RPM            // Use OBD-II RPM for gear/boost logic
// #define USE_OBD2_SPEED          // Use OBD-II speed for gear computation

/* RPM and Speed from existing analog/frequency inputs remain available
 * regardless — these defines just select the "primary" source.           */

/* ═══════════════════════════════════════════════════════════════════════════
 * Boost map sensor — keep existing Freescale sensor for real-time boost
 * control. OBD-II MAP (PID 0x0B) is too slow for PID boost control.
 * ═══════════════════════════════════════════════════════════════════════════ */
#define BOOST_FREESCALE_MPXA6400A

/* ═══════════════════════════════════════════════════════════════════════════
 * Lambda
 * ═══════════════════════════════════════════════════════════════════════════ */
#define LAMBDA_WIDEBAND
#define SLCOEM

/* ═══════════════════════════════════════════════════════════════════════════
 * RPM (onboard frequency input — kept for fast boost PID)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define USE_RPM_ONBOARD
#define RPMFACTOR 9.2

/* ═══════════════════════════════════════════════════════════════════════════
 * Speed
 * ═══════════════════════════════════════════════════════════════════════════ */
#define SPEEDFACTOR 0.38
#define SPEEDCORRECTIONFACTOR 0.9

/* ═══════════════════════════════════════════════════════════════════════════
 * Type K EGT (kept — EGT is analog, not available via OBD-II for most cars)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define NUMBER_OF_ATTACHED_TYPE_K 6


#endif /* MULTIDISPLAYDEFINESOBD2_H_ */
