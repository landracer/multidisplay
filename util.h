/*
    Copyright 2009-13 Stephan Martin, Dominik Gummel

    This file is part of Multidisplay.

    Multidisplay is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Multidisplay is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Multidisplay.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file util.h
 * @brief Utility functions: free memory, EEPROM multi-byte I/O,
 *        fixed-point conversions, and watchdog setup.
 *
 * These helpers are used throughout the firmware for:
 *   - Reading/writing 16-bit and 32-bit values to EEPROM (byte-at-a-time)
 *   - Converting between float and fixed-point integer representations
 *     used in the serial protocol and EEPROM storage
 *   - Reporting free SRAM on AVR targets
 *   - Configuring the hardware watchdog timer
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>

/**
 * Returns the number of free bytes between the heap and the stack.
 * Only meaningful on AVR targets with a flat memory model.
 * On ESP32 this returns 0 (use ESP.getFreeHeap() instead).
 */
uint16_t freeMem(void);

/* ---- EEPROM multi-byte read / write helpers ---- */

/** Write a 32-bit long to EEPROM at the given address (4 consecutive bytes). */
void EEPROMWriteLong(int p_address, long p_value);
/** Read a 32-bit long from EEPROM at the given address. */
long EEPROMReadLong(int p_address);

/** Write a 16-bit unsigned int to EEPROM (2 consecutive bytes, little-endian). */
void EEPROMWriteuint16(int p_address, uint16_t p_value);
/** Read a 16-bit unsigned int from EEPROM. */
uint16_t EEPROMReaduint16(int p_address);

/* ---- Fixed-point ↔ float conversion (scale factor 100) ---- */

/** Convert float to uint16_t fixed-point (× 100). E.g. 1.25 → 125 */
uint16_t float2fixedintb100 (float in);
/** Convert float to uint32_t fixed-point (× 100). */
uint32_t float2fixedint32b100 (float in);
/** Convert int fixed-point (÷ 100) back to float. */
float fixedintb1002float (int in);
/** Convert uint16_t fixed-point (÷ 100) back to float. */
float fixedintb1002float (uint16_t in);

/* ---- Fixed-point ↔ float conversion (scale factor 1000) ---- */

/** Convert float to uint16_t fixed-point (× 1000). Used for gear ratios. */
uint16_t float2fixedintb1000 (float in);
/** Convert uint16_t fixed-point (÷ 1000) back to float. */
float fixedintb10002float (uint16_t in);

/**
 * Enable the hardware watchdog timer.
 * On AVR: configures WDT for ~4-second timeout with interrupt mode.
 * On ESP32: uses the Task Watchdog Timer (see PlatformDefs.h macros).
 */
void watchdogOn();

#endif /* UTIL_H_ */
