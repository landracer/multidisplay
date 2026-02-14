/*
    Copyright 2009-14 Stephan Martin, Dominik Gummel

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
 * @file PlatformDefs.h
 * @brief Platform abstraction layer for cross-platform compatibility.
 *
 * This header provides compile-time abstractions so the Multidisplay firmware
 * can target multiple hardware platforms without scattering #ifdefs throughout
 * the application code.
 *
 * Currently supported platforms:
 *   - AVR  (Arduino Mega 2560 / ATmega1280)  — original & primary target
 *   - ESP32 (Espressif ESP32)                 — future hardware revision
 *
 * ──────────────────────────────────────────────────────────────────────────
 * What this file does:
 *   1. Detects the target MCU at compile time.
 *   2. Includes the correct low-level headers for each platform.
 *   3. Provides compatibility shims for PROGMEM, pgm_read_*, EEPROM, etc.
 *   4. Defines platform capability flags (HAS_HARDWARE_SERIAL1, etc.).
 * ──────────────────────────────────────────────────────────────────────────
 */

#ifndef PLATFORM_DEFS_H_
#define PLATFORM_DEFS_H_

#include <Arduino.h>
#include <stdint.h>

/* ========================================================================== */
/*  Section 1 – Platform Detection                                            */
/* ========================================================================== */

#if defined(__AVR__)
  /* ---- AVR (ATmega1280 / ATmega2560 / ATmega328P) ---- */
  #define PLATFORM_AVR        1
  #define PLATFORM_NAME       "AVR"

#elif defined(ESP32)
  /* ---- Espressif ESP32 ---- */
  #define PLATFORM_ESP32      1
  #define PLATFORM_NAME       "ESP32"

#else
  /* ---- Unknown platform – compile with warnings ---- */
  #warning "PlatformDefs.h: Unknown platform. Defaulting to generic Arduino."
  #define PLATFORM_GENERIC    1
  #define PLATFORM_NAME       "Generic"
#endif

/* ========================================================================== */
/*  Section 2 – Low-level Header Includes                                     */
/* ========================================================================== */

#ifdef PLATFORM_AVR
  /*
   * AVR-specific headers for direct register access, program-memory storage,
   * watchdog timer, and hardware I/O.
   */
  #include <avr/io.h>
  #include <avr/pgmspace.h>
  #include <avr/wdt.h>
  #include <avr/eeprom.h>
#endif

#ifdef PLATFORM_ESP32
  /*
   * ESP32 uses the Preferences library as a replacement for AVR EEPROM.
   * PROGMEM on ESP32 is a no-op (flash is memory-mapped), but the macro
   * is still defined by the ESP32 Arduino core for compatibility.
   *
   * The ESP32 Arduino core provides its own EEPROM emulation library that
   * stores data in a partition of SPI flash.
   */
  #include <EEPROM.h>
  #include <esp_task_wdt.h>   /* ESP32 watchdog API */
#endif

/* ========================================================================== */
/*  Section 3 – PROGMEM Compatibility                                         */
/* ========================================================================== */
/*
 * On AVR, PROGMEM stores constant data in flash rather than SRAM.
 * On ESP32, flash is memory-mapped so PROGMEM is essentially a no-op,
 * but the pgm_read_* macros must still resolve correctly.
 *
 * The ESP32 Arduino core already defines PROGMEM and pgm_read_* as
 * pass-through macros, so no extra work is needed here.  We just
 * verify they exist and provide fallbacks for truly generic builds.
 */
#if !defined(PROGMEM)
  #define PROGMEM                       /* no-op on platforms without flash separation */
#endif

#if !defined(pgm_read_byte)
  #define pgm_read_byte(addr)   (*(const uint8_t  *)(addr))
#endif

#if !defined(pgm_read_word)
  #define pgm_read_word(addr)   (*(const uint16_t *)(addr))
#endif

#if !defined(pgm_read_float)
  #define pgm_read_float(addr)  (*(const float    *)(addr))
#endif

/* ========================================================================== */
/*  Section 4 – Watchdog Abstraction                                          */
/* ========================================================================== */
/*
 * Provides platform-independent watchdog macros:
 *   PLATFORM_WDT_RESET()   — kick the watchdog to prevent reset
 *   PLATFORM_WDT_ENABLE()  — enable the watchdog (platform-specific timeout)
 *   PLATFORM_WDT_DISABLE() — disable the watchdog
 */

#ifdef PLATFORM_AVR
  /* AVR watchdog is managed via <avr/wdt.h> */
  #define PLATFORM_WDT_RESET()    wdt_reset()
  /* Note: AVR watchdog enable is done via custom watchdogOn() in util.cpp */
  #define PLATFORM_WDT_ENABLE()   /* handled by watchdogOn() */
  #define PLATFORM_WDT_DISABLE()  wdt_disable()

#elif defined(PLATFORM_ESP32)
  /* ESP32 Task Watchdog Timer (TWDT) */
  #define PLATFORM_WDT_RESET()    esp_task_wdt_reset()
  #define PLATFORM_WDT_ENABLE()   do { \
      esp_task_wdt_init(5, true); \
      esp_task_wdt_add(NULL); \
    } while(0)
  #define PLATFORM_WDT_DISABLE()  esp_task_wdt_delete(NULL)

#else
  /* Generic platform — watchdog is a no-op */
  #define PLATFORM_WDT_RESET()
  #define PLATFORM_WDT_ENABLE()
  #define PLATFORM_WDT_DISABLE()
#endif

/* ========================================================================== */
/*  Section 5 – Hardware Serial Availability                                  */
/* ========================================================================== */
/*
 * On Mega 2560 there are 4 hardware UARTs: Serial, Serial1, Serial2, Serial3
 * On ESP32, Serial, Serial1, Serial2 are available (3 UARTs)
 * On basic AVR (328P), only Serial is available
 */

#if defined(PLATFORM_AVR)
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define HAS_HARDWARE_SERIAL1  1
    #define HAS_HARDWARE_SERIAL2  1
    #define HAS_HARDWARE_SERIAL3  1
  #endif
#elif defined(PLATFORM_ESP32)
  #define HAS_HARDWARE_SERIAL1    1
  #define HAS_HARDWARE_SERIAL2    1
#endif

/* ========================================================================== */
/*  Section 6 – EEPROM Size                                                   */
/* ========================================================================== */
/*
 * ATmega2560 has 4 KB of EEPROM (addresses 0 – 4095).
 * ESP32 EEPROM emulation can be configured up to 4096 bytes (or more).
 *
 * Multidisplay allocates up to address ~4000, so 4 KB is the minimum.
 */

#ifdef PLATFORM_AVR
  #define PLATFORM_EEPROM_SIZE  4096
#elif defined(PLATFORM_ESP32)
  #define PLATFORM_EEPROM_SIZE  4096   /* Must call EEPROM.begin(4096) in setup */
#else
  #define PLATFORM_EEPROM_SIZE  4096
#endif

/* ========================================================================== */
/*  Section 7 – Serial Protocol Framing Bytes                                 */
/* ========================================================================== */
/*
 * The binary serial protocol uses STX (0x02) and ETX (0x03) as frame
 * delimiters.  These were previously sent via Serial.print("\2") which is
 * correct but unclear.  Explicit constants improve readability.
 */

#define SERIAL_STX_BYTE  0x02   /* Start-of-Text: frame start delimiter */
#define SERIAL_ETX_BYTE  0x03   /* End-of-Text:   frame end   delimiter */

/* ========================================================================== */
/*  Section 8 – Inline-Assembly Guard Macros                                  */
/* ========================================================================== */
/*
 * Direct port register manipulation via inline ASM is only valid on AVR.
 * On ESP32 and other platforms, fall back to standard Arduino I/O functions.
 *
 * Usage in application code:
 *   AVR_ONLY( asm("sbi %0,4" : : "I" (_SFR_IO_ADDR(PORTA)) ); )
 *   ESP32_ONLY( digitalWrite(26, HIGH); )
 */

#ifdef PLATFORM_AVR
  #define AVR_ONLY(code)    code
  #define ESP32_ONLY(code)  /* stripped on AVR */
#elif defined(PLATFORM_ESP32)
  #define AVR_ONLY(code)    /* stripped on ESP32 */
  #define ESP32_ONLY(code)  code
#else
  #define AVR_ONLY(code)    /* stripped on generic */
  #define ESP32_ONLY(code)  /* stripped on generic */
#endif

#endif /* PLATFORM_DEFS_H_ */
