/*
  EEPROM.h - EEPROM library
  Copyright (c) 2006 David A. Mellis.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef EEPROM_h
#define EEPROM_h

#include <inttypes.h>

/*
 * On ESP32, the Arduino core provides its own EEPROM emulation library
 * that stores data in SPI flash.  We only define our AVR-specific
 * EEPROMClass when building for AVR targets.
 */
#if defined(__AVR__)

class EEPROMClass
{
  public:
    uint8_t read(int);
    void write(int, uint8_t);
};

extern EEPROMClass EEPROM;

#else
  /* On non-AVR platforms (ESP32), include the platform's EEPROM header.
   * The ESP32 Arduino core provides a compatible EEPROMClass with
   * read(), write(), begin(), and commit() methods. */
#endif

#endif

