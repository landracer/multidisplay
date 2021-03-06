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


#include "Map32x1.h"

#include <avr/eeprom.h>
#include "stdlib.h"
#include "EEPROM.h"

Map32x1::Map32x1() {
	// TODO Auto-generated constructor stub

}

Map32x1::~Map32x1() {
	// TODO Auto-generated destructor stub
}


MapVdo5Bar::MapVdo5Bar () {
}

uint16_t MapVdo5Bar::map32(uint16_t idx) {
	float r = (idx & 0x7F) / 127.0 ;
	//map 0-4095 to 0-31
	idx = idx >> 7;
	return idx < 0x1F ? (pgm_read_word(&lookupVDOPressure5Bar[idx]) + ( (uint16_t) (r*(pgm_read_word(&lookupVDOPressure5Bar[idx+1])-pgm_read_word(&lookupVDOPressure5Bar[idx])))) ) : pgm_read_word(&lookupVDOPressure5Bar[idx]);
}
/*
 *
 * when supplied with 5V and a 220Ohm Resistor in series (measuring the Voltage on the Sensor)
 *it has a increasing Resistance with the Pressure (11-184 Ohm).
 *
 * potential divider (Spannungssteiler)
 *
 * +5V ---- [ R1 = 220 Ohm ] --|-- [ VDO Sensor R2 ] ------------ GND
 *                             |
 *                             |- AVR ADC ----------------------- GND
 *         |        U1         |     U2                |
 *         U2 = U * R2/(R1+R2)
 *
 *
 * 12 bit AD -> max 4095 >> 4 max 255
 * -> map to 32x1 map
 * max is 5Bar = 1866
 *
 * stores the pressure in mBar for 16 steps of ad readings
 * [0, 128, 256, 384, 512, 640, 768, 896, 1024, 1152, 1280, 1408, 1536, 1664, 1792, 1920]
 */
const uint16_t PROGMEM MapVdo5Bar::lookupVDOPressure5Bar[] =
	{ 0, 0, 107, 333, 571, 834, 1110, 1410, 1752, 2119, 2541, 2950, 3445, 3987, 4622, 5321, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


MapVdo10Bar::MapVdo10Bar () {
}

uint16_t MapVdo10Bar::map32(uint16_t idx) {
	float r = (idx & 0x7F) / 127.0 ;
	//map 0-4095 to 0-31
	idx = idx >> 7;
	return idx < 0x1F ? (pgm_read_word(&lookupVDOPressure10Bar[idx]) + ( (uint16_t) (r*(pgm_read_word(&lookupVDOPressure10Bar[idx+1])-pgm_read_word(&lookupVDOPressure10Bar[idx])))) ) : pgm_read_word(&lookupVDOPressure10Bar[idx]);
}
const uint16_t PROGMEM MapVdo10Bar::lookupVDOPressure10Bar[] =
	{ 0, 0, 214, 666, 1142, 1668, 2220, 2820, 3504, 4238, 5082, 5900, 6890, 7974, 9244, 10642, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


MapVdo5Bar mapVdo5Bar;
MapVdo10Bar mapVdo10Bar;
