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
 * @file Map16x1.h
 * @brief 16-element lookup maps used for N75 boost control.
 *
 * Map16x1       – 8-bit duty-cycle map (one value per RPM slice, 16 slices)
 * Map16x1Double – floating-point boost-setpoint map (same 16 slices)
 * MapThrottleBoostReduction – PROGMEM-based throttle→boost reduction curve
 *
 * The 256-value RPM range is divided into 16 bins.  Intermediate values
 * are linearly interpolated between adjacent bins.
 */

#ifndef MAP16X1_H_
#define MAP16X1_H_

#include "PlatformDefs.h"
#include "util.h"

class Map16x1 {
public:
	Map16x1();
	virtual ~Map16x1();

	uint8_t map (uint8_t idx);
	void loadFromEeprom (int address);
	void writeToEeprom (int address);

	uint8_t *data;
};

class Map16x1Double {
public:
	Map16x1Double();
	virtual ~Map16x1Double();

//	uint8_t map (uint8_t idx);
	double map (uint8_t idx);
	void loadFromEeprom (int address);
	void writeToEeprom (int address);

	double *data;
};

class MapThrottleBoostReduction {
public:
	MapThrottleBoostReduction();

	double map (uint8_t);

	static const float PROGMEM lookupThrottleBoostReduction[];
};

extern MapThrottleBoostReduction mapThrottleBoostReduction;

#endif /* MAP16X1_H_ */
