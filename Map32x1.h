/**
 * @file Map32x1.h
 * @brief 32-element PROGMEM lookup maps for VDO pressure sensor calibration.
 *
 * MapVdo5Bar   – VDO 0-5 bar oil pressure sensor lookup
 * MapVdo10Bar  – VDO 0-10 bar fuel/gas pressure sensor lookup
 *
 * These maps convert 12-bit ADC readings (0-4095) to millibar values
 * using PROGMEM-stored lookup tables with linear interpolation.
 */

#ifndef MAP32X1_H_
#define MAP32X1_H_

#include "PlatformDefs.h"
#include "util.h"

class Map32x1 {
public:
	Map32x1();
	virtual ~Map32x1();
};

class MapVdo5Bar {
public:
	MapVdo5Bar();

	uint16_t map32 (uint16_t);

	static const uint16_t PROGMEM lookupVDOPressure5Bar[];
};

class MapVdo10Bar {
public:
	MapVdo10Bar();

	uint16_t map32 (uint16_t);

	static const uint16_t PROGMEM lookupVDOPressure10Bar[];
};

extern MapVdo5Bar mapVdo5Bar;
extern MapVdo10Bar mapVdo10Bar;


#endif /* MAP32X1_H_ */
