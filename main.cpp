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
 * @file main.cpp
 * @brief Entry point for the Multidisplay firmware.
 *
 * On AVR (Mega 2560), this file provides a bare-metal main() function that
 * calls init() to set up the Arduino runtime, then enters an infinite loop
 * running the main controller.
 *
 * On ESP32, the standard Arduino setup()/loop() pattern is used instead,
 * because the ESP-IDF FreeRTOS scheduler manages the main task.
 */

#include "PlatformDefs.h"
#include <Arduino.h>

#include "LCDController.h"
#include "MultidisplayController.h"
#include "SensorData.h"
#include "RPMBoostController.h"

/*
 * ── Global objects ──────────────────────────────────────────────────────────
 * These are declared globally so every module can access them via `extern`.
 * The constructors are intentionally empty; real initialisation happens in
 * myconstructor() after init() has configured the hardware peripherals.
 */
SensorData data;

#ifdef LCD
LCDController lcdController;
#endif

MultidisplayController mController;

#ifdef BOOSTN75
RPMBoostController boostController;
#endif

/* LCD4Bit instance — drives the 20×4 HD44780-compatible character display
 * in 4-bit parallel mode.  Parameter '4' = number of display lines. */
LCD4Bit lcd(4);

/* ========================================================================== */
/*  AVR Entry Point                                                           */
/* ========================================================================== */
#ifdef PLATFORM_AVR

int main(void) {

	/* init() is provided by the Arduino core — it sets up timers, ADC,
	 * and other hardware peripherals.  Must be called before any
	 * Arduino API function (millis, analogRead, Serial, etc.). */
	init();

	/* Explicitly call myconstructor() on each global object.
	 * We avoid using C++ constructors for initialisation because
	 * the order of global constructor execution is undefined in C++
	 * and some objects depend on hardware being ready (Wire, Serial). */
	data.myconstructor();
#ifdef BOOSTN75
	boostController.myconstructor();
#endif
	mController.myconstructor();

	/* Main loop — never returns.
	 * All sensor reading, serial I/O, display updates, and boost control
	 * happen inside mainLoop(). */
	for (;;)
		mController.mainLoop();

	/* We must never return from main() on AVR. */
}

#endif /* PLATFORM_AVR */

/* ========================================================================== */
/*  ESP32 Entry Point (Arduino-style setup / loop)                            */
/* ========================================================================== */
#ifdef PLATFORM_ESP32

void setup() {
	/* On ESP32, init() is called automatically by the Arduino framework
	 * before setup().  We also need to initialise the EEPROM emulation
	 * with the required size (see PlatformDefs.h). */
	EEPROM.begin(PLATFORM_EEPROM_SIZE);

	data.myconstructor();
#ifdef BOOSTN75
	boostController.myconstructor();
#endif
	mController.myconstructor();
}

void loop() {
	mController.mainLoop();
}

#endif /* PLATFORM_ESP32 */
