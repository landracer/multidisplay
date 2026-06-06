# MultiDisplay Project Documentation

## Overview

MultiDisplay is an open-source automotive data logger, boost controller, and display system designed for vehicles. It runs on Arduino-based hardware (specifically the Seeeduino Mega with ATmega1280/2560) and provides comprehensive monitoring and control of various automotive parameters.

## Features

- 3.3V support for Bluetooth modules
- 12V input robust for automotive environment
- 20x4 LCD display support
- Speed input and gear computation
- Gear and RPM dependent N75 boost control with PID
- 8 Type K thermocouples for exhaust gas temperature measurement
- RPM measurement
- RGB warning LED for high EGT
- 8 VDO sensors for pressure or temperature
- Lambda measurement with narrow band probe or wideband signal
- Throttle position measurement
- Mass air flow measurement
- Boost measurement
- Data streaming to PC for visualization
- Event logging (maximum RPM, etc.)
- BorgWarner EFR speed sensor reading
- K-line connection to Bosch Digifant-1 ECU
- PC software for logging and analyzing measured data
- Smartphone app for Nokia N900 (Android app in development)

## Project Structure

```
multidisplay/
├── main.cpp                 # Main entry point
├── MultidisplayController.* # Main controller logic
├── SensorData.*             # Sensor data handling
├── RPMBoostController.*     # Boost control logic
├── LCDController.*          # LCD display handling
├── OledController.*         # OLED display handling (if applicable)
├── MultidisplayDefines.h    # Configuration defines
├── util.*                   # Utility functions
├── Map16x1.*                # Mapping functions
├── Map32x1.*                # Mapping functions
└── README.md                # Project overview
```

## Hardware Configuration

### Supported Hardware
- Seeeduino Mega with ATmega1280/2560 MCU
- 20x4 LCD Display
- Various automotive sensors:
  - Type K thermocouples (8 channels)
  - VDO pressure/temperature sensors (8 channels)
  - Lambda sensors
  - RPM sensors
  - Throttle position sensors
  - Boost sensors
  - Oil pressure sensors
  - Case temperature sensors

### Pin Configuration
- Boost sensor: Analog pin 1
- Boost2 sensor: Analog pin 2
- VDO Temperature pins: Analog pins 3-5
- VDO Pressure pins: Analog pins 6-8
- Lambda sensor: Analog pin 11
- Throttle position: Analog pin 12
- RPM input: Analog pin 13
- LMM input: Analog pin 14
- Case temperature: Analog pin 15
- AGT input: Analog pin 16

## Key Components

### 1. MultidisplayController
The main controller that orchestrates all system functions:
- Manages the main loop execution
- Handles sensor data collection and processing
- Controls display output
- Manages boost control logic
- Handles user input from buttons
- Manages communication with PC

### 2. SensorData
Handles all sensor data collection and processing:
- Collects data from various automotive sensors
- Processes raw analog values to physical units
- Manages maximum value tracking
- Provides data for display and control systems

### 3. RPMBoostController
Implements the boost control logic:
- Uses PID control for precise boost regulation
- Implements gear and RPM dependent control
- Includes safety features like EGT protection
- Manages N75 valve control

### 4. LCDController
Manages the 20x4 LCD display:
- Handles screen layout and content
- Implements screen switching
- Manages display updates and refresh
- Provides helper methods for drawing content

## Configuration Options

The system supports various configuration options through `MultidisplayDefines.h`:

### Engine Types
- VR6_MOTRONIC (for VR6 engines)
- DIGIFANT (for Digifant ECUs)

### Features
- BOOSTN75: Enable boost control
- TYPE_K: Enable Type K thermocouple support
- BW_EFR_SPEEDSENSOR: Enable EFR speed sensor support
- GEAR_RECOGNITION: Enable gear recognition

### Display Options
- LCD display support
- OLED display support (if configured)

## Communication Protocol

The system supports communication with a PC through serial interface with the following message format:

```
0: Command type (0=Manual, 1=Auto, 2=multidisplay command, else=ignore)
1-4: Setpoint (fixed point value, scale factor 1000)
5-8: Input (fixed point value, scale factor 1000)
9-12: Output (fixed point value, scale factor 1000)
13-16: P parameter (fixed point value, scale factor 1000)
17-20: I parameter (fixed point value, scale factor 1000)
21-24: D parameter (fixed point value, scale factor 1000)
2: Button commands (1=a pressed, 2=a hold, 3=b pressed, 4=b hold)
3: Serial mode change (2=enabled(string), 3=tunerpro adx request data)
4: V1 eeprom & config commands (1=save settings, 2=calibrate boost, 3=read settings, 5=set lcd brightness, 4=set v1 n75 duty cycles)
5: V2 dev debug/testing stuff
6: N75 v2 commands (gearX mode, duty cycle maps, setpoint maps)
```

## Safety Features

- EGT protection with yellow and critical thresholds
- EFR speed limit protection
- RPM-based boost control limits
- Memory management and watchdog support

## Development Notes

This project was originally developed by Stephan Martin and Dominik Gummel and is licensed under the GNU General Public License v3.0.

The system is designed to be modular and extensible, allowing for easy addition of new sensors or features.