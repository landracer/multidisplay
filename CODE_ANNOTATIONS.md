# MultiDisplay Code Annotations

## Overview

This document provides detailed annotations and explanations of the MultiDisplay firmware codebase. The system is designed to monitor and control automotive parameters using Arduino hardware.

## Core Components

### 1. Main Entry Point (main.cpp)

```cpp
int main(void) {
    init();
    data.myconstructor();
    mController.myconstructor();
    
    for (;;)
        mController.mainLoop();
}
```

**Key Points:**
- Initializes the Arduino system
- Sets up global objects (SensorData, MultidisplayController)
- Runs the main loop continuously

### 2. MultidisplayController

The main controller that orchestrates all system functions.

#### Key Methods:

```cpp
void MultidisplayController::myconstructor() {
    // Initialize all subsystems
    // Setup LCD display
    // Initialize boost controller if enabled
    // Setup serial communication
}
```

```cpp
void MultidisplayController::mainLoop() {
    // Main execution loop
    // Read sensors
    // Update displays
    // Handle user input
    // Manage system states
}
```

```cpp
void MultidisplayController::calibrateLD() {
    // Calibration routine for lambda sensor
    // Handles different calibration modes
}
```

#### Configuration Constants:

```cpp
// Serial communication protocol
#define SERIAL_MODE_MANUAL 0
#define SERIAL_MODE_AUTO 1
#define SERIAL_MODE_MULTIDISPLAY 2
```

### 3. SensorData

Handles all sensor data collection and processing.

#### Key Data Structures:

```cpp
class MaxDataSet {
    // Stores maximum values for various parameters
    float boost;
    float lmm;
    float lambda;
    uint16_t rpm;
    uint16_t speed;
    float oilpres;
    float gaspres;
    uint8_t gear;
    uint16_t efr_speed;
    uint8_t hottestTypKIndex;
};
```

#### Key Methods:

```cpp
void SensorData::readSensors() {
    // Read all analog sensors
    // Convert raw values to physical units
    // Update maximum values
}
```

```cpp
float SensorData::getBoost() {
    // Calculate boost pressure from sensor readings
    // Apply calibration factors
}
```

### 4. RPMBoostController

Implements the boost control logic with PID regulation.

#### Key Features:

- Gear and RPM dependent control
- PID control algorithm
- Safety protection features
- N75 valve control

#### Key Methods:

```cpp
void RPMBoostController::myconstructor() {
    // Initialize PID controller
    // Setup control parameters
    // Configure safety limits
}
```

```cpp
void RPMBoostController::updateBoost() {
    // Calculate new boost setpoint
    // Apply PID control
    // Update N75 valve duty cycle
}
```

### 5. LCDController

Manages the 20x4 LCD display output.

#### Key Features:

- Multiple screen layouts
- Screen switching
- Display update management
- Button input handling

#### Key Methods:

```cpp
void LCDController::drawScreen() {
    // Draw current screen content
    // Update display with sensor data
}
```

```cpp
void LCDController::handleButtonInput() {
    // Process button presses
    // Handle screen navigation
    // Execute menu commands
}
```

## Configuration Options

### Hardware Configuration

```cpp
// Choose your engine ECU
#define VR6_MOTRONIC
//#define DIGIFANT

// Enable boost control
#define BOOSTN75

// Enable Type K thermocouples
#define TYPE_K

// Enable EFR speed sensor
#define BW_EFR_SPEEDSENSOR
```

### Pin Assignments

```cpp
// Sensor pins
#define BOOSTPIN 1
#define BOOST2PIN 2
#define VDOT1PIN 3
#define VDOT2PIN 4
#define VDOT3PIN 5
#define VDOP1PIN 6
#define VDOP2PIN 7
#define VDOP3PIN 8
#define LAMBDAPIN 11
#define THROTTLEPIN 12
#define RPMPIN 13
#define LMMPIN 14
#define CASETEMPPIN 15
#define AGTPIN 16
```

## Communication Protocol

The system supports serial communication with a PC using the following message format:

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

### EGT Protection

```cpp
#define BOOST_MAX_EGT_YELLOW 960
#define BOOST_MAX_EGT_CRITICAL 975
```

### EFR Speed Protection

```cpp
#define BOOST_EFR_SPEEDLIMIT_PROTECTION
```

### RPM Control Limits

```cpp
#define RPM_MIN_FOR_BOOST_CONTROL 0
#define RPM_MAX_FOR_BOOST_CONTROL 10000
```

## Data Flow

1. **Sensor Reading**: Raw analog values read from sensors
2. **Data Processing**: Conversion to physical units, calibration
3. **Control Logic**: Boost control, safety checks
4. **Display Update**: LCD output with current data
5. **Communication**: Serial communication with PC
6. **User Input**: Button handling for menu navigation

## Memory Management

The system includes memory management features:

```cpp
// Watchdog support
#define WATCHDOG

// Memory monitoring
#define FREEMEM 0
```

## Extensibility

The modular design allows for easy extension:

- New sensor types can be added by implementing new reading functions
- Additional display screens can be created
- New control algorithms can be integrated
- Communication protocols can be extended

## Development Notes

This firmware was originally developed by Stephan Martin and Dominik Gummel and is licensed under the GNU General Public License v3.0. The codebase is designed to be modular and extensible, making it suitable for modification and enhancement by automotive enthusiasts and developers.