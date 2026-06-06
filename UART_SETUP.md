# UART Setup for Multidisplay

This document explains how to set up UART communication with the multidisplay system.

## Overview

The multidisplay system can output serial data via UART for external monitoring and integration with other systems like OpenDash. The system can also output data via Bluetooth using an HC-06 module connected to Serial2.

## Serial Output Format

The system outputs data in the following format:
```
2:12345:1200:15:25:14.5:12:10:25:30:12.5:15:20:25:30:35:40:45:50:55:60:65:70:75:80:85:90:95:100
```

Where:
- `2`: Output type (SERIALOUT_ENABLED)
- `12345`: Time in milliseconds
- `1200`: RPM
- `15`: Boost pressure
- `25`: Throttle position
- `14.5`: Lambda value
- `12`: LMM
- `10`: Case temperature
- `25,30,12.5,15,20,25,30,35`: 8 EGT values (maximum 8 sensors)
- `40`: Battery voltage
- `45,50,55,60,65,70,75,80,85,90,95,100`: VDO pressure and temperature values

## Hardware Setup

### UART Connection
To connect to an external system:
1. Connect the UART pins of the multidisplay to the external system
2. Use a level shifter if needed (multidisplay is 5V, external systems may be 3.3V)
3. Ensure common ground connection

### Bluetooth Connection
To connect via Bluetooth:
1. Connect an HC-06 Bluetooth module to Serial2 of the multidisplay
2. Configure the module to connect to "rAtTrax" or "multidisplay"
3. The module will forward the multidisplay serial data to the connected device

### Pin Configuration
- TX: Usually connected to pin 1 (Arduino pin 1) or configurable
- RX: Usually connected to pin 0 (Arduino pin 0) or configurable
- GND: Common ground
- VCC: 5V power supply

## Configuration

The serial output is enabled by default in the multidisplay system. To verify or modify the configuration:

1. Check the `MultidisplayDefines.h` file for serial output settings
2. Ensure `SERIALOUT_ENABLED` is defined
3. The baud rate is set to 115200

## Integration with OpenDash

To integrate with OpenDash:
1. Connect the UART pins between multidisplay and OpenDash
2. Configure OpenDash to read from UART (this is handled automatically)
3. The data will be parsed and displayed on the OpenDash UI

## Connection Toggle

The OpenDash system supports enabling/disabling the multidisplay connection via a configuration toggle:
- Set `MultiDisplay-Connection=true` to enable connection to multidisplay
- Set `MultiDisplay-Connection=false` to disable connection to multidisplay

## Troubleshooting

### No Data Received
1. Check UART connections
2. Verify the baud rate matches (115200)
3. Ensure common ground is connected
4. Check that serial output is enabled in the multidisplay configuration

### Incorrect Data Format
1. Verify the multidisplay firmware version
2. Check that the output format hasn't changed
3. Ensure the data is being sent in the correct format