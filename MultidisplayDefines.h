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
 * @file MultidisplayDefines.h
 * @brief Master configuration file for the Multidisplay automotive datalogger.
 *
 * This is the central configuration header that controls every compile-time
 * option for the Multidisplay firmware.  Edit the defines here to tailor the
 * build to your vehicle, sensors, and display hardware.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Sections:
 *   1.  Platform Detection       – auto-detect Mega 2560 / ESP32
 *   2.  ECU Selection            – choose VR6 Motronic or Digifant
 *   3.  Drivetrain / Gearing     – gear count, tire rolling circumference
 *   4.  N75 Boost Controller     – PID modes, safety limits
 *   5.  Exhaust Gas Temperature  – Type-K thermocouple & EFR speed sensor
 *   6.  Debug & Optional Flags   – Bluetooth, dev debug, memory check
 *   7.  Timing Constants         – init time, button hold, screen save
 *   8.  Shift Light (V1 & V2)    – RPM thresholds and LED pins
 *   9.  Pin Configuration        – SPI, buttons, sensors, PWM
 *  10.  Sensor Calibration       – lambda, throttle, RPM, speed
 *  11.  Type-K & VDO Lookup      – thermocouple and VDO sensor parameters
 *  12.  I2C Expander Addresses   – PCF8574 I/O expander bus addresses
 *  13.  Serial Protocol          – output modes, binary tags, frame rate
 *  14.  Display Configuration    – LCD & OLED options
 *  15.  Unit Conversions         – bar to psi, scope modes
 *  16.  EEPROM Memory Map        – persistent storage layout
 *  17.  Digifant K-Line           – DF ECU serial protocol parameters
 *  18.  KWP1281 Protocol          – Bosch Motronic K-Line diagnostics
 * ──────────────────────────────────────────────────────────────────────────
 */

#ifndef MULTIDISPLAY_DEFINES_H_
#define MULTIDISPLAY_DEFINES_H_

/* Include the platform abstraction layer first — it sets up PROGMEM shims,
   watchdog macros, and platform-detection flags used throughout the project. */
#include "PlatformDefs.h"

/* ========================================================================== */
/*  1. Platform Detection                                                     */
/* ========================================================================== */
/*
 * MULTIDISPLAY_V2 is the "full-featured" build that requires either an
 * Arduino Mega 2560 (ATmega1280/2560) or an ESP32.  It enables multiple
 * hardware serial ports, additional sensor channels, speed input capture,
 * and the expanded I/O used on the V2 PCB.
 *
 * MULTIDISPLAY_V1 (commented out) targets the original single-UART boards
 * such as the Duemilanove / UNO.
 */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(ESP32)
#define MULTIDISPLAY_V2
#else
/* #define MULTIDISPLAY_V1 */
#endif

/* Enable the hardware watchdog timer to reset the MCU if the main loop hangs.
 * This is critical for automotive safety — a locked-up controller could leave
 * the N75 boost solenoid in an unsafe state. */
#define WATCHDOG

/* ========================================================================== */
/*  2. ECU Selection                                                          */
/* ========================================================================== */
/*
 * Select exactly ONE of the engine-control-unit configurations below.
 * Each ECU type pulls in its own sub-header with sensor calibrations,
 * RPM factors, K-line settings, and lambda / boost sensor options.
 *
 *   VR6_MOTRONIC  –  VW VR6 with Bosch Motronic M3.8.1 or M2.x ECU
 *   DIGIFANT      –  VW Digifant-1 ECU (e.g. Audi S2, 2.0 8V turbo)
 */
#define VR6_MOTRONIC
/* #define DIGIFANT */

#if defined(VR6_MOTRONIC)
  #include "MultidisplayDefinesVR6.h"
#endif
#if defined(DIGIFANT)
  #include "MultidisplayDefinesDigifant.h"
#endif

/* ========================================================================== */
/*  3. Drivetrain / Gearing                                                   */
/* ========================================================================== */
/*
 * GEARS               – number of forward gears in the transmission.
 * GEAR_RECOGNITION    – enable automatic gear detection from speed / RPM.
 * ABROLLUMFANG        – tire rolling circumference in metres.
 *                        Used for speed calculation and gear-ratio matching.
 *   215/40 R16 → ~2.000 m
 *   195/50 R15 → ~1.757 m
 *   205/40 R17 → ~1.818 m
 */
#define GEARS 6
#define GEAR_RECOGNITION
#define ABROLLUMFANG 2            /* 215/40 R16 */
/* #define ABROLLUMFANG 1.757 */  /* 195/50 R15 */
/* #define ABROLLUMFANG 1.818 */  /* 205/40 R17 */

/* ========================================================================== */
/*  4. N75 Boost Controller (Turbo Wastegate Solenoid)                        */
/* ========================================================================== */
/*
 * BOOSTN75            – enable PWM-driven N75 wastegate solenoid control.
 * The controller supports two selectable boost maps:
 *   BOOST_MODE_NORMAL (1) – conservative / daily-driving map
 *   BOOST_MODE_RACE   (0) – aggressive / track-day map
 * Mode is toggled by an external switch wired to NORDSCHLEIFENPIN.
 *
 * Safety features:
 *   BOOST_EFR_SPEEDLIMIT_PROTECTION – reduce boost if turbo overspeeds
 *   BOOST_EGT_PROTECTION            – reduce boost on excessive EGT
 *   BOOST_MAX_EGT_YELLOW / _CRITICAL – EGT thresholds (°C)
 */
#define BOOSTN75
#define BOOST_MODE_NORMAL 1
#define BOOST_MODE_RACE 0
#define RPM_MIN_FOR_BOOST_CONTROL 0        /* RPM window lower bound */
#define RPM_MAX_FOR_BOOST_CONTROL 10000    /* RPM window upper bound */
#define BOOST_EFR_SPEEDLIMIT_PROTECTION
#define BOOST_EGT_PROTECTION
#define BOOST_MAX_EGT_YELLOW 960           /* first-stage EGT warning (°C) */
#define BOOST_MAX_EGT_CRITICAL 975         /* critical EGT limit (°C) */

/* ========================================================================== */
/*  5. Exhaust Gas Temperature & BorgWarner EFR Speed Sensor                  */
/* ========================================================================== */
/*
 * READFROMEEPROM      – load saved calibration & maps from EEPROM at boot.
 * TYPE_K              – enable Type-K thermocouple EGT measurement via
 *                        external MCP3208 12-bit ADC + analog mux.
 * BW_EFR_SPEEDSENSOR  – enable BorgWarner EFR turbo-speed sensor input.
 *                        Uses Timer4 input capture on Pin 49 (AVR Port PL0).
 * BW_EFR_TIME_2_RPM   – conversion factor from timer counts to turbo RPM.
 *                        Depends on the number of wheel blades:
 *                          12 blades (67 mm): 40000000
 *                          14 blades:         34285712
 */
#define READFROMEEPROM

#define TYPE_K
#define BW_EFR_SPEEDSENSOR
#define BW_EFR_SPEEDSENSOR_PIN 49          /* AVR: Port PL0 / Timer 4 ICP */
#define BW_EFR_SMOOTH 5                    /* moving-average window size */
#define BW_EFR_TIME_2_RPM 40000000         /* 12 wheel blades (67 mm) */
/* #define BW_EFR_TIME_2_RPM 34285712 */   /* 14 wheel blades */

/* ========================================================================== */
/*  6. Debug & Optional Feature Flags                                         */
/* ========================================================================== */
/*
 * Uncomment any of the flags below to enable the corresponding feature.
 * All of these are disabled by default for production builds.
 *
 *   V2DEVDEBUG              – enable developer debug commands via serial
 *   BLUETOOTH_ON_SERIAL2    – mirror data stream to Bluetooth module on Serial2
 *   BLUETOOTH_SETUP_ON_SERIAL2 – allow AT-command setup of BT module
 *   DEBUG                   – verbose debug prints to Serial
 *   LCDTEST                 – LCD hardware test pattern
 *   RPM_DEBUG               – internal RPM frequency generator for bench testing
 *   FREEMEM                 – print free SRAM each loop iteration
 */
/* #define V2DEVDEBUG 1 */
/* #define BLUETOOTH_ON_SERIAL2 1 */
/* #define BLUETOOTH_SETUP_ON_SERIAL2 */
/* #define DEBUG 1 */
/* #define LCDTEST 1 */
/* #define RPM_DEBUG 0 */
/* #define FREEMEM 0 */

/* ========================================================================== */
/*  7. Timing Constants                                                       */
/* ========================================================================== */
/*
 * INITTIME         – duration (ms) to show the splash / intro screen at boot.
 * BUTTONHOLD       – interval (ms) for triggering a "button hold" action.
 * SCREENSAVEDELAY  – idle time (ms) before the active screen is auto-saved
 *                     to EEPROM so it persists across power cycles.
 * FLASH_TIME       – blink interval (ms) when a sensor limit is exceeded.
 * FLASH_E_TIME     – minimum duration (ms) of the limit-exceeded flash.
 */
#define INITTIME 3000
#define BUTTONHOLD 300
#define SCREENSAVEDELAY 10000
#define FLASH_TIME 100
#define FLASH_E_TIME 5000

/* ========================================================================== */
/*  8. Shift Light / Warning LEDs                                             */
/* ========================================================================== */

/* --- V1 single-colour shift light (optional, uncomment to enable) --- */
/* #define V1_RPM_SHIFT_LIGHT */
#define V1_RPM_SHIFT_LOW 8000              /* RPM for half brightness */
#define V1_RPM_LOW_BRIGHT 40
#define V1_RPM_SHIFT_HIGH 9000             /* RPM for full brightness */
#define V1_RPM_HIGH_BRIGHT 255
#define V1_RPM_NO_BRIGHT 0                 /* brightness below shift RPM */

/* --- V2 RGB warning LED driven by shift-LED pins 1-3 ---
 * Colour changes based on EGT:
 *   Blue  (< 450°C)  → engine cold / normal
 *   Green (< 900°C)  → operating range
 *   Yellow(< 945°C)  → approaching limit
 *   Red   (≥ 945°C)  → danger / overtemp
 * Also goes red when EFR turbo speed exceeds the redline. */
#define V2_RGB_WARNLED_ON_SHIFTLED123
#define V2_RGB_WARNLED_EFR_SPEED_REDLINE 148000  /* real EFR6758 redline: 154k */
#define V2_SHIFTLED1PIN 39                 /* Red channel   */
#define V2_SHIFTLED2PIN 38                 /* Green channel */
#define V2_SHIFTLED3PIN 3                  /* Blue channel  */

/* ========================================================================== */
/*  9. Pin Configuration                                                      */
/* ========================================================================== */
/*
 * Pin assignments depend on the board version (V1 = UNO, V2 = Mega / ESP32).
 * All sensor pins below refer to MCP3208 ADC channel numbers (1-16), NOT
 * Arduino analog pins, unless otherwise noted.
 */

/* --- Software SPI for MCP3208 ADC --- */
#ifdef MULTIDISPLAY_V2
  #define DATAOUT 57                       /* MOSI  (57 on Mega, 17 on UNO) */
  #define DATAIN 12                        /* MISO */
  #define SPICLOCK 26                      /* SCK   (26 on V2 PCB) */

  /* Direct-wired buttons (active-high with external pull-down) */
  #define V2_BUTTON1 22
  #define V2_BUTTON2 23
  #define V2_BUTTON3 24
  #define V2_BUTTON4 25
#endif

#ifdef MULTIDISPLAY_V1
  #define DATAOUT 17                       /* MOSI (UNO / Duemilanove) */
  #define DATAIN 12                        /* MISO */
  #define SPICLOCK 13                      /* SCK  */
#endif

/* --- Misc pins (Arduino pin numbers) --- */
#define BATTERYPIN 0                       /* Analog: voltage divider for 12 V supply */
#define LCDBRIGHTPIN 5                     /* PWM output for LCD backlight brightness */
#define V1_RPMSHIFTLIGHTPIN 3              /* PWM output for V1 shift light */

/*
 * Nordschleifen switch (boost-mode toggle):
 *   V1: digital pin 15
 *   V2: analog pin A1  (read as digital; internal pull-up, switch to GND)
 *
 * K10 connector: 2× PWM (pins 10 & 11)
 *   Pin 11 → N75 boost solenoid output
 *   Pin 10 → spare PWM (warning LED / unused)
 *
 * K11 connector: 2× analog (A1 & A2) + 2× digital (15, 16)
 *   A1 → Nordschleifen switch (V2)
 *   Pin 53 → OLED reset (V2)
 */
#ifdef MULTIDISPLAY_V1
  #define NORDSCHLEIFENPIN 15
#else
  #define NORDSCHLEIFENPIN A1
#endif

#define FREEPWM2 10                        /* spare PWM output (left of LCD header) */
#define N75PIN 11                          /* N75 boost solenoid PWM output */

/* --- External MCP3208 ADC channels (1-based) ---
 * These correspond to the KLx connector positions on the MD03 PCB. */
#define BOOSTPIN 1                         /* (KL7-2) Primary boost pressure sensor */
#define BOOST2PIN 2                        /* (KL7-3) Secondary boost pressure sensor */

#define VDOT1PIN 3                         /* (KL6) VDO temperature sensor 1 */
#define VDOT2PIN 4                         /* (KL5) VDO temperature sensor 2 */
#define VDOT3PIN 5                         /* (KL4) VDO temperature sensor 3 */
#define VDOP1PIN 6                         /* (KL3) VDO pressure sensor 1 (oil) */
#define VDOP2PIN 7                         /* (KL2) VDO pressure sensor 2 (fuel) */
#define VDOP3PIN 8                         /* (KL1) VDO pressure sensor 3 */

#define LAMBDAPIN 11                       /* (KL9-4) Narrow/wideband lambda input */
#define THROTTLEPIN 12                     /* (KL9-3) Throttle position (0-5 V) */
#define RPMPIN 13                          /* (KL9-2) RPM signal (conditioned analog) */
#define LMMPIN 14                          /* (KL9-1) Mass Air Flow (LMM) signal */

#define CASETEMPPIN 15                     /* LM35 case-temperature sensor */
#define AGTPIN 16                          /* Analog mux output (Type-K input) */

/* ========================================================================== */
/*  10. Sensor Calibration                                                    */
/* ========================================================================== */

/* --- Lambda (oxygen sensor) --- */
#define LAMBDAMIN 100                      /* narrow-band ADC low limit */
#define LAMBDAMAX 1100                     /* narrow-band ADC high limit */
#define LAMBDALOWERLIMIT 3                 /* display floor */
#define LAMBDAUPPERLIMIT 15                /* display ceiling */

/* --- Throttle position sensor ---
 * Each ECU / throttle body has different ADC endpoints.
 * Values are 12-bit (0-4095) from MCP3208.
 * VR6 16-bit (M3.8.1): 5 V = closed, 0 V = open  → inverted mapping
 * VR6 8-bit  (M2.x):   0.5 V = closed, 4.5 V = open
 * Audi S2 (Digifant):  specific range */
#define THROTTLE_VR6_16BIT_MIN 3530
#define THROTTLE_VR6_16BIT_MAX 620
#define THROTTLE_VR6_8BIT_MIN 3600
#define THROTTLE_VR6_8BIT_MAX 409
#define THROTTLE_S2_MIN 3597
#define THROTTLE_S2_MAX 560

/* --- RPM ---
 * RPMFACTOR and USE_RPM_ONBOARD / USE_DIGIFANT_RPM are in ECU-specific headers.
 * RPMSMOOTH  – number of readings for the moving-average filter.
 *              5 = responsive; 10 = very smooth but adds latency.
 * RPMMAX     – ADC ceiling that maps to approximately 8000 RPM. */
#define RPMSMOOTH 5
#define RPMMAX 3400

/* --- Vehicle speed ---
 * SPEEDFACTOR and SPEEDCORRECTIONFACTOR are in ECU-specific headers.
 * SPEEDSMOOTH – moving-average window size for speed readings.
 * SPEEDPIN    – Arduino analog pin used for the vehicle speed signal. */
#define SPEEDSMOOTH 5
#define SPEEDPIN 4

/* ========================================================================== */
/*  11. Type-K Thermocouple & VDO Sensor Parameters                           */
/* ========================================================================== */
/*
 * TEMPTYPEKREADINGS   – entries in the Type-K µV → °C lookup table (0-1350 °C).
 * MAX_ATTACHED_TYPE_K – maximum number of thermocouple channels (hardware limit).
 * MAX_TYPE_K          – µV reading above which the channel is treated as open.
 *
 * NUMBER_OF_ATTACHED_TYPE_K is defined in the ECU-specific header (VR6 or DF). */
#define TEMPTYPEKREADINGS 28
#define MAX_ATTACHED_TYPE_K 8
#define MAX_TYPE_K 1200

/* ========================================================================== */
/*  12. I2C Expander Addresses (PCF8574)                                      */
/* ========================================================================== */
/*
 * Two PCF8574 I/O expanders on the I2C bus drive the MCP3208 chip-select
 * lines and the analog multiplexer channel-select for Type-K readings.
 *
 * NXP PCF8574:    A2=0, A1=0, A0=1  →  0b0100001 = 0x21
 *                 A2=0, A1=0, A0=0  →  0b0100000 = 0x20
 * TI  PCF8574AN:  different address range (0x38-0x3F) — uncomment if needed.
 */
#define EXPANDER  0b0100001                /* PCF8574 #1 (buttons + misc) */
#define EXPANDER2 0b0100000                /* PCF8574 #2 (MCP3208 CS + mux) */
/* #define EXPANDER  0b0100001 */          /* TI PCF8574AN #1 */
/* #define EXPANDER2 0b0111000 */          /* TI PCF8574AN #2 */

#define DEBUGRPM 10                        /* RPM debug generator divider */

/* ========================================================================== */
/*  13. Serial Protocol Configuration                                         */
/* ========================================================================== */
/*
 * The firmware supports several serial output modes selectable at run-time:
 *   SERIALOUT_DISABLED     – no serial output
 *   SERIALOUT_BINARY       – compact binary frames (STX/TAG/data/ETX)
 *   SERIALOUT_ENABLED      – human-readable CSV text
 *   SERIALOUT_TUNERPRO_ADX – TunerPro RT datalog format
 *
 * SERIALOUT_BINARY_TAG     – frame tag / length byte (identifies frame type).
 * SERIALFREQ               – data send interval in milliseconds.
 *                             10 ms ≈ 100 Hz; 100 ms ≈ 10 Hz.
 */
#define SERIALOUT_DISABLED 0
#define SERIALOUT_BINARY 1
#define SERIALOUT_ENABLED 2
#define SERIALOUT_TUNERPRO_ADX 3

#define SERIALOUT_BINARY_TAG 95
#define SERIALOUT_BINARY_TAG_N75_DUTY_MAP 22
#define SERIALOUT_BINARY_TAG_N75_SETPOINT_MAP 38
#define SERIALOUT_BINARY_TAG_ACK 4
#define SERIALOUT_BINARY_TAG_N75_PARAMS 23
#define SERIALOUT_BINARY_TAG_GEAR_RATIO_6G 17

#define SERIALFREQ 10                      /* send interval in ms (100 Hz) */
/* #define SERIALFREQ 100 */               /* 10 Hz */
/* #define SERIALFREQ 800 */               /* ~1.25 Hz (slow debug) */

/* ========================================================================== */
/*  14. Display Configuration                                                 */
/* ========================================================================== */
/*
 * LCD   – enable the 20×4 HD44780-compatible character LCD (4-bit mode).
 * OLED  – enable the Goldelox 4D serial OLED module (alternative display).
 *         Only one display type should be active at a time.
 *
 * LCD_DRAW_TIME  – minimum interval (ms) between LCD screen refreshes.
 * LCD_BUFSIZE    – internal print buffer size (must be ≥ LCD_WIDTH + 1).
 */
/* #define OLED */
#define OLED_RESET_PIN 53

#define LCD
#define LCD_DRAW_TIME 300

#define LCD_BUFSIZE 21
#define LCD_WIDTH 20
#define LCD_HEIGHT 4
#define LCD_BIGFONT_2 2                    /* 2-line big-font mode */
#define LCD_BIGFONT_4 4                    /* 4-line big-font mode */

#define SCREENA 1
#define SCREENB 2

/* Scope screen data sources */
#define SCOPE_BOOST 1
#define SCOPE_LMM 2
#define SCOPE_RPM 3
#define SCOPE_THROTTLE 4
#define SCOPE_LAMBDA 5

/* ========================================================================== */
/*  15. Unit Conversions                                                      */
/* ========================================================================== */

#define BAR2PSI 14.50377                   /* 1 bar = 14.50377 psi */

/* ========================================================================== */
/*  16. EEPROM Memory Map                                                     */
/* ========================================================================== */
/*
 * Persistent storage layout — addresses are byte offsets into the 4 KB EEPROM.
 *
 * Address | Size    | Description
 * --------+---------+---------------------------------------------------
 *    0    | 1 byte  | N75 manual duty cycle (normal mode)
 *    1    | 1 byte  | N75 manual duty cycle (race mode)
 *    2    | 2 bytes | PID aggressive Kp  (uint16, fixed-point base 100)
 *    4    | 2 bytes | PID aggressive Kd
 *    6    | 2 bytes | PID aggressive Ki
 *    8    | 2 bytes | PID conservative Kp
 *   10    | 2 bytes | PID conservative Kd
 *   12    | 2 bytes | PID conservative Ki
 *   14    | 2 bytes | PID aggressive activation threshold
 *   16    | 2 bytes | PID conservative activation threshold
 *   18    | 1 byte  | PID enable flag
 *   19    | 2 bytes | Max boost limit (uint16, fixed-point base 100)
 *   25    | 12 bytes| Gear ratios (6 × uint16, fixed-point base 1000)
 *  100    | 1 byte  | Last active LCD screen number
 *  101    | 2 bytes | Serial output frequency (ms)
 *  105    | 1 byte  | LCD backlight brightness
 *  200    | 4 bytes | Ambient pressure (long, float×1000)
 *  205    | 1 byte  | Boost calibration point
 * 3511    | 192 B   | 6 × low-setpoint PID boost maps  (16×1, uint16)
 * 3707    | 192 B   | 6 × high-setpoint PID boost maps (16×1, uint16)
 * 3903    | 96 B    | 6 × low duty-cycle maps           (16×1, uint8)
 * 3999    | 96 B    | 6 × high duty-cycle maps          (16×1, uint8)
 */
#define EEPROM_N75_MANUALDUTY_NORMAL 0
#define EEPROM_N75_MANUALDUTY_RACE 1
#define EEPROM_N75_PID_aKp 2
#define EEPROM_N75_PID_aKd 4
#define EEPROM_N75_PID_aKi 6
#define EEPROM_N75_PID_cKp 8
#define EEPROM_N75_PID_cKd 10
#define EEPROM_N75_PID_cKi 12
#define EEPROM_N75_PID_aAT 14
#define EEPROM_N75_PID_cAT 16
#define EEPROM_N75_ENABLE_PID 18
#define EEPROM_N75_MAX_BOOST 19            /* 16-bit */

#define EEPROM_GEAR_RATIO_MAP_START 25     /* 6 × 16-bit fixed-int base 1000 */

#define EEPROM_ACTIVESCREEN 100            /* 1 byte */
#define EEPROM_SERIALFREQ 101              /* 2 bytes */
#define EEPROM_BRIGHTNESS 105              /* 1 byte */

#define EEPROM_AMBIENTPRESSURE 200         /* 4 bytes (long) */
#define EEPROM_LDCALPOINT 205              /* 1 byte */
#define EEPROM_N75_PID_LOW_SETPOINT_MAPS 3511
#define EEPROM_N75_PID_HIGH_SETPOINT_MAPS 3707
#define EEPROM_N75_LOW_DUTY_CYCLE_MAPS 3903
#define EEPROM_N75_HIGH_DUTY_CYCLE_MAPS 3999

/* ========================================================================== */
/*  17. Digifant K-Line Protocol                                              */
/* ========================================================================== */
/*
 * Parameters for receiving real-time data from a Digifant-1 ECU over its
 * diagnostic K-line.  The ECU streams frames at a fixed baud rate (6667).
 *
 * DF_KLINEFRAMESIZE   – total bytes per frame including STX / ETX delimiters.
 * DF_KLINE_STORE_FRAME_COUNT – double-buffered: 2 frames stored.
 */
#define DF_KLINESERIALFREQ 1               /* polling interval (ms) */
#define DF_KLINEFRAMESIZE 34               /* includes STX + ETX bytes */
#define DF_KLINE_STORE_FRAME_COUNT 2
#define DF_KLINEBEGIN 2                    /* STX marker */
#define DF_KLINEEND 3                      /* ETX marker */
#define DF_KLINE_STATUS_RECEIVING 1
#define DF_KLINE_STATUS_FRAME_COMPLETE 2
#define DF_KLINE_STATUS_FRAMEERROR 3

/* ========================================================================== */
/*  18. KWP1281 Diagnostics Protocol (Bosch Motronic K-Line)                  */
/* ========================================================================== */
/*
 * KWP1281 is the legacy VAG diagnostics protocol used by older Bosch ECUs.
 * Communication is half-duplex over the K-line at 5-baud init, then 9600 baud.
 *
 * Note: KWP1281 support is experimental and disabled by default.
 * Enable KWP1281_KLINE in MultidisplayDefinesVR6.h to activate.
 */
#define V2_LLINE 29                        /* PA7 — L-line pin (AVR port A, bit 7) */
#define KWP1281_KLINEFRAMESIZE 40
#define KWP1281_KLINE_STORE_FRAME_COUNT 2

/* Connection state machine states */
#define KWP1281_STATE_NO_CONNECTION 0
#define KWP1281_STATE_HANDSHAKE 1
#define KWP1281_STATE_SLAVE_RCV_CONTROLLER_DATA 2
#define KWP1281_STATE_ESTABLISHED_MASTER 3
#define KWP1281_STATE_ESTABLISHED_SLAVE 4
#define KWP1281_STATE_ESTABLISHED_SLAVE_RECEIVING_BLOCK 5
#define KWP1281_STATE_ESTABLISHED_MASTER_SEND_ACK 6
#define KWP1281_STATE_ESTABLISHED_MASTER_SENDING_ACK 7

/* Block-read sub-states */
#define KWP1281_BLOCKREAD_STATE_WAIT_LENGTH 0
#define KWP1281_BLOCKREAD_STATE_WAIT_COUNTER 1
#define KWP1281_BLOCKREAD_STATE_WAIT_TITLE 2
#define KWP1281_BLOCKREAD_STATE_WAIT_DATA 3
#define KWP1281_BLOCKREAD_STATE_WAIT_BLOCKEND 4

/* KWP1281 command IDs */
#define KWP1281_COMMAND_ACK 9
#define KWP1281_COMMAND_GET_CONTROLLER_DATA 0
#define KWP1281_COMMAND_ASCII_DATA 0xF6
#define KWP1281_COMMAND_REQ_GROUP_READING 0x29
#define KWP1281_COMMAND_GROUP_READING_RESPONSE 0xE7

/* Timing and limits */
#define KWP1281_MAX_CONNECTION_ATTEMPTS 255
#define KWP1281_KEEPALIVE_MSECS 1000       /* ms between keep-alive ACKs */
#define KWP1281_INTERBLOCK_TIMEOUT 1200    /* ms timeout waiting for block */
#define KWP1281_DEBUG_TIME 1000            /* ms between debug prints */

/* Complement macro for KWP1281 byte acknowledgement */
#define KWP1281_COMP(a) (0xFF - (a))

/* ========================================================================== */
/*  Internal test mode — do NOT use in production                             */
/* ========================================================================== */
/* #define DEV_INTERNAL_TEST_MODE */

#endif /* MULTIDISPLAY_DEFINES_H_ */
