/*
    MDLoggerProtocol.h — Shared protocol between MDLogger (328P) and MDLoggerClient (Mega2560)
    
    This header defines the I2C frame layout and command protocol.
    Both projects include this file for type-safe, zero-copy compatibility.
    
    Copyright 2026 rAtTrax / MultiDisplay Project
    Licensed under GPL v3
*/

#ifndef MDLOGGER_PROTOCOL_H_
#define MDLOGGER_PROTOCOL_H_

#include <stdint.h>

/* ═════════════════════════════════════════════════════════════════════════════
 * I2C Protocol Constants
 * ═════════════════════════════════════════════════════════════════════════════ */

/* I2C slave address (0x08-0x77) */
#define MDLOGGER_I2C_ADDR       0x2A

/* I2C command IDs */
#define MDLOGGER_CMD_DATA_FRAME     0x01    // Data frame from Mega2560
#define MDLOGGER_CMD_LOG_REQUEST    0x02    // Request log write
#define MDLOGGER_CMD_STATUS_REQUEST 0x03    // Request status
#define MDLOGGER_CMD_CONFIG         0x04    // Configuration command

/* Response IDs */
#define MDLOGGER_RESP_ACK           0x80    // Acknowledgment
#define MDLOGGER_RESP_STATUS        0x83    // Status response
#define MDLOGGER_RESP_ERROR         0xFF    // Error response

/* Frame sizes */
#define MDLOGGER_FRAME_SIZE         74      // Total packed frame size
#define MDLOGGER_MD_DATA_SIZE       57      // MD binary frame data payload
#define MDLOGGER_STATUS_SIZE        5       // Status response size

/* ═════════════════════════════════════════════════════════════════════════════
 * MDLoggerFrame — packed struct (74 bytes total)
 * 
 * Layout:
 *   [0-3]    timestamp     uint32_t  millis() from Mega2560
 *   [4-60]   mdData[57]    uint8_t[] MD binary frame data (bytes 1-57 of SERIALOUT_BINARY)
 *   [61-64]  latitude      int32_t   degrees × 1,000,000
 *   [65-68]  longitude     int32_t   degrees × 1,000,000
 *   [69-70]  speedKmh100   uint16_t  km/h × 100
 *   [71-72]  course100     uint16_t  degrees × 100
 *   [73]     satellites    uint8_t   GPS satellite count
 *   [74]     fix           uint8_t   GPS fix type (0=none, 1=fix)
 * 
 * Note: mdData[57] contains the 57 bytes after TAG (offset 2-58 of the 95-byte frame).
 * This is the core sensor data: RPM, boost, EGTs, lambda, throttle, speed, gear, etc.
 * ═════════════════════════════════════════════════════════════════════════════ */

struct __attribute__((packed)) MDLoggerFrame {
    uint32_t timestamp;     // millis() from Mega2560 (4 bytes)
    uint8_t  mdData[MDLOGGER_MD_DATA_SIZE];  // MD binary frame data (57 bytes)
    int32_t  latitude;      // degrees × 1,000,000 (4 bytes)
    int32_t  longitude;     // degrees × 1,000,000 (4 bytes)
    uint16_t speedKmh100;   // km/h × 100 (2 bytes)
    uint16_t course100;     // degrees × 100 (2 bytes)
    uint8_t  satellites;    // GPS satellite count (1 byte)
    uint8_t  fix;           // GPS fix type (1 byte: 0=none, 1=fix)
};

/* Logger status flags (5-byte response from MDLogger 328P) */
union MDLoggerStatus {
    struct {
        uint8_t sdReady       : 1;
        uint8_t bufferFull    : 1;
        uint8_t loggingActive : 1;
        uint8_t gpsHasFix     : 1;
        uint8_t reserved      : 4;
    };
    uint8_t asByte;
};

/* ═════════════════════════════════════════════════════════════════════════════
 * Notes
 * ═════════════════════════════════════════════════════════════════════════════
 * 
 * MDLoggerClient (the I2C master driver) is defined in MDLoggerClient.h/.cpp
 * on the Mega2560 side. The MDLogger (328P slave) has its own class of the
 * same name in MDLogger.h. Both share this protocol header for type safety.
 * ═════════════════════════════════════════════════════════════════════════════ */

#endif /* MDLOGGER_PROTOCOL_H_ */
