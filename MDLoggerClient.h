/*
    MDLoggerClient.h — I2C master driver for the MDLogger (328P) slave

    Pairs with MDLogger firmware running on an ATmega328P at I2C address 0x2A.
    The two devices share MDLoggerProtocol.h for the wire format.

    Wire protocol (raw — no STX/LEN/ETX framing; I2C handles delimiting):
      Master → Slave:
        DATA_FRAME    : [0x01][74-byte MDLoggerFrame]
        STATUS_REQUEST: [0x03]                          + read 5-byte response
        LOG_REQUEST   : [0x02]
        CONFIG        : [0x04][type][valLo][valHi]
      Slave → Master (in response to STATUS_REQUEST):
        [statusFlags][satellites][fix][gpsConfigured][dataValid]   (5 bytes)

    Reconnect behavior:
      If the slave is absent at boot or NACKs a transmission, _ready is
      cleared and sendData() short-circuits. pollStatus() is retried at the
      caller's cadence (every 5 s in MultidisplayController). A successful
      poll re-arms the link.

    Copyright 2026 rAtTrax / MultiDisplay Project
    Licensed under GPL v3
*/

#ifndef MDLOGGER_CLIENT_H_
#define MDLOGGER_CLIENT_H_

#include <Arduino.h>
#include <Wire.h>
#include "MDLoggerProtocol.h"

class MDLoggerClient {
public:
    MDLoggerClient();

    /** Initialize I2C as master and probe the slave once. Call from setup(). */
    void begin();

    /** Pack the next outbound frame. Call before sendData(). Lightweight. */
    void packFrame(uint32_t ts, const uint8_t *mdData,
                   int32_t lat, int32_t lon, uint16_t speed, uint16_t course,
                   uint8_t sats, uint8_t fix);

    /** Transmit the packed frame. Returns true on bus ACK. Marks link
     *  unhealthy on NACK so pollStatus() can re-arm it. */
    bool sendData();

    /** Send STATUS_REQUEST, read 5-byte response, update _status & _ready.
     *  This is also the "reconnect" trigger after a failed sendData(). */
    bool pollStatus();

    /** Last successful status snapshot (zeroed if never received). */
    MDLoggerStatus getStatus() const { return _status; }

    /** True if the most recent I2C exchange succeeded. */
    bool isReady() const { return _ready; }

private:
    bool           _ready;   // link health flag, cleared on NACK
    MDLoggerStatus _status;  // last snapshot from STATUS response
    MDLoggerFrame  _frame;   // outbound frame staging area
};

extern MDLoggerClient mdLoggerClient;

#endif /* MDLOGGER_CLIENT_H_ */
