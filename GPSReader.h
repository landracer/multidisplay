/*
    GPSReader.h — GY-NEO6MV2 (u-blox NEO-6M) GPS Reader on Serial3
    Parses NMEA $GPRMC sentences for position, speed, course, and UTC time.
    Connected to ATmega2560 UART3 (RX3=pin14, TX3=pin15).

    Copyright 2026 rAtTrax / MultiDisplay Project
    Contributors: uknowmelast, Axiom
    Licensed under GPL v3
*/

#ifndef GPSREADER_H_
#define GPSREADER_H_

#include <Arduino.h>
#include <stdint.h>

/* GPS serial port — Serial3 on ATmega2560 (RX3=14, TX3=15) */
#define GPS_SERIAL          Serial3
#define GPS_BAUD            9600

/* NMEA sentence buffer size */
#define GPS_NMEA_BUF_SIZE   84

struct GPSData {
    // Position (degrees × 10^6 for integer math, or raw float)
    int32_t  latitude;      // degrees × 1,000,000 (positive = N)
    int32_t  longitude;     // degrees × 1,000,000 (positive = E)

    // Speed and course
    uint16_t speedKnots100; // speed in knots × 100
    uint16_t speedKmh100;   // speed in km/h × 100
    uint16_t course100;     // course over ground × 100 (degrees)

    // UTC time/date from GPS
    uint32_t utcTime;       // HHMMSS (e.g. 123456 = 12:34:56)
    uint32_t utcDate;       // DDMMYY (e.g. 300326 = 30-Mar-26)

    // Fix quality
    uint8_t  fixValid;      // 1 = valid fix (GPRMC status 'A')
    uint8_t  satellites;    // number of satellites (from GPGGA)

    // Freshness
    unsigned long lastFixMillis;
};

class GPSReader {
public:
    GPSReader();

    /** Call once from myconstructor(). Starts Serial3 at 9600. */
    void begin();

    /** Call from mainLoop(). Reads available bytes, parses sentences.
     *  Non-blocking — processes whatever is in the Serial3 buffer. */
    void poll();

    /** Returns true if we have a valid fix less than 3s old. */
    bool hasFix() const;

    /** Current GPS data (updated by poll()). */
    const GPSData& data() const { return _data; }

    /** Write GPS summary into a serial frame.
     *  Writes 12 bytes: lat(4) + lon(4) + speedKmh100(2) + course100(2)
     *  Returns number of bytes written. */
    uint8_t serialWriteGPS(uint8_t *buf) const;

private:
    GPSData  _data;
    char     _buf[GPS_NMEA_BUF_SIZE];
    uint8_t  _bufIdx;
    bool     _receiving;

    void processNMEA(const char *sentence);
    void parseGPRMC(const char *sentence);
    void parseGPGGA(const char *sentence);

    /** Parse NMEA lat/lon field: "ddmm.mmmm" + "N"/"S" or "E"/"W"
     *  Returns degrees × 1,000,000 */
    static int32_t parseLatLon(const char *field, const char *dir);

    /** Parse a comma-separated field from NMEA sentence by index. */
    static const char* getField(const char *sentence, uint8_t index);

    /** Verify NMEA checksum: *XX at end of sentence. */
    static bool verifyChecksum(const char *sentence);
};

extern GPSReader gpsReader;

#endif /* GPSREADER_H_ */
