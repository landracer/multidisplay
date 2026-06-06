/*
    GPSReader.cpp — GY-NEO6MV2 GPS Reader Implementation

    Reads NMEA sentences from Serial3 (RX3=pin14, TX3=pin15) on the
    ATmega2560. Parses $GPRMC for position/speed/time and $GPGGA for
    satellite count. Non-blocking — processes buffered bytes each loop.

    The NEO-6M outputs at 9600 baud by default, sending GPRMC/GPGGA/etc.
    at 1 Hz.

    Copyright 2026 rAtTrax / MultiDisplay Project
    Contributors: uknowmelast, Axiom
    Licensed under GPL v3
*/

#include "GPSReader.h"
#include "MultidisplayDefines.h"

#ifdef GPS_ENABLE

/* ── Global Instance ──────────────────────────────────────────────────── */
GPSReader gpsReader;

/* ── Constructor ──────────────────────────────────────────────────────── */
GPSReader::GPSReader() : _bufIdx(0), _receiving(false) {
    memset(&_data, 0, sizeof(_data));
}

/* ── begin() ─────────────────────────────────────────────────────────── */
void GPSReader::begin() {
    GPS_SERIAL.begin(GPS_BAUD);
    Serial.println(F("[GPS] NEO-6M on Serial3 @ 9600"));
}

/* ── poll() — non-blocking NMEA read ────────────────────────────────── */
void GPSReader::poll() {
    while (GPS_SERIAL.available()) {
        char c = GPS_SERIAL.read();

        if (c == '$') {
            // Start of a new sentence
            _bufIdx = 0;
            _receiving = true;
            _buf[_bufIdx++] = c;
        } else if (_receiving) {
            if (c == '\n' || c == '\r') {
                // End of sentence
                _buf[_bufIdx] = '\0';
                _receiving = false;
                if (_bufIdx > 6) {
                    processNMEA(_buf);
                }
                _bufIdx = 0;
            } else if (_bufIdx < GPS_NMEA_BUF_SIZE - 1) {
                _buf[_bufIdx++] = c;
            } else {
                // Buffer overflow, discard
                _receiving = false;
                _bufIdx = 0;
            }
        }
    }
}

/* ── hasFix() ────────────────────────────────────────────────────────── */
bool GPSReader::hasFix() const {
    return _data.fixValid && (millis() - _data.lastFixMillis < 3000);
}

/* ── processNMEA() — route to parser by sentence type ────────────────── */
void GPSReader::processNMEA(const char *sentence) {
    if (!verifyChecksum(sentence)) return;

    // $GPRMC or $GNRMC
    if ((sentence[3] == 'R' && sentence[4] == 'M' && sentence[5] == 'C')) {
        parseGPRMC(sentence);
    }
    // $GPGGA or $GNGGA
    else if ((sentence[3] == 'G' && sentence[4] == 'G' && sentence[5] == 'A')) {
        parseGPGGA(sentence);
    }
}

/* ── parseGPRMC() — Recommended Minimum sentence ────────────────────── */
// $GPRMC,HHMMSS.ss,A,ddmm.mmmm,N,dddmm.mmmm,W,knots,course,DDMMYY,...*XX
// Fields: 0=id, 1=time, 2=status, 3=lat, 4=N/S, 5=lon, 6=E/W,
//         7=speed, 8=course, 9=date, ...
void GPSReader::parseGPRMC(const char *sentence) {
    const char *f;

    // Field 2: Status (A=valid, V=void)
    f = getField(sentence, 2);
    if (!f) return;
    _data.fixValid = (f[0] == 'A') ? 1 : 0;

    if (!_data.fixValid) return;

    _data.lastFixMillis = millis();

    // Field 1: UTC time
    f = getField(sentence, 1);
    if (f) {
        // Parse HHMMSS (ignore fractional seconds)
        _data.utcTime = 0;
        for (uint8_t i = 0; i < 6 && f[i] >= '0' && f[i] <= '9'; i++) {
            _data.utcTime = _data.utcTime * 10 + (f[i] - '0');
        }
    }

    // Field 3,4: Latitude
    f = getField(sentence, 3);
    const char *d = getField(sentence, 4);
    if (f && d) {
        _data.latitude = parseLatLon(f, d);
    }

    // Field 5,6: Longitude
    f = getField(sentence, 5);
    d = getField(sentence, 6);
    if (f && d) {
        _data.longitude = parseLatLon(f, d);
    }

    // Field 7: Speed in knots
    f = getField(sentence, 7);
    if (f && f[0] != ',') {
        float kn = atof(f);
        _data.speedKnots100 = (uint16_t)(kn * 100);
        _data.speedKmh100   = (uint16_t)(kn * 185.2f);  // knots→km/h × 100
    }

    // Field 8: Course over ground
    f = getField(sentence, 8);
    if (f && f[0] != ',') {
        _data.course100 = (uint16_t)(atof(f) * 100);
    }

    // Field 9: Date DDMMYY
    f = getField(sentence, 9);
    if (f) {
        _data.utcDate = 0;
        for (uint8_t i = 0; i < 6 && f[i] >= '0' && f[i] <= '9'; i++) {
            _data.utcDate = _data.utcDate * 10 + (f[i] - '0');
        }
    }
}

/* ── parseGPGGA() — satellite count ──────────────────────────────────── */
// $GPGGA,time,lat,N,lon,E,fix,sats,hdop,alt,M,...*XX
// Field 7 = number of satellites
void GPSReader::parseGPGGA(const char *sentence) {
    const char *f = getField(sentence, 7);
    if (f) {
        _data.satellites = (uint8_t)atoi(f);
    }
}

/* ── serialWriteGPS() — pack 12 bytes into buffer ────────────────────── */
uint8_t GPSReader::serialWriteGPS(uint8_t *buf) const {
    // lat: int32 LE (4 bytes)
    buf[0] = _data.latitude & 0xFF;
    buf[1] = (_data.latitude >> 8) & 0xFF;
    buf[2] = (_data.latitude >> 16) & 0xFF;
    buf[3] = (_data.latitude >> 24) & 0xFF;
    // lon: int32 LE (4 bytes)
    buf[4] = _data.longitude & 0xFF;
    buf[5] = (_data.longitude >> 8) & 0xFF;
    buf[6] = (_data.longitude >> 16) & 0xFF;
    buf[7] = (_data.longitude >> 24) & 0xFF;
    // speedKmh100: uint16 LE (2 bytes)
    buf[8] = _data.speedKmh100 & 0xFF;
    buf[9] = (_data.speedKmh100 >> 8) & 0xFF;
    // course100: uint16 LE (2 bytes)
    buf[10] = _data.course100 & 0xFF;
    buf[11] = (_data.course100 >> 8) & 0xFF;
    return 12;
}

/* ── parseLatLon() — "ddmm.mmmm" + "N/S/E/W" → degrees × 10^6 ──────── */
int32_t GPSReader::parseLatLon(const char *field, const char *dir) {
    // Find the decimal point
    const char *dot = field;
    while (*dot && *dot != '.') dot++;
    if (!*dot) return 0;

    // Digits before dot: [dd]mm  (2 digits of minutes for lat, 3 for lon)
    int32_t raw = 0;
    const char *p = field;
    while (p < dot) {
        raw = raw * 10 + (*p - '0');
        p++;
    }

    // degrees = raw / 100, minutes = raw % 100
    int32_t degrees = raw / 100;
    int32_t minutes = raw % 100;

    // Fractional minutes: digits after the dot
    int32_t frac = 0;
    int32_t fracDiv = 1;
    p = dot + 1;
    while (*p >= '0' && *p <= '9' && fracDiv < 100000) {
        frac = frac * 10 + (*p - '0');
        fracDiv *= 10;
        p++;
    }

    // result = (degrees + minutes/60 + frac/(fracDiv*60)) × 1,000,000
    // = degrees * 1000000 + (minutes * 1000000 + frac * 1000000 / fracDiv) / 60
    int32_t result = degrees * 1000000L +
                     (minutes * 1000000L + (frac * 1000000L / fracDiv)) / 60;

    if (dir[0] == 'S' || dir[0] == 'W') result = -result;
    return result;
}

/* ── getField() — get pointer to Nth comma-delimited field ───────────── */
const char* GPSReader::getField(const char *sentence, uint8_t index) {
    uint8_t commas = 0;
    const char *p = sentence;
    while (*p) {
        if (*p == ',') {
            commas++;
            if (commas == index) return p + 1;
        }
        p++;
    }
    return NULL;
}

/* ── verifyChecksum() — XOR between $ and * vs hex after * ───────────── */
bool GPSReader::verifyChecksum(const char *sentence) {
    if (sentence[0] != '$') return false;

    uint8_t calc = 0;
    uint8_t i = 1;
    while (sentence[i] && sentence[i] != '*') {
        calc ^= sentence[i];
        i++;
    }
    if (sentence[i] != '*') return false;

    // Parse 2-char hex checksum
    uint8_t hi = sentence[i + 1];
    uint8_t lo = sentence[i + 2];
    if (hi >= 'A') hi = hi - 'A' + 10; else hi -= '0';
    if (lo >= 'A') lo = lo - 'A' + 10; else lo -= '0';
    uint8_t expected = (hi << 4) | lo;

    return (calc == expected);
}

#endif /* GPS_ENABLE */
