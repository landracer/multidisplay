/*
    RTCDriver.h — DS3231 Real-Time Clock Driver (I2C)
    Reads date/time from a DS3231 module on the I2C bus.
    Used for timestamping SD log entries and GPS time sync.

    Copyright 2026 rAtTrax / MultiDisplay Project
    Contributors: uknowmelast, Axiom
    Licensed under GPL v3
*/

#ifndef RTCDRIVER_H_
#define RTCDRIVER_H_

#include <Arduino.h>
#include <stdint.h>

/* DS3231 I2C address (fixed by hardware) */
#define RTC_DS3231_ADDR  0x68

struct RTCTime {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t dayOfWeek;
    uint8_t day;
    uint8_t month;
    uint8_t year;       // 0-99 (offset from 2000)
};

class RTCDriver {
public:
    RTCDriver();

    /** Call once from myconstructor(). Returns true if DS3231 responds. */
    bool begin();

    /** Read current time from DS3231. Returns true on success. */
    bool readTime();

    /** Set the DS3231 time. */
    void setTime(const RTCTime &t);

    /** Set time from GPS UTC: HHMMSS and DDMMYY. */
    void setFromGPS(uint32_t utcTime, uint32_t utcDate);

    /** Returns true if begin() succeeded and last readTime() was OK. */
    bool isReady() const { return _ready; }

    /** Current cached time (updated by readTime()). */
    const RTCTime& time() const { return _time; }

    /** Format time into buffer as "HH:MM:SS" (9 bytes incl NUL). */
    void formatTime(char *buf) const;

    /** Format date into buffer as "YY-MM-DD" (9 bytes incl NUL). */
    void formatDate(char *buf) const;

    /** Format full timestamp "YY-MM-DD HH:MM:SS" (18 bytes incl NUL). */
    void formatTimestamp(char *buf) const;

private:
    RTCTime  _time;
    bool     _ready;

    static uint8_t bcd2dec(uint8_t bcd);
    static uint8_t dec2bcd(uint8_t dec);
};

extern RTCDriver rtcDriver;

#endif /* RTCDRIVER_H_ */
