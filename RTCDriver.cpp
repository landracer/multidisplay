/*
    RTCDriver.cpp — DS3231 Real-Time Clock Driver Implementation

    Communicates with DS3231 via I2C (Wire library, already initialized
    by MultidisplayController). Reads BCD-encoded time registers 0x00-0x06.

    Copyright 2026 rAtTrax / MultiDisplay Project
    Contributors: uknowmelast, Axiom
    Licensed under GPL v3
*/

#include "RTCDriver.h"
#include "MultidisplayDefines.h"

#ifdef RTC_ENABLE

#include <Wire.h>

/* ── Global Instance ──────────────────────────────────────────────────── */
RTCDriver rtcDriver;

/* ── Constructor ──────────────────────────────────────────────────────── */
RTCDriver::RTCDriver() : _ready(false) {
    memset(&_time, 0, sizeof(_time));
}

/* ── BCD helpers ──────────────────────────────────────────────────────── */
uint8_t RTCDriver::bcd2dec(uint8_t bcd) {
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

uint8_t RTCDriver::dec2bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

/* ── begin() — probe DS3231, return true if it ACKs ──────────────────── */
bool RTCDriver::begin() {
    Wire.beginTransmission(RTC_DS3231_ADDR);
    _ready = (Wire.endTransmission() == 0);
    if (_ready) {
        readTime();
        Serial.print(F("[RTC] DS3231 found, time: "));
        char buf[18];
        formatTimestamp(buf);
        Serial.println(buf);
    } else {
        Serial.println(F("[RTC] DS3231 not found on I2C"));
    }
    return _ready;
}

/* ── readTime() — read 7 registers starting at 0x00 ─────────────────── */
bool RTCDriver::readTime() {
    Wire.beginTransmission(RTC_DS3231_ADDR);
    Wire.send(0x00);  // register pointer to seconds
    if (Wire.endTransmission() != 0) {
        _ready = false;
        return false;
    }

    Wire.requestFrom((uint8_t)RTC_DS3231_ADDR, (uint8_t)7);
    if (Wire.available() < 7) {
        _ready = false;
        return false;
    }

    _time.seconds   = bcd2dec(Wire.receive() & 0x7F);
    _time.minutes   = bcd2dec(Wire.receive() & 0x7F);
    _time.hours     = bcd2dec(Wire.receive() & 0x3F);  // 24h mode
    _time.dayOfWeek = bcd2dec(Wire.receive() & 0x07);
    _time.day       = bcd2dec(Wire.receive() & 0x3F);
    _time.month     = bcd2dec(Wire.receive() & 0x1F);
    _time.year      = bcd2dec(Wire.receive());

    _ready = true;
    return true;
}

/* ── setTime() ───────────────────────────────────────────────────────── */
void RTCDriver::setTime(const RTCTime &t) {
    Wire.beginTransmission(RTC_DS3231_ADDR);
    Wire.send(0x00);
    Wire.send(dec2bcd(t.seconds));
    Wire.send(dec2bcd(t.minutes));
    Wire.send(dec2bcd(t.hours));
    Wire.send(dec2bcd(t.dayOfWeek));
    Wire.send(dec2bcd(t.day));
    Wire.send(dec2bcd(t.month));
    Wire.send(dec2bcd(t.year));
    Wire.endTransmission();
    _time = t;
    _ready = true;
}

/* ── setFromGPS() — set RTC from NMEA UTC time/date ─────────────────── */
void RTCDriver::setFromGPS(uint32_t utcTime, uint32_t utcDate) {
    RTCTime t;
    // utcTime = HHMMSS (e.g. 123456 = 12:34:56)
    t.hours   = utcTime / 10000;
    t.minutes = (utcTime / 100) % 100;
    t.seconds = utcTime % 100;
    // utcDate = DDMMYY (e.g. 300326 = 30-Mar-26)
    t.day   = utcDate / 10000;
    t.month = (utcDate / 100) % 100;
    t.year  = utcDate % 100;
    t.dayOfWeek = 0;  // not computed from GPS
    setTime(t);
    Serial.println(F("[RTC] Time synced from GPS"));
}

/* ── formatTime() ────────────────────────────────────────────────────── */
void RTCDriver::formatTime(char *buf) const {
    buf[0] = '0' + (_time.hours / 10);
    buf[1] = '0' + (_time.hours % 10);
    buf[2] = ':';
    buf[3] = '0' + (_time.minutes / 10);
    buf[4] = '0' + (_time.minutes % 10);
    buf[5] = ':';
    buf[6] = '0' + (_time.seconds / 10);
    buf[7] = '0' + (_time.seconds % 10);
    buf[8] = '\0';
}

/* ── formatDate() ────────────────────────────────────────────────────── */
void RTCDriver::formatDate(char *buf) const {
    buf[0] = '0' + (_time.year / 10);
    buf[1] = '0' + (_time.year % 10);
    buf[2] = '-';
    buf[3] = '0' + (_time.month / 10);
    buf[4] = '0' + (_time.month % 10);
    buf[5] = '-';
    buf[6] = '0' + (_time.day / 10);
    buf[7] = '0' + (_time.day % 10);
    buf[8] = '\0';
}

/* ── formatTimestamp() ───────────────────────────────────────────────── */
void RTCDriver::formatTimestamp(char *buf) const {
    formatDate(buf);
    buf[8] = ' ';
    formatTime(buf + 9);
}

#endif /* RTC_ENABLE */
