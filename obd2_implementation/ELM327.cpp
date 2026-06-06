/*
    ELM327.cpp — ELM327DSL IC Communication Driver Implementation

    Handles the UART protocol with the ELM327DSL IC wired to Serial1.
    All interaction is AT-command based text protocol:
      - Send "command\r"
      - Read response lines until '>' prompt
      - Parse hex data bytes from OBD responses

    Reference: ELM327DSL Datasheet (Elm Electronics)
    https://www.elmelectronics.com/wp-content/uploads/2020/05/ELM327DSL.pdf

    Init sequence follows datasheet recommendations:
      ATZ → AT BRD (baud upgrade) → ATE0 → ATL0 → ATS0 → ATH0
      → AT AT1 (adaptive timing) → ATST FF → ATSP n

    Copyright 2026 rAtTrax / MultiDisplay Project
    Contributors: uknowmelast, Axiom
    Licensed under GPL v3
*/

#include "ELM327.h"
#include "../MultidisplayDefines.h"

#ifdef OBD2_ELM327

/* ── Constructor ──────────────────────────────────────────────────────── */

ELM327::ELM327()
    : _serial(NULL)
    , _ready(false)
    , _lastStatus(ELM_NOT_INIT)
    , _timeout(OBD2_ELM_TIMEOUT_MS)
    , _retries(OBD2_ELM_RETRIES)
    , _activeBaud(0)
{
}

/* ── begin() ──────────────────────────────────────────────────────────── */
/*
 * ELM327DSL Init Sequence (per datasheet):
 *
 * 1. Auto-detect baud: try OBD2_ELM327_BAUD first, then common rates
 * 2. Wake-up CR  — ELM327DSL §3: bare CR wakes from low-power
 * 3. ATZ         — Full reset; response contains "ELM327 vX.X"
 * 4. AT BRD hh   — Baud Rate Divisor (§ "AT BRD"): upgrade to target baud
 *                  Divisor = 4,000,000 / target_baud
 *                  Handshake: send "AT BRD hh\r" → ELM replies "OK" at old
 *                  baud → both switch → host sends CR → ELM replies with
 *                  "ELM327..." at new baud to confirm
 * 5. ATE0        — Echo Off
 * 6. ATL0        — Linefeeds Off
 * 7. ATS0        — Spaces Off (compact hex responses)
 * 8. ATH0        — Headers Off
 * 9. AT AT1      — Adaptive Timing mode 1 (auto-shorten ELM-side timeout
 *                  after first successful response — faster polls)
 * 10. ATST FF    — Set ELM-side OBD timeout to max (255 × 4ms ≈ 1s)
 *                  AT AT1 will shorten this adaptively after first reply
 * 11. ATSP n     — Set Protocol (ISO 9141-2 = 3, Auto = 0)
 */
bool ELM327::begin(HardwareSerial &serial, uint32_t baud, uint8_t protocol)
{
    _serial = &serial;
    _ready = false;

    // Delay on first call so USB serial monitor can connect for diagnostics
    static bool _firstCall = true;
    if (_firstCall) {
        _firstCall = false;
        delay(2000);  // allow USB serial capture
    }

    /* ── Phase 1: Auto-detect current ELM327 baud rate ────────────────── */
    // Scan order: power-on baud first, then target baud (for PP/Pin6
    // changes from previous boot), then remaining common rates.
    static const uint32_t PROGMEM scanBauds[] = {
        0,       // slot 0 = baud parameter (power-on rate)
        0,       // slot 1 = OBD2_ELM327_TARGET_BAUD
        9600, 38400, 115200, 57600, 19200, 4800
    };

    uint8_t numBauds = sizeof(scanBauds) / sizeof(scanBauds[0]);
    bool foundELM = false;
    char buf[ELM_RESP_BUF_SIZE];

    // Use short timeout for the detection phase
    uint32_t origTimeout = _timeout;
    _timeout = 1500;  // 1.5s — enough for ATZ which resets the IC

    for (uint8_t bi = 0; bi < numBauds; bi++) {
        uint32_t tryBaud;
        if (bi == 0) tryBaud = baud;
#if defined(OBD2_ELM327_TARGET_BAUD) && (OBD2_ELM327_TARGET_BAUD > 0)
        else if (bi == 1) tryBaud = OBD2_ELM327_TARGET_BAUD;
#else
        else if (bi == 1) continue;
#endif
        else tryBaud = pgm_read_dword(&scanBauds[bi]);
        if (bi > 0 && tryBaud == baud) continue;  // already tried

        if (bi > 0) _serial->end();
        _serial->begin(tryBaud);

        Serial.print(F("[ELM] try "));
        Serial.print(tryBaud);
        Serial.print(F("..."));

        // UART + ELM settling time (longer on first try for power-on)
        delay(bi == 0 ? 500 : 200);

        // Flush stale RX data
        while (_serial->available()) _serial->read();

        // Wake-up CR per ELM327DSL §3 — bare CR wakes from low-power
        _serial->print('\r');
        delay(100);
        while (_serial->available()) _serial->read();

        // ATZ — full reset; response should contain "ELM327"
        uint8_t n = sendCommand("ATZ", buf, sizeof(buf));

        // Print compact diagnostic
        Serial.print(n);
        Serial.print(F("b:'"));
        for (uint8_t i = 0; i < n && i < 30; i++) {
            char c = buf[i];
            Serial.print((c >= 32 && c <= 126) ? c : '.');
        }
        Serial.println(F("'"));

        if (n == 0) continue;

        // Scan for "ELM" in response
        for (uint8_t i = 0; i + 2 < n; i++) {
            if (buf[i] == 'E' && buf[i+1] == 'L' && buf[i+2] == 'M') {
                foundELM = true;
                break;
            }
        }

        if (foundELM) {
            _activeBaud = tryBaud;
            Serial.print(F("[ELM] FOUND @ "));
            Serial.println(tryBaud);
            break;
        }
    }

    _timeout = origTimeout;  // restore for PID operations

    if (!foundELM) {
        Serial.println(F("[ELM] NOT FOUND"));
        _lastStatus = ELM_ERROR;
        return false;
    }

    /* ── Phase 2: Baud Rate Upgrade ──────────────────────────────────────
     * AT BRD divisor formula (from ELM327 datasheet §"Using Higher RS232
     * Baud Rates"):  Baud(kbps) = 4000 / hh   →   hh = round(4000000/baud)
     *   38400 → 0x68   (actual 38462, 0.16% err)
     *   57600 → 0x45   (actual 57971, 0.64% err)  — datasheet example
     *  115200 → 0x23   (actual 114286, 0.79% err)
     *
     * Strategy A — AT BRD (temporary):
     *   1. Host sends "AT BRD hh\r" at old baud
     *   2. ELM responds "OK\r" (NO '>' prompt!) at old baud
     *   3. ELM switches to new baud, waits BRT time (default 75ms)
     *   4. ELM sends ID string ("ELM327 v1.4\r") at new baud
     *   5. Host verifies ID, sends CR to confirm
     *   6. ELM responds "OK\r>" at new baud — handshake complete
     *   Auto-reverts after BRT timeout if handshake fails.
     *
     * Strategy B — AT PP 0C SV hh (permanent, ELM327DSL):
     *   Burns the divisor into EEPROM. Different PP for DIP vs DSL chips.
     *   PP 0C for ELM327DSL (v2.x), PP 00 for original ELM327 (v1.x).
     */
#if defined(OBD2_ELM327_TARGET_BAUD) && (OBD2_ELM327_TARGET_BAUD > 0)
    if (_activeBaud != OBD2_ELM327_TARGET_BAUD) {
        uint32_t target = OBD2_ELM327_TARGET_BAUD;

        // AT BRD divisor: hh = round(4,000,000 / baud)
        uint8_t divisor = (uint8_t)((4000000UL + target / 2) / target);

        // Log PP state for diagnostics
        {
            uint8_t pn = sendCommand("AT PPS", buf, sizeof(buf));
            Serial.print(F("[ELM] PPS: "));
            for (uint8_t i = 0; i < pn && i < 60; i++) {
                char c = buf[i];
                Serial.print((c >= 32 && c <= 126) ? c : '.');
            }
            Serial.println();
            Serial.flush();
        }

        Serial.print(F("[ELM] BRD div=0x"));
        if (divisor < 16) Serial.print('0');
        Serial.print(divisor, HEX);
        Serial.print(F(" for "));
        Serial.println(target);
        Serial.flush();

        // ── Strategy A: AT BRD (temporary baud switch) ──────────────
        // Protocol per ELM327 datasheet "Using Higher RS232 Baud Rates":
        //   ELM sends "OK\r" at old baud (NO '>' prompt!), then switches.
        //   After BRT wait, ELM sends ID string at NEW baud.
        //   Host verifies ID, sends CR to confirm.
        //   ELM responds "OK\r>" at new baud.
        sendCommand("AT BRT FF", buf, sizeof(buf));  // max handshake timeout

        char brdCmd[12];
        snprintf(brdCmd, sizeof(brdCmd), "AT BRD %02X", divisor);

        // Flush RX, send BRD command
        while (_serial->available()) _serial->read();
        _serial->print(brdCmd);
        _serial->print('\r');
        _serial->flush();  // ensure TX complete

        // Read response at OLD baud — stop at "OK" (no '>' comes!)
        uint8_t rn = 0;
        unsigned long tCmd = millis();
        bool gotOK = false;
        while ((millis() - tCmd) < 1000 && rn < sizeof(buf) - 1) {
            if (_serial->available()) {
                char c = _serial->read();
                if (c != '\r' && c != '\n')
                    buf[rn++] = c;
                if (rn >= 2 && buf[rn-2] == 'O' && buf[rn-1] == 'K') {
                    gotOK = true;
                    break;
                }
            }
        }
        buf[rn] = '\0';

        Serial.print(F("[ELM] BRD resp='"));
        for (uint8_t i = 0; i < rn && i < 30; i++) {
            char c = buf[i];
            Serial.print((c >= 32 && c <= 126) ? c : '.');
        }
        Serial.print(F("' OK="));
        Serial.println(gotOK ? F("yes") : F("no"));
        Serial.flush();

        bool upgraded = false;
        if (gotOK) {
            // ELM has switched to new baud. Switch ours IMMEDIATELY.
            // (No prints, no delays between OK and begin!)
            _serial->begin(target);
            while (_serial->available()) _serial->read();  // discard old-baud remnants

            // ELM sends ID string after BRT wait. Read it at new baud.
            uint8_t cn = 0;
            unsigned long tS = millis();
            while ((millis() - tS) < 3000 && cn < sizeof(buf) - 1) {
                if (_serial->available()) {
                    char c = _serial->read();
                    buf[cn++] = c;
                    // ID ends with CR
                    if (cn > 5 && c == '\r') break;
                }
            }
            buf[cn] = '\0';

            // Check if we received the ELM327 ID at new baud
            bool gotID = false;
            for (uint8_t i = 0; i + 2 < cn; i++) {
                if (buf[i] == 'E' && buf[i+1] == 'L' && buf[i+2] == 'M') {
                    gotID = true;
                    break;
                }
            }

            // Hex dump of received ID
            Serial.print(F("[ELM] BRD ID["));
            Serial.print(cn);
            Serial.print(F("]:"));
            for (uint8_t i = 0; i < cn && i < 20; i++) {
                Serial.print(' ');
                if ((uint8_t)buf[i] < 0x10) Serial.print('0');
                Serial.print((uint8_t)buf[i], HEX);
            }
            Serial.println();
            Serial.flush();

            if (gotID) {
                // Send confirmation CR — ELM expects this within BRT timeout
                _serial->print('\r');

                // Read "OK" + prompt at new baud
                uint8_t okn = 0;
                unsigned long tOK = millis();
                while ((millis() - tOK) < 2000 && okn < sizeof(buf) - 1) {
                    if (_serial->available()) {
                        char c = _serial->read();
                        if (c == '>') break;
                        if (c != '\r' && c != '\n') buf[okn++] = c;
                    }
                }
                buf[okn] = '\0';

                for (uint8_t i = 0; i + 1 < okn; i++) {
                    if (buf[i] == 'O' && buf[i+1] == 'K') {
                        upgraded = true;
                        break;
                    }
                }

                if (upgraded) {
                    _activeBaud = target;
                    Serial.print(F("[ELM] BRD OK @ "));
                    Serial.println(target);
                } else {
                    Serial.print(F("[ELM] BRD confirm='"));
                    for (uint8_t i = 0; i < okn && i < 20; i++)
                        Serial.print(buf[i]);
                    Serial.println(F("'"));
                }
            } else {
                Serial.println(F("[ELM] BRD no ID at new baud"));
            }

            if (!upgraded) {
                // Handshake failed — ELM reverts after BRT timeout
                Serial.println(F("[ELM] BRD FAIL — reverting"));
                delay(2000);
                _serial->begin(_activeBaud);
                delay(100);
                while (_serial->available()) _serial->read();
            }
        } else {
            Serial.println(F("[ELM] BRD not accepted"));
        }

        // ── Strategy B: PP 0C permanent baud (if BRD failed) ────────
        // PP 0C (DSL) or PP 00 (DIP v1.x) — same divisor as BRD.
        if (!upgraded) {
            Serial.println(F("[ELM] Trying PP 0C permanent baud"));
            Serial.flush();

            char ppCmd[16];
            snprintf(ppCmd, sizeof(ppCmd), "AT PP 0C SV %02X", divisor);
            uint8_t ppn = sendCommand(ppCmd, buf, sizeof(buf));

            Serial.print(F("[ELM] PP0C SV resp["));
            Serial.print(ppn);
            Serial.print(F("]: "));
            for (uint8_t i = 0; i < ppn && i < 30; i++) {
                char c = buf[i];
                Serial.print((c >= 32 && c <= 126) ? c : '.');
            }
            Serial.println();
            Serial.flush();

            bool ppOK = false;
            for (uint8_t i = 0; i + 1 < ppn; i++) {
                if (buf[i] == 'O' && buf[i+1] == 'K') { ppOK = true; break; }
            }
            if (ppOK) {
                sendCommand("AT PP 0C ON", buf, sizeof(buf));

                // ATZ resets — chip comes up at new baud from PP 0C
                _serial->print("ATZ\r");
                _serial->flush();
                _serial->end();
                _serial->begin(target);

                uint8_t cn2 = 0;
                unsigned long tR = millis();
                while ((millis() - tR) < 3000 && cn2 < sizeof(buf) - 1) {
                    if (_serial->available()) {
                        char c = _serial->read();
                        if (c == '>') break;
                        if (c != '\r' && c != '\n') buf[cn2++] = c;
                    }
                }
                buf[cn2] = '\0';

                Serial.print(F("[ELM] PP0C resp["));
                Serial.print(cn2);
                Serial.print(F("]: "));
                for (uint8_t i = 0; i < cn2 && i < 30; i++) {
                    char c = buf[i];
                    Serial.print((c >= 32 && c <= 126) ? c : '.');
                }
                Serial.println();
                Serial.flush();

                for (uint8_t i = 0; i + 2 < cn2; i++) {
                    if (buf[i]=='E' && buf[i+1]=='L' && buf[i+2]=='M') {
                        upgraded = true;
                        break;
                    }
                }

                if (upgraded) {
                    _activeBaud = target;
                    Serial.print(F("[ELM] PP 0C OK @ "));
                    Serial.println(target);
                } else {
                    Serial.println(F("[ELM] PP 0C baud FAIL"));
                    // Try to undo at both bauds
                    sendCommand("AT PP 0C OFF", buf, sizeof(buf));
                    _serial->print("ATZ\r");
                    _serial->flush();
                    _serial->end();
                    _serial->begin(_activeBaud);
                    delay(2000);
                    while (_serial->available()) _serial->read();
                    sendCommand("AT PP 0C OFF", buf, sizeof(buf));
                    sendCommand("ATZ", buf, sizeof(buf));
                }
            } else {
                Serial.println(F("[ELM] PP 0C not supported"));
            }
        }

        if (!upgraded) {
            Serial.print(F("[ELM] baud stays at "));
            Serial.println(_activeBaud);
        }
    }
#endif

    /* ── Phase 3: Configure ELM327 for OBD operation ──────────────────── */

    // ATE0 — Echo Off (ELM327DSL §"AT E0")
    // Verify it worked: send ATI and check if echo is present
    sendCommand("ATE0", buf, sizeof(buf));
    {
        uint8_t rn = sendCommand("ATI", buf, sizeof(buf));
        // If echo is still on, response starts with "ATI" text
        bool echoOn = false;
        if (rn >= 3 && buf[0] == 'A' && buf[1] == 'T' && buf[2] == 'I') {
            echoOn = true;
            // Retry ATE0
            sendCommand("ATE0", buf, sizeof(buf));
            rn = sendCommand("ATI", buf, sizeof(buf));
            if (rn >= 3 && buf[0] == 'A' && buf[1] == 'T' && buf[2] == 'I') {
                Serial.println(F("[ELM] WARN: echo still on"));
            } else {
                Serial.println(F("[ELM] ATE0 OK on 2nd try"));
            }
        }
    }

    // ATL0 — Linefeeds Off (ELM327DSL §"AT L0")
    sendCommand("ATL0", buf, sizeof(buf));

    // ATS0 — Spaces Off / compact hex (ELM327DSL §"AT S0")
    sendCommand("ATS0", buf, sizeof(buf));

    // ATH0 — Headers Off (ELM327DSL §"AT H0")
    sendCommand("ATH0", buf, sizeof(buf));

    // AT AT2 — Adaptive Timing aggressive (ELM327DSL §"AT AT"):
    // More aggressively shortens the ELM-side OBD timeout after
    // successful responses — maximizes polling throughput for racing.
    sendCommand("ATAT2", buf, sizeof(buf));

    // ATST 32 — Set ELM-side OBD timeout to 50 × 4ms = 200ms
    // ISO 9141-2 typical response is 50-150ms; AT AT1 adapts down further.
    // Lower base timeout means faster failure detection for unsupported PIDs.
    sendCommand("ATST32", buf, sizeof(buf));

    // ATSP<protocol> — Set Protocol (ELM327DSL §"AT SP")
    char protoCmd[8];
    snprintf(protoCmd, sizeof(protoCmd), "ATSP%d", protocol);
    Serial.print(F("[ELM] "));
    Serial.println(protoCmd);
    sendCommand(protoCmd, buf, sizeof(buf));

    _ready = true;
    _lastStatus = ELM_OK;
    Serial.print(F("[ELM] READY @ "));
    Serial.println(_activeBaud);
    return true;
}

/* ── end() ────────────────────────────────────────────────────────────── */

void ELM327::end()
{
    if (_serial) {
        _serial->flush();
    }
    _ready = false;
    _lastStatus = ELM_NOT_INIT;
}

/* ── requestPID() — Mode 01 convenience ───────────────────────────────── */

bool ELM327::requestPID(uint8_t pid, OBD2PIDResponse *resp)
{
    return requestServicePID(0x01, pid, resp);
}

/* ── requestServicePID() — Any mode ───────────────────────────────────── */

bool ELM327::requestServicePID(uint8_t service, uint8_t pid, OBD2PIDResponse *resp)
{
    if (!_ready || !resp) return false;

    char cmd[8];
    snprintf(cmd, sizeof(cmd), "%02X%02X", service, pid);

    char buf[ELM_RESP_BUF_SIZE];
    uint8_t attempts = _retries + 1;

    while (attempts-- > 0) {
        uint8_t n = sendCommand(cmd, buf, sizeof(buf));
        if (n == 0) {
            _lastStatus = ELM_TIMEOUT;
            continue;
        }

        ELM327Status st = checkResponse(buf);
        if (st == ELM_BUS_INIT) {
            // ELM327DSL: "BUS INIT: ...OK" on first OBD request.
            // The actual PID response may follow in the same buffer.
            // Try to parse it before retrying.
            if (parseOBDResponse(buf, n, resp)) {
                _lastStatus = ELM_OK;
                return true;
            }
            _lastStatus = ELM_BUS_INIT;
            continue;
        }
        if (st != ELM_OK) {
            _lastStatus = st;
            continue;
        }

        if (parseOBDResponse(buf, n, resp)) {
            _lastStatus = ELM_OK;
            return true;
        }

        // ELM said OK but we can't parse the hex data — dump for diagnosis
        _lastStatus = ELM_PARSE_FAIL;
        Serial.print(F("[ELM] raw '"));
        for (uint8_t i = 0; i < n && i < 40; i++) {
            char c = buf[i];
            Serial.print((c >= 32 && c <= 126) ? c : '.');
        }
        Serial.print(F("' n="));
        Serial.println(n);
    }

    return false;
}

/* ── readDTCs() — Service 03 ──────────────────────────────────────────── */

uint8_t ELM327::readDTCs(OBD2DTC *dtcs, uint8_t maxCount)
{
    if (!_ready || !dtcs || maxCount == 0) return 0;

    char buf[ELM_RESP_BUF_SIZE];
    uint8_t n = sendCommand("03", buf, sizeof(buf));
    if (n == 0) return 0;

    if (checkResponse(buf) != ELM_OK) return 0;

    // Response: "43XXYY..." where each XXYY = one DTC (2 bytes)
    // Skip response header "43"
    const char *ptr = buf;
    // Find "43" response code
    while (*ptr && !(*ptr == '4' && *(ptr+1) == '3')) ptr++;
    if (*ptr != '4') return 0;
    ptr += 2; // skip "43"

    uint8_t count = 0;
    while (*ptr && count < maxCount) {
        // Need 4 hex chars for one DTC
        if (!ptr[0] || !ptr[1] || !ptr[2] || !ptr[3]) break;

        uint8_t hi = hexToByte(ptr[0], ptr[1]);
        uint8_t lo = hexToByte(ptr[2], ptr[3]);
        ptr += 4;

        if (hi == 0 && lo == 0) continue; // Null DTC = padding

        // Decode DTC: first 2 bits of hi = system, rest = code
        static const char system_codes[] = {'P', 'C', 'B', 'U'};
        char sys = system_codes[(hi >> 6) & 0x03];
        uint8_t d1 = (hi >> 4) & 0x03;
        uint8_t d2 = hi & 0x0F;
        uint8_t d3 = (lo >> 4) & 0x0F;
        uint8_t d4 = lo & 0x0F;

        dtcs[count].code[0] = sys;
        dtcs[count].code[1] = '0' + d1;
        dtcs[count].code[2] = (d2 < 10) ? ('0' + d2) : ('A' + d2 - 10);
        dtcs[count].code[3] = (d3 < 10) ? ('0' + d3) : ('A' + d3 - 10);
        dtcs[count].code[4] = (d4 < 10) ? ('0' + d4) : ('A' + d4 - 10);
        dtcs[count].code[5] = '\0';
        count++;
    }

    return count;
}

/* ── clearDTCs() — Service 04 ─────────────────────────────────────────── */

bool ELM327::clearDTCs()
{
    if (!_ready) return false;

    char buf[32];
    sendCommand("04", buf, sizeof(buf));
    return (checkResponse(buf) == ELM_OK);
}

/* ── getVIN() — Service 09, PID 02 ───────────────────────────────────── */

bool ELM327::getVIN(OBD2VIN *vin)
{
    if (!_ready || !vin) return false;

    vin->valid = false;
    memset(vin->vin, 0, sizeof(vin->vin));

    char buf[ELM_RESP_BUF_SIZE];
    uint8_t n = sendCommand("0902", buf, sizeof(buf));
    if (n == 0) return false;
    if (checkResponse(buf) != ELM_OK) return false;

    // Response format: "49020149..." where 4902 = response header,
    // 01 = message count, followed by hex-encoded VIN bytes.
    // VIN = 17 ASCII characters = 34 hex chars after header.
    // Find "4902" response code
    const char *ptr = buf;
    while (*ptr && !(*ptr == '4' && *(ptr+1) == '9' && *(ptr+2) == '0' && *(ptr+3) == '2')) ptr++;
    if (*ptr != '4') return false;
    ptr += 4; // skip "4902"

    // Skip message count byte (01 = single frame, 00 = multi-frame)
    if (ptr[0] && ptr[1]) ptr += 2;

    // Decode hex pairs to ASCII characters
    uint8_t idx = 0;
    while (*ptr && idx < 17) {
        if (!ptr[0] || !ptr[1]) break;
        // Skip non-hex characters (spaces, line breaks from multi-line responses)
        if ((ptr[0] >= '0' && ptr[0] <= '9') || (ptr[0] >= 'A' && ptr[0] <= 'F') ||
            (ptr[0] >= 'a' && ptr[0] <= 'f')) {
            uint8_t ch = hexToByte(ptr[0], ptr[1]);
            if (ch >= 0x20 && ch <= 0x7E) { // printable ASCII only
                vin->vin[idx++] = (char)ch;
            }
            ptr += 2;
        } else {
            ptr++; // skip non-hex char
        }
    }
    vin->vin[idx] = '\0';
    vin->valid = (idx == 17);
    return vin->valid;
}

/* ── getMILStatus() — Service 01, PID 01 ──────────────────────────────── */

int8_t ELM327::getMILStatus(bool *mil_on)
{
    if (!_ready) return -1;

    OBD2PIDResponse resp;
    if (!requestPID(0x01, &resp)) return -1;
    if (resp.dataLen < 4) return -1;

    // Byte A: bit 7 = MIL on/off, bits 0-6 = DTC count
    if (mil_on) *mil_on = (resp.data[0] & 0x80) != 0;
    return (int8_t)(resp.data[0] & 0x7F);
}

/* ── getSupportedPIDs_00_20() ─────────────────────────────────────────── */

bool ELM327::getSupportedPIDs_00_20(uint32_t *bitmap)
{
    if (!_ready || !bitmap) return false;

    OBD2PIDResponse resp;
    if (!requestPID(0x00, &resp)) return false;
    if (resp.dataLen < 4) return false;

    *bitmap = ((uint32_t)resp.data[0] << 24) |
              ((uint32_t)resp.data[1] << 16) |
              ((uint32_t)resp.data[2] << 8)  |
              ((uint32_t)resp.data[3]);
    return true;
}

/* ── sendATCommand() ──────────────────────────────────────────────────── */

uint8_t ELM327::sendATCommand(const char *cmd, char *buf, uint8_t bufLen)
{
    return sendCommand(cmd, buf, bufLen);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Private Methods
 * ══════════════════════════════════════════════════════════════════════════ */

/* ── sendCommand() ────────────────────────────────────────────────────── */

uint8_t ELM327::sendCommand(const char *cmd, char *buf, uint8_t bufLen)
{
    if (!_serial) return 0;

    // Flush input
    while (_serial->available()) _serial->read();

    // Send command with carriage return
    _serial->print(cmd);
    _serial->print('\r');

    // Read response until '>' prompt or timeout
    uint8_t idx = 0;
    unsigned long start = millis();

    while ((millis() - start) < _timeout) {
        if (_serial->available()) {
            char c = _serial->read();

            // '>' is the ELM327 prompt — end of response
            if (c == '>') break;

            // Skip \r and \n for cleaner parsing
            if (c == '\r' || c == '\n') continue;

            if (idx < bufLen - 1) {
                buf[idx++] = c;
            }
        }
    }

    buf[idx] = '\0';
    return idx;
}

/* ── parseOBDResponse() ───────────────────────────────────────────────── */

bool ELM327::parseOBDResponse(const char *buf, uint8_t bufLen, OBD2PIDResponse *resp)
{
    // Response format (spaces off): "410CABCD"
    // First byte = service + 0x40, second byte = PID, rest = data
    // Minimum valid response: 4 hex chars (2 bytes: service+pid)
    //
    // If ATE0 (Echo Off) failed, the buffer starts with echo:
    //   "SSPP" (4 chars, e.g. "010C") or "SS PP" (5 chars with spaces)
    // followed by the actual response "410CABCD".
    //
    // Strategy: try parsing at fixed offsets first (fast path),
    // then fall back to a full scan (handles BUS INIT text etc.)
    if (bufLen < 4) return false;

    // Try well-known offsets: 0 (no echo), 4 (echo, no spaces), 5 (echo, spaces)
    static const uint8_t offsets[] = {0, 4, 5};
    for (uint8_t oi = 0; oi < 3; oi++) {
        uint8_t off = offsets[oi];
        if (off + 4 > bufLen) continue;

        const char *p = buf + off;
        // Must start with hex digit
        if (!((p[0] >= '0' && p[0] <= '9') || (p[0] >= 'A' && p[0] <= 'F')
           || (p[0] >= 'a' && p[0] <= 'f'))) continue;

        uint8_t svc = hexToByte(p[0], p[1]);
        if (svc < 0x41 || svc > 0x4F) continue;  // Not a valid OBD response

        if (!p[2] || !p[3]) continue;
        if (tryParseAt(p, buf + bufLen, resp)) return true;
    }

    // Fallback: scan for first hex pair with value in [0x41 .. 0x4F]
    // This handles BUS INIT: "BUS INIT: ...OK410C1AF8" where the
    // non-hex text is naturally skipped.
    for (const char *p = buf; p + 3 < buf + bufLen; p++) {
        if (!((p[0] >= '0' && p[0] <= '9') || (p[0] >= 'A' && p[0] <= 'F')
           || (p[0] >= 'a' && p[0] <= 'f'))) continue;
        if (!p[1]) break;

        uint8_t svc = hexToByte(p[0], p[1]);
        if (svc < 0x41 || svc > 0x4F) continue;
        if (!p[2] || !p[3]) break;
        if (tryParseAt(p, buf + bufLen, resp)) return true;
    }

    return false;
}

/* ── tryParseAt() — helper: parse OBD hex starting at p ───────────────── */

bool ELM327::tryParseAt(const char *p, const char *end, OBD2PIDResponse *resp)
{
    resp->service = hexToByte(p[0], p[1]);
    resp->pid     = hexToByte(p[2], p[3]);

    resp->dataLen = 0;
    p += 4;
    for (uint8_t i = 0; i < 4 && p + 1 < end && p[0] && p[1]; i++) {
        if (!((p[0] >= '0' && p[0] <= '9') || (p[0] >= 'A' && p[0] <= 'F')
           || (p[0] >= 'a' && p[0] <= 'f'))) break;
        resp->data[i] = hexToByte(p[0], p[1]);
        resp->dataLen++;
        p += 2;
    }

    resp->value = 0.0f;
    return (resp->dataLen > 0);  // Must have at least 1 data byte
}

/* ── hexToByte() ──────────────────────────────────────────────────────── */

uint8_t ELM327::hexToByte(char hi, char lo)
{
    uint8_t val = 0;
    if (hi >= '0' && hi <= '9') val = (hi - '0') << 4;
    else if (hi >= 'A' && hi <= 'F') val = (hi - 'A' + 10) << 4;
    else if (hi >= 'a' && hi <= 'f') val = (hi - 'a' + 10) << 4;

    if (lo >= '0' && lo <= '9') val |= (lo - '0');
    else if (lo >= 'A' && lo <= 'F') val |= (lo - 'A' + 10);
    else if (lo >= 'a' && lo <= 'f') val |= (lo - 'a' + 10);

    return val;
}

/* ── checkResponse() ──────────────────────────────────────────────────── */

ELM327Status ELM327::checkResponse(const char *buf)
{
    if (!buf || !buf[0]) return ELM_TIMEOUT;

    // Check for known error strings
    // Use simple substring search (no strstr to reduce overhead)
    for (const char *p = buf; *p; p++) {
        if (p[0] == 'N' && p[1] == 'O') {
            // "NO DATA" or "NODATA"
            if (p[2] == ' ' || p[2] == 'D') return ELM_NO_DATA;
        }
        if (p[0] == 'E' && p[1] == 'R' && p[2] == 'R') {
            return ELM_ERROR; // "ERROR"
        }
        if (p[0] == 'U' && p[1] == 'N' && p[2] == 'A') {
            return ELM_ERROR; // "UNABLE TO CONNECT"
        }
        if (p[0] == 'B' && p[1] == 'U' && p[2] == 'S') {
            // "BUS INIT:" — happens on first query, normal
            return ELM_BUS_INIT;
        }
        if (p[0] == '?' ) {
            return ELM_ERROR; // Unrecognized command
        }
    }

    return ELM_OK;
}

#endif /* OBD2_ELM327 */
