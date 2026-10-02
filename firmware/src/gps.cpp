#include "gps.h"
#include "config.h"
#include <HardwareSerial.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static void gpsTask(void *);
static void gpsPoll();

// Lightweight NMEA GGA/RMC parser. Works with $GP, $GN, $GL, $GA talkers
// (u-blox NEO-6M / 7M / M8N / M10). No extra Arduino library.

static HardwareSerial GpsUart(1);

static portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED;
static bool haveFix = false;
static double latDeg = 0;
static double lonDeg = 0;
static float altM = 0;
static float spdKmh = 0;
static float crsDeg = 0;
static int sats = 0;
static uint32_t fixAt = 0;
static uint32_t nmeaAt = 0;
static uint32_t byteCount = 0;     // raw bytes seen since boot — drives baud probe

static char lineBuf[128];
static size_t lineLen = 0;

// Baud candidates tried when GPS_BAUD == 0 in config.h. NEO-7M clones ship
// from the factory at 9600 8N1, but a lot of cheap boards are flashed to
// 38400 or 115200 first; try in the order most NMEA streams show up on.
static const long kProbeBauds[] = { 9600, 38400, 115200, 4800, 19200 };
static const int kProbeCount = sizeof(kProbeBauds) / sizeof(kProbeBauds[0]);
static int probeIdx = 0;
static bool probeDone = false;
static uint32_t probeStartMs = 0;
static long activeBaud = 0;

static bool nmeaChecksumOk(const char *line) {
  const char *star = strrchr(line, '*');
  if (!star || star <= line) return false;
  uint8_t cs = 0;
  for (const char *p = line + 1; p < star; p++) {
    cs ^= (uint8_t)*p;
  }
  unsigned int got = 0;
  if (sscanf(star + 1, "%2x", &got) != 1) return false;
  return cs == (uint8_t)got;
}

static double nmeaToDeg(const char *nmea, char hemi) {
  if (!nmea || !nmea[0] || hemi == 0) return 0;
  double v = atof(nmea);
  int deg = (int)(v / 100.0);
  double minutes = v - (double)deg * 100.0;
  double d = (double)deg + minutes / 60.0;
  if (hemi == 'S' || hemi == 'W') d = -d;
  return d;
}

static bool talkerIs(const char *line, const char *sentence) {
  // $GPGGA / $GNGGA / $GLGGA ...
  if (!line || line[0] != '$' || strlen(line) < 6) return false;
  return strncmp(line + 3, sentence, 3) == 0;
}

static int splitCsv(char *s, char *fields[], int maxFields) {
  int n = 0;
  char *p = s;
  while (n < maxFields) {
    fields[n++] = p;
    char *comma = strchr(p, ',');
    if (!comma) break;
    *comma = 0;
    p = comma + 1;
  }
  return n;
}

static void noteNmea(int nsat) {
  portENTER_CRITICAL(&gpsMux);
  nmeaAt = millis();
  if (nsat >= 0) sats = nsat;
  portEXIT_CRITICAL(&gpsMux);
}

// Raw-talker-id detection: works for $GPGGA / $GNGGA / $GLGGA / $GAGGA and
// the same family on RMC. strncmp(line+3, "GGA", 3) matches every talker
// because the third-through-fifth chars are what NMEA defines as the
// sentence id, regardless of the leading two-letter talker prefix.
static const char *kKnownGga = "GGA";
static const char *kKnownRmc = "RMC";

static void applyFix(double lat, double lon, float alt, float spd, float crs, int nsat, bool valid) {
  if (!valid) return;
  if (lat == 0.0 && lon == 0.0) return;
  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return;
  portENTER_CRITICAL(&gpsMux);
  haveFix = true;
  latDeg = lat;
  lonDeg = lon;
  if (alt != -9999.0f) altM = alt;
  if (spd >= 0.0f) spdKmh = spd;
  if (crs >= 0.0f) crsDeg = crs;
  if (nsat >= 0) sats = nsat;
  fixAt = millis();
  nmeaAt = fixAt;
  portEXIT_CRITICAL(&gpsMux);
}

static void parseGsv(char *line) {
  // $GPGSV,totalMsgs,msgNum,satsInView,(sat1,elev1,azim1,snr1)*cs
  // field[3] is the "satellites in view" count. The message is split across
  // multiple sentences (max 3 sats each) so we only update the counter when
  // we see the first sentence of a batch (msgNum == 1) to avoid double
  // counting the same satellites.
  char *star = strchr(line, '*');
  if (star) *star = 0;
  char *f[20];
  int n = splitCsv(line, f, 20);
  if (n < 4) return;
  int msgNum = atoi(f[2]);
  int inView = atoi(f[3]);
  if (msgNum != 1) return;        // only the head of the batch updates the count
  if (inView <= 0 || inView > 64) return;
  noteNmea(inView);
}

static void parseGga(char *line) {
  char *star = strchr(line, '*');
  if (star) *star = 0;
  char *f[16];
  int n = splitCsv(line, f, 16);
  if (n < 10) return;
  // $GPGGA,time,lat,N,lon,E,fix,sats,hdop,alt,M,...
  int quality = atoi(f[6]);
  int nsat = f[7][0] ? atoi(f[7]) : -1;
  // Always record the satellite number when GGA arrives, even if quality is
  // zero. The previous code only updated sats when quality > 0, which meant
  // a module that took 5 minutes to get its first fix never reported sat
  // count progress to the UI.
  noteNmea(nsat);
  if (quality <= 0) return;
  if (!f[2][0] || !f[4][0]) return;
  double lat = nmeaToDeg(f[2], f[3][0]);
  double lon = nmeaToDeg(f[4], f[5][0]);
  float alt = f[9][0] ? (float)atof(f[9]) : -9999.0f;
  applyFix(lat, lon, alt, -1.0f, -1.0f, nsat, true);
}

static void parseRmc(char *line) {
  char *star = strchr(line, '*');
  if (star) *star = 0;
  char *f[16];
  int n = splitCsv(line, f, 16);
  if (n < 10) return;
  // $GPRMC,time,A,lat,N,lon,E,spd_knots,course,date,...
  if (f[2][0] != 'A' && f[2][0] != 'a') return;
  if (!f[3][0] || !f[5][0]) return;
  double lat = nmeaToDeg(f[3], f[4][0]);
  double lon = nmeaToDeg(f[5], f[6][0]);
  float knots = f[7][0] ? (float)atof(f[7]) : -1.0f;
  float spd = knots >= 0.0f ? knots * 1.852f : -1.0f;
  float crs = f[8][0] ? (float)atof(f[8]) : -1.0f;
  applyFix(lat, lon, -9999.0f, spd, crs, -1, true);
}

// Total NMEA lines parsed and total lines seen (including ones that failed
// checksum). Used by the periodic status log to tell "module is silent" from
// "module is talking but the parser drops every line".
static uint32_t nmeaOk = 0;
static uint32_t nmeaBad = 0;
static uint32_t lastNmeaLog = 0;
static char lastTalker[8] = {0};
static uint32_t lastTalkerCount = 0;

static void handleLine(char *line) {
  if (line[0] != '$') return;
  bool hasStar = strchr(line, '*') != nullptr;
  bool ok = !hasStar || nmeaChecksumOk(line);
  if (!ok) {
    nmeaBad++;
    return;
  }
  noteNmea(-1);
  nmeaOk++;
  // Stash the talker/sentence id so the periodic log can show what the GPS
  // is actually sending. Helps tell apart "no satellites" from "no GGA".
  size_t tl = strlen(line);
  if (tl >= 6) {
    size_t copy = tl < 6 ? tl : 6;
    if (copy > sizeof(lastTalker) - 1) copy = sizeof(lastTalker) - 1;
    memcpy(lastTalker, line, copy);
    lastTalker[copy] = 0;
    lastTalkerCount++;
  }
  if (talkerIs(line, "GGA")) {
    parseGga(line);
  } else if (talkerIs(line, "RMC")) {
    parseRmc(line);
  } else if (talkerIs(line, "GSV")) {
    parseGsv(line);
  }
}

static void gpsStartBaud(long baud) {
  if (activeBaud == baud && activeBaud != 0) {
    // already running at this rate
    return;
  }
  GpsUart.end();
  // setRxBufferSize must be called BEFORE begin(). Calling it after begin()
  // prints "RX Buffer can't be resized when Serial is already running" and
  // leaves the UART driver in an undefined state on some ESP32-S3 clones,
  // which then crashes WebSocketsClient on the next TCP event. Default 256
  // bytes is plenty for one-second NMEA bursts.
  GpsUart.setRxBufferSize(512);
  GpsUart.begin(baud, SERIAL_8N1, GPS_RX, GPS_TX);
  lineLen = 0;
  activeBaud = baud;
}

void gpsBegin() {
  probeStartMs = millis();
  probeIdx = 0;
  // Mark probeDone immediately when GPS_BAUD is fixed in config.h, otherwise
  // the periodic NMEA status log below never prints — it guards on
  // probeDone, and the auto-detect block that flips probeDone is itself
  // #if'd out when GPS_BAUD > 0.
  probeDone = (GPS_BAUD > 0);
  activeBaud = 0;
  byteCount = 0;
  // If GPS_BAUD is set in config.h we still go through gpsStartBaud() so the
  // log line is uniform. GPS_BAUD == 0 means "auto-detect on first run".
#if GPS_BAUD > 0
  gpsStartBaud(GPS_BAUD);
  Serial.printf("[GPS] UART1 %ld baud  RX=GPIO%d (fixed from config.h)\n",
                activeBaud, GPS_RX);
#else
  gpsStartBaud(kProbeBauds[0]);
  Serial.println("[GPS] UART1 auto-detect: trying 9600 (set GPS_BAUD in config.h to skip)");
#endif
  // Many NEO-7M clones ship with GGA disabled (only GLL/GSV/RMC enabled by
  // default). Without GGA our parser never sees a satellite count and the
  // dashboard reads "sats=0" forever even when the module is tracking 8
  // birds. u-blox accepts the proprietary PUBX config command over UART1 to
  // re-enable every standard sentence at 1 Hz.
  //
  // We do NOT have a TX wire (config.h: GPS_TX = -1), so we cannot push the
  // command back to the module. Best we can do is document it: if you want
  // auto-enable, wire GPS_RX<->GPS_TX (loopback) or drive a u-center session
  // once on the bench, send "$PUBX,40,GGA,1,1,1,1*5B" etc., then save the
  // config with "$PUBX,00*0A" then "$PUBX,06,1*3A".
  xTaskCreatePinnedToCore(gpsTask, "gps", 4096, nullptr, 1, nullptr, 0);
}

static void gpsPoll() {
  // Auto-detect baud: if no NMEA / no raw bytes after a probe window, cycle to
  // the next candidate. Also dump the first ~32 raw bytes so you can SEE the
  // GPS TX line on the serial monitor (helps diagnose wiring / dead modules).
  uint32_t now = millis();
#if GPS_BAUD == 0
  if (!probeDone) {
    // Each baud gets 2s. If a candidate produced bytes or parsed a fix, lock in.
    uint32_t slotStart = probeStartMs + (uint32_t)probeIdx * 2000u;
    if (now - slotStart >= 2000u) {
      if (byteCount > 0 || (nmeaAt && (now - nmeaAt) < 2000)) {
        Serial.printf("[GPS] locked at %ld baud (%lu bytes)\n",
                      activeBaud, (unsigned long)byteCount);
        probeDone = true;
      } else {
        probeIdx++;
        if (probeIdx >= kProbeCount) {
          // Wrap once and stop. With no data at any baud we still need to keep
          // polling, so leave activeBaud on the last attempt.
          Serial.println("[GPS] no NMEA on any baud - check wiring (TX->GPIO, VCC=3V3)");
          Serial.println("        also confirm the GPS has a fix (cold start 30-90s outdoors)");
          probeDone = true;
        } else {
          Serial.printf("[GPS] no data @ %ld baud, trying %ld\n",
                        activeBaud, kProbeBauds[probeIdx]);
          gpsStartBaud(kProbeBauds[probeIdx]);
        }
      }
    }
  }
#endif

  // First-time RX dump so wiring problems show up on Serial Monitor. Only the
  // first 32 bytes after boot; after that we trust the parser.
  static uint32_t dumpDone = 0;
  static char dumpBuf[64];
  static size_t dumpLen = 0;
  if (!dumpDone && byteCount >= 32) {
    dumpDone = 1;
    Serial.print("[GPS] raw RX: ");
    for (size_t i = 0; i < dumpLen && i < 32; i++) {
      uint8_t b = (uint8_t)dumpBuf[i];
      if (b >= 32 && b < 127) Serial.write(b);
      else Serial.printf("\\x%02x", b);
    }
    Serial.println();
  }

  while (GpsUart.available()) {
    char c = (char)GpsUart.read();
    byteCount++;
    if (dumpLen < sizeof(dumpBuf)) dumpBuf[dumpLen++] = c;
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineLen >= 10) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
      }
      lineLen = 0;
      continue;
    }
    if (lineLen + 1 < sizeof(lineBuf)) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
    }
  }

  // Periodic byte-rate log so a stuck-at-zero RX also shows up in the heartbeat.
  if (probeDone && byteCount == 0) {
    static uint32_t lastZeroWarn = 0;
    if (now - lastZeroWarn >= 30000) {
      lastZeroWarn = now;
      Serial.println("[GPS] 0 bytes received since boot - check GPS TX wire");
    }
  }
  // Periodic sentence breakdown so you can SEE what the module is sending.
  // Without this, "sats=0" can mean anything: no RX, all checksums failing,
  // only GSV sent, or only RMC with status=V (no fix).
  if (probeDone && now - lastNmeaLog >= 15000) {
    lastNmeaLog = now;
    Serial.printf("[GPS] NMEA ok=%lu bad=%lu last='%s' (x%lu) bytes=%lu\n",
                  (unsigned long)nmeaOk,
                  (unsigned long)nmeaBad,
                  lastTalker,
                  (unsigned long)lastTalkerCount,
                  (unsigned long)byteCount);
    lastTalkerCount = 0;
  }

  portENTER_CRITICAL(&gpsMux);
  if (haveFix && (millis() - fixAt) > 15000) {
    haveFix = false;
  }
  portEXIT_CRITICAL(&gpsMux);
}

bool gpsHasFix() {
  portENTER_CRITICAL(&gpsMux);
  bool ok = haveFix;
  portEXIT_CRITICAL(&gpsMux);
  return ok;
}

bool gpsHasRx() {
  portENTER_CRITICAL(&gpsMux);
  bool ok = nmeaAt && (millis() - nmeaAt) < 5000;
  portEXIT_CRITICAL(&gpsMux);
  return ok;
}

int gpsSatellites() {
  portENTER_CRITICAL(&gpsMux);
  int n = sats;
  portEXIT_CRITICAL(&gpsMux);
  return n;
}

uint32_t gpsAgeMs() {
  portENTER_CRITICAL(&gpsMux);
  uint32_t age = haveFix ? (millis() - fixAt) : 0;
  portEXIT_CRITICAL(&gpsMux);
  return age;
}

void gpsAppendJson(String &body) {
  bool ok;
  double lat, lon;
  float alt, spd, crs;
  int nsat;
  uint32_t age;
  bool rx;
  uint32_t nmeaMs;
  portENTER_CRITICAL(&gpsMux);
  ok = haveFix;
  lat = latDeg;
  lon = lonDeg;
  alt = altM;
  spd = spdKmh;
  crs = crsDeg;
  nsat = sats;
  age = haveFix ? (millis() - fixAt) : 0;
  nmeaMs = nmeaAt;
  portEXIT_CRITICAL(&gpsMux);
  rx = nmeaMs && (millis() - nmeaMs) < 5000;

  body += ",\"gps\":";
  body += ok ? "true" : "false";
  body += ",\"gps_on\":true";
  body += ",\"gps_rx\":";
  body += rx ? "true" : "false";
  body += ",\"sats\":";
  body += String(nsat);
  if (!ok) {
    body += ",\"lat\":null,\"lon\":null";
    return;
  }
  body += ",\"lat\":";
  body += String(lat, 6);
  body += ",\"lon\":";
  body += String(lon, 6);
  body += ",\"alt\":";
  body += String(alt, 1);
  body += ",\"spd\":";
  body += String(spd, 1);
  body += ",\"crs\":";
  body += String(crs, 1);
  body += ",\"sats\":";
  body += String(nsat);
  body += ",\"gps_age_ms\":";
  body += String(age);
}

static void gpsTask(void *) {
  for (;;) {
    gpsPoll();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
