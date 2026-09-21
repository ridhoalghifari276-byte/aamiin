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

static char lineBuf[128];
static size_t lineLen = 0;

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
  portEXIT_CRITICAL(&gpsMux);
}

static void parseGga(char *line) {
  char *star = strchr(line, '*');
  if (star) *star = 0;
  char *f[16];
  int n = splitCsv(line, f, 16);
  if (n < 10) return;
  // $GPGGA,time,lat,N,lon,E,fix,sats,hdop,alt,M,...
  int quality = atoi(f[6]);
  if (quality <= 0) return;
  if (!f[2][0] || !f[4][0]) return;
  double lat = nmeaToDeg(f[2], f[3][0]);
  double lon = nmeaToDeg(f[4], f[5][0]);
  int nsat = f[7][0] ? atoi(f[7]) : -1;
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

static void handleLine(char *line) {
  if (line[0] != '$') return;
  if (strchr(line, '*') && !nmeaChecksumOk(line)) return;
  if (talkerIs(line, "GGA")) {
    parseGga(line);
  } else if (talkerIs(line, "RMC")) {
    parseRmc(line);
  }
}

void gpsBegin() {
  GpsUart.setRxBufferSize(512);
  GpsUart.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  lineLen = 0;
  Serial.printf("[GPS] UART1 %d baud  RX=GPIO%d TX=GPIO%d (NEO-6M TX -> GPIO%d)\n",
                GPS_BAUD, GPS_RX, GPS_TX, GPS_RX);
  xTaskCreatePinnedToCore(gpsTask, "gps", 4096, nullptr, 1, nullptr, 0);
}

static void gpsPoll() {
  while (GpsUart.available()) {
    char c = (char)GpsUart.read();
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
  portENTER_CRITICAL(&gpsMux);
  ok = haveFix;
  lat = latDeg;
  lon = lonDeg;
  alt = altM;
  spd = spdKmh;
  crs = crsDeg;
  nsat = sats;
  age = haveFix ? (millis() - fixAt) : 0;
  portEXIT_CRITICAL(&gpsMux);

  body += ",\"gps\":";
  body += ok ? "true" : "false";
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
