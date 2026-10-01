#include "rt_proto.h"

#include <Arduino.h>
#include <WiFi.h>
#include <errno.h>
#include <fcntl.h>
#include <lwip/sockets.h>
#include <string.h>

#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "config.h"

static const size_t RT_HDR = 48;
// Stay under one Wi-Fi packet. A 2400-byte datagram is split by IP,
// sendto returns ENOMEM, and the two halves get mixed into a color strip.
static const size_t RT_CHUNK = 1200;
static const int RT_PIECES = 2;
static const int RT_SLOTS = 4;
static uint8_t rtPkt[RT_SLOTS][RT_PIECES][RT_HDR + RT_CHUNK] __attribute__((aligned(4)));
static uint8_t rtSide[RT_HDR + RT_CHUNK] __attribute__((aligned(4)));
static uint8_t rtSlot = 0;

static int rtFd = -1;
static struct sockaddr_in rtAddr;
static SemaphoreHandle_t rtMu = nullptr;
static uint16_t rtSeq = 0;
static uint16_t rtFrame = 0;
static char rtDev[16];
static char rtTok[12];
static uint32_t rtOk = 0;
static uint32_t rtFail = 0;
static uint32_t rtFrames = 0;
static uint32_t rtFpsTick = 0;
static uint8_t rtFps = 0;
static uint8_t rtFpsCount = 0;
static int rtHint = 0;
static uint32_t rtHbAt = 0;
static uint16_t rtRtt = 0;

static bool rtOpen(const char *host) {
  if (rtFd >= 0) return true;
  IPAddress ip;
  if (!host || !ip.fromString(host)) return false;
  rtFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (rtFd < 0) return false;
  int fl = fcntl(rtFd, F_GETFL, 0);
  fcntl(rtFd, F_SETFL, fl | O_NONBLOCK);
  memset(&rtAddr, 0, sizeof(rtAddr));
  rtAddr.sin_family = AF_INET;
  rtAddr.sin_port = htons(RT_UDP_PORT);
  uint8_t *d = (uint8_t *)&rtAddr.sin_addr.s_addr;
  d[0] = ip[0];
  d[1] = ip[1];
  d[2] = ip[2];
  d[3] = ip[3];
  Serial.printf("[RT] udp -> %u.%u.%u.%u:%d\n", ip[0], ip[1], ip[2], ip[3], RT_UDP_PORT);
  return true;
}

void rtBegin(const char *host, const char *device, const char *token) {
  if (!rtMu) rtMu = xSemaphoreCreateMutex();
  memset(rtDev, 0, sizeof(rtDev));
  memset(rtTok, 0, sizeof(rtTok));
  if (device) strncpy(rtDev, device, sizeof(rtDev) - 1);
  if (token) strncpy(rtTok, token, sizeof(rtTok) - 1);
  rtOpen(host);
}

static void rtFill(uint8_t *pkt, uint8_t type, uint8_t flags, uint8_t nchunks,
                   uint16_t frame, uint32_t ts, uint8_t idx, const uint8_t *data, size_t len) {
  memset(pkt, 0, RT_HDR);
  memcpy(pkt, "BCRT", 4);
  pkt[4] = 1;
  pkt[5] = type;
  pkt[6] = flags;
  pkt[7] = nchunks;
  pkt[8] = (uint8_t)rtSeq;
  pkt[9] = (uint8_t)(rtSeq >> 8);
  pkt[10] = (uint8_t)ts;
  pkt[11] = (uint8_t)(ts >> 8);
  pkt[12] = (uint8_t)(ts >> 16);
  pkt[13] = (uint8_t)(ts >> 24);
  pkt[14] = (uint8_t)frame;
  pkt[15] = (uint8_t)(frame >> 8);
  pkt[16] = (uint8_t)len;
  pkt[17] = (uint8_t)(len >> 8);
  pkt[18] = idx;
  memcpy(pkt + 20, rtDev, 16);
  memcpy(pkt + 36, rtTok, 12);
  if (len && data) memcpy(pkt + RT_HDR, data, len);
}

static size_t rtJpegEnd(const uint8_t *p, size_t n) {
  if (!p || n < 4 || p[0] != 0xFF || p[1] != 0xD8) return 0;
  for (size_t i = n; i >= 2; i--) {
    if (p[i - 2] == 0xFF && p[i - 1] == 0xD9) return i;
  }
  return 0;
}

static bool rtSendRaw(uint8_t *pkt, size_t pktLen) {
  if (rtFd < 0 || pktLen < RT_HDR) return false;
  for (int attempt = 0; attempt < 3; attempt++) {
    int n = sendto(rtFd, pkt, pktLen, MSG_DONTWAIT, (struct sockaddr *)&rtAddr, sizeof(rtAddr));
    if (n == (int)pktLen) return true;
    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOMEM) return false;
    vTaskDelay(pdMS_TO_TICKS(4));
  }
  return false;
}

bool rtSendJpeg(const uint8_t *jpeg, size_t len, uint32_t tsMs) {
  size_t n = rtJpegEnd(jpeg, len);
  if (!n) return false;
  int chunks = (int)((n + RT_CHUNK - 1) / RT_CHUNK);
  // Two pieces cover a normal 320x240 frame. A third piece is what got lost.
  if (chunks < 1 || chunks > RT_PIECES) {
    rtFail++;
    return false;
  }
  if (!rtMu || !rtOpen(nullptr)) return false;
  uint8_t slot = rtSlot;
  rtSlot = (uint8_t)((rtSlot + 1) % RT_SLOTS);
  uint16_t frame = rtFrame++;
  size_t off = 0;
  size_t pktLen[RT_PIECES];
  if (xSemaphoreTake(rtMu, pdMS_TO_TICKS(20)) != pdTRUE) return false;
  for (int i = 0; i < chunks; i++) {
    size_t c = n - off;
    if (c > RT_CHUNK) c = RT_CHUNK;
    uint8_t flags = (i + 1 == chunks) ? 1 : 0;
    rtFill(rtPkt[slot][i], 1, flags, (uint8_t)chunks, frame, tsMs, (uint8_t)i, jpeg + off, c);
    rtPkt[slot][i][8] = (uint8_t)rtSeq;
    rtPkt[slot][i][9] = (uint8_t)(rtSeq >> 8);
    rtSeq++;
    pktLen[i] = RT_HDR + c;
    off += c;
  }
  xSemaphoreGive(rtMu);
  for (int i = 0; i < chunks; i++) {
    if (!rtSendRaw(rtPkt[slot][i], pktLen[i])) {
      rtFail++;
      static uint32_t lastLog = 0;
      if (millis() - lastLog > 2000) {
        lastLog = millis();
        Serial.printf("[RT] send fail errno=%d jpg=%u piece=%d/%d\n", errno, (unsigned)n, i, chunks);
      }
      return false;
    }
    if (i + 1 < chunks) vTaskDelay(pdMS_TO_TICKS(6));
  }
  rtOk++;
  rtFrames++;
  rtFpsCount++;
  if (millis() - rtFpsTick >= 1000) {
    rtFps = rtFpsCount;
    rtFpsCount = 0;
    rtFpsTick = millis();
  }
  return true;
}

static bool rtSendSide(uint8_t type, uint32_t ts, const uint8_t *data, size_t len) {
  if (!rtMu || !rtOpen(nullptr) || !data || !len || len > RT_CHUNK) return false;
  bool ok = false;
  if (xSemaphoreTake(rtMu, pdMS_TO_TICKS(20)) != pdTRUE) return false;
  rtFill(rtSide, type, 1, 1, rtFrame, ts, 0, data, len);
  rtSide[8] = (uint8_t)rtSeq;
  rtSide[9] = (uint8_t)(rtSeq >> 8);
  rtSeq++;
  size_t pktLen = RT_HDR + len;
  for (int attempt = 0; attempt < 2 && !ok; attempt++) {
    int n = sendto(rtFd, rtSide, pktLen, MSG_DONTWAIT, (struct sockaddr *)&rtAddr, sizeof(rtAddr));
    if (n == (int)pktLen) ok = true;
    else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOMEM) break;
    else vTaskDelay(pdMS_TO_TICKS(4));
  }
  xSemaphoreGive(rtMu);
  if (ok) rtOk++;
  else rtFail++;
  return ok;
}

bool rtSendPcm(const int16_t *pcm, size_t samples, uint32_t tsMs) {
  if (!pcm || !samples) return false;
  size_t bytes = samples * sizeof(int16_t);
  if (bytes > RT_CHUNK) bytes = RT_CHUNK;
  return rtSendSide(2, tsMs, (const uint8_t *)pcm, bytes);
}

void rtHeartbeat(int rssi, uint32_t heapBytes) {
  uint8_t body[12];
  memset(body, 0, sizeof(body));
  int r = rssi;
  if (r < -127) r = -127;
  if (r > 127) r = 127;
  body[0] = (uint8_t)(int8_t)r;
  body[1] = rtFps;
  body[2] = (uint8_t)rtLevel();
  uint16_t heapKb = (uint16_t)(heapBytes / 1024);
  body[4] = (uint8_t)heapKb;
  body[5] = (uint8_t)(heapKb >> 8);
  body[6] = (uint8_t)rtRtt;
  body[7] = (uint8_t)(rtRtt >> 8);
  body[8] = (uint8_t)rtFrames;
  body[9] = (uint8_t)(rtFrames >> 8);
  body[10] = (uint8_t)rtFail;
  body[11] = (uint8_t)(rtFail >> 8);
  rtHbAt = millis();
  rtSendSide(3, rtHbAt, body, sizeof(body));
}

void rtPoll() {
  if (rtFd < 0) return;
  uint8_t buf[64];
  struct sockaddr_in from;
  socklen_t fl = sizeof(from);
  int n = recvfrom(rtFd, buf, sizeof(buf), MSG_DONTWAIT, (struct sockaddr *)&from, &fl);
  if (n < RT_HDR + 1 || memcmp(buf, "BCRT", 4) != 0 || buf[5] != 4) return;
  if (rtHbAt) {
    uint32_t dt = millis() - rtHbAt;
    if (dt < 5000) rtRtt = (uint16_t)dt;
  }
  if (n >= (int)RT_HDR + 1) rtHint = buf[RT_HDR];
}

int rtLevel() {
  uint32_t total = rtOk + rtFail;
  if (rtHint >= 2) return 2;
  if (rtHint == 1) return 1;
  if (total < 20) return 0;
  uint32_t pct = (rtFail * 100) / total;
  if (pct > 15) return 2;
  if (pct > 5) return 1;
  return 0;
}

void rtAdaptCam() {
  // Leave quality to tuneLiveSize. Raising q here did not shrink the
  // JPEG and the picture went to NO SIGNAL.
}

void rtApplyCam() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  Serial.printf("[RT] sensor 0x%04x — live stays 640x480\n", s->id.PID);
}
