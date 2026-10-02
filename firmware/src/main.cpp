// Handshake budget. 80 ms expired before the VPS answered, so the
// socket died with "disconnected" and never printed "connected".
// Video and mic do not use this write path.
#define WEBSOCKETS_TCP_TIMEOUT (3000)
#include <Arduino.h>
#include <WiFi.h>
#include <fcntl.h>
#include <errno.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WebSocketsClient.h>
#include <lwip/sockets.h>
#include <esp_wifi.h>
#include <esp_camera.h>
#include <esp_cache.h>
#include <esp_heap_caps.h>
#include <driver/i2s.h>
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"
#include "rt_proto.h"
#include "portal.h"
#include "gps.h"
#include "speaker.h"

// ============================================================
// ESP32 BODYCAM
//
// Audio: INMP441 >>14 into a PSRAM ring, WebSocket PCM on /ws/audio.
// Video: QVGA JPEG over UDP. A lost piece is sent again so the
// dashboard does not keep the previous picture.
// ============================================================

static volatile bool streamEnabled = false;
static volatile bool audioEnabled = false;
static volatile bool videoEnabled = false;
static volatile bool nightVision = false;
static volatile bool pttHeld = false;
static volatile bool sosActive = false;
static volatile bool stateDirty = false;

static Adafruit_NeoPixel pixel(1, RGB_LED, NEO_GRB + NEO_KHZ800);
static SemaphoreHandle_t stateMux = nullptr;
static SemaphoreHandle_t txMu = nullptr;
static SemaphoreHandle_t udpMu = nullptr;
static uint16_t udpVideoSeq = 0;
static uint8_t audioFrame[768];
static size_t audioLen = 0;
static size_t audioOff = 0;
static uint32_t jpegSig = 0;

// SO_SNDBUF is not supported on this lwIP (errno 109). A short write
// timeout is what keeps one JPEG from holding the radio for seconds.
class TunedWs : public WebSocketsClient {
 public:
  void tuneLink(const char *tag) {
    if (!_client.tcp) return;
    _client.tcp->setNoDelay(true);
    _client.tcp->setConnectionTimeout(5);
    Serial.printf("[WS] %s nodelay\n", tag);
  }

  // One non-blocking slice. tcp->write() keeps going while any byte is
  // accepted, so a slow link held the camera task for seconds.
  int pushRaw(const uint8_t *data, size_t n) {
    if (!_client.tcp || !_client.tcp->connected() || !data || !n) return -1;
    int sock = _client.tcp->fd();
    if (sock < 0) return -1;
    int r = ::send(sock, data, n, MSG_DONTWAIT);
    if (r > 0) return r;
    if (r == 0 || errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;
  }

  bool canSend() {
    if (!_client.tcp) return false;
    int sock = _client.tcp->fd();
    if (sock < 0) return false;
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    return select(sock + 1, nullptr, &wfds, nullptr, &tv) > 0;
  }
};

static TunedWs streamWs;
static volatile bool wsConnected = false;
static bool wsStarted = false;
static TunedWs audioWs;
static volatile bool audioWsConnected = false;
static bool audioWsStarted = false;
static uint8_t *txPacket = nullptr;   // reused video packet buffer (no per-frame malloc)
static uint8_t *grabBuf = nullptr;
static size_t grabLen = 0;
static uint32_t grabSeq = 0;
static SemaphoreHandle_t grabMu = nullptr;
static size_t txPacketCap = 0;
static uint8_t *wsFrame = nullptr;    // masked websocket frame, sent in slices
static size_t wsFrameCap = 0;
static size_t wsLen = 0;
static size_t wsOff = 0;
static uint32_t wsT0 = 0;
static size_t lastJpegBytes = 0;

static int16_t *audioRing = nullptr;
static volatile size_t ringWrite = 0;
static volatile size_t ringRead = 0;
static SemaphoreHandle_t ringMu = nullptr;

// Public VPS. Do not scan the STA /24 — that floods TCP and aborts /ws/device.
static String serverHost = SERVER_HOST;
static String deviceId = DEVICE_ID;
static String pttToken;

static String baseUrl() {
  return String("http://") + serverHost + ":" + String(SERVER_PORT);
}


static String sessionId;
static String sessionMode;
static volatile int sessionCmd = 0; // 0 none, 1 audio, 2 video, 3 stop

static uint32_t txAudioPackets = 0;
static uint32_t txVideoFrames = 0;
static uint32_t txAudioDrops = 0;
static uint32_t txVideoDrops = 0;
static uint32_t txVideoSkips = 0;
static volatile uint32_t lastVideoSendMs = 0;
static uint32_t lastStats = 0;

static void led(bool on) {
#if STATUS_LED >= 0
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

static void rgb(uint8_t r, uint8_t g, uint8_t b) {
#if USE_RGB_LED
  // GPIO 48 is next to the OV2640. Full-bright LED makes indoor AWB go magenta.
  pixel.setPixelColor(0, pixel.Color(r / 20, g / 20, b / 20));
  pixel.show();
#endif
}

static bool visualOn() {
  // Keep the live picture up during audio record. Blanking it made the
  // dashboard freeze every time the audio button was pressed.
  return streamEnabled || videoEnabled;
}

static void nightIr(bool on) {
#if NIGHT_IR_PIN >= 0
  pinMode(NIGHT_IR_PIN, OUTPUT);
  digitalWrite(NIGHT_IR_PIN, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

static void applyCamNight(bool on) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  if (on) {
    // Low-light capture only. Green NV look is applied on the server
    // when this flag is set — do not force grayscale on the sensor.
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gainceiling(s, GAINCEILING_64X);
    s->set_agc_gain(s, 24);
    s->set_aec_value(s, 1000);
    s->set_ae_level(s, 2);
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
    s->set_saturation(s, -1);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_special_effect(s, 0);
    s->set_lenc(s, 1);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_dcw(s, 1);
  } else {
    // Match the bright firmware: do not touch exposure, gain, or aec2.
    // Those overrides turned the live picture dark green.
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_special_effect(s, 0);
    s->set_vflip(s, CAM_VFLIP);
    s->set_hmirror(s, CAM_HMIRROR);
  }
  nightIr(on);
}

static void stateLed() {
  if (sosActive) rgb(255, 0, 60);
  else if (pttHeld) rgb(255, 120, 0);
  else if (!streamEnabled && !audioEnabled && !videoEnabled) rgb(0, 0, 0);
  else if (videoEnabled) rgb(255, 0, 0);
  else if (audioEnabled) rgb(0, 0, 255);
  else rgb(0, 255, 0);
}

static size_t ringCountUnsafe() {
  return (ringWrite + AUDIO_RING_SAMPLES - ringRead) % AUDIO_RING_SAMPLES;
}

static void ringLock() {
  if (ringMu) xSemaphoreTake(ringMu, portMAX_DELAY);
}

static void ringUnlock() {
  if (ringMu) xSemaphoreGive(ringMu);
}

static size_t ringCount() {
  ringLock();
  size_t n = ringCountUnsafe();
  ringUnlock();
  return n;
}

static void ringClear() {
  ringLock();
  ringWrite = 0;
  ringRead = 0;
  ringUnlock();
}

static void ringPush(const int16_t *src, size_t n) {
  // Mutex, not a critical section: the ring lives in PSRAM and a
  // cache-off critical section corrupts audio while Wi-Fi is busy.
  ringLock();
  for (size_t i = 0; i < n; i++) {
    size_t next = (ringWrite + 1) % AUDIO_RING_SAMPLES;
    if (next == ringRead) {
      ringRead = (ringRead + 1) % AUDIO_RING_SAMPLES;
    }
    audioRing[ringWrite] = src[i];
    ringWrite = next;
  }
  ringUnlock();
}

static size_t ringPop(int16_t *dst, size_t n) {
  ringLock();
  size_t avail = ringCountUnsafe();
  size_t take = n < avail ? n : avail;
  size_t first = AUDIO_RING_SAMPLES - ringRead;
  if (first > take) first = take;
  memcpy(dst, audioRing + ringRead, first * sizeof(int16_t));
  if (take > first) memcpy(dst + first, audioRing, (take - first) * sizeof(int16_t));
  ringRead = (ringRead + take) % AUDIO_RING_SAMPLES;
  ringUnlock();
  return take;
}

static void ringKeepLatest(size_t keep) {
  ringLock();
  size_t avail = ringCountUnsafe();
  if (avail > keep) {
    ringRead = (ringWrite + AUDIO_RING_SAMPLES - keep) % AUDIO_RING_SAMPLES;
  }
  ringUnlock();
}

static void copySessionHeaders(HTTPClient &h) {
  h.addHeader("X-Device-ID", deviceId.c_str());
  h.addHeader("X-Token", SERVER_TOKEN);
  if (sessionId.length()) {
    h.addHeader("X-Session-Id", sessionId);
    h.addHeader("X-Session-Mode", sessionMode);
  }
}

static bool postJson(const char *path, const String &body) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClient client;
  HTTPClient h;
  String url = baseUrl() + path;
  if (!h.begin(client, url)) return false;

  h.addHeader("Content-Type", "application/json");
  copySessionHeaders(h);
  h.setConnectTimeout(4000);
  h.setTimeout(4000);
  int code = h.POST(body);
  h.end();
  return code >= 200 && code < 300;
}

static void sessionStart(const char *mode) {
  sessionMode = mode;
  sessionId = deviceId + "-" + mode + "-" + String((uint32_t)millis());
  String body = String("{\"mode\":\"") + mode + "\",\"session_id\":\"" + sessionId + "\"}";
  postJson("/api/v1/session/start", body);
  Serial.printf("[SESSION] start %s %s\n", mode, sessionId.c_str());
}

static void sessionStop() {
  if (!sessionId.length()) return;
  String old = sessionId;
  String body = String("{\"session_id\":\"") + old + "\"}";
  postJson("/api/v1/session/stop", body);
  sessionId = "";
  sessionMode = "";
  Serial.printf("[SESSION] stop %s\n", old.c_str());
}

static void postDeviceState() {
  if (WiFi.status() != WL_CONNECTED) return;
  String body = String("{\"stream\":") + (streamEnabled ? "true" : "false") +
                ",\"audio\":" + (audioEnabled ? "true" : "false") +
                ",\"video\":" + (videoEnabled ? "true" : "false") +
                ",\"profile\":\"" + (videoEnabled ? "hd" : "ld") + "\"" +
                ",\"visual\":" + (visualOn() ? "true" : "false") +
                ",\"nightvision\":false" +
                ",\"ptt\":" + (pttHeld ? "true" : "false") +
                ",\"sos\":" + (sosActive ? "true" : "false") +
                (pttToken.length() ? (String(",\"ptt_token\":\"") + pttToken + "\"") : String()) +
                ",\"rssi\":" + String(WiFi.RSSI()) +
                ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  gpsAppendJson(body);
  body += "}";
  postJson("/api/v1/device/state", body);
}

// ============================================================
// WEBSOCKET TRANSPORT
// Two sockets so voice never queues behind a JPEG:
//   /ws/device : 0x01 + JPEG bytes
//   /ws/audio  : raw PCM s16le mono 16 kHz
// ============================================================

static void wsEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      wsConnected = true;
      WiFi.setSleep(false);
      esp_wifi_set_ps(WIFI_PS_NONE);
      streamWs.tuneLink("video");
      // Heartbeat ping fires on the first loop because lastPing starts at 0.
      // If the 8 Wi-Fi TX slots are full the ping fails and the library
      // closes the socket, which is the connect/disconnect loop.
      Serial.printf("[WS] connected: %s\n", payload ? (char *)payload : "");
      break;
    case WStype_DISCONNECTED:
      wsConnected = false;
      wsLen = 0;
      Serial.printf("[WS] disconnected %.*s\n", (int)length, (payload && length) ? (char *)payload : "");
      break;
    case WStype_ERROR:
      wsConnected = false;
      wsLen = 0;
      Serial.println("[WS] error");
      break;
    case WStype_TEXT:
      if (payload && length) {
        Serial.printf("[WS] server: %.*s\n", (int)length, (char *)payload);
      }
      break;
    case WStype_BIN:
      // 0x03 + s16le = radio downlink for the speaker. Mic TX path is unchanged.
      if (payload && length > 1 && payload[0] == 0x03) {
        speakerPush(payload + 1, length - 1);
      }
      break;
    default:
      break;
  }
}

static void audioWsEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      audioWsConnected = true;
      audioWs.tuneLink("audio");
      audioLen = 0;
      audioOff = 0;
      ringClear();
      Serial.println("[WS-AUDIO] connected");
      break;
    case WStype_DISCONNECTED:
      audioWsConnected = false;
      audioLen = 0;
      audioOff = 0;
      ringClear();
      Serial.println("[WS-AUDIO] disconnected");
      break;
    case WStype_ERROR:
      audioWsConnected = false;
      break;
    case WStype_BIN:
      // HT radio downlink on the audio socket (JPEG no longer blocks it).
      if (payload && length > 1 && payload[0] == 0x03) {
        speakerPush(payload + 1, length - 1);
      }
      break;
    default:
      break;
  }
}

static void stopWebSocket() {
  if (!wsStarted) return;
  streamWs.disconnect();
  wsStarted = false;
  wsConnected = false;
}

static void startAudioWebSocket() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (audioWsStarted) return;
  String path = String("/ws/audio?device=") + deviceId + "&token=" + SERVER_TOKEN;
  audioWs.onEvent(audioWsEvent);
  audioWs.setReconnectInterval(3000);
  audioWs.begin(serverHost.c_str(), SERVER_PORT, path.c_str(), "");
  audioWsStarted = true;
  Serial.printf("[WS-AUDIO] connecting %s:%d%s\n", serverHost.c_str(), SERVER_PORT, path.c_str());
}

static void stopAudioWebSocket() {
  if (!audioWsStarted) return;
  audioWs.disconnect();
  audioWsStarted = false;
  audioWsConnected = false;
}

static void bindPublicServer() {
  serverHost = SERVER_HOST;
  Serial.printf("[NET] ip=%s -> %s:%d\n",
                WiFi.localIP().toString().c_str(),
                SERVER_HOST,
                SERVER_PORT);
}

static void startWebSocket() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (wsStarted) return;
  if (!serverHost.length()) return;

  String path = String(SERVER_WS_PATH) +
                "?device=" + deviceId +
                "&token=" + SERVER_TOKEN;

  streamWs.onEvent(wsEvent);
  streamWs.setReconnectInterval(3000);
  // Empty subprotocol: FastAPI rejects Sec-WebSocket-Protocol: arduino.
  streamWs.begin(serverHost.c_str(), SERVER_PORT, path.c_str(), "");
  wsStarted = true;
  Serial.printf("[WS] connecting %s:%d%s\n", serverHost.c_str(), SERVER_PORT, path.c_str());
}

static bool sendWsPacket(uint8_t type, const uint8_t *data, size_t len) {
  if (!wsConnected || !data || len == 0) return false;
  if (len + 1 > txPacketCap || !txPacket) return false;

  txPacket[0] = type;
  memcpy(txPacket + 1, data, len);
  return streamWs.sendBIN(txPacket, len + 1);
}

// Voice goes out on its own socket, so it is never stuck behind a JPEG.
static bool queueBin(uint8_t *dst, size_t cap, size_t *outLen, const uint8_t *payload, size_t len);

static int pumpAudio() {
  if (!audioLen) return 1;
  int n = audioWs.pushRaw(audioFrame + audioOff, audioLen - audioOff);
  if (n < 0) {
    audioLen = 0;
    ++txAudioDrops;
    return -1;
  }
  if (n > 0) audioOff += (size_t)n;
  if (audioOff >= audioLen) {
    audioLen = 0;
    ++txAudioPackets;
    return 1;
  }
  return 0;
}

static bool sendAudioWs() {
  static int16_t txBuf[AUDIO_TX_SAMPLES];
  if (!audioWsConnected || !audioRing || audioLen) return false;
  if (ringCount() < (size_t)AUDIO_TX_SAMPLES) return false;
  size_t n = ringPop(txBuf, AUDIO_TX_SAMPLES);
  if (!n) return false;
  audioOff = 0;
  if (!queueBin(audioFrame, sizeof(audioFrame), &audioLen, (uint8_t *)txBuf, n * sizeof(int16_t))) {
    ++txAudioDrops;
    return false;
  }
  return true;
}

static bool sendAudioChunk() {
  static int16_t txBuf[AUDIO_TX_SAMPLES];
  static WiFiClient audioClient;
  static HTTPClient audioHttp;
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!audioRing) return false;
  if (ringCount() < AUDIO_TX_SAMPLES) return false;
  size_t n = ringPop(txBuf, AUDIO_TX_SAMPLES);
  if (!n) return false;

  String url = baseUrl() + "/api/v1/audio";
  if (!audioHttp.begin(audioClient, url)) {
    ++txAudioDrops;
    return false;
  }
  audioHttp.addHeader("Content-Type", "audio/pcm;rate=16000;channels=1;format=s16le");
  copySessionHeaders(audioHttp);
  audioHttp.setTimeout(200);
  audioHttp.setReuse(true);
  int code = audioHttp.POST((uint8_t *)txBuf, n * sizeof(int16_t));
  audioHttp.end();
  if (code >= 200 && code < 300) {
    ++txAudioPackets;
    return true;
  }
  audioClient.stop();
  ++txAudioDrops;
  return false;
}

static void drainCamFb() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
}

static bool camIsHd = false;
static int liveQ = LIVE_JPEG_Q;
static bool initCam();

static void tuneLiveSize(size_t jpegBytes, uint32_t sendMs) {
  (void)jpegBytes;
  (void)sendMs;
  // Quality stays at the value set in initCam. Raising it while frames
  // were moving left the JPEG the same size and tore the picture.
}

static void ensureCamProfile(bool hd) {
  if (camIsHd == hd) return;
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  s->set_quality(s, hd ? REC_JPEG_Q : liveQ);
  s->set_sharpness(s, hd ? 1 : 0);
  camIsHd = hd;
  Serial.printf("[CAM] %s 640x480 q=%d\n", hd ? "record" : "live",
                hd ? REC_JPEG_Q : liveQ);
}

static bool queueBin(uint8_t *dst, size_t cap, size_t *outLen, const uint8_t *payload, size_t len) {
  if (!dst || !outLen || *outLen || len < 1 || len > 65535) return false;
  if (len + 8 > cap) return false;
  uint32_t r = esp_random();
  uint8_t mask[4] = {
    (uint8_t)r, (uint8_t)(r >> 8), (uint8_t)(r >> 16), (uint8_t)(r >> 24)
  };
  size_t h;
  dst[0] = 0x82;
  if (len < 126) {
    dst[1] = (uint8_t)(0x80 | len);
    h = 2;
  } else {
    dst[1] = 0x80 | 126;
    dst[2] = (uint8_t)(len >> 8);
    dst[3] = (uint8_t)len;
    h = 4;
  }
  memcpy(dst + h, mask, 4);
  h += 4;
  uint8_t *body = dst + h;
  for (size_t i = 0; i < len; i++) body[i] = payload[i] ^ mask[i & 3];
  *outLen = h + len;
  return true;
}

static bool queueVideoFrame(const uint8_t *payload, size_t len) {
  if (wsLen) return false;
  if (!queueBin(wsFrame, wsFrameCap, &wsLen, payload, len)) return false;
  wsOff = 0;
  wsT0 = millis();
  return true;
}

// 1 = idle (frame finished or nothing queued), 0 = still sending, -1 = dropped.
static int pumpVideo() {
  if (!wsLen) return 1;
  // One full 720p/1080p JPEG fills every Wi-Fi TX slot. The mic then stalls
  // (ring hits 47999) and this socket is reset. 640 bytes keeps the link
  // measured in one slot per turn even at 720p30.
  size_t left = wsLen - wsOff;
  size_t slice = left > 640 ? 640 : left;
  int n = streamWs.pushRaw(wsFrame + wsOff, slice);
  if (n < 0) {
    wsLen = 0;
    ++txVideoDrops;
    streamWs.disconnect();
    wsConnected = false;
    return -1;
  }
  if (n > 0) wsOff += (size_t)n;
  if (wsOff >= wsLen) {
    lastVideoSendMs = millis() - wsT0;
    wsLen = 0;
    ++txVideoFrames;
    if (!camIsHd) tuneLiveSize(lastJpegBytes, lastVideoSendMs);
    return 1;
  }
  if (wsOff == 0 && millis() - wsT0 > 200) {
    wsLen = 0;
    ++txVideoSkips;
    return -1;
  }
  return 0;
}

static int udpFd = -1;
static struct sockaddr_in udpAddr;

static bool udpOpen() {
  if (udpFd >= 0) return true;
  IPAddress ip;
  if (!ip.fromString(serverHost)) return false;
  udpFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (udpFd < 0) return false;
  int fl = fcntl(udpFd, F_GETFL, 0);
  fcntl(udpFd, F_SETFL, fl | O_NONBLOCK);
  memset(&udpAddr, 0, sizeof(udpAddr));
  udpAddr.sin_family = AF_INET;
  udpAddr.sin_port = htons(MEDIA_UDP_PORT);
  uint8_t *d = (uint8_t *)&udpAddr.sin_addr.s_addr;
  d[0] = ip[0];
  d[1] = ip[1];
  d[2] = ip[2];
  d[3] = ip[3];
  Serial.printf("[UDP] video -> %u.%u.%u.%u:%d intact\n", ip[0], ip[1], ip[2], ip[3], MEDIA_UDP_PORT);
  return true;
}

// Each piece has its own DRAM buffer. Reusing one stack buffer while the
// radio was still reading it tore the JPEG into the colored strip.
static const size_t UDP_CHUNK = 1400;
static const size_t UDP_PKT = 1440;
static const int UDP_HOLD_N = 4;
static const int UDP_HOLD_CHUNKS = 3;
static uint8_t udpWire[UDP_HOLD_CHUNKS][UDP_PKT] __attribute__((aligned(4)));
static uint8_t udpRepair[UDP_PKT] __attribute__((aligned(4)));
static uint8_t *udpHoldBuf = nullptr;
static struct {
  uint16_t seq;
  uint8_t n;
  uint8_t nacks;
  uint16_t len[UDP_HOLD_CHUNKS];
} udpHold[UDP_HOLD_N];
static uint8_t udpHoldPos = 0;

static size_t jpegTrim(const uint8_t *p, size_t n) {
  if (!p || n < 4 || p[0] != 0xFF || p[1] != 0xD8) return 0;
  for (size_t i = n; i >= 2; i--) {
    if (p[i - 2] == 0xFF && p[i - 1] == 0xD9) return i;
  }
  return 0;
}

static bool udpSendWire(uint8_t *pkt, size_t pktLen) {
  if (!pkt || pktLen < 39 || pktLen > UDP_PKT || !udpOpen()) return false;
  int lastErr = 0;
  int lastN = 0;
  for (int attempt = 0; attempt < 2; attempt++) {
    lastN = sendto(udpFd, pkt, pktLen, MSG_DONTWAIT,
                   (struct sockaddr *)&udpAddr, sizeof(udpAddr));
    if (lastN == (int)pktLen) return true;
    lastErr = errno;
    if (lastErr != EAGAIN && lastErr != EWOULDBLOCK && lastErr != ENOMEM) break;
    vTaskDelay(pdMS_TO_TICKS(8));
  }
  static uint32_t lastUdpLog = 0;
  if (millis() - lastUdpLog > 2000) {
    lastUdpLog = millis();
    Serial.printf("[UDP] send fail n=%d errno=%d heap=%u\n",
                  lastN, lastErr, (unsigned)ESP.getFreeHeap());
  }
  return false;
}

static void udpBuild(uint8_t *pkt, uint8_t idx, uint8_t nchunks, uint16_t seq,
                     const uint8_t *data, size_t len) {
  memset(pkt, 0, 38);
  memcpy(pkt, "BCU1", 4);
  pkt[4] = 1;
  pkt[5] = nchunks;
  pkt[6] = idx;
  pkt[8] = (uint8_t)seq;
  pkt[9] = (uint8_t)(seq >> 8);
  pkt[10] = (uint8_t)len;
  pkt[11] = (uint8_t)(len >> 8);
  strncpy((char *)pkt + 12, deviceId.c_str(), 15);
  strncpy((char *)pkt + 28, SERVER_TOKEN, 9);
  memcpy(pkt + 38, data, len);
}

static uint8_t *udpHoldPtr(int slot, int idx) {
  return udpHoldBuf + (size_t)(slot * UDP_HOLD_CHUNKS + idx) * UDP_CHUNK;
}

static void udpRememberInit() {
  if (udpHoldBuf) return;
  size_t bytes = (size_t)UDP_HOLD_N * UDP_HOLD_CHUNKS * UDP_CHUNK;
  udpHoldBuf = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!udpHoldBuf) udpHoldBuf = (uint8_t *)malloc(bytes);
  if (!udpHoldBuf) Serial.println("[UDP] hold alloc failed");
}

static int udpFindHold(uint16_t seq) {
  for (int i = 0; i < UDP_HOLD_N; i++) {
    if (udpHold[i].n && udpHold[i].seq == seq) return i;
  }
  return -1;
}

static void udpPollNack() {
  if (udpFd < 0 || !udpHoldBuf) return;
  uint8_t buf[16];
  struct sockaddr_in from;
  for (int k = 0; k < 4; k++) {
    socklen_t fl = sizeof(from);
    int n = recvfrom(udpFd, buf, sizeof(buf), MSG_DONTWAIT,
                     (struct sockaddr *)&from, &fl);
    if (n < 0) return;
    if (n < 8 || memcmp(buf, "BCN1", 4) != 0) continue;
    uint8_t idx = buf[5];
    uint16_t seq = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
    int slot = udpFindHold(seq);
    if (slot < 0 || idx >= udpHold[slot].n || udpHold[slot].nacks >= 2) continue;
    size_t clen = udpHold[slot].len[idx];
    if (!clen || clen > UDP_CHUNK) continue;
    udpHold[slot].nacks++;
    udpBuild(udpRepair, idx, udpHold[slot].n, seq, udpHoldPtr(slot, idx), clen);
    if (udpSendWire(udpRepair, 38 + clen)) {
      static uint32_t lastLog = 0;
      if (millis() - lastLog > 800) {
        lastLog = millis();
        Serial.printf("[UDP] nack seq=%u idx=%u\n", seq, idx);
      }
    }
  }
}

static void udpService() {
  udpPollNack();
}

static bool sendVideoUdp(const uint8_t *jpeg, size_t len) {
  if (!jpeg || !udpHoldBuf || !udpOpen()) return false;
  size_t nlen = jpegTrim(jpeg, len);
  if (!nlen) {
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 2000) {
      lastLog = millis();
      Serial.printf("[CAM] jpeg no eoi bytes=%u\n", (unsigned)len);
    }
    return false;
  }
  uint8_t nchunks = (uint8_t)((nlen + UDP_CHUNK - 1) / UDP_CHUNK);
  if (nchunks == 0 || nchunks > UDP_HOLD_CHUNKS) {
    tuneLiveSize(nlen, 0);
    return false;
  }
  uint16_t seq = ++udpVideoSeq;
  int slot = udpHoldPos;
  udpHoldPos = (uint8_t)((udpHoldPos + 1) % UDP_HOLD_N);
  udpHold[slot].seq = seq;
  udpHold[slot].n = nchunks;
  udpHold[slot].nacks = 0;
  size_t off = 0;
  size_t pktLen[UDP_HOLD_CHUNKS];
  for (uint8_t i = 0; i < nchunks; i++) {
    size_t n = nlen - off;
    if (n > UDP_CHUNK) n = UDP_CHUNK;
    udpHold[slot].len[i] = (uint16_t)n;
    memcpy(udpHoldPtr(slot, i), jpeg + off, n);
    udpBuild(udpWire[i], i, nchunks, seq, jpeg + off, n);
    pktLen[i] = 38 + n;
    off += n;
  }
  uint32_t t0 = millis();
  bool all = true;
  for (uint8_t i = 0; i < nchunks; i++) {
    if (!udpSendWire(udpWire[i], pktLen[i])) all = false;
    if (i + 1 < nchunks) vTaskDelay(pdMS_TO_TICKS(12));
  }
  lastVideoSendMs = millis() - t0;
  lastJpegBytes = nlen;
  if (!camIsHd) tuneLiveSize(nlen, lastVideoSendMs);
  if (all) ++txVideoFrames;
  else ++txVideoDrops;
  return true;
}

static void freshenFb(void *buf, size_t len) {
  if (!buf || len < 4) return;
  const uintptr_t line = 64;
  uintptr_t addr = (uintptr_t)buf;
  uintptr_t start = addr & ~(line - 1);
  uintptr_t end = (addr + len + line - 1) & ~(line - 1);
  esp_cache_msync((void *)start, end - start, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
}

static bool sendVideoFrame() {
  // The camera task owns esp_camera_fb_get(). Calling it here blocked the
  // websocket handshake for seconds, so the dashboard stayed on NO SIGNAL.
  if (!visualOn() || !txPacket || !grabBuf || !grabMu || !wsConnected) {
    ++txVideoDrops;
    return false;
  }
  size_t len = 0;
  static uint32_t lastSeq = 0;
  if (xSemaphoreTake(grabMu, pdMS_TO_TICKS(15)) != pdTRUE) {
    ++txVideoDrops;
    return false;
  }
  uint32_t seq = grabSeq;
  len = grabLen;
  bool fresh = seq != lastSeq && len > 128 && len + 1 <= txPacketCap;
  if (fresh) {
    memcpy(txPacket + 1, grabBuf, len);
    lastSeq = seq;
  }
  xSemaphoreGive(grabMu);
  if (!fresh) {
    ++txVideoSkips;
    return false;
  }
  txPacket[0] = 0x01;
  lastJpegBytes = len;
  if (!queueVideoFrame(txPacket, len + 1)) {
    ++txVideoDrops;
    return false;
  }
  return true;
}

static void applySessionCmd() {
  int cmd = sessionCmd;
  if (!cmd) return;
  sessionCmd = 0;

  if (cmd == 3) {
    sessionStop();
  } else {
    if (sessionId.length()) sessionStop();
    if (cmd == 1) sessionStart("audio");
    if (cmd == 2) sessionStart("video");
  }
  stateDirty = false;
  postDeviceState();
}

// ============================================================
// AUDIO CAPTURE — firmware.zip: shift the INMP441 word and store it
// ============================================================

static void audioCaptureTask(void *) {
  const size_t RAW_N = 256;
  int32_t raw[RAW_N];
  int16_t pcm[RAW_N];
  for (;;) {
    bool live = audioWsConnected || audioEnabled || videoEnabled || pttHeld;
    if (!audioRing) {
      vTaskDelay(pdMS_TO_TICKS(40));
      continue;
    }
    size_t bytes = 0;
    esp_err_t e = i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytes, pdMS_TO_TICKS(100));
    if (!live) continue;
    if (e != ESP_OK || bytes < sizeof(int32_t)) continue;
    size_t n = bytes / sizeof(int32_t);
    // Soft single-pole high-pass around 50 Hz (R = 252/256). Below 50 Hz is
    // mic handling noise and DC bias; speech fundamentals start ~120 Hz.
    // Two poles at 150 Hz (the original) were eating the consonants and gave
    // the "radio" timbre. One gentle pole keeps speech intact.
    static int32_t hp_x = 0, hp_y = 0;
    static int gain = 256;     // 1.0x baseline
    int32_t peak = 0;
    const int R_HP = 252;
    for (size_t i = 0; i < n; i++) {
      int32_t s = raw[i] >> MIC_SHIFT;
      int32_t y = s - hp_x + ((hp_y * R_HP) >> 8);
      hp_x = s;
      hp_y = y;
      int32_t out = (y * gain) >> 8;
      // Tanh-style soft clip starting at 22000. Hard clip at 26k (the previous
      // setting) added odd harmonics that read as "radio distortion" on playback.
      if (out > 22000) {
        int32_t over = out - 22000;
        if (over > 32767) over = 32767;
        out = 22000 + ((over * 256) / (256 + (over >> 4)));
      } else if (out < -22000) {
        int32_t over = -22000 - out;
        if (over > 32767) over = 32767;
        out = -22000 - ((over * 256) / (256 + (over >> 4)));
      }
      if (out > 32767) out = 32767;
      if (out < -32768) out = -32768;
      int32_t mag = out < 0 ? -out : out;
      if (mag > peak) peak = mag;
      pcm[i] = (int16_t)out;
    }
    // Symmetric AGC. The previous code dropped gain 25% in one step but
    // recovered only 1 LSB per chunk (~80 ms), so a single loud blip pinned
    // gain to 96 and quiet speech stayed half-volume.
    if (peak > 26000) {
      if (gain > 128) gain -= 8;
    } else if (peak < 4000) {
      if (gain < 512) gain += 4;
    }
    ringPush(pcm, n);
  }
}

static void initMic() {
  audioRing = (int16_t *)heap_caps_malloc(AUDIO_RING_SAMPLES * sizeof(int16_t),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!audioRing) {
    Serial.println("[MIC] PSRAM ring alloc failed, falling back to internal");
    audioRing = (int16_t *)malloc(AUDIO_RING_SAMPLES * sizeof(int16_t));
  }
  if (!audioRing) {
    Serial.println("[MIC] audio ring alloc failed");
    return;
  }
  memset(audioRing, 0, AUDIO_RING_SAMPLES * sizeof(int16_t));
  if (!ringMu) ringMu = xSemaphoreCreateMutex();

  i2s_config_t c = {};
  c.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  c.sample_rate = MIC_SAMPLE_RATE;
  c.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  c.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  c.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  c.dma_desc_num = 8;
  c.dma_frame_num = 256;
  c.use_apll = false;
  c.tx_desc_auto_clear = false;
  c.fixed_mclk = 0;

  i2s_pin_config_t p = {};
  p.bck_io_num = MIC_I2S_BCLK;
  p.ws_io_num = MIC_I2S_WS;
  p.data_out_num = I2S_PIN_NO_CHANGE;
  p.data_in_num = MIC_I2S_DATA;

  i2s_driver_install(I2S_NUM_0, &c, 0, nullptr);
  i2s_set_pin(I2S_NUM_0, &p);
  i2s_zero_dma_buffer(I2S_NUM_0);
  xTaskCreatePinnedToCore(audioCaptureTask, "mic", 4096, nullptr, 6, nullptr, 0);
  Serial.printf("[MIC] %d Hz, %d ms chunks, ring=%d ms\n",
                MIC_SAMPLE_RATE, AUDIO_CHUNK_MS, AUDIO_RING_MS);
}

// ============================================================
// CAMERA
// ============================================================

static bool initCam() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = CAM_PIN_Y2;
  c.pin_d1 = CAM_PIN_Y3;
  c.pin_d2 = CAM_PIN_Y4;
  c.pin_d3 = CAM_PIN_Y5;
  c.pin_d4 = CAM_PIN_Y6;
  c.pin_d5 = CAM_PIN_Y7;
  c.pin_d6 = CAM_PIN_Y8;
  c.pin_d7 = CAM_PIN_Y9;
  c.pin_xclk = CAM_PIN_XCLK;
  c.pin_pclk = CAM_PIN_PCLK;
  c.pin_vsync = CAM_PIN_VSYNC;
  c.pin_href = CAM_PIN_HREF;
  c.pin_sccb_sda = CAM_PIN_SIOD;
  c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_pwdn = CAM_PIN_PWDN;
  c.pin_reset = CAM_PIN_RESET;
  // 16 MHz produced no JPEG at all (sig stayed 0) and the link never
  // finished the handshake. 20 MHz is the clock that actually outputs frames.
  // 24 MHz hits OV5640's max 720p@30fps ceiling; stay at 20 MHz if your clone
  // is unstable at 24.
  c.xclk_freq_hz = 24000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;
  c.jpeg_quality = JPEG_QUALITY;
  // 10-user scene: keep only one framebuffer in PSRAM. A 1080p JPEG is
  // ~310 KB, so 1 buf = 310 KB vs 620 KB. Tiny tearing is acceptable for
  // streaming; the second buffer is a record-only luxury we can't afford.
  c.fb_count = 1;
  // LATEST can hand back a buffer the sensor is still filling. That is a stripe.
  c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  c.fb_location = CAMERA_FB_IN_PSRAM;

  esp_err_t e = esp_camera_init(&c);
  if (e != ESP_OK) {
    Serial.printf("[CAM] init failed 0x%x\n", e);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    // 10-user scene: OV5640 already does its own AWB/AEC well; the legacy
    // OV2640 workarounds (aec2, lenc, bpc, wpc, raw_gma, dcw) cost cycles per
    // frame and shave a couple fps off the OV5640. Leave brightness/contrast/
    // saturation at neutral so night-vision presets can do their own thing.
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_aec2(s, 0);
    s->set_raw_gma(s, 0);
    s->set_lenc(s, 0);
    s->set_bpc(s, 0);
    s->set_wpc(s, 0);
    s->set_dcw(s, 0);
    s->set_quality(s, LIVE_JPEG_Q);
    s->set_vflip(s, CAM_VFLIP);
    s->set_hmirror(s, CAM_HMIRROR);
  }

  Serial.printf("[CAM] live %dx%d q=%d @ %d FPS · record %dx%d q=%d @ %d FPS\n",
                LIVE_WIDTH, LIVE_HEIGHT, LIVE_JPEG_Q, LIVE_FPS,
                REC_WIDTH, REC_HEIGHT, REC_JPEG_Q, REC_FPS);
  return true;
}

// ============================================================
// WIFI
// ============================================================

static void provisionNetwork() {
  PortalConfig net;
  net.deviceId = DEVICE_ID;
  bool haveCfg = portalLoad(net);
  if (net.deviceId.length()) deviceId = net.deviceId;

  bool forcePortal = digitalRead(BTN_POWER) == LOW;
  if (forcePortal) {
    Serial.println("[PORTAL] GPIO21 held at boot — opening setup");
  }

  String portalMsg;
  bool joined = false;
  if (!forcePortal && haveCfg) {
    joined = portalConnectSta(net, 25000);
    if (!joined) {
      portalMsg = "Gagal gabung ke '" + net.ssid +
                  "'. Pakai Wi-Fi 2.4 GHz dan cek password hotspot.";
    }
  }
  if (forcePortal || !haveCfg || !joined) {
    rgb(0, 160, 200);
    portalRun(net, portalMsg);
  }
  deviceId = net.deviceId;
  pttToken = net.pttToken;
  Serial.printf("[BODYCAM] device id = %s ptt_token=%s\n",
                deviceId.c_str(), pttToken.length() ? "yes" : "no");
}

struct BtnDebounce {
  int pin;
  bool last;
  bool held;
  uint32_t tChange;
};

static uint32_t btnIgnoreUntil = 0;

static void btnSeed(BtnDebounce &b) {
  bool down = digitalRead(b.pin) == LOW;
  b.last = down;
  b.held = down; // already held at boot ≠ a new press
  b.tChange = millis();
}

static bool btnPressed(BtnDebounce &b) {
  if ((int32_t)(millis() - btnIgnoreUntil) < 0) return false;
  bool down = digitalRead(b.pin) == LOW;
  uint32_t now = millis();
  if (down != b.last) {
    b.last = down;
    b.tChange = now;
    return false;
  }
  if (now - b.tChange < 40) return false;
  if (down && !b.held) {
    b.held = true;
    return true;
  }
  if (!down) b.held = false;
  return false;
}

// ============================================================
// AUDIO TX — owns the audio WebSocket. HTTP is only a last resort.
static void audioTxTask(void *) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      stopAudioWebSocket();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    // Open audio WS only after /ws/device is up so two handshakes do not
    // fight for the few lwIP sockets on a filtered meeting LAN.
    if (wsConnected) startAudioWebSocket();
    // Finish a half-sent mic frame before loop() writes anything else.
    if (audioWsConnected && audioLen && audioOff) {
      pumpAudio();
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    audioWs.loop();

    if (!(streamEnabled || audioEnabled || videoEnabled || pttHeld)) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (audioWsConnected) {
      // Mic stays on this socket. That is what feeds the HT and the speaker
      // downlink. UDP is video only, so a late picture cannot drop the radio.
      if (ringCount() > (size_t)AUDIO_TX_SAMPLES * 4) {
        ringKeepLatest((size_t)AUDIO_TX_SAMPLES * 2);
      }
      if (audioLen) pumpAudio();
      else sendAudioWs();
      vTaskDelay(pdMS_TO_TICKS(audioLen ? 1 : 5));
      continue;
    }
    // HTTP fallback only while recording a file. Live stays on WS so a
    // down socket is not drowned by POST /api/v1/audio.
    if (audioEnabled || videoEnabled) sendAudioChunk();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ============================================================
// HOUSEKEEPING — every blocking HTTP call lives here, never in the video task.
static void houseKeepTask(void *) {
  uint32_t lastHb = 0;
  uint32_t linkOkAt = millis();
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      linkOkAt = millis();
      vTaskDelay(pdMS_TO_TICKS(300));
      continue;
    }

    // Sockets down: bounce WS back to the public VPS. Never scan the LAN.
    if (wsConnected || audioWsConnected) {
      linkOkAt = millis();
    } else if (millis() - linkOkAt > 60000) {
      Serial.printf("[NET] still no VPS from %s — WS retry %s:%d\n",
                    WiFi.localIP().toString().c_str(), SERVER_HOST, SERVER_PORT);
      serverHost = SERVER_HOST;
      stopWebSocket();
      stopAudioWebSocket();
      WiFi.setSleep(true);
      linkOkAt = millis();
    }

    applySessionCmd();
    if (stateDirty) {
      stateDirty = false;
      postDeviceState();
    }
    uint32_t now = millis();
    if (now - lastHb >= 5000) {
      lastHb = now;
      postDeviceState();
      Serial.printf("[STAT] ws=%d aws=%d ptt=%d sos=%d gps=%d rx=%d sats=%d audio=%lu drops=%lu video=%lu drops=%lu skip=%lu send=%ums jpg=%u sig=%08x ring=%u RSSI=%d\n",
                    wsConnected,
                    audioWsConnected,
                    (int)pttHeld,
                    (int)sosActive,
                    (int)gpsHasFix(),
                    (int)gpsHasRx(),
                    gpsSatellites(),
                    (unsigned long)txAudioPackets,
                    (unsigned long)txAudioDrops,
                    (unsigned long)txVideoFrames,
                    (unsigned long)txVideoDrops,
                    (unsigned long)txVideoSkips,
                    (unsigned)lastVideoSendMs,
                    (unsigned)lastJpegBytes,
                    (unsigned)jpegSig,
                    (unsigned)ringCount(),
                    WiFi.RSSI());
#if RT_STREAM
      rtHeartbeat(WiFi.RSSI(), ESP.getFreeHeap());
      rtAdaptCam();
#endif
    }
#if RT_STREAM
    rtPoll();
#endif
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================
// CAMERA GRAB — this task is the only one allowed to call fb_get.
// The websocket task must not block here or the link drops.
// ============================================================
static void camGrabTask(void *) {
  uint32_t prevSig = 0;
  int same = 0;
  uint32_t lastKick = 0;
  uint32_t lastGood = millis();
  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      if (millis() - lastGood > 5000) {
        lastGood = millis();
        Serial.println("[CAM] stalled — restart sensor");
        esp_camera_deinit();
        camIsHd = false;
        initCam();
      }
      vTaskDelay(pdMS_TO_TICKS(30));
      continue;
    }
    size_t len = fb->len;
    // 10-user scene: drop the 1280x720 / 48 KB caps so OV5640 1080p frames
    // (1920x1080, ~50-70 KB @ q=12) are accepted by the grab path. The hard
    // upper bound is now txPacketCap (256 KB), which covers the largest
    // expected HD JPEG with headroom.
    bool ok = grabBuf && grabMu && fb->buf && fb->width >= 160 && fb->height >= 120 &&
              fb->width <= 1920 && fb->height <= 1080 &&
              len > 128 && len <= txPacketCap;
    if (ok) {
      // Look for FFD9 anywhere. Checking only the last 32 bytes threw
      // away later frames, grabSeq froze, and the dashboard stayed black.
      size_t end = jpegTrim(fb->buf, len);
      if (!end) ok = false;
      else len = end;
    }
    if (!ok) {
      static uint32_t lastBad = 0;
      if (millis() - lastBad > 2000) {
        lastBad = millis();
        Serial.printf("[CAM] drop len=%u\n", (unsigned)fb->len);
      }
    }
    if (ok) {
      uint32_t sig = (uint32_t)len * 16777619u;
      size_t step = len / 8;
      if (step < 1) step = 1;
      for (size_t i = 0; i < 8 && i * step < len; i++) sig = sig * 16777619u ^ fb->buf[i * step];
      if (xSemaphoreTake(grabMu, pdMS_TO_TICKS(40)) == pdTRUE) {
        memcpy(grabBuf, fb->buf, len);
        grabLen = len;
        grabSeq++;
        jpegSig = sig;
        lastGood = millis();
        xSemaphoreGive(grabMu);
      }
      if (sig == prevSig) {
        if (++same >= 30 && millis() - lastKick > 20000) {
          lastKick = millis();
          same = 0;
          Serial.printf("[CAM] frozen sig=%08x bytes=%u — restart sensor\n", sig, (unsigned)len);
          esp_camera_fb_return(fb);
          esp_camera_deinit();
          camIsHd = false;
          initCam();
          continue;
        }
      } else {
        same = 0;
        prevSig = sig;
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    esp_camera_fb_return(fb);
  }
}

// ============================================================
// VIDEO / WS TX — only this task touches WebSocketsClient.
// ============================================================

// Video only. No HTTP, no audio — nothing here may block for more than a frame.
static void streamTxTask(void *) {
  uint32_t nextFrame = millis();
  uint32_t wsUpAt = 0;

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      stopWebSocket();
      wsLen = 0;
      wsUpAt = 0;
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // Finish the JPEG before loop(). loop() peeks the socket and was
    // resetting the link ("Connection lost") in the middle of a frame.
    if (wsLen) {
      pumpVideo();
      vTaskDelay(pdMS_TO_TICKS(12));
      continue;
    }

    startWebSocket();
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

    streamWs.loop();
    udpService();
    if (wsConnected) {
      if (!wsUpAt) wsUpAt = millis();
    } else {
      wsUpAt = 0;
    }

    // Whole JPEG on the video socket. UDP pieces were arriving torn.
    // 10-user scene: FPS flips between live (30) and record (15). LIVE_FPS at
    // 30fps keeps the JS board from queueing; record drops to 15fps per the
    // OV5640 sensor ceiling.
    const uint32_t framePeriod = 1000 / (camIsHd ? REC_FPS : LIVE_FPS);

    uint32_t now = millis();
    if ((int32_t)(now - nextFrame) >= 0) {
      bool sent = false;
      bool hold = !wsConnected || (millis() - wsUpAt) < 500;
      if (!visualOn() || hold) {
        sent = false;
      } else {
        sent = sendVideoFrame();
      }
      now = millis();
      nextFrame = now + (sent ? framePeriod : 80);
    }

    now = millis();
    int32_t waitMs = visualOn() ? (int32_t)(nextFrame - now) : 10;
    if (waitMs < 1) waitMs = 1;
    if (waitMs > 8) waitMs = 8;
    vTaskDelay(pdMS_TO_TICKS((uint32_t)waitMs));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  txMu = xSemaphoreCreateMutex();
  udpMu = xSemaphoreCreateMutex();

  led(true);
#if USE_RGB_LED
  pixel.begin();
  pixel.clear();
  pixel.show();
#endif

  pinMode(BTN_AUDIO, INPUT_PULLUP);
  pinMode(BTN_VIDEO, INPUT_PULLUP);
  pinMode(BTN_PTT, INPUT_PULLUP);
  pinMode(BTN_SOS, INPUT_PULLUP);
  nightIr(false);

  provisionNetwork();
  bindPublicServer();

  if (!initCam()) rgb(255, 0, 255);
#if RT_STREAM
  rtBegin(serverHost.c_str(), deviceId.c_str(), SERVER_TOKEN);
  rtApplyCam();
#endif
  nightVision = false;
  applyCamNight(false);
  gpsBegin();
  speakerBegin();

  // Drain the sensor immediately so DMA cannot overflow during WS connect.
  // 10-user scene: 1080p JPEG is ~70 KB max — 256 KB headroom avoids a forced
  // re-alloc path on the very first HD frame.
  txPacketCap = 256 * 1024;
  txPacket = (uint8_t *)heap_caps_malloc(txPacketCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!txPacket) {
    txPacketCap = 48 * 1024;
    txPacket = (uint8_t *)malloc(txPacketCap);
  }
  if (!txPacket) {
    txPacketCap = 0;
    Serial.println("[WS] tx buffer alloc failed");
  }
  grabMu = xSemaphoreCreateMutex();
  if (txPacketCap) {
    grabBuf = (uint8_t *)heap_caps_malloc(txPacketCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!grabBuf) grabBuf = (uint8_t *)malloc(txPacketCap);
  }
  if (!grabBuf) Serial.println("[CAM] grab buffer alloc failed");
  udpRememberInit();
  // Internal RAM. A PSRAM send buffer was what the Wi-Fi stack aborted
  // ("Connection lost") after each JPEG. 64 KB covers a 1080p frame so we
  // don't fall back to the 48 KB realloc during the first HD frame.
  wsFrameCap = 64 * 1024;
  wsFrame = (uint8_t *)heap_caps_malloc(wsFrameCap, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!wsFrame) wsFrame = (uint8_t *)malloc(wsFrameCap);
  if (!wsFrame) {
    wsFrameCap = 0;
    Serial.println("[WS] frame buffer alloc failed");
  }
  streamEnabled = true;
  xTaskCreatePinnedToCore(camGrabTask, "camgrab", 8192, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(streamTxTask, "streamtx", 12288, nullptr, 4, nullptr, 0);

  initMic();

  nightVision = false;
  applyCamNight(false);
  btnIgnoreUntil = millis() + 1200;
  stateLed();
  postDeviceState();

  xTaskCreatePinnedToCore(audioTxTask, "audiotx", 8192, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(houseKeepTask, "house", 8192, nullptr, 1, nullptr, 0);

  led(false);
  Serial.printf("[BODYCAM] live A/V started - %dx%d @ %d FPS, audio=%d Hz/%d ms\n",
                STREAM_WIDTH, STREAM_HEIGHT, STREAM_FPS,
                MIC_SAMPLE_RATE, AUDIO_CHUNK_MS);
}

void loop() {
  static BtnDebounce bVideo = {BTN_VIDEO, false, false, 0};
  static bool seeded = false;
  static bool audioDown = false;
  static uint32_t audioAt = 0;
  static bool audioResetDone = false;
  static bool videoDown = false;
  static uint32_t videoAt = 0;
  static bool videoResetDone = false;
  static bool pttArmed = false;
  static bool pttRawLast = false;
  static uint32_t pttEdgeAt = 0;
  if (!seeded) {
    btnSeed(bVideo);
    pttRawLast = digitalRead(BTN_PTT) == LOW;
    pttArmed = !pttRawLast;  // pin held at boot is not PTT
    pttEdgeAt = millis();
    seeded = true;
  }

  // GPIO 1: short press = audio record.
  bool audioRaw = digitalRead(BTN_AUDIO) == LOW;
  if (audioRaw) {
    if (!audioDown) {
      audioDown = true;
      audioAt = millis();
      audioResetDone = false;
    }
  } else if (audioDown) {
    uint32_t held = millis() - audioAt;
    audioDown = false;
    if (!audioResetDone && held >= 40) {
      audioEnabled = !audioEnabled;
      if (audioEnabled) {
        videoEnabled = false;
        ringClear();
        sessionCmd = 1;
      } else {
        sessionCmd = 3;
      }
      stateDirty = true;
      stateLed();
      Serial.printf("[BTN] audio=%d visual=%d stream=%d\n",
                    (int)audioEnabled, visualOn(), (int)streamEnabled);
    }
  }

  // GPIO 3: VIDEO record button.
  // Short press (< 8s) = toggle video recording.
  // Hold for 8s = clear Wi-Fi credentials and reboot into captive portal.
  bool videoRaw = digitalRead(BTN_VIDEO) == LOW;
  uint32_t videoNow = millis();
  if (videoRaw) {
    if (!videoDown) {
      videoDown = true;
      videoAt = videoNow;
      videoResetDone = false;
    } else if (!videoResetDone && (videoNow - videoAt) >= WIFI_RESET_HOLD_MS) {
      videoResetDone = true;
      Serial.println("[BTN] hold VIDEO 8s — reset WiFi / captive portal");
      rgb(255, 80, 0);
      portalClear();
      delay(400);
      ESP.restart();
    }
  } else if (videoDown) {
    uint32_t held = videoNow - videoAt;
    videoDown = false;

    // Long press is consumed by the Wi-Fi reset above.
    if (!videoResetDone && held >= 40 && held < WIFI_RESET_HOLD_MS) {
      videoEnabled = !videoEnabled;
      if (videoEnabled) {
        audioEnabled = false;
        ringClear();
        sessionCmd = 2;
      } else {
        sessionCmd = 3;
      }
      stateDirty = true;
      stateLed();
      Serial.printf("[BTN] video=%d visual=%d stream=%d\n",
                    (int)videoEnabled, visualOn(), (int)streamEnabled);
    }
  }

  // GPIO 14: hold-to-talk (PQTALKIE). Replaces night vision.
  bool pttRaw = digitalRead(BTN_PTT) == LOW;
  if (!pttArmed) {
    if (!pttRaw) pttArmed = true;
  } else if (pttRaw != pttRawLast) {
    pttRawLast = pttRaw;
    pttEdgeAt = millis();
  } else if ((millis() - pttEdgeAt) >= 40 && pttRaw != pttHeld) {
    pttHeld = pttRaw;
    if (pttHeld) ringClear();
    speakerMute(pttHeld);
    stateDirty = true;
    stateLed();
    Serial.printf("[BTN] ptt=%d\n", (int)pttHeld);
  }

  // GPIO 21: short press = SOS on PQTALKIE (no Wi-Fi reset here).
  static bool sosDown = false;
  static uint32_t sosAt = 0;
  bool sosRaw = digitalRead(BTN_SOS) == LOW;
  if (sosRaw) {
    if (!sosDown) {
      sosDown = true;
      sosAt = millis();
    }
  } else if (sosDown) {
    uint32_t held = millis() - sosAt;
    sosDown = false;
    if (held >= 40) {
      sosActive = !sosActive;
      stateDirty = true;
      stateLed();
      Serial.printf("[BTN] sos=%d (GPIO21)\n", (int)sosActive);
    }
  }

  delay(1);
}
