#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WebSocketsClient.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <driver/i2s.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"
#include "portal.h"

// ============================================================
// ESP32 BODYCAM
//
// Audio: INMP441 >>14 into a PSRAM ring, WebSocket PCM on /ws/audio.
// Video: VGA JPEG over /ws/device @ STREAM_FPS (full FOV).
// ============================================================

static volatile bool streamEnabled = false;
static volatile bool audioEnabled = false;
static volatile bool videoEnabled = false;
static volatile bool nightVision = false;
static volatile bool stateDirty = false;

static Adafruit_NeoPixel pixel(1, RGB_LED, NEO_GRB + NEO_KHZ800);
static SemaphoreHandle_t stateMux = nullptr;
static WebSocketsClient streamWs;
static volatile bool wsConnected = false;
static bool wsStarted = false;
static WebSocketsClient audioWs;
static volatile bool audioWsConnected = false;
static bool audioWsStarted = false;
static uint8_t *txPacket = nullptr;   // reused video packet buffer (no per-frame malloc)
static size_t txPacketCap = 0;

static int16_t *audioRing = nullptr;
static volatile size_t ringWrite = 0;
static volatile size_t ringRead = 0;
static portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;

// Server address is discovered at runtime: the laptop's DHCP lease changes.
static String serverHost = SERVER_HOST;
static String deviceId = DEVICE_ID;
static Preferences prefs;

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
static uint32_t lastStats = 0;

static void led(bool on) {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, on ? HIGH : LOW);
}

static void rgb(uint8_t r, uint8_t g, uint8_t b) {
#if USE_RGB_LED
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
#endif
}

static bool visualOn() {
  // Audio-only record: no live picture. GPIO 21 off: no live picture
  // unless a video file is being recorded (frames still needed for mux).
  return videoEnabled || (streamEnabled && !audioEnabled);
}

static void nightIr(bool on) {
  pinMode(NIGHT_IR_PIN, OUTPUT);
  digitalWrite(NIGHT_IR_PIN, on ? HIGH : LOW);
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
    // Indoors the sensor was stretching exposure to ~1/4 s, which drops the
    // OV2640 to ~3 FPS and smears motion. Let AGC use gain instead: gain
    // ceiling high, aec2 (DSP night mode / auto frame-rate drop) off, and a
    // slightly darker AE target so frame time stays short.
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);
    s->set_gainceiling(s, GAINCEILING_16X);
    s->set_agc_gain(s, 0);
    s->set_aec_value(s, 200);
    s->set_ae_level(s, -1);
    s->set_brightness(s, 1);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_special_effect(s, 0);
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);
    s->set_dcw(s, 1);
  }
  nightIr(on);
}

static void stateLed() {
  if (nightVision && !videoEnabled && !audioEnabled) rgb(20, 180, 40);
  else if (!streamEnabled && !audioEnabled && !videoEnabled) rgb(0, 0, 0);
  else if (videoEnabled) rgb(255, 0, 0);
  else if (audioEnabled) rgb(0, 0, 255);
  else rgb(0, 255, 0);
}

static size_t ringCountUnsafe() {
  return (ringWrite + AUDIO_RING_SAMPLES - ringRead) % AUDIO_RING_SAMPLES;
}

static size_t ringCount() {
  portENTER_CRITICAL(&ringMux);
  size_t n = ringCountUnsafe();
  portEXIT_CRITICAL(&ringMux);
  return n;
}

static void ringClear() {
  portENTER_CRITICAL(&ringMux);
  ringWrite = 0;
  ringRead = 0;
  portEXIT_CRITICAL(&ringMux);
}

static void ringPush(const int16_t *src, size_t n) {
  portENTER_CRITICAL(&ringMux);
  // Trim the backlog in whole packets before writing, not one sample at a time
  // while writing. Dropping mid-write used to tear a word in half every time
  // the link stuttered; this drops only stale audio and lands on a packet edge.
  size_t held = ringCountUnsafe();
  if (held + n >= AUDIO_RING_SAMPLES) {
    size_t keep = AUDIO_LIVE_SAMPLES > n ? AUDIO_LIVE_SAMPLES - n : 0;
    size_t drop = held > keep ? held - keep : 0;
    ringRead = (ringRead + drop) % AUDIO_RING_SAMPLES;
  }
  // Two memcpys, not 320 single-sample writes with a modulo each. This runs
  // with interrupts masked on the same core as the I2S ISR, so the length of
  // this window is exactly what decides whether a DMA block gets dropped.
  size_t first = AUDIO_RING_SAMPLES - ringWrite;
  if (first > n) first = n;
  memcpy(audioRing + ringWrite, src, first * sizeof(int16_t));
  if (n > first) memcpy(audioRing, src + first, (n - first) * sizeof(int16_t));
  ringWrite = (ringWrite + n) % AUDIO_RING_SAMPLES;
  portEXIT_CRITICAL(&ringMux);
}

static size_t ringPop(int16_t *dst, size_t n) {
  portENTER_CRITICAL(&ringMux);
  size_t avail = ringCountUnsafe();
  size_t take = n < avail ? n : avail;
  size_t first = AUDIO_RING_SAMPLES - ringRead;
  if (first > take) first = take;
  memcpy(dst, audioRing + ringRead, first * sizeof(int16_t));
  if (take > first) memcpy(dst + first, audioRing, (take - first) * sizeof(int16_t));
  ringRead = (ringRead + take) % AUDIO_RING_SAMPLES;
  portEXIT_CRITICAL(&ringMux);
  return take;
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
  h.setConnectTimeout(500);
  h.setTimeout(800);
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
                ",\"visual\":" + (visualOn() ? "true" : "false") +
                ",\"nightvision\":" + (nightVision ? "true" : "false") +
                ",\"rssi\":" + String(WiFi.RSSI()) +
                ",\"ip\":\"" + WiFi.localIP().toString() + "\"}";
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
      streamWs.enableHeartbeat(15000, 3000, 2);
      Serial.printf("[WS] connected: %s\n", payload ? (char *)payload : "");
      break;
    case WStype_DISCONNECTED:
      wsConnected = false;
      Serial.println("[WS] disconnected");
      break;
    case WStype_ERROR:
      wsConnected = false;
      Serial.println("[WS] error");
      break;
    case WStype_TEXT:
      if (payload && length) {
        Serial.printf("[WS] server: %.*s\n", (int)length, (char *)payload);
      }
      break;
    default:
      break;
  }
}

static void audioWsEvent(WStype_t type, uint8_t *payload, size_t length) {
  (void)payload;
  (void)length;
  switch (type) {
    case WStype_CONNECTED:
      audioWsConnected = true;
      audioWs.enableHeartbeat(15000, 3000, 2);
      ringClear();  // start live, not with whatever piled up while offline
      Serial.println("[WS-AUDIO] connected");
      break;
    case WStype_DISCONNECTED:
      audioWsConnected = false;
      Serial.println("[WS-AUDIO] disconnected");
      break;
    case WStype_ERROR:
      audioWsConnected = false;
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
  audioWs.setReconnectInterval(2000);
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

// Does this address answer our gateway's /health? Used for discovery.
static bool probeHost(const String &ip, uint32_t connectMs) {
  WiFiClient c;
  if (!c.connect(ip.c_str(), SERVER_PORT, connectMs)) return false;
  c.print("GET /health HTTP/1.1\r\nHost: bodycam\r\nConnection: close\r\n\r\n");
  String resp;
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    while (c.available()) resp += (char)c.read();
    if (resp.indexOf("camera_rotate") >= 0) break;
    if (!c.connected() && !c.available()) break;
    delay(5);
  }
  c.stop();
  return resp.indexOf("camera_rotate") >= 0;
}

// The laptop's DHCP lease changes, which used to kill the link until the
// firmware was reflashed. Scan our own /24 for the gateway and remember it.
static bool discoverServer() {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (serverHost.length() && probeHost(serverHost, 600)) {
    Serial.printf("[NET] server ok at %s:%d\n", serverHost.c_str(), SERVER_PORT);
    return true;
  }
  IPAddress me = WiFi.localIP();
  Serial.printf("[NET] searching %d.%d.%d.0/24 for gateway...\n", me[0], me[1], me[2]);
  String prefix = String(me[0]) + "." + String(me[1]) + "." + String(me[2]) + ".";
  for (int i = 1; i <= 254; ++i) {
    if (i == me[3]) continue;
    String ip = prefix + String(i);
    if (probeHost(ip, 120)) {
      serverHost = ip;
      prefs.begin("bodycam", false);
      prefs.putString("srv", ip);
      prefs.end();
      Serial.printf("[NET] gateway found at %s:%d\n", ip.c_str(), SERVER_PORT);
      return true;
    }
  }
  Serial.println("[NET] gateway not found on this subnet");
  return false;
}

static void loadServerHost() {
  prefs.begin("bodycam", true);
  String saved = prefs.getString("srv", "");
  prefs.end();
  if (saved.length()) {
    serverHost = saved;
    Serial.printf("[NET] saved gateway %s\n", saved.c_str());
  }
}

static void startWebSocket() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (wsStarted) return;

  String path = String(SERVER_WS_PATH) +
                "?device=" + deviceId +
                "&token=" + SERVER_TOKEN;

  streamWs.onEvent(wsEvent);
  streamWs.setReconnectInterval(4000);
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
static bool sendAudioWs() {
  static int16_t txBuf[AUDIO_TX_SAMPLES];
  if (!audioWsConnected || !audioRing) return false;
  bool any = false;
  int sent = 0;
  // One packet per pass so a 4000-byte send cannot pile up in lwIP and
  // delay the JPEG socket that shares the radio.
  while (ringCount() >= (size_t)AUDIO_TX_SAMPLES && sent < 1) {
    size_t n = ringPop(txBuf, AUDIO_TX_SAMPLES);
    if (!n) break;
    if (!audioWs.sendBIN((uint8_t *)txBuf, n * sizeof(int16_t))) {
      ++txAudioDrops;
      break;
    }
    ++txAudioPackets;
    any = true;
    ++sent;
  }
  return any;
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

static bool sendVideoFrame() {
  if (!wsConnected || !visualOn()) return false;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ++txVideoDrops;
    return false;
  }

  bool ok = sendWsPacket(0x01, fb->buf, fb->len);
  esp_camera_fb_return(fb);
  if (ok) ++txVideoFrames;
  else ++txVideoDrops;
  return ok;
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
// AUDIO CAPTURE — same as firmware.zip: shift the INMP441 word and store it
// ============================================================

static void audioCaptureTask(void *) {
  const size_t RAW_N = 256;
  static int32_t raw[RAW_N];
  static int16_t pcm[RAW_N];
  for (;;) {
    bool need = streamEnabled || audioEnabled || videoEnabled;
    if (!need || !audioRing) {
      vTaskDelay(pdMS_TO_TICKS(40));
      continue;
    }
    size_t bytes = 0;
    esp_err_t e = i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytes, pdMS_TO_TICKS(100));
    if (e != ESP_OK || bytes < sizeof(int32_t)) continue;
    size_t n = bytes / sizeof(int32_t);
    for (size_t i = 0; i < n; i++) {
      int32_t s = raw[i] >> MIC_SHIFT;
      if (s > 32767) s = 32767;
      if (s < -32768) s = -32768;
      pcm[i] = (int16_t)s;
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
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;
  c.jpeg_quality = JPEG_QUALITY;
  c.fb_count = 3;
  c.grab_mode = CAMERA_GRAB_LATEST;
  c.fb_location = CAMERA_FB_IN_PSRAM;

  esp_err_t e = esp_camera_init(&c);
  if (e != ESP_OK) {
    Serial.printf("[CAM] init failed 0x%x\n", e);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_quality(s, JPEG_QUALITY);
    s->set_vflip(s, CAM_VFLIP);
    s->set_hmirror(s, CAM_HMIRROR);
    s->set_gainceiling(s, GAINCEILING_16X);
    s->set_aec2(s, 0);  // no auto frame-rate drop in dim rooms
    s->set_ae_level(s, -1);
    s->set_dcw(s, 1);
    s->set_special_effect(s, 0); // color until GPIO 14 night vision
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_saturation(s, 0);
  }

  Serial.printf("[CAM] VGA %dx%d JPEG q=%d @ %d FPS (full FOV)\n",
                STREAM_WIDTH, STREAM_HEIGHT, JPEG_QUALITY, STREAM_FPS);
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

  if (forcePortal || !haveCfg || !portalConnectSta(net, 20000)) {
    rgb(0, 160, 200);
    portalRun(net);  // saves then restarts
  }
  deviceId = net.deviceId;
  Serial.printf("[BODYCAM] device id = %s\n", deviceId.c_str());
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
    startAudioWebSocket();
    audioWs.loop();

    if (!(streamEnabled || audioEnabled || videoEnabled)) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (audioWsConnected) {
      sendAudioWs();
      vTaskDelay(pdMS_TO_TICKS(ringCount() >= (size_t)AUDIO_TX_SAMPLES ? 1 : 15));
      continue;
    }
    // Socket down: throw the backlog away instead of opening an HTTP
    // connection per chunk. Stale audio would only replay as delay later.
    ringClear();
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

    // Both sockets dead for a while usually means the laptop changed IP.
    if (wsConnected || audioWsConnected) {
      linkOkAt = millis();
    } else if (millis() - linkOkAt > 12000) {
      Serial.println("[NET] link down, re-discovering gateway");
      if (discoverServer()) {
        stopWebSocket();
        stopAudioWebSocket();
      }
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
      Serial.printf("[STAT] ws=%d aws=%d nv=%d audio=%lu drops=%lu video=%lu drops=%lu ring=%u RSSI=%d\n",
                    wsConnected,
                    audioWsConnected,
                    (int)nightVision,
                    (unsigned long)txAudioPackets,
                    (unsigned long)txAudioDrops,
                    (unsigned long)txVideoFrames,
                    (unsigned long)txVideoDrops,
                    (unsigned)ringCount(),
                    WiFi.RSSI());
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================
// VIDEO / WS TX — only this task touches WebSocketsClient.
// ============================================================

// Video only. No HTTP, no audio — nothing here may block for more than a frame.
static void streamTxTask(void *) {
  const uint32_t framePeriod = 1000 / STREAM_FPS;
  uint32_t nextFrame = millis();

  for (;;) {
    streamWs.loop();

    if (WiFi.status() != WL_CONNECTED) {
      stopWebSocket();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    startWebSocket();

    if (!wsConnected) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    uint32_t now = millis();
    if (visualOn() && (int32_t)(now - nextFrame) >= 0) {
      nextFrame += framePeriod;
      if ((int32_t)(now - nextFrame) > (int32_t)(framePeriod * 2)) {
        nextFrame = now + framePeriod;
      }
      sendVideoFrame();
    }

    now = millis();
    int32_t waitMs = visualOn() ? (int32_t)(nextFrame - now) : 10;
    if (waitMs < 1) waitMs = 1;
    if (waitMs > 10) waitMs = 10;
    vTaskDelay(pdMS_TO_TICKS((uint32_t)waitMs));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(STATUS_LED, OUTPUT);
  led(true);
#if USE_RGB_LED
  pixel.begin();
  pixel.clear();
  pixel.show();
#endif

  pinMode(BTN_AUDIO, INPUT_PULLUP);
  pinMode(BTN_VIDEO, INPUT_PULLUP);
  pinMode(BTN_STREAM, INPUT_PULLUP);
  pinMode(BTN_POWER, INPUT_PULLUP);
  pinMode(NIGHT_IR_PIN, OUTPUT);
  nightIr(false);

  provisionNetwork();

  if (!initCam()) rgb(255, 0, 255);
  nightVision = false;
  applyCamNight(false);
  initMic();
  loadServerHost();
  discoverServer();

  streamEnabled = true;
  nightVision = false;
  applyCamNight(false);
  btnIgnoreUntil = millis() + 1200;
  stateLed();
  postDeviceState();

  txPacketCap = 96 * 1024;
  txPacket = (uint8_t *)heap_caps_malloc(txPacketCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!txPacket) {
    txPacketCap = 48 * 1024;
    txPacket = (uint8_t *)malloc(txPacketCap);
  }
  if (!txPacket) {
    txPacketCap = 0;
    Serial.println("[WS] tx buffer alloc failed");
  }

  xTaskCreatePinnedToCore(audioTxTask, "audiotx", 8192, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(streamTxTask, "streamtx", 12288, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(houseKeepTask, "house", 8192, nullptr, 1, nullptr, 0);

  led(false);
  Serial.printf("[BODYCAM] live A/V started - %dx%d @ %d FPS, audio=%d Hz/%d ms\n",
                STREAM_WIDTH, STREAM_HEIGHT, STREAM_FPS,
                MIC_SAMPLE_RATE, AUDIO_CHUNK_MS);
}

void loop() {
  static BtnDebounce bAudio = {BTN_AUDIO, false, false, 0};
  static BtnDebounce bVideo = {BTN_VIDEO, false, false, 0};
  static BtnDebounce bNight = {BTN_STREAM, false, false, 0};
  static bool seeded = false;
  static bool powerDown = false;
  static uint32_t powerAt = 0;
  if (!seeded) {
    btnSeed(bAudio);
    btnSeed(bVideo);
    btnSeed(bNight);
    seeded = true;
  }

  // GPIO 1: audio record only; live picture off, mic keeps recording.
  if (btnPressed(bAudio)) {
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

  // GPIO 3: video recording + microphone.
  if (btnPressed(bVideo)) {
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

  // GPIO 14: night vision only on a real press (not a pin that is low at boot).
  if (btnPressed(bNight)) {
    nightVision = !nightVision;
    applyCamNight(nightVision);
    stateDirty = true;
    stateLed();
    Serial.printf("[BTN] nightvision=%d\n", (int)nightVision);
  }

  // GPIO 21: short press toggles livestream. Hold 3s clears Wi-Fi and
  // reopens the captive portal after reboot.
  bool pwr = digitalRead(BTN_POWER) == LOW;
  if (pwr) {
    if (!powerDown) {
      powerDown = true;
      powerAt = millis();
    } else if (millis() - powerAt >= POWER_HOLD_MS) {
      Serial.println("[BTN] hold GPIO21 — reset WiFi / captive portal");
      rgb(255, 80, 0);
      portalClear();
      delay(400);
      ESP.restart();
    }
  } else if (powerDown) {
    uint32_t held = millis() - powerAt;
    powerDown = false;
    if (held >= 40 && held < POWER_HOLD_MS) {
      streamEnabled = !streamEnabled;
      stateDirty = true;
      stateLed();
      Serial.printf("[BTN] stream=%d (GPIO21)\n", (int)streamEnabled);
    }
  }

  delay(1);
}
