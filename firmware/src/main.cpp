#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WebSocketsClient.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <driver/i2s.h>
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"
#include "portal.h"
#include "gps.h"
#include "speaker.h"

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
static volatile bool pttHeld = false;
static volatile bool sosActive = false;
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
  // Audio-only record: no live picture. GPIO 21 off: no live picture
  // unless a video file is being recorded (frames still needed for mux).
  return videoEnabled || (streamEnabled && !audioEnabled);
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
    // Outdoor / running: short exposure so motion does not smear, AGC for
    // brightness instead of dropping the frame rate (aec2).
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);
    s->set_gainceiling(s, GAINCEILING_16X);
    s->set_agc_gain(s, 0);
    s->set_aec_value(s, 180);
    s->set_ae_level(s, -1);
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_special_effect(s, 0);
    s->set_quality(s, JPEG_QUALITY);
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_vflip(s, CAM_VFLIP);
    s->set_hmirror(s, CAM_HMIRROR);
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);
    s->set_bpc(s, 0);
    s->set_wpc(s, 1);
    s->set_dcw(s, 1);
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
  for (size_t i = 0; i < n; i++) {
    size_t next = (ringWrite + 1) % AUDIO_RING_SAMPLES;
    if (next == ringRead) {
      ringRead = (ringRead + 1) % AUDIO_RING_SAMPLES;
    }
    audioRing[ringWrite] = src[i];
    ringWrite = next;
  }
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
      // Longer heartbeat — congested uplink was tripping 15s ping.
      streamWs.enableHeartbeat(30000, 8000, 3);
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
      audioWs.enableHeartbeat(30000, 8000, 3);
      ringClear();  // start live, not with whatever piled up while offline
      Serial.println("[WS-AUDIO] connected");
      break;
    case WStype_DISCONNECTED:
      audioWsConnected = false;
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
  // Faster reconnect on flaky MiFi/VTA uplinks.
  audioWs.setReconnectInterval(2500);
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
  streamWs.setReconnectInterval(2500);
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
  // Drain harder when ring is backing up (MiFi uplink lag).
  size_t backlog = ringCount();
  int maxBurst = backlog > (size_t)(AUDIO_TX_SAMPLES * 4) ? 4 : 2;
  while (ringCount() >= (size_t)AUDIO_TX_SAMPLES && sent < maxBurst) {
    size_t n = ringPop(txBuf, AUDIO_TX_SAMPLES);
    if (!n) break;
    if (!audioWs.sendBIN((uint8_t *)txBuf, n * sizeof(int16_t))) {
      ++txAudioDrops;
      // Drop a chunk of stale audio so the ring cannot stay pegged full.
      if (ringCount() > (size_t)(AUDIO_RING_SAMPLES / 2)) {
        ringPop(txBuf, AUDIO_TX_SAMPLES);
      }
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

static void drainCamFb() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
}

static bool sendVideoFrame() {
  // Grab + free the sensor buffer BEFORE sendBIN. Holding the FB across a
  // WebSocket write is what caused cam_hal FB-OVF and NO SIGNAL.
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ++txVideoDrops;
    return false;
  }
  size_t len = fb->len;
  bool copy_ok = wsConnected && visualOn() && txPacket &&
                 len > 128 && len + 1 <= txPacketCap;
  if (copy_ok) {
    txPacket[0] = 0x01;
    memcpy(txPacket + 1, fb->buf, len);
  }
  esp_camera_fb_return(fb);
  if (!copy_ok) {
    ++txVideoDrops;
    return false;
  }
  bool ok = streamWs.sendBIN(txPacket, len + 1);
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
    // Outdoor wind rumble sits below ~100 Hz. Cut it on-device so the
    // live feed is speech, not road/wind noise. R ≈ 0.961 at 16 kHz.
    static int32_t hpX = 0;
    static int32_t hpY = 0;
    for (size_t i = 0; i < n; i++) {
      int32_t s = raw[i] >> MIC_SHIFT;
      int32_t y = s - hpX + ((hpY * 246) >> 8);
      hpX = s;
      hpY = y;
      if (y > 32767) y = 32767;
      if (y < -32768) y = -32768;
      pcm[i] = (int16_t)y;
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
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_quality(s, JPEG_QUALITY);
    s->set_vflip(s, CAM_VFLIP);
    s->set_hmirror(s, CAM_HMIRROR);
    s->set_gainceiling(s, GAINCEILING_16X);
    s->set_aec2(s, 0);
    s->set_ae_level(s, -1);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_dcw(s, 1);
    s->set_special_effect(s, 0);
    s->set_lenc(s, 1);
    s->set_raw_gma(s, 1);
  }

  Serial.printf("[CAM] VGA %dx%d JPEG q=%d @ %d FPS (outdoor / run-stable)\n",
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
//
// Mic dual role (one INMP441):
//   1) Livestream / rec  → PCM ke gateway → terdengar di dashboard
//   2) PTT (GPIO 14)     → PCM yang sama di-feed ke PQTALKIE (HT TX)
// Speaker ESP hanya memutar RX radio (rekan / SOS), bukan monitor live.
// ============================================================
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
    audioWs.loop();

    if (!(streamEnabled || audioEnabled || videoEnabled || pttHeld)) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (audioWsConnected) {
      sendAudioWs();
      vTaskDelay(pdMS_TO_TICKS(ringCount() >= (size_t)AUDIO_TX_SAMPLES ? 1 : 15));
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
    } else if (millis() - linkOkAt > 20000) {
      Serial.printf("[NET] still no VPS from %s — WS retry %s:%d\n",
                    WiFi.localIP().toString().c_str(), SERVER_HOST, SERVER_PORT);
      serverHost = SERVER_HOST;
      stopWebSocket();
      stopAudioWebSocket();
      // Keep modem awake — sleep made MiFi reconnects worse.
      WiFi.setSleep(false);
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
      Serial.printf("[STAT] ws=%d aws=%d ptt=%d sos=%d gps=%d rx=%d sats=%d audio=%lu drops=%lu video=%lu drops=%lu ring=%u RSSI=%d\n",
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
      drainCamFb();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    startWebSocket();
    WiFi.setSleep(false);

    if (!wsConnected) {
      drainCamFb();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    uint32_t now = millis();
    if ((int32_t)(now - nextFrame) >= 0) {
      // Prefer audio socket: skip JPEG when mic ring is backing up.
      size_t backlog = ringCount();
      bool congested = backlog > (AUDIO_RING_SAMPLES / 4);
      if (congested) {
        drainCamFb();
        nextFrame = now + framePeriod * 2;
      } else {
        uint32_t t0 = now;
        bool ok = sendVideoFrame();
        now = millis();
        // On send fail, back off harder so lwIP can drain audio.
        nextFrame = t0 + (ok ? framePeriod : framePeriod * 3);
        if ((int32_t)(now - nextFrame) >= 0) {
          nextFrame = now + (ok ? 0 : framePeriod);
        }
      }
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
  WiFi.setSleep(false);
#if defined(WIFI_POWER_19_5dBm)
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
#endif
  bindPublicServer();

  if (!initCam()) rgb(255, 0, 255);
  nightVision = false;
  applyCamNight(false);
  gpsBegin();
  speakerBegin();

  // Drain the sensor immediately so DMA cannot overflow during WS connect.
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
  streamEnabled = true;
  xTaskCreatePinnedToCore(streamTxTask, "streamtx", 12288, nullptr, 4, nullptr, 1);

  initMic();

  nightVision = false;
  applyCamNight(false);
  btnIgnoreUntil = millis() + 1200;
  stateLed();
  postDeviceState();

  xTaskCreatePinnedToCore(audioTxTask, "audiotx", 8192, nullptr, 5, nullptr, 0);
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

  // GPIO 1: short press = audio record; hold 7s = reset Wi-Fi / captive portal.
  bool audioRaw = digitalRead(BTN_AUDIO) == LOW;
  if (audioRaw) {
    if (!audioDown) {
      audioDown = true;
      audioAt = millis();
      audioResetDone = false;
    } else if (!audioResetDone && (millis() - audioAt) >= WIFI_RESET_HOLD_MS) {
      Serial.println("[BTN] hold GPIO1 7s — reset WiFi / captive portal");
      rgb(255, 80, 0);
      portalClear();
      delay(400);
      ESP.restart();
    }
  } else if (audioDown) {
    uint32_t held = millis() - audioAt;
    audioDown = false;
    if (!audioResetDone && held >= 40 && held < WIFI_RESET_HOLD_MS) {
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

  // GPIO 14: hold-to-talk → mic role switches to HT (PQTALKIE).
  // While held: speaker muted so we do not play our own TX / echo.
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
