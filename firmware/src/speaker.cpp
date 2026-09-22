#include "speaker.h"
#include "config.h"
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <math.h>
#include <string.h>

// Playback only. Mic stays on I2S_NUM_0 and is not touched here.
//
// IMPORTANT: never touch SPIRAM inside portENTER_CRITICAL — that disables the
// flash/PSRAM cache and causes "Cache error / MMU entry fault" on ESP32-S3
// when HT downlink starts pushing PCM. Use a FreeRTOS mutex instead.
//
// MAX98357 SD/MODE (user wiring: SD not connected = floating):
//   float = stereo  → write L=R samples
//   VIN/3V3 = RIGHT only
//   GND = LEFT only / shutdown (depending on board)

static const size_t SPK_RING = 8000;  // 0.5 s s16le mono
static const int SPK_GAIN = 8;       // audible; soft-limit avoids square-wave crit
static int16_t *spkRing = nullptr;
static volatile size_t spkW = 0;
static volatile size_t spkR = 0;
static volatile bool muted = false;
static SemaphoreHandle_t spkMu = nullptr;
static uint32_t spkPushCount = 0;

static size_t spkCountUnsafe() {
  return (spkW + SPK_RING - spkR) % SPK_RING;
}

void speakerMute(bool on) {
  muted = on;
}

void speakerPush(const uint8_t *pcm, size_t bytes) {
  if (!spkRing || !spkMu || !pcm || bytes < 2) return;
  const int16_t *src = (const int16_t *)pcm;
  size_t n = bytes / 2;
  int32_t peak = 0;
  if (xSemaphoreTake(spkMu, pdMS_TO_TICKS(20)) != pdTRUE) return;
  for (size_t i = 0; i < n; i++) {
    int32_t v = (int32_t)src[i] * SPK_GAIN;
    if (v > 22000) v = 22000;
    if (v < -22000) v = -22000;
    int32_t a = v < 0 ? -v : v;
    if (a > peak) peak = a;
    size_t next = (spkW + 1) % SPK_RING;
    if (next == spkR) spkR = (spkR + 1) % SPK_RING;
    spkRing[spkW] = (int16_t)v;
    spkW = next;
  }
  spkPushCount++;
  uint32_t npush = spkPushCount;
  xSemaphoreGive(spkMu);
  if (npush <= 5 || (npush % 50) == 0) {
    Serial.printf("[SPK] push #%lu bytes=%u peak=%ld muted=%d\n",
                  (unsigned long)npush, (unsigned)bytes, (long)peak, (int)muted);
  }
}

static void speakerTask(void *) {
  // Stereo interleaved L,R — required when MAX98357 SD is floating.
  int16_t stereo[256];
  for (;;) {
    if (muted || !spkRing || !spkMu) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    size_t take = 0;
    if (xSemaphoreTake(spkMu, pdMS_TO_TICKS(20)) == pdTRUE) {
      size_t avail = spkCountUnsafe();
      take = avail > 128 ? 128 : avail;
      for (size_t i = 0; i < take; i++) {
        int16_t s = spkRing[spkR];
        spkR = (spkR + 1) % SPK_RING;
        stereo[i * 2] = s;
        stereo[i * 2 + 1] = s;
      }
      xSemaphoreGive(spkMu);
    }
    if (!take) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    size_t written = 0;
    i2s_write(I2S_NUM_1, stereo, take * 2 * sizeof(int16_t), &written, pdMS_TO_TICKS(50));
  }
}

static void bootBeep() {
  // Direct I2S tone so amp/wiring can be verified without HT.
  const int rate = MIC_SAMPLE_RATE;
  const int n = rate / 5;  // 200 ms
  int16_t *buf = (int16_t *)malloc((size_t)n * 2 * sizeof(int16_t));
  if (!buf) return;
  Serial.println("[SPK] boot beep start");
  for (int pass = 0; pass < 2; pass++) {
    float f = pass == 0 ? 880.0f : 1320.0f;
    for (int i = 0; i < n; i++) {
      float t = (float)i / (float)rate;
      int16_t s = (int16_t)(sinf(2.0f * 3.1415926f * f * t) * 12000.0f);
      buf[i * 2] = s;
      buf[i * 2 + 1] = s;
    }
    size_t off = 0;
    size_t total = (size_t)n * 2 * sizeof(int16_t);
    while (off < total) {
      size_t written = 0;
      i2s_write(I2S_NUM_1, (uint8_t *)buf + off, total - off, &written, pdMS_TO_TICKS(200));
      if (!written) break;
      off += written;
    }
    vTaskDelay(pdMS_TO_TICKS(80));
  }
  free(buf);
  i2s_zero_dma_buffer(I2S_NUM_1);
  Serial.println("[SPK] boot beep done");
}

void speakerBegin() {
  spkMu = xSemaphoreCreateMutex();
  spkRing = (int16_t *)heap_caps_malloc(SPK_RING * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!spkRing) {
    spkRing = (int16_t *)heap_caps_malloc(SPK_RING * sizeof(int16_t),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (!spkRing) spkRing = (int16_t *)malloc(SPK_RING * sizeof(int16_t));
  if (spkRing) memset(spkRing, 0, SPK_RING * sizeof(int16_t));
  else Serial.println("[SPK] ring alloc failed");

  i2s_config_t c = {};
  c.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  c.sample_rate = MIC_SAMPLE_RATE;
  c.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  // SD floating → stereo; duplicate mono into L and R.
  c.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  c.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  c.dma_desc_num = 6;
  c.dma_frame_num = 256;
  c.use_apll = false;
  c.tx_desc_auto_clear = true;
  c.fixed_mclk = 0;

  i2s_pin_config_t p = {};
  p.bck_io_num = SPK_I2S_BCLK;
  p.ws_io_num = SPK_I2S_LRC;
  p.data_out_num = SPK_I2S_DOUT;
  p.data_in_num = I2S_PIN_NO_CHANGE;

  if (i2s_driver_install(I2S_NUM_1, &c, 0, nullptr) != ESP_OK) {
    Serial.println("[SPK] I2S1 install failed");
    return;
  }
  i2s_set_pin(I2S_NUM_1, &p);
  i2s_set_clk(I2S_NUM_1, MIC_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  i2s_zero_dma_buffer(I2S_NUM_1);
  Serial.printf("[SPK] I2S1 DOUT=%d BCLK=%d LRC=%d gain=%dx STEREO\n",
                SPK_I2S_DOUT, SPK_I2S_BCLK, SPK_I2S_LRC, SPK_GAIN);
  bootBeep();
  xTaskCreatePinnedToCore(speakerTask, "spk", 4096, nullptr, 3, nullptr, 0);
}
