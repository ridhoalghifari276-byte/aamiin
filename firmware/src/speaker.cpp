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
// Wiring: MAX98357 SD -> 3V3 selects RIGHT channel only.
// Use I2S_CHANNEL_FMT_ONLY_RIGHT (LEFT-only would be silent).

static const size_t SPK_RING = 8000;  // 0.5 s s16le mono
static const int SPK_GAIN = 24;
static int16_t *spkRing = nullptr;
static volatile size_t spkW = 0;
static volatile size_t spkR = 0;
static volatile bool muted = false;
static SemaphoreHandle_t spkMu = nullptr;
static uint32_t spkPushCount = 0;
static uint32_t spkWriteCount = 0;
static volatile bool spkReady = false;

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
  int16_t peak = 0;
  if (xSemaphoreTake(spkMu, pdMS_TO_TICKS(20)) != pdTRUE) return;
  for (size_t i = 0; i < n; i++) {
    int32_t v = (int32_t)src[i] * SPK_GAIN;
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    int16_t s = (int16_t)v;
    if (s < 0) s = (int16_t)(-s);
    if (s > peak) peak = s;
    size_t next = (spkW + 1) % SPK_RING;
    if (next == spkR) spkR = (spkR + 1) % SPK_RING;
    spkRing[spkW] = (int16_t)v;
    spkW = next;
  }
  spkPushCount++;
  uint32_t npush = spkPushCount;
  xSemaphoreGive(spkMu);
  if (npush <= 3 || (npush % 50) == 0) {
    Serial.printf("[SPK] push #%lu bytes=%u peak=%d muted=%d\n",
                  (unsigned long)npush, (unsigned)bytes, (int)peak, (int)muted);
  }
}

static esp_err_t spkWriteMono(const int16_t *mono, size_t n) {
  size_t done = 0;
  while (done < n) {
    size_t chunk = n - done;
    if (chunk > 128) chunk = 128;
    size_t written = 0;
    esp_err_t err = i2s_write(I2S_NUM_1, mono + done, chunk * sizeof(int16_t),
                              &written, pdMS_TO_TICKS(80));
    if (err != ESP_OK) return err;
    done += chunk;
  }
  return ESP_OK;
}

static void speakerTask(void *) {
  int16_t mono[128];
  for (;;) {
    if (!spkReady || muted || !spkRing || !spkMu) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    size_t take = 0;
    if (xSemaphoreTake(spkMu, pdMS_TO_TICKS(20)) == pdTRUE) {
      size_t avail = spkCountUnsafe();
      take = avail > 128 ? 128 : avail;
      for (size_t i = 0; i < take; i++) {
        mono[i] = spkRing[spkR];
        spkR = (spkR + 1) % SPK_RING;
      }
      xSemaphoreGive(spkMu);
    }
    if (!take) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    esp_err_t err = spkWriteMono(mono, take);
    spkWriteCount++;
    if (spkWriteCount <= 3 || (spkWriteCount % 100) == 0) {
      Serial.printf("[SPK] i2s #%lu samples=%u err=%d\n",
                    (unsigned long)spkWriteCount, (unsigned)take, (int)err);
    }
  }
}

static void speakerBeepDirect(int hz, int ms) {
  const int rate = MIC_SAMPLE_RATE;
  const int n = (rate * ms) / 1000;
  int16_t mono[64];
  int filled = 0;
  for (int i = 0; i < n; i++) {
    float t = (float)i / (float)rate;
    mono[filled++] = (int16_t)(sinf(2.0f * 3.1415926f * hz * t) * 28000.0f);
    if (filled >= 64) {
      spkWriteMono(mono, filled);
      filled = 0;
    }
  }
  if (filled) spkWriteMono(mono, filled);
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
  // SD pin tied to 3V3 → amp listens to RIGHT slot only.
  c.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
  c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  c.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  c.dma_desc_num = 8;
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
  if (i2s_set_pin(I2S_NUM_1, &p) != ESP_OK) {
    Serial.println("[SPK] I2S1 set_pin failed");
    return;
  }
  i2s_set_clk(I2S_NUM_1, MIC_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  i2s_zero_dma_buffer(I2S_NUM_1);

  Serial.printf("[SPK] I2S1 DOUT=%d BCLK=%d LRC=%d gain=%dx RIGHT (SD=3V3)\n",
                SPK_I2S_DOUT, SPK_I2S_BCLK, SPK_I2S_LRC, SPK_GAIN);

  Serial.println("[SPK] boot beep start");
  speakerBeepDirect(880, 180);
  vTaskDelay(pdMS_TO_TICKS(60));
  speakerBeepDirect(1320, 180);
  Serial.println("[SPK] boot beep done");

  spkReady = true;
  xTaskCreatePinnedToCore(speakerTask, "spk", 4096, nullptr, 3, nullptr, 0);
}
