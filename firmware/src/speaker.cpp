#include "speaker.h"
#include "config.h"
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include <string.h>

// Playback only. Mic stays on I2S_NUM_0 and is not touched here.

static const size_t SPK_RING = 16000;  // 1 s of s16le mono
static int16_t *spkRing = nullptr;
static volatile size_t spkW = 0;
static volatile size_t spkR = 0;
static volatile bool muted = false;
static portMUX_TYPE spkMux = portMUX_INITIALIZER_UNLOCKED;

static size_t spkCountUnsafe() {
  return (spkW + SPK_RING - spkR) % SPK_RING;
}

void speakerMute(bool on) {
  muted = on;
}

void speakerPush(const uint8_t *pcm, size_t bytes) {
  if (!spkRing || !pcm || bytes < 2) return;
  const int16_t *src = (const int16_t *)pcm;
  size_t n = bytes / 2;
  portENTER_CRITICAL(&spkMux);
  for (size_t i = 0; i < n; i++) {
    size_t next = (spkW + 1) % SPK_RING;
    if (next == spkR) spkR = (spkR + 1) % SPK_RING;
    spkRing[spkW] = src[i];
    spkW = next;
  }
  portEXIT_CRITICAL(&spkMux);
}

static void speakerTask(void *) {
  int16_t stereo[256];
  for (;;) {
    if (muted || !spkRing) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    size_t got = 0;
    portENTER_CRITICAL(&spkMux);
    size_t avail = spkCountUnsafe();
    size_t take = avail > 128 ? 128 : avail;
    for (size_t i = 0; i < take; i++) {
      int16_t s = spkRing[spkR];
      spkR = (spkR + 1) % SPK_RING;
      stereo[got++] = s;
      stereo[got++] = s;
    }
    portEXIT_CRITICAL(&spkMux);
    if (!take) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    size_t written = 0;
    i2s_write(I2S_NUM_1, stereo, got * sizeof(int16_t), &written, pdMS_TO_TICKS(50));
  }
}

void speakerBegin() {
  spkRing = (int16_t *)heap_caps_malloc(SPK_RING * sizeof(int16_t),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!spkRing) spkRing = (int16_t *)malloc(SPK_RING * sizeof(int16_t));
  if (spkRing) memset(spkRing, 0, SPK_RING * sizeof(int16_t));

  i2s_config_t c = {};
  c.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  c.sample_rate = MIC_SAMPLE_RATE;
  c.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
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
  i2s_zero_dma_buffer(I2S_NUM_1);
  xTaskCreatePinnedToCore(speakerTask, "spk", 4096, nullptr, 3, nullptr, 0);
  Serial.printf("[SPK] I2S1 DIN=%d BCLK=%d LRC=%d\n", SPK_I2S_DOUT, SPK_I2S_BCLK, SPK_I2S_LRC);
}
