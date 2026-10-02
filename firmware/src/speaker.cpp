#include "speaker.h"
#include "config.h"
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <math.h>
#include <string.h>

// Mic stays on legacy I2S_NUM_0 — ESP_I2S (new driver) cannot share the chip
// with the legacy driver ("CONFLICT! The new i2s driver can't work along with
// the legacy i2s driver"). Speaker uses legacy I2S_NUM_1 with the same audio
// behaviour as PQTALKIE radio FW 1.0.30: STEREO @ 16 kHz, L=R, 2 boot beeps.
//
// Wiring (unchanged): DIN38 BCLK40 LRC39, SD hardwired to 3V3 (RIGHT slot).

static const size_t SPK_RING = 8000;  // 0.5 s s16le mono
// SPK_GAIN boosts the downlink before the MAX98357A. The radio mic on the
// other bodycam peaks around +/-8000 after AGC, so gain 1 sounds thin. Gain
// 4 (+12 dB) lands typical speech near full-scale. Bump to 6 or 8 if it is
// still too quiet, but anything above ~8 starts to clip the bodycam-mic peaks.
static const int SPK_GAIN = 4;
static int16_t *spkRing = nullptr;
static volatile size_t spkW = 0;
static volatile size_t spkR = 0;
static volatile bool muted = false;
static volatile bool sirenOn = false;
static uint32_t sirenPhase = 0;
static uint32_t sirenCount = 0;
static SemaphoreHandle_t spkMu = nullptr;
static uint32_t spkPushCount = 0;

static size_t spkCountUnsafe() {
  return (spkW + SPK_RING - spkR) % SPK_RING;
}

void speakerMute(bool on) {
  muted = on;
}

void speakerSiren(bool on) {
  if (sirenOn == on) return;
  sirenOn = on;
  if (!on) {
    sirenPhase = 0;
    sirenCount = 0;
  }
  Serial.printf("[SPK] siren %s\n", on ? "ON" : "off");
}

// European hi-lo ambulance: 650 Hz then 980 Hz, 400 ms each.
static int16_t nextSiren() {
  const uint32_t half = (uint32_t)MIC_SAMPLE_RATE * 400 / 1000;
  uint32_t inc = ((sirenCount / half) & 1) ? 4014u : 2662u;
  sirenCount++;
  sirenPhase = (sirenPhase + inc) & 65535u;
  float a = sinf((float)sirenPhase * (2.0f * 3.1415926f / 65536.0f));
  return (int16_t)(a * 20000.0f);
}

void speakerPush(const uint8_t *pcm, size_t bytes) {
  if (!spkRing || !spkMu || !pcm || bytes < 2) return;
  const int16_t *src = (const int16_t *)pcm;
  size_t n = bytes / 2;
  int32_t peak = 0;
  if (xSemaphoreTake(spkMu, pdMS_TO_TICKS(20)) != pdTRUE) return;
  for (size_t i = 0; i < n; i++) {
    int32_t v = (int32_t)src[i] * SPK_GAIN;
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
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
  // Interleaved L,R — SD=3V3 uses RIGHT; L=R keeps both slots filled like radio FW.
  int16_t stereo[256];
  for (;;) {
    if (!spkRing || !spkMu) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    const bool alert = sirenOn;
    if (muted && !alert) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    size_t take = 0;
    int16_t radio[128];
    if (!muted && xSemaphoreTake(spkMu, pdMS_TO_TICKS(20)) == pdTRUE) {
      size_t avail = spkCountUnsafe();
      take = avail > 128 ? 128 : avail;
      for (size_t i = 0; i < take; i++) {
        radio[i] = spkRing[spkR];
        spkR = (spkR + 1) % SPK_RING;
      }
      xSemaphoreGive(spkMu);
    }
    size_t n = alert ? 128 : take;
    if (!n) {
      // Stopping the I2S clock between packets is the click/crackle.
      memset(stereo, 0, 64 * 2 * sizeof(int16_t));
      size_t written = 0;
      i2s_write(I2S_NUM_1, stereo, 64 * 2 * sizeof(int16_t), &written, pdMS_TO_TICKS(20));
      continue;
    }
    for (size_t i = 0; i < n; i++) {
      int32_t s = (i < take) ? (int32_t)radio[i] : 0;
      if (alert) {
        // Keep a little of the radio under the siren so a voice is still heard.
        s = s / 3 + (int32_t)nextSiren();
        if (s > 32767) s = 32767;
        if (s < -32768) s = -32768;
      }
      stereo[i * 2] = (int16_t)s;
      stereo[i * 2 + 1] = (int16_t)s;
    }
    size_t written = 0;
    i2s_write(I2S_NUM_1, stereo, n * 2 * sizeof(int16_t), &written, pdMS_TO_TICKS(50));
  }
}

static void bootBeep() {
  const int rate = MIC_SAMPLE_RATE;
  const int n = rate / 5;
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

  Serial.printf("[SPK] OK STEREO %dHz BCLK%d LRC%d DIN%d (SD=3V3 RIGHT)\n",
                MIC_SAMPLE_RATE, SPK_I2S_BCLK, SPK_I2S_LRC, SPK_I2S_DOUT);
  bootBeep();
  xTaskCreatePinnedToCore(speakerTask, "spk", 6144, nullptr, 3, nullptr, 0);
}
