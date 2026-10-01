#pragma once

// ============================================================
// WIFI / SERVER
//
// First boot opens a captive portal (AP "BODYCAM-SETUP").
// Device ID + Wi-Fi are entered there and stored in NVS.
// These two strings are compile-time fallbacks only; leave them
// empty so the portal always runs until the unit is provisioned.
// Hold the VIDEO record button (GPIO 3) for 8 seconds during operation to
// clear Wi-Fi and re-open the portal. Hold GPIO 21 at boot also
// forces the portal.
// ============================================================
#define WIFI_SSID       ""
#define WIFI_PASSWORD   ""

#define AP_SSID         "BODYCAM-SETUP"
#define AP_PASSWORD     ""
#define AP_CHANNEL      1

// HTTP API (session/state)
// Public VPS. Firmware always uses this host — no LAN /24 scan.
#define SERVER_HOST     "45.250.101.17"
#define SERVER_PORT     7890
#define SERVER_WS_PATH  "/ws/device"
// Media follows the zip: signaling stays on TCP 7890, pictures and mic use UDP.
// One port for every bodycam. 50300 is inside the open range 50300-50400.
#define MEDIA_UDP_PORT  50300
// Live video uses BCRT on the same UDP port the gateway already has open.
// /ws/audio stays up for the HT radio.
#define RT_STREAM       1
#define RT_UDP_PORT     50300

#define DEVICE_ID       "bodycam-01"
#define SERVER_TOKEN    "X01040688"

// ============================================================
// CAMERA - ESP32-S3 CAM N16R8, DVP. OV5640 uses this same connector.
// OV2640 is what is on the board today; the driver reads the sensor id.
// ============================================================
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD    4
#define CAM_PIN_SIOC    5
#define CAM_PIN_Y9      16
#define CAM_PIN_Y8      17
#define CAM_PIN_Y7      18
#define CAM_PIN_Y6      12
#define CAM_PIN_Y5      10
#define CAM_PIN_Y4      8
#define CAM_PIN_Y3      9
#define CAM_PIN_Y2      11
#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK    13
#define CAM_VFLIP       0
#define CAM_HMIRROR     0

// ============================================================
// VIDEO — outdoor bodycam.
// Live (no record): VGA, moderate JPEG, steady FPS. Small frames stay
// smooth while walking/running and keep the server light.
// Record: 720p. Sharper file, slower FPS so the sensor does not overflow.
// ESP JPEG q: lower number = sharper, larger file. q=12 overflowed (FB-OVF).
// ============================================================
#define LIVE_WIDTH      640
#define LIVE_HEIGHT     480
#define LIVE_JPEG_Q     28
#define LIVE_FPS        30
#define REC_WIDTH       1280
#define REC_HEIGHT      720
#define REC_JPEG_Q      28
#define REC_FPS         8
#define STREAM_WIDTH    LIVE_WIDTH
#define STREAM_HEIGHT   LIVE_HEIGHT
#define JPEG_QUALITY    LIVE_JPEG_Q
#define STREAM_FPS      LIVE_FPS

// ============================================================
// INMP441
// ============================================================
#define MIC_I2S_BCLK    41
#define MIC_I2S_WS      42
#define MIC_I2S_DATA    47
#define MIC_SAMPLE_RATE 16000

// ============================================================
// AUDIO — capture identical to firmware.zip (INMP441 >>14, 3 s ring,
// 8×256 DMA, LEFT slot). Live still goes out on /ws/audio as 125 ms
// packets so one send fits the TCP buffer; two sends = one zip chunk.
// ============================================================
#define AUDIO_CHUNK_MS  20
#define AUDIO_RING_MS   3000
#define AUDIO_TX_SAMPLES ((MIC_SAMPLE_RATE * AUDIO_CHUNK_MS) / 1000)
#define AUDIO_RING_SAMPLES ((MIC_SAMPLE_RATE * AUDIO_RING_MS) / 1000)
#define MIC_SHIFT       14

// ============================================================
// BUTTON
// ============================================================
#define BTN_AUDIO       1   // short: audio record
#define BTN_VIDEO       3   // toggle video+audio record
#define BTN_PTT         14  // hold-to-talk → PQTALKIE (was night vision)
#define BTN_SOS         21  // short: SOS on PQTALKIE. hold at boot: open portal
#define BTN_POWER       BTN_SOS

// IR unused: GPIO 40 is speaker BCLK.
#define NIGHT_IR_PIN    -1

// ============================================================
// SPEAKER — MAX98357 via ESP_I2S (same path as radio FW 1.0.30)
//   VIN -> 5V   GND -> GND
//   DIN -> GPIO 38   BCLK -> GPIO 40   LRC -> GPIO 39
//   GAIN -> float
//   SD  -> 3V3 (hardwired RIGHT; amp listens to right I2S slot)
// Software: ESP_I2S STEREO @ 16 kHz, write L=R, 2 boot beeps, gain 1x
// ============================================================
#define SPK_I2S_DOUT    38
#define SPK_I2S_BCLK    40
#define SPK_I2S_LRC     39
#define SPK_SD_PIN      -1   // SD already tied to 3V3 on the board

// ============================================================
// GPS — UART1 receive-only (NMEA 9600)
//   GPS VCC -> 3.3V
//   GPS GND -> GND
//   GPS TX  -> GPIO 2 (ESP RX). GPS RX is not wired.
// ============================================================
#define GPS_RX          2
#define GPS_TX          -1
#define GPS_BAUD        9600

// ============================================================
// LED
// ============================================================
#define STATUS_LED      -1  // GPIO 2 is GPS RX; status is RGB on 48
#define RGB_LED         48
#define USE_RGB_LED     1
#define POWER_HOLD_MS       3000   // boot: hold SOS to force portal
#define WIFI_RESET_HOLD_MS  8000   // hold video button for 8s to clear Wi-Fi / portal
