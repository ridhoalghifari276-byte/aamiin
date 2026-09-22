#pragma once

// ============================================================
// WIFI / SERVER
//
// First boot opens a captive portal (AP "BODYCAM-SETUP").
// Device ID + Wi-Fi are entered there and stored in NVS.
// These two strings are compile-time fallbacks only; leave them
// empty so the portal always runs until the unit is provisioned.
// Hold record-audio (GPIO 1) for 7 seconds during operation to
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
// ESP ingest via 4301 (allowlist QMeet). Dashboard tetap :7890. Bukan 7891.
#define SERVER_PORT     4301
#define SERVER_WS_PATH  "/ws/device"

#define DEVICE_ID       "bodycam-01"
#define SERVER_TOKEN    "X01040688"

// ============================================================
// CAMERA - ESP32-S3 CAM N16R8 / OV2640
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
// VIDEO — MiFi/VTA uplink is narrow; keep JPEG small so WS stays up.
// ============================================================
#define STREAM_WIDTH    640
#define STREAM_HEIGHT   480
#define JPEG_QUALITY    38
#define STREAM_FPS      8

// ============================================================
// INMP441
//   GND -> GND   VDD -> 3V3
//   SD  -> GPIO 47   SCK -> GPIO 41   WS -> GPIO 42   L/R -> GND
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
#define AUDIO_CHUNK_MS  125
#define AUDIO_RING_MS   3000
#define AUDIO_TX_SAMPLES ((MIC_SAMPLE_RATE * AUDIO_CHUNK_MS) / 1000)
#define AUDIO_RING_SAMPLES ((MIC_SAMPLE_RATE * AUDIO_RING_MS) / 1000)
#define MIC_SHIFT       14

// ============================================================
// BUTTON (to GND, INPUT_PULLUP)
//   SW1 GPIO 1  = audio record / hold 7s Wi-Fi reset
//   SW2 GPIO 3  = video+audio record
//   SW3 GPIO 14 = PTT
//   SW4 GPIO 21 = SOS / hold at boot = portal
// ============================================================
#define BTN_AUDIO       1
#define BTN_VIDEO       3
#define BTN_PTT         14
#define BTN_SOS         21
#define BTN_POWER       BTN_SOS

// IR unused: GPIO 40 is speaker BCLK.
#define NIGHT_IR_PIN    -1

// ============================================================
// SPEAKER — MAX98357 (I2S_NUM_1)
//   VIN -> 5V   GND -> GND
//   DIN -> GPIO 38   BCLK -> GPIO 40   LRC -> GPIO 39
//   GAIN -> float (tidak perlu)   SD -> 3V3  (= RIGHT channel only)
// ============================================================
#define SPK_I2S_DOUT    38
#define SPK_I2S_BCLK    40
#define SPK_I2S_LRC     39

// ============================================================
// GPS — UART1 receive-only (NMEA 9600)
//   VCC -> 3.3V   GND -> GND
//   TX  -> GPIO 2 (ESP RX). GPS RX tidak disambung.
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
#define WIFI_RESET_HOLD_MS  7000   // hold record-audio to clear Wi-Fi / portal
