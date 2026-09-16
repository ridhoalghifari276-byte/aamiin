#pragma once

// ============================================================
// WIFI / SERVER
//
// First boot opens a captive portal (AP "BODYCAM-SETUP").
// Device ID + Wi-Fi are entered there and stored in NVS.
// These two strings are compile-time fallbacks only; leave them
// empty so the portal always runs until the unit is provisioned.
// Hold GPIO 21 for 3 seconds during operation to reset and
// re-open the portal.
// ============================================================
#define WIFI_SSID       ""
#define WIFI_PASSWORD   ""

#define AP_SSID         "BODYCAM-SETUP"
#define AP_PASSWORD     ""
#define AP_CHANNEL      1

// HTTP API (session/state)
// Starting guess only. The laptop gets its IP from DHCP, so the firmware
// probes /health and scans the local /24 when this address stops answering.
#define SERVER_HOST     "172.16.23.135"
#define SERVER_PORT     2222
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
// VIDEO - VGA capture (full FOV, 4:3)
// JPEG_QUALITY 26 keeps frames small enough for 20 FPS on this radio.
// The zip used quality 12 @ 12 FPS (larger JPEGs, slower live).
// ============================================================
#define STREAM_WIDTH    640
#define STREAM_HEIGHT   480
#define JPEG_QUALITY    26
#define STREAM_FPS      20

// ============================================================
// INMP441
// ============================================================
#define MIC_I2S_BCLK    41
#define MIC_I2S_WS      42
#define MIC_I2S_DATA    47
#define MIC_SAMPLE_RATE 16000

// ============================================================
// AUDIO — same mic path as firmware.zip (>>14, 3 s ring, 8×256 DMA).
//
// Zip sent 250 ms over HTTP. That payload is 8000 bytes and does not fit
// CONFIG_LWIP_TCP_SND_BUF_DEFAULT (5744), so a WebSocket send of the same
// size blocked lwIP and dropped live video to ~5 FPS. 125 ms is 4000 bytes
// (fits one TCP buffer) at 8 sends/s (under the device's ~10–25 send cap).
// The samples are identical; only the frame size on the wire changed.
// ============================================================
#define AUDIO_CHUNK_MS  125
#define AUDIO_RING_MS   3000
#define AUDIO_LIVE_MS   375
#define AUDIO_TX_SAMPLES ((MIC_SAMPLE_RATE * AUDIO_CHUNK_MS) / 1000)
#define AUDIO_RING_SAMPLES ((MIC_SAMPLE_RATE * AUDIO_RING_MS) / 1000)
#define AUDIO_LIVE_SAMPLES ((MIC_SAMPLE_RATE * AUDIO_LIVE_MS) / 1000)
#define MIC_SHIFT       14

// ============================================================
// BUTTON
// ============================================================
#define BTN_AUDIO       1   // toggle audio record (live picture off)
#define BTN_VIDEO       3   // toggle video+audio record
#define BTN_STREAM      14  // toggle night vision
#define BTN_POWER       21  // short: livestream on/off. hold 3s: reset Wi-Fi / portal

// Optional IR / white illuminator (safe if unconnected).
#define NIGHT_IR_PIN    40

// ============================================================
// LED
// ============================================================
#define STATUS_LED      2
#define RGB_LED         48
#define USE_RGB_LED     1
#define POWER_HOLD_MS   3000
