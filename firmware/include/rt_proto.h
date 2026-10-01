#pragma once
#include <stddef.h>
#include <stdint.h>

// BCRT header, 48 bytes, then payload. Little-endian.
//  0 magic "BCRT"
//  4 version (1)
//  5 type: 1 video, 2 audio, 3 heartbeat, 4 control (server → device)
//  6 flags: bit0 = last piece of this frame
//  7 chunk count
//  8 sequence u16
// 10 timestamp ms u32 (device millis)
// 14 frame id u16
// 16 payload length u16
// 18 chunk index
// 19 reserved
// 20 device id, 16 bytes, zero padded
// 36 token, 12 bytes, zero padded
// 48 payload

void rtBegin(const char *host, const char *device, const char *token);
bool rtSendJpeg(const uint8_t *jpeg, size_t len, uint32_t tsMs);
bool rtSendPcm(const int16_t *pcm, size_t samples, uint32_t tsMs);
void rtHeartbeat(int rssi, uint32_t heapBytes);
void rtPoll();
void rtApplyCam();
void rtAdaptCam();
int rtLevel();
