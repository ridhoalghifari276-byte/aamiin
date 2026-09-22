#pragma once

#include <Arduino.h>

void speakerBegin();
void speakerMute(bool on);
void speakerPush(const uint8_t *pcm, size_t bytes);
