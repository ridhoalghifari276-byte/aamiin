#pragma once

#include <Arduino.h>

void speakerBegin();
void speakerMute(bool on);
void speakerPush(const uint8_t *pcm, size_t bytes);
// Two-tone ambulance siren on the local speaker. Radio audio keeps playing under it.
void speakerSiren(bool on);
