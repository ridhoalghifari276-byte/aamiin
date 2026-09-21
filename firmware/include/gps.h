#pragma once

#include <Arduino.h>

void gpsBegin();
bool gpsHasFix();
int gpsSatellites();
uint32_t gpsAgeMs();

// Appends ,"gps":... fields onto an existing JSON object body.
void gpsAppendJson(String &body);
