#pragma once

#include <Arduino.h>

struct PortalConfig {
  String ssid;
  String pass;
  String deviceId;
  String pttToken;
};

bool portalLoad(PortalConfig &cfg);
void portalSave(const PortalConfig &cfg);
void portalClear();
bool portalConnectSta(const PortalConfig &cfg, uint32_t timeoutMs = 20000);

// Blocking captive portal. Saves NVS and restarts; STA join happens after reboot.
void portalRun(PortalConfig &cfg, const String &flash = "");
