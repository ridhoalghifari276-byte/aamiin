#include "portal.h"
#include "config.h"

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

static Preferences portalPrefs;
static WebServer portalHttp(80);
static DNSServer portalDns;
static PortalConfig *portalLive = nullptr;
static String portalFlash;
static bool portalSaved = false;

static const IPAddress kApIp(192, 168, 4, 1);
static const IPAddress kApGw(192, 168, 4, 1);
static const IPAddress kApMask(255, 255, 255, 0);

static String htmlEscape(const String &s) {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if (c == '&') o += F("&amp;");
    else if (c == '<') o += F("&lt;");
    else if (c == '>') o += F("&gt;");
    else if (c == '"') o += F("&quot;");
    else o += c;
  }
  return o;
}

static String normalizeDeviceId(String id) {
  id.trim();
  id.toLowerCase();
  id.replace(' ', '-');
  if (id == "1" || id == "bodycam1" || id == "bodycam-1") id = "bodycam-01";
  if (id == "2" || id == "bodycam2" || id == "bodycam-2") id = "bodycam-02";
  if (id == "3" || id == "bodycam3" || id == "bodycam-3") id = "bodycam-03";
  if (id == "bodycam-01" || id == "bodycam-02" || id == "bodycam-03") return id;
  return "";
}

bool portalLoad(PortalConfig &cfg) {
  portalPrefs.begin("bodycam", true);
  cfg.ssid = portalPrefs.getString("ssid", "");
  cfg.pass = portalPrefs.getString("pass", "");
  cfg.deviceId = portalPrefs.getString("devid", DEVICE_ID);
  cfg.pttToken = portalPrefs.getString("ptt", "");
  portalPrefs.end();

  if (!cfg.ssid.length() && strlen(WIFI_SSID) > 0) {
    cfg.ssid = WIFI_SSID;
    cfg.pass = WIFI_PASSWORD;
  }
  if (!cfg.deviceId.length()) cfg.deviceId = DEVICE_ID;
  return cfg.ssid.length() > 0;
}

void portalSave(const PortalConfig &cfg) {
  portalPrefs.begin("bodycam", false);
  portalPrefs.putString("ssid", cfg.ssid);
  portalPrefs.putString("pass", cfg.pass);
  portalPrefs.putString("devid", cfg.deviceId);
  portalPrefs.putString("ptt", cfg.pttToken);
  portalPrefs.end();
}

void portalClear() {
  portalPrefs.begin("bodycam", false);
  portalPrefs.remove("ssid");
  portalPrefs.remove("pass");
  portalPrefs.remove("devid");
  portalPrefs.remove("ptt");
  portalPrefs.end();
}

bool portalConnectSta(const PortalConfig &cfg, uint32_t timeoutMs) {
  if (!cfg.ssid.length()) return false;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  if (cfg.deviceId.length()) WiFi.setHostname(cfg.deviceId.c_str());
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  Serial.printf("[WiFi] connecting to '%s'", cfg.ssid.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeoutMs) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] IP=%s RSSI=%d device=%s\n",
                  WiFi.localIP().toString().c_str(),
                  WiFi.RSSI(),
                  cfg.deviceId.c_str());
    return true;
  }
  Serial.println("[WiFi] connection failed");
  WiFi.disconnect(true, true);
  return false;
}

static String cachedOptions;

static String scanOptions(const String &current, bool doScan) {
  if (doScan || !cachedOptions.length()) {
    String html;
    int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);
    if (n <= 0) {
      html += F("<option value=\"\">Tidak ada jaringan. Ketik SSID manual.</option>");
    } else {
      html += F("<option value=\"\">-- pilih WiFi --</option>");
      for (int i = 0; i < n; ++i) {
        String ssid = WiFi.SSID(i);
        if (!ssid.length()) continue;
        bool openNet = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
        html += F("<option value=\"");
        html += htmlEscape(ssid);
        if (ssid == current) html += F("\" selected>");
        else html += F("\">");
        html += htmlEscape(ssid);
        html += "  (";
        html += String(WiFi.RSSI(i));
        html += " dBm";
        html += openNet ? F(", terbuka") : F(", terkunci");
        html += F(")</option>");
      }
    }
    WiFi.scanDelete();
    cachedOptions = html;
  }
  return cachedOptions;
}

static String pageShell(const String &body) {
  String html;
  html.reserve(4096 + body.length());
  html += F(
    "<!DOCTYPE html><html lang=\"id\"><head>"
    "<meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>BODYCAM SETUP</title>"
    "<style>"
    ":root{--bg:#05080d;--panel:#0b1620;--line:#1a3a4a;--cyan:#2ee6d6;--text:#d7f6f2;--muted:#6f93a0;--red:#ff4d6a;--ok:#3dff9a;}"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{font-family:Segoe UI,system-ui,sans-serif;background:radial-gradient(900px 500px at 10% -10%,rgba(46,230,214,.12),transparent 55%),#05080d;color:var(--text);min-height:100vh;padding:24px 16px}"
    ".card{max-width:420px;margin:0 auto;background:rgba(11,22,32,.94);border:1px solid var(--line);padding:22px 20px 24px;box-shadow:0 0 40px rgba(46,230,214,.08)}"
    "h1{font-size:15px;letter-spacing:.22em;color:var(--cyan);text-transform:uppercase}"
    ".tag{margin:6px 0 18px;font-size:12px;letter-spacing:.14em;color:var(--muted);text-transform:uppercase}"
    "label{display:block;font-size:11px;letter-spacing:.14em;text-transform:uppercase;color:var(--muted);margin:14px 0 6px}"
    "select,input{width:100%;padding:12px 10px;background:#071018;border:1px solid var(--line);color:var(--text);font-size:16px}"
    "select:focus,input:focus{outline:none;border-color:var(--cyan)}"
    "button,.btn{display:block;width:100%;margin-top:20px;padding:13px;border:1px solid var(--cyan);background:rgba(46,230,214,.12);color:var(--cyan);font-size:13px;letter-spacing:.16em;text-transform:uppercase;text-align:center;text-decoration:none}"
    "button:active{background:rgba(46,230,214,.25)}"
    ".hint{margin-top:14px;font-size:12px;line-height:1.45;color:var(--muted)}"
    ".flash{margin-bottom:12px;padding:10px;border:1px solid var(--red);color:var(--red);font-size:13px}"
    ".ok{border-color:var(--ok);color:var(--ok)}"
    ".row{display:flex;gap:8px}"
    ".row>*{flex:1}"
    "</style></head><body><div class=\"card\">");
  html += body;
  html += F("</div></body></html>");
  return html;
}

static void sendForm(bool doScan) {
  String sel01 = portalLive->deviceId == "bodycam-01" ? " selected" : "";
  String sel02 = portalLive->deviceId == "bodycam-02" ? " selected" : "";
  String sel03 = portalLive->deviceId == "bodycam-03" ? " selected" : "";

  String body;
  body += F("<h1>Bodycam Setup</h1><div class=\"tag\">Captive portal · login pertama</div>");
  if (portalFlash.length()) {
    body += F("<div class=\"flash\">");
    body += htmlEscape(portalFlash);
    body += F("</div>");
  }
  body += F(
    "<form method=\"POST\" action=\"/save\">"
    "<label>ID Perangkat</label>"
    "<select name=\"device_id\" required>");
  body += F("<option value=\"bodycam-01\"");
  body += sel01;
  body += F(">Bodycam 1</option>");
  body += F("<option value=\"bodycam-02\"");
  body += sel02;
  body += F(">Bodycam 2</option>");
  body += F("<option value=\"bodycam-03\"");
  body += sel03;
  body += F(">Bodycam 3</option></select>");

  body += F("<label>Jaringan WiFi</label><select name=\"ssid\">");
  body += scanOptions(portalLive->ssid, doScan);
  body += F("</select>");
  body += F("<label>SSID manual (opsional)</label>");
  body += F("<input name=\"ssid_custom\" placeholder=\"isi jika tidak muncul di daftar\" value=\"\">");
  body += F("<label>Password WiFi</label>");
  body += F("<input name=\"password\" type=\"password\" placeholder=\"kosongkan jika WiFi terbuka\" value=\"");
  body += htmlEscape(portalLive->pass);
  body += F("\">");
  body += F("<label>Token radio PQTALKIE (opsional)</label>");
  body += F("<input name=\"ptt_token\" placeholder=\"tempel token dari https://45.250.101.16:3443\" value=\"");
  body += htmlEscape(portalLive->pttToken);
  body += F("\">");
  body += F("<button type=\"submit\">Simpan &amp; Hubungkan</button></form>");
  body += F("<a class=\"btn\" href=\"/\">Scan ulang WiFi</a>");
  body += F("<p class=\"hint\">Hubungkan HP/laptop ke WiFi <b>BODYCAM-SETUP</b>. "
            "Pilih Bodycam 1 / 2 / 3, lalu masukkan WiFi yang akan dipakai unit. "
            "Setelah berhasil, perangkat reboot dan masuk ke jaringan tersebut.</p>");

  portalHttp.send(200, "text/html", pageShell(body));
}

static void sendPortal() { sendForm(true); }
static void sendCaptive() { sendForm(false); }

static void handleSave() {
  String picked = portalHttp.arg("ssid");
  picked.trim();
  String custom = portalHttp.arg("ssid_custom");
  custom.trim();
  String ssid = picked.length() ? picked : custom;
  String pass = portalHttp.arg("password");
  String id = normalizeDeviceId(portalHttp.arg("device_id"));
  String ptt = portalHttp.arg("ptt_token");
  ptt.trim();
  for (size_t i = 0; i < ptt.length(); i++) {
    char c = ptt[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') || c == '_' || c == '-')) {
      ptt = "";
      break;
    }
  }

  if (!ssid.length() || !id.length()) {
    portalFlash = "ID perangkat dan nama WiFi wajib diisi.";
    sendForm(false);
    return;
  }

  portalLive->ssid = ssid;
  portalLive->pass = pass;
  portalLive->deviceId = id;
  portalLive->pttToken = ptt;

  Serial.printf("[PORTAL] trying '%s' as %s\n", ssid.c_str(), id.c_str());
  WiFi.begin(ssid.c_str(), pass.c_str());
  bool ok = false;
  for (int i = 0; i < 64 && !ok; ++i) {
    delay(250);
    ok = WiFi.status() == WL_CONNECTED;
    portalDns.processNextRequest();
  }

  if (!ok) {
    portalFlash = "Gagal terhubung ke WiFi. Cek nama jaringan dan password, lalu coba lagi.";
    Serial.println("[PORTAL] STA join failed");
    WiFi.disconnect(false, false);
    sendForm(false);
    return;
  }

  portalSave(*portalLive);
  portalSaved = true;
  Serial.printf("[PORTAL] saved device=%s ssid=%s ip=%s\n",
                id.c_str(), ssid.c_str(), WiFi.localIP().toString().c_str());

  String done = F(
    "<h1>Berhasil</h1><div class=\"tag\">Konfigurasi tersimpan</div>"
    "<div class=\"flash ok\">Terhubung. Perangkat reboot dalam 2 detik.</div>"
    "<p class=\"hint\">Kembalikan HP/laptop ke WiFi rumah/kantor, lalu buka dashboard bodycam.</p>");
  portalHttp.send(200, "text/html", pageShell(done));
  delay(1800);
  ESP.restart();
}

static void handleNotFound() {
  sendCaptive();
}

void portalRun(PortalConfig &cfg) {
  portalLive = &cfg;
  portalFlash = "";
  portalSaved = false;

  pinMode(STATUS_LED, OUTPUT);

  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(kApIp, kApGw, kApMask);
  bool apOk;
  if (strlen(AP_PASSWORD) >= 8) {
    apOk = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  } else {
    apOk = WiFi.softAP(AP_SSID, nullptr, AP_CHANNEL);
  }
  delay(150);

  Serial.printf("[PORTAL] AP '%s' %s  http://%s\n",
                AP_SSID, apOk ? "up" : "FAILED", WiFi.softAPIP().toString().c_str());
  Serial.println("[PORTAL] Connect phone/laptop to BODYCAM-SETUP, then open the login page");

  portalDns.setErrorReplyCode(DNSReplyCode::NoError);
  portalDns.start(53, "*", kApIp);

  portalHttp.on("/", HTTP_GET, sendPortal);
  portalHttp.on("/index.html", HTTP_GET, sendPortal);
  portalHttp.on("/save", HTTP_POST, handleSave);
  portalHttp.on("/generate_204", HTTP_GET, sendCaptive);
  portalHttp.on("/gen_204", HTTP_GET, sendCaptive);
  portalHttp.on("/hotspot-detect.html", HTTP_GET, sendCaptive);
  portalHttp.on("/library/test/success.html", HTTP_GET, sendCaptive);
  portalHttp.on("/ncsi.txt", HTTP_GET, sendCaptive);
  portalHttp.on("/connecttest.txt", HTTP_GET, sendCaptive);
  portalHttp.on("/canonical.html", HTTP_GET, sendCaptive);
  portalHttp.on("/success.txt", HTTP_GET, sendCaptive);
  portalHttp.on("/redirect", HTTP_GET, sendCaptive);
  portalHttp.on("/fwlink/", HTTP_GET, sendCaptive);
  portalHttp.on("/favicon.ico", HTTP_GET, []() {
    portalHttp.send(204, "text/plain", "");
  });
  portalHttp.onNotFound(handleNotFound);
  portalHttp.begin();

  uint32_t lastBlink = 0;
  bool ledOn = false;
  while (!portalSaved) {
    portalDns.processNextRequest();
    portalHttp.handleClient();
    uint32_t now = millis();
    if (now - lastBlink > 400) {
      lastBlink = now;
      ledOn = !ledOn;
      digitalWrite(STATUS_LED, ledOn ? HIGH : LOW);
    }
    delay(2);
  }
}
