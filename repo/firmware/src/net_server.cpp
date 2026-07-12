#include "net_server.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(HAS_WIFI) && HAS_WIFI

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// Embedded UI assets (see web_assets.S).
extern const uint8_t webasset_index_gz_start[];  extern const uint8_t webasset_index_gz_end[];
extern const uint8_t webasset_poster_start[];    extern const uint8_t webasset_poster_end[];
extern const uint8_t webasset_idle_start[];      extern const uint8_t webasset_idle_end[];
extern const uint8_t webasset_opening_start[];   extern const uint8_t webasset_opening_end[];
extern const uint8_t webasset_open_start[];      extern const uint8_t webasset_open_end[];
extern const uint8_t webasset_spin_start[];      extern const uint8_t webasset_spin_end[];
extern const uint8_t webasset_closing_start[];   extern const uint8_t webasset_closing_end[];
extern const uint8_t webasset_logo_gz_start[];   extern const uint8_t webasset_logo_gz_end[];

namespace {

WebServer http(80);
WebSocketsServer ws(81);
QueueHandle_t cmdQueue = nullptr;
QueueHandle_t replyQueue = nullptr;
volatile uint8_t g_clients = 0;

struct Reply { uint8_t client; char line[256]; };

// Serve an embedded byte range as video/mp4 WITH HTTP Range support -- iOS Safari
// requires 206/Range to play <video>. Streams straight from flash (PROGMEM-mapped).
void serveClip(const uint8_t *start, const uint8_t *end) {
  size_t total = static_cast<size_t>(end - start);
  if (http.hasHeader("Range")) {
    String r = http.header("Range");            // "bytes=A-[B]"
    long a = 0, b = static_cast<long>(total) - 1;
    int eq = r.indexOf('=');
    int dash = r.indexOf('-');
    if (eq >= 0) {
      a = atol(r.substring(eq + 1, dash < 0 ? r.length() : dash).c_str());
      if (dash >= 0 && dash + 1 < static_cast<int>(r.length())) {
        long be = atol(r.substring(dash + 1).c_str());
        if (be > 0) b = be;
      }
    }
    if (a < 0) a = 0;
    if (b >= static_cast<long>(total)) b = static_cast<long>(total) - 1;
    if (a > b) a = 0;
    size_t sliceLen = static_cast<size_t>(b - a + 1);
    http.sendHeader("Accept-Ranges", "bytes");
    http.sendHeader("Content-Range",
                    String("bytes ") + String(a) + "-" + String(b) + "/" + String((unsigned long)total));
    http.setContentLength(sliceLen);
    http.send(206, "video/mp4", "");
    http.sendContent_P(reinterpret_cast<PGM_P>(start + a), sliceLen);
  } else {
    http.sendHeader("Accept-Ranges", "bytes");
    http.setContentLength(total);
    http.send(200, "video/mp4", "");
    http.sendContent_P(reinterpret_cast<PGM_P>(start), total);
  }
}

void serveIndex() {
  size_t len = static_cast<size_t>(webasset_index_gz_end - webasset_index_gz_start);
  http.sendHeader("Content-Encoding", "gzip");
  http.sendHeader("Cache-Control", "no-store");   // always serve the fresh UI (no stale page after a reflash)
  http.setContentLength(len);
  http.send(200, "text/html", "");
  http.sendContent_P(reinterpret_cast<PGM_P>(webasset_index_gz_start), len);
}

void servePoster() {
  size_t len = static_cast<size_t>(webasset_poster_end - webasset_poster_start);
  http.setContentLength(len);
  http.send(200, "image/jpeg", "");
  http.sendContent_P(reinterpret_cast<PGM_P>(webasset_poster_start), len);
}

void serveLogo() {
  size_t len = static_cast<size_t>(webasset_logo_gz_end - webasset_logo_gz_start);
  http.sendHeader("Content-Encoding", "gzip");
  http.sendHeader("Cache-Control", "max-age=86400");
  http.setContentLength(len);
  http.send(200, "image/svg+xml", "");
  http.sendContent_P(reinterpret_cast<PGM_P>(webasset_logo_gz_start), len);
}

// Satisfy iOS/Android captive-portal probes so the client stays on the AP without nagging.
void serveSuccess() {
  http.send(200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
}

void wsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  if (type == WStype_CONNECTED || type == WStype_DISCONNECTED) {
    g_clients = ws.connectedClients();
  } else if (type == WStype_TEXT) {
    NetServer::Cmd c;
    c.client = num;
    size_t n = len < sizeof(c.line) - 1U ? len : sizeof(c.line) - 1U;
    memcpy(c.line, payload, n);
    c.line[n] = '\0';
    while (n > 0 && (c.line[n - 1] == '\n' || c.line[n - 1] == '\r')) c.line[--n] = '\0';
    if (n > 0 && cmdQueue) xQueueSend(cmdQueue, &c, 0);
  }
}

void setupHttp() {
  static const char *hdrs[] = { "Range" };
  http.collectHeaders(hdrs, 1);
  http.on("/", HTTP_GET, serveIndex);
  http.on("/index.html", HTTP_GET, serveIndex);
  http.on("/assets/poster.jpg", HTTP_GET, servePoster);
  http.on("/assets/logo.svg", HTTP_GET, serveLogo);
  http.on("/assets/clips/idle.mp4",    HTTP_GET, []() { serveClip(webasset_idle_start, webasset_idle_end); });
  http.on("/assets/clips/opening.mp4", HTTP_GET, []() { serveClip(webasset_opening_start, webasset_opening_end); });
  http.on("/assets/clips/open.mp4",    HTTP_GET, []() { serveClip(webasset_open_start, webasset_open_end); });
  http.on("/assets/clips/spin.mp4",    HTTP_GET, []() { serveClip(webasset_spin_start, webasset_spin_end); });
  http.on("/assets/clips/closing.mp4", HTTP_GET, []() { serveClip(webasset_closing_start, webasset_closing_end); });
  http.on("/hotspot-detect.html", HTTP_GET, serveSuccess);        // iOS
  http.on("/library/test/success.html", HTTP_GET, serveSuccess);  // iOS
  http.on("/generate_204", HTTP_GET, []() { http.send(204, "text/plain", ""); });  // Android
  http.on("/gen_204", HTTP_GET, []() { http.send(204, "text/plain", ""); });
  http.onNotFound([]() { http.sendHeader("Location", "/"); http.send(302, "text/plain", ""); });
}

void netTask(void *) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  // Cap TX power: the client (iPad/laptop) is right next to the unit, and lower TX current
  // eases the peak draw that can brown out the chip on a marginal USB supply.
  WiFi.setTxPower(WIFI_POWER_13dBm);
  setupHttp();
  http.begin();
  ws.begin();
  ws.onEvent(wsEvent);
  for (;;) {
    http.handleClient();
    ws.loop();
    Reply rp;
    while (replyQueue && xQueueReceive(replyQueue, &rp, 0) == pdTRUE) {
      ws.sendTXT(rp.client, rp.line);
    }
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

}  // namespace

namespace NetServer {

void begin() {
  cmdQueue = xQueueCreate(8, sizeof(Cmd));
  replyQueue = xQueueCreate(8, sizeof(Reply));
  // Pin the whole network stack to core 0; the control loop owns core 1 untouched.
  xTaskCreatePinnedToCore(netTask, "net", 10240, nullptr, 1, nullptr, 0);
}

bool popCommand(Cmd &out) {
  return cmdQueue && xQueueReceive(cmdQueue, &out, 0) == pdTRUE;
}

void pushReply(uint8_t client, const char *line) {
  if (!replyQueue) return;
  Reply rp;
  rp.client = client;
  strncpy(rp.line, line, sizeof(rp.line) - 1U);
  rp.line[sizeof(rp.line) - 1U] = '\0';
  xQueueSend(replyQueue, &rp, 0);
}

uint8_t clients() { return g_clients; }

}  // namespace NetServer

#else  // no WiFi on this target (e.g. the Due) -- no-op stubs

namespace NetServer {
void begin() {}
bool popCommand(Cmd &) { return false; }
void pushReply(uint8_t, const char *) {}
uint8_t clients() { return 0; }
}  // namespace NetServer

#endif
