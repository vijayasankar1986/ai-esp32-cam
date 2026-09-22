// Station-mode camera firmware for the Hiwonder ESP32-S3 CAM (GC2145).
//
// Replaces the factory access-point firmware so the camera joins the same
// Wi-Fi as the Raspberry Pi, removing the need for a second network or a
// second Wi-Fi adapter. Endpoints match what the factory firmware served, so
// the ROS node needs no change:
//
//   http://<ip>:81/stream   MJPEG, multipart/x-mixed-replace
//   http://<ip>/capture     single JPEG
//   http://<ip>/status      JSON health
//
// Two things about this board were found by measurement, not documentation:
//
//   Pins. Hiwonder publishes no pinout. Candidate 0 below is confirmed
//   working: probing it reached the sensor over SCCB and the driver replied
//   about pixel format rather than timing out, which only happens once the
//   sensor is talking.
//
//   Pixel format. The GC2145 has no hardware JPEG encoder, unlike the OV2640
//   most examples assume, so requesting PIXFORMAT_JPEG fails with "JPEG
//   format is not supported on this sensor". The camera is opened in RGB565
//   and frames are encoded to JPEG in software instead.
//
// Restore the factory firmware from the full flash backup taken before this
// was installed. See docs/CAMERA_FIRMWARE.md.

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include "secrets.h"

// -1 probes every candidate so one build runs on any supported board. Set an
// index to skip probing once the board is known: candidate 0 is the Hiwonder
// S3, candidate 1 the AI-Thinker.
#define PIN_FORCE  -1
#define MDNS_NAME  "armcam" // Reachable as armcam.local where mDNS is supported.
#define JPEG_QUALITY 80     // Software encoder, 0-100. Higher costs CPU and bandwidth.
// Frame size. The GC2145 has no hardware JPEG, so every frame is encoded in
// software on the ESP32 and the cost scales with pixel count. Measured on this
// board over /capture:
//
//   FRAMESIZE_QVGA  320x240   7.0 fps    8 KB/frame   usable video
//   FRAMESIZE_VGA   640x480   0.9 fps   23 KB/frame   effectively a slideshow
//
// QVGA is the only setting that streams smoothly. CIF (400x296) is untested
// middle ground. Colour detection measures the fraction of matching pixels, so
// it gains nothing from resolution: raise this only if you want a bigger
// picture and can live with the frame rate.
#define FRAME_SIZE FRAMESIZE_QVGA

struct PinMap {
  const char *name;
  int8_t pwdn, reset, xclk, sda, scl;
  int8_t d7, d6, d5, d4, d3, d2, d1, d0;
  int8_t vsync, href, pclk;
};

// Index 0 is confirmed on the Hiwonder board; it is also the GOOUUU
// ESP32-S3-CAM, Freenove ESP32-S3-WROOM CAM and ESP32-S3-EYE layout.
//
// AI-Thinker is a classic ESP32, not an S3, and needs the esp32:esp32:esp32cam
// build target rather than esp32s3. Its pin map is taken from Espressif's own
// camera_pins.h in the installed core. It carries an OV2640, which has a
// hardware JPEG encoder, so tryBothFormats succeeds on its first attempt and
// the board avoids the software encoding that limits the GC2145 to 7 fps.
// Selected by chip, not probed across chips. On a classic ESP32 the S3 maps
// use GPIO 6-11, which are wired to the SPI flash: driving them resets the
// board instantly, and probing them produced a boot loop
// (rst:0x8 TG1WDT_SYS_RESET) on an AI-Thinker before this guard existed.
static const PinMap CANDIDATES[] = {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  {"S3-CAM/Freenove/S3-EYE", -1, -1, 15,  4,  5, 16, 17, 18, 12, 10,  8,  9, 11,  6,  7, 13},
  {"XIAO ESP32S3 Sense",     -1, -1, 10, 40, 39, 48, 11, 12, 14, 16, 18, 17, 15, 38, 47, 13},
  {"ESP32-S3 alt (40/39)",   -1, -1, 40, 17, 18, 39, 41, 42, 12,  3, 14, 47, 13, 21, 38, 11},
#elif defined(CONFIG_IDF_TARGET_ESP32)
  {"AI-Thinker ESP32-CAM",   32, -1,  0, 26, 27, 35, 34, 39, 36, 21, 19, 18,  5, 25, 23, 22},
#else
#error "Unsupported chip: add this board's camera pin map"
#endif
};
static const size_t CANDIDATE_COUNT = sizeof(CANDIDATES) / sizeof(CANDIDATES[0]);

static const char *STREAM_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char *STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t control_server = NULL;
static httpd_handle_t stream_server = NULL;
static const PinMap *active_map = NULL;
static bool native_jpeg = false;        // true if the sensor encodes JPEG itself
static volatile uint32_t frames_served = 0;

static bool tryPinMap(const PinMap &m, pixformat_t fmt) {
  camera_config_t c = {};
  c.pin_pwdn = m.pwdn; c.pin_reset = m.reset; c.pin_xclk = m.xclk;
  c.pin_sccb_sda = m.sda; c.pin_sccb_scl = m.scl;
  c.pin_d7 = m.d7; c.pin_d6 = m.d6; c.pin_d5 = m.d5; c.pin_d4 = m.d4;
  c.pin_d3 = m.d3; c.pin_d2 = m.d2; c.pin_d1 = m.d1; c.pin_d0 = m.d0;
  c.pin_vsync = m.vsync; c.pin_href = m.href; c.pin_pclk = m.pclk;
  c.xclk_freq_hz = 20000000;            // GC2145 tops out at 20 MHz.
  c.ledc_timer = LEDC_TIMER_0; c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = fmt;
  c.frame_size = FRAME_SIZE;
  c.jpeg_quality = 12;                  // Only used when the sensor does JPEG.
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&c) != ESP_OK) return false;
  // Init can succeed on a wrong map yet never produce a frame, so demand one.
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb || fb->len == 0) {
    if (fb) esp_camera_fb_return(fb);
    esp_camera_deinit();
    return false;
  }
  esp_camera_fb_return(fb);
  native_jpeg = (fmt == PIXFORMAT_JPEG);
  return true;
}

// Try the sensor's own JPEG first, since it is far cheaper, then fall back to
// RGB565 with software encoding for sensors like the GC2145 that lack it.
static bool tryBothFormats(const PinMap &m) {
  if (tryPinMap(m, PIXFORMAT_JPEG)) return true;
  return tryPinMap(m, PIXFORMAT_RGB565);
}

static void reportSensor() {
  sensor_t *s = esp_camera_sensor_get();
  if (s) Serial.printf("Sensor PID 0x%04x (GC2145 is 0x2145)\n", s->id.PID);
  Serial.printf("JPEG source: %s\n", native_jpeg ? "sensor hardware" : "software encoder");
}

static bool startCamera() {
  if (PIN_FORCE >= 0 && PIN_FORCE < (int)CANDIDATE_COUNT) {
    Serial.printf("Using pin map %d: %s ... ", PIN_FORCE, CANDIDATES[PIN_FORCE].name);
    if (tryBothFormats(CANDIDATES[PIN_FORCE])) {
      Serial.println("OK");
      active_map = &CANDIDATES[PIN_FORCE];
      reportSensor();
      return true;
    }
    Serial.println("failed; set PIN_FORCE to -1 to probe every candidate");
    return false;
  }
  for (size_t i = 0; i < CANDIDATE_COUNT; ++i) {
    Serial.printf("Probing pin map %u: %s ... ", (unsigned)i, CANDIDATES[i].name);
    if (tryBothFormats(CANDIDATES[i])) {
      Serial.println("OK");
      active_map = &CANDIDATES[i];
      reportSensor();
      Serial.printf("Working map index %u -- set PIN_FORCE to skip probing\n", (unsigned)i);
      return true;
    }
    Serial.println("no");
    delay(120);
  }
  return false;
}

// Hands back a JPEG for the current frame. When the sensor cannot encode, the
// RGB565 buffer is converted here and *owned* is set so the caller frees it.
static bool grabJpeg(camera_fb_t **fb_out, uint8_t **buf, size_t *len, bool *owned) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;
  *fb_out = fb;
  if (native_jpeg) {
    *buf = fb->buf; *len = fb->len; *owned = false;
    return true;
  }
  if (!frame2jpg(fb, JPEG_QUALITY, buf, len)) {
    esp_camera_fb_return(fb);
    return false;
  }
  *owned = true;
  return true;
}

static esp_err_t captureHandler(httpd_req_t *req) {
  camera_fb_t *fb = NULL; uint8_t *buf = NULL; size_t len = 0; bool owned = false;
  if (!grabJpeg(&fb, &buf, &len, &owned)) return httpd_resp_send_500(req);
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t r = httpd_resp_send(req, (const char *)buf, len);
  if (owned) free(buf);
  esp_camera_fb_return(fb);
  return r;
}

static esp_err_t statusHandler(httpd_req_t *req) {
  char buf[384];
  int w = 0, h = 0;
  camera_fb_t *probe = active_map ? esp_camera_fb_get() : NULL;
  if (probe) { w = probe->width; h = probe->height; esp_camera_fb_return(probe); }
  int n = snprintf(buf, sizeof(buf),
                   "{\"pin_map\":\"%s\",\"camera\":%s,\"native_jpeg\":%s,"
                   "\"width\":%d,\"height\":%d,\"ip\":\"%s\","
                   "\"rssi\":%d,\"frames_served\":%u,\"psram\":%s,\"heap\":%u}",
                   active_map ? active_map->name : "none",
                   active_map ? "true" : "false", native_jpeg ? "true" : "false",
                   w, h, WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(),
                   (unsigned)frames_served, psramFound() ? "true" : "false",
                   (unsigned)ESP.getFreeHeap());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, buf, n);
}

static esp_err_t streamHandler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char part[80];
  while (true) {
    camera_fb_t *fb = NULL; uint8_t *buf = NULL; size_t len = 0; bool owned = false;
    if (!grabJpeg(&fb, &buf, &len, &owned)) { res = ESP_FAIL; break; }
    size_t hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)buf, len);
    if (owned) free(buf);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;   // Client disconnected.
    ++frames_served;
  }
  return res;
}

// cameraReady false still serves /status, so a board whose sensor did not come
// up is reachable over the network for diagnosis instead of silently dead.
static void startServers(bool cameraReady) {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.ctrl_port = 32768;
  httpd_uri_t capture = {"/capture", HTTP_GET, captureHandler, NULL};
  httpd_uri_t status = {"/status", HTTP_GET, statusHandler, NULL};
  if (httpd_start(&control_server, &cfg) == ESP_OK) {
    httpd_register_uri_handler(control_server, &status);
    if (cameraReady) httpd_register_uri_handler(control_server, &capture);
  }
  if (!cameraReady) return;
  // Separate instance on 81 so a long-lived stream cannot block /capture.
  cfg.server_port = 81;
  cfg.ctrl_port = 32769;
  httpd_uri_t stream = {"/stream", HTTP_GET, streamHandler, NULL};
  if (httpd_start(&stream_server, &cfg) == ESP_OK) {
    httpd_register_uri_handler(stream_server, &stream);
  }
}

static bool connectWifi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // Sleep adds latency and stalls MJPEG.
  if (strlen(HOST_IP) > 0) {
    IPAddress ip, gw, mask;
    if (ip.fromString(HOST_IP) && gw.fromString(GATEWAY_IP) && mask.fromString(SUBNET_MASK))
      WiFi.config(ip, gw, mask);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Joining %s", WIFI_SSID);
  uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 30000) {
    delay(400);
    Serial.print('.');
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("\nWi-Fi FAILED (status %d). Check SSID and password in secrets.h.\n",
                  (int)WiFi.status());
    return false;
  }
  Serial.printf("\nIP address: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Gateway:    %s   RSSI %d\n",
                WiFi.gatewayIP().toString().c_str(), (int)WiFi.RSSI());
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nArm camera: station mode");
  Serial.printf("PSRAM: %s\n", psramFound() ? "yes" : "NO (enable OPI PSRAM in Tools)");

  // Wi-Fi first, so the address is reported even when the sensor does not come
  // up. A camera fault should not also cost us network access to the board.
  bool online = connectWifi();

  bool cameraReady = startCamera();
  if (!cameraReady) {
    Serial.println("\nCamera not detected: no candidate pin map produced a frame.");
    Serial.println("Run firmware/camera_pin_finder to measure this board's pins.");
    Serial.println("Wi-Fi and /status stay up so the board is still reachable.");
  }

  if (online) {
    String ip = WiFi.localIP().toString();
    if (cameraReady) {
      Serial.printf("Stream  http://%s:81/stream\n", ip.c_str());
      Serial.printf("Still   http://%s/capture\n", ip.c_str());
    }
    Serial.printf("Status  http://%s/status\n", ip.c_str());
    if (MDNS.begin(MDNS_NAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("Also    http://%s.local/status\n", MDNS_NAME);
    }
    startServers(cameraReady);
  }
}

void loop() {
  // Reboot on a link that was up and dropped, so the camera reappears without
  // intervention. A link that never came up means bad credentials, and
  // restarting on those would just spin, so report once and sit still.
  static bool everConnected = false;
  static bool reported = false;
  if (WiFi.status() == WL_CONNECTED) {
    everConnected = true;
  } else if (everConnected) {
    Serial.println("Wi-Fi lost; restarting");
    delay(1000);
    ESP.restart();
  } else if (!reported) {
    reported = true;
    Serial.println("Never connected. Fix secrets.h and re-upload.");
  }
  delay(2000);
}
