// Arduino-ESP32 3.x. Controller firmware, NOT camera firmware.
#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include "secrets.h"   // WIFI_SSID, WIFI_PASSWORD. Gitignored; see secrets.h.example.

// ENABLED 2026-09-22 at the operator's request. The controller will now
// drive the servos. Limits below are still the conservative 80-100
// placeholder, not measured travel: find the real range with tools/jog.py
// and widen these only after seeing each joint move. Set false to disarm.
const bool CALIBRATED = true;
// Measured from the wiring on 2026-09-22, not an example: D27, D26, D25, D33.
// All four are LEDC-capable outputs on the classic ESP32 and none is a boot
// strapping pin. Joint order below is assumed to be base, shoulder, elbow,
// gripper and must be confirmed during calibration: command one joint at a
// time and record which physically moves. See docs/HARDWARE.md.
const int PINS[4] = {27, 26, 25, 33};
// Widened in stages at the operator's request: 80-100, 60-120, 30-150, now
// the servo's full 0-180. There is no software limit left: the mechanism
// is now the only thing that stops a joint, and driving into a hard stop
// stalls the servo at maximum current, which browns out the board and can
// strip an SG90's nylon gears. These stay per-joint arrays so each joint
// can be narrowed to its measured travel once known.
const int MIN_ANGLE[4] = {0, 0, 0, 0};
const int MAX_ANGLE[4] = {180, 180, 180, 180};
// Move one joint at a time. Four SG90s stall at roughly 650-750 mA each, so
// driving them together can pull well past what one supply provides; the
// rail sags and the ESP32 can brown out mid-motion and drop the arm.
// Stepping one joint at a time keeps the moving current to a single servo
// plus three holding. This makes a four-joint move about four times slower,
// so raise hold_seconds in poc.yaml to suit. Set false only with a supply
// sized for all four servos moving together.
const bool SEQUENTIAL_MOTION = true;
// Milliseconds per one-degree step: 50 gives 20 deg/s per joint. Slow on
// purpose: a servo's current scales with how fast it is asked to move, and
// gentler motion is easier on SG90 gears and on the supply rail.
const int STEP_MS = 50;
const int ATTACH_STAGGER_MS = 120; // Gap between bringing each servo up.
const int PULSE_MIN_US = 1000; // Verify your actual servo specifications.
const int PULSE_MAX_US = 2000;
// TCP port the Pi's ROS node connects to. Plain text, same line protocol
// (PING/STOP/MOVE) that used to run over USB serial at 115200. There is no
// authentication: whoever can reach this port on the Wi-Fi network can move
// the arm, same trust boundary as a wired serial cable on a private bench.
const uint16_t CONTROL_PORT = 3333;
int currentAngle[4] = {90, 90, 90, 90};
int targetAngle[4] = {90, 90, 90, 90};
bool active = false;
unsigned long lastMove = 0, lastStep = 0;
char input[96];
size_t inputLength = 0;
bool overflow = false;

WiFiServer server(CONTROL_PORT);
WiFiClient controller; // The one Pi connection accepted at a time.

void stopServos() {
  for (int pin : PINS) {
    if (active) ledcDetach(pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  active = false;
}

void writeAngle(int joint) {  // Caller must have attached PINS[joint] first.
  const long pulse = map(currentAngle[joint], 0, 180, PULSE_MIN_US, PULSE_MAX_US);
  ledcWrite(PINS[joint], (uint32_t)((pulse * 65535L) / 20000L));
}

// Bring the servos up one at a time. Attaching all four and writing an angle
// to each in the same instant makes every servo seek at once, and that inrush
// is what browns out the board on a marginal supply: the controller resets
// mid-command and the host sees a dropped connection. Staggering spreads the
// current.
bool enableServos() {
  for (int i = 0; i < 4; ++i) {
    if (!ledcAttach(PINS[i], 50, 16)) {
      for (int j = 0; j < i; ++j) ledcDetach(PINS[j]);
      stopServos();
      return false;
    }
    writeAngle(i);
    delay(ATTACH_STAGGER_MS);
  }
  active = true;
  return true;
}

void reply(const char *line) {
  if (controller && controller.connected()) controller.println(line);
  Serial.println(line); // Kept for a USB debug console; not the command path.
}

void command(char *line) {
  if (strcmp(line, "PING") == 0) { reply("READY"); return; }
  if (strcmp(line, "STOP") == 0) { stopServos(); reply("STOPPED"); return; }
  double value[4];
  char extra;
  if (sscanf(line, "MOVE %lf %lf %lf %lf %c", &value[0], &value[1], &value[2], &value[3], &extra) != 4) {
    reply("ERR format"); return;
  }
  if (!CALIBRATED) { reply("ERR calibration required"); return; }
  for (int i = 0; i < 4; ++i) {
    if (!isfinite(value[i]) || value[i] != floor(value[i]) ||
        value[i] < MIN_ANGLE[i] || value[i] > MAX_ANGLE[i]) {
      reply("ERR limits"); return;
    }
  }
  if (!active && !enableServos()) { reply("ERR PWM"); return; }
  for (int i = 0; i < 4; ++i) targetAngle[i] = (int)value[i];
  lastMove = millis();
  reply("OK");
}

void setup() {
  Serial.begin(115200);
  stopServos(); // No motion at boot.

  WiFi.mode(WIFI_STA);
  if (strlen(HOST_IP) > 0) {
    IPAddress ip, gw, mask;
    if (ip.fromString(HOST_IP) && gw.fromString(GATEWAY_IP) && mask.fromString(SUBNET_MASK))
      WiFi.config(ip, gw, mask);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  Serial.print("Controller ready at ");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(CONTROL_PORT);
  server.begin();
}

void loop() {
  // Accept a new Pi connection only when nobody is currently holding one, so
  // a second connect attempt cannot steal command input mid-move and leave
  // the first client's MOVE unanswered.
  if (!controller || !controller.connected()) {
    WiFiClient incoming = server.available();
    if (incoming) {
      if (controller) controller.stop();
      controller = incoming;
      controller.setNoDelay(true);
      inputLength = 0;
      overflow = false;
    }
  }

  // Bound processing so a noisy sender cannot starve the watchdog.
  for (int budget = 0; budget < 128 && controller && controller.available(); ++budget) {
    char c = controller.read();
    if (c == '\n') {
      if (overflow) reply("ERR line too long");
      else { input[inputLength] = '\0'; command(input); }
      inputLength = 0; overflow = false;
    } else if (c != '\r') {
      if (inputLength < sizeof(input) - 1 && !overflow) input[inputLength++] = c;
      else overflow = true;
    }
  }
  unsigned long now = millis();
  // Release torque once idle, but never mid-move: at STEP_MS a long travel
  // outlasts the 2 s window, and cutting PWM there would drop the arm. This
  // also covers a dropped Wi-Fi connection: no more MOVE commands arrive, so
  // the arm goes limp two seconds after the last accepted one, same as a
  // pulled USB cable used to.
  bool arrived = true;
  for (int i = 0; i < 4; ++i) if (currentAngle[i] != targetAngle[i]) arrived = false;
  if (active && arrived && now - lastMove > 2000) stopServos();
  if (active && now - lastStep >= STEP_MS) {
    lastStep = now;
    for (int i = 0; i < 4; ++i) {
      if (currentAngle[i] == targetAngle[i]) continue;
      currentAngle[i] += (currentAngle[i] < targetAngle[i]) ? 1 : -1;
      writeAngle(i);
      if (SEQUENTIAL_MOTION) break;   // Only one servo draws moving current.
    }
  }
}
