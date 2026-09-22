// Arduino-ESP32 3.x. Controller firmware, NOT camera firmware.
#include <Arduino.h>
#include <math.h>

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
// stalls the servo at maximum current, which browns out a USB-powered
// board and can strip an SG90's nylon gears. These stay per-joint arrays
// so each joint can be narrowed to its measured travel once known.
const int MIN_ANGLE[4] = {0, 0, 0, 0};
const int MAX_ANGLE[4] = {180, 180, 180, 180};
// Move one joint at a time. Four SG90s stall at roughly 650-750 mA each, so
// driving them together can pull well past what a USB port supplies; the rail
// sags, the ESP32 browns out mid-motion and the arm drops. Stepping one joint
// at a time keeps the moving current to a single servo plus three holding.
// This makes a four-joint move about four times slower, so raise hold_seconds
// in poc.yaml to suit. Set false only with a separate servo supply fitted.
const bool SEQUENTIAL_MOTION = true;
const int STEP_MS = 20;        // Milliseconds per one-degree step.
const int ATTACH_STAGGER_MS = 120; // Gap between bringing each servo up.
const int PULSE_MIN_US = 1000; // Verify your actual servo specifications.
const int PULSE_MAX_US = 2000;
int currentAngle[4] = {90, 90, 90, 90};
int targetAngle[4] = {90, 90, 90, 90};
bool active = false;
unsigned long lastMove = 0, lastStep = 0;
char input[96];
size_t inputLength = 0;
bool overflow = false;

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
// is what browns out a USB-powered board: the controller resets mid-command
// and the host sees an empty reply. Staggering spreads the current.
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

void command(char *line) {
  if (strcmp(line, "PING") == 0) { Serial.println("READY"); return; }
  if (strcmp(line, "STOP") == 0) { stopServos(); Serial.println("STOPPED"); return; }
  double value[4];
  char extra;
  if (sscanf(line, "MOVE %lf %lf %lf %lf %c", &value[0], &value[1], &value[2], &value[3], &extra) != 4) {
    Serial.println("ERR format"); return;
  }
  if (!CALIBRATED) { Serial.println("ERR calibration required"); return; }
  for (int i = 0; i < 4; ++i) {
    if (!isfinite(value[i]) || value[i] != floor(value[i]) ||
        value[i] < MIN_ANGLE[i] || value[i] > MAX_ANGLE[i]) {
      Serial.println("ERR limits"); return;
    }
  }
  if (!active && !enableServos()) { Serial.println("ERR PWM"); return; }
  for (int i = 0; i < 4; ++i) targetAngle[i] = (int)value[i];
  lastMove = millis();
  Serial.println("OK");
}

void setup() {
  Serial.begin(115200);
  stopServos(); // No motion at boot.
}

void loop() {
  // Bound processing so a noisy serial sender cannot starve the watchdog.
  for (int budget = 0; budget < 128 && Serial.available(); ++budget) {
    char c = Serial.read();
    if (c == '\n') {
      if (overflow) Serial.println("ERR line too long");
      else { input[inputLength] = '\0'; command(input); }
      inputLength = 0; overflow = false;
    } else if (c != '\r') {
      if (inputLength < sizeof(input) - 1 && !overflow) input[inputLength++] = c;
      else overflow = true;
    }
  }
  unsigned long now = millis();
  if (active && now - lastMove > 2000) stopServos();
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
