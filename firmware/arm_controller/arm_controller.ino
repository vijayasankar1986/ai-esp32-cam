// Arduino-ESP32 3.x. Controller firmware, NOT camera firmware.
#include <Arduino.h>
#include <math.h>

const bool CALIBRATED = false; // See docs/HARDWARE.md before enabling.
// Measured from the wiring on 2026-09-22, not an example: D27, D26, D25, D33.
// All four are LEDC-capable outputs on the classic ESP32 and none is a boot
// strapping pin. Joint order below is assumed to be base, shoulder, elbow,
// gripper and must be confirmed during calibration: command one joint at a
// time and record which physically moves. See docs/HARDWARE.md.
const int PINS[4] = {27, 26, 25, 33};
const int MIN_ANGLE[4] = {80, 80, 80, 80};
const int MAX_ANGLE[4] = {100, 100, 100, 100};
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

void writeAngle(int joint) {
  const long pulse = map(currentAngle[joint], 0, 180, PULSE_MIN_US, PULSE_MAX_US);
  ledcWrite(PINS[joint], (uint32_t)((pulse * 65535L) / 20000L));
}

bool enableServos() {
  for (int i = 0; i < 4; ++i) {
    if (!ledcAttach(PINS[i], 50, 16)) {
      for (int j = 0; j < i; ++j) ledcDetach(PINS[j]);
      stopServos();
      return false;
    }
  }
  active = true;
  for (int i = 0; i < 4; ++i) writeAngle(i);
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
  if (active && now - lastStep >= 20) {
    lastStep = now;
    for (int i = 0; i < 4; ++i) {
      if (currentAngle[i] < targetAngle[i]) ++currentAngle[i];
      else if (currentAngle[i] > targetAngle[i]) --currentAngle[i];
      writeAngle(i);
    }
  }
}
