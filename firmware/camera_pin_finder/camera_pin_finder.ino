// Finds this board's camera SCCB (I2C) and XCLK pins by measurement.
//
// Hiwonder publishes no pinout for the ESP32-S3 CAM, and camera_station's
// candidate list of known layouts did not match: every probe returned
// "i2c.master: probe device timeout", meaning nothing answered on the guessed
// SDA/SCL. This sketch discovers them instead.
//
// Method:
//   1. Camera modules fit external pull-ups on SDA and SCL. Drive each safe
//      GPIO with the internal pull-DOWN and see which still read HIGH. Only a
//      stronger external pull-up does that, so this shortlists the bus pins.
//   2. A GC2145 needs its master clock before it will answer on SCCB, so for
//      every plausible XCLK pin, generate 20 MHz and scan each ordered
//      SDA/SCL pair from the shortlist.
//
// Report the output and the working triple goes into camera_station.ino.
// This sketch only reads pins and drives a clock; it moves nothing.

#include <Arduino.h>
#include <Wire.h>

// ESP32-S3 pins that are safe to touch on this board.
// Excluded: 19/20 native USB, 26-37 SPI flash and octal PSRAM (driving those
// crashes the chip), 43/44 UART0 to the CH340 that carries this output.
static const uint8_t SAFE_PINS[] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
  21, 38, 39, 40, 41, 42, 45, 46, 47, 48
};
static const size_t SAFE_COUNT = sizeof(SAFE_PINS) / sizeof(SAFE_PINS[0]);

// GC2145 answers at 0x3C (0x78/0x79 as 8-bit). Others are common camera IDs.
static const uint8_t KNOWN_SENSORS[] = {0x3C, 0x30, 0x21, 0x2A, 0x10};

static uint8_t pulled[16];
static size_t pulledCount = 0;

static const char *sensorName(uint8_t addr) {
  switch (addr) {
    case 0x3C: return "GC2145 / OV2640";
    case 0x30: return "OV5640 / OV3660";
    case 0x21: return "OV7670 / OV7725";
    case 0x2A: return "GC0308";
    case 0x10: return "GC032A";
    default:   return "unknown";
  }
}

static void findPullups() {
  Serial.println("\n--- Phase 1: external pull-up detection ---");
  Serial.println("A pin reading HIGH against the internal pull-down has an external pull-up.");
  for (size_t i = 0; i < SAFE_COUNT; ++i) {
    uint8_t p = SAFE_PINS[i];
    pinMode(p, INPUT_PULLDOWN);
    delayMicroseconds(200);
    int withPulldown = digitalRead(p);
    pinMode(p, INPUT);
    delayMicroseconds(200);
    int floating = digitalRead(p);
    if (withPulldown == HIGH) {
      Serial.printf("  GPIO%-2u HIGH against pull-down  <-- candidate bus pin\n", p);
      if (pulledCount < sizeof(pulled)) pulled[pulledCount++] = p;
    } else if (floating == HIGH) {
      Serial.printf("  GPIO%-2u floats high (weak)\n", p);
    }
    pinMode(p, INPUT);
  }
  Serial.printf("Candidates found: %u\n", (unsigned)pulledCount);
}

static bool scanBus(uint8_t sda, uint8_t scl, uint8_t xclk, bool verbose) {
  bool hit = false;
  Wire.end();
  if (!Wire.begin(sda, scl, 100000)) return false;
  Wire.setTimeOut(20);
  for (size_t k = 0; k < sizeof(KNOWN_SENSORS); ++k) {
    uint8_t addr = KNOWN_SENSORS[k];
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("\n*** FOUND device 0x%02X (%s)\n", addr, sensorName(addr));
      if (xclk == 255) Serial.printf("    XCLK=none  SDA=GPIO%u  SCL=GPIO%u\n", sda, scl);
      else Serial.printf("    XCLK=GPIO%u  SDA=GPIO%u  SCL=GPIO%u\n", xclk, sda, scl);
      hit = true;
    }
  }
  if (verbose && !hit) Serial.print('.');
  return hit;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n\n=== Camera pin finder (ESP32-S3) ===");
  Serial.printf("PSRAM: %s\n", psramFound() ? "yes" : "no");

  findPullups();

  if (pulledCount < 2) {
    Serial.println("\nFewer than two pull-up pins found. The bus may sit on pins excluded");
    Serial.println("as unsafe, or the sensor may be held powered down. Reporting anyway.");
  }

  Serial.println("\n--- Phase 2a: scan with no clock ---");
  Serial.println("Some boards fit their own oscillator, in which case no XCLK is needed.");
  bool found = false;
  for (size_t a = 0; a < pulledCount && !found; ++a) {
    for (size_t b = 0; b < pulledCount && !found; ++b) {
      if (a == b) continue;
      if (scanBus(pulled[a], pulled[b], 255, false)) found = true;
    }
  }
  if (!found) Serial.println("  nothing without a clock, as expected for a GC2145");

  if (found) {
    Serial.println("\nSensor answered without an external clock; XCLK sweep skipped.");
  }

  Serial.println("\n--- Phase 2b: XCLK sweep against candidate bus pairs ---");
  Serial.println("The sensor needs its clock before it will answer, so each XCLK is tried.");

  for (size_t x = 0; x < SAFE_COUNT && !found; ++x) {
    uint8_t xclk = SAFE_PINS[x];
    bool isBusCandidate = false;
    for (size_t i = 0; i < pulledCount; ++i) if (pulled[i] == xclk) isBusCandidate = true;
    if (isBusCandidate) continue;              // a bus pin cannot also be the clock

    ledcDetach(xclk);
    if (!ledcAttach(xclk, 20000000, 1)) continue;   // 20 MHz, 1-bit -> 50% duty
    ledcWrite(xclk, 1);
    delay(30);                                  // let the sensor's PLL settle

    for (size_t a = 0; a < pulledCount && !found; ++a) {
      for (size_t b = 0; b < pulledCount && !found; ++b) {
        if (a == b) continue;
        if (scanBus(pulled[a], pulled[b], xclk, false)) found = true;
      }
    }
    ledcWrite(xclk, 0);
    ledcDetach(xclk);
    if (!found) Serial.printf("  XCLK GPIO%-2u: no response\n", xclk);
  }

  if (!found) {
    Serial.println("\nNo sensor answered on any combination.");
    Serial.println("Likely causes: the sensor is held in power-down or reset by a pin not");
    Serial.println("yet driven, or the bus sits on an excluded pin. Report this output.");
  } else {
    Serial.println("\nPut the triple above into CANDIDATES in camera_station.ino.");
    Serial.println("Data pins are still unknown; they are found in the next step.");
  }
}

void loop() {
  delay(5000);
}
