// HardwareTest.ino — serial menu-driven hardware test for "the interface"
//
// ASSUMPTIONS (confirm these match your hardware):
//   Board      : Arduino Uno R4 Minima
//   LEDs       : WS2812B, 44 per strip — uses Adafruit NeoPixel library
//   Motor IN2  : hardwired LOW (GND) on both DRV8833 boards; only IN1 is on Arduino
//   Sensor 2   : SoftWire on A4 (SDA) / A5 (SCL) — same traces as hardware I2C on Uno R4
//   PCA9515A   : physically removed from circuit, not in use
//   Button     : A0 active-LOW (INPUT_PULLUP)
//
// SETUP BEFORE COMPILING:
//   1. Install "Adafruit NeoPixel" via Arduino Library Manager
//   2. Install "SoftWire" via Arduino Library Manager (already required by InterfaceSensor)
//   3. Copy InterfaceSensor.h and InterfaceSensor.cpp into this HardwareTest/ folder
//      (or open theinterface/ root and target this file with Arduino CLI)
//
// USAGE:
//   Flash this sketch, open Serial Monitor at 115200 baud, send single characters:
//     1  Test LED strip 1   (D7)
//     2  Test LED strip 2   (D12)
//     3  Test motor 1       (D6)
//     4  Test motor 2       (D13)
//     5  Test heart sensor 1 (SoftWire D8/D9)
//     6  Test heart sensor 2 (SoftWire A4/A5)
//     7  Test button        (A0)  — blocks until press+release
//     8  Test potentiometer (A2)
//     a  Run all tests in sequence (button test last, it blocks)
//     h  Print this menu

#include <Adafruit_NeoPixel.h>
#include "InterfaceSensor.h"

// ── Pin map ──────────────────────────────────────────────────────────────────
#define PIN_LED1    7
#define PIN_LED2    12
#define PIN_MOTOR1  6
#define PIN_MOTOR2  13
#define PIN_BUTTON  A0
#define PIN_POT     A2
#define PIN_SDA1    8
#define PIN_SCL1    9
#define PIN_SDA2    A4
#define PIN_SCL2    A5

#define LED_COUNT   44
#define LED_BRIGHT  40   // 0-255; keep low during testing to protect eyes + power

// ── Objects ──────────────────────────────────────────────────────────────────
Adafruit_NeoPixel strip1(LED_COUNT, PIN_LED1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LED_COUNT, PIN_LED2, NEO_GRB + NEO_KHZ800);

InterfaceSensor sensor1(PIN_SDA1, PIN_SCL1);
InterfaceSensor sensor2(PIN_SDA2, PIN_SCL2);

// ── Helpers ──────────────────────────────────────────────────────────────────
static void sep()         { Serial.println(F("-----------------------------")); }
static void ok()          { Serial.println(F(">>> PASS")); }
static void fail(const __FlashStringHelper *msg) {
    Serial.print(F(">>> FAIL: ")); Serial.println(msg);
}

void printMenu() {
    sep();
    Serial.println(F("Hardware Test Harness — send a key:"));
    Serial.println(F("  1  LED strip 1  (D7)"));
    Serial.println(F("  2  LED strip 2  (D12)"));
    Serial.println(F("  3  Motor 1      (D6)"));
    Serial.println(F("  4  Motor 2      (D13)"));
    Serial.println(F("  5  Sensor 1     (SoftWire D8/D9)"));
    Serial.println(F("  6  Sensor 2     (SoftWire A4/A5)"));
    Serial.println(F("  7  Button       (A0) — blocks!"));
    Serial.println(F("  8  Potentiometer (A2)"));
    Serial.println(F("  a  All tests"));
    Serial.println(F("  h  This menu"));
    sep();
}

// ── Individual tests ─────────────────────────────────────────────────────────
void testLEDs(Adafruit_NeoPixel &strip, uint8_t num) {
    Serial.print(F("LED strip ")); Serial.print(num);
    Serial.print(F(" (pin ")); Serial.print(strip.getPin()); Serial.println(F("):"));

    const uint32_t colors[] = {
        strip.Color(LED_BRIGHT, 0, 0),
        strip.Color(0, LED_BRIGHT, 0),
        strip.Color(0, 0, LED_BRIGHT),
        strip.Color(LED_BRIGHT, LED_BRIGHT, LED_BRIGHT),
    };
    const char *labels[] = { "RED", "GREEN", "BLUE", "WHITE" };

    for (uint8_t i = 0; i < 4; i++) {
        Serial.print(F("  ")); Serial.println(labels[i]);
        strip.fill(colors[i]);
        strip.show();
        delay(700);
    }
    strip.clear();
    strip.show();
    Serial.println(F("  (off)"));
    Serial.println(F("  Visual confirm required — all 44 LEDs should have lit each color."));
    ok();
}

void testMotor(uint8_t pin, uint8_t num) {
    Serial.print(F("Motor ")); Serial.print(num);
    Serial.print(F(" (pin ")); Serial.print(pin); Serial.println(F("):"));
    Serial.println(F("  25% power 0.5s..."));
    analogWrite(pin, 64);
    delay(500);
    Serial.println(F("  50% power 0.5s..."));
    analogWrite(pin, 128);
    delay(500);
    Serial.println(F("  100% power 0.5s..."));
    analogWrite(pin, 255);
    delay(500);
    analogWrite(pin, 0);
    Serial.println(F("  off."));
    Serial.println(F("  You should have felt three vibration steps."));
    ok();
}

void testSensor(InterfaceSensor &sensor, uint8_t num) {
    Serial.print(F("Sensor ")); Serial.print(num); Serial.println(F(":"));

    Serial.print(F("  begin()... "));
    if (!sensor.begin()) {
        fail(F("Part ID mismatch or no I2C ACK. Check wiring + power."));
        sensor.dumpConfig();
        return;
    }
    Serial.println(F("OK"));

    Serial.print(F("  setupSensor()... "));
    if (!sensor.setupSensor()) {
        fail(F("Reset timed out. Check SDA/SCL not shorted."));
        return;
    }
    Serial.println(F("OK"));
    sensor.dumpConfig();

    Serial.println(F("  Reading 10 IR samples (up to 3s)..."));
    delay(200);

    int got = 0;
    unsigned long deadline = millis() + 3000;
    while (got < 10 && millis() < deadline) {
        byte count = sensor.getFIFOCount();
        for (byte i = 0; i < count && got < 10; i++) {
            long ir = sensor.getIR();
            if (ir >= 0) {
                Serial.print(F("    IR[")); Serial.print(got);
                Serial.print(F("] = ")); Serial.println(ir);
                got++;
            }
        }
        delay(20);
    }

    if (got == 0) {
        fail(F("No samples received within 3s. Place finger on sensor."));
        return;
    }

    Serial.print(F("  Got ")); Serial.print(got); Serial.println(F(" samples."));
    Serial.println(F("  Tip: values >50000 with finger on sensor = good signal."));
    ok();
}

void testButton() {
    Serial.println(F("Button (A0):"));
    Serial.println(F("  Waiting for press... (send 'h' to abort — NOT IMPLEMENTED, just press it)"));
    while (digitalRead(PIN_BUTTON) == HIGH)
        ;
    Serial.println(F("  Pressed!"));
    while (digitalRead(PIN_BUTTON) == LOW)
        ;
    Serial.println(F("  Released."));
    ok();
}

void testPot() {
    Serial.println(F("Potentiometer (A2):"));
    Serial.println(F("  10 readings at 200ms intervals — turn the pot while reading:"));
    for (uint8_t i = 0; i < 10; i++) {
        int v = analogRead(PIN_POT);
        Serial.print(F("    [")); Serial.print(i);
        Serial.print(F("] raw=")); Serial.print(v);
        Serial.print(F("  (")); Serial.print(v * 100 / 1023); Serial.println(F("%)"));
        delay(200);
    }
    Serial.println(F("  Values should span 0-1023 across full pot travel."));
    ok();
}

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);

    pinMode(PIN_MOTOR1, OUTPUT);
    pinMode(PIN_MOTOR2, OUTPUT);
    analogWrite(PIN_MOTOR1, 0);
    analogWrite(PIN_MOTOR2, 0);
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    strip1.begin(); strip1.setBrightness(LED_BRIGHT); strip1.clear(); strip1.show();
    strip2.begin(); strip2.setBrightness(LED_BRIGHT); strip2.clear(); strip2.show();

    Serial.println(F("=== Interface Hardware Test ==="));
    printMenu();
}

void loop() {
    if (!Serial.available()) return;

    char cmd = (char)Serial.read();
    while (Serial.available()) Serial.read();  // flush

    Serial.println();

    switch (cmd) {
        case '1': testLEDs(strip1, 1);   break;
        case '2': testLEDs(strip2, 2);   break;
        case '3': testMotor(PIN_MOTOR1, 1); break;
        case '4': testMotor(PIN_MOTOR2, 2); break;
        case '5': testSensor(sensor1, 1); break;
        case '6': testSensor(sensor2, 2); break;
        case '7': testButton();           break;
        case '8': testPot();              break;
        case 'a':
            testLEDs(strip1, 1);
            testLEDs(strip2, 2);
            testMotor(PIN_MOTOR1, 1);
            testMotor(PIN_MOTOR2, 2);
            testSensor(sensor1, 1);
            testSensor(sensor2, 2);
            testPot();
            testButton();  // last — blocks until physical press
            break;
        case 'h': break;  // fall through to menu
        case '\r': case '\n': return;
        default:
            Serial.print(F("Unknown: '")); Serial.print(cmd);
            Serial.println(F("' — send h for menu"));
    }

    Serial.println();
    printMenu();
}
