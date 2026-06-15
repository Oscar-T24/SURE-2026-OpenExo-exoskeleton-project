#include <Arduino.h>

constexpr int LED_PIN = LED_BUILTIN;

unsigned long lastBlinkMs = 0;
unsigned long lastPrintMs = 0;
bool ledState = false;
uint32_t counter = 0;

void setup() {
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait briefly for serial monitor, but don't block forever
    }

    Serial.println("Teensy 4.1 simple test started");
}

void loop() {
    unsigned long now = millis();

    if (now - lastBlinkMs >= 200) {
        lastBlinkMs = now;

        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }

    if (now - lastPrintMs >= 1000) {
        lastPrintMs = now;

        Serial.print("Counter: ");
        Serial.println(counter++);
    }
}