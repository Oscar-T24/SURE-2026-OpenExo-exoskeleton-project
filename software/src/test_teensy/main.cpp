//
// Created by Oscar Tesniere on 01/06/2026.
//
#include <Arduino.h>

// Teensy 4.1 basic debug sketch

const int LED_PIN = 13;   // Built-in LED on Teensy 4.1

void setup() {
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(115200);

    // Wait briefly for Serial Monitor, but don't hang forever
    unsigned long start = millis();
    while (!Serial && millis() - start < 3000) {
        // wait up to 3 seconds
    }

    Serial.println("Teensy 4.1 debug start");
}

void loop() {
    static unsigned long lastBlink = 0;
    static unsigned long lastPrint = 0;
    static bool ledState = false;

    unsigned long now = millis();

    // Blink LED every 500 ms
    if (now - lastBlink >= 500) {
        lastBlink = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }

    // Print debug info every 1 second
    if (now - lastPrint >= 1000) {
        lastPrint = now;

        Serial.print("Running. millis = ");
        Serial.println(now);
    }
}