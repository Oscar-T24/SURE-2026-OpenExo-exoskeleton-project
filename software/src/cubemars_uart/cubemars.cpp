#include <Arduino.h>

static constexpr uint32_t BAUD_USB = 921600;
static constexpr uint32_t BAUD_CUBEMARS = 921600;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(BAUD_USB);
    Serial1.begin(BAUD_CUBEMARS);

    unsigned long start = millis();
    while (!Serial && millis() - start < 1500) {
        delay(1);
    }
}

void loop() {
    while (Serial.available() > 0) {
        int b = Serial.read();
        if (b >= 0) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            Serial1.write((uint8_t)b);
        }
    }

    while (Serial1.available() > 0) {
        int b = Serial1.read();
        if (b >= 0) {
            Serial.write((uint8_t)b);
        }
    }
}