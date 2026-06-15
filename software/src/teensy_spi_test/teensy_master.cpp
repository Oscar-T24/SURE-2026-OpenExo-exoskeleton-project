//
// Created by Oscar Tesniere on 12/06/2026.
//

#include <Arduino.h>
#include <SPI.h>

constexpr uint8_t CS_PIN = 0;

SPISettings spiSettings(
    1000000,   // 1Mhz
    MSBFIRST,
    SPI_MODE0
);

uint8_t counter = 0;

uint8_t spi_transfer_byte(uint8_t value) {
    SPI1.beginTransaction(spiSettings);

    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(5);

    // First byte sends command.
    SPI1.transfer(value);

    delayMicroseconds(5);

    // Second byte clocks back the Arduino's prepared response.
    uint8_t response = SPI1.transfer(0x00);

    delayMicroseconds(5);
    digitalWrite(CS_PIN, HIGH);

    SPI1.endTransaction();

    return response;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    SPI1.begin();

    Serial.println("Teensy SPI master started");
}

void loop() {
    uint8_t sent = counter++;
    uint8_t received = spi_transfer_byte(sent);

    Serial.print("Sent: ");
    Serial.print(sent);

    Serial.print("  Received: ");
    Serial.println(received);

    delay(500);
}