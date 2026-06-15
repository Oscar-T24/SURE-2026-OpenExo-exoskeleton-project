//
// Created by Oscar Tesniere on 12/06/2026.
//

#include <Arduino.h>
#include <SPI.h>

volatile uint8_t lastReceived = 0;
volatile bool receivedFlag = false;

ISR(SPI_STC_vect) {
    uint8_t incoming = SPDR;

    lastReceived = incoming;
    receivedFlag = true;

    // Prepare response for the next SPI byte.
    // Teensy sends one command byte, then one dummy byte to read this.
    SPDR = incoming + 1;
}

void setup() {
    Serial.begin(115200);

    pinMode(MISO, OUTPUT);   // Arduino sends data back on MISO
    pinMode(MOSI, INPUT);
    pinMode(SCK, INPUT);
    pinMode(SS, INPUT);

    // Enable SPI in slave mode
    SPCR |= _BV(SPE);

    // Enable SPI interrupt
    SPI.attachInterrupt();

    // Default response before first command
    SPDR = 0xAA;

    Serial.println("Arduino SPI slave started");
}

void loop() {
    if (receivedFlag) {
        noInterrupts();
        uint8_t value = lastReceived;
        receivedFlag = false;
        interrupts();

        Serial.print("Received from master: ");
        Serial.println(value);
    }
}