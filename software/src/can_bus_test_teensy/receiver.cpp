//
// Created by Oscar Tesniere on 06/06/2026.
//
// A simple CAN receiver which receives data from an Arduino
// Assumes 250kB/s rate
// for reference : https://github.com/tonton81/FlexCAN_T4
#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

void canSniff(const CAN_message_t &msg) {
    Serial.print("MB: "); Serial.print(msg.mb); // MB  = mailbox
    Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print("  EXT: "); Serial.print(msg.flags.extended);
    Serial.print("  TS: "); Serial.print(msg.timestamp);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX);
    Serial.print("  Buffer: ");

    for (uint8_t i = 0; i < msg.len; i++) {
        if (msg.buf[i] < 0x10) Serial.print("0");
        Serial.print(msg.buf[i], HEX);
        Serial.print(" ");
    }

    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Starting CAN receiver...");

    Can1.begin();
    Can1.setBaudRate(250000);

    // Optional, but useful for general sniffing
    Can1.setMaxMB(16);
    Can1.enableFIFO();
    Can1.enableFIFOInterrupt();

    Can1.onReceive(canSniff);

    Serial.println("CAN receiver ready.");
}

void loop() {
    Can1.events();
}