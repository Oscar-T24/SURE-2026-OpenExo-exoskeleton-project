//
// Created by Oscar Tesniere on 06/06/2026.
//
//
// Simple CAN sender on Teensy 4.1 CAN1
// Sends one standard 11-bit CAN frame every second at 250 kbit/s
//
// for reference : https://github.com/tonton81/FlexCAN_T4
#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

void setup() {
 Serial.begin(115200);
 delay(1000);

 Serial.println("Starting CAN sender...");

 Can1.begin();
 Can1.setBaudRate(250000);

 Can1.setMaxMB(16);
 Can1.enableFIFO();

 Serial.println("CAN sender ready");
}

void loop() {
 CAN_message_t msg;

 msg.id = 0x123;             // standard 11-bit ID
 msg.flags.extended = 0;     // 0 = standard frame, 1 = extended frame
 msg.len = 2;
 msg.buf[0] = 0xAB;
 msg.buf[1] = 0xCD;

 int result = Can1.write(msg);

 Serial.print("Sent CAN frame, result = ");
 Serial.println(result);

 delay(1000);
}