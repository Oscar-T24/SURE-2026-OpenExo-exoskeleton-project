//
// Created by Oscar Tesniere on 04/06/2026.
//

// inspired from the Arduino CAN article : https://docs.arduino.cc/learn/communication/can/


#include <Arduino_CAN.h>
#include <Arduino.h>

// This script will send standard frames to another MCU on the CAN bus every second

static uint32_t msg_cnt = 0;


void setup(){
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting CAN communication");
    if (!CAN.begin(CanBitRate::BR_250k))
    {
        Serial.println("CAN.begin(...) failed.");
        for (;;) {}
    }

}
void loop()
{
    /* Assemble a CAN message with the format of
     * 0xCA 0xFE 0x00 0x00 [4 byte message counter]
     */
    uint8_t const msg_data[] = {0xCA,0xFE,0,0,0,0,0,0};
    memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
    CanMsg const msg(CanStandardId(CAN_ID), sizeof(msg_data), msg_data);

    /* Transmit the CAN message, capture and display an
     * error core in case of failure.
     */
    if (int const rc = CAN.write(msg); rc < 0)
    {
        Serial.print  ("CAN.write(...) failed with error code ");
        Serial.println(rc);
        for (;;) { }
    }

    /* Increase the message counter. */
    msg_cnt++;

    /* Only send one message per second. */
    delay(1000);
}