//
// A script to read the position from ATM120 encoder over SPI
//

#include <Arduino.h>
#include <SPI.h>

const int CS = 10;

void setup() {
    Serial.begin(115200);

    SPI.begin();

    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);
}

void loop() {
    uint16_t position = readPosition();

    Serial.print("Position is: ");
    Serial.println(position);

    delay(100);
}

byte sendCommand(byte command) {
    // releases the CS after transaction
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    digitalWrite(CS, LOW);
    delayMicroseconds(1);

    byte result = SPI.transfer(command);

    digitalWrite(CS, HIGH);

    SPI.endTransaction();

    return result;
}

uint16_t readPosition() {
    byte response;

    // Send read-position command
    response = sendCommand(0x10);

    delayMicroseconds(20);

    // Keep sending NOP while encoder replies 0xA5
    while (response == 0xA5) {
        response = sendCommand(0x00);
        delayMicroseconds(20);
    }

    // Encoder should echo 0x10 when ready
    if (response != 0x10) {
        Serial.print("Error, response: 0x");
        Serial.println(response, HEX);
        return 0xFFFF;
    }

    // Now read position bytes
    byte msb = sendCommand(0x00);
    delayMicroseconds(20);

    byte lsb = sendCommand(0x00);
    delayMicroseconds(20);

    // Datasheet: lower 4 bits of MSB byte are upper 4 bits of position
    uint16_t position = ((uint16_t)(msb & 0x0F) << 8) | lsb;

    return position;
}