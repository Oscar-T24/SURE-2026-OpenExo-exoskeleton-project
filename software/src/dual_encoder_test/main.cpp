// This script will sample the encoder position from the two encoders and print it to serial for viewing it with the serial plotter

#include <Arduino.h>
#include <SPI.h>

constexpr int CS_LEFT = 10;
constexpr int CS_RIGHT = 9;

/* to avoid time compile errors when the Arduino library already defines macros for SPI pins, we skip them
#ifndef SCK
constexpr int SCK = 13;
#endif

#ifndef MOSI
constexpr int MOSI = 11;
#endif

#ifndef MISO
constexpr int MISO = 12;
#endif
*/

constexpr unsigned long timeout = 200;
// what's the added value of constexpr vs const ?

// command bytes
constexpr byte wt_resp = 0x45; // wait response
constexpr byte rd_pos = 0x10; // read position command
constexpr byte nop = 0x00; // no operation
constexpr byte set_zero = 0x70;

//Normally, CSB goes low, then after 8 clock cycles the command is interprete

void setup(){
    Serial.begin(115200);
    pinMode(CS_RIGHT, OUTPUT);
    pinMode(CS_LEFT, OUTPUT);
    SPI.begin();
    SPI.beginTransaction(SPISettings(10000, MSBFIRST, SPI_MODE0));

    // idle the encoders
    digitalWrite(CS_LEFT, HIGH);
    digitalWrite(CS_RIGHT, HIGH);
}

uint8_t SPIWrite(uint8_t sendByte, bool isLeft=true) // by default left
{
    //holder for the received over SPI
    uint8_t data;
    //the AMT20 requires the release of the CS line after each byte
    digitalWrite(isLeft ? CS_LEFT : CS_RIGHT, LOW);
    data = SPI.transfer(sendByte);
    digitalWrite(isLeft ? CS_LEFT : CS_RIGHT, HIGH);
    //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
    delayMicroseconds(10);

    return data;
}

uint16_t getEncoderPosition(bool isLeft=true){
    unsigned long start = millis();
    SPIWrite(rd_pos,isLeft);
    delayMicroseconds(100);
    // is it better to use a numeric counter here ?
    while(SPIWrite(nop,isLeft) != rd_pos && millis() - start < timeout){
    }
    if(millis() - start >= timeout){
        //Serial.println("Error reading encoder");
        return 0x0000; // return dummy 0 and move on
    }
    uint16_t encoder_position = (SPIWrite(nop,isLeft) & 0x0F) << 8; // read the first 4 bits, shift them up to make room for the next byte (lower eight bits)
    encoder_position |= SPIWrite(nop,isLeft); // read the next byte (=12 bits)
    return encoder_position;
}

void loop(){
// send the rd_pos command and wait till it receives the command back
uint16_t encoder_left = getEncoderPosition(false);
//uint16_t encoder_right = getEncoderPosition(false);
Serial.print("Left encoder ");
Serial.println(encoder_left, DEC);
//Serial.print(",");
//Serial.print("Right encoder ");
//Serial.println(encoder_right, DEC);
// Open Serial Plotter to visualize
}




