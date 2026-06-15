// Modified sample AMT20 Encoder Arduino Code that returns absolute position count, relative position count and relative angle
// Type y to start running encoder
// Type n to stop running encoder
// Type z to set current position as zero
/* Arduino Pin Connections
SPI Clock (SCK): Pin 13
SPI MOSI:        Pin 11
SPI MISO:        Pin 12
SPI Chip Select: Pin 10
*/

#include <Arduino.h>
#include <SPI.h>

// Serial baud rate
#define baudRate 115200

// SPI timeout limit
#define timoutLimit 100

// AMT20 SPI commands
#define nop 0x00
#define rd_pos 0x10
#define set_zero_point 0x70

// Chip select pin
const int ENCODER_CS = 0;

// Control flags
bool running = false;

// Zero/reference position
uint16_t zeroPosition = 0;

// Function prototype
uint8_t SPIWrite(uint8_t sendByte);

uint16_t readEncoder()
{
    uint8_t data;
    uint8_t timeoutCounter = 0;
    uint16_t currentPosition;

    data = SPIWrite(rd_pos);

    while (data != rd_pos && timeoutCounter++ < timoutLimit)
    {
        data = SPIWrite(nop);
    }

    if (timeoutCounter >= timoutLimit)
    {
        Serial.println("Error obtaining position");
        return 0;
    }

    currentPosition = (SPIWrite(nop) & 0x0F) << 8;
    currentPosition |= SPIWrite(nop);

    return currentPosition;
}
void setup()
{
    Serial.begin(baudRate);

    //pinMode(SCK, OUTPUT);
    //pinMode(MOSI, OUTPUT);
    //pinMode(MISO, INPUT);
    pinMode(ENCODER_CS, OUTPUT);
    SPI1.begin();
    digitalWrite(ENCODER_CS, HIGH);
    SPI1.beginTransaction(
        SPISettings(1000000, MSBFIRST, SPI_MODE0));



    /*
    Serial.println("AMT20 Encoder Test");
    Serial.println("y = start running encoder");
    Serial.println("n = stop running");
    Serial.println("z = set current position as zero");
    */
}

void loop()
{
    // Handle serial commands
    if (Serial.available())
    {
        char c = Serial.read();

        if (c == 'y')
        {
            running = true;
            // Serial.println("Encoder run started");
        }
        else if (c == 'n')
        {
            running = false;
            // Serial.println("Encoder run stopped");
        }
        else if (c == 'z')
        {
            uint16_t currentPosition = readEncoder();
            zeroPosition = currentPosition;

            Serial.print("Zero set at count ");
            Serial.println(zeroPosition);
        }
    }

    if (!running)
    {
        return;
    }

    uint16_t currentPosition = readEncoder();

    // Relative position with wrap-around handling
    int16_t relativePosition =
        ((int32_t)currentPosition - (int32_t)zeroPosition + 2048) % 4096 - 2048;

    float angleDeg =
        relativePosition * 360.0f / 4096.0f;

    /*
    Serial.print("Raw: ");
    Serial.print(currentPosition);

    Serial.print("\tRelative: ");
    Serial.print(relativePosition);

    Serial.print("\tAngle: ");
    */
    Serial.println(angleDeg, 2);
    // Serial.println(" deg");

    delay(500);
}



uint8_t SPIWrite(uint8_t sendByte)
{
    uint8_t data;

    digitalWrite(ENCODER_CS, LOW);
    data = SPI1.transfer(sendByte);
    digitalWrite(ENCODER_CS, HIGH);

    delayMicroseconds(10);

    SPI1.endTransaction();

    return data;
}
