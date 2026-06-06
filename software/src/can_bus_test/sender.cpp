//
// Created by Oscar Tesniere on 04/06/2026.
//

// This code is intended at validating the communication between two SN65HVD230DR (one on the PCB and one on the module bought on Amazon)
// To try communicating : send extended frames messages
// Use one arduino as receiver and anotehr as sender

#include <Arduino_CAN.h>

static const uint32_t CAN_ID = 0x100;

// Send one CAN frame every 10 ms.
// 10 ms = 100 frames per second.
static const unsigned long SEND_INTERVAL_MS = 10;

uint16_t sequenceNumber = 0;
unsigned long lastSendTime = 0;

uint8_t calculateChecksum(const uint8_t *data, uint8_t length)
{
  uint8_t checksum = 0;

  for (uint8_t i = 0; i < length; i++)
  {
    checksum ^= data[i];   // Simple XOR checksum
  }

  return checksum;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}

  // Use the same bitrate on both Arduinos.
  // You can also use BR_500k if your setup supports it.
  if (!CAN.begin(CanBitRate::BR_250k))
  {
    Serial.println("CAN.begin(...) failed.");
    while (1) {}
  }

  Serial.println("Arduino R4 CAN integrity sender started.");
}

void loop()
{
  unsigned long now = millis();

  if (now - lastSendTime >= SEND_INTERVAL_MS)
  {
    lastSendTime = now;

    uint8_t data[8];

    // Bytes 0-1: sequence number, big-endian
    data[0] = (sequenceNumber >> 8) & 0xFF;
    data[1] = sequenceNumber & 0xFF;

    // Bytes 2-5: test pattern
    data[2] = 0xAA;
    data[3] = 0x55;
    data[4] = sequenceNumber & 0xFF;
    data[5] = (~sequenceNumber) & 0xFF;

    // Byte 6: checksum over bytes 0 to 5
    data[6] = calculateChecksum(data, 6);

    // Byte 7: reserved/debug byte
    data[7] = 0x00;

    CanMsg msg(CAN_ID, sizeof(data), data);

    if (CAN.write(msg))
    {
      Serial.print("Sent sequence: ");
      Serial.println(sequenceNumber);
    }
    else
    {
      Serial.println("CAN.write(...) failed.");
    }

    sequenceNumber++;
  }
}

