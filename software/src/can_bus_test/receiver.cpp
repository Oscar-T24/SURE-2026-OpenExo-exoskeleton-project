//
// Created by Oscar Tesniere on 04/06/2026.
//

#include <Arduino_CAN.h>

static const uint32_t CAN_ID = 0x100;

uint16_t expectedSequence = 0;

unsigned long receivedMessages = 0;
unsigned long missedMessages = 0;
unsigned long checksumErrors = 0;
unsigned long duplicateOrOldMessages = 0;
unsigned long wrongIdMessages = 0;
unsigned long wrongLengthMessages = 0;

unsigned long lastReportTime = 0;

uint8_t calculateChecksum(const uint8_t *data, uint8_t length)
{
  uint8_t checksum = 0;

  for (uint8_t i = 0; i < length; i++)
  {
    checksum ^= data[i];
  }

  return checksum;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}

  // Must match sender bitrate.
  if (!CAN.begin(CanBitRate::BR_250k))
  {
    Serial.println("CAN.begin(...) failed.");
    while (1) {}
  }

  Serial.println("Arduino R4 CAN integrity receiver started.");
}

void loop()
{
  if (CAN.available())
  {
    CanMsg const msg = CAN.read();

    if (msg.id != CAN_ID)
    {
      wrongIdMessages++;
      return;
    }

    if (msg.data_length != 8)
    {
      wrongLengthMessages++;
      return;
    }

    receivedMessages++;

    uint8_t data[8];

    for (uint8_t i = 0; i < 8; i++)
    {
      data[i] = msg.data[i];
    }

    uint16_t receivedSequence =
      ((uint16_t)data[0] << 8) | data[1];

    uint8_t receivedChecksum = data[6];
    uint8_t calculatedChecksum = calculateChecksum(data, 6);

    if (receivedChecksum != calculatedChecksum)
    {
      checksumErrors++;

      Serial.print("Checksum error. Sequence: ");
      Serial.println(receivedSequence);

      return;
    }

    if (receivedSequence == expectedSequence)
    {
      // Correct message.
      expectedSequence++;
    }
    else if (receivedSequence > expectedSequence)
    {
      // One or more messages were missed.
      uint16_t missed = receivedSequence - expectedSequence;
      missedMessages += missed;

      Serial.print("Missed ");
      Serial.print(missed);
      Serial.print(" message(s). Expected ");
      Serial.print(expectedSequence);
      Serial.print(", received ");
      Serial.println(receivedSequence);

      expectedSequence = receivedSequence + 1;
    }
    else
    {
      // receivedSequence < expectedSequence
      duplicateOrOldMessages++;

      Serial.print("Duplicate/old message. Expected ");
      Serial.print(expectedSequence);
      Serial.print(", received ");
      Serial.println(receivedSequence);
    }
  }

  unsigned long now = millis();

  if (now - lastReportTime >= 1000)
  {
    lastReportTime = now;

    Serial.println();
    Serial.println("----- CAN Integrity Report -----");

    Serial.print("Received messages: ");
    Serial.println(receivedMessages);

    Serial.print("Missed messages: ");
    Serial.println(missedMessages);

    Serial.print("Checksum errors: ");
    Serial.println(checksumErrors);

    Serial.print("Duplicate/old messages: ");
    Serial.println(duplicateOrOldMessages);

    Serial.print("Wrong CAN ID messages: ");
    Serial.println(wrongIdMessages);

    Serial.print("Wrong length messages: ");
    Serial.println(wrongLengthMessages);

    Serial.print("Expected next sequence: ");
    Serial.println(expectedSequence);

    Serial.println("--------------------------------");
  }
}