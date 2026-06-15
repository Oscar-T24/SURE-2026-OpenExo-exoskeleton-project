// This code is used to test 2 load cells connected the amplifier board connected to an Arduino board through the A0 and A1 analog channels
// Amplifier count is converted to voltage on 5V basis; for Teensy, need to change to 3.3V range

#include <Arduino.h>

#define LOAD_CELL_R A0
#define LOAD_CELL_L A1

void setup() {
    Serial.begin(115200);
    Serial.println("Starting load cell voltage reader");
}

void loop() {
    int left_raw = analogRead(LOAD_CELL_L);
    float left_voltage = left_raw * 5.0 / 1023.0;
    int right_raw = analogRead(LOAD_CELL_R);
    float right_voltage = right_raw * 5.0 / 1023.0;

    Serial.print("Left load cell: ");
    Serial.print(left_voltage, 4);

    Serial.print(" Right load cell: ");
    Serial.println(right_voltage, 4);

    delay(500);
}
