//
// Created by Oscar Tesniere on 10/06/2026.
//

// This script will read the voltage from both load cells and print it on the Arduino Serial Monitor

/*
https://docs.arduino.cc/language-reference/en/functions/analog-io/analogReference/
5.0 V reference:  5.0 / 16383  = 0.305 mV per count
1.5 V reference:  1.5 / 16383  = 0.092 mV per count

Thus setting the internal ADC reference to 1.5V increases the resolution by 3.3x !!
*/
#include <Arduino.h>

#define LOAD_CELL_R A0
#define LOAD_CELL_L A1

constexpr float voltage_scale = 1.5f;
constexpr uint16_t adc_resolution_bits = 14;
constexpr uint32_t adc_max_value = (1UL << adc_resolution_bits) - 1;

void setup() {
    analogReadResolution(adc_resolution_bits); // enable 14 bits resolution
    analogReference(AR_INTERNAL); // set the internal 1.5V reference : ONLY VALID FOR ARDUINO UNOR4 BOARD
    Serial.begin(115200);
    Serial.println("Starting load cell voltage reader");
}

void loop() {
    uint16_t left_raw = analogRead(LOAD_CELL_L);
    uint16_t right_raw = analogRead(LOAD_CELL_R);

    float left_voltage = left_raw * voltage_scale / adc_max_value;
    float right_voltage = right_raw * voltage_scale / adc_max_value;

    Serial.print("Left load cell: ");
    Serial.print(left_voltage, 4);
    Serial.print(" V, ");

    Serial.print("Right load cell: ");
    Serial.print(right_voltage, 4);
    Serial.println(" V");

    delay(200); // 200Hz update
}