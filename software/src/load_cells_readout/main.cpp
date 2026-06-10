//
// Created by Oscar Tesniere on 10/06/2026.
//

// This script will read the voltage from both load cells and print it on the Arduino Serial Monitor

#define LOAD_CELL_R A0 // right load cell output voltage pin
#define LOAD_CELL_L A1 // left load cell output voltage pin

void setup(){
// no need to set those pins as input here
analogReadResolution(14); // set ADC resolution to 14 bits
Serial.begin(115200);
Serial.println("Starting load cell voltage reader");
}

void loop(){

unsigned float left_lc = analogRead(LOAD_CELL_L);
unsigned float right_lc = analogRead(LOAD_CELL_R);
Serial.print("Left load cell :");
Serial.print(left_lc);
Serial.print(",");
Serial.print("Right load cell :");
Serial.println(right_lc);

delay(200); // 5Hz update
}

