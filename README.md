To run the Arduino-Teensy CAN sender receiver test do : 

(1) Connect each of the Arduino R4 and run `pio run -e can_module_arduinos_receiver` or `pio run -e can_module_arduinos_sender` for the receiver arduino and the sender arduino respectively 

(2) Connect one Teensy, run `pio run -e can_module_teensy_receiver` / `pio run -e can_module_teensy_sender` then do same as (1) for the remaining Arduino 

## CubeMars connection with the SN65HVD230 CAN Transceiver module + Arduino Uno R4

Run the Arduino code with `pio run -e arduino_can_cubemars` which will move the motor back and forth 

