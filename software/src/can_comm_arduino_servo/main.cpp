//
// Created by Oscar Tesniere on 09/06/2026.
//
// might be interesting : https://github.com/lezgin-alimoglu/CubeMars-AK60-6/blob/main/ServoMode/ServoMode.ino

// this script will attempt to control the motor in Servo mode
#include <Arduino.h>
#include <Arduino_CAN.h>

typedef enum {
CAN_PACKET_SET_DUTY = 0,
CAN_PACKET_SET_CURRENT,
CAN_PACKET_SET_CURRENT_BRAKE,
CAN_PACKET_SET_RPM,
CAN_PACKET_SET_POS,
CAN_PACKET_SET_ORIGIN_HERE,
CAN_PACKET_SET_POS_SPD,
CAN_PACKET_SET_mit=8, // what does this do ?
} CAN_PACKET_ID;

// TXMessage must be Extended CAN ID
// frame format : 21 first bits = control mode, and the last 8 are for the target CAN ID

// duty cycle mode : 4 bytes of data,  MSB first (Data[0] = higher 8 bits)
// current loop mode : 4 bytes of data, MSB first (Data[0] = higher 8 bits)
// current brake mode : Does that mean break at a given current ???
// veclocity loop mode : same as before
// position loop mode : same as before
// set origin mode : 1 byte -> what does this byte represent ????
/* position velocity mode :
position MSB first for the first 4 bytes [0,1,2,3]
speed MSB first for the next 2 bytes [4,5]
acceleration for the last two bytes [6,7]
*/
// how to convert signed floats to binary ?

// Motor response frame format
// what is the uplaod frequency ?

// Frame 0x09 repreents that the motor is in jump start state
//TODO what does that mean ?

constexpr float POSITION_SCALE = 10000.0f; // raw units per degree
constexpr float SPEED_SCALE    = 10.0f;    // ERPM per raw unit
constexpr float CURRENT_SCALE    = 10.0f;    // ERPM/s^2 per raw unit
constexpr float DUTY_SCALE = 1.0f;

// do two functions : unpack_motor and pack_motor

// Motor reply frame
typedef struct {
   uint8_t can_id;
   uint8_t func_id;
   float position;
   float speed;
   float current;
   float temperature;
   uint8_t error;
} MotorReply;

typedef struct {
   float p_min,p_max;
   float v_min,v_max;
   float i_min,i_max;
   float t_min,t_max;
   float duty_min,duty_max;
   // §put the parameters here

} AK60Params;

static constexpr AK60Params motorParams = {
   -36000.0f,  36000.0f,   // position (degrees)
   -327680.0f,  327680.0f,   // velocity (ERPM)
   -60.0f, 60.0f, // current (A)
   -20.0f,127.0f, // temperature
      0.0f,100.0, // PWM

};

float uint_to_float(uint32_t x, float x_min, float x_max, int bits)
   // will turn a uint into a float
   // first we determine the possible range of values from the given x min and x max
   // Given an integer x stored using bits bits, what real-world float value does it represent between x_min and x_max
   {
      float span = x_max - x_min;
      float max_int = (float)(((unsigned long)1 << bits) - 1); // max_int = 2^k - 1 = the number of possible values for k bits
      return ((float)x * span / max_int) + x_min; // normalize the integer from 0.0 to 1.0 then multiply it by the span and add x_min to set minimum.
   // minimum has the effect of shifting the value for example a signed range might mean shofting the "0" in the negatives
   }

uint32_t float_to_uint(float x, float x_min, float x_max, int bits)
{
   float span = x_max - x_min;
   float max_int = (float)(((unsigned long)1 << bits) -1);
   return (uint32_t)((x-x_min)* max_int / span);
}

MotorReply unpack_reply(const uint16_t CAN_eid,const uint8_t data_buf[8]){
// frame format (EID)
   /*
Extended CAN ID:  [ Function ID ][ Motor ID ]
DLC:              8
Data[0]:          position high byte
Data[1]:          position low byte
Data[2]:          speed high byte
Data[3]:          speed low byte
Data[4]:          current high byte
Data[5]:          current low byte
Data[6]:          temperature
Data[7]:          error code
*/
   MotorReply reply;

   reply.can_id = (CAN_eid & 0xFF00) >> 8;
   reply.func_id = (CAN_eid & 0x00FF);
   // the order is not sure. If it doesn't make sense swap them
   Serial.print("Received FUNCTION_ID  :");
   Serial.print(reply.func_id);
   Serial.print("  Received CAN_ID  :");
   Serial.println(reply.can_id);

   reply.position = uint_to_float(
                     (data_buf[0] & 0xFF) << 8 | (data_buf[1] & 0xFF),
                     motorParams.p_min,
                     motorParams.p_max,
                     16
                     )  / POSITION_SCALE;
   reply.speed   = uint_to_float(
                     (data_buf[2] & 0xFF) << 8 | (data_buf[3] & 0xFF),
                     motorParams.v_min,
                     motorParams.v_max,
                     16
                     )  /  SPEED_SCALE;
   reply.current   = uint_to_float(
                     (data_buf[4] & 0xFF) << 8 | (data_buf[5] & 0xFF),
                     motorParams.i_min,
                     motorParams.i_max,
                     16
                     )  / CURRENT_SCALE;

   reply.temperature   = uint_to_float(
                  (data_buf[6] & 0xFF),
                  motorParams.t_min,
                  motorParams.t_max,
                  8
                  ); // temperature is already scaled in degrees
   reply.error  =  data_buf[7] & 0xFF;

   return reply;
}

void int32_to_4byte(uint8_t *buf, uint32_t input_int)
{
   buf[0] = (uint8_t)(input_int >> 24);
   buf[1] = (uint8_t)(input_int >> 16);
   buf[2] = (uint8_t)(input_int >> 8);
   buf[3] = (uint8_t)(input_int);
}


bool can_set_duty(uint8_t motor_id, float duty) {
   uint8_t data_buf[4];
   int32_to_4byte(data_buf,float_to_uint((float) duty * DUTY_SCALE,motorParams.duty_min,motorParams.duty_max,32));
   CanMsg const msg_duty(CanExtendedId(motor_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8)), sizeof(data_buf), data_buf);
   return CAN.write(msg_duty) >= 0; // true if succeds
}

bool can_set_current(uint8_t motor_id, float current) {
   uint8_t data_buf[4];
   int32_to_4byte(data_buf,float_to_uint((float) current * CURRENT_SCALE,motorParams.i_min,motorParams.i_max,32));
   CanMsg const msg_current(CanExtendedId(motor_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8)), sizeof(data_buf), data_buf);
   return CAN.write(msg_current) >= 0; // true if succeds
}

bool can_set_position(uint8_t motor_id, float position) {
   uint8_t data_buf[4];
   int32_to_4byte(data_buf,float_to_uint((float) position * POSITION_SCALE,motorParams.p_min,motorParams.p_max,32));
   CanMsg const msg_position(CanExtendedId(motor_id | ((uint32_t)CAN_PACKET_SET_POS << 8)), sizeof(data_buf), data_buf);
   return CAN.write(msg_position) >= 0; // true if succeds
}
//TODO HOW DO WE SET THE KD, KP for that ?

//function ID definitions for the servo feedback frame format
constexpr byte JUMP_START_STATE = 0x09;
constexpr byte ENTER_SERVO_MODE = 0x2C;
constexpr byte real_time_FEEDBACK = 0x29;

// how to convert signed floats to binary ?

void setup() {
   Serial.begin(115200);
   while (!Serial) {}
   Serial.println("Now initializing CAN communication");
   if (!CAN.begin(CanBitRate::BR_1000k)) // 1M baudrate
   {
      Serial.println("CAN.begin(...) failed.");
      for (;;) {}
   }
   //TODO ADD COMMAND TO ENTER INTO SERVO MODE
}

void loop() {
   while (CAN.available()) {
      CanMsg const rxMsg = CAN.read();

      if (rxMsg.data_length == 8) {
         MotorReply reply = unpack_reply(rxMsg.getExtendedId() & 0xFFFF,rxMsg.data);
         Serial.print("  motor id  ");
         Serial.print(reply.can_id);
         Serial.print("  pos (deg)  ");
         Serial.print(reply.position);
         Serial.print("  vel (ERPM)  ");
         Serial.print(reply.speed);
         Serial.print("  current (A)  ");
         Serial.print(reply.current);
         Serial.print("  temperature (C)  ");
         Serial.println(reply.temperature);
      }
   }
   // Then send a command to set position to say 180 degrees positive
}

/*
src\can_comm_arduino_servo\main.cpp: In function 'void can_set_duty(uint8_t, float)':
src\can_comm_arduino_servo\main.cpp:163:80: error: 'const struct AK60Params' has no member named 'duty_min'; did you mean 't_min'?
    int32_to_4byte(data_buf,float_to_uint((float) duty * DUTY_SCALE,motorParams.duty_min,motorParams.duty_max,32));
                                                                                ^~~~~~~~
                                                                                t_min
src\can_comm_arduino_servo\main.cpp:163:101: error: 'const struct AK60Params' has no member named 'duty_max'; did you mean 't_max'?
    int32_to_4byte(data_buf,float_to_uint((float) duty * DUTY_SCALE,motorParams.duty_min,motorParams.duty_max,32));
                                                                                                     ^~~~~~~~
                                                                                                     t_max
src\can_comm_arduino_servo\main.cpp:165:34: error: return-statement with a value, in function returning 'void' [-fpermissive]
    return CAN.write(msg_duty) >= 0; // true if succeds
                                  ^
src\can_comm_arduino_servo\main.cpp: In function 'void can_set_current(uint8_t, float)':
src\can_comm_arduino_servo\main.cpp:172:37: error: return-statement with a value, in function returning 'void' [-fpermissive]
    return CAN.write(msg_current) >= 0; // true if succeds
                                     ^
src\can_comm_arduino_servo\main.cpp: In function 'void can_set_position(uint8_t, float)':
src\can_comm_arduino_servo\main.cpp:179:38: error: return-statement with a value, in function returning 'void' [-fpermissive]
    return CAN.write(msg_position) >= 0; // true if succeds
                                      ^
src\can_comm_arduino_servo\main.cpp: In function 'void loop()':
src\can_comm_arduino_servo\main.cpp:207:52: error: invalid conversion from 'const uint8_t* {aka const unsigned char*}' to 'uint16_t {aka short unsigned int}' [-fpermissive]
          MotorReply reply = unpack_reply(rxMsg.data);
                                                    ^
src\can_comm_arduino_servo\main.cpp:207:52: error: too few arguments to function 'MotorReply unpack_reply(uint16_t, const uint8_t*)'
src\can_comm_arduino_servo\main.cpp:98:12: note: declared here
 MotorReply unpack_reply(const uint16_t CAN_eid,const uint8_t data_buf[8]){
            ^~~~~~~~~~~~
Compiling .pio\build\arduino_can_cubemars_servo\FrameworkArduino\Serial.cpp.o
Compiling .pio\build\arduino_can_cubemars_servo\FrameworkArduino\SerialObj1.cpp.o
*** [.pio\build\arduino_can_cubemars_servo\src\can_comm_arduino_servo\main.cpp.o] Error 1
==================================================================================================== [FAILED] Took 3.84 seconds ====================================================================================================

Environment                 Status    Duration
--------------------------  --------  ------------
arduino_can_cubemars_servo  FAILED    00:00:03.838
 */