//
// Created by Oscar Tesniere on 09/06/2026.
//

// this script will attempt to control the motor in Servo mode

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

// Response frame :
// Position : Byte 0 and 1 (MSB first)
// Speed : Byte 2 and 3 (MSB first)
// Current : Byte 3 and 4 (MSB first)
// Motor temperature : byte 6
// Error : byte 7

// do two functions : unpack_motor and pack_motor

// Motor reply frame
typedef struct {
   uint8_t can_id;
   float position;
   float speed;
   float current;
   uint8_t temperature;
   uint8_t error;
} MotorReply;

MotorReply unpack_reply(const uint8_t rx_buf[8]){
// shift the bits

}

//function ID definitions for the servo feedback frame format
constexpr byte JUMP_START_STATE = 0x09;
constexpr byte ENTER_SERVO_MODE = 0x2C;
constexpr byte real_time_FEEDBACK = 0x29;

// how to convert signed floats to binary ?