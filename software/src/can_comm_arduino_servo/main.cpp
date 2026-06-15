//
// Created by Oscar Tesniere on 09/06/2026.
//
// might be interesting : https://github.com/lezgin-alimoglu/CubeMars-AK60-6/blob/main/ServoMode/ServoMode.ino

// this script will attempt to control the motor in Servo mode
/*
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

   MotorReply reply;

   reply.func_id = (CAN_eid >> 8) & 0x1FFFFF;  // upper 21 bits
   reply.can_id  = CAN_eid & 0xFF;             // lower 8 bits
   // the order is not sure. If it doesn't make sense swap them
   //TODO make sure that this is the correct way to obtain func ID and motor ID
   //Serial.print("Received FUNCTION_ID  :");
   //Serial.print(reply.func_id);
   //Serial.print("  Received CAN_ID  :");
   //Serial.println(reply.can_id);

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
      else {
         Serial.print("Invalid message of length ");
         Serial.print(rxMsg.data_length);
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

/*
 What to set on the CubeMars software : go to Application Functions
 Set Controller ID to whatever CAN_ID you want the motor to use
 Set the Baudrate to 1M : this is the CAN baudrate
 Set the update status to something reasonable.
 */

/*
NOTES FROM CHATGPT :
You are decoding Servo status frames as if they were MIT-style packed unsigned values. In Servo mode, the feedback frame 0x29 uses signed big-endian int16 values:

 */
//
// CubeMars AK Servo Mode CAN control
// Cleaned up feedback decoding + basic command helpers
//

#include <Arduino.h>
#include <Arduino_CAN.h>

enum CAN_PACKET_ID : uint32_t {
  CAN_PACKET_SET_DUTY          = 0,
  CAN_PACKET_SET_CURRENT       = 1,
  CAN_PACKET_SET_CURRENT_BRAKE = 2,
  CAN_PACKET_SET_RPM           = 3,
  CAN_PACKET_SET_POS           = 4,
  CAN_PACKET_SET_ORIGIN_HERE   = 5,
  CAN_PACKET_SET_POS_SPD       = 6,
  CAN_PACKET_SET_MIT           = 8
};

// Servo feedback function IDs
constexpr uint32_t SERVO_JUMP_START_STATE = 0x09;
constexpr uint32_t SERVO_ENTER_MODE_FRAME = 0x2C;
constexpr uint32_t SERVO_REALTIME_FEEDBACK = 0x29;

struct MotorReply {
  uint8_t motor_eid = 0; // lower byte in eid
  uint32_t func_id = 0; // upper byte in eid

  float position_deg = 0.0f;
  float speed_erpm = 0.0f;
  float current_a = 0.0f;
  int8_t temperature_c = 0;
  uint8_t error = 0;

  bool valid_feedback = false;
};

// ---------- byte helpers ----------

static int16_t read_i16_be(const uint8_t *buf) {
  return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

static void write_i16_be(uint8_t *buf, int16_t value) {
  uint16_t u = (uint16_t)value;
  buf[0] = (uint8_t)(u >> 8);
  buf[1] = (uint8_t)(u);
}

static void write_i32_be(uint8_t *buf, int32_t value) {
  uint32_t u = (uint32_t)value;
  buf[0] = (uint8_t)(u >> 24);
  buf[1] = (uint8_t)(u >> 16);
  buf[2] = (uint8_t)(u >> 8);
  buf[3] = (uint8_t)(u);
}

static uint32_t make_servo_eid(uint8_t motor_id, CAN_PACKET_ID packet_id) {
  // Extended CAN ID: [function/control mode in upper bits][motor id in low 8 bits]
  return ((uint32_t)packet_id << 8) | motor_id;
}

// ---------- feedback decode ----------

MotorReply unpack_servo_reply(uint32_t eid, const uint8_t data[8], uint8_t dlc) {
  // inspired from the "motor_receive_servo" methiod implemented in CubeMar's official documentation on Github
  MotorReply reply;

  reply.motor_eid = eid & 0xFF;
  reply.func_id = (eid >> 8) & 0x1FFFFF;

  if (dlc != 8) {
    Serial.println("Invalid Frame Length. Skipping");
    return reply;
  }

  if (reply.func_id == SERVO_REALTIME_FEEDBACK) {
    // assume the motor sends those values as SIGNED INT16 (=INT16 not UINT16) signed numbers. We first reconstructe the 16 bit numbers,
    // then cast them to signed int16
    // the read_i16_be function takes care of concatenating pairs of bytes into int16 by taking the data indexed at lower byte
    int16_t pos_raw = read_i16_be(&data[0]);
    int16_t spd_raw = read_i16_be(&data[2]);
    int16_t cur_raw = read_i16_be(&data[4]);

    reply.position_deg = (float)pos_raw * 0.1f;
    reply.speed_erpm = (float)spd_raw * 10.0f;
    reply.current_a = (float)cur_raw * 0.01f;
    reply.temperature_c = (int8_t)data[6]; // signed byte -> use SIGNED INT8
    reply.error = data[7];
    reply.valid_feedback = true;
  }

  return reply;
}

// ---------- command send helpers ----------

bool send_extended(uint32_t eid, const uint8_t *data, uint8_t len) {
  CanMsg const msg(CanExtendedId(eid), len, data);
  return CAN.write(msg) >= 0;
}

bool can_set_duty(uint8_t motor_id, float duty) {
  // Manual: int32 duty = duty * 100000
  // Typical duty range depends on configured limits, often about 0.005 to 0.95.
  uint8_t buf[4];
  write_i32_be(buf, (int32_t)(duty * 100000.0f));

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_DUTY),
    buf,
    sizeof(buf)
  );
}

bool can_set_current(uint8_t motor_id, float current_a) {
  // Manual: int32 current = current_A * 1000
  uint8_t buf[4];
  write_i32_be(buf, (int32_t)(current_a * 1000.0f));

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_CURRENT),
    buf,
    sizeof(buf)
  );
}

bool can_set_current_brake(uint8_t motor_id, float brake_current_a) {
  // Manual: int32 brake current = current_A * 1000
  uint8_t buf[4];
  write_i32_be(buf, (int32_t)(brake_current_a * 1000.0f));

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_CURRENT_BRAKE),
    buf,
    sizeof(buf)
  );
}

bool can_set_rpm(uint8_t motor_id, float erpm) {
  // Manual: int32 speed = ERPM directly
  uint8_t buf[4];
  write_i32_be(buf, (int32_t)erpm);

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_RPM),
    buf,
    sizeof(buf)
  );
}

bool can_set_position(uint8_t motor_id, float position_deg) {
  // Manual CAN Servo position mode:
  // int32 position = degrees * 10000
  uint8_t buf[4];
  write_i32_be(buf, (int32_t)(position_deg * 10000.0f));

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_POS),
    buf,
    sizeof(buf)
  );
}

bool can_set_origin(uint8_t motor_id, uint8_t origin_mode) {
  // 0 = temporary origin, lost after power cycle
  // 1 = permanent zero point, dual-encoder models only
  uint8_t buf[1] = { origin_mode };

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_ORIGIN_HERE),
    buf,
    sizeof(buf)
  );
}

bool can_set_position_speed(uint8_t motor_id,
                            float position_deg,
                            float speed_erpm,
                            float accel_erpm_s2) {
  // Manual:
  // position: int32 = deg * 10000
  // speed:    int16 = ERPM / 10
  // accel:    int16 = ERPM/s^2 / 10
  uint8_t buf[8];

  write_i32_be(&buf[0], (int32_t)(position_deg * 10000.0f));
  write_i16_be(&buf[4], (int16_t)(speed_erpm / 10.0f));
  write_i16_be(&buf[6], (int16_t)(accel_erpm_s2 / 10.0f));

  return send_extended(
    make_servo_eid(motor_id, CAN_PACKET_SET_POS_SPD),
    buf,
    sizeof(buf)
  );
}

// ---------- printing ----------

void print_error_code(uint8_t error) {
  switch (error) {
    case 0: Serial.print("none"); break;
    case 1: Serial.print("motor over-temp"); break;
    case 2: Serial.print("over-current"); break;
    case 3: Serial.print("over-voltage"); break;
    case 4: Serial.print("under-voltage"); break;
    case 5: Serial.print("encoder fault"); break;
    case 6: Serial.print("MOSFET over-temp"); break;
    case 7: Serial.print("motor stall"); break;
    default: Serial.print("unknown"); break;
  }
}

void print_reply(const MotorReply &reply) {
  Serial.print("motor_id=");
  Serial.print(reply.motor_eid);

  Serial.print(" func_id=0x");
  Serial.print(reply.func_id, HEX);

  if (!reply.valid_feedback) {
    if (reply.func_id == SERVO_JUMP_START_STATE) {
      Serial.println(" jump-start state");
    } else if (reply.func_id == SERVO_ENTER_MODE_FRAME) {
      Serial.println(" enter-servo-mode frame");
    } else {
      Serial.println(" non-feedback frame");
    }
    return;
  }

  Serial.print(" pos_deg=");
  Serial.print(reply.position_deg, 2);

  Serial.print(" speed_erpm=");
  Serial.print(reply.speed_erpm, 1);

  Serial.print(" current_a=");
  Serial.print(reply.current_a, 3);

  Serial.print(" temp_c=");
  Serial.print(reply.temperature_c);

  Serial.print(" error=");
  Serial.print(reply.error);
  Serial.print(" ");
  print_error_code(reply.error);

  Serial.println();
}

// ---------- Arduino ----------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Initializing CAN communication...");

  if (!CAN.begin(CanBitRate::BR_1000k)) {
    Serial.println("CAN.begin(...) failed.");
    while (true) {}
  }

  Serial.println("CAN ready.");
}

constexpr uint8_t MOTOR_ID = 2;

// RPM sweep settings
constexpr int32_t RPM_MIN = 10000;
constexpr int32_t RPM_MAX = 20000;
constexpr int32_t RPM_STEP = 1;

// Send a new RPM command every 100 ms, like your delay(100)
constexpr unsigned long RPM_UPDATE_PERIOD_MS = 1;

// Optional: print feedback slower than the CAN update rate
constexpr unsigned long PRINT_PERIOD_MS = 200;

int32_t commanded_rpm = RPM_MIN;
int32_t rpm_direction = +1;

unsigned long last_rpm_update_ms = 0;
unsigned long last_print_ms = 0;

void loop() {
  // 1) Always drain CAN frames first
  while (CAN.available()) {
    CanMsg const rxMsg = CAN.read();

    if (!rxMsg.isExtendedId()) {
      Serial.println("Skipping Standard ID CAN frames");
      continue;
    }

    MotorReply reply = unpack_servo_reply(
      rxMsg.getExtendedId(),
      rxMsg.data,
      rxMsg.data_length
    );

    if (!reply.valid_feedback) {
      continue;
    }

    // Optional but recommended: ignore other motors
    if (reply.motor_eid != MOTOR_ID) {
      Serial.println("Skipping invalid motor");
      continue;
    }

     //Do not print every frame; Serial printing can block/slow the loop
    unsigned long now = millis();
    if (now - last_print_ms >= PRINT_PERIOD_MS) {
      last_print_ms = now;
      print_reply(reply);
    }

  }

  // 2) Non-blocking RPM sweep
  unsigned long now = millis();

  if (now - last_rpm_update_ms >= RPM_UPDATE_PERIOD_MS) {
    last_rpm_update_ms = now;

    bool ok = can_set_rpm(MOTOR_ID, commanded_rpm);

    Serial.print("Commanding RPM=");
    Serial.print(commanded_rpm);
    Serial.print(" ok=");
    Serial.println(ok);

    commanded_rpm += rpm_direction * RPM_STEP;

    if (commanded_rpm >= RPM_MAX) {
      commanded_rpm = RPM_MAX;
      rpm_direction = -1;
    } else if (commanded_rpm <= RPM_MIN) {
      commanded_rpm = RPM_MIN;
      rpm_direction = +1;
    }
  }
}