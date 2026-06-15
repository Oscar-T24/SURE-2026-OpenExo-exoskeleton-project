//
// Created by Oscar Tesniere on 14/06/2026.
//

// This script will attempt to combine the multiple components used for the demo of the exoskeleton
/*
 SPI Encoders (2x) : Share one SPI bus (we might opt with separate SPI lines if latency is an issue) and separate CSB pins
 > Need to solder the SPI pins (MISO/MOSI/SCK and as well as one of the CS pins) to the SPI header made available at the corner of the PCB
 > Make sure to include a voltage divider (3x 10k resistors) for MISO and solder the second CS pin to the PCB underneath the Teensy
 Load cells (4x) : using the Amplifier's output
 > Need to unsolder two 1uF capacitors as well as two 22kOhms resistors. Then use the jumper pins to connect
 > the two other amplifier voltage outputs need to be soldered on the PCB.
 CAN transceiver (1x) : using the onboard transceiver chip connection.
 > Non extra soldering required
 */
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>

constexpr int CSB_L  = 0;
constexpr int CSB_R = 9;

// SPI pins : using SPI1 port (do a test before with the Arduino slave / Teensy Master code)
constexpr int SCK1 = 27;
constexpr int MOSI1 = 26;
constexpr int MISO1 = 1;

constexpr unsigned long timeout = 300; // timeout for SPI probing
constexpr uint16_t adc_resolution_bits = 12;
constexpr float voltage_scale = 3.3f;
constexpr uint32_t adc_max_value = (1UL << adc_resolution_bits) - 1;

// command bytes
constexpr byte wt_resp = 0x45; // wait response
constexpr byte rd_pos = 0x10; // read position command
constexpr byte nop = 0x00; // no operation
constexpr byte set_zero = 0x70;

// Load cells :

constexpr int LC_R_1 = 21;
constexpr int LC_L_1 = 41;
constexpr int LC_R_2 = 17;
constexpr int LC_L_2 = 16;

unsigned long lastUpdate;
unsigned long updatePeriod = 200; //200ms period

bool loadcells = false;
bool motors = false;
bool encoders = false;

// CAN :
// CANRX = 23
// CANTX = 22
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANbus;

String serialLine;

float p_target = 10.0f;
float v_target = 0.0f;
float kp_target = 0.5f;
float kd_target = 0.5f;
float trq_target = 0.0f;

static constexpr uint8_t exitMotorMode[8] = {
 0xFF, 0xFF, 0xFF, 0xFF,
 0xFF, 0xFF, 0xFF, 0xFD
};

static constexpr uint8_t enterMotorMode[8] = {
 0xFF, 0xFF, 0xFF, 0xFF,
 0xFF, 0xFF, 0xFF, 0xFC
};

static constexpr uint8_t setZeroPosition[8] = {
 0xFF, 0xFF, 0xFF, 0xFF,
 0xFF, 0xFF, 0xFF, 0xFE
};
static constexpr uint8_t LEFT_MOTOR_ID = 0x02;
static constexpr uint8_t RIGHT_MOTOR_ID = 0x03; //change that once we know


typedef struct {
 float p_min,p_max;
 float v_min,v_max;
 float kp_min,kp_max;
 float kd_min,kd_max;
 float trq_min,trq_max;
} AK60Params;

typedef struct
{
 uint8_t can_id;
 float position;
 float velocity;
 float torque;
 uint8_t temperature;
 uint8_t error;
} MotorReply;

static constexpr AK60Params motorParams = {
 -12.5f,  12.5f,   // position
 -45.0f,  45.0f,   // velocity
   0.0f, 500.0f,   // kp
   0.0f,   5.0f,   // kd
 -15.0f,  15.0f    // torque
};

float uint_to_float(uint16_t code, float x_min, float x_max, int bits)
{

 float span = x_max - x_min;
 float max_int = (float)(((unsigned long)1<< bits) - 1);
 return ((float)code * span / max_int) + x_min;
}

uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
 float span = x_max - x_min;
 float max_int = (float)(((unsigned long)1 << bits) -1);
 return (uint16_t)((x-x_min)* max_int / span);
}
float constrain_float(float x, float x_min, float x_max)
{
 if (x < x_min)
 {
  return x_min;
 }

 if (x > x_max)
 {
  return x_max;
 }

 return x;
}

void pack_cmd(
    uint8_t tx_buf[8],
    float p_in,
    float v_in,
    float kp_in,
    float kd_in,
    float trq_in
)
{
 uint16_t position = float_to_uint(
     p_in,
     motorParams.p_min,
     motorParams.p_max,
     16
 );

 uint16_t velocity = float_to_uint(
     v_in,
     motorParams.v_min,
     motorParams.v_max,
     12
 );

 uint16_t kp = float_to_uint(
     kp_in,
     motorParams.kp_min,
     motorParams.kp_max,
     12
 );

 uint16_t kd = float_to_uint(
     kd_in,
     motorParams.kd_min,
     motorParams.kd_max,
     12
 );

 uint16_t trq = float_to_uint(
     trq_in,
     motorParams.trq_min,
     motorParams.trq_max,
     12
 );

 tx_buf[0] = (position >> 8) & 0xFF;
 tx_buf[1] = position & 0xFF;

 tx_buf[2] = (velocity >> 4) & 0xFF;
 tx_buf[3] = ((velocity & 0x0F) << 4) | ((kp >> 8) & 0x0F);

 tx_buf[4] = kp & 0xFF;

 tx_buf[5] = (kd >> 4) & 0xFF;
 tx_buf[6] = ((kd & 0x0F) << 4) | ((trq >> 8) & 0x0F);

 tx_buf[7] = trq & 0xFF;
}

MotorReply unpack_reply(const uint8_t rx_buf[8])
{
 MotorReply reply;

 reply.can_id = rx_buf[0];

 uint16_t position_raw =
     ((uint16_t)(rx_buf[1] & 0xFF) << 8) |
      (uint16_t)(rx_buf[2] & 0xFF);

 uint16_t velocity_raw =
     ((uint16_t)(rx_buf[3] & 0xFF) << 4) |
     ((uint16_t)(rx_buf[4] & 0xF0) >> 4);

 uint16_t trq_raw =
     ((uint16_t)(rx_buf[4] & 0x0F) << 8) |
      (uint16_t)(rx_buf[5] & 0xFF);

 reply.position = uint_to_float(
     position_raw,
     motorParams.p_min,
     motorParams.p_max,
     16
 );

 reply.velocity = uint_to_float(
     velocity_raw,
     motorParams.v_min,
     motorParams.v_max,
     12
 );

 reply.torque = uint_to_float(
     trq_raw,
     motorParams.trq_min,
     motorParams.trq_max,
     12
 );

 reply.temperature = rx_buf[6];
 reply.error = rx_buf[7];

 return reply;
}

void apply_motor_param(const String &name, float value) {
 if (name == "pos") p_target = value;
 else if (name == "vel") v_target = value;
 else if (name == "trq") trq_target = value;
 else if (name == "kp")  kp_target = value;
 else if (name == "kd")  kd_target = value;
 else {
  Serial.print("bad param: ");
  Serial.println(name);
 }
}

bool apply_logging_token(const String &name, bool enabled) {
  if (name == "encoder" || name == "encoders") {
    encoders = enabled;
    return true;
  }

  if (name == "loadcell" || name == "loadcells") {
    loadcells = enabled;
    return true;
  }

  if (name == "motor" || name == "motors") {
    motors = enabled;
    return true;
  }

  if (name == "all") {
    encoders = enabled;
    loadcells = enabled;
    motors = enabled;
    return true;
  }

  Serial.print("bad logging target: ");
  Serial.println(name);
  return false;
}

void handle_serial_line(String line) {
  line.trim();
  line.toLowerCase();

  if (line.length() == 0) return;

  if (line.startsWith("set update ")) {
    String value_str = line.substring(strlen("set update "));
    value_str.trim();

    if (value_str.length() == 0) {
      Serial.println("usage: set update 10");
      return;
    }

    for (unsigned int i = 0; i < value_str.length(); i++) {
      if (!isDigit(value_str[i])) {
        Serial.println("error: update period must be an unsigned integer in ms");
        return;
      }
    }

    unsigned long new_period = value_str.toInt();

    if (new_period == 0) {
      Serial.println("error: update period must be > 0 ms");
      return;
    }

    updatePeriod = new_period;

    Serial.print("updatePeriod=");
    Serial.print(updatePeriod);
    Serial.println(" ms");

    return;
  }

  if (line.startsWith("enable ") || line.startsWith("disable ")) {
    bool enabled = line.startsWith("enable ");

    String rest;

    if (enabled) {
      rest = line.substring(strlen("enable "));
    } else {
      rest = line.substring(strlen("disable "));
    }

    rest.trim();

    if (rest.length() == 0) {
      Serial.println("usage: enable encoders motors loadcells");
      Serial.println("   or: disable encoders motors loadcells");
      Serial.println("   or: enable all");
      Serial.println("   or: disable all");
      return;
    }

    bool had_error = false;

    while (rest.length() > 0) {
      int sp = rest.indexOf(' ');

      String token;

      if (sp < 0) {
        token = rest;
        rest = "";
      } else {
        token = rest.substring(0, sp);
        rest = rest.substring(sp + 1);
        rest.trim();
      }

      token.trim();

      if (token.length() > 0) {
        if (!apply_logging_token(token, enabled)) {
          had_error = true;
        }
      }
    }

    if (!had_error) {
      print_logging_state();
    }

    return;
  }

  if (!line.startsWith("set ")) {
    Serial.println("unknown command");
    Serial.println("usage:");
    Serial.println("  set update 10");
    Serial.println("  set pos kd kp 1.2 0.3 10");
    Serial.println("  enable encoders motors loadcells");
    Serial.println("  disable encoders motors loadcells");
    return;
  }

  String tok[12];
  int n = 0;

  while (line.length() && n < 12) {
    int sp = line.indexOf(' ');

    if (sp < 0) {
      tok[n++] = line;
      break;
    }

    tok[n++] = line.substring(0, sp);
    line = line.substring(sp + 1);
    line.trim();
  }

  int count = n - 1; // excluding "set"

  if (count <= 0 || count % 2 != 0) {
    Serial.println("usage: set pos kd kp 1.2 0.3 10");
    return;
  }

  int num_params = count / 2;
  int param_start = 1;
  int value_start = 1 + num_params;

  for (int i = 0; i < num_params; i++) {
    apply_motor_param(
      tok[param_start + i],
      tok[value_start + i].toFloat()
    );
  }

  Serial.print("cmd pos=");
  Serial.print(p_target, 4);
  Serial.print(" vel=");
  Serial.print(v_target, 4);
  Serial.print(" kp=");
  Serial.print(kp_target, 4);
  Serial.print(" kd=");
  Serial.print(kd_target, 4);
  Serial.print(" trq=");
  Serial.println(trq_target, 4);
}

void read_serial_commands() {
 while (Serial.available()) {
  char c = Serial.read();

  if (c == '\r') continue;

  if (c == '\n') {
   handle_serial_line(serialLine);
   serialLine = "";
  } else {
   serialLine += c;
  }
 }
}

bool send_can8(uint32_t id, const uint8_t data[8]) {
 CAN_message_t msg;
 msg.id = id;
 msg.len = 8;
 msg.flags.extended = 0; // MIT mode uses standard CAN ID

 memcpy(msg.buf, data, 8);

 int rc = CANbus.write(msg);
 return rc > 0;
}

struct MotorRunningParams {
 uint8_t motor_ID;

 float p_target   = 10.0f;
 float v_target   = 0.0f;
 float kp_target  = 0.5f;
 float kd_target  = 0.5f;
 float trq_target = 0.0f;

 MotorRunningParams(uint8_t id) // constructor for identifying motor
   : motor_ID(id)
 {}
 void setTargets(float p, float v, float kp, float kd, float trq) {
  p_target = p;
  v_target = v;
  kp_target = kp;
  kd_target = kd;
  trq_target = trq;
 }

 bool update() {
  uint8_t tx_buf[8];
  pack_cmd(
             tx_buf,
             p_target,
             v_target,
             kp_target,
             kd_target,
             trq_target
         );
  if (!send_can8(motor_ID, tx_buf)) {
   Serial.println("MIT command send failed");
   return false;
  }
  return true;
 }
};

MotorRunningParams Left_Motor(LEFT_MOTOR_ID);
MotorRunningParams Right_Motor(RIGHT_MOTOR_ID);

uint8_t SPIWrite(uint8_t sendByte, bool isLeft=true) // by default left
{
 //holder for the received over SPI
 uint8_t data;
 //the AMT20 requires the release of the CS line after each byte
 digitalWrite(isLeft ? CS_LEFT : CS_RIGHT, LOW);
 data = SPI.transfer(sendByte);
 digitalWrite(isLeft ? CS_LEFT : CS_RIGHT, HIGH);
 //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
 delayMicroseconds(10);

 return data;
}
uint16_t getEncoderPosition(bool isLeft=true){
 unsigned long start = millis();
 SPIWrite(rd_pos,isLeft);
 delayMicroseconds(100);
 // is it better to use a numeric counter here ?
 while(SPIWrite(nop,isLeft) != rd_pos && millis() - start < timeout){
 }
 if(millis() - start >= timeout){
  Serial.println("Error reading encoder");
  return 0x0000; // return dummy 0 and move on
 }
 uint16_t encoder_position = (SPIWrite(nop,isLeft) & 0x0F) << 8; // read the first 4 bits, shift them up to make room for the next byte (lower eight bits)
 encoder_position |= SPIWrite(nop,isLeft); // read the next byte (=12 bits)
 return encoder_position;
}

void setup() {
 pinMode(CSB_L,OUTPUT);
 pinMode(CSB_R,OUTPUT);

 // IDLE both encoders by setting their Chip Select to high
 digitalWrite(CSB_L, HIGH);
 digitalWrite(CSB_R, HIGH);

 Serial.begin(115200);
 while (!Serial && millis() < 3000) {}

 Serial.println("Initializing FlexCAN_T4...");

 CANbus.begin();
 CANbus.setBaudRate(1000000);
 CANbus.setMaxMB(16);
 CANbus.enableFIFO();
 CANbus.enableFIFOInterrupt();

 Serial.println("CAN ready");
 byte motor_ids[] = {LEFT_MOTOR_ID,RIGHT_MOTOR_ID};

 for (uint8_t i = 0; i<2;i++) {
  Serial.print("Exiting MIT motor mode for motor ID ");
  Serial.println(motor_ids[i]);
  if (!send_can8(motor_ids[i], exitMotorMode)) {
   Serial.print("exitMotorMode send failed for motor ID");
   Serial.println(motor_ids[i]);
  }

  delay(500);

  Serial.print("Entering MIT motor mode for motor ID ");
  Serial.println(motor_ids[i]);
  if (!send_can8(motor_ids[i], enterMotorMode)) {
   Serial.print("enterMotorMode send failed for motor ID ");
   Serial.println(motor_ids[i]);
  }
 }
 Serial.println("CAN motors initialized");


 Serial.println("Starting up Teensy integration script");
 Serial.println("`set update xxxx` : set the Serial update period where xxxx is the update period in ms");
 Serial.println("`enable [encoder,motor,loadcells]`: enable periodic Serial update of a combination of the encoder, motor, loadcells values");
 Serial.println("`disable [encoder,motor,loadcells]`: disable periodic Serial update of a combination of the encoder, motor, loadcells values");

 analogReadResolution(adc_resolution_bits);   // 0–4095 instead of default
 analogReadAveraging(16);    // smoother readings

 SPI.begin();
 SPI.beginTransaction(SPISettings(10000, MSBFIRST, SPI_MODE0));

}

void loop() {
 read_serial_commands();
 // the loop continuously polls the values from the different sensors while printing, on request, these data periodicaly

 // query the encoder positions
 uint16_t encoder_left = getEncoderPosition(true);
 uint16_t encoder_right = getEncoderPosition(false);
 uint16_t left_lc_1_raw = analogRead(LC_L_1);
 uint16_t left_lc_2_raw = analogRead(LC_L_2);
 uint16_t right_lc_1_raw = analogRead(LC_R_1);
 uint16_t right_lc_2_raw = analogRead(LC_R_2);

 // query the load cell voltages
 float left_lc_1_voltage = left_lc_1_raw * voltage_scale / adc_max_value;
 float left_lc_2_voltage = left_lc_2_raw * voltage_scale / adc_max_value;
 float right_lc_1_voltage = right_lc_1_raw * voltage_scale / adc_max_value;
 float right_lc_2_voltage = right_lc_2_raw * voltage_scale / adc_max_value;


 if (millis() - lastUpdate > updatePeriod) {
  // send data to the Serial monitor

  if (loadcells) {
   Serial.print("Left load cell #1 : ");
   Serial.print(left_lc_1_voltage,5);
   Serial.print(" V, ");

   Serial.print("Left load cell #2 : ");
   Serial.print(left_lc_2_voltage,5);
   Serial.print(" V, ");

   Serial.print("Right load cell #1 : ");
   Serial.print(right_lc_1_voltage,5);
   Serial.print(" V, ");

   Serial.print("Right load cell #2 : ");
   Serial.print(right_lc_2_voltage,5);
   Serial.print(" V, ");
  }
  if (encoders) {
   Serial.print("Right encoder :");
   Serial.print(encoder_right,DEC);
   Serial.print(" , ");

   Serial.print("Left encoder :");
   Serial.print(encoder_left,DEC);
   Serial.print(" , ");
  }
  if (motors) {
   read_and_print_can_frames();
  }
  // Then make sure to flush :
  Serial.println();
  lastUpdate = millis();
 }

 // update Kp, Kd, pos, vel, feedforward trq to the motors
 Left_Motor.update();
 Right_Motor.update();
}
