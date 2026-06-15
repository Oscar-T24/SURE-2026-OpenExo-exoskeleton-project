//
// Created by Oscar Tesniere on 12/06/2026.
//
// this script will attempt to move the motor by small increments in MIT mode
// based off the CubeMars manual and this video which demoed the MIT mode with an Arduino : https://www.youtube.com/watch?v=UWc_7-8gXeE&t=296s

#include <FlexCAN_T4.h>
#include <Arduino.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan;

// Teensy 4.1 CAN1:
// RX = pin 23
// TX = pin 22

String serialLine;

float p_target = 10.0f;
float v_target = 0.0f;
float kp_target = 0.5f;
float kd_target = 0.5f;
float trq_target = 0.0f;

// function prototypes to move the function definitions after usage
float constrain_float(float x, float x_min, float x_max);
uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(uint16_t x, float x_min, float x_max, int bits);

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
static constexpr uint8_t MOTOR_ID = 0x02;
// send this command to CANTX to turn the motor into active motor mode first!
//
typedef struct {
    float p_min,p_max;
    float v_min,v_max;
    float kp_min,kp_max;
    float kd_min,kd_max;
    float trq_min,trq_max;
// put the parameters here

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

MotorReply unpack_reply(const uint8_t rx_buf[8]);

static constexpr AK60Params motorParams = {
    -12.5f,  12.5f,   // position
    -45.0f,  45.0f,   // velocity
      0.0f, 500.0f,   // kp
      0.0f,   5.0f,   // kd
    -15.0f,  15.0f    // torque
};

/*
sendMsgBuf(0x01,0,8,enterMotorMode); // this turns on motor
// motor ID, extended = 0, DCL(data length code) = 8, then the 8 bytes message
*/
// command packing :When using CAN communication to control the motor, you must enter the motor MIT
/*
buf[0] : [position[15-8]]
buf[1] : [position[7-0]]
buf[2] : [velocity[11-4]]
buf[3] : [velocity[3-0],kp[11-8]]
buf[4] : [kp[7-0]]
buf[5] : [kd[11-4]]
buf[6] : [kd[3-0],trq[11-8]]
buf[7] : [trq[7-0]]

// 0xFF = 8 bits
// 0x0F or 0xF0 = 4 bits (since 0xF = 1111, 0x0 = 0000)
// bit shifting has higher precedence than masking (bitwise AND) so put parenthesis
// is 0x0F the same as 0xF ? Might be less intuitive when masking 16 bit words
*/

void read_and_print_can_frames() {
    CAN_message_t rxMsg;

    while (myCan.read(rxMsg)) {
        if (rxMsg.len != 8) {
            //Serial.println("Wrong number");
            continue;
        }

        MotorReply reply = unpack_reply(rxMsg.buf);

        Serial.print("  motor id: ");
        Serial.print(reply.can_id);

        Serial.print(" pos(rad): ");
        Serial.print(reply.position, 4);

        Serial.print(" vel(rad/s): ");
        Serial.print(reply.velocity, 4);

        Serial.print(" trq(N*m): ");
        Serial.print(reply.torque, 4);

        Serial.print(" temp(C): ");
        Serial.print(reply.temperature);

        Serial.print(" err: ");
        Serial.println(reply.error);
    }
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

// command unpacking
/*
buf[0] : CAN_ID[7-0]
buf[1] : position[15-8]
buf[2] : position[7-0]
buf[3] : velocity[11-4]
buf[4] : velocity[3-0] | trq[11-8]
buf[5] : trq[7-0]
buf[6] : temp[7-0]
buf[7] : error[7-0]

// position (in rad) ranges from -12.5f to +12.5f (i.e +/- 4pi)
// velocity (in rad/s) ranges from -45.0f to + 45.0f
// torque ( in N*M) ranges from -15.0f to +15.0f
*/

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

void apply_param(const String &name, float value) {
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

void handle_serial_line(String line) {
    line.trim();
    line.toLowerCase();

    if (!line.startsWith("set ")) return;

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
        apply_param(
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

    int rc = myCan.write(msg);
    return rc > 0;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("Initializing FlexCAN_T4...");

    myCan.begin();
    myCan.setBaudRate(500000);//myCan.setBaudRate(1000000);
    myCan.setMaxMB(16);
    myCan.enableFIFO();
    myCan.enableFIFOInterrupt();

    Serial.println("CAN ready");

    Serial.println("Exiting MIT motor mode...");
    if (!send_can8(MOTOR_ID, exitMotorMode)) {
        Serial.println("exitMotorMode send failed");
    }

    delay(1000);

    Serial.println("Entering MIT motor mode...");
    if (!send_can8(MOTOR_ID, enterMotorMode)) {
        Serial.println("enterMotorMode send failed");
    }
}
unsigned long lastUpdate = 0;
const unsigned long updatePeriod = 10; // 100 Hz

void loop() {
    read_serial_commands();
    read_and_print_can_frames();

    if (millis() - lastUpdate >= updatePeriod) {
        lastUpdate = millis();

        uint8_t tx_buf[8];

        pack_cmd(
            tx_buf,
            p_target,
            v_target,
            kp_target,
            kd_target,
            trq_target
        );

        if (!send_can8(MOTOR_ID, tx_buf)) {
            Serial.println("MIT command send failed");
        }
    }
}