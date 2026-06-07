//
// Created by Oscar Tesniere on 07/06/2026.
//
// this script will attempt to move the motor by small increments in MIT mode
// based off the CubeMars manual and this video which demoed the MIT mode with an Arduino : https://www.youtube.com/watch?v=UWc_7-8gXeE&t=296s

#include <Arduino_CAN.h>
#include <Arduino.h>

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
static constexpr uint8_t MOTOR_ID = 0x01;
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

void read_and_print_can_frames()
{
    while (CAN.available())
    {
        CanMsg const rxMsg = CAN.read();

        if (rxMsg.data_length == 8)
        {
            MotorReply reply = unpack_reply(rxMsg.data);

            Serial.print("  motor id: ");
            Serial.print(reply.can_id);

            Serial.print(" pos: ");
            Serial.print(reply.position, 4);

            Serial.print(" vel: ");
            Serial.print(reply.velocity, 4);

            Serial.print(" trq: ");
            Serial.print(reply.torque, 4);

            Serial.print(" temp: ");
            Serial.print(reply.temperature);

            Serial.print(" err: ");
            Serial.println(reply.error);
        }
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

float uint_to_float(uint16_t x, float x_min, float x_max, int bits)
{
// how does the conversion work ?
    float span = x_max - x_min;
    float max_int = (float)((1UL << bits) - 1);
    return ((float)x * span / max_int) + x_min;
}

uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float max_int = (float)((1UL << bits) - 1);

    x = constrain_float(x, x_min, x_max);

    return (uint16_t)((x - x_min) * max_int / span);
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

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("Now initializing CAN communication");
    if (!CAN.begin(CanBitRate::BR_1000k)) // 1M baudrate
    {
        Serial.println("CAN.begin(...) failed.");
        for (;;) {}
    }
    Serial.println("Now leaving Motor Mode");
    CanMsg const msg_leave(CanStandardId(MOTOR_ID), sizeof(exitMotorMode), exitMotorMode);
    // first exit motor mode as a saefty
    if (int const rc = CAN.write(msg_leave); rc < 0)
    {
        Serial.print  ("CAN.write(...) failed with error code ");
        Serial.println(rc);
        for (;;) { }
    }
    delay(1000);
    Serial.println("Now entering Motor Mode");
    CanMsg const msg_enter(CanStandardId(MOTOR_ID), sizeof(enterMotorMode), enterMotorMode);
    // first exit motor mode as a saefty
    if (int const rc = CAN.write(msg_enter); rc < 0)
    {
        Serial.print  ("CAN.write(...) failed with error code ");
        Serial.println(rc);
        for (;;) { }
    }

}
void loop()
{
    read_and_print_can_frames();

    static float p_target = 0.0f;
    uint8_t tx_buf[8];

    p_target += 0.001f;

    if (p_target > 1.0f)
    {
        p_target = 1.0f;
    }

    pack_cmd(
        tx_buf,
        p_target,
        0.0f,
        20.0f,
        1.0f,
        0.0f
    );

    CanMsg const msg(CanStandardId(MOTOR_ID), 8, tx_buf);
    CAN.write(msg);

    delay(10);

    read_and_print_can_frames();
}