#include <Arduino.h>
#include <FlexCAN_T4.h>

typedef enum {
CAN_PACKET_SET_DUTY = 0, //Duty cycle mode
CAN_PACKET_SET_CURRENT = 1, //Current loop mode
CAN_PACKET_SET_CURRENT_BRAKE = 2, // Current brake mode
CAN_PACKET_SET_RPM = 3, // velocity mode
CAN_PACKET_SET_POS = 4, // Position mode
CAN_PACKET_SET_ORIGIN_HERE = 5, // Set Origin mode
CAN_PACKET_SET_POS_SPD = 6, //Position velocity loop mode
} CAN_PACKET_ID;

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
// data must be a pointer to an array of at most 8 bytes. Len is the length of that array
    uint8_t i=0;
    if (len > 8) {
        len = 8; // Is that to check that the data length does not exceed 8 bytes ???
    }
   CAN_message_t TxMessage;
   TxMessage.flags.extended = 1; // disable Std ID and use EID
   TxMessage.id = id; // Set the extended ID to the provided id
   TxMessage.len = len;
   memcpy(TxMessage.buf, data, len); // Copy the data bytes into the message payload
   Can1.write(msg);
}


void comm_can_set_duty(uint8_t motor_id, float duty_cycle) {
    uint32_t send_index = 0; // Initialize the index for the data buffer
    uint32_t id = motor_id | (CAN_PACKET_SET_DUTY << 8); // Construct the CAN ID by combining the motor ID and packet ID
    uint8_t data[4]; // the four bytes that will hold the float value of the duty cycle
    buffer_append_int32(data, (int32_t)(duty_cycle * 100000.0f), &send_index);
    comm_can_transmit_eid(id, data, send_index); // Transmit the CAN message with the constructed ID and data
}

void comm_can_set_current(uint8_t motor_id, float current) {
    int32_t send_index = 0; // Initialize the index for the data buffer
    // what is the difference between int32_t and uint32_t ??
    uint32_t id = motor_id | (CAN_PACKET_SET_CURRENT << 8); // Construct the CAN ID by combining the motor ID and packet ID
    uint8_t data[4]; // the four bytes that will hold the float value of the duty cycle
    buffer_append_int32(data, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(id, data, send_index); // Transmit the CAN message with the constructed ID and data
}

void comm_can_set_cb(uint8_t motor_id, float current) {
    uint32_t send_index = 0; // Initialize the index for the data buffer
    uint32_t id = motor_id | (CAN_PACKET_SET_CURRENT_BRAKE << 8); // Construct the CAN ID by combining the motor ID and packet ID
    uint8_t data[4]; // the four bytes that will hold the float value of the duty cycle
    buffer_append_int32(data, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(id, data, send_index); // Transmit the CAN message with the constructed ID and data
}

void buffer_append_int32(uint8_t *buffer, int32_t payload,  int32_t *send_index){
// splits a given number (payload) into 4 bytes
// MSB(yte) FIRST
    buffer[(*send_index)++] = (uint8_t)(payload >> 24); // first 8 bytes
    buffer[(*send_index)++] = (uint8_t)(payload >> 16); // next 8 bytes
    buffer[(*send_index)++] = (uint8_t)(payload >> 8); // next 8 bytes
    buffer[(*send_index)++] = (uint8_t)(payload); // last 8 bytes
}

void buffer_append_int16(uint8_t *buffer, int16_t payload,  int16_t *send_index){
    // splits a given number (payload) into 2 bytes
    // MSB(yte) FIRST
    buffer[(*send_index)++] = (uint8_t)(payload >> 8); // next 8 bytes
    buffer[(*send_index)++] = (uint8_t)(payload); // last 8 bytes
}

// Motor RX :
/*
Format :
buf[0] -> position(uint16_t) 8 upper bits
buf[1] -> position(uint16_t) 8 lower bits
buf[2] -> velocity(uint16_t) 8 upper bits
buf[3] -> velocity(uint16_t) 8 lower bits
buf[4] -> current(uint16_t) 8 upper bits
buf[5] -> current(uint16_t) 8 lower bits
buf[6] -> motor temperature(uint8_t)
buf[7] -> (if applicable?) error code (uint8_t)
*/

void comm_can_receive(CAN_message_t* rx_msg, float* motor_p, float* motor_v, float* motor_i,uint8_t*motor_temp,uint8_t* error_code){
// Assuming the RX buffer is stored in a Data defined with uint8_t[8] i.e an array of 8 bytes.
 *motor_p = ((uint16_t)rx_msg->buf[1]) | ((uint16_t)rx_msg->buf[0] << 8)* 0.1f; // left shift to pad with 0s
 *motor_v = ((uint16_t)rx_msg->buf[3]) | ((uint16_t)rx_msg->buf[2] << 8)* 10.0f;
 *motor_i = ((uint16_t)rx_msg->buf[5]) | ((uint16_t)rx_msg->buf[4] << 8)* 0.01f;
 *motor_temp = (uint8_t)rx_msg->buf[6];
 *error_code = (uint8_t)rx_msg->buf[7];
}

// The list of functions needs to be finished.



FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
// define an object for CAN communication with the CAN transceiver module on CAN 1
// RX_SIZE_256 => reception "buffer" of length 256
// TX_SIZE_16 => transmission "buffer" of length 16
// The buffers containes queued commands (to be processed from RX or to be sent on TX).
void setup(){
  Can1.begin();
  Can1.setBaudRate(1000000);  // CubeMars manual says 1 Mbps, no change recommended
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
}

/* from the header file of FlexCAN
typedef struct CAN_message_t {
  uint32_t id = 0;          // can identifier
  uint16_t timestamp = 0;   // FlexCAN time when message arrived
  uint8_t idhit = 0; // filter that id came from
  struct {
    bool extended = 0; // identifier is extended (29-bit)
    bool remote = 0;  // remote transmission request packet type
    bool overrun = 0; // message overrun
    bool reserved = 0;
  } flags;
  uint8_t len = 8;      // length of data
  uint8_t buf[8] = { 0 };       // data
  int8_t mb = 0;       // used to identify mailbox reception
  uint8_t bus = 0;      // used to identify where the message came from when events() is used.
  bool seq = 0;         // sequential frames
} CAN_message_t;

*/

void loop() {
  comm_can_set_duty(2, 0.20f); // change 5 to whatever the motor ID is. How can we find it ?
  delay(100);
}

