#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <string.h>

// === Chip Select GPIO Pin ===
#define MCP2515_CS_PIN 17

// === MCP2515 Register Definitions ===
#define MCP_TEC         0x1C
#define MCP_REC         0x1D
#define MCP_CANSTAT     0x0E
#define MCP_CANCTRL     0x0F
#define MCP_CNF1        0x2A
#define MCP_CNF2        0x29
#define MCP_CNF3        0x28
#define MCP_TXB0CTRL    0x30
#define MCP_TXB0SIDH    0x31
#define MCP_TXB0SIDL    0x32
#define MCP_TXB0EID8    0x33
#define MCP_TXB0EID0    0x34
#define MCP_TXB0DLC     0x35
#define MCP_TXB0D0      0x36
#define MCP_RXB0CTRL    0x60
#define MCP_RXB0SIDH    0x61
#define MCP_RXB0SIDL    0x62
#define MCP_RXB0EID8    0x63
#define MCP_RXB0EID0    0x64
#define MCP_RXB0DLC     0x65
#define MCP_RXB0D0      0x66
#define MCP_RXB1CTRL    0x70
#define MCP_RXB1SIDH    0x71
#define MCP_RXB1SIDL    0x72
#define MCP_RXB1EID8    0x73
#define MCP_RXB1EID0    0x74
#define MCP_RXB1DLC     0x75
#define MCP_RXB1D0      0x76
#define MCP_CANINTF     0x2C
#define MCP_EFLG        0x2D

// === SPI Commands ===
#define MCP_RESET        0xC0
#define MCP_READ         0x03
#define MCP_WRITE        0x02
#define MCP_BITMOD       0x05
#define MCP_READ_STATUS  0xA0
#define MCP_RTS_TX0      0x81

// === CAN Function Commands ===

#define CAN_PACKET_SET_DUTY             0
#define CAN_PACKET_SET_CURRENT_LOOP     1
#define CAN_PACKET_SET_CURRENT_BRAKE    2
#define CAN_PACKET_SET_RPM              3
#define CAN_PACKET_SET_POS              4
#define CAN_PACKET_SET_ORIGIN           5
#define CAN_PACKET_SET_POSRPM           6

// === Torque Constant ===

#define MOTOR_TORQUE_CONSTANT 0.078f


typedef struct {
    uint32_t id;         ///< 11-bit if extended==false, else 29-bit
    bool     extended;   ///< false=standard, true=extended
    uint8_t  dlc;        ///< number of data bytes (0–8)
    uint8_t  data[8];    ///< payload
} can_frame_t;

typedef struct {
    float   pos_deg;       // degrees
    float   speed_erpm;    // electrical RPM
    float   current_A;     // amps
    int8_t  temp_C;        // °C
    uint8_t err;           // error code
} servo_telem_t;

// === Function Prototypes ===
static bool mcp2515_set_mode(uint8_t reqop_bits);
void mcp2515_init();
void mcp2515_reset();
void mcp2515_select();
void mcp2515_deselect();

void mcp2515_write_register(uint8_t reg, uint8_t value);
uint8_t mcp2515_read_register(uint8_t reg);
void mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t data);
uint8_t mcp2515_read_status();

void mcp2515_enable_loopback();
void mcp2515_disable_loopback();

void mcp2515_send_extended(uint32_t id, const uint8_t *data, uint8_t len);
void mcp2515_send_standard(uint16_t id, const uint8_t *data, uint8_t len);

bool mcp2515_check_message();
void mcp2515_clear_rx0if();
void mcp2515_read_message(uint32_t *id, uint8_t *data, uint8_t *len);

// === Motor Commands ===
void send_rpm(uint8_t motor_id, int32_t rpm);
void send_position(uint8_t motor_id, float degrees);
void send_torque(uint8_t motor_id, float torque_nm);

// === Utilities ===
uint32_t float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits);

// === MIT Mode ===
void mcp2515_send_frame(const can_frame_t *frm);
bool mcp2515_receive_frame(can_frame_t *frm);
void mit_pack_cmd(can_frame_t *frm, uint8_t drv_id, float p_des, float v_des, float kp, float kd, float t_ff);
void mit_unpack_reply(const can_frame_t *frm, float *p, float *v, float *t, float *kd, int *rawTemp, int *err);
bool mit_recv_reply(uint8_t drv_id, float *p, float *v, float *t, float *kd, int *rawT, int *err);

static void mcp2515_print_bus_health(void);
static inline void decode_servo_telem(const can_frame_t *rx, servo_telem_t *t);
static inline bool is_servo_telem(const can_frame_t *rx, uint8_t motor_id);
void process_can_rx(uint8_t motor1_id, uint8_t motor2_id);

#endif // MCP2515_H