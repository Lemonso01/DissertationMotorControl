#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// === Chip Select GPIO Pin ===
#define MCP2515_CS_PIN 17

// === MCP2515 Register Definitions ===
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

// === Function Prototypes ===
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

void mcp2515_send_extended(uint32_t id, uint8_t *data, uint8_t len);
void mcp2515_send_standard(uint16_t id, uint8_t *data, uint8_t len);

bool mcp2515_check_message();
void mcp2515_clear_rx0if();
void mcp2515_read_message(uint32_t *id, uint8_t *data, uint8_t *len);

// === Motor Commands ===
void send_rpm(uint8_t motor_id, int32_t rpm);
void send_position(uint8_t motor_id, float position);

// === Utilities ===
uint16_t float_to_uint16(float val, float min_val, float max_val);

#endif // MCP2515_H