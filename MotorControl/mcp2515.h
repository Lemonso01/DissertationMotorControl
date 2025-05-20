#ifndef MCP2515_H
#define MCP2515_H

#include "hardware/spi.h"
#include "pico/stdlib.h"

// GPIO definitions
#define MCP2515_CS_PIN     17
#define MCP2515_INT_PIN    20

// MCP2515 Commands
#define MCP_RESET          0xC0
#define MCP_READ           0x03
#define MCP_WRITE          0x02
#define MCP_BITMOD         0x05
#define MCP_RTS_TX0        0x81

// MCP2515 Registers
#define MCP_CANCTRL        0x0F
#define MCP_CNF1           0x2A
#define MCP_CNF2           0x29
#define MCP_CNF3           0x28
#define MCP_TXB0CTRL       0x30
#define MCP_TXB0SIDH       0x31
#define MCP_TXB0SIDL       0x32
#define MCP_TXB0EID8       0x33
#define MCP_TXB0EID0       0x34
#define MCP_TXB0DLC        0x35
#define MCP_TXB0D0         0x36

void mcp2515_select();
void mcp2515_deselect();
void mcp2515_reset();
void mcp2515_write_register(uint8_t reg, uint8_t value);
void mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t data);
void mcp2515_init();
void mcp2515_send_extended(uint32_t id, uint8_t *data, uint8_t len);
void send_rpm(uint8_t motor_id, int32_t rpm);

#endif
