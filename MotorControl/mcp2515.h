#ifndef MCP2515_H
#define MCP2515_H

#include "hardware/spi.h"
#include "pico/stdlib.h"

// CAN command packet types
#define CAN_PACKET_SET_DUTY          0
#define CAN_PACKET_SET_CURRENT       1
#define CAN_PACKET_SET_CURRENT_BRAKE 2
#define CAN_PACKET_SET_RPM           3
#define CAN_PACKET_SET_POS           4
#define CAN_PACKET_SET_ORIGIN_HERE   5
#define CAN_PACKET_SET_POS_SPD       6

// GPIO definitions
#define MCP2515_CS_PIN     17
#define MCP2515_INT_PIN    20

// MCP2515 Commands
#define MCP_RESET          0xC0
#define MCP_READ           0x03
#define MCP_WRITE          0x02
#define MCP_BITMOD         0x05    // Modify Specific bits in reg
#define MCP_RTS_TX0        0x81    // Request to send message from TX0 buffer

/// MCP2515 Control and Status Registers
#define MCP_CANCTRL        0x0F    // CAN Control Register
#define MCP_CANSTAT        0x0E    // CAN Status Register
#define MCP_EFLG           0x2D    // Error Flag Register
#define MCP_TEC            0x1C    // Transmit Error Count Register
#define MCP_REC            0x1D    // Receive Error Count Register
#define MCP_CNF3           0x28    // Configuration Register 3
#define MCP_CNF2           0x29    // Configuration Register 2
#define MCP_CNF1           0x2A    // Configuration Register 1

// MCP2515 Transmit Buffers
#define MCP_TXB0CTRL       0x30    // Transmit Buffer 0 Control Register
#define MCP_TXB0SIDH       0x31    // Transmit Buffer 0 Standard Identifier High
#define MCP_TXB0SIDL       0x32    // Transmit Buffer 0 Standard Identifier Low
#define MCP_TXB0EID8       0x33    // Transmit Buffer 0 Extended Identifier High
#define MCP_TXB0EID0       0x34    // Transmit Buffer 0 Extended Identifier Low
#define MCP_TXB0DLC        0x35    // Transmit Buffer 0 Data Length Code (DLC)
#define MCP_TXB0D0         0x36    // Transmit Buffer 0 Data Byte 0
#define MCP_TXB1CTRL       0x40    // Transmit Buffer 1 Control Register
#define MCP_TXB1SIDH       0x41    // Transmit Buffer 1 Standard Identifier High
#define MCP_TXB1SIDL       0x42    // Transmit Buffer 1 Standard Identifier Low
#define MCP_TXB1EID8       0x43    // Transmit Buffer 1 Extended Identifier High
#define MCP_TXB1EID0       0x44    // Transmit Buffer 1 Extended Identifier Low
#define MCP_TXB1DLC        0x45    // Transmit Buffer 1 Data Length Code (DLC)
#define MCP_TXB1D0         0x46    // Transmit Buffer 1 Data Byte 0
#define MCP_TXB2CTRL       0x50    // Transmit Buffer 2 Control Register
#define MCP_TXB2SIDH       0x51    // Transmit Buffer 2 Standard Identifier High
#define MCP_TXB2SIDL       0x52    // Transmit Buffer 2 Standard Identifier Low
#define MCP_TXB2EID8       0x53    // Transmit Buffer 2 Extended Identifier High
#define MCP_TXB2EID0       0x54    // Transmit Buffer 2 Extended Identifier Low
#define MCP_TXB2DLC        0x55    // Transmit Buffer 2 Data Length Code (DLC)
#define MCP_TXB2D0         0x56    // Transmit Buffer 2 Data Byte 0

// MCP2515 Receive Buffers
#define MCP_RXB0CTRL       0x60    // Receive Buffer 0 Control Register
#define MCP_RXB0SIDH       0x61    // Receive Buffer 0 Standard Identifier High
#define MCP_RXB0SIDL       0x62    // Receive Buffer 0 Standard Identifier Low
#define MCP_RXB0EID8       0x63    // Receive Buffer 0 Extended Identifier High
#define MCP_RXB0EID0       0x64    // Receive Buffer 0 Extended Identifier Low
#define MCP_RXB0DLC        0x65    // Receive Buffer 0 Data Length Code (DLC)
#define MCP_RXB0D0         0x66    // Receive Buffer 0 Data Byte 0
#define MCP_RXB1CTRL       0x70    // Receive Buffer 1 Control Register
#define MCP_RXB1SIDH       0x71    // Receive Buffer 1 Standard Identifier High
#define MCP_RXB1SIDL       0x72    // Receive Buffer 1 Standard Identifier Low
#define MCP_RXB1EID8       0x73    // Receive Buffer 1 Extended Identifier High
#define MCP_RXB1EID0       0x74    // Receive Buffer 1 Extended Identifier Low
#define MCP_RXB1DLC        0x75    // Receive Buffer 1 Data Length Code (DLC)
#define MCP_RXB1D0         0x76    // Receive Buffer 1 Data Byte 0
#define MCP_RXB2CTRL       0x80    // Receive Buffer 2 Control Register
#define MCP_RXB2SIDH       0x81    // Receive Buffer 2 Standard Identifier High
#define MCP_RXB2SIDL       0x82    // Receive Buffer 2 Standard Identifier Low
#define MCP_RXB2EID8       0x83    // Receive Buffer 2 Extended Identifier High
#define MCP_RXB2EID0       0x84    // Receive Buffer 2 Extended Identifier Low
#define MCP_RXB2DLC        0x85    // Receive Buffer 2 Data Length Code (DLC)
#define MCP_RXB2D0         0x86    // Receive Buffer 2 Data Byte 0

// MCP2515 Other Registers
#define MCP_RTS_TX0        0x81    // Request to Send (TX0)
#define MCP_RTS_TX1        0x82    // Request to Send (TX1)
#define MCP_RTS_TX2        0x83    // Request to Send (TX2)
#define MCP_READ           0x03    // Read register command
#define MCP_WRITE          0x02    // Write register command
#define MCP_BITMOD         0x05    // Bit modify register command
#define MCP_RESET          0xC0    // Reset command
#define MCP_STATUS         0xA0    // Read status register


void mcp2515_select();
void mcp2515_deselect();
void mcp2515_reset();
void mcp2515_write_register(uint8_t reg, uint8_t value);
uint8_t mcp2515_read_register(uint8_t reg);
void mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t data);
void mcp2515_enable_loopback();
void mcp2515_disable_loopback();
void mcp2515_init();
void mcp2515_send_extended(uint32_t id, uint8_t *data, uint8_t len);
void send_rpm(uint8_t motor_id, int32_t rpm);
void send_position(uint8_t motor_id, float position);
bool mcp2515_check_message();
void mcp2515_clear_rx0if();
void mcp2515_read_message(uint32_t *id, uint8_t *data, uint8_t *len);


#endif
