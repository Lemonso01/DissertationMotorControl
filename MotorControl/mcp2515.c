#include "mcp2515.h"

void mcp2515_select() {
    gpio_put(MCP2515_CS_PIN, 0);
}

void mcp2515_deselect() {
    gpio_put(MCP2515_CS_PIN, 1);
}

void mcp2515_reset() {
    mcp2515_select();
    uint8_t cmd = MCP_RESET;
    spi_write_blocking(spi0, &cmd, 1);
    mcp2515_deselect();
    sleep_ms(10);
}

void mcp2515_write_register(uint8_t reg, uint8_t value) {
    mcp2515_select();
    uint8_t buf[3] = {MCP_WRITE, reg, value};
    spi_write_blocking(spi0, buf, 3);
    mcp2515_deselect();
}

uint8_t mcp2515_read_register(uint8_t reg) {
    mcp2515_select();
    spi_write_blocking(spi0, &reg, 1);  // Send the register address
    uint8_t value;
    spi_read_blocking(spi0, 0x00, &value, 1);  // Read the value
    mcp2515_deselect();
    return value;
}

void mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t data) {
    mcp2515_select();
    uint8_t buf[4] = {MCP_BITMOD, reg, mask, data};
    spi_write_blocking(spi0, buf, 4);
    mcp2515_deselect();
}

void mcp2515_enable_loopback() {
    uint8_t ctrl_value = 0x10;  // 0x10 sets the loopback mode bit (LOM)
    mcp2515_bit_modify(MCP_CANCTRL, 0x10, ctrl_value);  // Modify the CANCTRL register to enable loopback mode
}

void mcp2515_disable_loopback() {
    // MCP2515 CANCTRL register's loopback mode bit (LOM)
    uint8_t ctrl_value = 0x00;  // 0x00 clears the loopback mode bit (LOM)
    mcp2515_bit_modify(MCP_CANCTRL, 0x10, ctrl_value);  // Modify the CANCTRL register to disable loopback mode
}

void mcp2515_init() {
    gpio_init(MCP2515_CS_PIN);
    gpio_set_dir(MCP2515_CS_PIN, GPIO_OUT);
    mcp2515_deselect();

    spi_init(spi0, 1000 * 1000);
    gpio_set_function(16, GPIO_FUNC_SPI); // MISO
    gpio_set_function(17, GPIO_FUNC_SPI); // CS (manual control)
    gpio_set_function(18, GPIO_FUNC_SPI); // SCK
    gpio_set_function(19, GPIO_FUNC_SPI); // MOSI

    mcp2515_reset();

    mcp2515_write_register(MCP_CNF1, 0x00);  // 1 Mbps
    mcp2515_write_register(MCP_CNF2, 0x90);
    mcp2515_write_register(MCP_CNF3, 0x02);

    // Normal mode (REQOP=000)
    mcp2515_write_register(MCP_CANCTRL, 0x00);
}

void mcp2515_send_extended(uint32_t id, uint8_t *data, uint8_t len) {
    uint8_t sidh = (uint8_t)(id >> 21);
    uint8_t sidl = (uint8_t)(((id >> 16) & 0x03) << 5) | (1 << 3) | ((id >> 18) & 0x07);
    uint8_t eid8 = (uint8_t)(id >> 8);
    uint8_t eid0 = (uint8_t)(id);

    mcp2515_write_register(MCP_TXB0SIDH, sidh);
    mcp2515_write_register(MCP_TXB0SIDL, sidl);
    mcp2515_write_register(MCP_TXB0EID8, eid8);
    mcp2515_write_register(MCP_TXB0EID0, eid0);
    mcp2515_write_register(MCP_TXB0DLC, len);

    for (int i = 0; i < len; i++) {
        mcp2515_write_register(MCP_TXB0D0 + i, data[i]);
    }

    mcp2515_select();
    uint8_t rts = MCP_RTS_TX0;
    spi_write_blocking(spi0, &rts, 1);
    mcp2515_deselect();
}

#define CAN_PACKET_SET_RPM 3

void send_rpm(uint8_t motor_id, int32_t rpm) {
    uint8_t data[4];
    data[0] = (rpm >> 24) & 0xFF;
    data[1] = (rpm >> 16) & 0xFF;
    data[2] = (rpm >> 8) & 0xFF;
    data[3] = rpm & 0xFF;

    uint32_t id = (CAN_PACKET_SET_RPM << 8) | motor_id;
    mcp2515_send_extended(id, data, 4);
}

#define CAN_PACKET_SET_POS 4

void send_position(uint8_t motor_id, float position) {
    // Scale position from float to int32 as per CubeMars scaling (pos * 10000)
    int32_t pos_int = (int32_t)(position * 10000);

    uint8_t data[4];
    data[0] = (pos_int >> 24) & 0xFF;
    data[1] = (pos_int >> 16) & 0xFF;
    data[2] = (pos_int >> 8) & 0xFF;
    data[3] = pos_int & 0xFF;

    uint32_t can_id = (CAN_PACKET_SET_POS << 8) | motor_id;

    mcp2515_send_extended(can_id, data, 4);
}

bool mcp2515_check_message() {
    // Check the status register of the MCP2515 to see if a message is available in the RX buffer
    uint8_t status = mcp2515_read_register(MCP_CANSTAT);

    // If the "RX0IF" bit is set in the status register, there's a message in RX buffer 0
    if (status & 0x01) {
        return true;  // Message is available
    }

    return false;  // No message available
}

void mcp2515_read_message(uint32_t *id, uint8_t *data, uint8_t *len) {
    // Read the ID from the TXB0SIDH/SIDL and RXB0EID8/EID0 registers
    *id = (mcp2515_read_register(MCP_RXB0SIDH) << 3) | (mcp2515_read_register(MCP_RXB0SIDL) >> 5);
    *len = mcp2515_read_register(MCP_RXB0DLC) & 0x0F;  // Data length code (DLC)

    // Read the data from the RXB0D0 to RXB0D7 registers
    for (int i = 0; i < *len; i++) {
        data[i] = mcp2515_read_register(MCP_RXB0D0 + i);
    }
}