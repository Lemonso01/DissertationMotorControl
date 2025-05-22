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

void mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t data) {
    mcp2515_select();
    uint8_t buf[4] = {MCP_BITMOD, reg, mask, data};
    spi_write_blocking(spi0, buf, 4);
    mcp2515_deselect();
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
