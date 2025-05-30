// === MCP2515 CAN Helper Functions (merged with full working implementation) ===
#include "mcp2515.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// === GPIO / SPI Interface ===
void mcp2515_select() {
    gpio_put(MCP2515_CS_PIN, 0);
}

void mcp2515_deselect() {
    gpio_put(MCP2515_CS_PIN, 1);
}

// === MCP2515 Core Functions ===
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
    uint8_t cmd_buf[2] = {MCP_READ, reg};
    uint8_t value = 0;
    mcp2515_select();
    spi_write_blocking(spi0, cmd_buf, 2);
    spi_read_blocking(spi0, 0x00, &value, 1);
    mcp2515_deselect();
    return value;
}

void mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t data) {
    uint8_t cmd[] = {MCP_BITMOD, reg, mask, data};
    mcp2515_select();
    spi_write_blocking(spi0, cmd, 4);
    mcp2515_deselect();
}

uint8_t mcp2515_read_status() {
    uint8_t tx_buf[2] = {MCP_READ_STATUS, 0x00};
    uint8_t rx_buf[2] = {0};
    mcp2515_select();
    spi_write_read_blocking(spi0, tx_buf, rx_buf, 2);
    mcp2515_deselect();
    return rx_buf[1];
}

// === Initialization ===
void mcp2515_init() {
    gpio_init(MCP2515_CS_PIN);
    gpio_set_dir(MCP2515_CS_PIN, GPIO_OUT);
    gpio_put(MCP2515_CS_PIN, 1);

    spi_init(spi0, 1000 * 1000);
    gpio_set_function(16, GPIO_FUNC_SPI);
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);

    mcp2515_reset();
    sleep_ms(10);

    mcp2515_write_register(MCP_CANCTRL, 0x80);
    sleep_ms(10);
    mcp2515_write_register(MCP_CNF1, 0x00);
    mcp2515_write_register(MCP_CNF2, 0xD0);
    mcp2515_write_register(MCP_CNF3, 0x82);
    mcp2515_write_register(MCP_RXB0CTRL, 0x60);

    uint8_t stat = mcp2515_read_register(MCP_CANSTAT);
    printf("CANSTAT after init: 0x%02X\n", stat);
}

// === CAN Mode Switching ===
void mcp2515_enable_loopback() {
    mcp2515_write_register(MCP_CANCTRL, 0x40);
    sleep_ms(10);
}

void mcp2515_disable_loopback() {
    mcp2515_write_register(MCP_CANCTRL, 0x00);
    sleep_ms(10);
}

// === CAN Send Functions ===
void mcp2515_send_extended(uint32_t id, uint8_t *data, uint8_t len) {
    while (mcp2515_read_register(MCP_TXB0CTRL) & 0x08);
    uint8_t sidh = (uint8_t)(id >> 21);
    uint8_t sidl = (uint8_t)(((id >> 16) & 0x03) << 5) | (1 << 3) | ((id >> 18) & 0x07);
    uint8_t eid8 = (uint8_t)((id >> 8) & 0xFF);
    uint8_t eid0 = (uint8_t)(id & 0xFF);

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

void mcp2515_send_standard(uint16_t can_id, uint8_t *data, uint8_t len) {
    while (mcp2515_read_register(MCP_TXB0CTRL) & 0x08);
    uint8_t sidh = (uint8_t)(can_id >> 3);
    uint8_t sidl = (uint8_t)((can_id & 0x07) << 5);
    mcp2515_write_register(MCP_TXB0SIDH, sidh);
    mcp2515_write_register(MCP_TXB0SIDL, sidl);
    mcp2515_write_register(MCP_TXB0DLC, len);
    for (int i = 0; i < len; i++) {
        mcp2515_write_register(MCP_TXB0D0 + i, data[i]);
    }
    mcp2515_select();
    uint8_t rts = MCP_RTS_TX0;
    spi_write_blocking(spi0, &rts, 1);
    mcp2515_deselect();
}

// === CAN Command Utilities ===
void send_rpm(uint8_t motor_id, int32_t rpm) {
    uint8_t data[4];
    data[0] = (rpm >> 24) & 0xFF;
    data[1] = (rpm >> 16) & 0xFF;
    data[2] = (rpm >> 8) & 0xFF;
    data[3] = rpm & 0xFF;
    uint32_t id = (CAN_PACKET_SET_RPM << 8) | motor_id;
    mcp2515_send_extended(id, data, 4);
}

void send_position(uint8_t motor_id, float position) {
    int32_t pos_int = (int32_t)(position * 10000);
    uint8_t data[4];
    data[0] = (pos_int >> 24) & 0xFF;
    data[1] = (pos_int >> 16) & 0xFF;
    data[2] = (pos_int >> 8) & 0xFF;
    data[3] = pos_int & 0xFF;
    uint32_t can_id = (CAN_PACKET_SET_POS << 8) | motor_id;
    mcp2515_send_extended(can_id, data, 4);
}

// === Message Handling ===
bool mcp2515_check_message() {
    uint8_t intf = mcp2515_read_register(MCP_CANINTF);
    return (intf & 0x01) != 0;
}

void mcp2515_clear_rx0if() {
    mcp2515_bit_modify(MCP_CANINTF, 0x01, 0x00);
}

void mcp2515_read_message(uint32_t *id, uint8_t *data, uint8_t *len) {
    uint8_t status = mcp2515_read_status();
    uint8_t addr;
    if (status & 0x40) {
        addr = MCP_RXB0SIDH;
    } else if (status & 0x80) {
        addr = MCP_RXB1SIDH;
    } else {
        *len = 0;
        return;
    }

    uint8_t sidh = mcp2515_read_register(addr);
    uint8_t sidl = mcp2515_read_register(addr + 1);
    uint8_t eid8 = mcp2515_read_register(addr + 2);
    uint8_t eid0 = mcp2515_read_register(addr + 3);

    *id = ((uint32_t)sidh << 21) |
           ((uint32_t)(sidl & 0xE0) << 13) |
           ((uint32_t)(sidl & 0x03) << 16) |
           ((uint32_t)eid8 << 8) |
           (uint32_t)eid0;

    *len = mcp2515_read_register(addr + 4) & 0x0F;
    for (int i = 0; i < *len; i++) {
        data[i] = mcp2515_read_register((addr + 5) + i);
    }

    if (addr == MCP_RXB0SIDH) {
        mcp2515_bit_modify(MCP_CANINTF, 0x01, 0x00);
    } else {
        mcp2515_bit_modify(MCP_CANINTF, 0x02, 0x00);
    }
}

// === Utilities ===
uint16_t float_to_uint16(float val, float min_val, float max_val) {
    if (val < min_val) val = min_val;
    if (val > max_val) val = max_val;
    return (uint16_t)(((val - min_val) / (max_val - min_val)) * 65535.0f);
}
    