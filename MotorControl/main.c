#include "pico/stdlib.h"
#include "mcp2515.h"
#include <stdio.h>


// Error counter registers
#define MCP_TEC 0x1C
#define MCP_REC 0x1D


void scan_motor_ids() {
    printf("Starting motor ID scan (1–127) using RPM commands...\n");

    for (uint8_t node_id = 1; node_id <= 127; node_id++) {
        uint32_t full_can_id = ((uint32_t)CAN_PACKET_SET_RPM << 8) | node_id;

        printf("Trying node ID: %3d (0x%02X), Full CAN ID: 0x%03X\n",
               node_id, node_id, full_can_id);

        send_rpm(node_id, 1000);  // Send 1000 ERPM
        send_rpm(node_id, 0); //Safety stop 
        sleep_ms(250);
    }

    printf("Scan complete. Look for movement or feedback.\n");
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void can_receive_loop() {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;

    while (true) {
        if (mcp2515_check_message()) {
            mcp2515_read_message(&id, data, &len);
            printf("Received CAN message ID: 0x%03X, len: %d\n", id, len);
            for (int i = 0; i < len; i++) {
                printf("%02X ", data[i]);
            }
            printf("\n");
        } else {
            printf("Waiting for message...\n");
        }
        sleep_ms(500);  // Polling interval
    }
}

void can_feedback_real() {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;

    while (true) {
        while (mcp2515_check_message()) {
            mcp2515_read_message(&id, data, &len);

            printf("ID: 0x%03X | LEN: %d | DATA: ", id, len);
            for (int i = 0; i < len; i++) {
                printf("%02X ", data[i]);
            }
            printf("\n");
        }

        // Clear overflow flags (RX0/1IF)
        mcp2515_bit_modify(0x2C, 0x03, 0x00);  // MCP_CANINTF = 0x2C
        mcp2515_bit_modify(0x2D, 0xFF, 0x00);  // MCP_EFLG = 0x2D (clear all errors)

        sleep_ms(1);  // stay aggressive
    }
}



void can_receive_loop_mit() {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;

    while (true) {
        if (mcp2515_check_message()) {
            mcp2515_read_message(&id, data, &len);

            if (len == 8) {
                int motor_id = data[0];

                int p_int = (data[1] << 8) | data[2];
                int v_int = (data[3] << 4) | (data[4] >> 4);
                int t_int = ((data[4] & 0xF) << 8) | data[5];

                float position = ((float)p_int) * 25.0f / 65535.0f - 12.5f;
                float velocity = ((float)v_int) * 100.0f / 4095.0f - 50.0f;
                float torque   = ((float)t_int) * 36.0f / 4095.0f - 18.0f;

                int temperature = (int)data[6] - 40;
                uint8_t error_flags = data[7];

                printf("MIT FEEDBACK | ID: %d | Pos: %.2f rad | Vel: %.2f rad/s | Tq: %.2f Nm | Temp: %d°C | Err: 0x%02X\n",
                       motor_id, position, velocity, torque, temperature, error_flags);
            } else {
                printf("Received unexpected frame length: %d\n", len);
            }
        } else {
            printf("...Waiting for MIT feedback\n");
        }

        sleep_ms(250);  // Adjust polling rate if needed
    }
}

void print_can_errors() {
    uint8_t eflg = mcp2515_read_register(MCP_EFLG);
    uint8_t tec = mcp2515_read_register(MCP_TEC);
    uint8_t rec = mcp2515_read_register(MCP_REC);

    printf("CAN Error Flags: EFLG = 0x%02X | TEC = %d | REC = %d\n", eflg, tec, rec);

    if (eflg) {
        if (eflg & 0x01) printf("  - RX0 Overflow\n");
        if (eflg & 0x02) printf("  - RX1 Overflow\n");
        if (eflg & 0x04) printf("  - TX Error Warning\n");
        if (eflg & 0x08) printf("  - RX Error Warning\n");
        if (eflg & 0x10) printf("  - TX Passive\n");
        if (eflg & 0x20) printf("  - RX Passive\n");
        if (eflg & 0x40) printf("  - TX Bus-Off\n");
        if (eflg & 0x80) printf("  - RX Bus-Off\n");
    }
}



int main() {

    stdio_init_all();
    sleep_ms(10000);
    mcp2515_init();
    //mcp2515_enable_loopback();
    mcp2515_disable_loopback();
    sleep_ms(300);

    uint8_t stat = mcp2515_read_register(MCP_CANSTAT);
    printf("CANSTAT after disabling loopback: 0x%02X\n", stat);
    if ((stat & 0xE0) == 0x00) {
        printf("MCP2515 is in normal mode.\n");
    } else {
        printf("MCP2515 NOT in normal mode! Check wiring.\n");
    }

    //scan_motor_ids();  // Try CAN IDs

    send_rpm(104, 1000);

    print_can_errors();

    sleep_ms(1000);

    send_rpm(104, 0);

    // Start listening
    can_feedback_real();
    //can_receive_loop();

    

    return 0;
}

