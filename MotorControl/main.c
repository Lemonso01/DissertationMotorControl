#include "pico/stdlib.h"
#include "mcp2515.h"
#include <stdio.h>


// Error counter registers
#define MCP_TEC 0x1C
#define MCP_REC 0x1D

#define MOTOR_ID 1


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

static void ping_motor(uint8_t node_id) {
    uint8_t ping[8] = {
        0xFF,0xFF,0xFF,0xFF,
        0xFF,0xFF,0xFF,0xFC
    };
    mcp2515_send_standard(node_id, ping, 8);
    printf(">> MIT-ENTER ping sent to ID %u\n", node_id);
}



int main() {

    stdio_init_all();
    sleep_ms(10000);
    mcp2515_init();
    //mcp2515_enable_loopback();
    mcp2515_disable_loopback();
    sleep_ms(300);

    // prepare a frame
    can_frame_t tx = { .id = MOTOR_ID, .extended = false, .dlc = 8 };
    can_frame_t rx;
    float p, v, t, kd; int rawT, err;

    // enter control mode
    ping_motor(MOTOR_ID);   // use the MIT-ENTER ping from before
    sleep_ms(20);

    // now drive it in MIT-mode
    mit_pack_cmd(&tx, MOTOR_ID,
                 1.57f,    // desired pos = 90°
                 0.0f,     // desired vel
                 100.0f,   // Kp
                 1.0f,     // Kd
                 0.0f);    // torque ff
    mcp2515_send_standard(tx.id, tx.data, tx.dlc);
    printf("MIT CMD sent\n");

    // read back reply
    sleep_ms(10);
    if(mcp2515_check_message() && mcp2515_receive_frame(&rx)){
        mit_unpack_reply(&rx, &p, &v, &t, &kd, &rawT, &err);
        printf("MIT REPLY P=%.3f v=%.3f t=%.3f kd=%.3f Traw=%d err=%d\n",
               p, v, t, rawT, err);
    } else {
        printf("No MIT reply\n");
    }

    while(1) tight_loop_contents();
}

