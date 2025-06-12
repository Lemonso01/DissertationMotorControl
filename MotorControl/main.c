#include "pico/stdlib.h"
#include "mcp2515.h"
#include <stdio.h>
#include <math.h>
#include <string.h>


#ifndef M_PI
  #define M_PI 3.14159265358979323846f
#endif
#define POS_TOL    0.01f      // ~0.6°
#define LOOP_DELAY 5          // 200 Hz update rate


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

void move_to(float p_target) {
    can_frame_t tx, rx;
    float p_act, v_act, t_act, kd_act;
    int rawT, err;

    for (int i = 0; i < 500; i++) {
        // 1) pack & send
        mit_pack_cmd(&tx,
                     MOTOR_ID,
                     p_target,
                     2.0f,   // v_des
                     50.0f,  // Kp
                     1.0f,   // Kd
                     0.0f    // torque ff
        );
        mcp2515_send_standard(tx.id, tx.data, tx.dlc);

        // 2) try to read reply
        if (mcp2515_receive_frame(&rx)) {
            mit_unpack_reply(&rx,
                             &p_act,
                             &v_act,
                             &t_act,
                             &kd_act,
                             &rawT,
                             &err
            );
            if (fabsf(p_act - p_target) < POS_TOL) {
                printf("→ reached %.3f rad (error=%.3f)\n",
                       p_target, p_act - p_target);
                return;
            }
        }

        sleep_ms(LOOP_DELAY);
    }
    printf("! move_to(%.3f) timed out\n", p_target);
}

bool read_line(char *buf, size_t bufmax) {
    static size_t idx = 0;
    while (1) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) break;
        if (c == '\r') continue;
        if (c == '\n' || idx >= bufmax-1) {
            buf[idx] = '\0';
            idx = 0;
            return true;
        }
        buf[idx++] = (char)c;
    }
    return false;
}


int main() {

    stdio_init_all();
    sleep_ms(10000);
    mcp2515_init();
    //mcp2515_enable_loopback();
    mcp2515_disable_loopback();
    sleep_ms(300);


    printf("\n--- Pico CAN console ready ---\n");

    char line[64];
    while (true) {
        printf("> "); fflush(stdout);
        // wait for a full line
        while (!read_line(line, sizeof(line)))
            tight_loop_contents();

        // HELP
        if (strcasecmp(line,"HELP")==0) {
            printf(
              "POS <deg>\n"
              "RPM <erpm>\n"
              "MITPING\n"
              "MITPOS <rad>\n"
              "SCAN\n"
              "EXIT\n"
            );
        }
        // SERVO position
        else if (strncmp(line,"POS ",4)==0) {
            float deg = atof(line+4);
            send_position(MOTOR_ID, deg);
            printf("→ servo POS %.1f°\n", deg);
        }
        // SERVO speed
        else if (strncmp(line,"RPM ",4)==0) {
            int erpm = atoi(line+4);
            send_rpm(MOTOR_ID, erpm);
            printf("→ servo RPM %d\n", erpm);
        }
        // MIT enter
        else if (strcasecmp(line,"MITPING")==0) {
            uint8_t ping[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
            mcp2515_send_standard(MOTOR_ID, ping, 8);
            printf("→ MIT-ENTER sent\n");
        }
        // MIT position
        else if (strncmp(line,"MITPOS ",7)==0) {
            float rad = atof(line+7);
            can_frame_t cmd;
            mit_pack_cmd(&cmd,
                         MOTOR_ID,
                         rad,      // pos [rad]
                         0.0f,     // v_des
                         100.0f,   // Kp
                         1.0f,     // Kd
                         0.0f);    // t_ff
            mcp2515_send_standard(cmd.id, cmd.data, cmd.dlc);
            printf("→ MIT POS %.3f rad sent\n", rad);
        }
        // ID scan
        else if (strcasecmp(line,"SCAN")==0) {
            for (uint8_t id=0; id<128; id++) {
                send_rpm(id,1000);
                sleep_ms(20);
            }
            printf("→ scan done\n");
        }
        // reboot to BOOTSEL
        else if (strcasecmp(line,"EXIT")==0) {
            watchdog_reboot(0,0,IRQ_CTRL_SHUTDOWN);
        }
        else {
            printf("Unknown: %s\n", line);
        }
    }

    /*ping_motor(MOTOR_ID);
    sleep_ms(50);

    can_frame_t cmd0;
    mit_pack_cmd(&cmd0,
                MOTOR_ID,
                0,   // 90° in radians
                0.0f,      // no speed offset
                2.0f,    // Kp
                1.0f,      // Kd
                0.0f       // torque feed-forward
    );
    mcp2515_send_standard(cmd0.id, cmd0.data, cmd0.dlc);
    printf("Sent MIT → 0° to ID %d\n", MOTOR_ID);

    float p,v,t, kd; int rawT, err;
    if (mit_recv_reply(MOTOR_ID, &p,&v,&t, &kd, &rawT, &err)) {
      printf("<< MIT REPLY  P=%.3f  v=%.3f  t=%.3f  rawT=%d  err=%d\n",
             p,v,t,rawT,err);
    } else {
      printf("<< No MIT reply\n");
    }

    sleep_ms(5000);

    can_frame_t cmd1;
    mit_pack_cmd(&cmd1,
                MOTOR_ID,
                +M_PI/2,   // 90° in radians
                0.0f,      // no speed offset
                2.0f,    // Kp
                1.0f,      // Kd
                0.0f       // torque feed-forward
    );
    mcp2515_send_standard(cmd1.id, cmd1.data, cmd1.dlc);
    printf("Sent MIT → +90° to ID %d\n", MOTOR_ID);

    if (mit_recv_reply(MOTOR_ID, &p,&v,&t, &kd, &rawT, &err)) {
      printf("<< MIT REPLY  P=%.3f  v=%.3f  t=%.3f  rawT=%d  err=%d\n",
             p,v,t,rawT,err);
    } else {
      printf("<< No MIT reply\n");
    }

    sleep_ms(5000);

    can_frame_t cmd2;
    mit_pack_cmd(&cmd2,
                MOTOR_ID,
                -M_PI/2,   // -90° in radians
                0.0f,      // no speed offset
                2.0f,    // Kp
                1.0f,      // Kd
                0.0f       // torque feed-forward
    );
    mcp2515_send_standard(cmd2.id, cmd2.data, cmd2.dlc);
    printf("Sent MIT → -90° to ID %d\n", MOTOR_ID);

    if (mit_recv_reply(MOTOR_ID, &p,&v,&t, &kd, &rawT, &err)) {
      printf("<< MIT REPLY  P=%.3f  v=%.3f  t=%.3f  rawT=%d  err=%d\n",
             p,v,t,rawT,err);
    } else {
      printf("<< No MIT reply\n");
    }
    

    sleep_ms(5000);

    while (1) tight_loop_contents();*/
}

