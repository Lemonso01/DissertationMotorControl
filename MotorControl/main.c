#include "pico/stdlib.h"
#include "mcp2515.h"
#include <stdio.h>


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

void send_mit_command(uint8_t motor_id, float position, float velocity, float kp, float kd, float torque) {
    // Clamp and convert to unsigned ints
    int p_int = float_to_uint(position, -12.5f, 12.5f, 16);
    int v_int = float_to_uint(velocity, -50.0f, 50.0f, 12);
    int kp_int = float_to_uint(kp, 0.0f, 500.0f, 12);
    int kd_int = float_to_uint(kd, 0.0f, 5.0f, 12);
    int t_int  = float_to_uint(torque, -18.0f, 18.0f, 12);

    uint8_t data[8];
    data[0] = (p_int >> 8) & 0xFF;
    data[1] = p_int & 0xFF;
    data[2] = (v_int >> 4) & 0xFF;
    data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF);
    data[4] = kp_int & 0xFF;
    data[5] = (kd_int >> 4) & 0xFF;
    data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF);
    data[7] = t_int & 0xFF;

    uint32_t can_id = motor_id;  // Standard ID in MIT mode
    mcp2515_send_extended(can_id, data, 8);
}

void scan_mit_ids() {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;

    printf("Starting MIT Mode ID scan...\n");

    for (uint8_t motor_id = 0; motor_id <= 127; motor_id++) {
        printf("Trying ID: %3d (0x%02X)\n", motor_id, motor_id);

        // Step 1: Enter MIT mode
        uint8_t enter_mit[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        mcp2515_send_extended(motor_id, enter_mit, 8);
        sleep_ms(100);

        // Step 2: Send MIT control command (e.g., velocity)
        send_mit_command(motor_id, 0.0f, 2.0f, 5.0f, 0.1f, 0.0f);
        sleep_ms(200);

        // Step 3: Wait for response
        if (mcp2515_check_message()) {
            mcp2515_read_message(&id, data, &len);

            if (len == 8) {
                int received_id = data[0];

                int p_int = (data[1] << 8) | data[2];
                int v_int = (data[3] << 4) | (data[4] >> 4);
                int t_int = ((data[4] & 0xF) << 8) | data[5];

                float position = ((float)p_int) * 25.0f / 65535.0f - 12.5f;
                float velocity = ((float)v_int) * 100.0f / 4095.0f - 50.0f;
                float torque   = ((float)t_int) * 36.0f / 4095.0f - 18.0f;

                int temperature = (int)data[6] - 40;
                uint8_t error_flags = data[7];

                printf("Found MIT Motor ID: %d (0x%02X)\n", motor_id, motor_id);
                printf("Pos: %.2f rad | Vel: %.2f rad/s | Tq: %.2f Nm | Temp: %d°C | Err: 0x%02X\n",
                       position, velocity, torque, temperature, error_flags);

                return;  // Stop on first match
            }
        }
    }

    printf("Scan complete. No MIT response detected.\n");
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
        if (mcp2515_check_message()) {
            mcp2515_read_message(&id, data, &len);

            // 🔍 DEBUG: Print raw message
            printf("Raw CAN ID: 0x%03X, Length: %d, Data: ", id, len);
            for (int i = 0; i < len; i++) {
                printf("%02X ", data[i]);
            }
            printf("\n");

            // Try to decode only if length matches CubeMars format
            if (len == 8) {
                int16_t pos_raw = (data[0] << 8) | data[1];
                int16_t spd_raw = (data[2] << 8) | data[3];
                int16_t cur_raw = (data[4] << 8) | data[5];
                int8_t temperature = (int8_t)data[6];
                uint8_t error_code = data[7];

                float position_deg = pos_raw * 0.1f;
                float speed_erpm = spd_raw * 10.0f;
                float current_a = cur_raw * 0.01f;

                printf("DECODED: POS: %.1f°, SPD: %.1f ERPM, CUR: %.2f A, TEMP: %d°C, ERR: %u\n",
                       position_deg, speed_erpm, current_a, temperature, error_code);
            } else {
                printf("Skipped decode. Expected 8 bytes, got %d.\n", len);
            }
        } else {
            printf("...Waiting for feedback\n");
        }

        sleep_ms(500);
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


int main() {

    stdio_init_all();
    sleep_ms(10000);
    mcp2515_init();
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

    send_rpm(0x0A, 1000);

    sleep_ms(10000);

    send_rpm(0x0A, 0);


    // Start listening
    //can_receive_loop();

    can_feedback_real();

    return 0;
}

