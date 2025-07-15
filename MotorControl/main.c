#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "mcp2515.h"

// Motor CAN settings
#define MOTOR_ID   0x01  // CAN node ID
#define CMD_BUFFER_SIZE      64

#define ENDSTOP_PIN_TOP      22
#define ENDSTOP_PIN_BOTTOM   21
#define ENDSTOP_PIN_LEFT     14
#define ENDSTOP_PIN_RIGHT    15

#define LOOP_DT_MS           100     // control loop period
#define POS_TOL_DEG          2.0f    // assist threshold
#define ASSIST_GAIN          0.5f    // Nm per degree error


// Read one feedback frame, extract position in degrees
static bool read_position_feedback(float *out_deg) {
    if (!mcp2515_check_message()) return false;
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    mcp2515_read_message(&id, data, &len);
    if (len < 2) return false;
    int16_t pos_raw = (int16_t)((data[0] << 8) | data[1]);
    *out_deg = pos_raw * 0.1f;  // LSB = 0.1°
    return true;
}

// Assist-as-needed trajectory: move from start->end in duration, assist if lag
static void do_assist_as_needed(float start_deg, float end_deg, float duration_s) {
    const int steps = (int)(duration_s * 1000 / LOOP_DT_MS);
    const float vel = (end_deg - start_deg) / (steps * (LOOP_DT_MS/1000.0f));
    float desired = start_deg;
    printf("Starting AAN from %.1f° to %.1f° over %.1fs (%d steps)\n",
           start_deg, end_deg, duration_s, steps);
    for (int i = 0; i < steps; i++) {
        uint64_t t0 = to_ms_since_boot(get_absolute_time());
        // send position setpoint
        send_position(MOTOR_ID, desired);
        // read feedback
        float actual;
        if (read_position_feedback(&actual)) {
            float err = desired - actual;
            if (err > POS_TOL_DEG) {
                float torque = err * ASSIST_GAIN;
                printf("  step %d, desired=%.1f°, actual=%.1f°, assist=%.2fNm\n",
                       i, desired, actual, torque);
                send_torque(MOTOR_ID, torque);
            }
        }
        // increment desired
        desired += vel;
        // wait remainder of loop
        uint64_t dt = to_ms_since_boot(get_absolute_time()) - t0;
        if (dt < LOOP_DT_MS) sleep_ms(LOOP_DT_MS - dt);
    }
    printf("Assit as Needed trajectory complete\n");
}

int main() {
    // Initialize USB serial
    stdio_init_all();
    sleep_ms(2000);  // wait for USB
    printf("Pico CAN\n");

    // Initialize MCP2515 CAN controller
    mcp2515_init();               // SPI setup + reset + CNF regs
    mcp2515_disable_loopback();   // NORMAL mode
    sleep_ms(100);

    char line[CMD_BUFFER_SIZE];
    while (true) {
        // Read a full line from USB serial
        if (!fgets(line, sizeof(line), stdin)) {
            continue;
        }

        // Parse command keyword and parameter
        char cmd[16];
        float v1, v2, v3;
        int args = sscanf(line, "%15s %f", cmd, &v1, &v2, &v3);
        if (args < 2) {
            printf("Invalid input. Use 'RPM', 'POS', or 'TORQUE' followed by a value.\n");
            continue;
        }

        // Dispatch commands
        if (strcmp(cmd, "RPM") == 0) {
            int32_t erpm = (int32_t)v1;
            printf("Setting RPM to %d\n", erpm);
            send_rpm(MOTOR_ID, erpm);
        } else if (strcmp(cmd, "POS") == 0) {
            printf("Moving to %.2f degrees\n", v1);
            send_position(MOTOR_ID, v1);
        } else if (strcmp(cmd, "RESISTANCE") == 0) {
            printf("Applying %.2f Nm torque\n", v1);
            send_torque(MOTOR_ID, v1);
        } else if (strcmp(cmd, "AAN") == 0 && args >= 4) {
            do_assist_as_needed(v1, v2, v3);
        } else {
            printf("Unknown command '%s'. Valid commands: RPM, POS, RESISTANCE.\n", cmd);
        }
    }

    return 0;  // never reached
}
