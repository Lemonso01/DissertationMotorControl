#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515.h"

// Motor CAN settings
#define MOTOR_ID   0x01  // CAN node ID
#define CMD_BUFFER_SIZE      64

#define ENDSTOP_PIN_TOP      22
#define ENDSTOP_PIN_BOTTOM   21
#define ENDSTOP_PIN_LEFT     14
#define ENDSTOP_PIN_RIGHT    15

int main() {
    // Initialize USB serial
    stdio_init_all();
    sleep_ms(2000);  // wait for USB
    printf("Pico CAN RPM Ramp Demo\n");

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
        float value;
        int args = sscanf(line, "%15s %f", cmd, &value);
        if (args < 2) {
            printf("Invalid input. Use 'RPM', 'POS', or 'TORQUE' followed by a value.\n");
            continue;
        }

        // Dispatch commands
        if (strcmp(cmd, "RPM") == 0) {
            int32_t erpm = (int32_t)value;
            printf("Setting RPM to %d\n", erpm);
            send_rpm(MOTOR_ID, erpm);
        } else if (strcmp(cmd, "POS") == 0) {
            printf("Moving to %.2f degrees\n", value);
            send_position(MOTOR_ID, value);
        } else if (strcmp(cmd, "RESISTANCE") == 0) {
            printf("Applying %.2f Nm torque\n", value);
            send_torque(MOTOR_ID, value);
        } else {
            printf("Unknown command '%s'. Valid commands: RPM, POS, RESISTANCE.\n", cmd);
        }
    }

    return 0;  // never reached
}
