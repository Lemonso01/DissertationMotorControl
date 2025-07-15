#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515.h"

// Motor CAN settings
#define MOTOR_ID   0x01  // CAN node ID

int main() {
    // Initialize USB serial
    stdio_init_all();
    sleep_ms(2000);  // wait for USB
    printf("Pico CAN RPM Ramp Demo\n");

    // Initialize MCP2515 CAN controller
    mcp2515_init();               // SPI setup + reset + CNF regs
    mcp2515_disable_loopback();   // NORMAL mode
    sleep_ms(100);

    // Step 1: run at 200 RPM for 5 seconds
    int32_t rpm = 200;
    printf("Setting RPM = %d for 5 seconds...\n", rpm);
    uint32_t t_start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - t_start < 5000) {
        send_rpm(MOTOR_ID, rpm);
        sleep_ms(100);  // send at 10 Hz
    }

    // Step 2: increase to 500 RPM thereafter
    rpm = 500;
    printf("Increasing RPM to %d indefinitely...\n", rpm);
    while (true) {
        send_rpm(MOTOR_ID, rpm);
        sleep_ms(100);
    }

    return 0;
}
