#include "pico/stdlib.h"
#include "mcp2515.h"

int main() {
    stdio_init_all();
    mcp2515_init();

    while (1) {
        //send_rpm(1, 1000); // send 1000 ERPM to motor ID 1
        send_position(0x01, 15.0f); // move by 15 radians
        sleep_ms(1000);
    }

    return 0;
}

