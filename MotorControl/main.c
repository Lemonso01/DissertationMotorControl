#include "pico/stdlib.h"
#include "mcp2515.h"
#include <stdio.h>

void can_receive_loop() {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;

    while (true) {
        if (mcp2515_check_message()) {
            mcp2515_read_message(&id, data, &len);

            printf("✅ Received CAN message ID: 0x%03X, len: %d\n", id, len);
            for (int i = 0; i < len; i++) {
                printf("%02X ", data[i]);
            }
            printf("\n");

            sleep_ms(1000);  // Wait before reading again
        } else {
            printf("Waiting for message...\n");
            sleep_ms(500);
        }
    }
}

int main() {

    stdio_init_all();
    sleep_ms(10000);
    mcp2515_init();
    mcp2515_enable_loopback();
    printf("Loopback mode ON\n");

    send_position(0x78, 15.0f);
    printf("POS command sent\n");

    can_receive_loop();
    

    return 0;
}

