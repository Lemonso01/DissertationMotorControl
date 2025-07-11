#include <stdio.h>
#include "pico/stdlib.h"

#define BUTTON_PIN 22

bool read_line(char *buf, size_t bufmax) {
    static size_t idx = 0;
    while (true) {
        int c = getchar_timeout_us(0);  // non-blocking
        if (c == PICO_ERROR_TIMEOUT) break;
        if (c == '\r') continue;
        if (c == '\n' || idx >= bufmax - 1) {
            buf[idx] = '\0';
            idx = 0;
            return true;
        }
        buf[idx++] = (char)c;
    }
    return false;
}

int main() {
    stdio_init_all(); // Initialize USB CDC (serial)

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);  // enable internal pull-up

    char line[64];

    while (true) {
        sleep_ms(1000);  // check every 1 second

        // Check button (low = pressed)
        bool pressed = gpio_get(BUTTON_PIN) == 0;
        printf("Button on GPIO %d is %s\n", BUTTON_PIN, pressed ? "PRESSED" : "RELEASED");

        // Check serial input
        if (read_line(line, sizeof(line))) {
            printf("Serial said: %s\n", line);
        }
    }
}
