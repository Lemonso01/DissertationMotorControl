#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "mcp2515.h"

// Motor CAN settings
#define MOTOR_ID_1   0x01  // CAN node ID
#define MOTOR_ID_2   0x02

#define CMD_BUFFER_SIZE      64

#define ENDSTOP_PIN_TOP      22
#define ENDSTOP_PIN_BOTTOM   21
#define ENDSTOP_PIN_LEFT     14
#define ENDSTOP_PIN_RIGHT    15

#define LOOP_DT_MS           500     // AAN control loop period
#define POS_TOL_DEG          2.0f
#define ASSIST_GAIN          0.5f

#define LOOP_MAIN_MS         10
#define DEBOUNCE_MS          50     // debounce time in ms

typedef enum {
    MODE_IDLE = 0,
    MODE_RPM,
    MODE_POS,
    MODE_TORQUE,
    MODE_POSSPD,
    MODE_AAN,
    MODE_ORIGIN
} CommandMode;

typedef struct {
    CommandMode mode;
    float      p1, p2, p3;
} CurrentCmd;

static uint pin_list[4] = { ENDSTOP_PIN_TOP, ENDSTOP_PIN_BOTTOM,
                            ENDSTOP_PIN_LEFT, ENDSTOP_PIN_RIGHT };
static bool    last_state[4];
static uint64_t last_time[4];

void init_endstops(void) {
    for (int i = 0; i < 4; ++i) {
        gpio_init(pin_list[i]);
        gpio_set_dir(pin_list[i], GPIO_IN);
        gpio_pull_up(pin_list[i]);
        last_state[i] = gpio_get(pin_list[i]);
        last_time[i]  = to_ms_since_boot(get_absolute_time());
    }
}

bool any_endstop_pressed_debounced(void) {
    uint64_t now = to_ms_since_boot(get_absolute_time());
    for (int i = 0; i < 4; ++i) {
        bool st = gpio_get(pin_list[i]);       // 1 = open, 0 = pressed
        if (st != last_state[i]) {
            last_state[i] = st;
            last_time[i]  = now;
        } else if (!st && (now - last_time[i] >= DEBOUNCE_MS)) {

            return true;
        }
    }
    return false;
}

// Read one feedback frame, extract position in degrees
bool read_position_feedback(float *out_deg) {
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

// Assist-as-needed trajectory
static void do_assist_as_needed(float start_deg, float end_deg, float duration_s) {
    const int steps = (int)(duration_s * 1000 / LOOP_DT_MS);
    const float vel = (end_deg - start_deg) / (steps * (LOOP_DT_MS/1000.0f));
    float desired = start_deg;
    printf("Starting AAN from %.1f° to %.1f° over %.1fs (%d steps)",
           start_deg, end_deg, duration_s, steps);
    for (int i = 0; i < steps; i++) {
        uint64_t t0 = to_ms_since_boot(get_absolute_time());
        // send position setpoint
        send_position(MOTOR_ID_1, desired);
        // read feedback
        float actual;
        if (read_position_feedback(&actual)) {
            float err = desired - actual;
            if (err > POS_TOL_DEG) {
                // compute assist speed [rpm] based on error
                float speed = err * ASSIST_GAIN;
                if (speed > 6.0f) speed = 6.0f;
                printf("  step %d, desired=%.1f°, actual=%.1f°, assist speed=%.2f rpm",
                       i, desired, actual, speed);
                send_rpm(MOTOR_ID_1, (int32_t)speed);
            }
        }
        // increment desired trajectory
        desired += vel;
        // wait remainder of loop
        uint64_t dt = to_ms_since_boot(get_absolute_time()) - t0;
        if (dt < LOOP_DT_MS) sleep_ms(LOOP_DT_MS - dt);
    }
    printf("Assist-as-needed trajectory complete");
}

void send_pos_spd(uint8_t motor_id, float pos_deg, float vel_dps, float acc_dps2) {
    // Build extended CAN ID
    uint32_t can_id = ((uint32_t)CAN_PACKET_SET_POSRPM << 8) | motor_id;
    uint8_t buf[8];
    // Position scaled: LSB = 0.0001 deg (i.e., pos_deg*10000)
    int32_t p = (int32_t)(pos_deg * 10000.0f);
    buf[0] = (p >> 24) & 0xFF;
    buf[1] = (p >> 16) & 0xFF;
    buf[2] = (p >>  8) & 0xFF;
    buf[3] = (p      ) & 0xFF;
    // Velocity scaled: LSB = 1 deg/s
    int16_t v = (int16_t)vel_dps;
    buf[4] = (v >> 8) & 0xFF;
    buf[5] = (v     ) & 0xFF;
    // Acceleration scaled: LSB = 1 deg/s^2
    int16_t a = (int16_t)acc_dps2;
    buf[6] = (a >> 8) & 0xFF;
    buf[7] = (a     ) & 0xFF;
    // Transmit
    mcp2515_send_extended(can_id, buf, 8);
    printf("POSSPD: %.2f° @%.2f°/s accel=%.2f°/s²\n", pos_deg, vel_dps, acc_dps2);
}

void send_origin(uint8_t motor_id) {
    uint32_t can_id = ((uint32_t)CAN_PACKET_SET_ORIGIN << 8) | motor_id;
    // No data payload
    mcp2515_send_extended(can_id, NULL, 0);
    printf("Origin set (current position = zero)\n");
}

void stop_motor(void) {
    // zero velocity
    send_rpm(MOTOR_ID_1, 0);
    // clear torque
    send_torque(MOTOR_ID_1, 0.0f);
    printf("All commands zeroed\n");
}

static void process_current_cmd(const CurrentCmd *cmd){
    switch(cmd->mode){
        case MODE_RPM:
            send_rpm(MOTOR_ID_1,(int32_t)cmd->p1);
            break;
        case MODE_POS:
            send_position(MOTOR_ID_1,cmd->p1);
            break;
        case MODE_TORQUE:
            send_torque(MOTOR_ID_1,cmd->p1);
            break;
        case MODE_POSSPD:
            send_pos_spd(MOTOR_ID_1,cmd->p1,cmd->p2,cmd->p3);
            break;
        case MODE_ORIGIN:
            send_origin(MOTOR_ID_1);
            break;
        default: break;
    }
}

// Read line from USB
static bool read_line(char *buf,size_t bufmax){
    static size_t idx=0;
    while(1){int c=getchar_timeout_us(0);
        if(c==PICO_ERROR_TIMEOUT)break;
        if(c=='\r')continue;
        if(c=='\n'||idx>=bufmax-1){buf[idx]=0;idx=0;return true;}
        buf[idx++]=(char)c;
    }
    return false;
}

int main() {
    // Initialize USB serial
    stdio_init_all();
    sleep_ms(2000);  // wait for USB
    printf("Pico CAN\n");

    gpio_init(ENDSTOP_PIN_TOP);    gpio_set_dir(ENDSTOP_PIN_TOP, GPIO_IN);    gpio_pull_up(ENDSTOP_PIN_TOP);
    gpio_init(ENDSTOP_PIN_BOTTOM); gpio_set_dir(ENDSTOP_PIN_BOTTOM, GPIO_IN); gpio_pull_up(ENDSTOP_PIN_BOTTOM);
    gpio_init(ENDSTOP_PIN_LEFT);   gpio_set_dir(ENDSTOP_PIN_LEFT, GPIO_IN);   gpio_pull_up(ENDSTOP_PIN_LEFT);
    gpio_init(ENDSTOP_PIN_RIGHT);  gpio_set_dir(ENDSTOP_PIN_RIGHT, GPIO_IN);  gpio_pull_up(ENDSTOP_PIN_RIGHT);


    // Initialize MCP2515 CAN controller
    mcp2515_init();               // SPI setup + reset + CNF regs
    mcp2515_disable_loopback();   // NORMAL mode
    sleep_ms(100);

    int cnt_top=0, cnt_bottom=0, cnt_left=0, cnt_right=0;

    CurrentCmd cur={MODE_IDLE,0,0,0};
    char line[CMD_BUFFER_SIZE],cmd[16];
    float v1,v2,v3; int args;

    while(1){

        if (any_endstop_pressed_debounced()) {
            printf("Endstop! Stopping.\n");
            cur.mode = MODE_IDLE;
            stop_motor();
        }

        // read incoming
        if(read_line(line,sizeof(line))){
            args=sscanf(line,"%15s %f %f %f",cmd,&v1,&v2,&v3);

            if(strcasecmp(cmd,"STOP")==0){ cur.mode=MODE_IDLE; stop_motor(); }

            else if(strcasecmp(cmd,"AUTO_MOVE")==0&&args>=4){ cur.mode=MODE_POSSPD; cur.p1=v1;cur.p2=v2;cur.p3=v3; }

            else if(strcasecmp(cmd,"RPM")==0&&args>=2){ cur.mode=MODE_RPM; cur.p1=v1; }

            else if(strcasecmp(cmd,"POS")==0&&args>=2){ cur.mode=MODE_POS; cur.p1=v1; }

            else if(strcasecmp(cmd,"TORQUE")==0&&args>=2){ cur.mode=MODE_TORQUE; cur.p1=v1; }

            else if(strcasecmp(cmd,"POSSPD")==0&&args>=4){ cur.mode=MODE_POSSPD; cur.p1=v1;cur.p2=v2;cur.p3=v3; }

            else if(strcasecmp(cmd,"AAN")==0&&args>=4){ cur.mode=MODE_IDLE; do_assist_as_needed(v1,v2,v3);}   

            else if(strcasecmp(cmd,"ORIGIN")==0){ cur.mode=MODE_ORIGIN; }
        }
        // maintain active command
        if(cur.mode!=MODE_IDLE){ process_current_cmd(&cur); }
        sleep_ms(LOOP_MAIN_MS);
    }
    return 0;
}
