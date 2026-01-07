// === MCP2515 CAN Helper Functions (merged with full working implementation) ===
#include "mcp2515.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <string.h>

// === GPIO / SPI Interface ===
void mcp2515_select() {
    gpio_put(MCP2515_CS_PIN, 0);
}

void mcp2515_deselect() {
    gpio_put(MCP2515_CS_PIN, 1);
}

// === MCP2515 Core Functions ===
void mcp2515_reset() {
    mcp2515_select();
    uint8_t cmd = MCP_RESET;
    spi_write_blocking(spi0, &cmd, 1);
    mcp2515_deselect();
    sleep_ms(10);
}

void mcp2515_write_register(uint8_t reg, uint8_t value) {
    mcp2515_select();
    uint8_t buf[3] = {MCP_WRITE, reg, value};
    spi_write_blocking(spi0, buf, 3);
    mcp2515_deselect();
}

uint8_t mcp2515_read_register(uint8_t reg) {
    uint8_t cmd_buf[2] = {MCP_READ, reg};
    uint8_t value = 0;
    mcp2515_select();
    spi_write_blocking(spi0, cmd_buf, 2);
    spi_read_blocking(spi0, 0x00, &value, 1);
    mcp2515_deselect();
    return value;
}

void mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t data) {
    uint8_t cmd[] = {MCP_BITMOD, reg, mask, data};
    mcp2515_select();
    spi_write_blocking(spi0, cmd, 4);
    mcp2515_deselect();
}

uint8_t mcp2515_read_status() {
    uint8_t tx_buf[2] = {MCP_READ_STATUS, 0x00};
    uint8_t rx_buf[2] = {0};
    mcp2515_select();
    spi_write_read_blocking(spi0, tx_buf, rx_buf, 2);
    mcp2515_deselect();
    return rx_buf[1];
}

// === Initialization ===
void mcp2515_init() {
    gpio_init(MCP2515_CS_PIN);
    gpio_set_dir(MCP2515_CS_PIN, GPIO_OUT);
    gpio_put(MCP2515_CS_PIN, 1);

    spi_init(spi0, 1000 * 1000);
    gpio_set_function(16, GPIO_FUNC_SPI);
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);

    mcp2515_reset();
    sleep_ms(10);

    mcp2515_write_register(MCP_CANCTRL, 0x80);
    sleep_ms(10);
    mcp2515_write_register(MCP_CNF1, 0x00);
    mcp2515_write_register(MCP_CNF2, 0xD0);
    mcp2515_write_register(MCP_CNF3, 0x82);
    mcp2515_write_register(MCP_RXB0CTRL, 0x60);
    mcp2515_write_register(MCP_RXB1CTRL, 0x60);
    mcp2515_write_register(MCP_CANINTF, 0x00);


    mcp2515_write_register(MCP_CANCTRL, 0x00);
    uint8_t stat = mcp2515_read_register(MCP_CANSTAT);
    printf("CANSTAT after init: 0x%02X\n", stat);
}

// === CAN Mode Switching ===
void mcp2515_enable_loopback() {
    mcp2515_write_register(MCP_CANCTRL, 0x40);
    sleep_ms(10);
}

void mcp2515_disable_loopback() {
    mcp2515_write_register(MCP_CANCTRL, 0x00);
    sleep_ms(10);
}

// === CAN Send Functions ===
void mcp2515_send_extended(uint32_t id, const uint8_t *data, uint8_t len) {
    while (mcp2515_read_register(MCP_TXB0CTRL) & 0x08);
    uint8_t sidh = (uint8_t)(id >> 21);
    uint8_t sidl = (uint8_t)(((id >> 16) & 0x03) << 5) | (1 << 3) | ((id >> 18) & 0x07);
    uint8_t eid8 = (uint8_t)((id >> 8) & 0xFF);
    uint8_t eid0 = (uint8_t)(id & 0xFF);

    mcp2515_write_register(MCP_TXB0SIDH, sidh);
    mcp2515_write_register(MCP_TXB0SIDL, sidl);
    mcp2515_write_register(MCP_TXB0EID8, eid8);
    mcp2515_write_register(MCP_TXB0EID0, eid0);
    mcp2515_write_register(MCP_TXB0DLC, len);

    for (int i = 0; i < len; i++) {
        mcp2515_write_register(MCP_TXB0D0 + i, data[i]);
    }

    mcp2515_select();
    uint8_t rts = MCP_RTS_TX0;
    spi_write_blocking(spi0, &rts, 1);
    mcp2515_deselect();
}

void mcp2515_send_standard(uint16_t can_id, const uint8_t *data, uint8_t len) {
    while (mcp2515_read_register(MCP_TXB0CTRL) & 0x08);
    uint8_t sidh = (uint8_t)(can_id >> 3);
    uint8_t sidl = (uint8_t)((can_id & 0x07) << 5);
    mcp2515_write_register(MCP_TXB0SIDH, sidh);
    mcp2515_write_register(MCP_TXB0SIDL, sidl);
    mcp2515_write_register(MCP_TXB0DLC, len);
    for (int i = 0; i < len; i++) {
        mcp2515_write_register(MCP_TXB0D0 + i, data[i]);
    }
    mcp2515_select();
    uint8_t rts = MCP_RTS_TX0;
    spi_write_blocking(spi0, &rts, 1);
    mcp2515_deselect();
}

// === CAN Command Utilities ===
void send_rpm(uint8_t motor_id, int32_t rpm) {
    uint8_t data[4];
    data[0] = (rpm >> 24) & 0xFF;
    data[1] = (rpm >> 16) & 0xFF;
    data[2] = (rpm >> 8) & 0xFF;
    data[3] = rpm & 0xFF;
    uint32_t id = (CAN_PACKET_SET_RPM << 8) | motor_id;
    mcp2515_send_extended(id, data, 4);
    printf("Reaches end of RMP command with data: %f", data);
}

// Send position command in degrees
void send_position(uint8_t motor_id, float degrees) {
    // Convert degrees to the MCP2515 servo unit: 1 revolution = 10000 units
    // 1 deg = 10000/360 units
    int32_t pos_int = (int32_t)(degrees * (10000.0f / 360.0f));
    uint8_t data[4];
    data[0] = (pos_int >> 24) & 0xFF;
    data[1] = (pos_int >> 16) & 0xFF;
    data[2] = (pos_int >>  8) & 0xFF;
    data[3] =  pos_int        & 0xFF;
    uint32_t id = ((uint32_t)CAN_PACKET_SET_POS << 8) | motor_id;
    mcp2515_send_extended(id, data, 4);
}

// Send torque command as current in amps
void send_torque(uint8_t motor_id, float torque_nm) {
    // Convert torque [Nm] to current [A]: I = torque / Kt
    float current_a = torque_nm / MOTOR_TORQUE_CONSTANT;
    // Pack as integer (in mA)
    int32_t cur_int = (int32_t)(current_a * 1000.0f);
    uint8_t data[4];
    data[0] = (cur_int >> 24) & 0xFF;
    data[1] = (cur_int >> 16) & 0xFF;
    data[2] = (cur_int >>  8) & 0xFF;
    data[3] =  cur_int        & 0xFF;
    uint32_t id = ((uint32_t)CAN_PACKET_SET_CURRENT_LOOP << 8) | motor_id;
    mcp2515_send_extended(id, data, 4);
}


// === Message Handling ===
bool mcp2515_check_message() {
    uint8_t intf = mcp2515_read_register(MCP_CANINTF);
    return (intf & 0x03) != 0; // RX0IF or RX1IF
}

void mcp2515_clear_rx0if() {
    mcp2515_bit_modify(MCP_CANINTF, 0x01, 0x00);
}

void mcp2515_read_message(uint32_t *id, uint8_t *data, uint8_t *len) {
    uint8_t status = mcp2515_read_status();  // 0xA0 command
    uint8_t addr;

    if (status & 0x40) addr = MCP_RXB0SIDH;
    else if (status & 0x80) addr = MCP_RXB1SIDH;
    else {
      *len = 0;
      return;  // no message
    }

    uint8_t sidh = mcp2515_read_register(addr+0);
    uint8_t sidl = mcp2515_read_register(addr+1);
    bool isExt = sidl & 0x08;   // IDE bit

    if (isExt) {
        // extended: 29-bit
        uint8_t eid8 = mcp2515_read_register(addr+2);
        uint8_t eid0 = mcp2515_read_register(addr+3);
        *id = ((uint32_t)sidh << 21)
            | ((uint32_t)(sidl & 0xE0) << 13)
            | ((uint32_t)(sidl & 0x03) << 16)
            | ((uint32_t)eid8 << 8)
            |  eid0;
        *len = mcp2515_read_register(addr+4) & 0x0F;
        for (int i = 0; i < *len; i++)
            data[i] = mcp2515_read_register(addr+5 + i);
    } else {
        // standard: 11-bit
        // ID is in SIDH:[7:0] + SIDL:[7:5]
        *id = ((uint32_t)sidh << 3)
            | ((sidl & 0xE0) >> 5);
        *len = mcp2515_read_register(addr+2) & 0x0F;  // DLC at offset 2 in std frame
        for (int i = 0; i < *len; i++)
            data[i] = mcp2515_read_register(addr+3 + i);
    }

    // clear the appropriate RXnIF
    if      (addr == MCP_RXB0SIDH) mcp2515_bit_modify(MCP_CANINTF, 0x01, 0);
    else if (addr == MCP_RXB1SIDH) mcp2515_bit_modify(MCP_CANINTF, 0x02, 0);
}

// === Utilities ===
uint32_t float_to_uint(float x, float x_min, float x_max, int bits) {
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    float span = x_max - x_min;
    return (uint32_t)((x - x_min) * ((float)((1u << bits) - 1u)) / span + 0.5f);
}

float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits) {
    float span  = x_max - x_min;
    float ratio = (float)x_int / (float)((1u << bits) - 1u);
    return ratio * span + x_min;
}

// === MIT Mode ===

void mcp2515_send_frame(const can_frame_t *frm) {
    if (frm->extended) {
        mcp2515_send_extended(frm->id, frm->data, frm->dlc);
    } else {
        // std_id must fit 11 bits
        mcp2515_send_standard((uint16_t)frm->id, frm->data, frm->dlc);
    }
}

bool mcp2515_receive_frame(can_frame_t *frm) {
    uint8_t status = mcp2515_read_status();
    uint8_t addr   = 0;
    if (status & 0x40) addr = MCP_RXB0SIDH;       // RXB0
    else if (status & 0x80) addr = MCP_RXB1SIDH;  // RXB1
    else return false;                            // no message

    uint8_t sidh = mcp2515_read_register(addr + 0);
    uint8_t sidl = mcp2515_read_register(addr + 1);
    bool    ext  = sidl & (1<<3);  // EXIDE bit

    if (!ext) {
        // Standard Frame ID = SIDH[7:0]<<3 | SIDL[7:5]
        frm->extended = false;
        frm->id       = ((uint16_t)sidh << 3) | (sidl >> 5);
    } else {
        // Extended frame: same as you had before
        uint8_t eid8 = mcp2515_read_register(addr + 2);
        uint8_t eid0 = mcp2515_read_register(addr + 3);
        frm->extended = true;
        frm->id = ((uint32_t)sidh << 21)
                | (((uint32_t)(sidl & 0xE0)) << 13)
                | (((uint32_t)(sidl & 0x03)) << 16)
                | ((uint32_t)eid8 << 8)
                | (uint32_t)eid0;
    }

    // DLC is always at offset +4
    frm->dlc = mcp2515_read_register(addr + 4) & 0x0F;
    for (int i = 0; i < frm->dlc; i++) {
        frm->data[i] = mcp2515_read_register(addr + 5 + i);
    }

    // clear the flag
    if (addr == MCP_RXB0SIDH) mcp2515_bit_modify(MCP_CANINTF, 0x01, 0);
    else                     mcp2515_bit_modify(MCP_CANINTF, 0x02, 0);

    return true;
}

// Pack a MIT-mode command (8 bytes) into a standard CAN frame
//   p_des: desired position (rad), v_des: desired speed (rad/s)
//   kp, kd: gains, t_ff: feed-forward torque (Nm)
void mit_pack_cmd(can_frame_t *frm, uint8_t drv_id, float p_des, float v_des, float kp, float kd, float t_ff) {

    // convert to unsigned ints
    int32_t p_int  = float_to_uint(p_des, -12.5f, 12.5f, 16);
    int32_t v_int  = float_to_uint(v_des,  -30.0f, 30.0f, 12);
    int32_t kp_int = float_to_uint(kp,      0.0f,500.0f, 12);
    int32_t kd_int = float_to_uint(kd,      0.0f,  5.0f, 12);
    int32_t t_int  = float_to_uint(t_ff,   -18.0f,18.0f, 12);

    uint8_t *d = frm->data;
    // data[0..1] = p_int (16 bits)
    d[0] = (p_int >> 8) & 0xFF;
    d[1] =  p_int       & 0xFF;
    // data[2..3] = v_int (12 bits => high 8 in d2, low4 in d3[7:4])
    d[2] = (v_int >> 4) & 0xFF;
    d[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF);
    // data[4] = kp_int low 8 bits
    d[4] = kp_int & 0xFF;
    // data[5] = kd_int high 8 bits
    d[5] = (kd_int >> 4) & 0xFF;
    // data[6] = (kd_int low4 <<4) | (t_int >>8 &0xF)
    d[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF);
    // data[7] = t_int low 8 bits
    d[7] = t_int & 0xFF;

    // fill out frame header
    frm->extended = false;          // standard ID
    frm->dlc      = 8;
    frm->id       = drv_id;         // 11-bit ID = driver/node number
}

// Unpack a MIT-mode status reply from the motor
void mit_unpack_reply(const can_frame_t *frm, float *p, float *v, float *t, float *kd, int *rawTemp, int *err)
{
    uint8_t const *d = frm->data;
    int16_t p_int  = (d[0]<<8) | d[1];
    int16_t v_int  = (d[2]<<4) | (d[3]>>4);
    int16_t t_int  = ((d[3]&0xF)<<8) | d[6];
    int8_t  temp   = d[7];  // raw temperature
    uint8_t ecode  = d[6] & 0x0F; // low nibble of byte6

    // convert back to floats
    *p      = uint_to_float(p_int, -12.5f, 12.5f, 16);
    *v      = uint_to_float(v_int, -30.0f, 30.0f, 12);
    *t      = uint_to_float(t_int,-18.0f,18.0f, 12);
    *kd     = (*p); // or unused
    *rawTemp= temp;
    *err    = ecode;
}

bool mit_recv_reply(uint8_t drv_id,
                    float *p, float *v, float *t, float *kd,
                    int   *rawT, int *err) {
    can_frame_t rx;
    if (!mcp2515_receive_frame(&rx)) return false;
    if (rx.extended)                return false;
    if (rx.id != drv_id)            return false;
    if (rx.dlc != 8)                return false;
    mit_unpack_reply(&rx, p, v, t, kd, rawT, err);
    return true;
}

static void mcp2515_print_bus_health(void) {
    uint8_t eflg = mcp2515_read_register(MCP_EFLG);
    uint8_t tec  = mcp2515_read_register(MCP_TEC);
    uint8_t rec  = mcp2515_read_register(MCP_REC);
    uint8_t tx0  = mcp2515_read_register(MCP_TXB0CTRL);
    printf("EFLG=0x%02X TEC=%u REC=%u TXB0CTRL=0x%02X\n", eflg, tec, rec, tx0);
}

static inline void decode_servo_telem(const can_frame_t *rx, servo_telem_t *t)
{
    int16_t pos_i = (int16_t)((rx->data[0] << 8) | rx->data[1]);
    int16_t spd_i = (int16_t)((rx->data[2] << 8) | rx->data[3]);
    int16_t cur_i = (int16_t)((rx->data[4] << 8) | rx->data[5]);

    t->pos_deg    = pos_i * 0.1f;     // manual scaling
    t->speed_erpm = spd_i * 10.0f;    // manual scaling
    t->current_A  = cur_i * 0.01f;    // manual scaling
    t->temp_C     = (int8_t)rx->data[6];
    t->err        = rx->data[7];
}

static inline bool is_servo_telem(const can_frame_t *rx, uint8_t motor_id)
{
    if (!rx->extended) return false;        // servo telemetry is EID
    if (rx->dlc != 8)  return false;        // always 8 bytes
    if ((rx->id & 0xFFu) != motor_id) return false;

    // Optional (uncomment if you want to be strict like manufacturer example)
    // uint8_t type = (rx->id >> 8) & 0xFF;
    // if (type != 0x29) return false;

    return true;
}

void process_can_rx(uint8_t motor1_id, uint8_t motor2_id)
{
    can_frame_t rx;

    while (mcp2515_receive_frame(&rx)) {

        // Debug: print everything once (keep while testing)
        printf("%s ID=0x%lX DLC=%u DATA=",
               rx.extended ? "EID" : "SID",
               (unsigned long)rx.id,
               rx.dlc);
        for (int i = 0; i < rx.dlc; i++)
            printf("%02X ", rx.data[i]);
        printf("\n");

        servo_telem_t t;

        if (is_servo_telem(&rx, motor1_id)) {
            decode_servo_telem(&rx, &t);
            printf("[M1] pos=%.1f deg spd=%.0f eRPM cur=%.2f A temp=%d err=%u\n",
                   t.pos_deg, t.speed_erpm, t.current_A, t.temp_C, t.err);
        }

        if (is_servo_telem(&rx, motor2_id)) {
            decode_servo_telem(&rx, &t);
            printf("[M2] pos=%.1f deg spd=%.0f eRPM cur=%.2f A temp=%d err=%u\n",
                   t.pos_deg, t.speed_erpm, t.current_A, t.temp_C, t.err);
        }
    }
}