/**
 * cubemars_can_pico.c
 *
 * Raspberry Pi Pico 2 + MCP2515
 * CubeMars AK-series CAN test (Servo + MIT)
 *
 * Based directly on CubeMars Arduino reference code
 * and AK Series Module Driver User Manual v1.0.15.X
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

uint8_t motor1_id = 1, motor2_id = 2;


/* ============================================================
 * SWITCH END STOP DEFENITIONS
 * ============================================================ */

#define WRIST_STOP_LEFT     15
#define WRIST_STOP_RIGHT    14
#define FOREARM_STOP_TOP    13
#define FOREARM_STOP_BOTTOM 12


/* ============================================================
 * MCP2515 DEFINITIONS
 * ============================================================ */

#define MCP2515_SPI        spi0
#define MCP2515_CS_PIN     17
#define MCP2515_SCK_PIN    18
#define MCP2515_MOSI_PIN   19
#define MCP2515_MISO_PIN   16

#define MCP2515_RESET       0xC0
#define MCP2515_READ        0x03
#define MCP2515_WRITE       0x02
#define MCP2515_RTS_TX0     0x81
#define MCP2515_RTS_TX1     0x82
#define MCP2515_READ_STATUS 0xA0

// TX buffer base addresses (register maps)
#define TXB0CTRL   0x30
#define TXB0SIDH   0x31

#define TXB1CTRL   0x40
#define TXB1SIDH   0x41

#define TXREQ_BIT  (1u << 3)

/* MCP2515 registers (minimal set) */
#define CANCTRL  0x0F
#define CNF1     0x2A
#define CNF2     0x29
#define CNF3     0x28

/* ============================================================
 * CAN FRAME STRUCTURE
 * ============================================================ */

typedef struct {
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  data[8];
    bool     extended;
} can_frame_t;

/* ============================================================
 * SPI LOW LEVEL
 * ============================================================ */

static inline void mcp2515_select(void) {
    gpio_put(MCP2515_CS_PIN, 0);
}

static inline void mcp2515_deselect(void) {
    gpio_put(MCP2515_CS_PIN, 1);
}

static void mcp2515_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[3] = { MCP2515_WRITE, reg, val };
    mcp2515_select();
    spi_write_blocking(MCP2515_SPI, buf, 3);
    mcp2515_deselect();
}

static uint8_t mcp2515_read_reg(uint8_t reg) {
    uint8_t tx[3] = { MCP2515_READ, reg, 0x00 };
    uint8_t rx[3];
    mcp2515_select();
    spi_write_read_blocking(MCP2515_SPI, tx, rx, 3);
    mcp2515_deselect();
    return rx[2];
}

static void mcp2515_reset(void) {
    uint8_t cmd = MCP2515_RESET;
    mcp2515_select();
    spi_write_blocking(MCP2515_SPI, &cmd, 1);
    mcp2515_deselect();
    sleep_ms(10);
}

/* ============================================================
 * MCP2515 INIT (1 Mbps, 8 MHz)
 * ============================================================ */

static void mcp2515_init(void) {
    mcp2515_reset();

    /* Configuration mode */
    mcp2515_write_reg(CANCTRL, 0x80);

    /* 1 Mbps @ 16 MHz (CubeMars recommended) */
    mcp2515_write_reg(CNF1, 0x00);   // SJW=1, BRP=0
    mcp2515_write_reg(CNF2, 0xD1);   // BTLMODE=1, PHSEG1=3, PRSEG=0
    mcp2515_write_reg(CNF3, 0x81);   // PHSEG2=3

    /* Normal mode */
    mcp2515_write_reg(CANCTRL, 0x00);
}

/* ============================================================
 * CAN TRANSMISSION
 * ============================================================ */

static void can_send(const can_frame_t *frm) {
    uint8_t buf[14];
    uint8_t i = 0;

    buf[i++] = MCP2515_WRITE;
    buf[i++] = 0x31; // TXB0SIDH

    if (frm->extended) {
        uint32_t id = frm->can_id;
        buf[i++] = (id >> 21) & 0xFF;
        buf[i++] = ((id >> 13) & 0xE0) | 0x08 | ((id >> 16) & 0x03);
        buf[i++] = (id >> 8) & 0xFF;
        buf[i++] = id & 0xFF;
    } else {
        buf[i++] = (frm->can_id >> 3) & 0xFF;
        buf[i++] = (frm->can_id & 0x07) << 5;
        buf[i++] = 0;
        buf[i++] = 0;
    }

    buf[i++] = frm->can_dlc & 0x0F;
    memcpy(&buf[i], frm->data, frm->can_dlc);
    i += frm->can_dlc;

    mcp2515_select();
    spi_write_blocking(MCP2515_SPI, buf, i);
    mcp2515_deselect();

    uint8_t rts = MCP2515_RTS_TX0;
    mcp2515_select();
    spi_write_blocking(MCP2515_SPI, &rts, 1);
    mcp2515_deselect();
}

static bool mcp2515_wait_txb_free(uint8_t txb_ctrl_reg, uint32_t timeout_us)
{
    absolute_time_t t_end = make_timeout_time_us(timeout_us);

    while (mcp2515_read_reg(txb_ctrl_reg) & TXREQ_BIT) {
        if (absolute_time_diff_us(get_absolute_time(), t_end) <= 0) {
            return false; // timed out
        }
        tight_loop_contents();
    }
    return true;
}

static bool can_send_on_txb(uint8_t txb_index, const can_frame_t *frm)
{
    uint8_t sidh_addr, ctrl_addr, rts_cmd;

    if (txb_index == 0) {
        ctrl_addr = TXB0CTRL;
        sidh_addr = TXB0SIDH;
        rts_cmd   = MCP2515_RTS_TX0;
    } else if (txb_index == 1) {
        ctrl_addr = TXB1CTRL;
        sidh_addr = TXB1SIDH;
        rts_cmd   = MCP2515_RTS_TX1;
    } else {
        return false;
    }

    // Wait until this TX buffer is free
    if (!mcp2515_wait_txb_free(ctrl_addr, 2000)) { // 2 ms timeout
        return false;
    }

    // Build write sequence: WRITE, start_reg, then ID regs + DLC + data
    uint8_t buf[2 + 4 + 1 + 8]; // cmd+addr + 4 ID bytes + DLC + up to 8 data
    uint8_t i = 0;

    buf[i++] = MCP2515_WRITE;
    buf[i++] = sidh_addr; // start at SIDH for selected TX buffer

    if (frm->extended) {
        // Extended 29-bit ID -> SIDH/SIDL/EID8/EID0
        uint32_t id = frm->can_id;
        buf[i++] = (id >> 21) & 0xFF;
        buf[i++] = ((id >> 13) & 0xE0) | 0x08 | ((id >> 16) & 0x03); // EXIDE=1
        buf[i++] = (id >> 8) & 0xFF;
        buf[i++] = id & 0xFF;
    } else {
        // Standard 11-bit ID
        buf[i++] = (frm->can_id >> 3) & 0xFF;
        buf[i++] = (frm->can_id & 0x07) << 5;
        buf[i++] = 0x00;
        buf[i++] = 0x00;
    }

    buf[i++] = frm->can_dlc & 0x0F;

    uint8_t dlc = frm->can_dlc;
    if (dlc > 8) dlc = 8;
    memcpy(&buf[i], frm->data, dlc);
    i += dlc;

    // Write TXBnSIDH..TXBnDLC..TXBnDATA
    mcp2515_select();
    spi_write_blocking(MCP2515_SPI, buf, i);
    mcp2515_deselect();

    // Request to send on that buffer
    mcp2515_select();
    spi_write_blocking(MCP2515_SPI, &rts_cmd, 1);
    mcp2515_deselect();

    return true;
}

static inline uint8_t txb_for_motor(uint8_t motor_id)
{
    // Map motor IDs to TX buffers deterministically
    // motor 1 -> TXB0, motor 2 -> TXB1
    // Adjust if your IDs differ.
    return (motor_id == 1) ? 0 : 1;
}

/* ============================================================
 * BUFFER HELPERS (IDENTICAL TO MANUAL)
 * ============================================================ */

static void buffer_append_int16(uint8_t *b, int16_t n, int32_t *i) {
    b[(*i)++] = n >> 8;
    b[(*i)++] = n;
}

static void buffer_append_int32(uint8_t *b, int32_t n, int32_t *i) {
    b[(*i)++] = n >> 24;
    b[(*i)++] = n >> 16;
    b[(*i)++] = n >> 8;
    b[(*i)++] = n;
}

/* ============================================================
 * SERVO MODE COMMANDS
 * ============================================================ */

typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_ORIGIN_HERE,
    CAN_PACKET_SET_POS_SPD
} CAN_PACKET_ID;

typedef enum {
    UI_CMD_NONE = 0,
    UI_CMD_SPD,
    UI_CMD_POS,
    UI_CMD_PSA,
    UI_CMD_ORG,
    UI_CMD_STOP,
    UI_CMD_STOPALL,
    UI_CMD_TRQ,
    UI_CMD_AAN,
    UI_CMD_BRK
} ui_cmd_type_t;

typedef struct {
    ui_cmd_type_t type;
    uint8_t motor_id;

    float   pos_deg;
    int16_t spd_erpm;
    int16_t acc_erpm_s2;
    uint8_t origin_mode;
    float current_a;
    float current_brk;

    float aan_start_deg;
    float aan_end_deg;
    float aan_rpm;
} ui_cmd_t;

typedef enum {
    MCTL_NONE = 0,
    MCTL_SPD,
    MCTL_POS,
    MCTL_PSA,
    MCTL_TRQ,
    MCTL_STOP,
    MCTL_IDLE,
    MCTL_BRK
} motor_ctl_mode_t;

typedef struct {
    motor_ctl_mode_t mode;
    float   pos_deg;
    int16_t spd_erpm;
    int16_t acc_erpm_s2;
    float   cur_a;
    absolute_time_t last_ui_update;
} motor_ctl_t;

static motor_ctl_t g_m1 = { .mode = MCTL_IDLE };
static motor_ctl_t g_m2 = { .mode = MCTL_IDLE };

typedef struct {
    bool            valid;
    float           pos_deg;
    float           spd_erpm;
    float           cur_a;
    absolute_time_t t_last;
} motor_fb_t;

static motor_fb_t g_fb1 = {0};
static motor_fb_t g_fb2 = {0};

static inline motor_fb_t *fb_for_id(uint8_t id)
{
    if (id == motor1_id) return &g_fb1;
    if (id == motor2_id) return &g_fb2;
    return NULL;
}

typedef struct {
    bool            enabled;        // AAN is running
    bool            assisting;      // assistance already triggered
    float           start_deg;
    float           end_deg;
    float           duration_s;
    absolute_time_t t0;

    // Assist ramp (slowly increase help)
    float           assist_u;       // 0..1 assistance blending factor
    float           assist_spd_erpm;
} aan_state_t;

static aan_state_t g_aan1 = {0};
static aan_state_t g_aan2 = {0};

static inline aan_state_t *aan_for_id(uint8_t id)
{
    if (id == motor1_id) return &g_aan1;
    if (id == motor2_id) return &g_aan2;
    return NULL;
}

#define AAN_DELAY_S            5.0f
#define AAN_POS_TOL_DEG        1.0f
#define AAN_ASSIST_SPD_ERPM    300
#define AAN_ASSIST_ACC_ERPM_S2 3000

// How fast assistance ramps up once triggered (per second)
#define AAN_ASSIST_RAMP_RATE   0.25f   // 0->1 in ~4s

#define POLEPAIR_1 21
#define POLEPAIR_2 14
#define MOTOR1_RATIO 10.f
#define MOTOR2_RATIO 6.0f

#define AAN_RPM_MAX   30.0f
#define AAN_RPM_MIN   0.1f

static inline float joint_rpm_to_erpm(uint8_t motor_id, float joint_rpm)
{
    float pole_pairs = 0.0f;
    float gear       = 1.0f;

    if (motor_id == motor1_id) { pole_pairs = POLEPAIR_1; gear = MOTOR1_RATIO; }
    else if (motor_id == motor2_id) { pole_pairs = POLEPAIR_2; gear = MOTOR2_RATIO; }
    else return 0.0f;

    // ERPM = joint_rpm * gear_ratio * pole_pairs
    return joint_rpm * gear * pole_pairs;
}

static inline float erpm_to_joint_rpm(uint8_t motor_id, float erpm)
{
    float pole_pairs = 0.0f;
    float gear       = 1.0f;

    if (motor_id == motor1_id) { pole_pairs = POLEPAIR_1; gear = MOTOR1_RATIO; }
    else if (motor_id == motor2_id) { pole_pairs = POLEPAIR_2; gear = MOTOR2_RATIO; }
    else return 0.0f;

    // joint_rpm = ERPM / (pole_pairs * gear_ratio)
    return erpm / (pole_pairs * gear);
}

static inline float duration_by_rpm(float start_deg, float end_deg, float rpm) {
    
    float delta_deg = fabsf(end_deg - start_deg);
    float revolutions = delta_deg / 360.0f;

    float rps = fabsf(rpm) / 60.0f;
    if (rps < 1e-6f) return 0.0f; // avoid divide by zero

    return revolutions / rps;

}

static inline uint32_t time_ms_now(void)
{
    return (uint32_t)(to_ms_since_boot(get_absolute_time()));
}

static inline float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

static inline float lerp_f(float a, float b, float u) {
    return a + (b - a) * u;
}

// Decide “5 seconds behind” without needing velocity estimation.
// Compare actual position to expected position at (t - 5s), direction-aware.
static inline bool behind_by_5s(float act, float exp_now, float exp_m5)
{
    float dir = exp_now - exp_m5; // sign indicates direction
    if (dir >= 0.0f) return act < exp_m5; // moving positive
    else             return act > exp_m5; // moving negative
}


static bool comm_can_transmit_eid_motor(uint8_t motor_id, uint32_t eid, const uint8_t *data, uint8_t len)
{
    can_frame_t f = {
        .can_id    = eid,
        .can_dlc   = len,
        .extended  = true
    };
    if (len > 8) len = 8;
    memcpy(f.data, data, len);

    return can_send_on_txb(txb_for_motor(motor_id), &f);
}

static void comm_can_set_pos(uint8_t motor_id, float pos_deg)
{
    uint8_t buf[4];
    int32_t idx = 0;

    buffer_append_int32(buf, (int32_t)(pos_deg * 10000.0f), &idx);

    uint32_t eid = motor_id | ((uint32_t)CAN_PACKET_SET_POS << 8);
    (void)comm_can_transmit_eid_motor(motor_id, eid, buf, (uint8_t)idx);
}


static void comm_can_set_spd(uint8_t motor_id, float erpm)
{
    uint8_t buffer[4];
    int32_t idx = 0;

    buffer_append_int32(buffer, (int32_t)erpm, &idx);

    uint32_t eid = motor_id | ((uint32_t)CAN_PACKET_SET_RPM << 8);
    (void)comm_can_transmit_eid_motor(motor_id, eid, buffer, (uint8_t)idx);
}

static inline float clamp_f(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void comm_can_set_current_brake(uint8_t motor_id, float brake_current_a)
{
    uint8_t buf[4];
    int32_t idx = 0;

    brake_current_a = clamp_f(brake_current_a, 0.0f, 10.0f);

    buffer_append_int32(buf, (int32_t)(brake_current_a * 1000.0f), &idx);

    uint32_t eid = motor_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
    (void)comm_can_transmit_eid_motor(motor_id, eid, buf, (uint8_t)idx);
}


static void comm_can_set_current(uint8_t motor_id, float current_a)
{
    uint8_t buf[4];
    int32_t idx = 0;

    current_a = clamp_f(current_a, -2.0f, 2.0f);

    buffer_append_int32(buf, (int32_t)(current_a * 1000.0f), &idx);

    uint32_t eid = motor_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
    (void)comm_can_transmit_eid_motor(motor_id, eid, buf, (uint8_t)idx);
}

void stop_motor(uint8_t motor_id) {

    comm_can_set_current(motor_id, 0.0f);
    comm_can_set_current_brake(motor_id, 0.0f);

}

static uint8_t mcp2515_read_status(void)
{
    uint8_t cmd = MCP2515_READ_STATUS;
    uint8_t status = 0;
    mcp2515_select();
    spi_write_blocking(MCP2515_SPI, &cmd, 1);
    spi_read_blocking(MCP2515_SPI, 0x00, &status, 1);
    mcp2515_deselect();
    return status;
}

static void mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t data)
{
    uint8_t buf[4] = { 0x05, reg, mask, data }; // BIT MODIFY
    mcp2515_select();
    spi_write_blocking(MCP2515_SPI, buf, 4);
    mcp2515_deselect();
}

/**
 * Reads one frame from MCP2515 (RXB0 or RXB1).
 * Returns true if a frame was read.
 */
static bool mcp2515_receive_frame(can_frame_t *frm)
{
    uint8_t status = mcp2515_read_status();
    uint8_t canintf = mcp2515_read_reg(0x2C);

    uint8_t addr;
    bool from_rxb0 = false;

    // RX0IF is indicated via status bits; common approach:
    // If either RX buffer has data, pick one.
    if (canintf & 0x01) {          // RX0IF
        addr = 0x61;              // RXB0SIDH register address
        from_rxb0 = true;
    } else if (canintf & 0x02) {   // RX1IF
        addr = 0x71;              // RXB1SIDH
        from_rxb0 = false;
    } else {
        return false;
    }

    uint8_t sidh = mcp2515_read_reg(addr + 0);
    uint8_t sidl = mcp2515_read_reg(addr + 1);
    uint8_t eid8 = mcp2515_read_reg(addr + 2);
    uint8_t eid0 = mcp2515_read_reg(addr + 3);
    uint8_t dlc  = mcp2515_read_reg(addr + 4) & 0x0F;

    frm->can_dlc = dlc;

    // EXIDE bit (bit 3 of SIDL) indicates extended frame
    bool ext = (sidl & (1u << 3)) != 0;
    frm->extended = ext;

    if (!ext) {
        // Standard 11-bit ID: SIDH[7:0]<<3 | SIDL[7:5]
        frm->can_id = ((uint32_t)sidh << 3) | (sidl >> 5);
    } else {
        // Extended 29-bit ID reconstruction:
        // SID = (SIDH<<3) | (SIDL>>5)
        // EID = (SIDL & 0x03)<<16 | (EID8<<8) | EID0
        uint32_t sid = ((uint32_t)sidh << 3) | (sidl >> 5);
        uint32_t eid = ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
        frm->can_id = (sid << 18) | eid;
    }

    for (uint8_t i = 0; i < dlc; i++) {
        frm->data[i] = mcp2515_read_reg(addr + 5 + i);
    }

    // Clear the corresponding interrupt flag in CANINTF
    // CANINTF register is 0x2C; RX0IF=0x01, RX1IF=0x02
    if (from_rxb0) {
        mcp2515_bit_modify(0x2C, 0x01, 0x00);
    } else {
        mcp2515_bit_modify(0x2C, 0x02, 0x00);
    }

    return true;
}

static void print_can_frame(const can_frame_t *f)
{
    printf("%s ID:0x%lX DLC:%u DATA:",
           f->extended ? "EID" : "SID",
           (unsigned long)f->can_id,
           f->can_dlc);

    for (uint8_t i = 0; i < f->can_dlc; i++) {
        printf(" %02X", f->data[i]);
    }
    printf("\n");
}

static void servo_decode_and_print(const can_frame_t *f)
{
    int8_t motor_id = (uint8_t)(f->can_id & 0xFF);

    int16_t pos_i = (int16_t)((f->data[0] << 8) | f->data[1]);
    int16_t spd_i = (int16_t)((f->data[2] << 8) | f->data[3]);
    int16_t cur_i = (int16_t)((f->data[4] << 8) | f->data[5]);

    float pos_deg  = pos_i * 0.1f;
    float spd_erpm = spd_i * 10.0f;
    float cur_a    = cur_i * 0.01f;

    int8_t  temp_c = (int8_t)f->data[6];
    uint8_t err    = f->data[7];

    uint32_t t_ms = time_ms_now();

    printf("%lu,SERVO,%u,%.1f,%.0f,%.2f,%d,%u\n", (unsigned long)t_ms, motor_id, pos_deg, spd_erpm, cur_a, temp_c, err);
}

static void comm_can_set_origin(uint8_t motor_id, uint8_t mode)
{
    // mode is typically:
    // 0 = temporary zero
    // 1 = save zero to flash (persistent)
    // 2 = clear saved zero (if supported by firmware)

    uint8_t buffer[1];
    buffer[0] = mode;

    uint32_t eid = motor_id | ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8);

    comm_can_transmit_eid_motor(motor_id, eid, buffer, 1);
}

static inline int16_t clamp_i16(int32_t x)
{
    if (x >  32767) return  32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

static inline int16_t clamp_i16_nonneg(int32_t x)
{
    if (x < 0)      return 0;
    if (x > 32767)  return 32767;
    return (int16_t)x;
}

/**
 * Servo Position–Speed–Acceleration command (CAN_PACKET_SET_POS_SPD = 6)
 *
 * motor_id: target motor CAN ID (e.g., 1 or 2)
 * pos_deg: desired position in degrees
 * spd_erpm: desired electrical RPM (ERPM)
 * acc_erpm_s2: desired acceleration in ERPM/s^2 (>=0)
 */
static void servo_set_pos_spd(uint8_t motor_id, float pos_deg, int16_t spd_erpm, int16_t acc_erpm_s2)
{
    uint8_t buf[8];
    int32_t idx = 0;

    // Position: int32 scaled by 10000
    buffer_append_int32(buf, (int32_t)(pos_deg * 10000.0f), &idx);

    // Speed and Accel are sent in units of 10 ERPM / 10 ERPM/s^2
    int16_t spd_tx = clamp_i16((int32_t)spd_erpm / 10);
    int16_t acc_tx = clamp_i16_nonneg((int32_t)acc_erpm_s2 / 10);

    buffer_append_int16(buf, spd_tx, &idx);
    buffer_append_int16(buf, acc_tx, &idx);

    uint32_t eid = motor_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8);

    // Reuse your existing TX routing (TXB0/TXB1)
    (void)comm_can_transmit_eid_motor(motor_id, eid, buf, (uint8_t)idx);
}

static void servo_decode_to_feedback(const can_frame_t *f)
{
    uint8_t motor_id = (uint8_t)(f->can_id & 0xFF);
    motor_fb_t *fb = fb_for_id(motor_id);
    if (!fb) return;
    if (f->can_dlc < 8) return;

    int16_t pos_i = (int16_t)((f->data[0] << 8) | f->data[1]);
    int16_t spd_i = (int16_t)((f->data[2] << 8) | f->data[3]);
    int16_t cur_i = (int16_t)((f->data[4] << 8) | f->data[5]);

    fb->pos_deg  = pos_i * 0.1f;
    fb->spd_erpm = spd_i * 10.0f;
    fb->cur_a    = cur_i * 0.01f;
    fb->t_last   = get_absolute_time();
    fb->valid    = true;
}

static void aan_eval_and_assist(uint8_t motor_id)
{
    aan_state_t  *aan = aan_for_id(motor_id);
    motor_fb_t   *fb  = fb_for_id(motor_id);
    if (!aan || !fb) return;
    if (!aan->enabled) return;
    if (!fb->valid) return;

    float t_s = (float)absolute_time_diff_us(aan->t0, get_absolute_time()) / 1e6f;
    if (t_s < 0.0f) t_s = 0.0f;

    float T = (aan->duration_s > 0.05f) ? aan->duration_s : 0.05f;

    float u_now = clamp01(t_s / T);
    float u_m5  = clamp01((t_s - AAN_DELAY_S) / T);

    float exp_now = lerp_f(aan->start_deg, aan->end_deg, u_now);
    float exp_m5  = lerp_f(aan->start_deg, aan->end_deg, u_m5);

    float act = fb->pos_deg;

    // Trigger assistance if behind by 5 seconds (only after 5 seconds elapsed)
    if (!aan->assisting) {
        if (t_s >= AAN_DELAY_S && behind_by_5s(act, exp_now, exp_m5)) {
            aan->assisting = true;
            aan->assist_u  = 0.0f; // start ramp at zero
            printf("AAN TRIGGER id=%u t=%.2f act=%.2f exp_m5=%.2f exp=%.2f\n",
                   motor_id, t_s, act, exp_m5, exp_now);
        }
    }

    if (aan->assisting) {
        // Ramp assistance smoothly over real time (no counters)
        static absolute_time_t last_call_1 = {0}, last_call_2 = {0};
        absolute_time_t *last_call = (motor_id == motor1_id) ? &last_call_1 : &last_call_2;

        float dt_s = 0.0f;
        if (to_us_since_boot(*last_call) != 0) {
            dt_s = (float)absolute_time_diff_us(*last_call, get_absolute_time()) / 1e6f;
            if (dt_s < 0.0f) dt_s = 0.0f;
            if (dt_s > 0.2f) dt_s = 0.2f; // guard against long pauses
        }
        *last_call = get_absolute_time();

        aan->assist_u = clamp01(aan->assist_u + AAN_ASSIST_RAMP_RATE * dt_s);

        // “Slowly assist”: don’t jump straight to end.
        // Move target gradually from current position toward end using assist_u.
        float target = lerp_f(act, aan->end_deg, aan->assist_u);

        servo_set_pos_spd(motor_id, target, AAN_ASSIST_SPD_ERPM, AAN_ASSIST_ACC_ERPM_S2);
    }

    // End conditions
    if (fabsf(aan->end_deg - act) <= AAN_POS_TOL_DEG || t_s > (T + AAN_DELAY_S + 2.0f)) {
        printf("AAN DONE id=%u act=%.2f end=%.2f\n", motor_id, act, aan->end_deg);
        aan->enabled = false;
        aan->assisting = false;
        aan->assist_u = 0.0f;
    }
}


static bool ui_read_line(char *out, size_t out_len)
{
    static char line[128];
    static size_t n = 0;

    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == '\r') continue;

        if (ch == '\n') {
            line[n] = '\0';
            strncpy(out, line, out_len);
            out[out_len - 1] = '\0';
            n = 0;
            return true;
        }

        if (n < sizeof(line) - 1) {
            line[n++] = (char)ch;
        } else {
            // overflow: reset buffer
            n = 0;
        }
    }

    return false;
}

static bool ui_parse_command(const char *line, ui_cmd_t *cmd)
{
    // initialize
    memset(cmd, 0, sizeof(*cmd));
    cmd->type = UI_CMD_NONE;

    // Trim leading spaces
    while (*line == ' ' || *line == '\t') line++;

    if (*line == '\0') return false;

    // STOPALL
    if (strncmp(line, "STOPALL", 7) == 0) {
        cmd->type = UI_CMD_STOPALL;
        return true;
    }

    // SPD <id> <erpm>
    unsigned id_u = 0;
    int spd_i = 0;
    if (sscanf(line, "RPM %u %d", &id_u, &spd_i) == 2) {
        cmd->type = UI_CMD_SPD;
        cmd->motor_id = (uint8_t)id_u;
        cmd->spd_erpm = (int16_t)spd_i;
        return true;
    }

    // POS <id> <deg>
    float pos_f = 0.0f;
    if (sscanf(line, "POS %u %f", &id_u, &pos_f) == 2) {
        cmd->type = UI_CMD_POS;
        cmd->motor_id = (uint8_t)id_u;
        cmd->pos_deg = pos_f;
        return true;
    }

    // PSA <id> <deg> <erpm> <acc>
    float acc_f = 0.00, spd_f = 0.00;
    if (sscanf(line, "PSA %u %f %f %f", &id_u, &pos_f, &spd_f, &acc_f) == 4) {
        cmd->type = UI_CMD_PSA;
        cmd->motor_id = (uint8_t)id_u;
        cmd->pos_deg = pos_f;
        cmd->spd_erpm = spd_f;
        cmd->acc_erpm_s2 = acc_f;
        return true;
    }

    // ORG <id> <mode>
    unsigned mode_u = 0;
    if (sscanf(line, "ORIGIN %u", &mode_u) == 1) {
        cmd->type = UI_CMD_ORG;
        cmd->origin_mode = (uint8_t)mode_u;
        return true;
    }

    // STOP <id>
    if (sscanf(line, "STOP %u", &id_u) == 1) {
        cmd->type = UI_CMD_STOP;
        cmd->motor_id = (uint8_t)id_u;
        return true;
    }

    // TRQ <id> <current_A>
    float cur_f = 0.0f;
    if (sscanf(line, "TORQUE %u %f", &id_u, &cur_f) == 2) {
        cmd->type = UI_CMD_TRQ;
        cmd->motor_id = (uint8_t)id_u;
        cmd->current_a = cur_f;
        return true;
    }

    // AAN <id> <starting_deg> <end_deg> <rpm>
    float s_deg=0.0f, e_deg=0.0f, rpm=0.0f;
    if (sscanf(line, "AAN %u %f %f %f", &id_u, &s_deg, &e_deg, &rpm) == 4) {
        cmd->type = UI_CMD_AAN;
        cmd->motor_id = (uint8_t)id_u;
        cmd->aan_start_deg = s_deg;
        cmd->aan_end_deg = e_deg;
        cmd->aan_rpm = rpm;
        return true;
    }

    // BRK <id> <brake_current_A>
    float brk_a = 0.0f;
    if (sscanf(line, "BRK %u %f", &id_u, &brk_a) == 2) {
        cmd->type = UI_CMD_BRK;
        cmd->motor_id = (uint8_t)id_u;
        cmd->current_brk = brk_a;
        return true;
    }

    return false;
}

static motor_ctl_t *ctl_for_id(uint8_t id)
{
    if (id == motor1_id) return &g_m1;
    if (id == motor2_id) return &g_m2;
    return NULL;
}


static void ui_poll_and_apply(void)
{
    char line[128];
    ui_cmd_t cmd;

    while (ui_read_line(line, sizeof(line))) {

        // Debug: confirm what the UI actually sent
        // printf("UI RAW: '%s'\n", line);

        if (!ui_parse_command(line, &cmd)) {
            printf("UI: parse error: '%s'\n", line);
            continue;
        }

        motor_ctl_t *ctl = ctl_for_id(cmd.motor_id);

        printf("UI RAW: '%s'\n", line);


        switch (cmd.type) {
            case UI_CMD_SPD:
                if (ctl) {
                    ctl->mode = MCTL_SPD;
                    ctl->spd_erpm = cmd.spd_erpm;
                    ctl->last_ui_update = get_absolute_time();
                }
                printf("TX: SPD %u %d\n", cmd.motor_id, cmd.spd_erpm);
                break;

            case UI_CMD_POS:
                if (ctl) {
                    ctl->mode = MCTL_POS;
                    ctl->pos_deg = cmd.pos_deg;
                    ctl->last_ui_update = get_absolute_time();
                }
                printf("TX: POS %u %.2f\n", cmd.motor_id, cmd.pos_deg);
                break;

            case UI_CMD_PSA:
                if (ctl) {
                    ctl->mode = MCTL_PSA;
                    ctl->pos_deg = cmd.pos_deg;
                    ctl->spd_erpm = cmd.spd_erpm;
                    ctl->acc_erpm_s2 = cmd.acc_erpm_s2;
                    ctl->last_ui_update = get_absolute_time();
                }
                printf("TX: PSA %u %.2f %d %d\n",
                       cmd.motor_id, cmd.pos_deg, cmd.spd_erpm, cmd.acc_erpm_s2);
                break;

            case UI_CMD_TRQ:
                if (ctl) {
                    ctl->mode = MCTL_TRQ;
                    ctl->cur_a = cmd.current_a;
                    ctl->last_ui_update = get_absolute_time();
                }
                printf("TX: TORQUE %u %.3f\n", cmd.motor_id, cmd.current_a);
                break;

            case UI_CMD_STOP: {

                comm_can_set_current(cmd.motor_id, 0.0f);
                comm_can_set_current_brake(cmd.motor_id, 0.0f);
                aan_state_t *aan = aan_for_id(cmd.motor_id);
                if (aan){
                    aan->enabled = false; aan->assisting = false; aan->assist_u = 0.0f;
                }

                if (ctl) {
                    ctl->mode = MCTL_IDLE;
                    ctl->last_ui_update = get_absolute_time();
                }
                printf("TX: STOP %u\n", cmd.motor_id);
            } break;

            case UI_CMD_STOPALL:

                comm_can_set_current(motor1_id, 0.0f);
                comm_can_set_current(motor2_id, 0.0f);

                comm_can_set_current_brake(motor1_id, 0.0f);
                comm_can_set_current_brake(motor2_id, 0.0f);

                g_aan1.enabled = g_aan1.assisting = false; g_aan1.assist_u = 0.0f;
                g_aan2.enabled = g_aan2.assisting = false; g_aan2.assist_u = 0.0f;

                g_m1.mode = MCTL_IDLE;
                g_m2.mode = MCTL_IDLE;
                g_m1.last_ui_update = get_absolute_time();
                g_m2.last_ui_update = get_absolute_time();
                printf("TX: STOPALL\n");
                break;

            case UI_CMD_AAN: {
                aan_state_t *aan = aan_for_id(cmd.motor_id);
                if (aan) {
                    aan->enabled   = true;
                    aan->assisting = false;
                    aan->assist_u  = 0.0f;

                    aan->start_deg = cmd.aan_start_deg;
                    aan->end_deg   = cmd.aan_end_deg;

                    // 180 deg in 1 s => 30 RPM max (JOINT rpm)
                    const float RPM_MAX = 30.0f;
                    const float RPM_MIN = 0.1f;   // avoid division by zero / nonsense

                    // Declare once
                    float rpm_cmd = cmd.aan_rpm;

                    // Clamp magnitude (keep sign)
                    float rpm_abs = fabsf(rpm_cmd);
                    if (rpm_abs > RPM_MAX) rpm_cmd = (rpm_cmd < 0.0f) ? -RPM_MAX : RPM_MAX;
                    if (rpm_abs < RPM_MIN) rpm_cmd = (rpm_cmd < 0.0f) ? -RPM_MIN : RPM_MIN;

                    // Duration computed with CLAMPED rpm
                    float dur = duration_by_rpm(aan->start_deg, aan->end_deg, rpm_cmd);
                    aan->duration_s = (dur > 0.05f) ? dur : 0.05f;

                    // Convert JOINT rpm -> motor ERPM for servo_set_pos_spd
                    float erpm_f = joint_rpm_to_erpm(cmd.motor_id, rpm_cmd);
                    aan->assist_spd_erpm = clamp_i16((int32_t)lroundf(erpm_f));

                    aan->t0 = get_absolute_time();

                    printf("AAN START id=%u start=%.2f end=%.2f rpm_req=%.2f rpm_used=%.2f dur=%.3f spd_erpm=%d\n",
                        cmd.motor_id, aan->start_deg, aan->end_deg, cmd.aan_rpm, rpm_cmd,
                        aan->duration_s, aan->assist_spd_erpm);
                }
            } break;

            case UI_CMD_ORG:
                // One-shot, apply to both motors
                comm_can_set_origin(motor1_id, cmd.origin_mode);
                comm_can_set_origin(motor2_id, cmd.origin_mode);
                printf("UI: ORG both motors mode=%u (ids %u,%u)\n",
                       cmd.origin_mode, motor1_id, motor2_id);
                break;

            case UI_CMD_BRK:
                if (ctl) {
                    ctl->mode = MCTL_BRK;
                    ctl->cur_a = cmd.current_brk;
                    ctl->last_ui_update = get_absolute_time();
                }
                printf("TX: BRK %u %.3f\n", cmd.motor_id, cmd.current_brk);
                break;



            default:
                break;
        }
    }
}

static void motor_keepalive_tick(uint32_t timeout_ms)
{
    absolute_time_t now = get_absolute_time();

    // convenience arrays for iterating motors
    motor_ctl_t *m[2] = { &g_m1, &g_m2 };
    uint8_t      id[2] = { motor1_id, motor2_id };

    for (int k = 0; k < 2; k++) {

        if (timeout_ms > 0) {
            int64_t dt_us = absolute_time_diff_us(m[k]->last_ui_update, now);
            if (dt_us > (int64_t)timeout_ms * 1000) {

                comm_can_set_current(id[k], 0.0f);
                m[k]->mode = MCTL_IDLE;
            }
        }

        aan_state_t *aan = aan_for_id(id[k]);
        if (aan && aan->enabled) {

    
            motor_fb_t *fb = fb_for_id(id[k]);
            if (!fb || !fb->valid) {
                continue;
            }

            // Time since AAN start
            float t_s = (float)absolute_time_diff_us(aan->t0, now) / 1e6f;
            if (t_s < 0.0f) t_s = 0.0f;

            float T = (aan->duration_s > 0.05f) ? aan->duration_s : 0.05f;

            float u_now = clamp01(t_s / T);
            float u_m5  = clamp01((t_s - AAN_DELAY_S) / T);

            float exp_now = lerp_f(aan->start_deg, aan->end_deg, u_now);
            float exp_m5  = lerp_f(aan->start_deg, aan->end_deg, u_m5);
            float act     = fb->pos_deg;

            // Trigger: behind by >= 5s (direction-aware), only after 5s elapsed
            if (!aan->assisting) {
                if (t_s >= AAN_DELAY_S && behind_by_5s(act, exp_now, exp_m5)) {
                    aan->assisting = true;
                    aan->assist_u  = 0.0f;

                    // Use last_ui_update as a generic "last eval time" to avoid adding a field
                    // (Alternatively, add aan->t_last_eval in aan_state_t)
                    m[k]->last_ui_update = now;

                    printf("AAN TRIGGER id=%u t=%.2f act=%.2f exp_m5=%.2f exp=%.2f\n",
                           id[k], t_s, act, exp_m5, exp_now);
                }
            }

            if (aan->assisting) {
                // Ramp assistance smoothly (time-based, not tick-count based)
                float dt_s = (float)absolute_time_diff_us(m[k]->last_ui_update, now) / 1e6f;
                if (dt_s < 0.0f) dt_s = 0.0f;
                if (dt_s > 0.2f) dt_s = 0.2f; // guard
                m[k]->last_ui_update = now;

                aan->assist_u = clamp01(aan->assist_u + AAN_ASSIST_RAMP_RATE * dt_s);

                // "Slowly assist": set a target that moves gradually from current->end
                float target = lerp_f(act, aan->end_deg, aan->assist_u);

                servo_set_pos_spd(id[k],
                                  target,
                                  (int16_t)aan->assist_spd_erpm,
                                  (int16_t)AAN_ASSIST_ACC_ERPM_S2);
            }

            // End conditions: position close enough OR time exceeded with grace
            if (fabsf(aan->end_deg - act) <= AAN_POS_TOL_DEG) {

                printf("AAN DONE id=%u act=%.2f end=%.2f\n", id[k], act, aan->end_deg);

                aan->enabled   = false;
                aan->assisting = false;
                aan->assist_u  = 0.0f;
            }

            // AAN owns the bus for this motor this iteration
            continue;
        }

        switch (m[k]->mode) {

            case MCTL_SPD:
                comm_can_set_spd(id[k], (float)m[k]->spd_erpm);
                break;

            case MCTL_POS:
                comm_can_set_pos(id[k], m[k]->pos_deg);
                break;

            case MCTL_PSA:
                servo_set_pos_spd(id[k],
                                  m[k]->pos_deg,
                                  m[k]->spd_erpm,
                                  m[k]->acc_erpm_s2);
                break;

            case MCTL_TRQ:
                comm_can_set_current(id[k], m[k]->cur_a);
                break;

            case MCTL_BRK:
                comm_can_set_current_brake(id[k], m[k]->cur_a);
                break;

            case MCTL_IDLE:
            default:

                break;
        }
    }
}

static void hard_stop_motor(uint8_t motor_id)
{

    comm_can_set_current(motor_id, 0.0f);
    comm_can_set_current_brake(motor_id, 0.0f);

    aan_state_t *aan = aan_for_id(motor_id);
    if (aan) {
        aan->enabled   = false;
        aan->assisting = false;
        aan->assist_u  = 0.0f;
    }

    motor_ctl_t *ctl = ctl_for_id(motor_id);
    if (ctl) {
        ctl->mode = MCTL_IDLE;
        ctl->spd_erpm = 0;
        ctl->acc_erpm_s2 = 0;
        ctl->cur_a = 0.0f;
        ctl->last_ui_update = get_absolute_time();
    }
}

/* ============================================================
 * SWITCHES
 * ============================================================ */

void switch_init(void) {
    gpio_init(WRIST_STOP_LEFT);
    gpio_init(WRIST_STOP_RIGHT);
    gpio_init(FOREARM_STOP_TOP);
    gpio_init(FOREARM_STOP_BOTTOM);

    gpio_set_dir(WRIST_STOP_LEFT, GPIO_IN);
    gpio_set_dir(WRIST_STOP_RIGHT, GPIO_IN);
    gpio_set_dir(FOREARM_STOP_TOP, GPIO_IN);
    gpio_set_dir(FOREARM_STOP_BOTTOM, GPIO_IN);

    gpio_pull_up(WRIST_STOP_LEFT);
    gpio_pull_up(WRIST_STOP_RIGHT);
    gpio_pull_up(FOREARM_STOP_TOP);
    gpio_pull_up(FOREARM_STOP_BOTTOM);
}

static inline bool switch_tripped(uint pin) {
    return gpio_get(pin);  
}

static volatile bool wrist_stop_latched   = false;
static volatile bool forearm_stop_latched = false;

static inline void update_limit_stops(void) {

    bool wrist_trip_left = false;
    bool wrist_trip_right = false;
    bool forearm_trip_top = false;
    bool forearm_trip_bottom = false;

    wrist_trip_left = switch_tripped(WRIST_STOP_LEFT);  
    wrist_trip_right = switch_tripped(WRIST_STOP_RIGHT);
    forearm_trip_top = switch_tripped(FOREARM_STOP_TOP);
    forearm_trip_bottom = switch_tripped(FOREARM_STOP_BOTTOM);


    if (wrist_trip_left) {
        hard_stop_motor(motor2_id);
        printf("Wrist Left End-stop trigeered. Motor %d stopped.\n", motor2_id);
    }
    if (wrist_trip_right) {
        hard_stop_motor(motor2_id);
        printf("Wrist Right End-stop trigeered. Motor %d stopped.\n", motor2_id);
    }
    if (forearm_trip_top) {
        hard_stop_motor(motor1_id);
        printf("Elbow Top End-stop trigeered. Motor %d stopped.\n", motor1_id);
    }
    if (forearm_trip_bottom) {
        hard_stop_motor(motor1_id);
        printf("Elbow Bottom End-stop trigeered. Motor %d stopped.\n", motor1_id);
    }
}



/* ============================================================
 * MAIN
 * ============================================================ */

int main(void) {
    stdio_init_all();
    switch_init();


    sleep_ms(15000);


    spi_init(MCP2515_SPI, 1000 * 1000);
    gpio_set_function(MCP2515_SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(MCP2515_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MCP2515_MISO_PIN, GPIO_FUNC_SPI);

    gpio_init(MCP2515_CS_PIN);
    gpio_set_dir(MCP2515_CS_PIN, GPIO_OUT);
    gpio_put(MCP2515_CS_PIN, 1);

    printf("CubeMars CAN test start\n");

    mcp2515_init();

    printf("MCP2515 Init Successful\n");

    // Optional: disable filters/masks (often not strictly required if RXM=11)
    mcp2515_write_reg(0x20, 0x00); // RXM0SIDH
    mcp2515_write_reg(0x21, 0x00); // RXM0SIDL
    mcp2515_write_reg(0x22, 0x00); // RXM0EID8
    mcp2515_write_reg(0x23, 0x00); // RXM0EID0

    mcp2515_write_reg(0x24, 0x00); // RXM1SIDH
    mcp2515_write_reg(0x25, 0x00); // RXM1SIDL
    mcp2515_write_reg(0x26, 0x00); // RXM1EID8
    mcp2515_write_reg(0x27, 0x00); // RXM1EID0

    printf("MCP2515 all masks disabled.\n");

    // Accept all messages (standard + extended) into RXB0 and RXB1
    mcp2515_write_reg(0x60, 0x60); // RXB0CTRL: receive any
    mcp2515_write_reg(0x70, 0x60); // RXB1CTRL: receive any
    mcp2515_write_reg(0x2C, 0x00); // clear CANINTF

    printf("MCP2515 Buffers recieve any\n");

    can_frame_t rx1, rx2;

    absolute_time_t t_next = get_absolute_time();

    //comm_can_set_origin(motor1_id, 1);
    //comm_can_set_origin(motor2_id, 1);

    absolute_time_t t_dbg = make_timeout_time_ms(1000);


    while (true) {

        ui_poll_and_apply();
        update_limit_stops();

        

        //comm_can_set_pos(motor1_id, 180);
        //comm_can_set_pos(motor2_id, -90);

        //servo_set_pos_spd(motor1_id, 180.0f, 2000, 800);
        //servo_set_pos_spd(motor2_id, -90.0f, 4000, 20000);

        /*if (absolute_time_diff_us(get_absolute_time(), t_dbg) <= 0) {
            t_dbg = make_timeout_time_ms(1000);

            uint8_t canstat = mcp2515_read_reg(0x0E);
            uint8_t eflg    = mcp2515_read_reg(0x2D);
            uint8_t tec     = mcp2515_read_reg(0x1C);
            uint8_t rec     = mcp2515_read_reg(0x1D);

            printf("CANSTAT=0x%02X EFLG=0x%02X TEC=%u REC=%u\n", canstat, eflg, tec, rec);

            uint8_t canintf = mcp2515_read_reg(0x2C);   // CANINTF
            uint8_t status  = mcp2515_read_status();    // READ_STATUS
            printf("CANINTF=0x%02X READ_STATUS=0x%02X\n", canintf, status);

        }*/

        can_frame_t rx;
        while (mcp2515_receive_frame(&rx)) {

            //print_can_frame(&rx);

            if (rx.extended && (rx.can_id == (0x2900u | motor1_id) ||
                                rx.can_id == (0x2900u | motor2_id))) {
                servo_decode_and_print(&rx);
                servo_decode_to_feedback(&rx);
            }
        }

        motor_keepalive_tick(60000);

        sleep_ms(10);

    }

}
