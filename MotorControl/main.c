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
    UI_CMD_TRQ
} ui_cmd_type_t;

typedef struct {
    ui_cmd_type_t type;
    uint8_t motor_id;

    float   pos_deg;
    int16_t spd_erpm;
    int16_t acc_erpm_s2;
    uint8_t origin_mode;
    float current_a;
} ui_cmd_t;

typedef enum {
    MCTL_NONE = 0,
    MCTL_SPD,
    MCTL_POS,
    MCTL_PSA,
    MCTL_TRQ,
    MCTL_STOP,
    MCTL_IDLE
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

static void comm_can_set_current(uint8_t motor_id, float current_a)
{
    uint8_t buf[4];
    int32_t idx = 0;

    current_a = clamp_f(current_a, -5.0f, 5.0f);

    buffer_append_int32(buf, (int32_t)(current_a * 1000.0f), &idx);

    uint32_t eid = motor_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
    (void)comm_can_transmit_eid_motor(motor_id, eid, buf, (uint8_t)idx);
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

    uint8_t addr;
    bool from_rxb0 = false;

    // RX0IF is indicated via status bits; common approach:
    // If either RX buffer has data, pick one.
    if (status & 0x01) {          // RX0IF
        addr = 0x61;              // RXB0SIDH register address
        from_rxb0 = true;
    } else if (status & 0x02) {   // RX1IF
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

    printf("SERVO,%u,%.1f,%.0f,%.2f,%d,%u\n", motor_id, pos_deg, spd_erpm, cur_a, temp_c, err);
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
    if (strcmp(line, "ORIGIN") == 0) {
        cmd->type = UI_CMD_ORG;
        cmd->origin_mode = 0;
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

            case UI_CMD_STOP:

                comm_can_set_current(cmd.motor_id, 0.0f);

                if (ctl) {
                    ctl->mode = MCTL_IDLE;
                    ctl->last_ui_update = get_absolute_time();
                }
                printf("TX: STOP %u\n", cmd.motor_id);
                break;

            case UI_CMD_STOPALL:

                comm_can_set_current(motor1_id, 0.0f);
                comm_can_set_current(motor2_id, 0.0f);

                g_m1.mode = MCTL_IDLE;
                g_m2.mode = MCTL_IDLE;
                g_m1.last_ui_update = get_absolute_time();
                g_m2.last_ui_update = get_absolute_time();
                printf("TX: STOPALL\n");
                break;

            case UI_CMD_ORG:
                // One-shot, apply to both motors
                comm_can_set_origin(motor1_id, cmd.origin_mode);
                comm_can_set_origin(motor2_id, cmd.origin_mode);
                printf("UI: ORG both motors mode=%u (ids %u,%u)\n",
                       cmd.origin_mode, motor1_id, motor2_id);
                break;

            default:
                break;
        }
    }
}

static void motor_keepalive_tick(uint32_t timeout_ms)
{
    absolute_time_t now = get_absolute_time();

    motor_ctl_t *m[2] = { &g_m1, &g_m2 };
    uint8_t id[2] = { motor1_id, motor2_id };

    for (int k = 0; k < 2; k++) {
        // Optional safety: if UI is silent too long, stop the motor
        if (timeout_ms > 0) {
            int64_t dt_us = absolute_time_diff_us(m[k]->last_ui_update, now);
            if (dt_us > (int64_t)timeout_ms * 1000) {
                m[k]->mode = MCTL_STOP;
            }
        }

        switch (m[k]->mode) {
            case MCTL_SPD:
                comm_can_set_spd(id[k], (float)m[k]->spd_erpm);
                break;
            case MCTL_POS:
                comm_can_set_pos(id[k], m[k]->pos_deg);
                break;
            case MCTL_PSA:
                servo_set_pos_spd(id[k], m[k]->pos_deg, m[k]->spd_erpm, m[k]->acc_erpm_s2);
                break;
            case MCTL_TRQ:
                comm_can_set_current(id[k], m[k]->cur_a);
                break;
            case MCTL_IDLE:
                // True idle: do not send any command for this motor
                break;

            default:
                // If something unknown happens, fail safe to IDLE (no commands)
                m[k]->mode = MCTL_IDLE;
                break;
        }
    }
}






/* ============================================================
 * MIT MODE
 * ============================================================ */

static unsigned int float_to_uint(float x, float min, float max, unsigned bits) {
    float span = max - min;
    if (x < min) x = min;
    if (x > max) x = max;
    return (unsigned)((x - min) * (((1 << bits) - 1) / span));
}

#define P_MIN -12.5f
#define P_MAX  12.5f
#define V_MIN -45.0f
#define V_MAX  45.0f
#define T_MIN -18.0f
#define T_MAX  18.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

static void mit_pack_cmd(uint8_t id, float p, float v, float kp, float kd, float t) {
    uint8_t buf[8];

    int p_i  = float_to_uint(p,  P_MIN,  P_MAX, 16);
    int v_i  = float_to_uint(v,  V_MIN,  V_MAX, 12);
    int kp_i = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int kd_i = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    int t_i  = float_to_uint(t,  T_MIN,  T_MAX, 12);

    buf[0] = p_i >> 8;
    buf[1] = p_i & 0xFF;
    buf[2] = v_i >> 4;
    buf[3] = ((v_i & 0xF) << 4) | (kp_i >> 8);
    buf[4] = kp_i & 0xFF;
    buf[5] = kd_i >> 4;
    buf[6] = ((kd_i & 0xF) << 4) | (t_i >> 8);
    buf[7] = t_i & 0xFF;

    can_frame_t f = {
        .can_id = id,
        .can_dlc = 8,
        .extended = false
    };
    memcpy(f.data, buf, 8);
    can_send(&f);
}

/* ============================================================
 * MAIN
 * ============================================================ */

static void dbg_mcp2515_tx_state(void)
{
    uint8_t tx0 = mcp2515_read_reg(0x30); // TXB0CTRL
    uint8_t tx1 = mcp2515_read_reg(0x40); // TXB1CTRL
    printf("DBG TXB0CTRL=0x%02X TXB1CTRL=0x%02X (TXREQ0=%u TXREQ1=%u)\n",
           tx0, tx1, (tx0 >> 3) & 1, (tx1 >> 3) & 1);
}

static void dbg_mcp2515_eflg(void)
{
    uint8_t eflg = mcp2515_read_reg(0x2D);
    printf("DBG EFLG=0x%02X\n", eflg);
}


int main(void) {
    stdio_init_all();
    sleep_ms(10000);

    spi_init(MCP2515_SPI, 1000 * 1000);
    gpio_set_function(MCP2515_SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(MCP2515_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MCP2515_MISO_PIN, GPIO_FUNC_SPI);

    gpio_init(MCP2515_CS_PIN);
    gpio_set_dir(MCP2515_CS_PIN, GPIO_OUT);
    gpio_put(MCP2515_CS_PIN, 1);


    // Optional: disable filters/masks (often not strictly required if RXM=11)
    mcp2515_write_reg(0x20, 0x00); // RXM0SIDH
    mcp2515_write_reg(0x21, 0x00); // RXM0SIDL
    mcp2515_write_reg(0x22, 0x00); // RXM0EID8
    mcp2515_write_reg(0x23, 0x00); // RXM0EID0

    mcp2515_write_reg(0x24, 0x00); // RXM1SIDH
    mcp2515_write_reg(0x25, 0x00); // RXM1SIDL
    mcp2515_write_reg(0x26, 0x00); // RXM1EID8
    mcp2515_write_reg(0x27, 0x00); // RXM1EID0


    printf("CubeMars CAN test start\n");

    mcp2515_init();

    // Accept all messages (standard + extended) into RXB0 and RXB1
    mcp2515_write_reg(0x60, 0x60); // RXB0CTRL: receive any
    mcp2515_write_reg(0x70, 0x60); // RXB1CTRL: receive any

    can_frame_t rx1, rx2;

    absolute_time_t t_next = get_absolute_time();

    //comm_can_set_origin(motor1_id, 1);
    //comm_can_set_origin(motor2_id, 1);


    while (true) {

        ui_poll_and_apply();
        motor_keepalive_tick(10000);

        //comm_can_set_pos(motor1_id, 180);
        //comm_can_set_pos(motor2_id, -90);

        //servo_set_pos_spd(motor1_id, 180.0f, 2000, 800);
        //servo_set_pos_spd(motor2_id, -90.0f, 4000, 20000);

        can_frame_t rx;
        while (mcp2515_receive_frame(&rx)) {
            /*if (rx.extended && (rx.can_id == (0x2900u | motor1_id) ||
                                rx.can_id == (0x2900u | motor2_id))) {
                servo_decode_and_print(&rx);
            }*/

                dbg_mcp2515_tx_state();
                dbg_mcp2515_eflg();

            uint32_t t0 = to_ms_since_boot(get_absolute_time());
            while (to_ms_since_boot(get_absolute_time()) - t0 < 5000) {
                can_frame_t rx;
                while (mcp2515_receive_frame(&rx)) {
                    print_can_frame(&rx); // raw print
                }
                sleep_ms(10);
            }
        }

        sleep_ms(10);

    }

}
