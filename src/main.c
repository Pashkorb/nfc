#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"

#define AUTH_MODE_URL 1
#define AUTH_MODE_KEY 2
#ifndef AUTH_MODE
#define AUTH_MODE AUTH_MODE_URL
#endif

#define PROGRAM_MODE_URL_ONLY 1
#define PROGRAM_MODE_URL_THEN_KEYS 2
#ifndef PROGRAM_MODE
#define PROGRAM_MODE PROGRAM_MODE_URL_ONLY
#endif

#define ERR_OK 0
#define ERR_TIMEOUT 1
#define ERR_RC522 2
#define ERR_PROTO 3
#define ERR_SW 4
#define ERR_CC_PARSE 5
#define ERR_NOT_IMPLEMENTED 6
#define ERR_SIZE 7
#define ERR_UID 8
#define ERR_CHAINING_UNSUPPORTED 9

#define RC522_CMD_IDLE 0x00u
#define RC522_CMD_SOFTRESET 0x0Fu
#define RC522_CMD_TRANSCEIVE 0x0Cu

#define RC522_REG_COMMAND 0x01u
#define RC522_REG_COMM_IRQ 0x04u
#define RC522_REG_ERROR 0x06u
#define RC522_REG_FIFO_DATA 0x09u
#define RC522_REG_FIFO_LEVEL 0x0Au
#define RC522_REG_BIT_FRAMING 0x0Du
#define RC522_REG_MODE 0x11u
#define RC522_REG_TX_MODE 0x12u
#define RC522_REG_RX_MODE 0x13u
#define RC522_REG_TX_CONTROL 0x14u
#define RC522_REG_TX_ASK 0x15u
#define RC522_REG_T_MODE 0x2Au
#define RC522_REG_T_PRESCALER 0x2Bu
#define RC522_REG_T_RELOAD_H 0x2Cu
#define RC522_REG_T_RELOAD_L 0x2Du
#define RC522_REG_VERSION 0x37u

enum stage_id {
    ST_HW_INIT = 1,
    ST_RC522_RESET,
    ST_RC522_READ_VERSION,
    ST_RC522_CONFIG,
    ST_PICC_REQA,
    ST_PICC_ANTICOLL,
    ST_PICC_SELECT,
    ST_PICC_RATS,
    ST_AUTH,
    ST_T4_SELECT_APP,
    ST_T4_SELECT_CC,
    ST_T4_READ_CC,
    ST_T4_PARSE_CC,
    ST_T4_SELECT_NDEF,
    ST_T4_APDU_PARSE,
    ST_T4_UPDATE_NDEF,
    ST_T4_POST_URL_KEYS,
    ST_DONE
};

volatile int g_stage;
volatile int g_err;
volatile uint8_t g_rc522_ver;
volatile uint8_t g_last_atqa[2];
volatile uint8_t g_last_uid[10];
volatile uint8_t g_last_sak;
volatile uint8_t g_last_errorreg, g_last_commirq, g_last_fifolevel;
volatile uint8_t g_last_resp[128];
volatile uint16_t g_last_resp_len;
volatile uint8_t g_last_sw1, g_last_sw2;
volatile uint8_t g_last_uid_len;
volatile uint8_t g_rx_has_crc;
volatile uint8_t g_t4_last_rx_len;
volatile uint8_t g_t4_last_tail4[4];
volatile uint8_t g_t4_parse_mode;

static const char g_url[] = "https://www.google.com";
static const char g_key_source_url[] = "https://example.com/keys";
static const uint8_t g_app_key[16] = {
    0x00u, 0x11u, 0x22u, 0x33u, 0x44u, 0x55u, 0x66u, 0x77u,
    0x88u, 0x99u, 0xAAu, 0xBBu, 0xCCu, 0xDDu, 0xEEu, 0xFFu
};

static uint8_t g_iso_block_num;

static inline void rc522_cs_low(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_4); }
static inline void rc522_cs_high(void) { GPIO_SetBits(GPIOA, GPIO_Pin_4); }

static void spin_delay(volatile uint32_t n)
{
    while (n--) {
        __NOP();
    }
}

static bool is_plausible_sw1(uint8_t sw1)
{
    if (sw1 == 0x90u || sw1 == 0x61u || sw1 == 0x62u || sw1 == 0x63u) {
        return true;
    }
    return (sw1 >= 0x67u && sw1 <= 0x6Fu);
}

void hw_init_periph(void)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef spi;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    rc522_cs_high();
    GPIO_SetBits(GPIOA, GPIO_Pin_10);

    SPI_I2S_DeInit(SPI1);
    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
}

uint8_t rc522_spi_txrx8(uint8_t v)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
    }
    SPI_I2S_SendData(SPI1, v);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
    }
    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

void rc522_write_reg(uint8_t reg, uint8_t val)
{
    rc522_cs_low();
    rc522_spi_txrx8((uint8_t)((reg << 1u) & 0x7Eu));
    rc522_spi_txrx8(val);
    rc522_cs_high();
}

uint8_t rc522_read_reg(uint8_t reg)
{
    uint8_t v;
    rc522_cs_low();
    rc522_spi_txrx8((uint8_t)(((reg << 1u) & 0x7Eu) | 0x80u));
    v = rc522_spi_txrx8(0x00u);
    rc522_cs_high();
    return v;
}

void rc522_set_auto_crc(bool en)
{
    uint8_t tx = rc522_read_reg(RC522_REG_TX_MODE);
    uint8_t rx = rc522_read_reg(RC522_REG_RX_MODE);
    if (en) {
        tx |= 0x80u;
        rx |= 0x80u;
    } else {
        tx &= (uint8_t)~0x80u;
        rx &= (uint8_t)~0x80u;
    }
    rc522_write_reg(RC522_REG_TX_MODE, tx);
    rc522_write_reg(RC522_REG_RX_MODE, rx);
}

int rc522_transceive(const uint8_t* tx, uint8_t tx_len,
                     uint8_t* rx, uint8_t rx_max, uint8_t* rx_len,
                     uint8_t bit_framing, bool use_crc)
{
    uint32_t i;
    uint8_t irq;

    *rx_len = 0;
    rc522_set_auto_crc(use_crc);

    rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_IDLE);
    rc522_write_reg(RC522_REG_COMM_IRQ, 0x7Fu);
    rc522_write_reg(RC522_REG_FIFO_LEVEL, 0x80u);
    rc522_write_reg(RC522_REG_BIT_FRAMING, (uint8_t)(bit_framing & 0x07u));

    for (i = 0; i < tx_len; i++) {
        rc522_write_reg(RC522_REG_FIFO_DATA, tx[i]);
    }

    rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);
    rc522_write_reg(RC522_REG_BIT_FRAMING, (uint8_t)((bit_framing & 0x07u) | 0x80u));

    for (i = 0; i < 40000u; i++) {
        irq = rc522_read_reg(RC522_REG_COMM_IRQ);
        if ((irq & 0x30u) != 0u) {
            break;
        }
        if ((irq & 0x01u) != 0u) {
            g_last_commirq = irq;
            g_last_errorreg = rc522_read_reg(RC522_REG_ERROR);
            g_last_fifolevel = rc522_read_reg(RC522_REG_FIFO_LEVEL);
            return ERR_TIMEOUT;
        }
    }

    rc522_write_reg(RC522_REG_BIT_FRAMING, (uint8_t)(bit_framing & 0x07u));

    if (i >= 40000u) {
        g_last_commirq = rc522_read_reg(RC522_REG_COMM_IRQ);
        g_last_errorreg = rc522_read_reg(RC522_REG_ERROR);
        g_last_fifolevel = rc522_read_reg(RC522_REG_FIFO_LEVEL);
        return ERR_TIMEOUT;
    }

    g_last_commirq = rc522_read_reg(RC522_REG_COMM_IRQ);
    g_last_errorreg = rc522_read_reg(RC522_REG_ERROR);
    g_last_fifolevel = rc522_read_reg(RC522_REG_FIFO_LEVEL);

    if ((g_last_errorreg & 0x1Bu) != 0u) {
        return ERR_RC522;
    }

    if (g_last_fifolevel > rx_max) {
        return ERR_SIZE;
    }

    for (i = 0; i < g_last_fifolevel; i++) {
        rx[i] = rc522_read_reg(RC522_REG_FIFO_DATA);
    }
    *rx_len = g_last_fifolevel;

    return ERR_OK;
}

static int stage_rc522_reset_and_basic_init(void)
{
    g_stage = ST_RC522_RESET;

    GPIO_ResetBits(GPIOA, GPIO_Pin_10);
    spin_delay(120000u);
    GPIO_SetBits(GPIOA, GPIO_Pin_10);
    spin_delay(120000u);

    rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_SOFTRESET);
    spin_delay(120000u);

    rc522_write_reg(RC522_REG_T_MODE, 0x8Du);
    rc522_write_reg(RC522_REG_T_PRESCALER, 0x3Eu);
    rc522_write_reg(RC522_REG_T_RELOAD_H, 0x00u);
    rc522_write_reg(RC522_REG_T_RELOAD_L, 30u);
    rc522_write_reg(RC522_REG_TX_ASK, 0x40u);
    rc522_write_reg(RC522_REG_MODE, 0x3Du);
    rc522_write_reg(RC522_REG_TX_CONTROL, (uint8_t)(rc522_read_reg(RC522_REG_TX_CONTROL) | 0x03u));

    return ERR_OK;
}

static int stage_reqa(void)
{
    uint8_t reqa = 0x26u;
    uint8_t rx[8];
    uint8_t rx_len = 0;
    int st;

    g_stage = ST_PICC_REQA;
    st = rc522_transceive(&reqa, 1u, rx, sizeof(rx), &rx_len, 0x07u, false);
    if (st != ERR_OK || rx_len < 2u) {
        return st == ERR_OK ? ERR_PROTO : st;
    }

    g_last_atqa[0] = rx[0];
    g_last_atqa[1] = rx[1];
    return ERR_OK;
}

static int picc_anticoll_one_level(uint8_t sel, uint8_t* uidcl)
{
    uint8_t cmd[2] = {sel, 0x20u};
    uint8_t rx[10];
    uint8_t rx_len = 0;
    int st;
    uint8_t i;

    st = rc522_transceive(cmd, sizeof(cmd), rx, sizeof(rx), &rx_len, 0x00u, false);
    if (st != ERR_OK || rx_len < 5u) {
        return st == ERR_OK ? ERR_PROTO : st;
    }

    if ((uint8_t)(rx[0] ^ rx[1] ^ rx[2] ^ rx[3]) != rx[4]) {
        return ERR_UID;
    }

    for (i = 0; i < 5u; i++) {
        uidcl[i] = rx[i];
    }
    return ERR_OK;
}

static int picc_select_one_level(uint8_t sel, const uint8_t* uidcl, uint8_t* sak)
{
    uint8_t cmd[7];
    uint8_t rx[8];
    uint8_t rx_len = 0;
    int st;
    uint8_t i;

    cmd[0] = sel;
    cmd[1] = 0x70u;
    for (i = 0; i < 5u; i++) {
        cmd[2u + i] = uidcl[i];
    }

    st = rc522_transceive(cmd, sizeof(cmd), rx, sizeof(rx), &rx_len, 0x00u, true);
    if (st != ERR_OK || rx_len < 1u) {
        return st == ERR_OK ? ERR_PROTO : st;
    }

    *sak = rx[0];
    return ERR_OK;
}

static int picc_anticoll_select(void)
{
    uint8_t uidcl[5];
    uint8_t uid_len = 0;
    uint8_t level = 0;
    uint8_t sel = 0x93u;
    uint8_t sak = 0;
    uint8_t i;
    int st;

    memset((void*)g_last_uid, 0, sizeof(g_last_uid));

    while (level < 3u) {
        st = picc_anticoll_one_level(sel, uidcl);
        if (st != ERR_OK) {
            return st;
        }

        st = picc_select_one_level(sel, uidcl, &sak);
        if (st != ERR_OK) {
            return st;
        }

        if (uidcl[0] == 0x88u) {
            for (i = 1; i < 4u; i++) {
                if (uid_len >= sizeof(g_last_uid)) {
                    return ERR_SIZE;
                }
                g_last_uid[uid_len++] = uidcl[i];
            }
        } else {
            for (i = 0; i < 4u; i++) {
                if (uid_len >= sizeof(g_last_uid)) {
                    return ERR_SIZE;
                }
                g_last_uid[uid_len++] = uidcl[i];
            }
        }

        if ((sak & 0x04u) == 0u) {
            break;
        }

        level++;
        if (level == 1u) {
            sel = 0x95u;
        } else if (level == 2u) {
            sel = 0x97u;
        } else {
            return ERR_UID;
        }
    }

    g_last_uid_len = uid_len;
    g_last_sak = sak;
    return (uid_len >= 4u) ? ERR_OK : ERR_UID;
}

static int stage_anticoll(void)
{
    g_stage = ST_PICC_ANTICOLL;
    return picc_anticoll_select();
}

static int stage_select(void)
{
    g_stage = ST_PICC_SELECT;
    if (g_last_uid_len == 0u) {
        return ERR_UID;
    }
    return ERR_OK;
}

static int stage_rats(void)
{
    uint8_t cmd[2] = {0xE0u, 0x50u};
    uint8_t rx[32];
    uint8_t rx_len = 0;
    int st;

    g_stage = ST_PICC_RATS;
    st = rc522_transceive(cmd, sizeof(cmd), rx, sizeof(rx), &rx_len, 0x00u, true);
    if (st != ERR_OK || rx_len < 3u) {
        return st == ERR_OK ? ERR_PROTO : st;
    }

    g_last_resp_len = rx_len > sizeof(g_last_resp) ? sizeof(g_last_resp) : rx_len;
    memcpy((void*)g_last_resp, rx, g_last_resp_len);

    g_iso_block_num = 0u;
    return ERR_OK;
}

static int parse_isodep_inf(const uint8_t* rx, uint8_t rx_len,
                            uint8_t* inf, uint16_t inf_max, uint16_t* inf_len)
{
    uint16_t inf_a = 0;
    uint16_t inf_b = 0;
    bool has_a = false;
    bool has_b = false;
    uint8_t sw1_a = 0;
    uint8_t sw1_b = 0;
    uint16_t i;

    g_stage = ST_T4_APDU_PARSE;
    g_t4_last_rx_len = rx_len;
    g_t4_last_tail4[0] = 0u;
    g_t4_last_tail4[1] = 0u;
    g_t4_last_tail4[2] = 0u;
    g_t4_last_tail4[3] = 0u;
    for (i = 0; i < 4u && i < rx_len; i++) {
        g_t4_last_tail4[3u - i] = rx[rx_len - 1u - i];
    }

    if (rx_len >= 5u) {
        inf_a = (uint16_t)rx_len - 3u;
        sw1_a = rx[rx_len - 4u];
        has_a = is_plausible_sw1(sw1_a);
    }
    if (rx_len >= 4u) {
        inf_b = (uint16_t)rx_len - 1u;
        sw1_b = rx[rx_len - 2u];
        has_b = is_plausible_sw1(sw1_b);
    }

    if (g_rx_has_crc == 1u) {
        has_b = false;
    } else if (g_rx_has_crc == 2u) {
        has_a = false;
    }

    if (!has_a && !has_b) {
        return ERR_PROTO;
    }

    if (has_a && (!has_b || sw1_a == 0x90u)) {
        if (inf_a > inf_max) {
            return ERR_SIZE;
        }
        for (i = 0; i < inf_a; i++) {
            inf[i] = rx[1u + i];
        }
        *inf_len = inf_a;
        g_last_sw1 = inf[*inf_len - 2u];
        g_last_sw2 = inf[*inf_len - 1u];
        g_rx_has_crc = 1u;
        g_t4_parse_mode = 1u;
    } else {
        if (inf_b > inf_max) {
            return ERR_SIZE;
        }
        for (i = 0; i < inf_b; i++) {
            inf[i] = rx[1u + i];
        }
        *inf_len = inf_b;
        g_last_sw1 = inf[*inf_len - 2u];
        g_last_sw2 = inf[*inf_len - 1u];
        g_rx_has_crc = 2u;
        g_t4_parse_mode = 2u;
    }

    g_last_resp_len = *inf_len > sizeof(g_last_resp) ? sizeof(g_last_resp) : *inf_len;
    for (i = 0; i < g_last_resp_len; i++) {
        g_last_resp[i] = inf[i];
    }

    return (*inf_len >= 2u) ? ERR_OK : ERR_PROTO;
}

static int isodep_exchange(const uint8_t* apdu, uint8_t apdu_len,
                           uint8_t* inf, uint16_t inf_max, uint16_t* inf_len)
{
    uint8_t tx[260];
    uint8_t rx[260];
    uint8_t tx_len;
    uint8_t rx_len = 0;
    uint8_t last_i[260];
    uint8_t last_i_len = 0;
    uint8_t pcb;
    uint8_t wtxm;
    uint8_t loops = 0;
    uint8_t wtx_rx_len = 0;
    uint8_t wtx_resp[4];
    uint8_t wtx_rx[8];
    int st;

    if (apdu_len == 0u || (uint16_t)apdu_len + 1u > sizeof(tx)) {
        return ERR_SIZE;
    }

    if ((uint16_t)apdu_len + 1u > 60u) {
        return ERR_SIZE;
    }

    tx[0] = (uint8_t)(0x02u | (g_iso_block_num & 0x01u));
    memcpy(&tx[1], apdu, apdu_len);
    tx_len = (uint8_t)(apdu_len + 1u);
    memcpy(last_i, tx, tx_len);
    last_i_len = tx_len;

    while (loops++ < 8u) {
        st = rc522_transceive(tx, tx_len, rx, sizeof(rx), &rx_len, 0x00u, true);
        if (st != ERR_OK) {
            return st;
        }
        if (rx_len < 2u) {
            return ERR_PROTO;
        }

        pcb = rx[0];

        if ((pcb & 0xC0u) == 0x00u) {
            if ((pcb & 0x10u) != 0u) {
                return ERR_CHAINING_UNSUPPORTED;
            }
            st = parse_isodep_inf(rx, rx_len, inf, inf_max, inf_len);
            if (st != ERR_OK) {
                return st;
            }
            g_iso_block_num ^= 1u;
            return ERR_OK;
        }

        if ((pcb & 0xC0u) == 0x80u && pcb <= 0xBFu) {
            memcpy(tx, last_i, last_i_len);
            tx_len = last_i_len;
            continue;
        }

        if ((pcb & 0xC0u) == 0xC0u) {
            if ((pcb & 0x30u) == 0x30u) {
                if (rx_len < 3u) {
                    return ERR_PROTO;
                }
                wtxm = rx[1];
                wtx_resp[0] = (uint8_t)(pcb | 0x20u);
                wtx_resp[1] = wtxm;
                st = rc522_transceive(wtx_resp, 2u, wtx_rx, sizeof(wtx_rx), &wtx_rx_len, 0x00u, true);
                if (st != ERR_OK) {
                    return st;
                }
                spin_delay(200000u);
                continue;
            }
            return ERR_PROTO;
        }
    }

    return ERR_TIMEOUT;
}

int t4_send_apdu(const uint8_t* apdu, uint8_t apdu_len,
                 uint8_t* inf, uint16_t inf_max, uint16_t* inf_len)
{
    return isodep_exchange(apdu, apdu_len, inf, inf_max, inf_len);
}

uint8_t build_ndef_from_url(uint8_t* out, uint8_t out_max)
{
    uint16_t url_len = (uint16_t)strlen(g_url);
    const char* p = g_url;
    uint8_t prefix = 0x00u;
    uint16_t rest_len = url_len;
    uint16_t payload_len;
    uint16_t ndef_len;

    if (strncmp(g_url, "https://", 8) == 0) {
        prefix = 0x04u;
        p = g_url + 8;
        rest_len = (uint16_t)(url_len - 8u);
    } else if (strncmp(g_url, "http://", 7) == 0) {
        prefix = 0x03u;
        p = g_url + 7;
        rest_len = (uint16_t)(url_len - 7u);
    }

    payload_len = (uint16_t)(1u + rest_len);
    ndef_len = (uint16_t)(4u + payload_len);
    if (ndef_len + 2u > out_max || ndef_len + 2u > 255u || payload_len > 255u) {
        return 0u;
    }

    out[0] = (uint8_t)(ndef_len >> 8);
    out[1] = (uint8_t)(ndef_len & 0xFFu);
    out[2] = 0xD1u;
    out[3] = 0x01u;
    out[4] = (uint8_t)payload_len;
    out[5] = 0x55u;
    out[6] = prefix;
    memcpy(&out[7], p, rest_len);

    return (uint8_t)(2u + ndef_len);
}

static int stage_authenticate(void)
{
    g_stage = ST_AUTH;
#if AUTH_MODE == AUTH_MODE_URL
    (void)g_key_source_url;
    return ERR_OK;
#elif AUTH_MODE == AUTH_MODE_KEY
    (void)g_app_key;
    return ERR_NOT_IMPLEMENTED;
#else
    return ERR_NOT_IMPLEMENTED;
#endif
}

static int stage_post_url_keys(void)
{
    g_stage = ST_T4_POST_URL_KEYS;
#if PROGRAM_MODE == PROGRAM_MODE_URL_THEN_KEYS
    return ERR_NOT_IMPLEMENTED;
#else
    return ERR_OK;
#endif
}

static int stage_program_ntag424_url(void)
{
    static const uint8_t apdu_select_app[] = {0x00u, 0xA4u, 0x04u, 0x00u, 0x07u, 0xD2u, 0x76u, 0x00u, 0x00u, 0x85u, 0x01u, 0x01u, 0x00u};
    static const uint8_t apdu_select_cc[] = {0x00u, 0xA4u, 0x00u, 0x0Cu, 0x02u, 0xE1u, 0x03u};
    static const uint8_t apdu_read_cc[] = {0x00u, 0xB0u, 0x00u, 0x00u, 0x0Fu};
    uint8_t apdu_select_ndef[] = {0x00u, 0xA4u, 0x00u, 0x0Cu, 0x02u, 0x00u, 0x00u};
    uint8_t apdu_update[260];
    uint8_t ndef[255];
    uint8_t ndef_len;
    uint8_t cc[32];
    uint16_t inf_len;
    int st;
    uint8_t fid_h;
    uint8_t fid_l;

    g_stage = ST_T4_SELECT_APP;
    st = t4_send_apdu(apdu_select_app, sizeof(apdu_select_app), cc, sizeof(cc), &inf_len);
    if (st != ERR_OK || g_last_sw1 != 0x90u || g_last_sw2 != 0x00u) {
        return st == ERR_OK ? ERR_SW : st;
    }

    g_stage = ST_T4_SELECT_CC;
    st = t4_send_apdu(apdu_select_cc, sizeof(apdu_select_cc), cc, sizeof(cc), &inf_len);
    if (st != ERR_OK || g_last_sw1 != 0x90u || g_last_sw2 != 0x00u) {
        return st == ERR_OK ? ERR_SW : st;
    }

    g_stage = ST_T4_READ_CC;
    st = t4_send_apdu(apdu_read_cc, sizeof(apdu_read_cc), cc, sizeof(cc), &inf_len);
    if (st != ERR_OK || g_last_sw1 != 0x90u || g_last_sw2 != 0x00u) {
        return st == ERR_OK ? ERR_SW : st;
    }
    if (inf_len < 17u) {
        return ERR_CC_PARSE;
    }

    g_stage = ST_T4_PARSE_CC;
    if (cc[6] != 0x04u || cc[7] != 0x06u) {
        return ERR_CC_PARSE;
    }
    fid_h = cc[8];
    fid_l = cc[9];

    g_stage = ST_T4_SELECT_NDEF;
    apdu_select_ndef[5] = fid_h;
    apdu_select_ndef[6] = fid_l;
    st = t4_send_apdu(apdu_select_ndef, sizeof(apdu_select_ndef), cc, sizeof(cc), &inf_len);
    if (st != ERR_OK || g_last_sw1 != 0x90u || g_last_sw2 != 0x00u) {
        return st == ERR_OK ? ERR_SW : st;
    }

    g_stage = ST_T4_UPDATE_NDEF;
    ndef_len = build_ndef_from_url(ndef, sizeof(ndef));
    if (ndef_len == 0u) {
        return ERR_SIZE;
    }
    apdu_update[0] = 0x00u;
    apdu_update[1] = 0xD6u;
    apdu_update[2] = 0x00u;
    apdu_update[3] = 0x00u;
    apdu_update[4] = ndef_len;
    memcpy(&apdu_update[5], ndef, ndef_len);

    st = t4_send_apdu(apdu_update, (uint8_t)(5u + ndef_len), cc, sizeof(cc), &inf_len);
    if (st != ERR_OK || g_last_sw1 != 0x90u || g_last_sw2 != 0x00u) {
        return st == ERR_OK ? ERR_SW : st;
    }

    return ERR_OK;
}

int main(void)
{
    int st;

    g_err = ERR_OK;
    g_rx_has_crc = 0u;
    g_stage = ST_HW_INIT;
    hw_init_periph();

    st = stage_rc522_reset_and_basic_init();
    if (st != ERR_OK) { g_err = st; while (1) {} }

    g_stage = ST_RC522_READ_VERSION;
    g_rc522_ver = rc522_read_reg(RC522_REG_VERSION);
    if (g_rc522_ver == 0x00u || g_rc522_ver == 0xFFu) { g_err = ERR_PROTO; while (1) {} }

    g_stage = ST_RC522_CONFIG;
    rc522_write_reg(RC522_REG_TX_CONTROL, (uint8_t)(rc522_read_reg(RC522_REG_TX_CONTROL) | 0x03u));
    rc522_set_auto_crc(true);

    st = stage_reqa();
    if (st != ERR_OK) { g_err = st; while (1) {} }

    st = stage_anticoll();
    if (st != ERR_OK) { g_err = st; while (1) {} }

    st = stage_select();
    if (st != ERR_OK) { g_err = st; while (1) {} }

    st = stage_rats();
    if (st != ERR_OK) { g_err = st; while (1) {} }

    st = stage_authenticate();
    if (st != ERR_OK) { g_err = st; while (1) {} }

    st = stage_program_ntag424_url();
    if (st != ERR_OK) { g_err = st; while (1) {} }

    st = stage_post_url_keys();
    if (st != ERR_OK) { g_err = st; while (1) {} }

    g_stage = ST_DONE;
    while (1) {
    }
}
