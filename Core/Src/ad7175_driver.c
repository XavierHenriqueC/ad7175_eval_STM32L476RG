#include "ad7175_driver.h"
#include "uart_printf.h" /* for debug prints */
#include <string.h>
#include <stdio.h>

/* External UART handle used by uart_printf module */
extern UART_HandleTypeDef huart2;

/* Internal SPI handle pointer stored after init */
static SPI_HandleTypeDef *ad7175_hspi = NULL;

/* ----------------- low-level SPI/CS helpers ----------------- */

/* Toggle CS around each SPI transaction to keep device in defined state */
static inline void cs_low(void)  { HAL_GPIO_WritePin(AD7175_CS_PORT, AD7175_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(AD7175_CS_PORT, AD7175_CS_PIN, GPIO_PIN_SET); }

/* do SPI transmit+receive while toggling CS for atomic transaction */
static int spi_txrx_with_cs(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    if (!ad7175_hspi) return -1;
    cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(ad7175_hspi, (uint8_t*)tx, rx, n, AD7175_SPI_TIMEOUT);
    cs_high();
    return (st == HAL_OK) ? 0 : -1;
}

/* do SPI transmit (no rx) with cs */
static int spi_tx_with_cs(const uint8_t *tx, uint16_t n)
{
    if (!ad7175_hspi) return -1;
    cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(ad7175_hspi, (uint8_t*)tx, n, AD7175_SPI_TIMEOUT);
    cs_high();
    return (st == HAL_OK) ? 0 : -1;
}

/* ----------------- reg read/write (per-operation CS) ----------------- */
/* write reg (no CRC) */
static int reg_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[8];
    if (len > 6) return -1;
    buf[0] = reg & 0x3F; /* write opcode + address */
    memcpy(&buf[1], data, len);
    return spi_tx_with_cs(buf, len + 1);
}

/* read reg (no CRC) */
static int reg_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (len > 6) return -1;
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};
    tx[0] = 0x40 | (reg & 0x3F);
    if (spi_txrx_with_cs(tx, rx, len + 1) < 0) return -1;
    memcpy(data, &rx[1], len);
    return 0;
}

/* reg write with readback (retries) */
static int reg_write_with_readback(uint8_t reg, const uint8_t *data, uint8_t len, int retries)
{
    uint8_t rb[8];
    for (int i = 0; i <= retries; ++i) {
        if (reg_write(reg, data, len) < 0) {
            uart_printf("WERR reg 0x%02X try %d\r\n", reg, i);
            continue;
        }
        HAL_Delay(1);
        if (reg_read(reg, rb, len) < 0) {
            uart_printf("RERR reg 0x%02X try %d\r\n", reg, i);
            continue;
        }
        if (memcmp(rb, data, len) == 0) {
            if (len == 2) uart_printf("OK: reg 0x%02X -> 0x%02X 0x%02X\r\n", reg, rb[0], rb[1]);
            else {
                uart_printf("OK: reg 0x%02X ->", reg);
                for (int j=0;j<len;++j) uart_printf(" 0x%02X", rb[j]);
                uart_printf("\r\n");
            }
            return 0;
        } else {
            uart_printf("MIS reg 0x%02X wrote", reg);
            for (int j=0;j<len;++j) uart_printf(" 0x%02X", data[j]);
            uart_printf(" read");
            for (int j=0;j<len;++j) uart_printf(" 0x%02X", rb[j]);
            uart_printf(" try %d\r\n", i);
        }
        HAL_Delay(2);
    }
    uart_printf("FAIL reg 0x%02X after %d retries\r\n", reg, retries);
    return -1;
}

/* ----------------- Reset ----------------- */
/* Issue 8 bytes of 0xFF with CS toggled (device reset) */
static void ad7175_reset(void)
{
    uint8_t ones[8];
    memset(ones, 0xFF, sizeof(ones));
    /* one transaction is enough */
    spi_tx_with_cs(ones, sizeof(ones));
    HAL_Delay(5);
}

/* read DRDY pin state via GPIO IDR */
static inline GPIO_PinState read_drdy_pin(void)
{
    return HAL_GPIO_ReadPin(AD7175_MISO_PORT, AD7175_MISO_PIN);
}

/* ----------------- Public API ----------------- */

void AD7175_Init(AD7175_Handle_t *h, SPI_HandleTypeDef *spi)
{
    (void)h;
    ad7175_hspi = spi;
    if (!ad7175_hspi) {
        uart_printf("AD7175 Init: SPI handle NULL!\r\n");
        Error_Handler();
    }

    uart_printf("==== Inicializando AD7175-2 (multicanal) ====\r\n");

    /* Reset device */
    ad7175_reset();

    /* Read ID (reg 0x07) */
    uint8_t id[2] = {0};
    if (reg_read(0x07, id, 2) == 0) {
        uint16_t idv = ((uint16_t)id[0] << 8) | id[1];
        uart_printf("ID = 0x%04X\r\n", idv);
    } else {
        uart_printf("ID read failed\r\n");
    }

    /* IFMODE: DATA+STATUS (0x0040) */
    uint8_t ifmode[2] = {0x00, 0x40};
    reg_write_with_readback(0x02, ifmode, 2, 3);

    /* ADCMODE: REF_EN=1, continuous, internal clock (0x8000) */
    uint8_t adcmode[2] = {0x80, 0x00};
    reg_write_with_readback(0x01, adcmode, 2, 3);

    /* SETUP0/SETUP1: UNIPOLAR + input buffer enabled -> 0x0300 */
    uint8_t setup0[2] = {0x03, 0x00};
    uint8_t setup1[2] = {0x03, 0x00};
    reg_write_with_readback(0x20, setup0, 2, 3);
    reg_write_with_readback(0x21, setup1, 2, 3);

    /* FILTER0/FILTER1: ODR ~200 SPS -> ODR=0x0D, SINC5+SINC1 -> 0x000D */
    uint8_t filt0[2] = {0x00, 0x0D};
    uint8_t filt1[2] = {0x00, 0x0D};
    reg_write_with_readback(0x28, filt0, 2, 3);
    reg_write_with_readback(0x29, filt1, 2, 3);

    /* CHMAP0 = 0x8001 (CH0: AIN0+/AIN1-, setup0)
       CHMAP1 = 0x9043 (CH1: AIN2+/AIN3-, setup1)
       NOTE: high byte: bit15=1 (enable), bits14..12 = setup index
    */
    uint8_t ch0[2] = {0x80, 0x01};   /* enable + AINPOS=0,AINNEG=1 (setup0) */
    uint8_t ch1[2] = {0x90, 0x43};   /* enable + setup1 + AINPOS=2,AINNEG=3 */
    reg_write_with_readback(0x10, ch0, 2, 3);
    reg_write_with_readback(0x11, ch1, 2, 3);

    uart_printf("Done init (unipolar, ext 5V, ~200SPS, CH0/CH1)\r\n");
}

int AD7175_PollAndRead(uint32_t timeout_ms, uint8_t *channel, uint32_t *raw)
{
    uint32_t t0 = HAL_GetTick();

    /* Wait DRDY LOW on MISO */
    while (read_drdy_pin() != GPIO_PIN_RESET) {
        if ((HAL_GetTick() - t0) >= timeout_ms)
            return -1; // timeout
    }

    /* Read DATA register */
    uint8_t tx[4] = {0x44, 0, 0, 0};
    uint8_t rx[4] = {0};

    if (spi_txrx_with_cs(tx, rx, 4) < 0)
        return -2; // spi error

    uint8_t status = rx[0];
    uint32_t val = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];

    /* Check ADC Error bit */
    if (status & 0x40)
        return -3;

    /* Extract CH ID (bits 5:0) */
    uint8_t chid = status & 0x3F;

    if (channel) {
        if (chid == 1) *channel = 0;
        else if (chid == 3) *channel = 1;
        else *channel = 255; // invalid channel
    }

    if (raw)
        *raw = val;

    return 0;
}


double AD7175_ConvertToMilliVolts(uint32_t raw)
{
    double code = (double)(raw & 0xFFFFFFu);
    return (code / 16777215.0) * AD7175_VREF_MV;
}
