/* ad7175_driver.c - polling + verbose debug
 * Replace your current file with this to gather debug info about CH sequencing
 */
#include "ad7175_driver.h"
#include "uart_printf.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

static SPI_HandleTypeDef *ad7175_hspi = NULL;

/* shared last sample */
volatile uint8_t  ad7175_last_channel = 0xFF;
volatile uint32_t ad7175_last_raw     = 0xFFFFFFFF;
volatile uint8_t  ad7175_new_sample   = 0;

/* ---------------- CS helpers ---------------- */
static inline void cs_low(void)
{
    HAL_GPIO_WritePin(AD7175_CS_PORT, AD7175_CS_PIN, GPIO_PIN_RESET);
}

static inline void cs_high(void)
{
    HAL_GPIO_WritePin(AD7175_CS_PORT, AD7175_CS_PIN, GPIO_PIN_SET);
}

/* ---------------- SPI low-level ---------------- */
static int spi_txrx(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    if (!ad7175_hspi) return -1;
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(ad7175_hspi, (uint8_t*)tx, rx, n, AD7175_SPI_TIMEOUT);
    return (st == HAL_OK) ? 0 : -1;
}

/* ------------ Register R/W (no-contread) ----------- */
static int reg_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[8];
    if (len > 6) return -1;

    buf[0] = reg & 0x3F;   // write cmd
    memcpy(&buf[1], data, len);

    cs_low();
    int r = (HAL_SPI_Transmit(ad7175_hspi, buf, len+1, AD7175_SPI_TIMEOUT) == HAL_OK) ? 0 : -1;
    cs_high();

    if (r == 0) {
        uart_printf("W: reg0x%02X wrote", reg);
        for (int i=0;i<len;i++) uart_printf(" 0x%02X", data[i]);
        uart_printf("\r\n");
    } else {
        uart_printf("WERR: reg0x%02X txfail\r\n", reg);
    }
    return r;
}

static int reg_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};

    tx[0] = 0x40 | (reg & 0x3F);

    cs_low();
    int r = spi_txrx(tx, rx, len+1);
    cs_high();

    if (r < 0) {
        uart_printf("RERR: reg0x%02X rxfail\r\n", reg);
        return -1;
    }

    memcpy(data, &rx[1], len);

    uart_printf("R: reg0x%02X ->", reg);
    for (int i=0;i<len;i++) uart_printf(" 0x%02X", data[i]);
    uart_printf("\r\n");

    return 0;
}

static int reg_write_rb(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t rb[8];

    for (int i = 0; i < 6; i++)
    {
        if (reg_write(reg, data, len) < 0) {
            uart_printf("MIS reg 0x%02X wrote", reg);
            for (int j=0;j<len;j++) uart_printf(" 0x%02X", data[j]);
            uart_printf(" (txfail) try %d\r\n", i);
            HAL_Delay(3);
            continue;
        }
        HAL_Delay(3);

        if (reg_read(reg, rb, len) < 0) {
            uart_printf("MIS reg 0x%02X readback fail try %d\r\n", reg, i);
            HAL_Delay(3);
            continue;
        }

        if (memcmp(rb, data, len) == 0) {
            uart_printf("OK: reg 0x%02X ->", reg);
            for (int j=0;j<len;++j) uart_printf(" 0x%02X", rb[j]);
            uart_printf(" (try %d)\r\n", i);
            return 0;
        } else {
            uart_printf("MIS reg 0x%02X wrote", reg);
            for (int j=0;j<len;++j) uart_printf(" 0x%02X", data[j]);
            uart_printf(" read");
            for (int j=0;j<len;++j) uart_printf(" 0x%02X", rb[j]);
            uart_printf(" try %d\r\n", i);
        }
        HAL_Delay(3);
    }
    uart_printf("FAIL reg 0x%02X after %d retries\r\n", reg, 6);
    return -1;
}



/* Send 64 SCLKs with DIN=1 (8 bytes 0xFF) to reset device interface.
   Use the driver SPI handle (same approach as ad7175_reset()). */
void AD7175_ResetToDefaults(void)
{
    if (!ad7175_hspi) {
        uart_printf("AD7175_ResetToDefaults: SPI handle NULL!\r\n");
        return;
    }

    uint8_t ff[8];
    memset(ff, 0xFF, sizeof(ff));

    uart_printf("AD7175_ResetToDefaults: sending 8x 0xFF (64 SCLKs)\r\n");

    cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(ad7175_hspi, ff, sizeof(ff), AD7175_SPI_TIMEOUT);
    cs_high();

    if (st != HAL_OK) {
        uart_printf("AD7175_ResetToDefaults: SPI transmit failed\r\n");
    }

    /* small delay to allow device to process reset */
    HAL_Delay(5);
}

/* Enable both CH0 and CH1 safely:
   - zero CHMAP0..CHMAP3 first
   - write CHMAP0 = 0x8001 (CH0 enable, AINPOS=0, AINNEG=1, setup0)
   - write CHMAP1 = 0x9043 (CH1 enable, AINPOS=2, AINNEG=3, setup1)
   Returns 0 on success, -1 on error.
*/
int AD7175_Enable_CH0_CH1(void)
{
    int res = 0;
    uint8_t zero[2] = {0x00, 0x00};

    uart_printf("AD7175_Enable_CH0_CH1: clearing CHMAP0..CHMAP3 first\r\n");
    for (int r = 0x10; r <= 0x13; ++r) {
        if (reg_write_rb(r, zero, 2) < 0) {
            uart_printf("AD7175_Enable_CH0_CH1: failed to clear CHMAP 0x%02X\r\n", r);
            res = -1;
            /* continue anyway to attempt best-effort writes */
        }
    }

    /* Now write CH0 and CH1 */
    uint8_t ch0[2] = {0x80, 0x01};   /* enable + AINPOS=0, AINNEG=1 (setup0) */
    uint8_t ch1[2] = {0x90, 0x43};   /* enable + setup1 + AINPOS=2,AINNEG=3 */

    uart_printf("AD7175_Enable_CH0_CH1: writing CHMAP0 = 0x%02X%02X\r\n", ch0[0], ch0[1]);
    if (reg_write_rb(0x10, ch0, 2) < 0) {
        uart_printf("AD7175_Enable_CH0_CH1: failed to write CHMAP0\r\n");
        res = -1;
    }

    uart_printf("AD7175_Enable_CH0_CH1: writing CHMAP1 = 0x%02X%02X\r\n", ch1[0], ch1[1]);
    if (reg_write_rb(0x11, ch1, 2) < 0) {
        uart_printf("AD7175_Enable_CH0_CH1: failed to write CHMAP1\r\n");
        res = -1;
    }

    /* readback for sanity */
    uint8_t rb[2];
    if (reg_read(0x10, rb, 2) == 0)
        uart_printf("R: CHMAP0 -> 0x%02X 0x%02X\r\n", rb[0], rb[1]);
    if (reg_read(0x11, rb, 2) == 0)
        uart_printf("R: CHMAP1 -> 0x%02X 0x%02X\r\n", rb[0], rb[1]);

    return res;
}


/* ---------------- Full register reset to datasheet defaults ---------------- */
void AD7175_FullRegisterReset(void)
{
    uart_printf("AD7175_FullRegisterReset: zeroing registers to datasheet defaults...\r\n");
    uint8_t d2[2];
    uint8_t d3[3];

    /* ADCMODE/IFMODE/GPIOCON -> 0 */
    d2[0] = 0x00; d2[1] = 0x00;
    reg_write_rb(0x01, d2, 2);   // ADCMODE
    reg_write_rb(0x02, d2, 2);   // IFMODE
    reg_write_rb(0x05, d2, 2);   // GPIOCON

    /* GPO -> 0 */
    uint8_t gpo = 0x00;
    reg_write_rb(0x06, &gpo, 1);

    /* CHMAP0..CHMAP7 -> 0x0000 */
    d2[0] = 0x00; d2[1] = 0x00;
    for (int r = 0x10; r <= 0x17; ++r) {
        reg_write_rb(r, d2, 2);
    }

    /* SETUP0..SETUP7 -> 0x0000 */
    for (int r = 0x20; r <= 0x27; ++r) {
        reg_write_rb(r, d2, 2);
    }

    /* FILTER0..FILTER7 -> default 0x0600 (datasheet) */
    d2[0] = 0x06; d2[1] = 0x00;
    for (int r = 0x28; r <= 0x2F; ++r) {
        reg_write_rb(r, d2, 2);
    }

    /* OFFSET0..OFFSET7 -> 0x800000 */
    d3[0] = 0x80; d3[1] = 0x00; d3[2] = 0x00;
    for (int r = 0x30; r <= 0x37; ++r) {
        reg_write_rb(r, d3, 3);
    }

    /* GAIN0..GAIN7 -> 0x500000 */
    d3[0] = 0x50; d3[1] = 0x00; d3[2] = 0x00;
    for (int r = 0x38; r <= 0x3F; ++r) {
        reg_write_rb(r, d3, 3);
    }

    uart_printf("AD7175_FullRegisterReset: done\r\n");
}

/* ---------------- Register dump helper ---------------- */
void AD7175_DumpRegisters(void)
{
    uart_printf("Dumping key registers:\r\n");
    uint8_t buf[3];

    /* ADCMODE, IFMODE */
    if (reg_read(0x01, buf, 2) == 0) uart_printf("ADCMODE = 0x%02X%02X\r\n", buf[0], buf[1]);
    if (reg_read(0x02, buf, 2) == 0) uart_printf("IFMODE  = 0x%02X%02X\r\n", buf[0], buf[1]);

    /* CHMAP0..CHMAP3 */
    for (int r = 0x10; r <= 0x13; ++r) {
        if (reg_read(r, buf, 2) == 0) uart_printf("CHMAP%u = 0x%02X%02X\r\n", r-0x10, buf[0], buf[1]);
    }

    /* SETUP0..SETUP1 */
    for (int r = 0x20; r <= 0x21; ++r) {
        if (reg_read(r, buf, 2) == 0) uart_printf("SETUP%u = 0x%02X%02X\r\n", r-0x20, buf[0], buf[1]);
    }

    /* FILTER0..FILTER1 */
    for (int r = 0x28; r <= 0x29; ++r) {
        if (reg_read(r, buf, 2) == 0) uart_printf("FILTER%u = 0x%02X%02X\r\n", r-0x28, buf[0], buf[1]);
    }

    /* ID */
    if (reg_read(0x07, buf, 2) == 0) uart_printf("ID      = 0x%02X%02X\r\n", buf[0], buf[1]);
}

/* ---------------- Public API ---------------- */

void AD7175_Init(AD7175_Handle_t *h, SPI_HandleTypeDef *spi)
{
    (void)h;
    ad7175_hspi = spi;
    if (!ad7175_hspi) {
        uart_printf("AD7175 Init: SPI handle NULL!\r\n");
        Error_Handler();
    }

    uart_printf("\r\n==== Inicializando AD7175-2 (polling, debug) ====\r\n");

    /* Reset interface physically (64 SCLKs) */
    AD7175_ResetToDefaults();

    /* Optional: full-register reset to datasheet defaults (if you have that function) */
    AD7175_FullRegisterReset();

    /* small delay */
    HAL_Delay(5);

    /* Ensure CHMAP0..3 are cleared before configuring */
    uint8_t zero[2] = {0x00, 0x00};
    for (int r = 0x10; r <= 0x13; ++r) {
        reg_write_rb(r, zero, 2);
    }

    /* Read ID (reg 0x07) */
    uint8_t id[2] = {0};
    if (reg_read(0x07, id, 2) == 0) {
        uint16_t idv = ((uint16_t)id[0] << 8) | id[1];
        uart_printf("ID = 0x%04X\r\n", idv);
    } else {
        uart_printf("ID read failed\r\n");
    }

    /* IFMODE: DATA+STATUS only (no CONTREAD yet) */
    uint8_t ifmode[2] = {0x00, 0x40};
    if (reg_write_rb(0x02, ifmode, 2) < 0) {
        uart_printf("WARN: IFMODE write/readback failed\r\n");
    }

    /* ADCMODE: REF_EN=1, continuous, internal clock (0x8000) */
    uint8_t adcmode[2] = {0x80, 0x00};
    reg_write_rb(0x01, adcmode, 2);

    /* SETUP0/SETUP1: UNIPOLAR + input buffer enabled -> 0x0300 */
    uint8_t setup0[2] = {0x03, 0x00};
    uint8_t setup1[2] = {0x03, 0x00};
    reg_write_rb(0x20, setup0, 2);
    reg_write_rb(0x21, setup1, 2);


//    /* FILTER0/FILTER1: ODR ~200 SPS -> ODR=0x0D, SINC5+SINC1 -> 0x000D */
//    uint8_t filt0[2] = {0x00, 0x0D};
//    uint8_t filt1[2] = {0x00, 0x0D};

    /* FILTER0/FILTER1: ODR = 500 SPS total (250 SPS por canal) */
    uint8_t filt0[2] = {0x00, 0x0B};
    uint8_t filt1[2] = {0x00, 0x0B};

    reg_write_rb(0x28, filt0, 2);
    reg_write_rb(0x29, filt1, 2);

    /* Finally enable both channels simultaneously */
	if (AD7175_Enable_CH0_CH1() < 0) {
		  uart_printf("Warning: AD7175_Enable_CH0_CH1 returned error\r\n");
	}

    /* final dump for debug */
    AD7175_DumpRegisters();

    uart_printf("Init done (unipolar, ext 5V, ~200SPS, CH0/CH1) - debug enabled\r\n");
}

/* Poll DRDY via MISO pin and read DATA register (0x04) with IFMODE=DATA_STAT
   Implementation: assert CS low, poll MISO (HAL_GPIO_ReadPin) until low (RDY),
   then clock out 4 bytes via SPI while CS still low. */
int AD7175_PollAndRead(uint32_t timeout_ms, uint8_t *channel, uint32_t *raw)
{
    uint32_t t0 = HAL_GetTick();

    /* CS baixo antes de checar RDY */
    cs_low();

    /* Poll em RDY via MISO (ativo em low) */
    while (1) {
        if (HAL_GPIO_ReadPin(AD7175_MISO_PORT, AD7175_MISO_PIN) == GPIO_PIN_RESET) {
            /* debounce tosco */
            for (volatile int i = 0; i < 200; ++i) __NOP();
            if (HAL_GPIO_ReadPin(AD7175_MISO_PORT, AD7175_MISO_PIN) == GPIO_PIN_RESET)
                break;
        }
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            cs_high();
            return -1; /* timeout */
        }
        __NOP();
    }

    /* Ler DATA (0x04) com DATA_STAT=1:
       - 1 byte comando 0x44
       - 4 bytes válidos: DATA[23:16], DATA[15:8], DATA[7:0], STATUS
       Total: 5 bytes TX/RX
    */
    uint8_t tx[5] = { 0x44, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rx[5] = { 0 };

    if (spi_txrx(tx, rx, 5) < 0) {
        cs_high();
        uart_printf("SPI read error during data read\r\n");
        return -2; /* spi error */
    }

    cs_high();

    /* rx[0] = lixo durante o envio do comando
       rx[1..3] = DATA[23:0]
       rx[4]    = STATUS
    */
    uint32_t val   = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
    uint8_t  status = rx[4];

    //uart_printf("DBG: STATUS=0x%02X DATA=0x%06X\r\n", status, val);

    /* Verifica erro de ADC (bit6) */
    if (status & (1<<6)) {
        //uart_printf("WARN: ADC status error bit set (0x%02X) — usando valor mesmo assim\r\n", status);
        // NÃO retorna erro — continua
    }

    /* Datasheet: bits [1:0] = CHANNEL (0..3), bits [3:2] reservados */
    uint8_t chid = status & 0x03;

    //uart_printf("DBG_CHID(STATUS[1:0]) = %u\r\n", chid);

    if (channel) *channel = chid;
    if (raw)     *raw     = val & 0xFFFFFFu;

    /* Atualiza variáveis globais */
    ad7175_last_channel = chid;
    ad7175_last_raw     = val & 0xFFFFFFu;
    ad7175_new_sample   = 1;

    return 0;
}


double AD7175_ConvertToMilliVolts(uint32_t raw)
{
    double code = (double)(raw & 0xFFFFFFu);
    return (code / 16777215.0) * AD7175_VREF_MV;
}
