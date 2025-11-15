#include "ad7175_driver.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;
extern void Error_Handler(void);

#define DBG_PRINTF(fmt, ...) do { char _b[128]; int _l=snprintf(_b,sizeof(_b),fmt,##__VA_ARGS__); HAL_UART_Transmit(&huart2,(uint8_t*)_b,_l,HAL_MAX_DELAY);} while(0)
#define DBG_LOG(msg) HAL_UART_Transmit(&huart2,(uint8_t*)(msg),strlen(msg),HAL_MAX_DELAY)

/* SPI config */
#define AD7175_SPI_TIMEOUT   1000
#define AD7175_BAUDRATE_DIV  SPI_BAUDRATEPRESCALER_64
#define AD7175_RESET_MS_DELAY 500

/* SPI reference */
static SPI_HandleTypeDef *ad7175_hspi = NULL;

/* -------------------------------------------------------------------------- */
/* Low-level SPI helpers                                                      */
/* -------------------------------------------------------------------------- */
static int spi_txrx(uint8_t *tx, uint8_t *rx, uint16_t n)
{
    HAL_GPIO_WritePin(AD7175_CS_PORT, AD7175_CS_PIN, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(ad7175_hspi, tx, rx, n, AD7175_SPI_TIMEOUT);
    HAL_GPIO_WritePin(AD7175_CS_PORT, AD7175_CS_PIN, GPIO_PIN_SET);
    return (st == HAL_OK) ? 0 : -1;
}

static int spi_tx(uint8_t *tx, uint16_t n)
{
    HAL_GPIO_WritePin(AD7175_CS_PORT, AD7175_CS_PIN, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(ad7175_hspi, tx, n, AD7175_SPI_TIMEOUT);
    HAL_GPIO_WritePin(AD7175_CS_PORT, AD7175_CS_PIN, GPIO_PIN_SET);
    return (st == HAL_OK) ? 0 : -1;
}

static void AD7175_ReconfigureSPI(void)
{
    ad7175_hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
    ad7175_hspi->Init.CLKPhase    = SPI_PHASE_2EDGE;
    ad7175_hspi->Init.NSS         = SPI_NSS_SOFT;
    ad7175_hspi->Init.FirstBit    = SPI_FIRSTBIT_MSB;
    ad7175_hspi->Init.BaudRatePrescaler = AD7175_BAUDRATE_DIV;
    HAL_SPI_Init(ad7175_hspi);
}

/* -------------------------------------------------------------------------- */
/* Basic register I/O (no CRC)                                                */
/* -------------------------------------------------------------------------- */
static int reg_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[1+4];
    buf[0] = reg & 0x3F;
    memcpy(&buf[1], data, len);
    return spi_tx(buf, len+1);
}

static int reg_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t tx[1+4] = {0}, rx[1+4] = {0};
    tx[0] = 0x40 | (reg & 0x3F);
    if (spi_txrx(tx, rx, len+1) < 0) return -1;
    memcpy(data, &rx[1], len);
    return 0;
}

/* -------------------------------------------------------------------------- */
/* Reset and status wait                                                      */
/* -------------------------------------------------------------------------- */
void AD7175_Reset(void)
{
    uint8_t ones[8];
    memset(ones, 0xFF, sizeof(ones));
    AD7175_ReconfigureSPI();
    spi_tx(ones, sizeof(ones));
    HAL_Delay(AD7175_RESET_MS_DELAY);
}

/* Wait DRDY flag */
static int wait_drdy(uint32_t timeout_ms)
{
    uint8_t st;
    while (timeout_ms--) {
        if (reg_read(0x00, &st, 1) == 0 && (st & 0x80) == 0)
            return 0;
        HAL_Delay(1);
    }
    DBG_LOG("⚠ Timeout DRDY\r\n");
    return -1;
}

/* -------------------------------------------------------------------------- */
/* Initialization sequence (verified functional)                              */
/* -------------------------------------------------------------------------- */
void AD7175_Init(AD7175_Handle_t *hadc, SPI_HandleTypeDef *hspi)
{
    ad7175_hspi = hspi;
    DBG_LOG("==== Inicializando AD7175-2 ====\r\n");
    if (!ad7175_hspi) Error_Handler();

    AD7175_Reset();

    /* Read ID */
    uint8_t id[2];
    reg_read(0x07, id, 2);
    uint16_t id_val = ((uint16_t)id[0] << 8) | id[1];
    DBG_PRINTF("ID = 0x%04X\r\n", id_val);
    if ((id_val & 0xFFF0) != 0x0CD0)
        DBG_LOG("⚠️ ID incorreto!\r\n");


    /* IFMODE = 0x0000 */
	uint8_t ifmode[2] = {0x00, 0x00};
	reg_write(0x02, ifmode, 2);

	reg_read(0x02, ifmode, 2);
	DBG_PRINTF("IFMODE READBACK = 0x%02X %02X\r\n", ifmode[0], ifmode[1]);

	/* ADCMODE = 0x8000: REF_EN=1, modo contínuo, clock interno */
	uint8_t adcmode[2] = {0x80, 0x00};
	reg_write(0x01, adcmode, 2);

	reg_read(0x01, adcmode, 2);
	DBG_PRINTF("ADCMODE READBACK = 0x%02X %02X\r\n", adcmode[0], adcmode[1]);

	/* SETUPCON0: */
//	uint8_t setup0[2] = { 0x13, 0x00 }; //BIPOLAR

	uint8_t setup0[2] = { 0x03, 0x00 };   // 0x0300
	reg_write(0x20, setup0, 2);

	reg_read(0x20, setup0, 2);
	DBG_PRINTF("SETUPCON0 READBACK = 0x%02X %02X\r\n", setup0[0], setup0[1]);


	/* FILTCON0: */
	uint8_t filt0[2] = { 0x05, 0x11 };
	reg_write(0x28, filt0, 2);


	reg_read(0x28, filt0, 2);
	DBG_PRINTF("FILTCON0 READBACK = 0x%02X %02X\r\n", filt0[0], filt0[1]);

	/* CHMAP0 = 0x8001 (enable CH0, AIN0+/AIN1−, setup0) */
	uint8_t ch0[2] = {0x80, 0x01};
	reg_write(0x10, ch0, 2);

	reg_read(0x10, ch0, 2);
	DBG_PRINTF("CHMAP0 READBACK = 0x%02X %02X\r\n", ch0[0], ch0[1]);


    DBG_LOG("✅ AD7175 configurado (AIN0+/AIN1−, ref=5V, 20SPS)\r\n");
}

/* -------------------------------------------------------------------------- */
/* Data read / conversion                                                     */
/* -------------------------------------------------------------------------- */
int32_t AD7175_ReadRaw(AD7175_Handle_t *hadc, uint32_t *raw)
{
    if (wait_drdy(500) != 0) return -1;

    uint8_t tx[4] = {0x44, 0, 0, 0};
    uint8_t rx[4] = {0};
    if (spi_txrx(tx, rx, 4) < 0) return -2;

    uint32_t val = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
    *raw = val & 0xFFFFFF;
    return 0;
}

uint32_t AD7175_RawUnsigned(uint32_t raw)
{
    return (raw & 0xFFFFFF);  // 0..2^24-1
}


double AD7175_ConvertToMilliVolts(uint32_t raw)
{
    uint32_t code = raw & 0xFFFFFF;          // 0 .. 16777215
    double v = ((double)code / 16777215.0) * AD7175_VREF_MV;  // 0 .. Vref
    return v;
}
