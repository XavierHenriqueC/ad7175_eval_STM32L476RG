#ifndef AD7175_DRIVER_H
#define AD7175_DRIVER_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define AD7175_CS_PORT   GPIOB
#define AD7175_CS_PIN    GPIO_PIN_12

#define AD7175_MISO_PORT GPIOC
#define AD7175_MISO_PIN  GPIO_PIN_2

#define AD7175_VREF_MV   5000.0   /* mV */

#define AD7175_SPI_TIMEOUT 1000

typedef struct {
    SPI_HandleTypeDef *hspi;
} AD7175_Handle_t;

/* Initialize AD7175:
 * - CH0 = AIN0+/AIN1- (setup0)
 * - CH1 = AIN2+/AIN3- (setup1)
 * - unipolar, external ref, ~200 SPS (filter ODR 0x0D)
 */
void AD7175_Init(AD7175_Handle_t *h, SPI_HandleTypeDef *spi);

/* Poll DRDY (via MISO) and read STATUS + 24-bit DATA
 * Returns:
 *  0 = OK
 * -1 = timeout waiting DRDY
 * -2 = SPI error on read
 * -3 = ADC_ERROR reported in STATUS (bit6)
 */
int AD7175_PollAndRead(uint32_t timeout_ms, uint8_t *channel, uint32_t *raw);

/* Convert raw (24-bit) to millivolts (unipolar, 0..Vref) */
double AD7175_ConvertToMilliVolts(uint32_t raw);

/* Raw unsigned helper */
static inline uint32_t AD7175_RawUnsigned(uint32_t raw) { return raw & 0xFFFFFFu; }

#endif /* AD7175_DRIVER_H */
