#ifndef __AD7175_DRIVER_H__
#define __AD7175_DRIVER_H__

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* Configurações gerais                                                        */
/* -------------------------------------------------------------------------- */

/* Valor da referência externa (REF+ − REF−) em mV */
#define AD7175_VREF_MV   5000.0    // 5.000 V

/* Pino de CS do AD7175 (configure conforme seu hardware) */
#ifndef AD7175_CS_PORT
#define AD7175_CS_PORT   GPIOB
#define AD7175_CS_PIN    GPIO_PIN_12
#endif

/* Timeout SPI */
#define AD7175_SPI_TIMEOUT   1000


/* -------------------------------------------------------------------------- */
/* Estrutura do ADC (mínima, suficiente)                                      */
/* -------------------------------------------------------------------------- */
typedef struct
{
    SPI_HandleTypeDef *hspi;   // handler SPI usado pelo driver
} AD7175_Handle_t;


/* -------------------------------------------------------------------------- */
/* API pública                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Reseta o AD7175 enviando 64 clocks com MOSI=1
 */
void AD7175_Reset(void);

/**
 * @brief Inicializa o AD7175 (registradores, modo contínuo, CH0)
 */
void AD7175_Init(AD7175_Handle_t *hadc, SPI_HandleTypeDef *hspi);

/**
 * @brief Lê um valor RAW 24 bits do registrador DATA
 * @return 0 em sucesso, <0 erros
 */
int32_t AD7175_ReadRaw(AD7175_Handle_t *hadc, uint32_t *raw);

/**
 * @brief Retorna RAW em formato não assinado (0...2^24−1)
 */
uint32_t AD7175_RawUnsigned(uint32_t raw);

/**
 * @brief Converte RAW bipolar offset-binary para mV
 *        Faixa: −Vref → +Vref
 */
double AD7175_ConvertToMilliVolts(uint32_t raw);

#endif /* __AD7175_DRIVER_H__ */
