/* -------------------------------------------------------------------------- */
/* main.c – Leitura AD7175-2 via SPI2 + CSV na USART2                         */
/* Autor: Henrique (ajustado por ChatGPT)                                     */
/* -------------------------------------------------------------------------- */

#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "ad7175_driver.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "core_cm4.h"   // Necessário para DWT (timestamp em µs)

/* Configurações */
#define CSV_INTERVAL_MS 200   // intervalo entre prints CSV

AD7175_Handle_t hadc7175;

/* -------------------------------------------------------------------------- */
/* UART printf                                                                */
/* -------------------------------------------------------------------------- */
static void uart_printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;

    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, HAL_MAX_DELAY);
}

/* -------------------------------------------------------------------------- */
/* DWT - Timestamp em microssegundos                                          */
/* -------------------------------------------------------------------------- */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // habilita DWT
    DWT->CYCCNT = 0;                                // zera contador
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // liga contador
}

static uint32_t micros(void)
{
    uint32_t hclk = HAL_RCC_GetHCLKFreq();          // ex: 80 MHz no L476
    return DWT->CYCCNT / (hclk / 1000000U);
}

/* -------------------------------------------------------------------------- */
/* main                                                                        */
/* -------------------------------------------------------------------------- */
int main(void)
{
    /* Inicialização básica STM32 */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI2_Init();

    /* Timestamp em microssegundos */
    dwt_init();

    uart_printf("\r\n==== Iniciando sistema ====\r\n");

    /* Inicializa AD7175 */
    AD7175_Init(&hadc7175, &hspi2);

    HAL_Delay(100);

    /* Cabeçalho CSV */
    uart_printf("t_us,raw_dec,raw_hex,mV\r\n");

    /* Loop principal */
    while (1)
    {
        uint32_t raw = 0;

        if (AD7175_ReadRaw(&hadc7175, &raw) == 0)
        {
            uint32_t t_us = micros();
            double mv     = AD7175_ConvertToMilliVolts(raw);
            uint32_t raw_dec = AD7175_RawUnsigned(raw);

            uart_printf("%lu,%lu,%.6f\r\n",
                        (unsigned long)t_us,
                        (unsigned long)raw_dec,
                        mv);
        }
        else
        {
            uart_printf("0,0,0x000000,0.0\r\n");
        }

        HAL_Delay(CSV_INTERVAL_MS);
    }
}

/* -------------------------------------------------------------------------- */
/* System Clock (não alterado)                                                */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

/* -------------------------------------------------------------------------- */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        HAL_Delay(200);
    }
}
