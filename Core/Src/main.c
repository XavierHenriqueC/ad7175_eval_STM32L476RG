#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "ad7175_driver.h"
#include "uart_printf.h"

#include "core_cm4.h" /* DWT */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t micros(void)
{
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    return DWT->CYCCNT / (hclk / 1000000U);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI2_Init();

    dwt_init();

    uart_printf("\r\n==== Iniciando sistema (multicanal) ====");

    AD7175_Handle_t hadc;
    AD7175_Init(&hadc, &hspi2);

    uart_printf("t_us,chan,raw_dec,raw_hex,mV");

    while (1)
    {
        uint8_t ch;
        uint32_t raw;
        int r = AD7175_PollAndRead(500, &ch, &raw);
        if (r == 0) {
            double mv = AD7175_ConvertToMilliVolts(raw);
            uart_printf("%lu,%u,%lu,0x%06lX,%.6f",
                       (unsigned long)micros(), (unsigned)ch,
                       (unsigned long)AD7175_RawUnsigned(raw),
                       (unsigned long)AD7175_RawUnsigned(raw),
                       mv);
        } else if (r == -1) {
            uart_printf("ERR_POLL -1 (timeout DRDY)");
        } else if (r == -2) {
            uart_printf("ERR_POLL -2 (SPI error)");
        } else if (r == -3) {
            uart_printf("ERR_POLL -3 (ADC_ERROR)");
        }
        /* small sleep to avoid extreme CPU usage; removal is possible if needed */
        HAL_Delay(1);
    }
}

/* -------------------------------------------------------------------------- */
/* SystemClock_Config / Error_Handler – mantenha a mesma implementação que
 * já está funcionando no seu projeto (CubeMX / HAL gerados).
 * -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/* System Clock (não alterado)                                                */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
