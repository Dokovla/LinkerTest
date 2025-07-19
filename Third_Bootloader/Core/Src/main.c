/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*pFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
#define APP_ADDRESS 0x08004000 //Origin of main app.
//#define FLASH_PAGE_SIZE    2048 -
#define MAX_APP_FLASH_SIZE  (256 * 1024 - 16 * 1024) // bootloader 16KB
#define UART_TIMEOUT        1000

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void jump_to_app(void);
uint8_t is_app_valid(void);
void receive_firmware(void);
void erase_app_flash(void);
uint32_t calculate_crc32(uint8_t *data, uint32_t length);  // CRC funkcija


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//The origin of the code and Vector table has offset, defined in .ld
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  uint8_t received = 0;
  const char *msg = "BOOT... Wait U-update, E-erase ...\r\n";
  HAL_UART_Transmit(&huart4, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  HAL_UART_Receive(&huart4, &received, 1, 10000);  // Wait 3s

  if (received == 'U')
  {
      // FW update
      receive_firmware();

      // After update, reset MCU
      NVIC_SystemReset();
  }
  else if (received == 'E')
  {
          erase_app_flash();
          NVIC_SystemReset();
  }
  else
  {
      if (is_app_valid()) {
          jump_to_app();
      } else {
          const char *msg = "No App!\r\n";
          HAL_UART_Transmit(&huart4, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
          while(1);
      }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MISOA7_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MISOA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DM_Pin DP_Pin */
  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t is_app_valid(void)
{
    uint32_t sp = *(volatile uint32_t *)APP_ADDRESS;
    return (sp >= 0x20000000 && sp <= 0x2000FFFF);  // RAM STM32F303
}

void jump_to_app(void)
{
    uint32_t app_sp = *(volatile uint32_t *)APP_ADDRESS;
    uint32_t app_reset = *(volatile uint32_t *)(APP_ADDRESS + 4);

    HAL_UART_DeInit(&huart4);
    HAL_RCC_DeInit();
    HAL_DeInit();

    __disable_irq();

    SCB->VTOR = APP_ADDRESS;
    __set_MSP(app_sp);
    ((pFunction)app_reset)();
}

void receive_firmware(void)
{
    HAL_UART_Transmit(&huart4, (uint8_t *)"Bootloader ready\r\n", 18, HAL_MAX_DELAY);

    // 1. Receive FW size
    uint32_t fw_total_size = 0;
    if (HAL_UART_Receive(&huart4, (uint8_t *)&fw_total_size, 4, UART_TIMEOUT) != HAL_OK ||
        fw_total_size == 0 || fw_total_size > MAX_APP_FLASH_SIZE)
    {
        HAL_UART_Transmit(&huart4, (uint8_t *)"Invalid firmware size\r\n", 24, HAL_MAX_DELAY);
        return;
    }

    // 2. Erase Flash block
    HAL_FLASH_Unlock();
    for (uint32_t addr = APP_ADDRESS; addr < (APP_ADDRESS + fw_total_size); addr += FLASH_PAGE_SIZE)
    {
        FLASH_EraseInitTypeDef eraseConfig = {0};
        uint32_t pageError = 0;

        eraseConfig.TypeErase = FLASH_TYPEERASE_PAGES;
        eraseConfig.PageAddress = addr;
        eraseConfig.NbPages = 1;

        if (HAL_FLASHEx_Erase(&eraseConfig, &pageError) != HAL_OK)
        {
            HAL_UART_Transmit(&huart4, (uint8_t *)"Flash erase fail\r\n", 19, HAL_MAX_DELAY);
            HAL_FLASH_Lock();
            return;
        }
    }

    // 3. Receive block and write to Flash
    uint8_t block_buffer[FLASH_PAGE_SIZE];
    uint32_t offset = 0;

    while (offset < fw_total_size)
    {
        uint32_t block_len = 0;
        uint32_t block_crc = 0;
        uint8_t retry = 1;

        while (retry)
        {
            retry = 0;

            // Block length
            if (HAL_UART_Receive(&huart4, (uint8_t *)&block_len, 4, UART_TIMEOUT) != HAL_OK ||
                block_len == 0 || block_len > FLASH_PAGE_SIZE || (offset + block_len > fw_total_size))
            {
                HAL_UART_Transmit(&huart4, (uint8_t *)"RESEND\r\n", 8, HAL_MAX_DELAY);
                retry = 1;
                continue;
            }

            // Receive CRC
            if (HAL_UART_Receive(&huart4, (uint8_t *)&block_crc, 4, UART_TIMEOUT) != HAL_OK)
            {
                HAL_UART_Transmit(&huart4, (uint8_t *)"RESEND\r\n", 8, HAL_MAX_DELAY);
                retry = 1;
                continue;
            }

            // Receive Data
            if (HAL_UART_Receive(&huart4, block_buffer, block_len, 2000) != HAL_OK)
            {
                HAL_UART_Transmit(&huart4, (uint8_t *)"RESEND\r\n", 8, HAL_MAX_DELAY);
                retry = 1;
                continue;
            }

            // CRC check
            uint32_t calc_crc = calculate_crc32(block_buffer, block_len);
            if (calc_crc != block_crc)
            {
                HAL_UART_Transmit(&huart4, (uint8_t *)"RESEND\r\n", 8, HAL_MAX_DELAY);
                retry = 1;
                continue;
            }
        }

        // Write block to flash
        uint32_t flash_addr = APP_ADDRESS + offset;
        for (uint32_t i = 0; i < block_len; i += 4)
        {
            uint32_t word = 0xFFFFFFFF;
            uint32_t remaining = block_len - i;
            if (remaining >= 4)
                memcpy(&word, &block_buffer[i], 4);
            else
                memcpy(&word, &block_buffer[i], remaining);  // padding - No 2kB

            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr, word) != HAL_OK)
            {
                HAL_UART_Transmit(&huart4, (uint8_t *)"RESEND\r\n", 8, HAL_MAX_DELAY);
                retry = 1;
                break;  // Retry
            }

            flash_addr += 4;
        }

        if (retry)
            continue;

        HAL_UART_Transmit(&huart4, (uint8_t *)"OK\r\n", 4, HAL_MAX_DELAY);
        offset += block_len;
    }

    HAL_FLASH_Lock();

    HAL_UART_Transmit(&huart4, (uint8_t *)"Firmware OK. Resetting...\r\n", 28, HAL_MAX_DELAY);
    HAL_Delay(100);
}

void erase_app_flash(void)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseConfig = {0};
    uint32_t pageError = 0;

    eraseConfig.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseConfig.PageAddress = APP_ADDRESS;
    eraseConfig.NbPages = MAX_APP_FLASH_SIZE / FLASH_PAGE_SIZE;

    if (HAL_FLASHEx_Erase(&eraseConfig, &pageError) != HAL_OK)
    {
    	const char *msg = "Flash erase fail\r\n";
    	HAL_UART_Transmit(&huart4, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
    else
    {
    	const char *msg = "Flash erased\r\n";
    	HAL_UART_Transmit(&huart4, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }

    HAL_FLASH_Lock();
}


uint32_t calculate_crc32(uint8_t *data, uint32_t length) //WO HW support
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            crc = (crc >> 1) ^ (0xEDB88320 * (crc & 1));
        }
    }
    return ~crc;
}
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

#ifdef  USE_FULL_ASSERT
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
