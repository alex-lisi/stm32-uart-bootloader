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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define APPLICATION_ADDRESS   0x08008000

void bootloader_jump_to_application(void)
{
	// Function pointer type for void func(void)
	typedef void (*pFunction)(void);
	pFunction jump_to_application;
	uint32_t jump_address;

	// Get the application's reset handler address
	// From PM0214 page 40: Vector table structure
	// Offset 0x00: Initial Stack Pointer
	// Offset 0x04: Reset Handler
	jump_address = *(__IO uint32_t*)(APPLICATION_ADDRESS + 4);

	// Create function pointer
	jump_to_application = (pFunction)jump_address;

	// Deinitialize HAL (important!)
	HAL_DeInit();

	// Reset clock configuration to default (HSI)
	HAL_RCC_DeInit();

	// Disable and reset SysTick timer
	// This fixes HAL_Delay() timing issues in the application
	SysTick->CTRL = 0;      // Disable SysTick
	SysTick->LOAD = 0;      // Clear reload value
	SysTick->VAL = 0;       // Clear current value

	// Disable all interrupts
	// From PM0214 page 24: PRIMASK register
	__disable_irq();

	// Relocate vector table to application address
	// From RM0090 page 199: VTOR register
	SCB->VTOR = APPLICATION_ADDRESS;

	// Set the stack pointer to the application's initial stack pointer
	// From PM0214 page 40: First word at application address is stack pointer
	// From PM0214 page 29: MSP (Main Stack Pointer)
	__set_MSP(*(__IO uint32_t*)APPLICATION_ADDRESS);

	// Jump to application!
	jump_to_application();
}

uint8_t bootloader_erase_flash(void)
{
    HAL_StatusTypeDef status;
    uint32_t sector_error = 0;

    // From RM0090 page 75: Flash sector organization
    // Application uses sectors 2-11 (0x08008000 onwards)

    FLASH_EraseInitTypeDef erase_config;
    erase_config.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_config.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 2.7-3.6V
    erase_config.Sector = FLASH_SECTOR_2;               // Start sector
    erase_config.NbSectors = 10;                         // Erase sectors 2-11

    // Unlock flash for writing
    HAL_FLASH_Unlock();

    // Erase the sectors
    status = HAL_FLASHEx_Erase(&erase_config, &sector_error);

    // Lock flash again
    HAL_FLASH_Lock();

    if(status == HAL_OK)
    {
        return 1;  // Success
    }
    else
    {
        return 0;  // Failed
    }
}

uint8_t bootloader_write_flash(uint32_t address, uint8_t *data, uint32_t length)
{
    HAL_StatusTypeDef status;

    // Unlock flash
    HAL_FLASH_Unlock();

    // Clear any pending flash errors
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // Write data word by word
    for(uint32_t i = 0; i < length; i += 4)
    {
        uint32_t word_data;
        uint32_t bytes_left = length - i;

        if(bytes_left >= 4)
        {
            word_data = (uint32_t)data[i] |
                       ((uint32_t)data[i+1] << 8) |
                       ((uint32_t)data[i+2] << 16) |
                       ((uint32_t)data[i+3] << 24);
        }
        else
        {
            word_data = 0xFFFFFFFF;
            for(uint32_t j = 0; j < bytes_left; j++)
            {
                word_data &= ~(0xFF << (j * 8));
                word_data |= ((uint32_t)data[i + j] << (j * 8));
            }
        }

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word_data);

        if(status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0;
        }
    }

    HAL_FLASH_Lock();
    return 1;
}

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Turn on orange LED to show bootloader is running
  HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);

  // Send message immediately on boot
  char test_msg[] = "Bootloader starting...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, sizeof(test_msg)-1, 1000);

  // Small delay to let button settle
  HAL_Delay(100);

  // Check if USER button is pressed
  if(HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET)
  {
      // Button is pressed - stay in bootloader mode
      // Orange LED stays ON, we never jump to application

	  if(HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET)
	  {
	      // Button is pressed - stay in bootloader mode
		  // Orange LED stays ON, we never jump to application

		    // Send welcome message over UART
		    char* msg = "\r\n========================================\r\n";
		    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

		    msg = "  STM32 Bootloader v1.0\r\n";
		    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

		    msg = "  Ready for firmware update\r\n";
		    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

		    msg = "========================================\r\n";
		    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

		    msg = "\r\nCommands:\r\n";
		    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

		    msg = "  E - Erase application flash\r\n";
		    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

		    msg = "  W - Write firmware (Python script)\r\n";
		    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

		    msg = "  T - Test flash write\r\n";
		    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

		    msg = "  R - Reset to application\r\n";
		    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

		    msg = "\r\nWaiting for commands...\r\n\r\n";
		    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
	  }
  }
  else
  {
      // Button not pressed - jump to application after delay
      HAL_Delay(2000);

      // Turn off orange LED before jumping
      HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);

      // Jump to application
      bootloader_jump_to_application();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // If we're here, we're in bootloader mode
	  // Wait for data from UART
	  uint8_t received_cmd;

	  if(HAL_UART_Receive(&huart2, &received_cmd, 1, 100) == HAL_OK)
	  {
	        // Check which command
	        if(received_cmd == 'E' || received_cmd == 'e')
	        {
	            // ERASE command
	            char* msg = "Erasing application flash...\r\n";
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

	            // Erase flash
	            uint8_t result = bootloader_erase_flash();

	            if(result == 1)
	            {
	                msg = "Erase complete - OK\r\n";
	                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
	            }
	            else
	            {
	                msg = "Erase failed - ERROR\r\n";
	                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
	            }

	            // Blink orange LED
	            HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
	        }
	        else if(received_cmd == 'W' || received_cmd == 'w')
	        {
	            // WRITE command - simplified protocol
	            // Send immediate ACK so Python knows we're ready
	            char* ack = "ACK\r\n";
	            HAL_UART_Transmit(&huart2, (uint8_t*)ack, strlen(ack), 1000);

	            // Now receive 8-byte header (4 addr + 4 len) with SHORT timeout
	            uint8_t header[8];
	            HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, header, 8, 2000);

	            if(status != HAL_OK)
	            {
	                char* msg = "HDR_TIMEOUT\r\n";
	                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
	                continue;
	            }

	            // Parse header
	            uint32_t address = (header[0]) | (header[1] << 8) | (header[2] << 16) | (header[3] << 24);
	            uint32_t length = (header[4]) | (header[5] << 8) | (header[6] << 16) | (header[7] << 24);

	            // Sanity check
	            if(length == 0 || length > 1024)
	            {
	                char msg[50];
	                sprintf(msg, "BAD_LEN:%lu\r\n", length);
	                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
	                continue;
	            }

	            // Send ACK for header
	            ack = "HDR_OK\r\n";
	            HAL_UART_Transmit(&huart2, (uint8_t*)ack, strlen(ack), 1000);

	            // Receive data with timeout based on length
	            uint8_t data_buffer[1024];
	            uint32_t timeout = 2000 + (length / 10);  // 2s base + extra for data
	            status = HAL_UART_Receive(&huart2, data_buffer, length, timeout);

	            if(status != HAL_OK)
	            {
	                char* msg = "DATA_TIMEOUT\r\n";
	                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
	                continue;
	            }

	            // Write to flash (without debug spam)
	            uint8_t result = bootloader_write_flash(address, data_buffer, length);

	            if(result == 1)
	            {
	                // Success - just send OK
	                char* msg = "OK\r\n";
	                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
	            }
	            else
	            {
	                char* msg = "WRITE_FAIL\r\n";
	                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
	            }

	            HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
	        }
	        else if(received_cmd == 'T' || received_cmd == 't')
	        {
	            // TEST command - write simple test pattern
	            char* msg = "Testing flash write with pattern...\r\n";
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

	            // Simple test data
	            uint8_t test_pattern[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
	                                         0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00};

	            // Write directly using HAL
	            HAL_FLASH_Unlock();

	            HAL_StatusTypeDef status;
	            for(int i = 0; i < 4; i++)  // Write 4 words (16 bytes)
	            {
	                uint32_t word = (test_pattern[i*4]) |
	                               (test_pattern[i*4+1] << 8) |
	                               (test_pattern[i*4+2] << 16) |
	                               (test_pattern[i*4+3] << 24);

	                status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APPLICATION_ADDRESS + (i*4), word);

	                char status_msg[50];
	                sprintf(status_msg, "Word %d: status=%d\r\n", i, status);
	                HAL_UART_Transmit(&huart2, (uint8_t*)status_msg, strlen(status_msg), 1000);
	            }

	            HAL_FLASH_Lock();

	            msg = "Test write complete\r\n";
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

	            HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
	        }
	        else if(received_cmd == 'R' || received_cmd == 'r')
	        {
	            // RESET command - jump to application
	            char* msg = "Resetting to application...\r\n";
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

	            HAL_Delay(100);

	            // Turn off orange LED
	            HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);

	            // Jump to application
	            bootloader_jump_to_application();
	        }
	        else
	        {
	            // Unknown command - echo back
	            char response[50];
	            sprintf(response, "Received: 0x%02X - OK\r\n", received_cmd);
	            HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 1000);

	            HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
	        }
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

	// Enable USART2 clock
	__HAL_RCC_USART2_CLK_ENABLE();

	// Enable GPIOA clock for UART pins
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Configure PA2 (TX) and PA3 (RX)
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_ORANGE_Pin */
  GPIO_InitStruct.Pin = LED_ORANGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ORANGE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
