/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
volatile char a;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void I2C_read()
{
		I2C2->CR2 |= (0x69 << 1);
		I2C2->CR2 |= (1<<16);
		I2C2->CR2 |= (1<<10);
		I2C2->CR2 |= (1<<13);
	while((((I2C2->ISR & (1<<2)) == 0) & ((I2C2->ISR & (1<<4)) == 0)))
	{
		if ((I2C2->ISR & (1<<4)) == 1)
		{
			GPIOC->ODR |=(1<<6);
			continue;
		}
		else if ((I2C2->ISR & (1<<2)) == 1)
		{
			break;
		}
	}
	while ((I2C2->ISR & (1<<6)) == 0)
			{
			}
		if (I2C2->RXDR == 0xD3)
		{
			GPIOC->ODR |=(1<<7);
		}
		I2C2->CR2 |= (1<<14); // STOP condition
		
}
	
	
//Writing the Slave Address
void I2C_write()
{
		I2C2->CR2 |= (0x69 << 1);
		I2C2->CR2 |= (1<<16);
		I2C2->CR2 &= ~(1<<10);
		I2C2->CR2 |= (1<<13);
	while((((I2C2->ISR & (1<<4)) == 0) & ((I2C2->ISR & (1<<1)) == 0)))
	{
		if ((I2C2->ISR & (1<<4)) == 1)
		{
			GPIOC->ODR |=(1<<6);
			continue;
		}
		else if ((I2C2->ISR & (1<<1)) == 1)
		{
			break;
		}
	}		
			I2C2->TXDR = (0xF << 0);
			while ((I2C2->ISR & (1<<6)) == 0)
			{				
			}
			I2C_read();
}
int main(void)
{
  /* USER CODE BEGIN 1 */
		// Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		// Enable GPIOB clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		//Enable the clock for I2C2
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		//Setting PB11 to alternate function mode
		GPIOB->MODER |= (2 << 22); //Setting MODER to alternate function 
		GPIOB->OTYPER |= (1<<11); //Setting Output Open-Drain
		GPIOB->AFR[1] |= (1 << 12); //Setting to Alternate Function 1
		//Setting PB13 to alternate function mode
		GPIOB->MODER |= (2 << 26); //Setting MODER to alternate function
		GPIOB->OTYPER |= (1<<13); //Setting Output Open-Drain
		GPIOB->AFR[1] |= (5 << 20); //Setting to Alternate Function 5
		//Setting PB14
		GPIOB->MODER |= (1<<28); //Setting MODER to Output Mode
		GPIOB->OTYPER &= ~(1<<14); //Setting Output type to Push-Pull
		GPIOB->ODR  |= (1<<14); //Setting PB14 to high
		//Setting PC0
		GPIOC->MODER |= (1<<0); //Setting MODER to Output Mode
		GPIOC->OTYPER &= ~(1<<0); //Setting Output type to Push-Pull
		GPIOC->ODR |= (1<<0); //Setting PB14 to high
		//100kHz I2C2 Timing
		I2C2->TIMINGR |= (1<<28); //Set PRESC
		I2C2->TIMINGR |= (0x13<<0); //Set SCLL
		I2C2->TIMINGR |= (0xF<<8); //Set SCLH
		I2C2->TIMINGR |= (0x2<<16); //Set SDAEL
		I2C2->TIMINGR |= (0x4<<20); //Set SCLDEL
		I2C2->CR1 |= (1<<0); // Enabling the I2C Peripheral
		// Initializing GPIO LEDs
		GPIOC->MODER |= ((1<<12) | (1<<14) | (1<<16) | (1<<18));
		GPIOC->OTYPER = 0;
		GPIOC->OSPEEDR = 0;
		GPIOC->PUPDR = 0;
		GPIOC->ODR &= ~(1<<6);
		GPIOC->ODR &= ~(1<<7);
		GPIOC->ODR &= ~(1<<8);
		GPIOC->ODR &= ~(1<<9);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		I2C_write();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

//Reading the data

	
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
