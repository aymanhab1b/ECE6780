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
#include "stm32f0xx.h"
uint32_t button_count;
uint32_t led_state = 0; // To determine if the LED1 is ON
uint32_t counter = 0; // To check if the button has been pressed
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
int main(void)
{
  /* USER CODE BEGIN 1 */
		// Enable TIM2 clock
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		// Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		// Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		TIM2->PSC = 7999; // Prescaler value tused to change clock frequency to an appropriate scale
    TIM2->ARR = 250; // Auto-reload value (ARR) holds the number of units to get to the target value
		// Enable update event interrupt
    TIM2->DIER |= TIM_DIER_UIE;
		// Start TIM2
    TIM2->CR1 |= TIM_CR1_CEN;
		// Enable TIM2 interrupt in NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
		// Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		// Configure GPIO pins
		GPIOC->ODR |= (1<<8);
		GPIOC->ODR &= ~(1<<9);
		GPIOC->MODER |= (0x5A << 12);
		GPIOC->OTYPER &= ~(0xF << 6);
		GPIOC->OSPEEDR |= (0xFF << 12);
		GPIOC->PUPDR &= ~ (0XFF << 12);
		// Configure prescaler and auto-reload register for TIM3 for a frequency of 800Hz
    TIM3->PSC = 7; // Prescaler value (PSC)
    TIM3->ARR = 1250;    // Auto-reload value (ARR)
		// Configure CCMR1 for PWM mode for channels 1 and 2
    TIM3->CCMR1 &= ~((0x3 << 8) | (0x3 << 0)); 
		TIM3->CCMR1 |=(0x7 <<4);// PWM mode 2 for channel 1
    TIM3->CCMR1 |= (0x6 <<12); // PWM mode 1 for channel 2
		// Enable output compare preload for channels 1 and 2
    TIM3->CCMR1 |= (1 << 3 | 1 << 11);
		// Enable output for channels 1 and 2
    TIM3->CCER |= (1 << 0 | 1 << 4);
		// Set capture/compare register value for both channels to 250
    TIM3->CCR1 = 250;
    TIM3->CCR2 = 250;		
		// Configure PC6 and PC7 to use AF1 (alternate function 1), which corresponds to Timer 3
    GPIOC->AFR[0] &= ~(0xF << 24); // Set AF1 for PC6
    GPIOC->AFR[0] &= ~(0xF << 28); // Set AF1 for PC7
		// Enable TIM3
    TIM3->CR1 |= (0x1<<0);
		
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

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void TIM2_IRQHandler(void) 
	{
        // Clear update event interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;
				GPIOC->ODR ^= (1 << 8) | (1 << 9); // Toggling Orange and Green LED
	}


/**
  * @brief System Clock Configuration
  * @retval None
  */
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
