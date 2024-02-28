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
volatile char rec;
volatile char global_flag =0;
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
	void USARTAyman(char a)
{
	uint32_t mask = (1<<7);
	while((USART3->ISR & mask) == 0)
	{
		
	}
	USART3->TDR = a;
}
	void Trans_Str(char *input)
	{
		uint32_t i = 0;
		while(input[i] != '\0')
		{
			USARTAyman(input[i]);
			i++;
		}		
	}
	char Read_Char()
	{
		uint32_t mask_2 = (1<<5);
		char value;
		while((USART3->ISR & mask_2) == 0)
		{
			
		}
		value = USART3->RDR;
		return value;
	}
	void USART3_4_IRQHandler()
	{
		rec = USART3->RDR;
		global_flag = 1;
	}
int main(void)
{
	HAL_Init();
	SystemClock_Config();
  /* USER CODE BEGIN 1 */
		// Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		// Enable Clock for USART3
		RCC->APB1ENR |= (1<<18);
		GPIOC->MODER |= ((1<<9) | (1<<11) | (0x55<<12));
		GPIOC->OTYPER &= ~(0xF << 6);
		GPIOC->OSPEEDR &= ~(0xFF << 12);
		GPIOC->PUPDR &= ~(0xFF << 12); 
		GPIOC->AFR[0] |= (0x11 << 16); 
		USART3->BRR = 69;
		USART3->CR1 |= (0x1<<2); //Rx
		USART3->CR1 |= (0x1<<3); //Tx
		USART3->CR1 |= (0x1<<5); //RXNE
		USART3->CR1 |= (0X1<<0); //Enable
		NVIC_EnableIRQ(USART3_4_IRQn); //29
		NVIC_SetPriority(USART3_4_IRQn,1); // set priority
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	char color;
	char number;
	char led_Pos = 0;
	char state = 0;
  while (1)
  {
    /* USER CODE END WHILE */
			//Trans_Str("Hello World");
			
			char rec_value = Read_Char();
		  /*USARTAyman(rec_value);
			if (rec_value == 'r')
			{
				GPIOC->ODR^=(1<<6);
			}
			if (rec_value == 'b')
			{
				GPIOC->ODR^=(1<<7);
			}
			if (rec_value == 'o')
			{
				GPIOC->ODR^=(1<<8);
			}
			if (rec_value == 'g')
			{
				GPIOC->ODR^=(1<<9);
			}
			else
				Trans_Str("E"); */
				
				/*if (global_flag == 1)
				{
					char state+=1;
					if (state % 2 == 0)
					{
						char rec_value = Read_Char();
						color = rec_value;
						global_flag = 0;
					}
					else
					{
						char rec_value = Read_Char();
						number = rec_value;
						global_flag = 0;
					}
				}*/
				while (global_flag == 0)
				{
					
				}
				if (state % 2 == 0)
				{
					Trans_Str("CMD?");
					state = state + 1;
					color = rec_value;
					global_flag = 0;
					switch(color)
					{
						case 'r': 
						{
							led_Pos = 6;
							Trans_Str("You pressed red");
						}
						break;
						case 'b' : 
						{
							led_Pos = 7;
							Trans_Str("You pressed blue");
						}
						break;
						case 'o' : 
						{
							led_Pos = 8;
							Trans_Str("You pressed orange");
						}
						break;
						case 'g' : 
						{
							led_Pos = 9;
							Trans_Str("You pressed green");
						}
						break;
						default: 
						{
							Trans_Str("Error");
							continue;
						}
						break;
					}
				}
				else
				{
					Trans_Str("CMD?");
					state = state + 1;
					number = rec_value;
					global_flag = 0;
					switch(number)
					{
						case '1': 
						{
							GPIOC->ODR |= (1<<led_Pos);
							Trans_Str("You pressed ON");
						}
						break;
						case '2': 
						{
							GPIOC->ODR &= ~(1<<led_Pos);
							Trans_Str("You pressed OFF");
						}
						break;
						case '3': 
						{
							GPIOC->ODR ^= (1<<led_Pos);
							Trans_Str("You pressed Toggle");
						}
						break;
						default: 
						{
							Trans_Str("Error");
							continue;
						}
						break;
					}
				}
				
				
				
				
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
