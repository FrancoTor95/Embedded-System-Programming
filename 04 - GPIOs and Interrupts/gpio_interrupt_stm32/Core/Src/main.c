/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
void processA(void);
void processB(void);

// Flag
volatile int b_pressed_flag = 0;

void EXTI2_IRQHandler(void){
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	b_pressed_flag = 1;
	EXTI->PR |= (0x01 << 2); // Clear the EXTI pending register
}

void EXTI15_10_IRQHandler(void){
  if(EXTI->PR & (0x01 << 13)){
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    b_pressed_flag = 2;
    EXTI->PR |= (0x01 << 13); // Clear the EXTI pending register
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  // Abilitazine clock per GPIOA e GPIOC
  RCC->AHB1ENR |= (0x1 << 0);
  RCC->AHB1ENR |= (0x1 << 2);

  /* Pulsanti su GPIOC13 e GPIOC2 */
  // GPIOC13 <-> PC_13 (USER_BUTTON)
  GPIOC->MODER &= ~(0x03 << 26); // Clear MODER GPIOC13
  GPIOC->MODER |= (0x00 << 26);  // Set Input
  GPIOC->PUPDR &= ~(0x03 << 26); // Clear PUPDR GPIOC13
  GPIOC->PUPDR |= (0x01 << 26);  // Set Pull-Up

  //GPIOC2 <-> PC_2
  GPIOC->MODER &= ~(0x03 << 4); // Clear MODER GPIOC2
  GPIOC->MODER |= (0x00 << 4);  // Set Input
  GPIOC->PUPDR &= ~(0x03 << 4); // Clear PUPDR GPIOC2
  GPIOC->PUPDR |= (0x01 << 4);  // Set Pull-Up

  /* LED su GPIOA5 e GPIOA10 */
  // GPIOA5 <-> PA_5 (LD2)
  GPIOA->MODER &= ~(0x03 << 10); // Clear MODER GPIOA5
  GPIOA->MODER |= (0x01 << 10);  // Set Output
  GPIOA->OTYPER &= ~(0x1 << 5);  // Output Push-Pull
  GPIOA->PUPDR &= ~(0x03 << 10); // Clear PUPDR GPIOA5
  GPIOA->PUPDR |= (0x01 << 10);  // Set Pull-Up
  GPIOA->ODR |= (0x1 << 5);      // Set GPIOA5 Output

  // GPIOA10 <-> PA_10
  GPIOA->MODER &= ~(0x03 << 20); // Clear MODER GPIOA10
  GPIOA->MODER |= (0x01 << 20);  // Set Output
  GPIOA->OTYPER &= ~(0x1 << 10); // Output Push-Pull
  GPIOA->PUPDR &= ~(0x03 << 20); // Clear PUPDR GPIOA10
  GPIOA->PUPDR |= (0x01 << 20);  // Set Pull-Up
  GPIOA->ODR |= (0x1 << 10);     // Set GPIO10 Output

  /* Gestione interrupts */
  // Abilitazione clock per SYSCFG (bit 14)
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC;	// External interrupt on GPIOC2
  SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;//0x0020U; // External interrupt on GPIOC13

  EXTI->IMR |= (0x01 << 13) | (0x01 << 2);     // Set not masked interrupt
  EXTI->RTSR |= (0x01 << 13) | (0x01 << 2);    // Rising Edge
  //EXTI->PR |= (0x01 << 13) | (0x01 << 2); // Clear the EXTI pending register

  // Abilitazione Interrupt
  __asm volatile ("cpsie i" : : : "memory"); // Change Processor State, Enable Interrupts

  /* Gestione NVIC */

  // PC_13
  NVIC_SetPriority(EXTI15_10_IRQn, 0);
  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  // PC_2
  NVIC_SetPriority(EXTI2_IRQn, 0);
  NVIC_ClearPendingIRQ(EXTI2_IRQn);
  NVIC_EnableIRQ(EXTI2_IRQn);


  /* Infinite loop */
  while (1){
	  if(b_pressed_flag == 1){
		  processA();
	  }else if(b_pressed_flag == 2){
		  processB();
	  }

	  if(b_pressed_flag == 0){
		  __NOP(); // No Operation does nothing.
	  }
  }

}

void processA(void){
	GPIOA->ODR  ^= (0x01 << 10); // Toggle on PA_10
	HAL_Delay(400);

	b_pressed_flag = 0; // Reset Flag
}
void processB(void){
	GPIOA->ODR  ^= (0x01 << 5); // Toggle on PA_5
	HAL_Delay(400);

	b_pressed_flag = 0; // Reset Flag
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

