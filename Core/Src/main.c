/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include "stm32f0xx_hal_conf.h"

#include <limits.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PWM_PERIOD 20000
#define PWM_Pin GPIO_PIN_9
#define PWM_GPIO_Port GPIOC
#define LD4_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_8
#define NUM_CASES 9
#define MAX_BUTTON_CLICKS 8



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */





/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;


/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
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


	 static uint8_t buttonClickCount = 0;
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//  MotorDirection motor_direction = CLOCKWISE;
//  MotorSpeed motor_speed = STOPPED;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Infinite loop */

  while (1)
  {
      /* Check if the button is pressed */

      if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
      {
          /* Button is pressed */
          HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); // Turn on LD4
          HAL_Delay(500); // Wait for 500ms
          HAL_GPIO_WritePin(blue_Dir_GPIO_Port, blue_Dir_Pin, GPIO_PIN_RESET); // Set the direction to clockwise
          switch (buttonClickCount % NUM_CASES)
          {
          	  case 0:
                /* Stop the motor */
                if (HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4) != HAL_OK) {
                          Error_Handler();
                    }

                	/* Turn off the LED */


                 break;

          	  	case 1:
          	  		 /* Start the motor at low speed */

          	  		 htim3.Init.Prescaler = 100;
          	  		 htim3.Init.Period = 1000;
          	  		 if (HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4) != HAL_OK) {
          	  			 Error_Handler();
          	  		 }

          	  		 if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) {
          	  			 Error_Handler();
          	  		 }
          	  		 break;

          	  case 2:
                  /* Increase the motor speed to mid */
                  htim3.Init.Prescaler = 50;
                  htim3.Init.Period = 1000;
                  HAL_TIM_Base_Init(&htim3);
                  break;

              case 3:
                  /* Increase the motor speed to high */
                  htim3.Init.Prescaler = 5;
                  htim3.Init.Period = 500;
                  HAL_TIM_Base_Init(&htim3);
                  break;

              case 4:
                  /* Gradually decrease the brightness of the LED */
                  for (int i = 200; i >= 0; i--)
                  {
                	  TIM3->CCR4 = i; // Set the duty cycle of the PWM signal
                      HAL_Delay(10);
                  }
                  if (HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4) != HAL_OK)
                      {
                        Error_Handler();
                      }


                  break;

              case 5:

            	  if (HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4) != HAL_OK) {
            	       Error_Handler();
            	     }
            	      TIM3->CCR4 = 200;
            	      htim3.Init.Prescaler = 100;
            	      htim3.Init.Period = 3000;
            	      HAL_TIM_Base_Init(&htim3);
            	      if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) {
            	           Error_Handler();
            	        }
            	  HAL_GPIO_WritePin(blue_Dir_GPIO_Port, blue_Dir_Pin, GPIO_PIN_SET);
                  /* Start the motor at low speed in counter-clockwise direction */

                  break;


              case 6:

            	  	 /* Start the motor at low speed in counter-clockwise direction */
            	     if (HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4) != HAL_OK) {
            	         Error_Handler();
            	     }
            	     TIM3->CCR4 = 100;
            	     htim3.Init.Prescaler = 60;
            	     htim3.Init.Period = 2000;
            	      HAL_TIM_Base_Init(&htim3);
            	      if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) {
            	          Error_Handler();
            	      }
            	      HAL_GPIO_WritePin(blue_Dir_GPIO_Port, blue_Dir_Pin, GPIO_PIN_SET); // Set the direction to counter-clockwise
            	      break;

              case 7:
            	  /* Start the motor at low speed in counter-clockwise direction */
            	      if (HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4) != HAL_OK) {
            	          Error_Handler();
            	      }
            	      htim3.Init.Prescaler = 50;
            	      htim3.Init.Period = 1000;
            	      HAL_TIM_Base_Init(&htim3);
            	      if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) {
            	          Error_Handler();
            	      }
            	      HAL_GPIO_WritePin(blue_Dir_GPIO_Port, blue_Dir_Pin, GPIO_PIN_RESET); // Set the direction to counter-clockwise
            	      break;

              case 8:
                  /* Stop the motor */
                  if (HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4) != HAL_OK) {
                      Error_Handler();
                  }

                  /* Turn off the LED */
                  TIM3->CCR4 = 0;
                  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Turn off LD4
                  HAL_GPIO_WritePin(blue_Dir_GPIO_Port, blue_Dir_Pin, GPIO_PIN_RESET); // Turn off blue direction pin
                  buttonClickCount = 0;
                  break;



              default:

                  break;
          }

          /* Increment button click count */
          buttonClickCount++;

          /* Reset button click count if it reaches 8 */



          /* Wait for the button to be released */
          while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
          {
              /* Do nothing */
          }
      }
  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(blue_Dir_GPIO_Port, blue_Dir_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : blue_Dir_Pin */
  GPIO_InitStruct.Pin = blue_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(blue_Dir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PWM_Pin */
  GPIO_InitStruct.Pin = PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(PWM_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 479;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;   // PWM_PERIOD = 20000
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;  // Set the pulse width to 100 (out of a max of PWM_PERIOD)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  printf("Error occurred, entering infinite loop...\n");

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


