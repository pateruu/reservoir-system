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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Struct for Pipeline Config
typedef struct{
	int pwm; //PWM value for Pipeline
	uint8_t first_hour; //First operating hour for pipeline
	uint8_t last_hour; //Last operating hour for pipeline

}Pipeline;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t byte;
#define NUM_PIPELINES 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
volatile uint8_t wall_clock_hr_update_flag = 0;
volatile uint8_t clock_hours = 0;
volatile uint8_t clock_mins = 0;
volatile uint8_t clock_secs = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rcv_intpt_flag = 0; //Flag for UART Interrupt
bool setup_complete = false; //Setup complete boolean
uint8_t pipeline_by_user = 0; //variable to store PIPELINE option given by user (0-3)
Pipeline pipelines[NUM_PIPELINES]; //Array for 4 pipelines.
char txd_message_buffer_1[128] = {0};
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
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart6, &byte, 1); //Enable UART Interrupt
  HAL_TIM_Base_Start_IT(&htim4);
  clock_hours = 0;
  clock_mins = 0;
  clock_secs = 0;
  void timer_print_check();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(setup_complete == false){

		HAL_Delay(1000);

		//Print Statement
		sprintf(txd_message_buffer_1, "\r\nSETUP MODE\r\n");
		HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer_1, strlen(txd_message_buffer_1), HAL_MAX_DELAY);

		//Print Statement
		sprintf(txd_message_buffer_1, "Enter SETUP Parameters\n");
		HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer_1, strlen(txd_message_buffer_1), HAL_MAX_DELAY);
		//LED Flashing Logic
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //Start the timer channel for the blinking LD2

		//run_setup_process(); //Run Setup Process

		//Print Space Between Sections
		sprintf(txd_message_buffer_1, "\r\n");
		HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer_1, strlen(txd_message_buffer_1), HAL_MAX_DELAY);

		//Tell User to press the Blue Button
		sprintf(txd_message_buffer_1, "\r\nSETUP is done. Press Blue Button for RUN Mode");
		HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer_1, strlen(txd_message_buffer_1), HAL_MAX_DELAY);

		  //Polling for Blue Button
		  while(1){
			  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){ //if active low
				  HAL_Delay(50);
				  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){ //if active low
					  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Stop blinking
					  //Set Timer Period to small value to make it look like a on LED.
					  __HAL_TIM_SET_AUTORELOAD(&htim2, 10-1);
					  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 10-1);
					  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
					  HAL_Delay(2000);
					  break;
				  }
			  }
		  }
	  }
	  //Add stuff after setup logic
	  if (wall_clock_hr_update_flag == 1){
		  wall_clock_hr_update_Flag = 0; //reset the interrupt flag.

		  if(timer_print_check){
			  char output_message[128];
			  sprintf(output_message, "\r\nCLOCK : %02d : PIPE : %d : PWM : %d : RPM : %d : DEPTH : %d", clock_hours, 0,0,207 + (clock_hours % 5)*2, 85);
			  sprintf(output_message, sizeof(output_message),
			 "CLOCK : %02d : PIPE : %d : PWM : %d : RPM : %d : DEPTH : %d\r\n",
			 clock_hours, 0, 0, 207 + (clock_hours % 5) * 2, 85);

			// Transmit the output message via UART
			HAL_UART_Transmit(&huart6, (uint8_t *)output_message, strlen(output_message), HAL_MAX_DELAY);

		  }


	  }


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 53.33-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart -> Instance == USART6){
		HAL_UART_Transmit(&huart6, &byte, 1, 100);
		rcv_intpt_flag = 1;
		HAL_UART_Receive_IT(&huart6, &byte, 1); //re-enable interrupt
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim -> Instance == TIM5){
		if(clock_hours != 23){
			clock_hours++; //each interrupt add an hour.
		} else{
			clock_hours = 0;  //Reset to 0 after 23 hours.
		}
		wall_clock_hr_update_flag = 1; //Set the to indicate hour changes.
	}
}

void run_setup_process(void){
	char txd_message_buffer[128] = {0};
	char rx_buffer[16] = {0};
	int index = 0;



	//Iterate through all the pipelines.
	for(int i = 0; i < NUM_PIPELINES; i++){

		//Ask user to select Pipeline from 0-3
		sprintf(txd_message_buffer, "\r\nPIPELINE (options: 0 to 3): ");
		HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer, strlen(txd_message_buffer), HAL_MAX_DELAY);


		while(1){
			if(rcv_intpt_flag == 1){
				rcv_intpt_flag = 0;
				if(byte == '\r'){ //If user pressed the enter key.
					rx_buffer[index] = '\0'; //Null terminate the string.
					pipeline_by_user = atoi(rx_buffer); //convert to integer
					index = 0; //reset the index
					break; //exit look
				} else{
					if(index <= sizeof(rx_buffer) - 1){
						rx_buffer[index++] = byte; //store recieved character
					}
				}
			}
		}

		//Ask user to set the Pump PWM for given Pipeline
		sprintf(txd_message_buffer, "\r\nPUMP PWM (options: 0 to 3): ");
		HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer, strlen(txd_message_buffer), HAL_MAX_DELAY);


		while(1){
			if(rcv_intpt_flag == 1){
				rcv_intpt_flag = 0;
				if(byte == '\r'){ //If user pressed the enter key.
					rx_buffer[index] = '\0'; //Null terminate the string.
					pipelines[pipeline_by_user].pwm = atoi(rx_buffer); //convert to integer
					index = 0; //reset the index
					break; //exit look
				} else{
					if(index <= sizeof(rx_buffer) - 1){
						rx_buffer[index++] = byte; //store recieved character
					}
				}
			}
		}
	}

	//Print Space Between Sections
	sprintf(txd_message_buffer, "\r\n");
	HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer, strlen(txd_message_buffer), HAL_MAX_DELAY);

	//Iterate through all the pipelines to set Times
	for(int i = 0; i < NUM_PIPELINES; i++){
		sprintf(txd_message_buffer, "\r\nPipeline %d  Pump FIRST HOUR (options: 00 to 23): ", i);
		HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer, strlen(txd_message_buffer), HAL_MAX_DELAY);

		while(1){
			if(rcv_intpt_flag == 1){
				rcv_intpt_flag = 0;
				if(byte == '\r'){ //If user pressed the enter key.
					rx_buffer[index] = '\0'; //Null terminate the string.
					pipelines[i].first_hour = atoi(rx_buffer); //convert to integer
					index = 0; //reset the index
					break; //exit look
				} else{
					if(index <= sizeof(rx_buffer) - 1){
						rx_buffer[index++] = byte; //store recieved character
					}
				}
			}
		}

		sprintf(txd_message_buffer, "\r\nPipeline %d  Pump LAST HOUR (options: 00 to 23): ", i);
		HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer, strlen(txd_message_buffer), HAL_MAX_DELAY);

		while(1){
			if(rcv_intpt_flag == 1){
				rcv_intpt_flag = 0;
				if(byte == '\r'){ //If user pressed the enter key.
					rx_buffer[index] = '\0'; //Null terminate the string.
					pipelines[i].last_hour = atoi(rx_buffer); //convert to integer
					index = 0; //reset the index
					break; //exit look
				} else{
					if(index <= sizeof(rx_buffer) - 1){
						rx_buffer[index++] = byte; //store recieved character
					}
				}
			}
		}
	}

	//Print Space Between Sections
	sprintf(txd_message_buffer, "\r\n");
	HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer, strlen(txd_message_buffer), HAL_MAX_DELAY);


	sprintf(txd_message_buffer, "\r\nPrinting SETUP Parameters");
	HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer, strlen(txd_message_buffer), HAL_MAX_DELAY);

	//ADD PRINT FOR CURRENT WALL CLOCK HOUR
	sprintf(txd_message_buffer, "\r\nADD LOGIC FOR WALL CLOCK HOUR PRINT USING VARIABLE OR SOMETHING");
	HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer, strlen(txd_message_buffer), HAL_MAX_DELAY);

	//Print all Pipeline Data
	for(int i = 0; i < NUM_PIPELINES; i++){
		snprintf(txd_message_buffer, sizeof(txd_message_buffer),
		         "\r\nPIPELINE: %d   Pump PWM: %d   Pump FIRST HOUR: %d   Pump LAST HOUR %d",
		         i, pipelines[i].pwm, pipelines[i].first_hour, pipelines[i].last_hour);
		HAL_UART_Transmit(&huart6, (uint8_t*)txd_message_buffer, strlen(txd_message_buffer), HAL_MAX_DELAY);
	}

	setup_complete = true;
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
