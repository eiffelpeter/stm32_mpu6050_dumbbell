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
#include <stdlib.h> 
#include <math.h>
#include <stdbool.h> 
#include <stdio.h>
#include "string.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum 
{  
  AXIS_X = 0,
  AXIS_Y = 1, 
  AXIS_Z = 2 
} SampleAxis_TypeDef; 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ACCEL_THRESHOLD_UP 1.0  
#define ACCEL_THRESHOLD_DOWN -1.0  
#define TRAINING_COUNT 10  

typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
	int repetition_count;
	int is_moving_up;
	int training_reps_done;
	float accel_up_max;
	float accel_down_min;
	float trained_up_threshold;
	float trained_down_threshold;
	SampleAxis_TypeDef sample_axis;
} DumbbellData;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
DumbbellData dumbbell_data = {0};
MPU6050_t MPU6050;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dumbbell_train_and_recognize(void) {
	float temp;

	// determine which axix to sample
	if (dumbbell_data.training_reps_done == 0 ) {

		if ((fabs(dumbbell_data.accel_x) > fabs(dumbbell_data.accel_y)) && (fabs(dumbbell_data.accel_x) > fabs(dumbbell_data.accel_z))) {
			dumbbell_data.sample_axis = AXIS_X;
			//printf("Training by AXIS_X \n\r");

		} else if ((fabs(dumbbell_data.accel_y) > fabs(dumbbell_data.accel_x)) && (fabs(dumbbell_data.accel_y) > fabs(dumbbell_data.accel_z))) {
			dumbbell_data.sample_axis = AXIS_Y;
			//printf("Training by AXIS_Y \n\r");

		} else /*if ((fabs(dumbbell_data.accel_z) > fabs(dumbbell_data.accel_x)) && (fabs(dumbbell_data.accel_z) > fabs(dumbbell_data.accel_y)))*/ {
			dumbbell_data.sample_axis = AXIS_Z;
			//printf("Training by AXIS_Z \n\r");

		}
	}

	// select axis data
	switch (dumbbell_data.sample_axis) {
	case AXIS_X:
		temp = dumbbell_data.accel_x;
		break;
	case AXIS_Y:
		temp = dumbbell_data.accel_y;
		break;
	case AXIS_Z:
	default:
		temp = dumbbell_data.accel_z;
		break;
	}



	if (dumbbell_data.training_reps_done < TRAINING_COUNT) {
		if (temp > dumbbell_data.accel_up_max) {
			dumbbell_data.accel_up_max = temp;  
		}
		if (temp < dumbbell_data.accel_down_min) {
			dumbbell_data.accel_down_min = temp;  
		}

		if (temp > ACCEL_THRESHOLD_UP && dumbbell_data.is_moving_up == 0) {
			dumbbell_data.is_moving_up = 1;
			//printf("is_moving_up AccelX:%f, AccelY:%f, AccelZ:%f \n\r", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
		} else if (temp < ACCEL_THRESHOLD_DOWN && dumbbell_data.is_moving_up == 1) {
			dumbbell_data.is_moving_up = 0;
			//printf("is_moving_down AccelX:%f, AccelY:%f, AccelZ:%f \n\r", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
			dumbbell_data.training_reps_done++;  
			printf("Training rep %d \n\r", dumbbell_data.training_reps_done);
		}

		if (dumbbell_data.training_reps_done == TRAINING_COUNT) {
			dumbbell_data.trained_up_threshold = dumbbell_data.accel_up_max * 0.7; //0.9;
			dumbbell_data.trained_down_threshold = dumbbell_data.accel_down_min * 0.7; //1.1;
			printf("Training complete. Up threshold: %f, Down threshold: %f\n\r", dumbbell_data.trained_up_threshold, dumbbell_data.trained_down_threshold);
			printf("accel_up_max: %f, accel_down_min: %f\n\r", dumbbell_data.accel_up_max, dumbbell_data.accel_down_min);
		}
	} else {
		if (temp > dumbbell_data.trained_up_threshold && dumbbell_data.is_moving_up == 0) {
			dumbbell_data.is_moving_up = 1;
		} else if (temp < dumbbell_data.trained_down_threshold && dumbbell_data.is_moving_up == 1) {
			dumbbell_data.is_moving_up = 0;
			dumbbell_data.repetition_count++;
			printf("Valid rep detected. Count: %d\n\r", dumbbell_data.repetition_count);
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t mpu6050_tick = 0, led_tick = 0, led_period = 1000;

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	printf("MPU6050_Init");
	while (MPU6050_Init(&hi2c1) == 1) 
    {
        printf(".");
    }
	printf(" successfully \n\r");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		// read every 100ms
		if ((HAL_GetTick() - mpu6050_tick) > 100) {
			mpu6050_tick = HAL_GetTick();

			MPU6050_Read_All(&hi2c1, &MPU6050);
			//printf("AccelX:%f, AccelY:%f, AccelZ:%f \n\r", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);

			dumbbell_data.accel_x = MPU6050.Ax;
			dumbbell_data.accel_y = MPU6050.Ay;
			dumbbell_data.accel_z = MPU6050.Az;
			dumbbell_train_and_recognize();
		}

		/* User push-button press */
		if (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET) {
			uint32_t button_pressed_tick = HAL_GetTick();
			while (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET) {	 // wait until release
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                HAL_Delay(100);
            }

			// long press
			if ((HAL_GetTick() - button_pressed_tick) > 2000) {
				if (dumbbell_data.training_reps_done != 0) {
					memset(&dumbbell_data, 0, sizeof(dumbbell_data));
					printf("trig to train again\n\r");
				} else 
                    printf("already in train state\n\r");
			} 
            //else {
			//	printf("button short press\n\r");
			//}
		}

		// LED
		if ((HAL_GetTick() - led_tick) > led_period) {
			led_tick = HAL_GetTick();

			if (dumbbell_data.training_reps_done == TRAINING_COUNT)
				led_period = 500;
			else
				led_period = 1000;

			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0090194B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART2 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
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
