/**
  ******************************************************************************
  * @file    I2C/I2C_TwoBoards_ComPolling/Src/main.c
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32F0xx I2C HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          Polling transfer.
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h> 
#include <math.h>
#include <stdbool.h> 
#include "string.h"
#include "main.h"
#include "mpu6050.h"

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup I2C_TwoBoards_ComPolling
  * @{
  */
typedef enum 
{  
  AXIS_X = 0,
  AXIS_Y = 1, 
  AXIS_Z = 2 
} SampleAxis_TypeDef; 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_ADDRESS        0x0F//0x30F

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 48 MHz */
/* This example use TIMING to 0x00A51314 to reach 1 MHz speed (Rise time = 100 ns, Fall time = 100 ns) */
#define I2C_TIMING      0x00A51314

/* Private macro -------------------------------------------------------------*/
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

DumbbellData dumbbell_data = {0};

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;

/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

MPU6050_t MPU6050;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PFP */

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    
}

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

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	uint32_t mpu6050_tick = 0, led_tick = 0, led_period = 1000;
	/* STM32F0xx HAL library initialization:
		 - Configure the Flash prefetch
		 - Systick timer is configured by default as source of time base, but user 
		   can eventually implement his proper time base source (a general purpose 
		   timer for example or other time source), keeping in mind that Time base 
		   duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
		   handled in milliseconds basis.
		 - Low Level Initialization
	   */
	HAL_Init();

	/* Configure the system clock to 48 MHz */
	SystemClock_Config();

	MX_USART2_UART_Init();

	/* Configure LED2 */
	BSP_LED_Init(LED2);
	BSP_LED_On(LED2);

	/* Configure User push-button */
	BSP_PB_Init(BUTTON_USER,BUTTON_MODE_GPIO);    

	/*##-1- Configure the I2C peripheral ######################################*/
	I2cHandle.Instance             = I2Cx;
	I2cHandle.Init.Timing          = I2C_TIMING;
	I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
	I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2cHandle.Init.OwnAddress2     = 0xFF;
	I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&I2cHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/* Enable the Analog I2C Filter */
	HAL_I2CEx_ConfigAnalogFilter(&I2cHandle,I2C_ANALOGFILTER_ENABLE);

	printf("MPU6050_Init");
	while (MPU6050_Init(&I2cHandle) == 1) 
    {
        printf(".");
    }
	printf(" successfully \n\r");

	/* Infinite loop */
	while (1) {

		// read every 100ms
		if ((HAL_GetTick() - mpu6050_tick) > 100) {
			mpu6050_tick = HAL_GetTick();

			MPU6050_Read_All(&I2cHandle, &MPU6050);
			//printf("AccelX:%f, AccelY:%f, AccelZ:%f \n\r", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);

			dumbbell_data.accel_x = MPU6050.Ax;
			dumbbell_data.accel_y = MPU6050.Ay;
			dumbbell_data.accel_z = MPU6050.Az;
			dumbbell_train_and_recognize();
		}

		/* User push-button press */
		if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET) {
			uint32_t button_pressed_tick = HAL_GetTick();
			while (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET) {	 // wait until release
                BSP_LED_Toggle(LED2); 
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

			BSP_LED_Toggle(LED2);
		}
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI/2)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 12
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* No HSE Oscillator on Nucleo, Activate PLL with HSI/2 as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK) {
		/* Initialization Error */
		while (1); 
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK) {
		/* Initialization Error */
		while (1); 
	}
}
/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
	/** Error_Handler() function is called when error occurs.
	  * 1- When Slave don't acknowledge it's address, Master restarts communication.
	  * 2- When Master don't acknowledge the last data transferred, Slave don't care in this example.
	  */
	if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF) {
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
	GPIO_InitTypeDef  InitStruct;
	__HAL_RCC_USART2_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART2 GPIO Configuration
	PA2     ------> USART2_TX
	PA3     ------> USART2_RX
	*/
	InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	InitStruct.Mode = GPIO_MODE_AF_PP;
	InitStruct.Pull = GPIO_PULLUP;
	InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	InitStruct.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(GPIOA, &InitStruct);

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	UartHandle.Instance        = USART2;

	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_DeInit(&UartHandle) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UART_Init(&UartHandle) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
	/* Error if LED2 is slowly blinking (1 sec. period) */
	while (1) {
		BSP_LED_Toggle(LED2); 
		HAL_Delay(1000);
	} 
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1) {
	}
}
#endif


/**
  * @}
  */

/**
  * @}
  */
