
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_azure_rtos.c
  * @author  MCD Application Team
  * @brief   azure_rtos application implementation file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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

#include "app_azure_rtos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include "arm_math.h"
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ZERO_IN_ASCII	48
#define DOT_IN_ASCII	46
#define APB1_CLOCK		84000000
#define FFT_SAMPLES		512
#define FFT_SIZE		(FFT_SAMPLES / 2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN TX_Pool_Buffer */
/* USER CODE END TX_Pool_Buffer */
static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE];
static TX_BYTE_POOL tx_app_byte_pool;

/* USER CODE BEGIN PV */
TX_THREAD FFT_task, LCD_task;
TX_QUEUE ADC_queue, LCD_queue;
uint32_t ADC_queue_stack[3];
float32_t LCD_queue_stack[3];

uint32_t DMA_buffer;

float32_t FFT_input[FFT_SAMPLES] = {0};
float32_t FFT_output[FFT_SAMPLES];
float32_t freqTable[FFT_SIZE];
float32_t freqOrder[FFT_SIZE];
arm_rfft_fast_instance_f32 FFTHandler;

float32_t baseFreq;

UINT status1;
UINT status2;
UINT status3;
UINT status4;

//UCHAR trace_buffer[4096];

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void convert_to_ASCII(float32_t number, uint8_t* ascii);
void FFT_init(float32_t samplingFreq);

VOID FFT_task_entry(ULONG initial_input)
{
	uint32_t received_from_ADC;
	uint32_t maxFFTValueIndex = 0;
	float32_t maxFFTValue = 0;
	uint16_t freqBufferIndex = 0;
	float32_t f_s = APB1_CLOCK / (htim3.Init.Prescaler + 1) / (htim3.Init.Period + 1);

	FFT_init(f_s);
	arm_rfft_fast_init_f32(&FFTHandler, FFT_SAMPLES);

	while(1)
	{
		//xQueueReceive(ADC_queue, &received_from_ADC, portMAX_DELAY);
		tx_queue_receive(&ADC_queue, (void*)&received_from_ADC, TX_WAIT_FOREVER);
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

		FFT_input[freqBufferIndex] = (float32_t)received_from_ADC;
		freqBufferIndex++;

		if(freqBufferIndex >= FFT_SAMPLES)
		{
			freqBufferIndex = 0;

			arm_rfft_fast_f32(&FFTHandler, FFT_input, FFT_output, 0);
			arm_cmplx_mag_f32(FFT_output, freqTable, FFT_SIZE);
			arm_max_f32(freqTable + 1, FFT_SIZE - 1, &maxFFTValue, &maxFFTValueIndex);

			baseFreq = freqOrder[maxFFTValueIndex];

			//xQueueSend(LCD_queue, (void*)&baseFreq, 0);
			tx_queue_send(&LCD_queue, (void*)&baseFreq, TX_NO_WAIT);
		}

		tx_thread_sleep(1);
	}
}

VOID LCD_task_entry(ULONG initial_input)
{
	float32_t received_from_FFT;
	uint8_t display_text[6];

	while(1)
	{
		//xQueueReceive(LCD_queue, &received_from_FFT, portMAX_DELAY);
		tx_queue_receive(&LCD_queue, (void*)&received_from_FFT, TX_WAIT_FOREVER);

		BSP_LCD_Clear(LCD_COLOR_WHITE);
		convert_to_ASCII(received_from_FFT, display_text);
		BSP_LCD_DisplayStringAt(80, 80, (uint8_t*)display_text, LEFT_MODE);
		tx_thread_sleep(1);
	}
}
/* USER CODE END PFP */

/**
  * @brief  Define the initial system.
  * @param  first_unused_memory : Pointer to the first unused memory
  * @retval None
  */
VOID tx_application_define(VOID *first_unused_memory)
{
  /* USER CODE BEGIN  tx_application_define */
  VOID *ptr;
  /* USER CODE END  tx_application_define */

  VOID *memory_ptr;

  if (tx_byte_pool_create(&tx_app_byte_pool, "Tx App memory pool", tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    /* USER CODE BEGIN TX_Byte_Pool_Error */

    /* USER CODE END TX_Byte_Pool_Error */
  }
  else
  {
    /* USER CODE BEGIN TX_Byte_Pool_Success */

    /* USER CODE END TX_Byte_Pool_Success */

    memory_ptr = (VOID *)&tx_app_byte_pool;

    if (App_ThreadX_Init(memory_ptr) != TX_SUCCESS)
    {
      /* USER CODE BEGIN  App_ThreadX_Init_Error */

      /* USER CODE END  App_ThreadX_Init_Error */
    }

    /* USER CODE BEGIN  App_ThreadX_Init_Success */
	tx_byte_allocate(&tx_app_byte_pool, &ptr, 512, TX_NO_WAIT);
	tx_thread_create(&FFT_task, "FFT Task", FFT_task_entry, 0x1234, ptr, 512, 2, 2, 1, TX_AUTO_START);
	status4 = tx_byte_allocate(&tx_app_byte_pool, &ptr, 256, TX_NO_WAIT);
	status3 = tx_thread_create(&LCD_task, "LCD Task", LCD_task_entry, 0x1234, ptr, 512, 2, 2, 1, TX_AUTO_START);

	tx_byte_allocate(&tx_app_byte_pool, &ptr, 12, TX_NO_WAIT);
	status1 = tx_queue_create(&ADC_queue, "ADC_queue", 1, &ptr, 12);
	tx_byte_allocate(&tx_app_byte_pool, &ptr, 12, TX_NO_WAIT);
	status2 = tx_queue_create(&LCD_queue, "LCD_queue", 1, &ptr, 12);

	/* done here because otherwise ADC will start before queue is even initialized */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&DMA_buffer, 1);

	//tx_trace_enable(&trace_buffer, 4096, 30);
    /* USER CODE END  App_ThreadX_Init_Success */

  }

}

/* USER CODE BEGIN  0 */
void convert_to_ASCII(float32_t number, uint8_t* ascii)
{
	uint32_t number_without_decimals = number * 100;

	uint8_t hundreds = number_without_decimals / 10000;
	uint8_t tens = (number_without_decimals - (hundreds * 10000)) / 1000;
	uint8_t ones = (number_without_decimals - (hundreds * 10000) - (tens * 1000)) / 100;
	uint8_t decimals = (number_without_decimals - (hundreds * 10000) - (tens * 1000) - (ones * 100)) / 10;
	uint8_t hundredth = (number_without_decimals - (hundreds * 10000) - (tens * 1000) - (ones * 100) - (decimals * 10));

	ascii[0] = hundreds + ZERO_IN_ASCII;
	ascii[1] = tens + ZERO_IN_ASCII;
	ascii[2] = ones + ZERO_IN_ASCII;
	ascii[3] = DOT_IN_ASCII;
	ascii[4] = decimals + ZERO_IN_ASCII;
	ascii[5] = hundredth + ZERO_IN_ASCII;
}

void FFT_init(float32_t samplingFreq)
{
	for(uint16_t i = 0; i < FFT_SIZE; i++)
	{
		freqOrder[i] = i * samplingFreq / FFT_SAMPLES;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	if (AdcHandle == &hadc1)
	{
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		//xQueueSendFromISR(ADC_queue, (void*)&DMA_buffer, &xHigherPriorityTaskWoken);
		tx_queue_send(&ADC_queue, (void*)&DMA_buffer, TX_NO_WAIT);
	}
}
/* USER CODE END  0 */
