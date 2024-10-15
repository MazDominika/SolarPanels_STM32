/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <Service/dataType.h>
#include <Service/sensorService.h>
#include <Service/motorService.h>
#include "adc.h"
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
/* USER CODE BEGIN Variables */
extern volatile uint8_t limitSwitch[3];
extern volatile MotorState_t* horizontalMotorHandle;
extern volatile MotorState_t* verticalMotorHandle;
extern volatile ModeState_t* modeStateHandle;
extern volatile uint32_t measurement[4];
extern volatile uint8_t RxBuffor[9];

/* USER CODE END Variables */
osThreadId RxUARTHandle;
osThreadId ADC2ResistanceHandle;
osThreadId MotorControlHandle;
osThreadId LimiterHandlingHandle;
osThreadId TxUARTHandle;
osMessageQId LDRResistanceHandle;
osSemaphoreId MotorControlSemaphoreHandle;
osSemaphoreId LimiterSemaphoreHandle;
osSemaphoreId RxUARTSemaphoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartRxUART(void const * argument);
void StartADC2Resistance(void const * argument);
void StartMotorControl(void const * argument);
void StartLimiterHandling(void const * argument);
void StartTask05(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of MotorControlSemaphore */
  osSemaphoreDef(MotorControlSemaphore);
  MotorControlSemaphoreHandle = osSemaphoreCreate(osSemaphore(MotorControlSemaphore), 1);

  /* definition and creation of LimiterSemaphore */
  osSemaphoreDef(LimiterSemaphore);
  LimiterSemaphoreHandle = osSemaphoreCreate(osSemaphore(LimiterSemaphore), 1);

  /* definition and creation of RxUARTSemaphore */
  osSemaphoreDef(RxUARTSemaphore);
  RxUARTSemaphoreHandle = osSemaphoreCreate(osSemaphore(RxUARTSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of LDRResistance */
  osMessageQDef(LDRResistance, 10, LDRresistance_t);
  LDRResistanceHandle = osMessageCreate(osMessageQ(LDRResistance), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of RxUART */
  osThreadDef(RxUART, StartRxUART, osPriorityAboveNormal, 0, 256);
  RxUARTHandle = osThreadCreate(osThread(RxUART), NULL);

  /* definition and creation of ADC2Resistance */
  osThreadDef(ADC2Resistance, StartADC2Resistance, osPriorityNormal, 0, 128);
  ADC2ResistanceHandle = osThreadCreate(osThread(ADC2Resistance), NULL);

  /* definition and creation of MotorControl */
  osThreadDef(MotorControl, StartMotorControl, osPriorityNormal, 0, 128);
  MotorControlHandle = osThreadCreate(osThread(MotorControl), NULL);

  /* definition and creation of LimiterHandling */
  osThreadDef(LimiterHandling, StartLimiterHandling, osPriorityAboveNormal, 0, 128);
  LimiterHandlingHandle = osThreadCreate(osThread(LimiterHandling), NULL);

  /* definition and creation of TxUART */
  osThreadDef(TxUART, StartTask05, osPriorityNormal, 0, 128);
  TxUARTHandle = osThreadCreate(osThread(TxUART), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartRxUART */
/**
  * @brief  Function implementing the RxUART thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRxUART */
void StartRxUART(void const * argument)
{
  /* USER CODE BEGIN StartRxUART */
	xSemaphoreTake(RxUARTSemaphoreHandle, portMAX_DELAY );
  /* Infinite loop */
  for(;;)
  {
	  if(xSemaphoreTake(RxUARTSemaphoreHandle, portMAX_DELAY ) == pdTRUE)
	  {
		  uartServiceManualModePositionSettings(modeStateHandle, RxBuffor);
	  }
  }
  /* USER CODE END StartRxUART */
}

/* USER CODE BEGIN Header_StartADC2Resistance */
/**
* @brief Function implementing the ADC2Resistance thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC2Resistance */
void StartADC2Resistance(void const * argument)
{
  /* USER CODE BEGIN StartADC2Resistance */
  /* Infinite loop */
	LDRresistance_t LDRresistance = {0};
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) measurement, 4);
	  SensorServiceADC2Resistance((uint16_t*) measurement, &LDRresistance);
	  xQueueSend(LDRResistanceHandle, &LDRresistance, 0);
	  osDelay(10);
  }
  /* USER CODE END StartADC2Resistance */
}

/* USER CODE BEGIN Header_StartMotorControl */
/**
* @brief Function implementing the MotorControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControl */
void StartMotorControl(void const * argument)
{
  /* USER CODE BEGIN StartMotorControl */
	LDRresistance_t resistance = {0};
  /* Infinite loop */
  for(;;)
  {
	if(osSemaphoreGetCount(MotorControlSemaphoreHandle) > 0)
	{
		xQueueReceive(LDRResistanceHandle, &resistance, 0);

		if (modeStateHandle->MODE == MANUAL)
		{
			MotorServiceManualMode(modeStateHandle, horizontalMotorHandle);
			MotorServiceManualMode(modeStateHandle, verticalMotorHandle);
		}
		else
		{
			MotorServiceAutoMode(&resistance, limitSwitch, horizontalMotorHandle);
			MotorServiceAutoMode(&resistance, limitSwitch, verticalMotorHandle);
		}
		osDelay(10);
	}
  }
  /* USER CODE END StartMotorControl */
}

/* USER CODE BEGIN Header_StartLimiterHandling */
/**
* @brief Function implementing the LimiterHandling thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLimiterHandling */
void StartLimiterHandling(void const * argument)
{
  /* USER CODE BEGIN StartLimiterHandling */
	xSemaphoreTake(LimiterSemaphoreHandle, portMAX_DELAY );
  /* Infinite loop */
  for(;;)
  {
	  if(xSemaphoreTake(LimiterSemaphoreHandle, portMAX_DELAY ) == pdTRUE)
	  {
		  SensorServiceReadLimiter(limitSwitch);
		  MotorServiceMotorStop(horizontalMotorHandle);
		  MotorServiceMotorStop(verticalMotorHandle);
	  }
  }
  /* USER CODE END StartLimiterHandling */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the TxUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
	  uartServiceSendCurrentPosition(horizontalMotorHandle, verticalMotorHandle);
	  osDelay(1000);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

