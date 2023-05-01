/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "../../pm2.5/sps30.h"
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
typedef struct IP_TASK_COMMANDS
{
	uint8_t from_Task; /*0 - gps_so; 1 - PM; 2 - CO */
	void *pvData; /* Holds or points to any data associated with the event. */

} xIPStackEvent_t;

QueueHandle_t xQueueCollate;
/* USER CODE END Variables */
osThreadId PMHandle;
osThreadId COHandle;
osThreadId GPS_SOHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void PM_Task(void const * argument);
void CO_Task(void const * argument);
void GPS_SO_Task(void const * argument);

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

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	QueueHandle_t xQueueCollate = xQueueCreate( 4, sizeof( xIPStackEvent_t ) );
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of PM */
	osThreadDef(PM, PM_Task, osPriorityNormal, 0, 128);
	PMHandle = osThreadCreate(osThread(PM), NULL);

	/* definition and creation of CO */
	osThreadDef(CO, CO_Task, osPriorityNormal, 0, 128);
	COHandle = osThreadCreate(osThread(CO), NULL);

	/* definition and creation of GPS_SO */
	osThreadDef(GPS_SO, GPS_SO_Task, osPriorityNormal, 0, 128);
	GPS_SOHandle = osThreadCreate(osThread(GPS_SO), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_PM_Task */
/**
 * @brief  Function implementing the PM thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_PM_Task */
void PM_Task(void const * argument)
{
	/* USER CODE BEGIN PM_Task */
	/* Infinite loop */
	struct sps30_measurement m;
	int16_t ret;

	void* pointer = &m;
	xIPStackEvent_t toQueue = { 1, pointer };


	/* Busy loop for initialization, because the main loop does not work without
	 * a sensor.
	 */
	while (sps30_probe() != 0) {
		printf("SPS sensor probing failed\n");
		sensirion_sleep_usec(1000000); /* wait 1s */
	}
	printf("SPS sensor probing successful\n");

	uint8_t fw_major;
	uint8_t fw_minor;
	ret = sps30_read_firmware_version(&fw_major, &fw_minor);
	if (ret) {
		printf("error reading firmware version\n");
	} else {
		printf("FW: %u.%u\n", fw_major, fw_minor);
	}

	char serial_number[SPS30_MAX_SERIAL_LEN];
	ret = sps30_get_serial(serial_number);
	if (ret) {
		printf("error reading serial number\n");
	} else {
		printf("Serial Number: %s\n", serial_number);
	}

	ret = sps30_start_measurement();
	if (ret < 0)
		printf("error starting measurement\n");
	printf("measurements started\n");
	sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC); /* wait 1s */

	for(;;)
	{
		ret = sps30_read_measurement(&m);
		if (ret < 0) {
			printf("error reading measurement\n");

		} else {
			printf("measured values:\n"
					"\t%0.2f pm1.0\n"
					"\t%0.2f pm2.5\n"
					"\t%0.2f pm4.0\n"
					"\t%0.2f pm10.0\n"
					"\t%0.2f nc0.5\n"
					"\t%0.2f nc1.0\n"
					"\t%0.2f nc2.5\n"
					"\t%0.2f nc4.5\n"
					"\t%0.2f nc10.0\n"
					"\t%0.2f typical particle size\n\n",
					m.mc_1p0, m.mc_2p5, m.mc_4p0, m.mc_10p0, m.nc_0p5, m.nc_1p0,
					m.nc_2p5, m.nc_4p0, m.nc_10p0, m.typical_particle_size);
			xQueueSend( xQueueCollate, ( void* ) &toQueue, ( TickType_t ) 10);
			vTaskSuspend( NULL );
		}
	}
	/* USER CODE END PM_Task */
}

/* USER CODE BEGIN Header_CO_Task */
/**
 * @brief Function implementing the CO thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CO_Task */
void CO_Task(void const * argument)
{
	/* USER CODE BEGIN CO_Task */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END CO_Task */
}

/* USER CODE BEGIN Header_GPS_SO_Task */
/**
 * @brief Function implementing the GPS_SO thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GPS_SO_Task */
void GPS_SO_Task(void const * argument)
{
	/* USER CODE BEGIN GPS_SO_Task */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END GPS_SO_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
