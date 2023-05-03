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
#include "../../co2/scd30.h"
#include "../../gps/gps.h"
#include "../../so2/so2.h"


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
	uint8_t from_Task; /*0 - GPS; 1 - PM; 2 - CO; 3 - SO */
	void *pvData; /* Holds or points to any data associated with the event. */

} xIPStackEvent_t;

QueueHandle_t xQueueCollate;
/* USER CODE END Variables */
osThreadId PMHandle;
osThreadId COHandle;
osThreadId GPSHandle;
osThreadId SOHandle;
osThreadId COLLATEHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void PM_Task(void const * argument);
void CO_Task(void const * argument);
void GPS_Task(void const * argument);
void SO_Task(void const * argument);
void COLLATE_Task(void const * argument);

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
	xQueueCollate = xQueueCreate( 4, sizeof( xIPStackEvent_t ) );
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of PM */
  osThreadDef(PM, PM_Task, osPriorityNormal, 0, 128);
  PMHandle = osThreadCreate(osThread(PM), NULL);

  /* definition and creation of CO */
  osThreadDef(CO, CO_Task, osPriorityNormal, 0, 128);
  COHandle = osThreadCreate(osThread(CO), NULL);

  /* definition and creation of GPS */
  osThreadDef(GPS, GPS_Task, osPriorityNormal, 0, 128);
  GPSHandle = osThreadCreate(osThread(GPS), NULL);

  /* definition and creation of SO */
  osThreadDef(SO, SO_Task, osPriorityNormal, 0, 128);
  SOHandle = osThreadCreate(osThread(SO), NULL);

  /* definition and creation of COLLATE */
  osThreadDef(COLLATE, COLLATE_Task, osPriorityNormal, 0, 128);
  COLLATEHandle = osThreadCreate(osThread(COLLATE), NULL);

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
	int16_t err;
	uint16_t interval_in_seconds = 2;

	struct Data {
		float co2_ppm;
		float temperature;
		float relative_humidity;
	} data;

	void* pointer = &data;
	xIPStackEvent_t toQueue = { 2, pointer };
	/* Busy loop for initialization, because the main loop does not work without
	 * a sensor.
	 */
	while (scd30_probe() != NO_ERROR) {
		printf("SCD30 sensor probing failed\n");
		sensirion_sleep_usec(1000000u);
	}
	printf("SCD30 sensor probing successful\n");

	scd30_set_measurement_interval(interval_in_seconds);
	sensirion_sleep_usec(20000u);
	scd30_start_periodic_measurement(0);
	/* Infinite loop */
	for(;;)
	{
		uint16_t data_ready = 0;

		/* Poll data_ready flag until data is available. Allow 20% more than
		 * the measurement interval to account for clock imprecision of the
		 * sensor.
		 */
		err = scd30_get_data_ready(&data_ready);
		if (err != NO_ERROR) {
			printf("Error reading data_ready flag: %i\n", err);
		}

		/* Measure co2, temperature and relative humidity and store into
		 * variables.
		 */
		err =
				scd30_read_measurement(&data.co2_ppm, &data.temperature, &data.relative_humidity);
		if (err != NO_ERROR) {
			printf("error reading measurement\n");

		} else {
			printf("measured co2 concentration: %0.2f ppm, "
					"measured temperature: %0.2f degreeCelsius, "
					"measured humidity: %0.2f %%RH\n",
					data.co2_ppm, data.temperature, data.relative_humidity);
			xQueueSend( xQueueCollate, ( void* ) &toQueue, ( TickType_t ) 10);
			vTaskSuspend( NULL );
		}
	}
  /* USER CODE END CO_Task */
}

/* USER CODE BEGIN Header_GPS_Task */
/**
 * @brief Function implementing the GPS thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GPS_Task */
void GPS_Task(void const * argument)
{
  /* USER CODE BEGIN GPS_Task */
	void* pointer = &GPS;
	xIPStackEvent_t toQueue = { 0, pointer };
	GPS_Init();
	vTaskSuspend( NULL );
	/* Infinite loop */
	for(;;)
	{
		if (rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
			rx_buffer[rx_index++] = rx_data;
			HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
			vTaskSuspend( NULL );
		} else {
			GPS_print((char*)rx_buffer);
			if(GPS_validate((char*) rx_buffer))
				GPS_parse((char*) rx_buffer);
			rx_index = 0;
			memset(rx_buffer, 0, sizeof(rx_buffer));
			xQueueSend( xQueueCollate, ( void* ) &toQueue, ( TickType_t ) 10);
			vTaskSuspend( NULL );
			HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
			vTaskSuspend( NULL );
		}
	}
  /* USER CODE END GPS_Task */
}

/* USER CODE BEGIN Header_SO_Task */
/**
 * @brief Function implementing the SO thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SO_Task */
void SO_Task(void const * argument)
{
  /* USER CODE BEGIN SO_Task */
	void* pointer = Rx_data;
	xIPStackEvent_t toQueue = { 3, pointer };
	SO2_GET_DATA();
	vTaskSuspend( NULL );
	/* Infinite loop */
	for(;;)
	{
		for(int i=0; i<13; i++) {
			printf("%d/n", Rx_data[i]);
		}
		xQueueSend( xQueueCollate, ( void* ) &toQueue, ( TickType_t ) 10);
		vTaskSuspend( NULL );
		SO2_GET_DATA();
		vTaskSuspend( NULL );
	}
  /* USER CODE END SO_Task */
}

/* USER CODE BEGIN Header_COLLATE_Task */
/**
* @brief Function implementing the COLLATE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_COLLATE_Task */
void COLLATE_Task(void const * argument)
{
  /* USER CODE BEGIN COLLATE_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END COLLATE_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void GPS_UART_CallBack(){
	vTaskResumeFromISR( GPSHandle );
}
void SO2_UART_CallBack(void)
{
	vTaskResumeFromISR( SOHandle );
}
/* USER CODE END Application */
