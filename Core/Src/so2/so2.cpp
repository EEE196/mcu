/*
 * so2.cpp
 *
 *  Created on: Mar 15, 2023
 *      Author: Mark
 */

#include "so2.h"
#include "usart.h"


uint8_t Rx_data[13];
uint8_t Tx_data[9] = { 0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78 };


void HAL_UART_TXCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&hlpuart1, Rx_data, 13);
}
int SO2_GET_DATA(void)
{
	HAL_UART_Transmit_IT(&hlpuart1, Tx_data, 9);

}