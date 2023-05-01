/*
 * so2.h
 *
 *  Created on: Mar 15, 2023
 *      Author: Mark
 */

#ifndef SO2_H_
#define SO2_H_

#include <stdint.h>
#include "usart.h"
#include "stdio.h"
#define DEBUG 1
#define SO2_USART &hlpuart1
extern uint8_t Rx_data[13];
extern uint8_t Tx_data[9];
void SO2_GET_DATA(void);
void SO2_UART_CallBack(void);



#endif /* SO2_H_ */
