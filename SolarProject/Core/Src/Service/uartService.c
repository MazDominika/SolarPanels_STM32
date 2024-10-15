/*
 * uartService.c
 *
 *  Created on: Sep 9, 2024
 *      Author: DELL
 */


#include <Service/uartService.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

volatile uint8_t RxBuffor[9];
volatile uint8_t TxBuffor[20];
volatile uint16_t TxSize;

void uartServiceManualModePositionSettings(ModeState_t* mode, uint8_t* rxBuffor)
{
	char *token = strtok((char*) RxBuffor, ",");
	uint8_t config_values[3];

	uint8_t index = 0;
	config_values[index] = atoi(token);

	  while (token != NULL && index <= 2)
	  {
	        // Get the next token
	        token = strtok(NULL, ",");
	        if(index < 2)
	        {
	        	index++;
	        	config_values[index] = atoi(token);
	        }
	    }

	mode->HORIZONTALSETPOSITION = config_values[0];
	mode->VERTICALSETPOSITION = config_values[1];
	mode->MODE = config_values[2] ;
}


void uartServiceSendCurrentPosition(MotorState_t* HorizontalMotor, MotorState_t* VerticalMotor)
{
	TxSize = sniprintf((char*) TxBuffor, sizeof(TxBuffor), "%d,%d\n", (uint16_t) (HorizontalMotor->CURRENTPOSITION * 10), (uint16_t) (VerticalMotor->CURRENTPOSITION * 10));
	HAL_UART_Transmit_DMA(&huart1, TxBuffor, TxSize);
}

