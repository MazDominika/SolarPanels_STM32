/*
 * uartService.h
 *
 *  Created on: Sep 9, 2024
 *      Author: DELL
 */

#ifndef INC_SERVICE_UARTSERVICE_H_
#define INC_SERVICE_UARTSERVICE_H_

#include "dataType.h"

void uartServiceManualModePositionSettings(ModeState_t* mode, uint8_t* rxBuffor);
void uartServiceSendCurrentPosition(MotorState_t* HorizontalMotor, MotorState_t* VerticalMotor);

/*---------------------------------		RX Frame	---------------------------------*
 *
							[VSP|HSP|M|] - values separated by '|'

 *			LEGEND:						|				VALUE:
 *	----------------------------------------------------------------------------------
 *	"M" - MODE							|	uint8_t 1 - MANUAL
 *	"VSP" - VERTICALSETTINGPOSITION		|	uint8_t 0 - 200 degree
 *	"HSP" - HORIZONTALSETTINGPOSITION	|	uint8_t 30 - 90 degree
 */



/*---------------------------------		TX Frame	---------------------------------*
 *
 *  						[CVP|CHP] - values separated by '|'
 *
 *			CODE:						|				VALUE:
 *	----------------------------------------------------------------------------------
 *	"CVP" - CURRVERTICALPOSITION		|		uint8_t 0 - 200 degree
 *	"CHP" - CURRHORIZONTALPOSITION		|		uint8_t 30 - 90 degree
*/

#endif /* INC_SERVICE_UARTSERVICE_H_ */
