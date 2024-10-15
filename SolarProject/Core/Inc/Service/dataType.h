/*
 * dataType.h
 *
 *  Created on: Sep 9, 2024
 *      Author: DELL
 */

#ifndef INC_SERVICE_DATATYPE_H_
#define INC_SERVICE_DATATYPE_H_


#include "main.h"
#include "stdbool.h"
#include "tim.h"
#include "usart.h"

#define LDRRESISTOR 	2000.0
#define DIFFLOWERBOUND 	-30.0
#define DIFFUPPERBOUND  30.0



typedef enum {
	LEFT = 1,
	RIGHT = 0,
	BOTTOM = 1,
	TOP = 0
} MotorDirection_t ;

typedef enum
{
	VERTICAL,
	HORIZONTAL
} MotorOrientation_t;

typedef enum
{
	AUTO,
	MANUAL
} MotorMode_t;

typedef struct
{
	float LDRTOP;
	float LDRBOTTOM;
	float LDRLEFT;
	float LDRRIGHT;
} LDRresistance_t;

typedef struct{
	MotorOrientation_t ORIENTATION;
	MotorDirection_t DIR;
	bool RESET;
	bool STOP;
	float CURRENTPOSITION;
} MotorState_t;

typedef struct{
	uint8_t VERTICALSETPOSITION;
	uint8_t HORIZONTALSETPOSITION;
	MotorMode_t MODE;
} ModeState_t;

#endif /* INC_SERVICE_DATATYPE_H_ */
