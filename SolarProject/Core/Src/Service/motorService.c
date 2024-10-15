/*
 * motorSensor.c
 *
 *  Created on: Sep 9, 2024
 *      Author: DELL
 */

#include <Service/motorService.h>

volatile MotorState_t horizontalMotor = {0};
volatile MotorState_t verticalMotor = {0};
volatile ModeState_t modeState = {0};
volatile MotorState_t* horizontalMotorHandle = &horizontalMotor;
volatile MotorState_t* verticalMotorHandle = &verticalMotor;
volatile ModeState_t* modeStateHandle = &modeState;

void MotorServiceMotorInit(MotorState_t* HorizontalMotor, MotorState_t* VerticalMotor, ModeState_t* ModeState)
{
	/* HORIZONTAL MOTOR INIT */
	HorizontalMotor->ORIENTATION = HORIZONTAL;
	HorizontalMotor->DIR = TOP;
	HorizontalMotor->RESET = true;
	HorizontalMotor->STOP = true;
	HorizontalMotor->CURRENTPOSITION = 1000;

	/*VERTICAL MOTOR INIT*/
	VerticalMotor->ORIENTATION = VERTICAL;
	VerticalMotor->DIR = LEFT;
	VerticalMotor->RESET = true;
	VerticalMotor->STOP = true;
	VerticalMotor->CURRENTPOSITION = 1000;

	/*CONTROL MODE INIT*/
	ModeState->HORIZONTALSETPOSITION = 0;
	ModeState->VERTICALSETPOSITION = 0;
	ModeState->MODE = AUTO;
}

void MotorServiceMotorHoming(MotorState_t* motor, uint8_t* limiter)
{
	uint8_t limit1 = limiter[0];
	uint8_t limit2 = limiter[2];


	if(motor->ORIENTATION == VERTICAL)
	{
		if(limiter[2] == GPIO_PIN_RESET)
		{
			motor->RESET = true;
			motor->STOP = true;
		}
		else
		{
			motor->DIR = LEFT;
			motor->RESET = false;
			motor->STOP = false;
		}
	}

	if(motor->ORIENTATION == HORIZONTAL)
	{
		if(limiter[0] == GPIO_PIN_RESET)
		{
			motor->RESET = true;
			motor->STOP = true;
		}
		else
		{
			motor->DIR = BOTTOM;
			motor->RESET = false;
			motor->STOP = false;
		}
	}

	MotorServiceMotorControl(motor);
}

void MotorServiceMotorStop(MotorState_t* motor)
{
	motor->STOP = true;
	motor->RESET = true;
	MotorServiceMotorControl(motor);
}

void MotorServiceMotorControl(MotorState_t* motor)
{
	if(motor->ORIENTATION == VERTICAL)
	{
		if(motor->STOP == true)
		{
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
		}
		else
		{
			HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
		}
		HAL_GPIO_WritePin(DIR_VERTICAL_GPIO_Port, DIR_VERTICAL_Pin, motor->DIR);
		HAL_GPIO_WritePin(RESET_VERTICAL_GPIO_Port, RESET_VERTICAL_Pin, !(motor->RESET));
	}

	if(motor->ORIENTATION == HORIZONTAL)
	{
		if(motor->STOP == true)
		{
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		}
		else
		{
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		}

		HAL_GPIO_WritePin(DIR_HORIZONTAL_GPIO_Port, DIR_HORIZONTAL_Pin, motor->DIR);
		HAL_GPIO_WritePin(RESET_HORIZONTAL_GPIO_Port, RESET_HORIZONTAL_Pin, !(motor->RESET)); 	 // reset jest aktywowany stanem niskim, logika odwrotna
	}
}

void MotorServiceAutoMode(LDRresistance_t* resistance, uint8_t* limiter, MotorState_t* motor)
{
	/* HORIZONTAL MOTOR SETTINGS IN AUTO MODE */
	if(motor->ORIENTATION == HORIZONTAL)
	{
		float horizontalResistanceDiff = resistance->LDRTOP - resistance->LDRBOTTOM;

		if(horizontalResistanceDiff < DIFFUPPERBOUND && horizontalResistanceDiff > DIFFLOWERBOUND)
		{
			motor->STOP = true;
			motor->RESET = true;
		}
		else
		{
			if(resistance->LDRBOTTOM > resistance->LDRTOP && limiter[1] == GPIO_PIN_SET && motor->CURRENTPOSITION < 90 )
			{
				motor->DIR = TOP;
				motor->RESET = false;
				motor->STOP = false;
			}
			else if(resistance->LDRBOTTOM < resistance->LDRTOP && limiter[0] == GPIO_PIN_SET && motor->CURRENTPOSITION > 30 )
			{
				motor->DIR = BOTTOM;
				motor->RESET = false;
				motor->STOP = false;
			}
			else
			{
				motor->STOP = true;
				motor->RESET = true;
			}
		}
	}

	/* VERICAL MOTOR SETTINGS IN AUTO MODE */
	if(motor->ORIENTATION == VERTICAL)
	{
		float verticalResistanceDiff = resistance->LDRLEFT - resistance->LDRRIGHT;

		if(verticalResistanceDiff < DIFFUPPERBOUND && verticalResistanceDiff > DIFFLOWERBOUND)
		{
			motor->STOP = true;
			motor->RESET = true;
		}
		else
		{
			if(resistance->LDRLEFT < resistance->LDRRIGHT && limiter[2] == GPIO_PIN_SET && motor->CURRENTPOSITION < 180)
			{
				motor->DIR = LEFT;
				motor->RESET = false;
				motor->STOP = false;
			}
			else if(resistance->LDRLEFT > resistance->LDRRIGHT && motor->CURRENTPOSITION > 0)
			{
				motor->DIR = RIGHT;
				motor->RESET = false;
				motor->STOP = false;
			}
			else
			{
				motor->STOP = true;
				motor->RESET = true;
			}
		}
	}

	MotorServiceMotorControl(motor);
}

void MotorServiceManualMode(ModeState_t* mode, MotorState_t* motor)
{
	if(motor->ORIENTATION == HORIZONTAL)
	{
		if(motor->CURRENTPOSITION == mode->HORIZONTALSETPOSITION)
		{
			motor->STOP = true;
			motor->RESET = true;
		}
		else
		{
			if((motor->CURRENTPOSITION < mode->HORIZONTALSETPOSITION) && motor->CURRENTPOSITION < 90 )
			{
				motor->DIR = TOP;
				motor->RESET = false;
				motor->STOP = false;
			}
			else if((motor->CURRENTPOSITION > mode->HORIZONTALSETPOSITION) && motor->CURRENTPOSITION > 30)
			{
				motor->DIR = BOTTOM;
				motor->RESET = false;
				motor->STOP = false;
			}
			else
			{
				motor->STOP = true;
				motor->RESET = true;
			}
		}
	}

	if(motor->ORIENTATION == VERTICAL)
	{
		if(motor->CURRENTPOSITION == mode->VERTICALSETPOSITION)
		{
			motor->STOP = true;
			motor->RESET = true;
		}
		else
		{
			if( (motor->CURRENTPOSITION < mode->VERTICALSETPOSITION) && motor->CURRENTPOSITION < 180 )
			{
				motor->DIR = LEFT;
				motor->RESET = false;
				motor->STOP = false;
			}
			else if(motor->CURRENTPOSITION > mode->VERTICALSETPOSITION && motor->CURRENTPOSITION > 0)
			{
				motor->DIR = RIGHT;
				motor->RESET = false;
				motor->STOP = false;
			}
			else
			{
				motor->STOP = true;
				motor->RESET = true;
			}
		}
	}

	MotorServiceMotorControl(motor);
}

void MotorServiceVerticalCurrentPosition(MotorState_t* motor)
{
	if (motor->DIR == LEFT )
	{
		motor->CURRENTPOSITION++;
	}
	else
	{
		motor->CURRENTPOSITION --;
	}
}

void MotorServiceHorizontalCurrentPosition(MotorState_t* motor)
{
	if (motor->DIR == TOP )
	{
		motor->CURRENTPOSITION += 1;
	}
	else
	{
		motor->CURRENTPOSITION -= 1;
	}
}
