/*
 * motorService.h
 *
 *  Created on: Sep 9, 2024
 *      Author: DELL
 */

#ifndef INC_SERVICE_MOTORSERVICE_H_
#define INC_SERVICE_MOTORSERVICE_H_

#include <Service/dataType.h>

void MotorServiceMotorInit(MotorState_t* HorizontalMotor, MotorState_t* VerticalMotor, ModeState_t* ModeState);
void MotorServiceMotorStop(MotorState_t* motor);
void MotorServiceMotorControl(MotorState_t* motor);
void MotorServiceAutoMode(LDRresistance_t* resistance, uint8_t* limiter, MotorState_t* motor);
void MotorServiceManualMode(ModeState_t* ManualMode, MotorState_t* motor);
void MotorServiceMotorStop(MotorState_t* motor);
void MotorServiceMotorHoming(MotorState_t* motor, uint8_t* limiter);
void MotorServiceVerticalCurrentPosition(MotorState_t* motor);
void MotorServiceHorizontalCurrentPosition(MotorState_t* motor);

#endif /* INC_SERVICE_MOTORSERVICE_H_ */
