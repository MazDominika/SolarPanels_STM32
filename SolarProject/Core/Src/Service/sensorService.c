/*
 * sensorService.c
 *
 *  Created on: Sep 9, 2024
 *      Author: DELL
 */

#include <Service/sensorService.h>

volatile uint8_t limitSwitch[3];
volatile uint16_t measurement[4];

void SensorServiceADC2Resistance(uint16_t* measurement, LDRresistance_t* LDRResistance)
{
	float ldrResistance[4];

	for (uint8_t i = 0; i < 4; i++)
	{
		uint16_t meas = *(measurement + i);
		float ldrvoltage = SensorServiceADC2Voltage(meas);
		float current = ldrvoltage/LDRRESISTOR;
		float photoresistorResistance = (3.3 - ldrvoltage)/current;
		ldrResistance[i] = photoresistorResistance;
	}

	LDRResistance->LDRTOP = ldrResistance[0];
	LDRResistance->LDRBOTTOM = ldrResistance[1];
	LDRResistance->LDRLEFT = ldrResistance[2];
	LDRResistance->LDRRIGHT = ldrResistance[3];
}

float SensorServiceADC2Voltage(uint16_t measurement)
{
	return 3.3 * (float) measurement / (float) 4095;
}

void SensorServiceReadLimiter(uint8_t* limiter)
{
	limiter[0] = HAL_GPIO_ReadPin(LIMIT_HORIZONTAL_30DEG_GPIO_Port, LIMIT_HORIZONTAL_30DEG_Pin);
	limiter[1] = HAL_GPIO_ReadPin(LIMIT_HORIZONTAL_90DEG_GPIO_Port, LIMIT_HORIZONTAL_90DEG_Pin);
	limiter[2] = HAL_GPIO_ReadPin(LIMIT_VERTICAL_GPIO_Port, LIMIT_VERTICAL_Pin);
}



