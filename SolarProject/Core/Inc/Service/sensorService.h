/*
 * sensorService.h
 *
 *  Created on: Sep 9, 2024
 *      Author: DELL
 */

#ifndef INC_SERVICE_SENSORSERVICE_H_
#define INC_SERVICE_SENSORSERVICE_H_

#include <Service/dataType.h>

void SensorServiceADC2Resistance(uint16_t* measurement, LDRresistance_t* LDRResistance);
float SensorServiceADC2Voltage(uint16_t measurement);
void SensorServiceReadLimiter(uint8_t* limiter);

#endif /* INC_SERVICE_SENSORSERVICE_H_ */
