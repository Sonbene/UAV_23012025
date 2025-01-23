/*
 * bmp280_with_kalman.h
 *
 *  Created on: Jan 16, 2025
 *      Author: Sonbe
 */

#ifndef INC_BMP280_WITH_KALMAN_H_
#define INC_BMP280_WITH_KALMAN_H_

#include "kalman.h"
#include "bmp280.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void BMP_Filter_Kalman_Init();
void getBMPAltitude(float* altitude);

#endif /* INC_BMP280_WITH_KALMAN_H_ */
