/*
 * icm20948_with_kalman.h
 *
 *  Created on: Jan 19, 2025
 *      Author: Sonbe
 */

#ifndef INC_ICM20948_WITH_KALMAN_H_
#define INC_ICM20948_WITH_KALMAN_H_

#include "kalman.h"
#include "icm20948.h"
#include "main.h"

typedef struct{
	float yaw;
	float pitch;
	float roll;
	float gyro_x;
	float gyro_y;
	float gyro_z;
}ICM_Data_t;

void ICM_Filter_Kalman_Init(void);
void ICMRef(ICM_Data_t* ICM_set_point);
void getICMAccel(axises* accel);
void getICMGyro(axises* gyro);
void getICMMag(axises* mag);
void getICMData(ICM_Data_t* icmData);

#endif /* INC_ICM20948_WITH_KALMAN_H_ */
