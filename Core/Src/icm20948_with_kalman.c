/*
 * icm20948_with_kalman.c
 *
 *  Created on: Jan 19, 2025
 *      Author: Sonbe
 */

#include "icm20948_with_kalman.h"

// Gia tốc kế
Kalman_t accelKalmanX, accelKalmanY, accelKalmanZ;

// Con quay hồi chuyển
Kalman_t gyroKalmanX, gyroKalmanY, gyroKalmanZ;

// Từ kế
Kalman_t magKalmanX, magKalmanY, magKalmanZ;

void ICM_Filter_Kalman_Init()
{
	icm20948_init();
	ak09916_init();

	kalman_init(&accelKalmanX, 0.01, 0.1, 0.01); // Trục X
	kalman_init(&accelKalmanY, 0.01, 0.1, 0.01); // Trục Y
	kalman_init(&accelKalmanZ, 0.01, 0.1, 0.01); // Trục Z

	kalman_init(&gyroKalmanX, 0.5, 1.0, 0.01);
	kalman_init(&gyroKalmanY, 0.5, 1.0, 0.01);
	kalman_init(&gyroKalmanZ, 0.5, 1.0, 0.01);

	kalman_init(&magKalmanX, 0.5, 1.0, 0.001);
	kalman_init(&magKalmanY, 0.5, 1.0, 0.001);
	kalman_init(&magKalmanZ, 0.5, 1.0, 0.001);
}

void ICMRef(ICM_Data_t* ICM_set_point)
{
	float yTotal = 0, zTotal = 0, xTotal = 0;
	float yRef = 0, zRef = 0, xRef = 0;

	axises my_accel, my_mag;
	int i = 0;

	//HAL_Delay(1000);

	while(i < 100)
	{
		i += 1;
		icm20948_accel_read_g(&my_accel);
		ak09916_mag_read_uT(&my_mag);
		yTotal += my_accel.y;
		xTotal += my_accel.x;
		zTotal += my_accel.z;
	}
	yRef = yTotal / 100;
	zRef = zTotal / 100;
	xRef = xTotal / 100;


	ICM_set_point->pitch = atan2(yRef, sqrt(xRef * xRef + zRef * zRef)) * 180.0 / M_PI;  // Chuyển sang độ
	ICM_set_point->roll = atan2(xRef, sqrt(yRef * yRef + zRef * zRef)) * 180.0 / M_PI;
	//ICM_set_point->yaw = atan2(-mag_y_Ref,mag_x_Ref) * 180 / M_PI;
}

void getICMData(ICM_Data_t* icmData)
{
	static axises accel_reading;
	static axises gyro_reading;
	static axises mag_reading;
	float mx, my, mz;
	float ax, ay, az;

	bool ak09916_state = ak09916_mag_read_uT(&mag_reading);
	icm20948_gyro_read_dps(&gyro_reading);
	icm20948_accel_read_g(&accel_reading);

	ax = kalman_updateEstimate(&accelKalmanX, accel_reading.x);
	ay = kalman_updateEstimate(&accelKalmanY, accel_reading.y);
	az = kalman_updateEstimate(&accelKalmanZ, accel_reading.z);

	icmData->gyro_x = kalman_updateEstimate(&gyroKalmanX, gyro_reading.x);
	icmData->gyro_y = kalman_updateEstimate(&gyroKalmanY, gyro_reading.y);
	icmData->gyro_z = kalman_updateEstimate(&gyroKalmanZ, gyro_reading.z);

	if(ak09916_state)
	{
		mx = kalman_updateEstimate(&magKalmanX, mag_reading.x);
		my = kalman_updateEstimate(&magKalmanY, mag_reading.y);
		mz = kalman_updateEstimate(&magKalmanZ, mag_reading.z);
	}

	icmData->roll = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI ;  // Chuyển sang độ
	icmData->pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
	//icmData->yaw = atan2(-my,mx) * 180 / M_PI;
}

void getICMAccel(axises* accel)
{
	static axises accel_reading;
	icm20948_accel_read_g(&accel_reading);

	accel->x = kalman_updateEstimate(&accelKalmanX, accel_reading.x);
	accel->y = kalman_updateEstimate(&accelKalmanY, accel_reading.y);
	accel->z = kalman_updateEstimate(&accelKalmanZ, accel_reading.z);
}

void getICMGyro(axises* gyro)
{
	static axises gyro_reading;
	icm20948_gyro_read_dps(&gyro_reading);

	gyro->x = kalman_updateEstimate(&gyroKalmanX, gyro_reading.x);
	gyro->y = kalman_updateEstimate(&gyroKalmanY, gyro_reading.y);
	gyro->z = kalman_updateEstimate(&gyroKalmanZ, gyro_reading.z);
}

void getICMMag(axises* mag)
{
	static axises mag_reading;
	if(ak09916_mag_read_uT(&mag_reading))
	{
		mag->x = kalman_updateEstimate(&magKalmanX, mag_reading.x);
		mag->y = kalman_updateEstimate(&magKalmanY, mag_reading.y);
		mag->z = kalman_updateEstimate(&magKalmanZ, mag_reading.z);
	}
}










