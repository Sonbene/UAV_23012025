/*
 * bmp280_with_kalman.c
 *
 *  Created on: Jan 16, 2025
 *      Author: Sonbe
 */

#include "bmp280_with_kalman.h"

float pressureRef = 0;
Kalman_t kalman_altitude;
BMP280_HandleTypedef bmp280;

void BMP_Filter_Kalman_Init(){
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;
	while(!bmp280_init(&bmp280, &bmp280.params)){

	}

	HAL_Delay(100);

	float pres_total = 0;
	float pressure, temperature, humidity;

	for(int i = 0; i < 100; ++i){
		while(bmp280_is_measuring(&bmp280)) continue;
		bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
		pres_total = pres_total + pressure;
	}
	pressureRef = pres_total / 100;

	kalman_init(&kalman_altitude, 0.12, 0.12, 0.055);
}

void getBMPAltitude(float* altitude){
	if(!bmp280_is_measuring(&bmp280)){
		  static float pressure, temperature, humidity;
		  bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
		  static float altitude_reading;
		  altitude_reading = bmp280_read_altitude(pressure, pressureRef);
		  *altitude = kalman_updateEstimate(&kalman_altitude, altitude_reading);
	}
}



