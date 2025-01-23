/*
 * control_motor.h
 *
 *  Created on: Jan 21, 2025
 *      Author: Sonbe
 */

#ifndef INC_CONTROL_MOTOR_H_
#define INC_CONTROL_MOTOR_H_

#include "ibus.h"
#include "math.h"
#include "main.h"
#include "pid.h"

typedef enum {
	FLY_MODE_OFF,
	FLY_MODE_ON,
	FLY_MODE_HOLD
}FLY_MODE_t;

typedef struct{
	PIDType_t PIDYaw;
	PIDType_t PIDRoll;
	PIDType_t PIDPitch;
	PIDType_t PIDAltitude;

	uint16_t pulseESC1;
	uint16_t pulseESC2;
	uint16_t pulseESC3;
	uint16_t pulseESC4;

	uint16_t inputYaw;
	uint16_t inputPitch;
	uint16_t inputRoll;
	uint16_t inputAltitude;

	uint16_t inputThrottle;
	uint16_t inputFlyMode;
	uint16_t holdThrottle;

	FLY_MODE_t flyMode;
}Motor_param_t;


void motor_init(TIM_HandleTypeDef* htim);
void motor_stop(TIM_HandleTypeDef* htim);
void motor_reset(TIM_HandleTypeDef* htim, Motor_param_t* motor_param);
void motor_control(TIM_HandleTypeDef* htim, Motor_param_t* motor_param, FSiA6B_iBus iBus);


#endif /* INC_CONTROL_MOTOR_H_ */
