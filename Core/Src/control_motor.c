/*
 * control_motor.c
 *
 *  Created on: Jan 21, 2025
 *      Author: Sonbe
 */

#include "pid.h"
#include "control_motor.h"

void motor_init(TIM_HandleTypeDef* htim)
{
	  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);

//	  htim->Instance->ARR = 19999;
//	  htim->Instance->PSC = 99;

	  htim->Instance->CCR1 = 1000;
	  htim->Instance->CCR2 = 1000;
	  htim->Instance->CCR3 = 1000;
	  htim->Instance->CCR4 = 1000;
}

void motor_stop(TIM_HandleTypeDef* htim)
{
	  htim->Instance->CCR1 = 1000;
	  htim->Instance->CCR2 = 1000;
	  htim->Instance->CCR3 = 1000;
	  htim->Instance->CCR4 = 1000;
}

void motor_reset(TIM_HandleTypeDef* htim, Motor_param_t* motor_param)
{
	htim->Instance->CCR1 = 1000;
	htim->Instance->CCR2 = 1000;
	htim->Instance->CCR3 = 1000;
	htim->Instance->CCR4 = 1000;

	motor_param->PIDAltitude.sumIntegral = motor_param->PIDPitch.sumIntegral = motor_param->PIDRoll.sumIntegral = motor_param->PIDYaw.sumIntegral = 0;
	motor_param->PIDAltitude.output = motor_param->PIDPitch.output = motor_param->PIDRoll.output = motor_param->PIDYaw.output = 0;
}

void motor_control(TIM_HandleTypeDef* htim, Motor_param_t* motor_param, FSiA6B_iBus iBus)
{
	/*
				CW 2\     /1 CCW
					 \   /
					  \ /
					  / \
					 /   \
			   CCW 3/     \4 CW
				   QuadCopter
				   Motor 2205 KV
				   Propeller : 6 x 4.3
				   MaX thrust = 83.36 N
				   Thrust for each motor = 20.84
			Formula :
			F = 1.225 * ((3.14(0.0254 * d)^2)/4) *(RPM*0.0254*pitch*1/60)^2 * (d/(3.29546*pitch))^1.5;
			F = 0.050252560785 * 522.5796 * 0.47279287884410658042558653798071
		*/

	float rollPWM = motor_param->PIDRoll.output;
	float pitchPWM = motor_param->PIDPitch.output;
	float yawPWM = motor_param->PIDYaw.output;
	float altitudePWM = motor_param->PIDAltitude.output;
	float thrust, Altitudetrush;
	motor_param->inputThrottle = iBus.LV;

	if(motor_param->flyMode == FLY_MODE_ON)
	{
		thrust = map(motor_param->inputThrottle, 1000, 2000, 1000, 2000);
	}
	else if(motor_param->flyMode == FLY_MODE_HOLD)
	{
		if (motor_param->inputThrottle >= 1200)
		{
			Altitudetrush = 1.3 * motor_param->PIDAltitude.output;
			thrust = 20.84 + Altitudetrush;
		}
		else
			thrust = map(motor_param->inputThrottle, 1000, 2000, 0, 83.36);
	}

	motor_param->pulseESC1 = thrust + rollPWM + pitchPWM;
	motor_param->pulseESC2 = thrust - rollPWM + pitchPWM;
	motor_param->pulseESC3 = thrust + rollPWM - pitchPWM;
	motor_param->pulseESC4 = thrust - rollPWM - pitchPWM;

	motor_param->pulseESC1 = constrain(motor_param->pulseESC1, 1000, 2000);
	motor_param->pulseESC2 = constrain(motor_param->pulseESC2, 1000, 2000);
	motor_param->pulseESC3 = constrain(motor_param->pulseESC3, 1000, 2000);
	motor_param->pulseESC4 = constrain(motor_param->pulseESC4, 1000, 2000);

	htim->Instance->CCR1 = motor_param->pulseESC1;
	htim->Instance->CCR2 = motor_param->pulseESC2;
	htim->Instance->CCR3 = motor_param->pulseESC3;
	htim->Instance->CCR4 = motor_param->pulseESC4;

}


