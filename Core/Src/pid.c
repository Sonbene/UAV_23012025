/*
 * pid.c
 *
 *  Created on: Jan 21, 2025
 *      Author: Sonbe
 */

#include "pid.h"

void PIDControlAccel(PIDType_t* pidtype, float accel, float setPoint)
{
	setPoint = map(setPoint, 1000, 2000, -30, 30);
	pidtype->setPoint = constrain(setPoint, -30, 30);
	pidtype->error = pidtype->setPoint - accel;

	pidtype->sumIntegral += pidtype->error * pidtype->timesampling;
	pidtype->sumIntegral = constrain(pidtype->sumIntegral, -500, 500);

	pidtype->derivative = (pidtype->error - pidtype->preverror)/pidtype->timesampling;
	pidtype->preverror = pidtype->error;

	pidtype->output = (pidtype->kp * pidtype->error) + (pidtype->kd * pidtype->derivative) + (pidtype->ki * pidtype->sumIntegral);
}

void PIDControlGyro(PIDType_t* pidtype, float gyro, float setPoint)
{
	//Write later
}

void PIDControlMag(PIDType_t* pidtype, float mag, float setPoint)
{
	//Write later
}

void PIDControlAltitude(PIDType_t* pidtype, float altitude, float setPoint)
{
	setPoint = map(setPoint, 1000, 2000, 0, 100);
	pidtype->setPoint = constrain(setPoint, 0, 100);
	pidtype->error = pidtype->setPoint - altitude;

	pidtype->sumIntegral += pidtype->error * pidtype->timesampling;
	pidtype->sumIntegral = constrain(pidtype->sumIntegral, -1000, 1000);

	pidtype->derivative = (pidtype->error - pidtype->preverror) / pidtype->timesampling;
	pidtype->preverror = pidtype->error;

	pidtype->output = (pidtype->kp * pidtype->error) + (pidtype->kd * pidtype->derivative) + (pidtype->ki * pidtype->sumIntegral);
}

void PIDReset(PIDType_t * pidtype)
{
	pidtype->sumIntegral = 0;
	pidtype->output = 0;
}

void PIDInit(PIDType_t *pidtype, double kp, double ki, double kd, double timesampling){
	PIDReset(pidtype);

	pidtype->kp = kp;
	pidtype->kd = kd;
	pidtype->ki = ki;

	pidtype->timesampling = timesampling;
}







