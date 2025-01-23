/*
 * pid.h
 *
 *  Created on: Jan 21, 2025
 *      Author: Sonbe
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
#define constrain(nilaix,bawah,atas) ( (nilaix)<(bawah) ? (bawah) : ( (nilaix)>(atas) ? (atas) : (nilaix) ) )

#include "main.h"
#include "math.h"

typedef struct {
	float error;
	float preverror;

	float derivative;
	float sumIntegral;
	float setPoint;

	float output;

	float kp;
	float kd;
	float ki;

	float timesampling;
} PIDType_t;

void PIDInit(PIDType_t *pidtype, double kp, double ki, double kd, double timesampling);
void PIDReset(PIDType_t * pidtype);
void PIDControlAltitude(PIDType_t* pidtype, float altitude, float setPoint);
void PIDControlMag(PIDType_t* pidtype, float mag, float setPoint);
void PIDControlGyro(PIDType_t* pidtype, float gyro, float setPoint);
void PIDControlAccel(PIDType_t* pidtype, float accel, float setPoint);



#endif /* INC_PID_H_ */



