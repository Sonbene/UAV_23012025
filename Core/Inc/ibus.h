/*
 * icm20948_with_kalman.h
 *
 *  Created on: Jan 14, 2025
 *      Author: Sonbe
 */

#ifndef IBUS_H
#define IBUS_H

#include "main.h"

typedef struct _FSiA6B_iBus
{
	unsigned short RH; //Right Horizontal
	unsigned short RV; //Right Vertical
	unsigned short LV; //Left Vertical
	unsigned short LH; //Left Horizontal
	unsigned short SwA;
	unsigned short SwB;
	unsigned short SwC;
	unsigned short SwD;
	unsigned short VrA;
	unsigned short VrB;

	unsigned char FailSafe;
	unsigned char isReceived;// Does the STM check whether data from Ibus
}FSiA6B_iBus;

typedef enum
{
	IBUS_FREE,
	IBUS_PROCESS,
	IBUS_ERROR,
	IBUS_TIMEOUT
}IBUS_STATE_t;


unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len);
void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus);


#endif

