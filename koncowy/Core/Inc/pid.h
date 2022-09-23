/*
 * pid.h
 *
 *  Created on: Sep 21, 2022
 *      Author: hubli
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>

#define resolution 1024
#define enkoder_cyclic_cnt 4
#define enkoder_freq  15000
#define Ts 0.000066
#define INA GPIO_PIN_11
#define INB GPIO_PIN_7

enum motorMode
{
	DEF_MODE,
	CURR_MODE,
	SPEED_MODE,
	POS_MODE,
	STOP_MODE
};

struct motor
{
	//wartości zadane w unii?
	float refCurr;
	float refSpeed;
	float refPos;

	uint16_t idx;
	uint32_t dmaMeasurCurr;
	float measurCurr;
	float measurSpeed[8000];
	float measurPos[8000];

	uint8_t tmpRx;
	uint8_t tmpData[1+7*2/*7*6+1*/];
	uint8_t stateRx;
};

struct currentControler
{
	float Kp;
	float Ti;
	float Td;
	float Kff;
	float sat;
	float Kaw;

	float pid_I_prev;
	float y;
	float y_curr;
	float u_prev;
};

struct speedControler
{
	float Kp;
	float Ti;
	float Td;
	float Kff;
	float sat;
	float Kaw;

	float pid_I_prev;
	float y;
	float y_speed;
	float u_prev;
};

struct positionControler
{
	float Kp;
	float Ti;
	float Td;
	float Kff;
	float sat;
	float Kaw;

	float pid_I_prev;
	float y;
	float y_pos;
	float u_prev;
};

void init();
void TxDataUART();
void RxDecoding();
void RxDecoding2();
void testowa();
void controlMotor();


#endif /* INC_PID_H_ */
