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
#include <stdbool.h>

#define resolution 1024
#define enkoder_cyclic_cnt 4
#define enkoder_freq  15000
#define Ts 0.000066
#define INA_PIN GPIO_PIN_11
#define INB_PIN GPIO_PIN_7
#define DIR_PORT GPIOA
typedef enum
{
	DEF_MODE,
	CURR_MODE,
	SPEED_MODE,
	POS_MODE,
	STOP_MODE
}motorMode;

typedef enum
{
	LEFT_DIR,
	RIGHT_DIR,
	STOP_DIR
}Direction;

typedef struct
{
	//wartości zadane w unii?
	float refCurr;
	float refSpeed;
	float refPos;

	uint16_t idx;
	bool endMeasurFlag;
	uint32_t dmaMeasurCurr;
	float measurCurr[8000];
	float measurSpeed[8000];
	float measurPos[8000];

	uint8_t tmpRx;
	uint8_t tmpData[1+7*2/*7*6+1*/];
	uint8_t stateRx;
}motor;

typedef struct
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
}currentControler;

typedef struct
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
}speedControler;

typedef struct
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
}positionControler;

void initPeripherals();
void initMotor();
void TxDataUART();
void measurTx();
void RxDecoding();
void RxDecoding2();
void testowa();
void controlMotor();
void defaultMotorMove();
void controlMotorMove();
void changeDir(Direction dir);
void regulator_PID_curr();
void regulator_PID_speed();
void speed_motor_calc();


#endif /* INC_PID_H_ */
