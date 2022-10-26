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
#include "dac.h"
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
#define TsSpeed 0.000660
#define INA_PIN GPIO_PIN_11
#define INB_PIN GPIO_PIN_7
#define DIR_PORT GPIOA
#define FRAME_RECIVE_WIDITH 6+30*4
typedef enum
{
	DEF_MODE,
	CURR_MODE,
	SPEED_MODE,
	POS_MODE,
	STOP_MODE,
	FREE_MODE
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
	volatile float refCurr;
	volatile float refSpeed;
	volatile float refPos;
	uint16_t valueLoad;

	uint16_t idx;
	bool endMeasurFlag;
	bool moveInProgress;
	uint32_t dmaMeasurCurr;
	volatile float measurCurr[8000];
	volatile float measurSpeed[8000];
	volatile float measurPos[8000];
	volatile float simTime;
	volatile float time;

	uint8_t tmpRx;
	uint8_t tmpData[FRAME_RECIVE_WIDITH];
	uint8_t stateRx;

	float timeValuePattern[5];
	float refValuePattern[5];
	float timeLoadPattern[5];
	uint16_t valueLoadPattern[5];
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

typedef struct
{
	int32_t enkoder_tmp;
	int32_t enkoder_cnt;
	int16_t enkoder_cnt_old;
	int16_t ringbuffer[100];
	uint32_t idx;
	int32_t tmp;
	int32_t diff;
}measureSpeed;

void initPeripherals();
void initMotor();

void transmitData();

void reciveData();
void reciveRefPattern();
void reciveLoadPattern();
void reciveMotorParameters();
void reciveCurrPIDParameters();
void reciveSpeedPIDParameters();
void recivePosPIDParameters();

void setRefValue(float *refValue);
void setMotorLoad();

void controlMotor2();
void controlMotor();
void defaultMotorMove();
void controlMotorMove();
void changeDir(Direction dir);

void regulator_PID_curr();
void regulator_PID_speed();
void regulator_PID_pos();

void enkoderMeasure();;
void speedCalc();
void posCalc();

void resetPID();
void resetData();

void initControler();
void initCurrPID();
void initSpeedPID();
void initPosPID();
#endif /* INC_PID_H_ */
