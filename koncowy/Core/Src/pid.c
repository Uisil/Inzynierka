/*
 * pid.c
 *
 *  Created on: Sep 21, 2022
 *      Author: hubli
 */

#include "pid.h"


enum motorMode motorMode;
struct motor motor;
struct currentControler c_c;
struct speedControler s_c;
struct positionControler p_c;



void init()
{
	  //__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 3000);
	  //HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  //HAL_ADC_Start_DMA(&hadc2, &motor.dmaMeasurCurr, 1);
	  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	  //HAL_UART_Receive_IT(&huart2, &motor.tmpRx, 1);
	  HAL_UART_Receive_DMA(&huart2, motor.tmpData, 1+7*2/*7*6+1*/);
}



int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC2)
	{
		motor.measurCurr = -1*(0.00395522*motor.dmaMeasurCurr-3.68421159);
		//TxDataUART();
		//motor.idx++;
		//if(idx>=8000) motor.idx = 0;
	}
}

void TxDataUART()
{
	printf("%d Kp:%f Ti:%f Td:%f \n",motorMode,c_c.Kp,c_c.Ti,c_c.Td);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		//static int nr=0;
		//motor.tmpData[nr] = motor.tmpRx;
		//printf("nufka sztuyka: %d\n",motor.tmpRx);
		/*static uint8_t idx = 0;
		switch(motor.stateRx)
		{
		case 0:
			if(motor.tmpRx == '0') motorMode = DEF_MODE;
			if(motor.tmpRx == '1') motorMode = CURR_MODE;
			if(motor.tmpRx == '2') motorMode = SPEED_MODE;
			if(motor.tmpRx == '3') motorMode = POS_MODE;
			motor.stateRx = 1;
			TxDataUART();
			break;
		case 1:
			if(idx==0) c_c.Kp = (motor.tmpRx-48)*100;
			if(idx==1) c_c.Kp += (motor.tmpRx-48)*10;
			if(idx==2) c_c.Kp += (motor.tmpRx-48)*1;
			if(idx==3) c_c.Kp += (motor.tmpRx-48)*0.1;
			if(idx==4) c_c.Kp += (motor.tmpRx-48)*0.01;
			if(idx==5)
			{
				c_c.Kp = (motor.tmpRx-48)*0.001;
				idx=0;
				motor.stateRx = 2;
			}
			idx++;
			TxDataUART();
			break;
		case 2:
			if(idx==0) c_c.Ti = (motor.tmpRx-48)*100;
			if(idx==1) c_c.Ti = (motor.tmpRx-48)*10;
			if(idx==2) c_c.Ti = (motor.tmpRx-48)*1;
			if(idx==3) c_c.Ti = (motor.tmpRx-48)*0.1;
			if(idx==4) c_c.Ti = (motor.tmpRx-48)*0.01;
			if(idx==5)
			{
				c_c.Ti = (motor.tmpRx-48)*0.001;
				idx=0;
				motor.stateRx = 3;
			}
			idx++;
			TxDataUART();
			break;
		case 3:
			if(idx==0) c_c.Td = (motor.tmpRx-48)*100;
			if(idx==1) c_c.Td = (motor.tmpRx-48)*10;
			if(idx==2) c_c.Td = (motor.tmpRx-48)*1;
			if(idx==3) c_c.Td = (motor.tmpRx-48)*0.1;
			if(idx==4) c_c.Td = (motor.tmpRx-48)*0.01;
			if(idx==5)
			{
				c_c.Td = (motor.tmpRx-48)*0.001;
				idx=0;
				motor.stateRx = 0;
			}
			idx++;
			TxDataUART();
			break;
		default:
			break;
		}*/
		//HAL_UART_Receive_IT(&huart2, &motor.tmpRx, 1);
	}
}

void RxDecoding()
{

	if(motor.tmpData[0] == '0') motorMode = DEF_MODE;
	else if(motor.tmpData[0] == '1') motorMode = CURR_MODE;
	else if(motor.tmpData[0] == '2') motorMode = SPEED_MODE;
	else if(motor.tmpData[0] == '3') motorMode = POS_MODE;
	else if(motor.tmpData[0] == '4') motorMode = STOP_MODE;

	motor.refCurr = (motor.tmpData[1]-48)*100;
	motor.refCurr += (motor.tmpData[2]-48)*10;
	motor.refCurr += (motor.tmpData[3]-48)*1;
	motor.refCurr += (motor.tmpData[4]-48)*0.1;
	motor.refCurr += (motor.tmpData[5]-48)*0.01;
	motor.refCurr += (motor.tmpData[6]-48)*0.001;

	//c_c.Kp = tmpData[7]*100;
	//c_c.Kp = tmpData[8]*10;
	//c_c.Kp = tmpData[9]*1;
	//c_c.Kp = tmpData[10]*0.1;
	//c_c.Kp = tmpData[11]*0.01;
	//c_c.Kp = tmpData[12]*0.001;

	c_c.Kp = (motor.tmpData[13]-48)*100;
	c_c.Kp += (motor.tmpData[14]-48)*10;
	c_c.Kp += (motor.tmpData[15]-48)*1;
	c_c.Kp += (motor.tmpData[16]-48)*0.1;
	c_c.Kp += (motor.tmpData[17]-48)*0.01;
	c_c.Kp += (motor.tmpData[18]-48)*0.001;

	c_c.Ti = (motor.tmpData[19]-48)*100;
	c_c.Ti += (motor.tmpData[20]-48)*10;
	c_c.Ti += (motor.tmpData[21]-48)*1;
	c_c.Ti += (motor.tmpData[22]-48)*0.1;
	c_c.Ti += (motor.tmpData[23]-48)*0.01;
	c_c.Ti += (motor.tmpData[24]-48)*0.001;

	c_c.Td = (motor.tmpData[25]-48)*100;
	c_c.Td += (motor.tmpData[26]-48)*10;
	c_c.Td += (motor.tmpData[27]-48)*1;
	c_c.Td += (motor.tmpData[28]-48)*0.1;
	c_c.Td += (motor.tmpData[29]-48)*0.01;
	c_c.Td += (motor.tmpData[30]-48)*0.001;

	c_c.Kff = (motor.tmpData[31]-48)*100;
	c_c.Kff += (motor.tmpData[32]-48)*10;
	c_c.Kff += (motor.tmpData[33]-48)*1;
	c_c.Kff += (motor.tmpData[34]-48)*0.1;
	c_c.Kff += (motor.tmpData[35]-48)*0.01;
	c_c.Kff += (motor.tmpData[36]-48)*0.001;

	c_c.Kaw = (motor.tmpData[37]-48)*100;
	c_c.Kaw += (motor.tmpData[38]-48)*10;
	c_c.Kaw += (motor.tmpData[39]-48)*1;
	c_c.Kaw += (motor.tmpData[40]-48)*0.1;
	c_c.Kaw += (motor.tmpData[41]-48)*0.01;
	c_c.Kaw += (motor.tmpData[42]-48)*0.001;
}

void RxDecoding2()
{
	if(motor.tmpData[0] == 0) motorMode = DEF_MODE;
	else if(motor.tmpData[0] == 1) motorMode = CURR_MODE;
	else if(motor.tmpData[0] == 2) motorMode = SPEED_MODE;
	else if(motor.tmpData[0] == 3) motorMode = POS_MODE;
	else if(motor.tmpData[0] == 4) motorMode = STOP_MODE;

	motor.refCurr = (float)((motor.tmpData[1]<<8) + motor.tmpData[2])/1000;
	//c_c.Ti = (float)((motor.tmpData[3]<<8) + motor.tmpData[4])/1000;
	c_c.Kp = (float)((motor.tmpData[5]<<8) + motor.tmpData[6])/1000;
	c_c.Ti = (float)((motor.tmpData[7]<<8) + motor.tmpData[8])/1000;
	c_c.Td = (float)((motor.tmpData[9]<<8) + motor.tmpData[10])/1000;
	c_c.Kff = (float)((motor.tmpData[11]<<8) + motor.tmpData[12])/1000;
	c_c.Kaw = (float)((motor.tmpData[13]<<8) + motor.tmpData[14])/1000;
}

void testowa()
{
	  if(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
	  {
		  RxDecoding2();
		  printf("tryb:%d ref:%f Kp:%f Ti:%f Td:%f sat:%f Kw:%f\n",motorMode,motor.refCurr,c_c.Kp,c_c.Ti,c_c.Td,c_c.sat,c_c.Kaw);


		  /*for(int i = 0; i<8*6+1;i++)
		  {
			  printf("bla bla: %d\n",motor.tmpData[i]);
		  }*/
	  }
}

void controlMotor()
{
	printf("asd");
}
