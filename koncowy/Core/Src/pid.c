/*
 * pid.c
 *
 *  Created on: Sep 21, 2022
 *      Author: hubli
 */

#include "pid.h"

Direction dir;
motorMode mode;
motor m;
currentControler c_c;
speedControler s_c;
positionControler p_c;



void initPeripherals()
{
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  //HAL_UART_Receive_DMA(&huart2, m.tmpData, 1+7*2/*7*6+1*/);
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, m.tmpData, 1+7*2);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart2)
	{
		RxDecoding2();
		//printf("tryb:%d ref:%f Kp:%f Ti:%f Td:%f sat:%f Kw:%f\n",mode,m.refCurr,c_c.Kp,c_c.Ti,c_c.Td,c_c.sat,c_c.Kaw);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, m.tmpData, 1+7*2);

	}
}


void initMotor()
{
	//zatrzymanie silnika
	mode = STOP_MODE;
	m.endMeasurFlag = false;

	//inicjalizacja parametrow regulatora PID prądu
	c_c.Kp = 0;
	c_c.Ti = 0;
	c_c.Td = 0;
	c_c.Kff = 0;
	c_c.Kaw = 0;
	c_c.sat = 0;
	c_c.pid_I_prev = 0;
	c_c.u_prev = 0;

	//inicjalizacja parametrow regulatora PID predkości
	s_c.Kp = 0;
	s_c.Ti = 0;
	s_c.Td = 0;
	s_c.Kff = 0;
	s_c.Kaw = 0;
	s_c.sat = 0;
	s_c.pid_I_prev = 0;
	s_c.u_prev = 0;

	//inicjalizacja parametrow regulatora PID położenia
	p_c.Kp = 0;
	p_c.Ti = 0;
	p_c.Td = 0;
	p_c.Kff = 0;
	p_c.Kaw = 0;
	p_c.sat = 0;
	p_c.pid_I_prev = 0;
	p_c.u_prev = 0;
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
		m.measurCurr[m.idx] = -1*(0.00395522*m.dmaMeasurCurr-3.68421159);
		m.idx++;

		//TxDataUART();
		//motor.idx++;
		//if(idx>=8000) motor.idx = 0;
	}
}

void TxDataUART()
{
	for(int i=0;i<=8000;i++)
	{
		printf("%f\n",m.measurCurr[i]);
	}
}

void RxDecoding()
{

	if(m.tmpData[0] == '0') mode = DEF_MODE;
	else if(m.tmpData[0] == '1') mode = CURR_MODE;
	else if(m.tmpData[0] == '2') mode = SPEED_MODE;
	else if(m.tmpData[0] == '3') mode = POS_MODE;
	else if(m.tmpData[0] == '4') mode = STOP_MODE;

	m.refCurr = (m.tmpData[1]-48)*100;
	m.refCurr += (m.tmpData[2]-48)*10;
	m.refCurr += (m.tmpData[3]-48)*1;
	m.refCurr += (m.tmpData[4]-48)*0.1;
	m.refCurr += (m.tmpData[5]-48)*0.01;
	m.refCurr += (m.tmpData[6]-48)*0.001;

	//c_c.Kp = tmpData[7]*100;
	//c_c.Kp = tmpData[8]*10;
	//c_c.Kp = tmpData[9]*1;
	//c_c.Kp = tmpData[10]*0.1;
	//c_c.Kp = tmpData[11]*0.01;
	//c_c.Kp = tmpData[12]*0.001;

	c_c.Kp = (m.tmpData[13]-48)*100;
	c_c.Kp += (m.tmpData[14]-48)*10;
	c_c.Kp += (m.tmpData[15]-48)*1;
	c_c.Kp += (m.tmpData[16]-48)*0.1;
	c_c.Kp += (m.tmpData[17]-48)*0.01;
	c_c.Kp += (m.tmpData[18]-48)*0.001;

	c_c.Ti = (m.tmpData[19]-48)*100;
	c_c.Ti += (m.tmpData[20]-48)*10;
	c_c.Ti += (m.tmpData[21]-48)*1;
	c_c.Ti += (m.tmpData[22]-48)*0.1;
	c_c.Ti += (m.tmpData[23]-48)*0.01;
	c_c.Ti += (m.tmpData[24]-48)*0.001;

	c_c.Td = (m.tmpData[25]-48)*100;
	c_c.Td += (m.tmpData[26]-48)*10;
	c_c.Td += (m.tmpData[27]-48)*1;
	c_c.Td += (m.tmpData[28]-48)*0.1;
	c_c.Td += (m.tmpData[29]-48)*0.01;
	c_c.Td += (m.tmpData[30]-48)*0.001;

	c_c.Kff = (m.tmpData[31]-48)*100;
	c_c.Kff += (m.tmpData[32]-48)*10;
	c_c.Kff += (m.tmpData[33]-48)*1;
	c_c.Kff += (m.tmpData[34]-48)*0.1;
	c_c.Kff += (m.tmpData[35]-48)*0.01;
	c_c.Kff += (m.tmpData[36]-48)*0.001;

	c_c.Kaw = (m.tmpData[37]-48)*100;
	c_c.Kaw += (m.tmpData[38]-48)*10;
	c_c.Kaw += (m.tmpData[39]-48)*1;
	c_c.Kaw += (m.tmpData[40]-48)*0.1;
	c_c.Kaw += (m.tmpData[41]-48)*0.01;
	c_c.Kaw += (m.tmpData[42]-48)*0.001;
}

void RxDecoding2()
{
	if(m.tmpData[0] == 0) mode = DEF_MODE;
	else if(m.tmpData[0] == 1) mode = CURR_MODE;
	else if(m.tmpData[0] == 2) mode = SPEED_MODE;
	else if(m.tmpData[0] == 3) mode = POS_MODE;
	else if(m.tmpData[0] == 4) mode = STOP_MODE;

	m.refCurr = (float)((m.tmpData[1]<<8) + m.tmpData[2])/1000;
	//c_c.Ti = (float)((motor.tmpData[3]<<8) + motor.tmpData[4])/1000;
	c_c.Kp = (float)((m.tmpData[5]<<8) + m.tmpData[6])/1000;
	c_c.Ti = (float)((m.tmpData[7]<<8) + m.tmpData[8])/1000;
	c_c.Td = (float)((m.tmpData[9]<<8) + m.tmpData[10])/1000;
	c_c.Kff = (float)((m.tmpData[11]<<8) + m.tmpData[12])/1000;
	c_c.Kaw = (float)((m.tmpData[13]<<8) + m.tmpData[14])/1000;
}

void testowa()
{
	  if(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
	  {
		  RxDecoding2();
		  //printf("tryb:%d ref:%f Kp:%f Ti:%f Td:%f sat:%f Kw:%f\n",mode,m.refCurr,c_c.Kp,c_c.Ti,c_c.Td,c_c.sat,c_c.Kaw);
	  }
}

void measurTx()
{
	  if(m.endMeasurFlag == true && __HAL_TIM_GET_COMPARE(&htim8,TIM_CHANNEL_1) == 0)
	  {
		  for(int i=0;i<=8000;i++)
		  {
			  printf("%f\n",m.measurCurr[i]);
		  }
		  m.endMeasurFlag = false;
	  }
}

void changeDir(Direction dir)
{
	if(dir == LEFT_DIR)
	{
		HAL_GPIO_WritePin(DIR_PORT, INA_PIN,1);
		HAL_GPIO_WritePin(DIR_PORT, INB_PIN,0);
	}
	if(dir == RIGHT_DIR)
	{
		HAL_GPIO_WritePin(DIR_PORT, INA_PIN,0);
		HAL_GPIO_WritePin(DIR_PORT, INB_PIN,1);
	}
	if(dir == STOP_DIR)
	{
		HAL_GPIO_WritePin(DIR_PORT, INA_PIN,0);
		HAL_GPIO_WritePin(DIR_PORT, INB_PIN,0);
	}
}

void defaultMotorMove()
{
	static uint32_t tmp = 0;
	if(__HAL_TIM_GET_COMPARE(&htim8,TIM_CHANNEL_1) == 0) tmp=HAL_GetTick();

	if(HAL_GetTick() - tmp<200)
	{
		changeDir(RIGHT_DIR);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, 3000);
	}
	else if(HAL_GetTick()-tmp>=200 && HAL_GetTick()-tmp<400)
	{
		changeDir(LEFT_DIR);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, 3000);
	}
	else
	{
		m.endMeasurFlag = true;
		mode = STOP_MODE;
	}
}

void controlMotor()
{
	switch(mode)
	{
	case DEF_MODE:
		HAL_ADC_Start_DMA(&hadc2, &m.dmaMeasurCurr, 1);
		defaultMotorMove();
		break;
	case CURR_MODE:
		HAL_ADC_Start_DMA(&hadc2, &m.dmaMeasurCurr, 1);
		break;
	case SPEED_MODE:
		break;
	case POS_MODE:
		break;
	case STOP_MODE:
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		changeDir(STOP_DIR);
		HAL_ADC_Stop_DMA(&hadc2);
		m.idx = 0;
		break;
	default:
		break;
	}
}
