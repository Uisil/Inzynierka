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
int32_t enkoder_tmp = 0;
int32_t enkoder_cnt = 0;



void initPeripherals()
{
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
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
	mode = SPEED_MODE;
	m.endMeasurFlag = false;

	//inicjalizacja parametrow regulatora PID prądu
	c_c.Kp = 0.8;
	c_c.Ti = 0.01;
	c_c.Td = 0;
	c_c.Kff = 0;
	c_c.Kaw = 0;
	c_c.sat = 1;
	c_c.pid_I_prev = 0;
	c_c.u_prev = 0;
	m.refCurr = 0.5;

	//inicjalizacja parametrow regulatora PID predkości
	s_c.Kp = 1;
	s_c.Ti = 100000000000;
	s_c.Td = 0;
	s_c.Kff = 0;
	s_c.Kaw = 0;
	s_c.sat = 3.3;
	s_c.pid_I_prev = 0;
	s_c.u_prev = 0;
	m.refSpeed = 1000;

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

void speed_motor_calc()
{
	static int16_t ringbuffer[100] = {0};
	static uint32_t idx = 0;
	static int32_t tmp = 0;
	static int32_t diff = 0;

	// rzutowanie na int16 daje zakres od -100 do 100;
	enkoder_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);

	if(idx++ >= 100) idx = 0;

	tmp = ringbuffer[idx];
	ringbuffer[idx] = enkoder_cnt;

	diff = (enkoder_cnt - tmp);
	enkoder_tmp +=diff;
	/*if(diff>1500)
	{
		diff = -32768-tmp-(32768-enkoder_cnt);
		enkoder_tmp -= 32768;
	}
	if(diff<-1500)
	{
		diff = 32768-tmp+(32768+enkoder_cnt);
		enkoder_tmp += 32768;
	}*/
	m.measurPos[m.idx] = (float)enkoder_tmp/(resolution*enkoder_cyclic_cnt);
	//s_m.speed = (float)s_m.enkoder_cnt * enkoder_freq * 60/(resolution*enkoder_cyclic_cnt);
	m.measurSpeed[m.idx] = (60*(float)diff*enkoder_freq/99)/(resolution*enkoder_cyclic_cnt);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC2)
	{
		m.measurCurr[m.idx] = -1*(0.00395522*m.dmaMeasurCurr-3.68421159);
		speed_motor_calc();
		if(mode == CURR_MODE)
		{
			regulator_PID_curr();
		}
		if(mode == SPEED_MODE)
		{
			regulator_PID_speed();
			regulator_PID_curr();
		}
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
	printf("tryb:%d ref:%f Kp:%f Ti:%f Td:%f sat:%f Kw:%f\n",mode,m.refCurr,c_c.Kp,c_c.Ti,c_c.Td,c_c.sat,c_c.Kaw);
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
		  for(int i=0;i<=8000;i++)
		  {
			  printf("%f\n",m.measurSpeed[i]);
		  }
		  for(uint32_t i=0;i<=8000;i++)
		  {
			  printf("%f\n",m.measurPos[i]);
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

void controlMotorMove()
{
	static uint32_t tmp = 0;
	if(__HAL_TIM_GET_COMPARE(&htim8,TIM_CHANNEL_1) == 0) tmp=HAL_GetTick();

	if(HAL_GetTick() - tmp<400)
	{

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
		controlMotorMove();
		break;
	case SPEED_MODE:
		HAL_ADC_Start_DMA(&hadc2, &m.dmaMeasurCurr, 1);
		controlMotorMove();
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

void regulator_PID_curr()
{
	static uint16_t new_pwm = 0;
	float e = m.refCurr - m.measurCurr[m.idx];

	float pid_P = c_c.Kp*e;
	float pid_I = c_c.pid_I_prev + Ts*(e*c_c.Kp-c_c.Kaw*(c_c.y-c_c.y_curr));
	float pid_D = (e*c_c.Kp-c_c.u_prev)/Ts;

	c_c.pid_I_prev = pid_I;
	c_c.u_prev = pid_P;

	c_c.y = pid_P + pid_I/c_c.Ti + pid_D*c_c.Td;

	if(c_c.y > c_c.sat) c_c.y_curr = c_c.sat;
	else if(c_c.y < -c_c.sat) c_c.y_curr = -c_c.sat;
	else c_c.y_curr = c_c.y;

	if(c_c.y_curr>=0)
	{
		changeDir(LEFT_DIR);
		new_pwm = (uint16_t)(6000*c_c.y_curr);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,new_pwm);
	}
	else
	{
		changeDir(RIGHT_DIR);
		new_pwm = (uint16_t)(-6000*c_c.y_curr);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, new_pwm);
	}
}


void regulator_PID_speed()
{
	float e = m.refSpeed - m.measurSpeed[m.idx];

	float pid_P = s_c.Kp*e;
	float pid_I = s_c.pid_I_prev + Ts*(e*s_c.Kp-s_c.Kaw*(s_c.y-s_c.y_speed));
	float pid_D = (e*s_c.Kp-s_c.u_prev)/Ts;

	s_c.pid_I_prev = pid_I;
	s_c.u_prev = pid_P;

	s_c.y = pid_P + pid_I/s_c.Ti + pid_D*s_c.Td;

	if(s_c.y > s_c.sat) s_c.y_speed = s_c.sat;
	else if(s_c.y < -s_c.sat) s_c.y_speed = -s_c.sat;
	else s_c.y_speed = s_c.y;

	m.refCurr = s_c.y_speed;
}
