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
measureSpeed mSpeed;



void initPeripherals()
{
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim1,0);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, m.tmpData, FRAME_RECIVE_WIDITH);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4000);// 610 TO WARTOŚĆ 0.140 V
	HAL_ADC_Start_DMA(&hadc2, &m.dmaMeasurCurr, 1);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart2)
	{
		reciveData();
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, m.tmpData, FRAME_RECIVE_WIDITH);

	}
}


void initMotor()
{
	initControler();
	initCurrPID();
	initSpeedPID();
	initPosPID();
}

void initControler()
{
	mode = DEF_MODE;
	m.endMeasurFlag = false;
	m.moveInProgress = false;
}

void initCurrPID()
{
	c_c.Kp = 0.8;
	c_c.Ti = 0.01;
	c_c.Td = 0;
	c_c.Kff = 0;
	c_c.Kaw = 1;
	c_c.sat = 1;
	c_c.pid_I_prev = 0;
	c_c.u_prev = 0;
	m.refCurr = 0.5;
}

void initSpeedPID()
{
	s_c.Kp = 1;
	s_c.Ti = 100000;
	s_c.Td = 0;
	s_c.Kff = 0;
	s_c.Kaw = 0;
	s_c.sat = 3.3;
	s_c.pid_I_prev = 0;
	s_c.u_prev = 0;
	m.refSpeed = 1000;
}

void initPosPID()
{
	p_c.Kp = 10;
	p_c.Ti = 10000000000;
	p_c.Td = 0;
	p_c.Kff = 0;
	p_c.Kaw = 0;
	p_c.sat = 3000;
	p_c.pid_I_prev = 0;
	p_c.u_prev = 0;
	m.refPos = 1;
}

void resetPID()
{
	c_c.pid_I_prev = 0;
	c_c.u_prev = 0;

	s_c.pid_I_prev = 0;
	s_c.u_prev = 0;

	p_c.pid_I_prev = 0;
	p_c.u_prev = 0;

	__HAL_TIM_SET_COUNTER(&htim1,0);
}

void resetData()
{
    for(int i = 0;i<=8000-1;i++)
    {
        m.measurCurr[i] = 0;
        m.measurSpeed[i] = 0;
        m.measurPos[i] = 0;
    }
    for(int i = 0;i<=100-1;i++)
    {
    	mSpeed.ringbuffer[i] = 0;
    }
    mSpeed.enkoder_tmp = 0;
    mSpeed.enkoder_cnt = 0;
    mSpeed.enkoder_cnt_old = 0;
    mSpeed.idx = 0;
    mSpeed.tmp = 0;
    mSpeed.diff = 0;
}

int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}

void enkoderMeasure()
{

	// rzutowanie na int16 daje dodatnie i ujemne wartości
	mSpeed.enkoder_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);

	posCalc();
	speedCalc();
}

void posCalc()
{
	mSpeed.diff = (mSpeed.enkoder_cnt - mSpeed.enkoder_cnt_old);
	mSpeed.enkoder_tmp += mSpeed.diff;
	mSpeed.enkoder_cnt_old = mSpeed.enkoder_cnt;
	m.measurPos[m.idx] = (float)mSpeed.enkoder_tmp/(resolution*enkoder_cyclic_cnt);
}

void speedCalc()
{
	if(++mSpeed.idx >= 100) mSpeed.idx = 0;

	mSpeed.tmp = mSpeed.ringbuffer[mSpeed.idx];
	mSpeed.ringbuffer[mSpeed.idx] = mSpeed.enkoder_cnt;

	mSpeed.diff = (mSpeed.enkoder_cnt - mSpeed.tmp);
	m.measurSpeed[m.idx] = (60*(float)mSpeed.diff*enkoder_freq/100)/(resolution*enkoder_cyclic_cnt);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC2)
	{
		if(mode!=STOP_MODE)
		{
		m.time=Ts*m.idx;

		// pomiar prądu i odczyt wartości z enkodera
		m.measurCurr[m.idx] = -1*(0.00395522*m.dmaMeasurCurr-3.68421159-8.6);
		enkoderMeasure();
		if(mode == DEF_MODE)
		{
			if(m.time<0.200)
			{
				changeDir(LEFT_DIR);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,1500);
			}
			else if(m.time<0.400 && m.time>=0.200)
			{
				changeDir(RIGHT_DIR);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,1500);
			}
			else
			{
				changeDir(STOP_DIR);
				mode = STOP_MODE;
				m.endMeasurFlag = true;
			}
			if(++m.idx >= 8000) m.idx = 8000;
		}
		if(mode == CURR_MODE)
		{
			if(m.time>=m.timeLoadPattern[0]/1000&&m.time<m.timeLoadPattern[1]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[0]);
			if(m.time>=m.timeLoadPattern[1]/1000&&m.time<m.timeLoadPattern[2]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[1]);
			if(m.time>=m.timeLoadPattern[2]/1000&&m.time<m.timeLoadPattern[3]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[2]);
			if(m.time>=m.timeLoadPattern[3]/1000&&m.time<m.timeLoadPattern[4]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[3]);
			if(m.time>=m.timeLoadPattern[4]/1000&&m.time<m.simTime/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[4]);

			if(m.time>=m.timeValuePattern[0]/1000&&m.time<m.timeValuePattern[1]/1000) m.refCurr=m.refValuePattern[0];
			if(m.time>=m.timeValuePattern[1]/1000&&m.time<m.timeValuePattern[2]/1000) m.refCurr=m.refValuePattern[1];
			if(m.time>=m.timeValuePattern[2]/1000&&m.time<m.timeValuePattern[3]/1000) m.refCurr=m.refValuePattern[2];
			if(m.time>=m.timeValuePattern[3]/1000&&m.time<=m.timeValuePattern[4]/1000) m.refCurr=m.refValuePattern[3];
			if(m.time>=m.timeValuePattern[4]/1000&&m.time<m.simTime/1000) m.refCurr=m.refValuePattern[4];
			if(m.time>=m.simTime/1000)
			{
				changeDir(STOP_DIR);
				mode = STOP_MODE;
				m.endMeasurFlag = true;
			}

			regulator_PID_curr();
			if(++m.idx >= 8000) m.idx = 8000;
		}
		if(mode == SPEED_MODE)
		{
			if(m.time>=m.timeLoadPattern[0]/1000&&m.time<m.timeLoadPattern[1]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[0]);
			if(m.time>=m.timeLoadPattern[1]/1000&&m.time<m.timeLoadPattern[2]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[1]);
			if(m.time>=m.timeLoadPattern[2]/1000&&m.time<m.timeLoadPattern[3]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[2]);
			if(m.time>=m.timeLoadPattern[3]/1000&&m.time<m.timeLoadPattern[4]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[3]);
			if(m.time>=m.timeLoadPattern[4]/1000&&m.time<m.simTime/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[4]);

			if(m.time>=m.timeValuePattern[0]/1000&&m.time<m.timeValuePattern[1]/1000) m.refSpeed=m.refValuePattern[0];
			if(m.time>=m.timeValuePattern[1]/1000&&m.time<m.timeValuePattern[2]/1000) m.refSpeed=m.refValuePattern[1];
			if(m.time>=m.timeValuePattern[2]/1000&&m.time<m.timeValuePattern[3]/1000) m.refSpeed=m.refValuePattern[2];
			if(m.time>=m.timeValuePattern[3]/1000&&m.time<m.timeValuePattern[4]/1000) m.refSpeed=m.refValuePattern[3];
			if(m.time>=m.timeValuePattern[4]/1000&&m.time<m.simTime/1000) m.refSpeed=m.refValuePattern[4];
			if(m.time>=m.simTime/1000)
			{
				changeDir(STOP_DIR);
				mode = STOP_MODE;
				m.endMeasurFlag = true;
			}

			regulator_PID_speed();
			regulator_PID_curr();
			if(++m.idx >= 8000) m.idx = 8000;
		}
		if(mode == POS_MODE)
		{
			if(m.time>=m.timeLoadPattern[0]/1000&&m.time<m.timeLoadPattern[1]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[0]);
			if(m.time>=m.timeLoadPattern[1]/1000&&m.time<m.timeLoadPattern[2]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[1]);
			if(m.time>=m.timeLoadPattern[2]/1000&&m.time<m.timeLoadPattern[3]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[2]);
			if(m.time>=m.timeLoadPattern[3]/1000&&m.time<m.timeLoadPattern[4]/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[3]);
			if(m.time>=m.timeLoadPattern[4]/1000&&m.time<m.simTime/1000)
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoadPattern[4]);

			if(m.time>=m.timeValuePattern[0]/1000&&m.time<m.timeValuePattern[1]/1000) m.refPos=m.refValuePattern[0];
			if(m.time>=m.timeValuePattern[1]/1000&&m.time<m.timeValuePattern[2]/1000) m.refPos=m.refValuePattern[1];
			if(m.time>=m.timeValuePattern[2]/1000&&m.time<m.timeValuePattern[3]/1000) m.refPos=m.refValuePattern[2];
			if(m.time>=m.timeValuePattern[3]/1000&&m.time<m.timeValuePattern[4]/1000) m.refPos=m.refValuePattern[3];
			if(m.time>=m.timeValuePattern[4]/1000&&m.time<m.simTime/1000) m.refPos=m.refValuePattern[4];
			if(m.time>=m.simTime/1000)
			{
				changeDir(STOP_DIR);
				mode = STOP_MODE;
				m.endMeasurFlag = true;
			}

			regulator_PID_pos();
			regulator_PID_speed();
			regulator_PID_curr();
			if(++m.idx >= 8000) m.idx = 8000;
		}
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
			changeDir(STOP_DIR);
			m.time=0;

			m.idx = 0;
			m.moveInProgress = false;
		}
	}
}

void setRefValue(float *refValue)
{
	//if(time>=m.timeValuePattern[0]&&time<m.timeValuePattern[1]) *refValue=m.refValuePattern[0];
	//if(time>=m.timeValuePattern[1]&&time<m.timeValuePattern[2]) *refValue=m.refValuePattern[1];
	//if(time>=m.timeValuePattern[2]&&time<m.timeValuePattern[3]) *refValue=m.refValuePattern[2];
	//if(time>=m.timeValuePattern[3]&&time<=m.timeValuePattern[4]) *refValue=m.refValuePattern[3];
	//if(time>=m.timeValuePattern[4]) *refValue=m.refValuePattern[4];
}

void reciveData()
{
	/*budowa ramki danych: mode, m.valueLoad, m.simTime,c_c.Kp, c_c.Ti, c_c.Td, c_c.Kff, c_c.Kaw, m.refCurr
	 * s_c.Kp, s_c.Ti, s_c.Td, s_c.Kff, s_c.Kaw, m.refSpeed
	 * p_c.Kp, p_c.Ti, p_c.Td, p_c.Kff, p_c.Kaw, m.refPos
	 * */
	reciveMotorParameters();
	reciveCurrPIDParameters();
	reciveSpeedPIDParameters();
	recivePosPIDParameters();

	float tmpDecoding = c_c.Ti - 0;
	if(tmpDecoding<0.00001) c_c.Ti = 100000000;
	tmpDecoding = s_c.Ti - 0;
	if(tmpDecoding<0.00001) s_c.Ti = 100000000;
	tmpDecoding = p_c.Ti - 0;
	if(tmpDecoding<0.00001) p_c.Ti = 100000000;

}

void reciveMotorParameters()
{
	uint8_t *ptr;
	int parameterNo;

	if(m.tmpData[0] == 0) mode = DEF_MODE;
	else if(m.tmpData[0] == 1) mode = CURR_MODE;
	else if(m.tmpData[0] == 2) mode = SPEED_MODE;
	else if(m.tmpData[0] == 3) mode = POS_MODE;
	else if(m.tmpData[0] == 4) mode = STOP_MODE;

	reciveLoadPattern();
	reciveRefPattern();

	parameterNo = 15;
	ptr = (uint8_t *)&m.simTime;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];
}

void reciveRefPattern()
{
	uint8_t *ptr;
	int parameterNo;

	// odbiór wartości paternu dla sygnału referencyjnego
	parameterNo = 5;
	ptr = (uint8_t *)&m.refValuePattern[0];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 6;
	ptr = (uint8_t *)&m.refValuePattern[1];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 7;
	ptr = (uint8_t *)&m.refValuePattern[2];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 8;
	ptr = (uint8_t *)&m.refValuePattern[3];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 9;
	ptr = (uint8_t *)&m.refValuePattern[4];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	// odbiór czasów paternu dla sygnału referencyjnego
	parameterNo = 10;
	ptr = (uint8_t *)&m.timeValuePattern[0];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 11;
	ptr = (uint8_t *)&m.timeValuePattern[1];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 12;
	ptr = (uint8_t *)&m.timeValuePattern[2];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 13;
	ptr = (uint8_t *)&m.timeValuePattern[3];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 14;
	ptr = (uint8_t *)&m.timeValuePattern[4];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];
}

void reciveLoadPattern()
{
	uint8_t *ptr;
	int parameterNo;

	// odbiór wartości paternu dla obciążenia
	m.valueLoadPattern[0] = m.tmpData[1]*4098/100;
	m.valueLoadPattern[1] = m.tmpData[2]*4098/100;
	m.valueLoadPattern[2] = m.tmpData[3]*4098/100;
	m.valueLoadPattern[3] = m.tmpData[4]*4098/100;
	m.valueLoadPattern[4] = m.tmpData[5]*4098/100;
	// odbiór czasów paternu dla obciążenia
	parameterNo = 0;
	ptr = (uint8_t *)&m.timeLoadPattern[0];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 1;
	ptr = (uint8_t *)&m.timeLoadPattern[1];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 2;
	ptr = (uint8_t *)&m.timeLoadPattern[2];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 3;
	ptr = (uint8_t *)&m.timeLoadPattern[3];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 4;
	ptr = (uint8_t *)&m.timeLoadPattern[4];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];
}

void reciveCurrPIDParameters()
{
	uint8_t *ptr;
	int parameterNo;

	parameterNo = 16;
	ptr = (uint8_t *)&c_c.Kp;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 17;
	ptr = (uint8_t *)&c_c.Ti;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 18;
	ptr = (uint8_t *)&c_c.Td;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 19;
	ptr = (uint8_t *)&c_c.Kff;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 20;
	ptr = (uint8_t *)&c_c.Kaw;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];
}

void reciveSpeedPIDParameters()
{
	uint8_t *ptr;
	int parameterNo;

	parameterNo = 21;
	ptr = (uint8_t *)&s_c.Kp;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 22;
	ptr = (uint8_t *)&s_c.Ti;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 23;
	ptr = (uint8_t *)&s_c.Td;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 24;
	ptr = (uint8_t *)&s_c.Kff;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 25;
	ptr = (uint8_t *)&s_c.Kaw;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];
}

void recivePosPIDParameters()
{
	uint8_t *ptr;
	int parameterNo;

	parameterNo = 26;
	ptr = (uint8_t *)&p_c.Kp;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 27;
	ptr = (uint8_t *)&p_c.Ti;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 28;
	ptr = (uint8_t *)&p_c.Td;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 29;
	ptr = (uint8_t *)&p_c.Kff;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];

	parameterNo = 30;
	ptr = (uint8_t *)&p_c.Kaw;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[6+parameterNo*sizeof(float)+i];
}

void transmitData()
{
	  if(m.endMeasurFlag == true && __HAL_TIM_GET_COMPARE(&htim8,TIM_CHANNEL_1) == 0)
	  {
		  uint8_t tmp[2+4*sizeof(float)];
		  uint8_t *ptr;
		  float time = 0;
		  for(int i=0;i<=8000;i++)
		  {
			  time = i*Ts;
			  // AA 55 ILE B1 B2 ... Bn

			  tmp[0] = 0xAA;
			  tmp[1] = 0x55;

			  ptr = (uint8_t *)&time;
			  for(uint8_t j = 0; j < sizeof(float); j++)		tmp[2+0*sizeof(float) + j] = ptr[j];
			  ptr = (uint8_t *)&m.measurCurr[i];
			  for(uint8_t j = 0; j < sizeof(float); j++)		tmp[2+1*sizeof(float) + j] = ptr[j];
			  ptr = (uint8_t *)&m.measurSpeed[i];
			  for(uint8_t j = 0; j < sizeof(float); j++)		tmp[2+2*sizeof(float) + j] = ptr[j];
			  ptr = (uint8_t *)&m.measurPos[i];
			  for(uint8_t j = 0; j < sizeof(float); j++)		tmp[2+3*sizeof(float) + j] = ptr[j];

			  HAL_UART_Transmit(&huart2, tmp, 18, HAL_MAX_DELAY);
		  }

		  m.endMeasurFlag = false;
		  resetPID();
		  resetData();
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
	if(m.moveInProgress == false) tmp=HAL_GetTick();

	m.moveInProgress = true;
	if(HAL_GetTick() - tmp<200)
	{
		changeDir(LEFT_DIR);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, 3000);
	}
	else if((HAL_GetTick()-tmp>=200) && (HAL_GetTick()-tmp<400))
	{
		changeDir(RIGHT_DIR);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, 3000);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		changeDir(STOP_DIR);
		HAL_ADC_Stop_DMA(&hadc2);
		m.idx = 0;
		m.endMeasurFlag = true;
		m.moveInProgress = false;
		mode = STOP_MODE;
	}
}

void controlMotorMove()
{
	static uint32_t tmp = 0;
	if(m.moveInProgress == false) tmp=HAL_GetTick();

	m.moveInProgress = true;
	if(HAL_GetTick() - tmp<400)
	{

	}
	else
	{
		HAL_ADC_Stop_DMA(&hadc2);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		changeDir(STOP_DIR);
		m.idx = 0;
		m.endMeasurFlag = true;
		m.moveInProgress = false;
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
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoad);
		HAL_ADC_Start_DMA(&hadc2, &m.dmaMeasurCurr, 1);
		controlMotorMove();
		break;
	case SPEED_MODE:
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoad);
		HAL_ADC_Start_DMA(&hadc2, &m.dmaMeasurCurr, 1);
		controlMotorMove();
		break;
	case POS_MODE:
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, m.valueLoad);
		HAL_ADC_Start_DMA(&hadc2, &m.dmaMeasurCurr, 1);
		controlMotorMove();
		break;
	case FREE_MODE:
		HAL_ADC_Start_DMA(&hadc2, &m.dmaMeasurCurr, 1);
		break;
	case STOP_MODE:
		__HAL_TIM_SET_COUNTER(&htim1,0);
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
		changeDir(RIGHT_DIR);
		new_pwm = (uint16_t)(6000*c_c.y_curr);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,new_pwm);
	}
	else
	{
		changeDir(LEFT_DIR);
		new_pwm = (uint16_t)(6000*(-1*c_c.y_curr));
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


void regulator_PID_pos()
{
	float e = m.refPos - m.measurPos[m.idx];

	float pid_P = p_c.Kp*e;
	float pid_I = p_c.pid_I_prev + Ts*(e*p_c.Kp-p_c.Kaw*(p_c.y-p_c.y_pos));
	float pid_D = (e*p_c.Kp-p_c.u_prev)/Ts;

	p_c.pid_I_prev = pid_I;
	p_c.u_prev = pid_P;

	p_c.y = pid_P + pid_I/p_c.Ti + pid_D*p_c.Td;

	if(p_c.y > p_c.sat) p_c.y_pos = p_c.sat;
	else if(p_c.y < -p_c.sat) p_c.y_pos = -p_c.sat;
	else p_c.y_pos = p_c.y;

	m.refSpeed = p_c.y_pos;
}
