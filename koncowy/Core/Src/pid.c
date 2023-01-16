/*
 * pid.c
 *
 *  Created on: Sep 21, 2022
 *      Author: hubli
 */

#include "pid.h"

#define PI 3.1415926535897932384626433832795

Direction dir;
motorMode mode;
motor m;
currentControler c_c;
speedControler s_c;
positionControler p_c;
measureSpeed mSpeed;
observer lto;



void initPeripherals()
{
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim1,0);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, m.tmpData, FRAME_RECIVE_WIDITH);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);// 610 TO WARTOŚĆ 0.140 V
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
	initObserver();
}

void initObserver()
{
	lto.Jm = 0.00005732/*0.0001743*/; // [kg/m^2]
	lto.kt = 0.055;
	lto.Mob = 0;
	lto.wyFCN2 = 0;
	lto.prev_u = 0;
	lto.Bs = 0.00006369/*0.00581*/;

	lto.prevFCN = 0;
	lto.speedDif = 0;
	lto.K = 100;
	lto.tm = 0.9;

	lto.Ra = 2.857;
	lto.l1 = 10000;
	lto.l2 = -5;
}

void initControler()
{
	mode = STOP_MODE;
	m.endMeasurFlag = false;
	m.moveInProgress = false;
	m.tmp = 0; // zmienna tymczasowa ogólnego użytku
	m.sampleDiv = 8;
	m.sampleCounter = 0;
}

void initCurrPID()
{
	c_c.Kp = 1.2;
	c_c.Ti = 0.01;
	c_c.Td = 0;
	c_c.KffLoad = 0;
	c_c.Kaw = 1;
	c_c.sat = 1;
	c_c.pid_I_prev = 0;
	c_c.u_prev = 0;
	m.refCurr = 0.5;
}

void initSpeedPID()
{
	s_c.Kp = 0.008;
	s_c.Ti = 1;
	s_c.Td = 0;
	s_c.Kaw = 1;
	s_c.sat = 3.3;
	s_c.pid_I_prev = 0;
	s_c.u_prev = 0;
	m.refSpeed = 1000;
}

void initPosPID()
{
	p_c.Kp = 900;
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

	lto.prev_u = 0;
	lto.prevFCN = 0;
	lto.speedDif = 0;
	lto.Mob = 0;
	lto.wyFCN2 = 0;

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

    m.sampleCounter = 0;
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
	if(mSpeed.diff>1000)
	{
		mSpeed.diff = (-32768+mSpeed.enkoder_cnt_old) - (32768-mSpeed.enkoder_cnt);
	}
	else if(mSpeed.diff<-1000)
	{
		mSpeed.diff = (32768-mSpeed.enkoder_cnt_old) + (mSpeed.enkoder_cnt + 32768);
	}
	mSpeed.enkoder_tmp += mSpeed.diff;
	mSpeed.enkoder_cnt_old = mSpeed.enkoder_cnt;
	m.actualPos = (float)mSpeed.enkoder_tmp/(resolution*enkoder_cyclic_cnt);
}

void speedCalc()
{
	if(++mSpeed.idx >= 100) mSpeed.idx = 0;

	mSpeed.tmp = mSpeed.ringbuffer[mSpeed.idx];
	mSpeed.ringbuffer[mSpeed.idx] = mSpeed.enkoder_cnt;

	mSpeed.diff = (mSpeed.enkoder_cnt - mSpeed.tmp);
	if(mSpeed.diff>1000)
	{
		mSpeed.diff = (-32768+mSpeed.tmp) - (32768-mSpeed.enkoder_cnt);
	}
	else if(mSpeed.diff<-1000)
	{
		mSpeed.diff = (32768-mSpeed.tmp) + (mSpeed.enkoder_cnt + 32768);
	}
	m.actualSpeed = (60*(float)mSpeed.diff*enkoder_freq/100)/(resolution*enkoder_cyclic_cnt);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

	bool dir = (htim8.Instance->CR1==33);
	if(((hadc->Instance == ADC2)&&dir)&& m.moveInProgress)
	{
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
		m.actualCurr = -1*(0.01403*m.dmaMeasurCurr-23.81);
		if(mode!=STOP_MODE)
		{
			m.time=Ts*m.idx; // wyznaczanie czasu próbki

			// konwersja pomiaru pradu
			m.actualCurr = -1*(0.01403*m.dmaMeasurCurr-23.81); //-1*(0.00395522*m.dmaMeasurCurr-6.71596356);

			enkoderMeasure();
			if(mode == DEF_MODE)
			{
				defaultMotorMove();
				m.sampleDiv=8;
			}
			if(mode == CURR_MODE)
			{
				currControlMotorMove();
				m.sampleDiv = 1;
			}
			if(mode == SPEED_MODE)
			{
				speedControlMotorMove();
				m.sampleDiv = 8;
			}
			if(mode == POS_MODE)
			{
				posControlMotorMove();
				m.sampleDiv = 8;
			}

			if(m.idx%m.sampleDiv==0)
			{
				m.sampleCounter++;
				m.measurCurr[m.idx/m.sampleDiv] = m.actualCurr;
				m.measurSpeed[m.idx/m.sampleDiv] = m.actualSpeed;
				m.measurPos[m.idx/m.sampleDiv] = m.actualPos/*(lto.wyFCN2*60)/(2*PI)*/;
			}
		}
		else
		{
			stopMotor();
		}
	}
}

void stopMotor()
{
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	changeDir(STOP_DIR);
	m.time = 0;
	m.idx = 0;
	m.moveInProgress = false;
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
	if(m.tmpData[0] == 16 && m.tmpData[1] == 18)
	{
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
		m.tmp++;

		if(m.tmp==2)
		{
			m.tmp=0;
			m.moveInProgress = true;
		}
	}
	else
	{
		m.tmp=0;
	}

}

void reciveMotorParameters()
{
	uint8_t *ptr;
	int parameterNo;

	if(m.tmpData[2] == 0) mode = DEF_MODE;
	else if(m.tmpData[2] == 1) mode = CURR_MODE;
	else if(m.tmpData[2] == 2) mode = SPEED_MODE;
	else if(m.tmpData[2] == 3) mode = POS_MODE;
	else if(m.tmpData[2] == 4) mode = STOP_MODE;

	reciveLoadPattern();
	reciveRefPattern();

	parameterNo = 20;
	ptr = (uint8_t *)&m.simTime;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];
}

void reciveRefPattern()
{
	uint8_t *ptr;
	int parameterNo;

	// odbiór wartości paternu dla sygnału referencyjnego
	parameterNo = 10;
	ptr = (uint8_t *)&m.refValuePattern[0];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 11;
	ptr = (uint8_t *)&m.refValuePattern[1];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 12;
	ptr = (uint8_t *)&m.refValuePattern[2];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 13;
	ptr = (uint8_t *)&m.refValuePattern[3];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 14;
	ptr = (uint8_t *)&m.refValuePattern[4];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	// odbiór czasów paternu dla sygnału referencyjnego
	parameterNo = 15;
	ptr = (uint8_t *)&m.timeValuePattern[0];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 16;
	ptr = (uint8_t *)&m.timeValuePattern[1];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 17;
	ptr = (uint8_t *)&m.timeValuePattern[2];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 18;
	ptr = (uint8_t *)&m.timeValuePattern[3];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 19;
	ptr = (uint8_t *)&m.timeValuePattern[4];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];
}

void reciveLoadPattern()
{
	uint8_t *ptr;
	int parameterNo;

	// odbiór wartości paternu dla obciążenia
	parameterNo = 0;
	ptr = (uint8_t *)&m.valueLoadPattern[0];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];
	//m.valueLoadPattern[0]=(m.valueLoadPattern[0]*(41.5-38)/100)+38;
	m.valueLoadPattern[0]=m.valueLoadPattern[0]*4096/100;

	parameterNo = 1;
	ptr = (uint8_t *)&m.valueLoadPattern[1];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];
	//m.valueLoadPattern[1]=(m.valueLoadPattern[1]*(41.5-38)/100)+38;
	m.valueLoadPattern[1]=m.valueLoadPattern[1]*4096/100;

	parameterNo = 2;
	ptr = (uint8_t *)&m.valueLoadPattern[2];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];
	//m.valueLoadPattern[2]=(m.valueLoadPattern[2]*(41.5-38)/100)+38;
	m.valueLoadPattern[2]=m.valueLoadPattern[2]*4096/100;

	parameterNo = 3;
	ptr = (uint8_t *)&m.valueLoadPattern[3];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];
	//m.valueLoadPattern[3]=(m.valueLoadPattern[3]*(41.5-38)/100)+38;
	m.valueLoadPattern[3]=m.valueLoadPattern[3]*4096/100;

	parameterNo = 4;
	ptr = (uint8_t *)&m.valueLoadPattern[4];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];
	//m.valueLoadPattern[4]=(m.valueLoadPattern[4]*(41.5-38)/100)+38;
	m.valueLoadPattern[4]=m.valueLoadPattern[4]*4096/100;

	// odbiór czasów paternu dla obciążenia
	parameterNo = 5;
	ptr = (uint8_t *)&m.timeLoadPattern[0];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 6;
	ptr = (uint8_t *)&m.timeLoadPattern[1];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 7;
	ptr = (uint8_t *)&m.timeLoadPattern[2];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 8;
	ptr = (uint8_t *)&m.timeLoadPattern[3];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 9;
	ptr = (uint8_t *)&m.timeLoadPattern[4];
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];
}

void reciveCurrPIDParameters()
{
	uint8_t *ptr;
	int parameterNo;

	parameterNo = 21;
	ptr = (uint8_t *)&c_c.Kp;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 22;
	ptr = (uint8_t *)&c_c.Ti;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 23;
	ptr = (uint8_t *)&c_c.Td;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 24;
	ptr = (uint8_t *)&c_c.Kaw;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];
}

void reciveSpeedPIDParameters()
{
	uint8_t *ptr;
	int parameterNo;

	parameterNo = 25;
	ptr = (uint8_t *)&s_c.Kp;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 26;
	ptr = (uint8_t *)&s_c.Ti;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 27;
	ptr = (uint8_t *)&s_c.Td;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 28;
	ptr = (uint8_t *)&s_c.Kaw;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];
}

void recivePosPIDParameters()
{
	uint8_t *ptr;
	int parameterNo;

	parameterNo = 29;
	ptr = (uint8_t *)&p_c.Kp;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 30;
	ptr = (uint8_t *)&p_c.Ti;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 31;
	ptr = (uint8_t *)&p_c.Td;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 32;
	ptr = (uint8_t *)&p_c.Kff;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 33;
	ptr = (uint8_t *)&c_c.KffLoad;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];

	parameterNo = 34;
	ptr = (uint8_t *)&p_c.Kaw;
	for(int i = 0;i < sizeof(float);i++) ptr[i] = m.tmpData[3+parameterNo*sizeof(float)+i];
}

void transmitData()
{
	  if(m.endMeasurFlag == true && __HAL_TIM_GET_COMPARE(&htim8,TIM_CHANNEL_1) == 0)
	  {
		  uint8_t tmp[4+4*sizeof(float)];
		  uint8_t *ptr;
		  float time = 0;
		  for(int i=0;i<=m.sampleCounter;i++)
		  {
			  time = i*Ts*m.sampleDiv;
			  // AA 55 ILE B1 B2 ... Bn

			  tmp[0] = 0xAA;
			  tmp[1] = 0x55;
			  tmp[2] = (uint8_t)(m.sampleCounter>>8);
			  tmp[3] = (uint8_t)m.sampleCounter;

			  ptr = (uint8_t *)&time;
			  for(uint8_t j = 0; j < sizeof(float); j++)		tmp[4+0*sizeof(float) + j] = ptr[j];
			  ptr = (uint8_t *)&m.measurCurr[i];
			  for(uint8_t j = 0; j < sizeof(float); j++)		tmp[4+1*sizeof(float) + j] = ptr[j];
			  ptr = (uint8_t *)&m.measurSpeed[i];
			  for(uint8_t j = 0; j < sizeof(float); j++)		tmp[4+2*sizeof(float) + j] = ptr[j];
			  ptr = (uint8_t *)&m.measurPos[i];
			  for(uint8_t j = 0; j < sizeof(float); j++)		tmp[4+3*sizeof(float) + j] = ptr[j];

			  HAL_UART_Transmit(&huart2, tmp, 20, HAL_MAX_DELAY);
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
			if(m.time<0.200)
			{
				changeDir(LEFT_DIR);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,1500);
			}
/*			else if(m.time<0.200 && m.time>=0.100)
			{
				changeDir(LEFT_DIR);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
			}
			else if(m.time<0.300 && m.time>=0.200)
			{
				changeDir(LEFT_DIR);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,1500);
			}*/
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

void currControlMotorMove()
{
	setMotorLoad();

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
	m.idx++;
	//if(++m.idx >= 8000) m.idx = 8000;
}
void speedControlMotorMove()
{
	setMotorLoad();

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
	//if(m.idx%10 == 0)
	loadTorqueObserver3();
	regulator_PID_speed();
	regulator_PID_curr();
	m.idx++;
	//if(++m.idx >= 8000) m.idx = 8000;
}
void posControlMotorMove()
{
	setMotorLoad();

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
	loadTorqueObserver();
	regulator_PID_pos();
	regulator_PID_speed();
	regulator_PID_curr();
	m.idx++;
	//if(++m.idx >= 8000) m.idx = 8000;
}

void setMotorLoad()
{
	if(m.time>=m.timeLoadPattern[0]/1000&&m.time<m.timeLoadPattern[1]/1000)
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)m.valueLoadPattern[0]);
	if(m.time>=m.timeLoadPattern[1]/1000&&m.time<m.timeLoadPattern[2]/1000)
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)m.valueLoadPattern[1]);
	if(m.time>=m.timeLoadPattern[2]/1000&&m.time<m.timeLoadPattern[3]/1000)
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)m.valueLoadPattern[2]);
	if(m.time>=m.timeLoadPattern[3]/1000&&m.time<m.timeLoadPattern[4]/1000)
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)m.valueLoadPattern[3]);
	if(m.time>=m.timeLoadPattern[4]/1000&&m.time<m.simTime/1000)
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)m.valueLoadPattern[4]);
}

void regulator_PID_curr()
{
	static uint16_t new_pwm = 0;
	float e = m.refCurr - m.actualCurr;

	float pid_P = c_c.Kp*e;
	float pid_I = c_c.pid_I_prev + Ts*(e*c_c.Kp-c_c.Kaw*(c_c.y-c_c.y_curr));
	float pid_D = (e*c_c.Kp-c_c.u_prev)/Ts;

	c_c.pid_I_prev = pid_I;
	c_c.u_prev = pid_P;

	c_c.y = pid_P + pid_I/c_c.Ti + pid_D*c_c.Td;

	c_c.y += c_c.KffLoad*lto.Mob; // moment obciążenia

	if(c_c.y > c_c.sat) c_c.y_curr = c_c.sat;
	else if(c_c.y < -c_c.sat) c_c.y_curr = -c_c.sat;
	else c_c.y_curr = c_c.y;

	if(c_c.y_curr>=0)
	{
		changeDir(RIGHT_DIR);
		new_pwm = (uint16_t)(6000*c_c.y_curr);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,new_pwm);
	}
	else if(c_c.y_curr<0)
	{
		changeDir(LEFT_DIR);
		new_pwm = (uint16_t)(6000*(-1*c_c.y_curr));
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, new_pwm);
	}
}


void regulator_PID_speed()
{
	float e = m.refSpeed - m.actualSpeed;

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
	float e = m.refPos - m.actualPos;

	float pid_P = p_c.Kp*e;
	float pid_I = p_c.pid_I_prev + Ts*(e*p_c.Kp-p_c.Kaw*(p_c.y-p_c.y_pos));
	float pid_D = (e*p_c.Kp-p_c.u_prev)/Ts;

	p_c.pid_I_prev = pid_I;
	p_c.u_prev = pid_P;

	p_c.y = pid_P + pid_I/p_c.Ti + pid_D*p_c.Td;

	p_c.y += p_c.Kff*m.refPos;

	if(p_c.y > p_c.sat) p_c.y_pos = p_c.sat;
	else if(p_c.y < -p_c.sat) p_c.y_pos = -p_c.sat;
	else p_c.y_pos = p_c.y;

	m.refSpeed = p_c.y_pos;
}

void loadTorqueObserver()
{
	float speed = (m.actualSpeed*2*3.14)/60;
	float uSpeed = lto.Jm*speed;
	float speedElement = (uSpeed - lto.prev_u)/(Ts);
	float currentElement = lto.kt*m.actualCurr;
	float fricElement = (lto.Bs/*-0.00425*/)*speed;

	lto.prev_u = uSpeed;
	lto.Mob = currentElement - speedElement - fricElement-0.066;
	lto.Mob = lto.prevFCN + 0.1*(lto.Mob - lto.prevFCN);
	lto.prevFCN = lto.Mob;
}

void loadTorqueObserver2()
{
	float speed = (m.actualSpeed*2*3.14)/60;
	float current = lto.kt*m.actualCurr;
	float vel = speed*lto.l1;
	float obser_speed = current + vel - lto.Mob/lto.Jm - lto.wyFCN2*lto.l1;
	lto.wyFCN2 = lto.wyFCN2+ Ts*obser_speed;
	float diff_speed = speed-lto.wyFCN2;
	lto.Mob = lto.l2*(lto.Mob+ Ts*diff_speed);
}

void loadTorqueObserver3()
{
	float speed = (m.actualSpeed*2*PI)/60;
	float current = lto.kt*m.actualCurr - 0.0715;
	float diff_speed = speed-lto.wyFCN2;
	lto.Mob = (lto.Mob+ Ts*lto.l2*diff_speed);
	//lto.Mob = lto.prevFCN + 0.1*(lto.Mob - lto.prevFCN);
	//lto.prevFCN = lto.Mob;
	float obser_speed = current + diff_speed*lto.l1*lto.Jm - lto.Mob;
	lto.wyFCN2 = lto.wyFCN2 + (Ts/lto.tm)*(obser_speed/lto.Bs - lto.wyFCN2);
}
