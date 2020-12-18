/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "usart.h"
#include "APPTooL.h"
#include "tim.h"
#include "BspConfig.h"
#include "application.h"
#include "adc.h"
#include "gpio.h"
#include "WTN6.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
QueueHandle_t xQueuel_sportmes;//消息队列  ，在tim.h中声明
uint8_t pstate = 0;
uint8_t iscolsesystem = 0;//
//线程同步
extern SemaphoreHandle_t xSemaphore_WTN6_TFT; //串口，语音播放互斥量



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId DataDetectionHandle;
osThreadId Uart_TFTHandle;
osThreadId DataInteractionHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void PlayCallback(uint8_t val);//语音播放回调函数
Customerinfo SportInfo_Get = { 0 };   //获取到的运动信息
void SendMessageToTFT(uint16_t address);
void SendToBle(UART_HandleTypeDef *huart);					//上传运动信息
uint16_t Cal_In_Voltage(uint16_t v);//计算外部电源电压
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void DataDetection_CallBack(void const * argument);
void Uart_TFT_CallBack(void const * argument);
void DataInteraction_CallBack(void const * argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */


/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
 
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);


}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	/* definition and creation of DataDetection */
	taskENTER_CRITICAL();//进入临界区
	xQueuel_sportmes = xQueueCreate(5, sizeof(struct SportInfo*));//创建消息队列
	if (xQueuel_sportmes == 0)
	{
		Error_Handler_t(ERROR_XQUEUE_CREAT);
	}
	osThreadDef(DataDetection, DataDetection_CallBack, 5, 0, 128);
	DataDetectionHandle = osThreadCreate(osThread(DataDetection), NULL);

	/* definition and creation of Uart_TFT */
	osThreadDef(Uart_TFT, Uart_TFT_CallBack, 1, 0, 128);
	Uart_TFTHandle = osThreadCreate(osThread(Uart_TFT), NULL);

	/* definition and creation of DataInteraction */
	osThreadDef(DataInteraction, DataInteraction_CallBack,6, 0, 128);
	DataInteractionHandle = osThreadCreate(osThread(DataInteraction), NULL);
	
  /* Infinite loop */
	Uart_printf(&huart2, "Start sub stask\r\n");
	vTaskDelete(defaultTaskHandle); //删除任务
	taskEXIT_CRITICAL();//推出临界区
	
    osDelay(100);
  
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_DataDetection_CallBack */
/**
* @brief Function implementing the DataDetection thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataDetection_CallBack */
void DataDetection_CallBack(void const * argument)
{
  /* USER CODE BEGIN DataDetection_CallBack */
  /* Infinite loop */
	Customerinfo *ptMsg;
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000); /* 设置最大等待时间为200ms */
	
  for(;;)
  {
	 
	  xResult = xQueueReceive(
		  xQueuel_sportmes,                   /* 消息队列句柄 */
		  (void *)&ptMsg,             /* 这里获取的是结构体的地址 */
		  (TickType_t)xMaxBlockTime);/* 设置阻塞时间 */

	  if (xResult == pdPASS)
	  {
		 // Uart_printf(&huart2, "xQueueData:Freq=%d, Tim=%d ,Sportcount=%d, Cal=%d  playstate=%d\r\n", ptMsg->freq, ptMsg->tim, ptMsg->count, ptMsg->hot, ptMsg->playstate);
		  SportInfo_Get.count = ptMsg->count;
		  SportInfo_Get.freq = ptMsg->freq;
		  SportInfo_Get.hot = ptMsg->hot;
		  SportInfo_Get.tim = ptMsg->tim;
		  SportInfo_Get.playstate = ptMsg->playstate;
		  xSemaphoreTake(xSemaphore_WTN6_TFT, portMAX_DELAY);
		  {
			  //Uart_printf_Debug(&huart2, "SportInfo_Get.count=%d  iscolsesystem=%d\r\n", SportInfo_Get.count, iscolsesystem);
			  SendMessageToTFT(TFT_VARIABLE_START);
			  SingleTrig(PlayCallback, SportInfo_Get.playstate, 0, 0, 1);
			 
		  }
		  xSemaphoreGive(xSemaphore_WTN6_TFT);
		  
	  }
	 // Uartx_printf(&huart1, "thread2\r\n");
	
    osDelay(200);
  }
  /* USER CODE END DataDetection_CallBack */
}

/* USER CODE BEGIN Header_Uart_TFT_CallBack */
/**
* @brief Function implementing the Uart_TFT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Uart_TFT_CallBack */
void DataInteraction_CallBack(void const * argument)//
{
  /* USER CODE BEGIN Uart_TFT_CallBack */
  /* Infinite loop */
	static uint16_t tick = 0, loop = 0, voltage_tim = 0;
	uint8_t LowVoltage_num = 0;
  for(;;)
  {
	  //Uart_printf(UART_CONNECTION, "System closed!\r\n");
	 // Uart_printf(&huart2, "SportCount==%d\r\n",SportInfo_Get.count);
	  SingleTrig(SensorCallBack, HAL_GPIO_ReadPin(Sensor_Mag_GPIO_Port, Sensor_Mag_Pin), 0, 1, 0);
	  if (iscolsesystem == 1)
	  {
		 // Uart_printf(&huart2, "Stop...%d\r\n", tick);
		  if (tick++ > TIMER_CLOSESYSTEM / 50)    //播放完毕5s后关闭系统
		  {
			  tick = 0;
			  SYSIO = 1;
			
			  //Uart_printf_Debug(UART_CONNECTION, "System closed!\r\n");
		  }
		  
	  }
	  else
		  tick = 0;
	  if (SportInfo_Get.count == 0)     //关闭系统:当开机后没运动10s关闭系统
	  {
		  if (loop++ > 2 * TIMER_CLOSESYSTEM / 50)
		  {
			  loop = 0;
			  SYSIO = 1;
			 
			  //Uart_printf(&huart2, "System closed---loop!\r\n");
		  }
	  }
	  else
		  loop = 0;
	  if (voltage_tim++>VOLTAGE_T/50)
	  {
		  voltage_tim = 0;
		  uint16_t vt = ADC_Conversion(&hadc1, 10);
		  vt = Cal_In_Voltage(vt);
		  
		  if (vt < POWER_VOLTAGE_LOW)
		  {
			  Turen_Pic(2);
			  if (LowVoltage_num++>5)
			  {
				  LowVoltage_num = 0;
				  SYSIO = 1;                     //电压低关闭系统
			  }
		  }
			 
		  
	  }
	  
	  
	  //Uart_printf(&huart1, "Uart2\r\n");
    osDelay(50);
  }
  /* USER CODE END Uart_TFT_CallBack */
}

/* USER CODE BEGIN Header_DataInteraction_CallBack */
/**
* @brief Function implementing the DataInteraction thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataInteraction_CallBack */
void Uart_TFT_CallBack(void const * argument)
{
  /* USER CODE BEGIN DataInteraction_CallBack */
  /* Infinite loop */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000;
	xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
	  
		  xSemaphoreTake(xSemaphore_WTN6_TFT, portMAX_DELAY);
		  {
			  //发送蓝牙数据
			  SendToBle(UART_BLE);
		  }
		  xSemaphoreGive(xSemaphore_WTN6_TFT);
		 // Uartx_printf(UART_CONNECTION, "System closed---loop!\r\n");
	  
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END DataInteraction_CallBack */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationIdleHook(void)
{
	static uint16_t x = 0;
}
void PlayCallback(uint8_t val)//语音播放回调函数
{
	uint8_t playarray[PLAYARRAYLENGTH] = { 0 };//播放数据数组
	uint8_t playdatalen = 0;
	TFT_Beep(2);// bi-bi 声音
	//Uart_printf(&huart1, "playstate=%d   Sound is beginning...!\r\n",SportInfo_Get.playstate);
	playdatalen = GetPlayData(SportInfo_Get.count, SportInfo_Get.tim, (double)SportInfo_Get.hot, playarray);
	pstate = WTN6_PlayArray(playdatalen, playarray,1000);
	//Uart_printf(&huart2, "value==%d\r\n", pstate);
	iscolsesystem = 1;
	//Uart_printf(&huart2, "iscolsesystem is %d\r\n", iscolsesystem);
}
uint16_t Cal_In_Voltage(uint16_t v)
{
	float k = (R_DOWN + R_UP) / R_DOWN;
	return	(uint16_t)(k*((v * 825) >> 10));
}
void  QF_CRC(uint8_t *dat, uint8_t len)
{
	uint8_t crctemp_lo = 0, crctemp_hi, i = 0;
	//SendByte(0xaa);
	for (i = 0; i < len - 2; i++)
	{
		if (i == 0)
		{
			crctemp_hi = dat[i] ^ 0xe1;

			crctemp_lo = dat[i] ^ 0xe2;
		}

		else
		{
			crctemp_hi = crctemp_hi^dat[i];
			crctemp_lo = crctemp_lo^dat[i];
		}

	}
	//SendByte(crctemp_hi);
	dat[18] = crctemp_hi;
	dat[19] = crctemp_lo;
}
//向TFT屏发送数据
void SendMessageToTFT(uint16_t address)
{
	uint16_t TFT_SendArray[8] = { 0 };
	if (SportInfo_Get.freq > 3)
		SportInfo_Get.freq = 3;
	///********************
	TFT_SendArray[0] = SportInfo_Get.tim / 3600;			//时
	TFT_SendArray[1] = SportInfo_Get.tim % 3600 / 60;		//分
	TFT_SendArray[2] = SportInfo_Get.tim % 60;				//秒
	TFT_SendArray[3] = SportInfo_Get.count;					//次数
	TFT_SendArray[4] = SportInfo_Get.count*CAL_K;					//热量
	TFT_SendArray[5] = SportInfo_Get.freq;					//速度
	TFT_SendArray[6] = 0;									//NULL
	TFT_SendArray[7] = ADC_GetValue(&hadc1, 10);			//电池电量
	write_multiple_variable_store_82(address, 8, TFT_SendArray);
	write_register_80_1byte(TFT_BUTTON, 1);
}
void SendToBle(UART_HandleTypeDef *huart)// 数据整合
{

	uint8_t serialdat_1[20] = { 0 };//设备号
	uint8_t serialdat_2[20] = { 0 };//设备号
	uint8_t serialdat_3[20] = { 0 };//数据1
	uint8_t serialdat_4[20] = { 0 };//数据2
	uint8_t serialdat_5[20] = { 0 };//数据3
	uint8_t serialdat_6[20] = { 0 };//数据4
	char EquipmentNumber[20] = EMID;//设备ID
	//第3帧数据
	serialdat_3[0] = 0xaa;
	serialdat_3[1] = 0x55;
	serialdat_3[2] = 0x03;
	serialdat_3[3] = SportInfo_Get.tim >> 8;
	serialdat_3[4] = SportInfo_Get.tim & 0x00ff;
	serialdat_3[5] = SportInfo_Get.count >> 8;
	serialdat_3[6] = SportInfo_Get.count & 0x00ff;
	serialdat_3[7] = SportInfo_Get.hot >> 8;
	serialdat_3[8] = SportInfo_Get.hot & 0x00ff;//转速无
	serialdat_3[9] = 0; //心率 -----------------无心率传感器
	serialdat_3[10] = 10;
	serialdat_3[11] = SportInfo_Get.freq >> 8;//坡度
	serialdat_3[12] = SportInfo_Get.freq & 0x00ff;//档位
	serialdat_3[13] = 13;//功率高位
	serialdat_3[14] = 14;//功率低位
	serialdat_3[15] = 15;//力矩高
	serialdat_3[16] = 16;//力矩低位
	serialdat_3[17] = 17;//校验位
	serialdat_3[18] = 18;//校验位
	serialdat_3[19] = 0xff;//校验位
	QF_CRC(serialdat_3, 20);

	//第4帧数据
	serialdat_4[0] = 0xaa;
	serialdat_4[1] = 0x55;
	serialdat_4[2] = 0x04;
	serialdat_4[3] = 1;//模式
	serialdat_4[4] = 2;//负荷高
	serialdat_4[5] = 3;//负荷低
	serialdat_4[6] = 4;//频率
	serialdat_4[7] = SportInfo_Get.count >> 8;
	serialdat_4[8] = SportInfo_Get.count & 0x00ff;//次数
	serialdat_4[9] = 7;
	serialdat_4[10] = 8;//最大肌力高
	serialdat_4[11] = 9;//最大肌力低
	serialdat_4[12] = 10;
	serialdat_4[13] = ((190 - 82) *(65 / 100) + 73) >> 8;//最大摄氧量高
	serialdat_4[14] = ((190 - 82) *(65 / 100) + 73) & 0x00ff;//最大摄氧量低位
	serialdat_4[15] = 0;//空
	serialdat_4[16] = 0;//空
	serialdat_4[17] = 0;//校验位
	serialdat_4[18] = 0;//校验位
	QF_CRC(serialdat_4, 20);
	//第5帧数据
	serialdat_5[0] = 0xaa;
	serialdat_5[1] = 0x55;
	serialdat_5[2] = 0x05;
	serialdat_5[3] = 0;
	serialdat_5[4] = 0;
	serialdat_5[5] = 0;
	serialdat_5[6] = 0;
	serialdat_5[7] = 0;
	serialdat_5[8] = 0;
	serialdat_5[9] = 0;
	serialdat_5[10] = 0;
	serialdat_5[11] = 0;
	serialdat_5[12] = 0;
	serialdat_5[13] = 0;
	serialdat_5[14] = 0;
	serialdat_5[15] = 0;
	serialdat_5[16] = 0;
	serialdat_5[17] = 0;
	serialdat_5[18] = 0;
	QF_CRC(serialdat_5, 20);


	serialdat_6[0] = 0xaa;
	serialdat_6[1] = 0x55;
	serialdat_6[2] = 0x0;
	serialdat_6[3] = 0;
	serialdat_6[4] = 0;
	serialdat_6[5] = 1;
	serialdat_6[6] = 0;
	serialdat_6[7] = 0;
	serialdat_6[8] = 0;
	serialdat_6[9] = 0;
	serialdat_6[10] = 0;
	serialdat_6[11] = 0;
	serialdat_6[12] = 0;
	serialdat_6[13] = 0;
	serialdat_6[14] = 0;
	serialdat_6[15] = 0;
	serialdat_6[16] = 0;
	serialdat_6[17] = 0;
	serialdat_6[18] = 0;
	serialdat_6[19] = 0xff;

	//设备信息aa 55 01 30 30 30 4a 53 2d 35 30 32 38 00 00 00 00 00 00 f6

	serialdat_1[0] = 0xaa;
	serialdat_1[1] = 0x55;
	serialdat_1[2] = 0x01;
	serialdat_1[3] = EquipmentNumber[0];
	serialdat_1[4] = EquipmentNumber[1];
	serialdat_1[5] = EquipmentNumber[2];
	serialdat_1[6] = EquipmentNumber[3];
	serialdat_1[7] = EquipmentNumber[4];
	serialdat_1[8] = EquipmentNumber[5];
	serialdat_1[9] = EquipmentNumber[6];
	serialdat_1[10] = EquipmentNumber[7];
	serialdat_1[11] = EquipmentNumber[8];
	serialdat_1[12] = EquipmentNumber[9];
	serialdat_1[13] = 0;
	serialdat_1[14] = 0;
	serialdat_1[15] = 0;
	serialdat_1[16] = 0;
	serialdat_1[17] = 0;
	serialdat_1[18] = 0;
	serialdat_1[19] = 0xf6;
	QF_CRC(serialdat_1, 20);
	//aa 55 02 30 30 30 4a 53 2d 35 30 32 38 00 00 00 00 00 00 f6
	serialdat_2[0] = 0xaa;
	serialdat_2[1] = 0x55;
	serialdat_2[2] = 0x02;
	serialdat_2[3] = EquipmentNumber[10];
	serialdat_2[4] = EquipmentNumber[11];
	serialdat_2[5] = EquipmentNumber[12];
	serialdat_2[6] = EquipmentNumber[13];
	serialdat_2[7] = EquipmentNumber[14];
	serialdat_2[8] = EquipmentNumber[15];
	serialdat_2[9] = EquipmentNumber[16];
	serialdat_2[10] = EquipmentNumber[17];
	serialdat_2[11] = EquipmentNumber[18];
	serialdat_2[12] = EquipmentNumber[19];
	serialdat_2[13] = 0x00;
	serialdat_2[14] = 0;
	serialdat_2[15] = 0;
	serialdat_2[16] = 0;
	serialdat_2[17] = 0;
	serialdat_2[18] = 0;
	serialdat_2[19] = 0xf6;
	QF_CRC(serialdat_2, 20);

	/*MY_USART_SendData(huart, serialdat_1, SERIALDATLEN);
	MY_USART_SendData(huart, serialdat_2, SERIALDATLEN);
	MY_USART_SendData(huart, serialdat_3, SERIALDATLEN);
	MY_USART_SendData(huart, serialdat_4, SERIALDATLEN);
	MY_USART_SendData(huart, serialdat_5, SERIALDATLEN);
	MY_USART_SendData(huart, serialdat_6, SERIALDATLEN);*/
	HAL_UART_Transmit(huart, serialdat_1, SERIALDATLEN, 20);
	HAL_UART_Transmit(huart, serialdat_2, SERIALDATLEN, 20);
	HAL_UART_Transmit(huart, serialdat_3, SERIALDATLEN, 20);
	HAL_UART_Transmit(huart, serialdat_4, SERIALDATLEN, 20);
	HAL_UART_Transmit(huart, serialdat_5, SERIALDATLEN, 20);
	HAL_UART_Transmit(huart, serialdat_6, SERIALDATLEN, 20);
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
