/*!
 *
 * TFT����74HC595����
 * date ���� 2019
 */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "application.h"
#include "system.h"
#include "usart.h"
#include "dwt_stm32_delay.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "BspConfig.h"

/*TFT����*/
//************************************
// ����:    write_register_80_1byte
// ����ȫ��:  write_register_80_1byte
// ��������:    public 
// ����ֵ:   void
// Qualifier:д�Ĵ���1�ֽ�
// ����: uint8_t address
// ����: uint8_t data
//************************************
void write_register_80_1byte(uint8_t address, uint8_t data) 
{
	uint8_t i;
	uint8_t array[array_length];

	array[0] = USER_R3;
	array[1] = USER_RA;
	array[2] = 0x03;               //ָ���
	array[3] = 0x80;
	array[4] = address;
	array[5] = data;
	HAL_UART_Transmit(UART_TFT, array, 6,0xffff);
}

void write_multiple_register_80(uint8_t address, uint8_t data_length, uint8_t *data)
{
	uint8_t i, nDataLen;
	uint8_t array[array_length];

	array[0] = USER_R3;
	array[1] = USER_RA;
	array[2] = 1 + 1 + data_length;   //����+��ַ+���ݳ���
	array[3] = 0x80;
	array[4] = address;

	for (i = 0; i < data_length; i++)
	{
		array[5 + i] = data[i];
	}

	nDataLen = array[2] + 3;        //��Ч�����

	MY_USART_SendData(UART_TFT, array, nDataLen);
}
//************************************
// ����:    write_variable_store_82_1word
// ����ȫ��:  write_variable_store_82_1word
// ��������:    public 
// ����ֵ:   void
// Qualifier:дһ������ֵ
// ����: uint16_t address
// ����: uint16_t data
//************************************
void write_variable_store_82_1word(uint16_t address, uint16_t data)
{
	uint8_t i;
	uint8_t array[array_length];

	array[0] = USER_R3;
	array[1] = USER_RA;
	array[2] = 1 + 2 + 2;                  
	array[3] = 0x82;
	array[4] = (address & 0xFF00) >> 8;    
	array[5] = address & 0x00FF;         
	array[6] = (data & 0xFF00) >> 8;      
	array[7] = data & 0x00FF;            

	//MY_USART_SendData(UART_TFT, array, 8);
	HAL_UART_Transmit(UART_TFT, array, 8, 0xffff);
}

//************************************
// ����:    write_multiple_variable_store_82
// ����ȫ��:  write_multiple_variable_store_82
// ��������:    public 
// ����ֵ:   void
// Qualifier:д�������ֵ
// ����: uint16_t address
// ����: uint8_t data_length
// ����: uint16_t * data
//************************************
void write_multiple_variable_store_82(uint16_t address, uint8_t data_length, uint16_t *data)
{
	uint8_t i, nDataLen;
	uint8_t array[array_length];

	array[0] = USER_R3;
	array[1] = USER_RA;
	array[2] = 1 + 2 + data_length * 2;  
	array[3] = 0x82;
	array[4] = (address & 0xFF00) >> 8;        
	array[5] = address & 0x00FF;             

	for (i = 0; i < data_length; i++)
	{
		array[6 + 2 * i] = (data[i] & 0xFF00) >> 8; 
		array[7 + 2 * i] = data[i] & 0x00FF;      
	}

	nDataLen = array[2] + 3;                 

	//MY_USART_SendData(UART_TFT, array, nDataLen);
	HAL_UART_Transmit(UART_TFT, array, nDataLen, 0xffff);
}

#ifdef TFTPLAY
void TFT_playsound(uint8_t data_length, uint8_t *data, UART_HandleTypeDef *huart)//TFT��������
{
	uint8_t i, nDataLen;
	uint8_t array[array_length];

	array[0] = USER_R3;
	array[1] = USER_RA;
	array[2] = 1 + 2 + data_length * 2;
	array[3] = 0x85;
	array[4] = 0x03;
	array[5] = 0x01;

	for (i = 0; i < data_length; i++)
	{
		//array[6 + 2 * i] = (data[i] & 0xFF00) >> 8;
		array[6 + 2 * i] = 0;
		array[7 + 2 * i] = data[i] & 0x00FF;
	}
	
	nDataLen = array[2] + 3;
	
	MY_USART_SendData(huart, array, nDataLen);
}


#endif
//************************************
// ����:    playmusic
// ����ȫ��:  playmusic
// ��������:    public 
// ����ֵ:   void
// Qualifier:���ֲ���
// ����: uint16_t num
// ����: uint8_t val
//************************************
void playmusic(uint16_t num, uint8_t val)
{
	uint8_t senddat[5] = {0};
	senddat[0] = 0x5b;
	senddat[1] = num >> 8;
	senddat[2] = num & 0x00ff;
	senddat[3] = 0x5A;
	if (val > 64)
		val = 64;
	senddat[4] = val;
	write_multiple_register_80(0x50, 5, senddat);
}
//************************************
// ����:    stopmusic
// ����ȫ��:  stopmusic
// ��������:    public 
// ����ֵ:   void
// Qualifier: //ֹͣ��������
// ����: uint16_t num    Ҫֹͣ�����ֵ�ַ��
//************************************
void stopmusic(uint16_t num)
{
	uint8_t senddat[3] = { 0 };
	senddat[0] = 0x5c;
	senddat[1] = num >> 8;
	senddat[2] = num & 0x00ff;
	write_multiple_register_80(0x50, 3, senddat);
}
//************************************
// ����:    Turen_Pic
// ����ȫ��:  Turen_Pic
// ��������:    public 
// ����ֵ:   void
// Qualifier:�л�ҳ��
// ����: uint16_t num
//************************************
void Turen_Pic(uint16_t num)
{
	uint8_t senddat[2] = { 0 };
	
	senddat[0] = num >> 8;
	senddat[1] = num & 0x00ff;
	write_multiple_register_80(0x03, 2, senddat);
	//pageID = num;
}
void TFT_Beep(uint8_t n)//TFT��������
{
	write_register_80_1byte(0x02, n);
}
void TFT_Readbytes(uint8_t adress,uint8_t readlen,UART_HandleTypeDef *huart)//���Ĵ���
{
	uint8_t senddat[3] = { 0 };
	senddat[0] = USER_R3;
	senddat[1] = USER_RA;
	senddat[2] = 3;
    senddat[3] = 0x81;
	senddat[4] = adress;
	senddat[5] = readlen;
	MY_USART_SendData(huart, senddat, 6);//���Ͷ�����

	
}
//���ͽ��շ�
void HMI_SendEnd(UART_HandleTypeDef *huart)
{
	MY_USART_SendByte(huart, 0xff);
	MY_USART_SendByte(huart, 0xff);
	MY_USART_SendByte(huart, 0xff);
}
//************************************
// ����:    HMI_SetVal
// ����ȫ��:  HMI_SetVal
// ��������:    public 
// ����ֵ:   void
// Qualifier: //�������ݵ�HMI��
// ����: UART_HandleTypeDef * huart
// ����: const char * com  �����ַ��� ��"n0.val="
// ����: uint32_t val    ����ֵ
//************************************
void HMI_SetVal(UART_HandleTypeDef *huart, const char *com,uint32_t val )//�������ݵ�HMI��
{
	Uart_printf(huart, "%s%d", com, val);
	HMI_SendEnd(huart);

}
void HMI_SetVal_t(UART_HandleTypeDef *huart, const char *com, uint32_t val)
{
	printf("%s%d", com, val); fflush(stdout);
	HMI_SendEnd(huart);

}

void HMI_SetTxt(UART_HandleTypeDef *huart, const char *com, char *txt)//�������ݵ�HMI��
{
	//char DArray[HMIARRAYLENGTH] = {0};
	//sprintf(DArray,"n0.val=%d", val);
	//sprintf(DArray, "%s%d", com, val);
	//MY_USART_chars(huart, "x");
	//sprintf(DArray, "%s\"%s\"",com, txt);
	//MY_USART_chars(huart, DArray);
	Uart_printf(huart, "%s\"%s\"", com, txt);
	HMI_SendEnd(huart);

}