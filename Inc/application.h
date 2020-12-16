#ifndef __application_H
#define __application_H
#include "main.h"


/*
WTN6040语音芯片
*/
#define  CLK_2A                 PBout(13) //时钟输出（位带操作）
#define  P_DATA_2A               PBout(12) //数据输出（位带操作）
#define  CLK_2A_PIN             GPIO_PIN_13//时钟引脚
#define  P_DATA_2A_PIN          GPIO_PIN_12//数据引脚
//#define  SYSCLOSE_PIN		GPIO_PIN_8 //关闭系统引脚，高电平关闭
//#define  SYSCLOSE_PORT		GPIOB
//#define  SYSIO				PBout(8)   //位带操作


/*TFT操作函数*/
#define array_length          100  //定义数组长度
#define USER_R3               0xA5  //帧头首字节
#define USER_RA               0x5A  //帧头次字节

void write_register_80_1byte(uint8_t address, uint8_t data);
void write_multiple_register_80(uint8_t address, uint8_t data_length, uint8_t *data);
void write_variable_store_82_1word(uint16_t address, uint16_t data);
void write_multiple_variable_store_82(uint16_t address, uint8_t data_length, uint16_t *data);

void TFT_playsound(uint8_t data_length, uint8_t *data, UART_HandleTypeDef *huart);
void playmusic(uint16_t num, uint8_t val);
void stopmusic(uint16_t num);
void Turen_Pic(uint16_t num);
void TFT_Beep(uint8_t n);//屏蜂鸣器响
void TFT_Readbytes(uint8_t adress, uint8_t readlen, UART_HandleTypeDef *huart);

void HMI_SetVal(UART_HandleTypeDef *huart, const char *com, uint32_t val);//发送数据到HMI屏;
void HMI_SetVal_t();//发送数据到HMI屏;
void HMI_SetVal_t(UART_HandleTypeDef *huart, const char *com, uint32_t val);
void HMI_SetTxt(UART_HandleTypeDef *huart, const char *com, char *txt);//发送数据字符串
#endif
