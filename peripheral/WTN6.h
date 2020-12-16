#ifndef __WTN6_H
#define __WTN6_H
#include "main.h"
#include "stdint.h"
#include "stm32f1xx_hal.h"
/*
WTN6系列语音芯片
*/
#define  CLK_2A                 PBout(13) //时钟输出（位带操作）
#define  P_DATA_2A               PBout(12) //数据输出（位带操作）
#define  CLK_2A_PIN             GPIO_PIN_13//时钟引脚
#define  P_DATA_2A_PIN          GPIO_PIN_12//数据引脚
#define WTN6_BUSY_Pin		GPIO_PIN_14
#define WTN_GPIO_Port			GPIOB
//Function
void WTN6_Gpio_Init();				//初始化
uint8_t Line_2A_WTN6(uint8_t SB_DATA);//发送一个字节
void WTN6_PlayOneByte(uint8_t val, uint32_t timout);//播放一个地址
uint8_t WTN6_PlayArray(uint8_t arrlen, uint8_t *soundarr, uint32_t timout);//播放一组地址（联播）
uint8_t WTN6_PlayArray_1(uint8_t arrlen, uint8_t *soundarr, uint32_t timout);//播放一组地址（联播）
#endif
