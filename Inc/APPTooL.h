#ifndef _APPTOOL_H
#define _APPTOOL_H
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#define HIGHT                 1
#define LOW                   0
//触发器数组大小
#define TRLENGHT              10


typedef void(*TrlCallback)(uint8_t);

void SingleTrig(TrlCallback p, uint8_t val, uint8_t inval, uint8_t index, uint8_t com);
uint8_t PrepartData(uint16_t dat, uint8_t *res_dat);// 语音地址分解计算;
uint8_t MergedData(double numdata, uint8_t *dat,uint8_t com);
uint8_t GetPlayData(uint16_t num, uint16_t tim, double mcal, uint8_t *dat);
int insertArray(uint8_t *arr, uint16_t arrlen, uint8_t *inarr, uint16_t inarrlen, uint16_t pos);
int insertOneByte(uint8_t *arr, uint16_t arrlen, uint8_t indata, uint16_t pos);
uint8_t StrJoin(char *mac, char*id, char *lastdata);
uint16_t ConsumeHeat(float weight, float tim, float v);
#endif // !_APPTOOL_H
