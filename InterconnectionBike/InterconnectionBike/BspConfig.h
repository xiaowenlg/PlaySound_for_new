#ifndef __BSPCONFIG_H
#define __BSPCONFIG_H
#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "system.h"
//传感器引脚
#define Sensor_Mag_Pin GPIO_PIN_0
#define Sensor_Mag_GPIO_Port GPIOA

#define  SYSCLOSE_PIN		GPIO_PIN_8 //关闭系统引脚，高电平关闭
#define  SYSCLOSE_PORT		GPIOB
#define  SYSIO				PBout(8)   //位带操作

//设备串口波特率设置
#define BLE_BAND				57600        //蓝牙串口
#define DEBUG_BAND				115200        //调试用串口
#define HMI_BAND				115200		//HML串口屏(淘晶驰)

//串口分配

#define  UART_BLE				&huart1			//蓝牙
#define  UART_CONNECTION		&huart2			//互联接口	
#define  UART_TFT				&huart3			//TFT屏(武汉晶显)
#define  UART_DEBUG				0              //Uart调试开关
#define SERIALDATLEN			20				//上传数据长度
//周期内分段运行
#define TIMER_ALL_PERIOD        10            //单位ms  定时器运行周期(大周期)
//按周期运行

#define PERIOD_DO_EXECUTE(TICK,PERIOD)          ((TICK)%(PERIOD/TIMER_ALL_PERIOD) == 0)

#define TEST_SENSOR_DATA        1000				//检测传感器值
#define SENDDATA				200
#define MES_PERIOD              200
#define TIMER2_PLAY_WAIT          5                 //等待播放时间，单位秒s
//语音地址
#define S_TEN               10               //十
#define S_HUNDRED			11				//百
#define S_THOUSAND          12				//千
#define S_THISSPORT         0x00		    //本次您活动了，
#define S_SPORTTIM          0x19              //活动时间
#define S_THISKCAL          0x02				//消耗热量
#define S_NUMBER			0x01			//次
#define S_HOUR				0x12				//时
#define S_MINUTE            0x13				//分
#define S_SECOND			0x14				//秒
#define S_KCAL              0x03				//千卡
#define S_POINT             21           //点 地址
#define S_WELCOME           0x04				//欢迎再次使用
#define S_MAXSPORT_TIP		0x20            //运动频率过大提示

#define PLAYARRAYLENGTH        50			//播放数组长度
#define TIMER_CLOSESYSTEM	   10*1000       //系统等待停止时间单位ms

#define TFT_VARIABLE_START		0x0004      //TFT屏变量起始地址
#define TFT_BUTTON				0x4F		//TFT屏上的按钮
//互联信息头
#define RES_AA                  0xaa
#define RES_55				    0x55
#define REQUEST_DATA			0x01				//请求数据命令
#define RESPONSE_DATA			0x02                //对方响应码
//信息
#define WEIGHT 60.00                     //体重
#define WHEEL_R				  0.5       //车轮半径 单位:m
#define PI						3.14 
#define PERIMETER				2*PI*WHEEL_R
//运动信息结构体
typedef struct SportInfo      //运动信息
{
	uint16_t count;//运动次数
	uint16_t freq;//运动频率
	long hot;//消耗热量
	uint16_t tim;//运动时间
	uint8_t    playstate;//播放状态
}Customerinfo;
//错误信息列表
#define ERROR_XQUEUE_CREAT				0x01			//消息队列创建错误

//计算外部电源电压，用来设置低电压关机
#define R_UP							39.00				//上偏置电阻	K
#define R_DOWN							15.00				//下偏置电阻	K
#define CAL_K_RES(voltage)				((R_UP+R_DOWN)/R_DOWN)*voltage         //计算电池电压值
#define VOLTAGE_T						2000				//电压检测周期单位ms
#define POWER_VOLTAGE_LOW				9500				//系统要求最低电压单位mV
//不同器材不同参数
#define CAL_K							2				//cal系数
#define EMID         "00010000300023000143"
//

#endif // !__BSPCONFIG_H
