#ifndef __BSPCONFIG_H
#define __BSPCONFIG_H
#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "system.h"
//����������
#define Sensor_Mag_Pin GPIO_PIN_0
#define Sensor_Mag_GPIO_Port GPIOA

#define  SYSCLOSE_PIN		GPIO_PIN_8 //�ر�ϵͳ���ţ��ߵ�ƽ�ر�
#define  SYSCLOSE_PORT		GPIOB
#define  SYSIO				PBout(8)   //λ������

//�豸���ڲ���������
#define BLE_BAND				57600        //��������
#define DEBUG_BAND				115200        //�����ô���
#define HMI_BAND				115200		//HML������(�Ծ���)

//���ڷ���

#define  UART_BLE				&huart1			//����
#define  UART_CONNECTION		&huart2			//�����ӿ�	
#define  UART_TFT				&huart3			//TFT��(�人����)
#define  UART_DEBUG				0              //Uart���Կ���
#define SERIALDATLEN			20				//�ϴ����ݳ���
//�����ڷֶ�����
#define TIMER_ALL_PERIOD        10            //��λms  ��ʱ����������(������)
//����������

#define PERIOD_DO_EXECUTE(TICK,PERIOD)          ((TICK)%(PERIOD/TIMER_ALL_PERIOD) == 0)

#define TEST_SENSOR_DATA        1000				//��⴫����ֵ
#define SENDDATA				200
#define MES_PERIOD              200
#define TIMER2_PLAY_WAIT          5                 //�ȴ�����ʱ�䣬��λ��s
//������ַ
#define S_TEN               10               //ʮ
#define S_HUNDRED			11				//��
#define S_THOUSAND          12				//ǧ
#define S_THISSPORT         0x00		    //��������ˣ�
#define S_SPORTTIM          0x19              //�ʱ��
#define S_THISKCAL          0x02				//��������
#define S_NUMBER			0x01			//��
#define S_HOUR				0x12				//ʱ
#define S_MINUTE            0x13				//��
#define S_SECOND			0x14				//��
#define S_KCAL              0x03				//ǧ��
#define S_POINT             21           //�� ��ַ
#define S_WELCOME           0x04				//��ӭ�ٴ�ʹ��
#define S_MAXSPORT_TIP		0x20            //�˶�Ƶ�ʹ�����ʾ

#define PLAYARRAYLENGTH        50			//�������鳤��
#define TIMER_CLOSESYSTEM	   10*1000       //ϵͳ�ȴ�ֹͣʱ�䵥λms

#define TFT_VARIABLE_START		0x0004      //TFT��������ʼ��ַ
#define TFT_BUTTON				0x4F		//TFT���ϵİ�ť
//������Ϣͷ
#define RES_AA                  0xaa
#define RES_55				    0x55
#define REQUEST_DATA			0x01				//������������
#define RESPONSE_DATA			0x02                //�Է���Ӧ��
//��Ϣ
#define WEIGHT 60.00                     //����
#define WHEEL_R				  0.5       //���ְ뾶 ��λ:m
#define PI						3.14 
#define PERIMETER				2*PI*WHEEL_R
//�˶���Ϣ�ṹ��
typedef struct SportInfo      //�˶���Ϣ
{
	uint16_t count;//�˶�����
	uint16_t freq;//�˶�Ƶ��
	long hot;//��������
	uint16_t tim;//�˶�ʱ��
	uint8_t    playstate;//����״̬
}Customerinfo;
//������Ϣ�б�
#define ERROR_XQUEUE_CREAT				0x01			//��Ϣ���д�������

//�����ⲿ��Դ��ѹ���������õ͵�ѹ�ػ�
#define R_UP							39.00				//��ƫ�õ���	K
#define R_DOWN							15.00				//��ƫ�õ���	K
#define CAL_K_RES(voltage)				((R_UP+R_DOWN)/R_DOWN)*voltage         //�����ص�ѹֵ
#define VOLTAGE_T						2000				//��ѹ������ڵ�λms
#define POWER_VOLTAGE_LOW				9500				//ϵͳҪ����͵�ѹ��λmV
//��ͬ���Ĳ�ͬ����
#define CAL_K							2				//calϵ��
#define EMID         "00010000300023000143"
//

#endif // !__BSPCONFIG_H
