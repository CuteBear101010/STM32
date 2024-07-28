#ifndef __MODBUS_H__
#define __MODBUS_H__

#include "stm32f10x.h"
 
#define DEVICE_ADDR              Device_Addr   //�ӻ���ַ

#define MASSAGE_MAX_LENGTH       270   //modbus������ֽ��� �ֽ�
#define HOLD_REGISTER_MAX        39   //�Ĵ����������

#ifndef NULL
#define NULL       0
#endif

#define   RO         0x10//�Ĵ���ֻ��
#define   WO         0x20//�Ĵ���ֻд
#define   RW         0x40//�Ĵ����ɶ�д
#define   UINT16_T   0x01  //uint16_t ����
#define   INT16_T    0x02  //int16_t ����

typedef struct{
  uint16_t   Serial_Number;//���õļĴ������
  uint16_t   Type;//��������
  int16_t    MinValue;//ֻ��д������Ч
  int16_t    MaxValue;//ֻ��д������Ч
  void*      Source;//����Դ
}Modbus_Data_ChartTypedef;

extern uint16_t Device_Addr;//�ӻ���ַ

void Modbus_Init(void);
void Task_Modbus(void);
/*���ڳ�ʼ��*/
static void Serial_Init(void);
/*�ص����� д���ض�ֵʱ ʵ��һЩ����*/
static void Modbus_Function(uint16_t serial,uint16_t value);
/*���д��Ե����ݼ��
���� 1 ���ݴּ� ͨ�� 
���� 0 ���ݴּ� ��ͨ��*/
static uint8_t Data_Rough_Check(uint8_t *message,uint8_t length);
/*�Ƿ����ݵĴ������*/
static void Misdata_Dispose(uint8_t *message,uint8_t erro_code);
/*��������*/
static void Modbus_Serial_SendData(uint8_t *data,uint16_t length);
/*��ȡ�ӻ���ַ*/
static uint8_t Get_Slave_Addr(void);
/*CRCУ��*/
static uint16_t Modbus_CRC16(uint8_t *arr_buff, uint8_t len);
#endif
