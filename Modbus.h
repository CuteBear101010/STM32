#ifndef __MODBUS_H__
#define __MODBUS_H__

#include "stm32f10x.h"
 
#define DEVICE_ADDR              Device_Addr   //从机地址

#define MASSAGE_MAX_LENGTH       270   //modbus报文最长字节数 字节
#define HOLD_REGISTER_MAX        39   //寄存器最大数量

#ifndef NULL
#define NULL       0
#endif

#define   RO         0x10//寄存器只读
#define   WO         0x20//寄存器只写
#define   RW         0x40//寄存器可读写
#define   UINT16_T   0x01  //uint16_t 类型
#define   INT16_T    0x02  //int16_t 类型

typedef struct{
  uint16_t   Serial_Number;//启用的寄存器序号
  uint16_t   Type;//数据类型
  int16_t    MinValue;//只对写数据有效
  int16_t    MaxValue;//只对写数据有效
  void*      Source;//数据源
}Modbus_Data_ChartTypedef;

extern uint16_t Device_Addr;//从机地址

void Modbus_Init(void);
void Task_Modbus(void);
/*串口初始化*/
static void Serial_Init(void);
/*回调函数 写入特定值时 实现一些功能*/
static void Modbus_Function(uint16_t serial,uint16_t value);
/*进行粗略的数据检查
返回 1 数据粗检 通过 
返回 0 数据粗检 不通过*/
static uint8_t Data_Rough_Check(uint8_t *message,uint8_t length);
/*非法数据的处理过程*/
static void Misdata_Dispose(uint8_t *message,uint8_t erro_code);
/*发送数据*/
static void Modbus_Serial_SendData(uint8_t *data,uint16_t length);
/*获取从机地址*/
static uint8_t Get_Slave_Addr(void);
/*CRC校验*/
static uint16_t Modbus_CRC16(uint8_t *arr_buff, uint8_t len);
#endif
