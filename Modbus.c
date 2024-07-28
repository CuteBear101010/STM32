#include "Modbus.h"
#include "stdlib.h"
#include "string.h"


static uint8_t DisposeState;
static uint8_t Tx_Buff[MASSAGE_MAX_LENGTH];
static uint8_t Rx_Buff[MASSAGE_MAX_LENGTH];
static uint16_t Device_Code;//设备型号

uint16_t Device_Addr;//从机地址

Modbus_Data_ChartTypedef  Modbus_Data[] = {
  
  {0,   RO|UINT16_T,  0,  0,        &Device_Code},                
  {1,   RW|UINT16_T,  1,  127,      &Device_Addr},                
  {2,   RW|UINT16_T,  0,  255,      &Motor.Run_State.AllBits},                
  {3,   RW|UINT16_T,  0,  63,       &Motor.Mode.AllBits},                
  {4,   RW|UINT16_T,  0,  100,      &Motor.Set_Angle[0]},                
  {5,   RW|UINT16_T,  0,  100,      &Motor.Set_Angle[1]},                
  {6,   RW|UINT16_T,  0,  100,      &Motor.Set_Angle[2]},                
  {7,   RW|UINT16_T,  0,  100,      &Motor.Set_Angle[3]},                
  {8,   RW|UINT16_T,  0,  4095,     &Motor.Function_State.AllBits},                
  {9,   RW|UINT16_T,  0,  100,      &Alarm.Angle_DIF[0]},                
  {10,  RW|UINT16_T,  0,  100,      &Alarm.Angle_DIF[1]}, 
  {11,  RW|UINT16_T,  0,  100,      &Alarm.OversizeCurrents_Value[0]},                
  {12,  RW|UINT16_T,  0,  100,      &Alarm.OversizeCurrents_Value[1]}, 
  {13,  RW|UINT16_T,  0,  500,      &Alarm.OversizeVol_Value}, 
  {14,  RW|UINT16_T,  0,  500,      &Alarm.UnderVol_Value}, 
  {15,  RW|UINT16_T,  0,  100,      &Motor.Adjust_Value[0]},                
  {16,  RW|UINT16_T,  0,  100,      &Motor.Adjust_Value[1]}, 
  {17,  RO|UINT16_T,  0,  0,        &Alarm.State.AllBits}, 
  {18,  RO|INT16_T,   0,  0,        &Motor.Cur_Angle[0]}, 
  {19,  RO|INT16_T,   0,  0,        &Motor.Cur_Angle[1]}, 
  {20,  RO|INT16_T,   0,  0,        &Motor.Cur_Angle[2]}, 
  {21,  RO|INT16_T,   0,  0,        &Motor.Cur_Angle[3]}, 
  {22,  RO|UINT16_T,  0,  0,        &Motor.Cur_Angle_Value[0]}, 
  {23,  RO|UINT16_T,  0,  0,        &Motor.Cur_Angle_Value[1]}, 
  {24,  RO|UINT16_T,  0,  0,        &Motor.Cur_Angle_Value[2]}, 
  {25,  RO|UINT16_T,  0,  0,        &Motor.Cur_Angle_Value[3]}, 
  {26,  RO|UINT16_T,  0,  0,        &Electricity.Cur_Currents[0]}, 
  {27,  RO|UINT16_T,  0,  0,        &Electricity.Cur_Currents[1]},
  {28,  RO|UINT16_T,  0,  0,        &Electricity.Cur_Vol},
  {29,  WO|UINT16_T,  1,  255,      &Motor.Calibrate.AllBits}, 
  {30,  WO|UINT16_T,  1,  1,        &Alarm.Clean},   
  {31,  RO|UINT16_T,  0,  0,        &Motor.Low_Angle_Value[0]},                
  {32,  RO|UINT16_T,  0,  0,        &Motor.Top_Angle_Value[0]},                
  {33,  RO|UINT16_T,  0,  0,        &Motor.Low_Angle_Value[1]},                
  {34,  RO|UINT16_T,  0,  0,        &Motor.Top_Angle_Value[1]},                
  {35,  RO|UINT16_T,  0,  0,        &Motor.Low_Angle_Value[2]},                
  {36,  RO|UINT16_T,  0,  0,        &Motor.Top_Angle_Value[2]},                
  {37,  RO|UINT16_T,  0,  0,        &Motor.Low_Angle_Value[3]},                
  {38,  RO|UINT16_T,  0,  0,        &Motor.Top_Angle_Value[3]},                

};
const uint16_t Modbus_Data_Max = (sizeof(Modbus_Data)/sizeof(Modbus_Data_ChartTypedef));

void Modbus_Init(void){
  Device_Code = 6208;
  Device_Addr = 0x01;
  Serial_Init();
}
/*串口初始化*/
static void Serial_Init(void){
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  
  GPIO_InitTypeDef    GPIO_InitStruct;
  NVIC_InitTypeDef    NVIC_InitStructure;
  USART_InitTypeDef   USART_InitStruct;
  DMA_InitTypeDef     DMA_InitStructure;

  GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  USART_InitStruct.USART_BaudRate = 9600;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;//无校验
  USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_Init(USART3, &USART_InitStruct);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;             //抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;                  //从优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                     //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);                                     //根据指定的参数初始化VIC寄存器  
   
  
  DMA_DeInit(DMA1_Channel2);
  DMA_DeInit(DMA1_Channel3);

  DMA_InitStructure.DMA_PeripheralBaseAddr   = (uint32_t)(&USART3->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr       = (uint32_t)Tx_Buff;
  DMA_InitStructure.DMA_DIR                  = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize           = MASSAGE_MAX_LENGTH;
  DMA_InitStructure.DMA_PeripheralInc        = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc            = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize   = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize       = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode                 = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority             = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M                  = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2,&DMA_InitStructure);

  USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
    
  /*串口DMA配置*/
  DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)&(USART3->DR);          //DMA外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr        = (uint32_t)Rx_Buff;   //DMA存储器0地址
  DMA_InitStructure.DMA_DIR                   = DMA_DIR_PeripheralSRC ;           //外设到存储器模式
  DMA_InitStructure.DMA_BufferSize            = MASSAGE_MAX_LENGTH;                //数据传输量
  DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;        //外设非增量模式
  DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Enable;             //存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_Byte;      //外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize        = DMA_MemoryDataSize_Byte;          //存储器数据长度:8位
  DMA_InitStructure.DMA_Mode                  = DMA_Mode_Normal;                //使用普通模式
  DMA_InitStructure.DMA_Priority              = DMA_Priority_Medium;              //中等优先级
  DMA_InitStructure.DMA_M2M                   = DMA_M2M_Disable;                  //禁止内存到内存的传输
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
  
  DMA_Cmd(DMA1_Channel3, ENABLE); /*使能DMA*/
  
  USART_ClearFlag(USART3, USART_FLAG_TC); //清除发送标志位
  USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);/*使能串口接收超时中断*/
  USART_Cmd(USART3, ENABLE);
 
  USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE); /*使能串口接收DMA*/
 
}

void Task_Modbus(void){
  uint16_t rx_cnt  = 0;    
  uint16_t register_addr = 0;    //起始寄存器地址
  uint16_t register_NUM  = 0;    //寄存器的数量
  uint16_t tx_CRC = 0;    //反馈消息的CRC值
  uint16_t value_temp  = 0;    
  uint8_t  value_err   = 0;    //数据错误 标志位
  uint8_t  addr_exist  = 0;    //访问寄存器地址错误 标志位
  
  if(DisposeState == 0)
    return;
  rx_cnt = MASSAGE_MAX_LENGTH - DMA_GetCurrDataCounter(DMA1_Channel3);//缓存最大长度 - 缓存池剩余长度 等于所用长度
  
  if(!Data_Rough_Check(Rx_Buff,rx_cnt)){//如果通过了数据粗检查 
    register_addr = (Rx_Buff[2]<<8)|Rx_Buff[3];//寄存器起始地址
    register_NUM = (Rx_Buff[4]<<8)|Rx_Buff[5];//计算操作的保持寄存器数量
    addr_exist = 0;//访问寄存器地址存在与否标志位 0 地址不存在 1 地址存在
    value_err = 0;//数据错误 标志位 0 数据无错 1 数据错误
    switch(Rx_Buff[1]){//选择功能码
      
    case 0x03://读保持寄存器        
      /*编写正常的反馈报文*/
      Tx_Buff[0] = DEVICE_ADDR;
      Tx_Buff[1] = Rx_Buff[1];
      Tx_Buff[2] = register_NUM*2;
      
      for(uint16_t i = 0; i < register_NUM; i++){
        Tx_Buff[3+i*2]   = 0;
        Tx_Buff[3+i*2+1] = 0;
        for(uint16_t j = 0; j < Modbus_Data_Max; j++){
          if(Modbus_Data[j].Serial_Number == (i + register_addr)){//如果查询到了目标寄存器
            if(((Modbus_Data[j].Type&0x0F) == UINT16_T)||((Modbus_Data[j].Type&0x0F) == INT16_T)){//如果数据格式是uint16_t 或者 int16_t 类型
              memcpy(&value_temp,Modbus_Data[j].Source,2);
              Tx_Buff[3+i*2]   = (value_temp&0xFF00)>>8;
              Tx_Buff[3+i*2+1] = (value_temp&0x00FF)>>0;
            }
          }
        }
      }
      tx_CRC = Modbus_CRC16(Tx_Buff,(Tx_Buff[2]+3));
      
      Tx_Buff[Tx_Buff[2]+3] = (tx_CRC&0xFF00)>>8;    //CRC高八位
      Tx_Buff[Tx_Buff[2]+4] = (tx_CRC&0x00FF)>>0;    //CRC低八位
      
      Modbus_Serial_SendData(Tx_Buff,(Tx_Buff[2]+5));//发送反馈报文
      break; 
      
    case 0x06://写单个保持寄存器
      
      value_temp = (Rx_Buff[4]<<8)|Rx_Buff[5];
      for(uint16_t j = 0; j < Modbus_Data_Max; j++){
        if((Modbus_Data[j].Serial_Number == register_addr)&&((Modbus_Data[j].Type&0xF0) != RO)){//查询到可写寄存器编号
          
          if((Modbus_Data[j].Type&0x0F) == UINT16_T){//uint16_t类型 
            if((value_temp >= Modbus_Data[j].MinValue)&&(value_temp <= Modbus_Data[j].MaxValue)){//数据在范围之内
              *(uint16_t *)Modbus_Data[j].Source = value_temp;
              Modbus_Function(Modbus_Data[j].Serial_Number,value_temp);
            }
            else{
              value_err = 1;//数值错误
            }
          }
          
          if((Modbus_Data[j].Type&0x0F) == INT16_T){//int16_t类型 
            if(((int16_t)value_temp >= Modbus_Data[j].MinValue)&&((int16_t)value_temp <= Modbus_Data[j].MaxValue)){//数据在范围之内
              *(int16_t *)Modbus_Data[j].Source = (int16_t)value_temp;
              Modbus_Function(Modbus_Data[j].Serial_Number,value_temp);//记得在回调函数case里将value_temp强制转换为int16_t型数据
            }
            else{
              value_err = 1;//数值错误
            }
          }
          
          addr_exist = 1;//寄存器存在
          break;
        }
      }
      
      if(addr_exist == 0){//未查询到可写入的寄存器
        Misdata_Dispose(Rx_Buff,0x02); //非法数据地址处理过程
        break;
      }
      
      if(value_err == 1){//写入的数值错误
        Misdata_Dispose(Rx_Buff,0x03); //数值错误处理过程
        break;
      }
      
      /*编写正常的反馈报文*/
      Tx_Buff[0] = DEVICE_ADDR;
      for(uint8_t i = 1; i < 6; i++){
        Tx_Buff[i] = Rx_Buff[i];
      } 
      tx_CRC = Modbus_CRC16(Tx_Buff,6);
      Tx_Buff[6] = (tx_CRC&0xFF00)>>8; //CRC高八位
      Tx_Buff[7] = (tx_CRC&0x00FF)>>0; //CRC低八位

      Modbus_Serial_SendData(Tx_Buff,8);//发送反馈报文
      break;
      
    case 0x10://写连续多个保持寄存器    
      for(uint16_t i = 0; i < register_NUM; i++){
        addr_exist = 0;
        value_temp = (Rx_Buff[i*2+7]<<8)|Rx_Buff[i*2+8];//计算
        for(uint16_t j = 0; j < Modbus_Data_Max; j++){
          if((Modbus_Data[j].Serial_Number == (register_addr + i))&&(Modbus_Data[j].Type != RO)){//查询到可写寄存器编号
            
            if((Modbus_Data[j].Type&0x0F) == UINT16_T){//uint16_t类型 
              if((value_temp < Modbus_Data[j].MinValue)||(value_temp > Modbus_Data[j].MaxValue)){
                value_err = 1;//数值错误
              }
            }
            
            if((Modbus_Data[j].Type&0x0F) == INT16_T){//int16_t类型 
              if(((int16_t)value_temp < Modbus_Data[j].MinValue)||((int16_t)value_temp > Modbus_Data[j].MaxValue)){
                value_err = 1;//数值错误
              }
            }
            
            addr_exist = 1;//该寄存器存在
            break;
          }  
        }
        if((addr_exist == 0)||(value_err == 1))//如果有寄存器不存在，或者有的寄存器写入数值不正确
          break;
      }
      
      if(addr_exist == 0){//未查询到可写入的寄存器
        Misdata_Dispose(Rx_Buff,0x02); //非法数据地址
        break;
      }
      
      if(value_err == 1){//写入的数值错误
        Misdata_Dispose(Rx_Buff,0x03); //数值错误
        break;
      }
      
      for(uint16_t i = 0; i < register_NUM; i++){
        value_temp = (Rx_Buff[i*2+7]<<8)|Rx_Buff[i*2+8];//计算
        for(uint16_t j = 0; j < Modbus_Data_Max; j++){
          if(Modbus_Data[j].Serial_Number == (register_addr + i)){//查询到可写寄存器编号
            
            if(((Modbus_Data[j].Type&0x0F) == UINT16_T)||((Modbus_Data[j].Type&0x0F) == INT16_T)){//如果数据格式是uint16_t 或者 int16_t 类型
              memcpy(Modbus_Data[j].Source,&value_temp,2);
            }
            
            Modbus_Function(Modbus_Data[j].Serial_Number,value_temp);//记得在回调函数case里将value_temp强制转换为int16_t型数据
          }
        }
      }
      
      Tx_Buff[0] = DEVICE_ADDR;
      /*编写正常的反馈报文*/
      for(uint8_t i = 1; i < 6; i++){
        Tx_Buff[i] = Rx_Buff[i];
      } 
      tx_CRC = Modbus_CRC16(Tx_Buff,6);
      Tx_Buff[6] = (tx_CRC&0xFF00)>>8; //CRC高八位
      Tx_Buff[7] = (tx_CRC&0x00FF)>>0; //CRC低八位
      
      Modbus_Serial_SendData(Tx_Buff,8);//发送反馈报文
      break;
    }
  }
  DisposeState = 0;
  DMA_SetCurrDataCounter(DMA1_Channel3,MASSAGE_MAX_LENGTH);//重设数据剩余长度
  DMA_Cmd(DMA1_Channel3,ENABLE);//开启接收DMA 重新接收数据
}
/*回调函数 写入特定值时 调用回调函数*/
static void Modbus_Function(uint16_t serial,uint16_t value){
  
  switch(serial){
  case 0:break;
  case 1:break;
  case 2:break;
  case 3:break;
  }
}
/*进行粗略的数据检查
返回 0 数据粗检 正确 
返回 1 数据粗检 错误*/
static uint8_t Data_Rough_Check(uint8_t *message,uint8_t length){
  uint16_t massageCRC = 0;    //消息中的CRC
  uint16_t computeCRC = 0;    //计算出的CRC
  uint16_t register_addr = 0;    //起始寄存器地址
  uint16_t register_NUM  = 0;    //寄存器的数量
  
  /*获取消息中的CRC校验码*/
  massageCRC = (message[length-2]<<8)|message[length-1];
  computeCRC = Modbus_CRC16(message,length - 2);
  
  if((massageCRC != computeCRC)||((message[0] != DEVICE_ADDR)&&(message[0] != 0x00)))//CRC校验失败或从机地址错误 直接丢弃报文
    return 1;
  
  if((((message[1] == 0x03)||(message[1] == 0x06))&&(length != 8))||((message[1] == 0x10)&&(length != message[6] + 9)))//数据长度错误 直接丢弃报文
    return 1;
  
  if((message[1] == 0x03)||(message[1] == 0x10)){
    register_NUM = (message[4]<<8)|message[5];//计算操作的保持寄存器数量
    if(register_NUM == 0)//如果寄存器数量为0 认为数据错误
      return 1;
  }
  
  if((message[1] == 0x10)&&(register_NUM*2 != message[6]))//如果功能码为0x10 且 寄存器数量和数据长度计算结果不相符 认为数据错误
    return 1;  
  
  if((message[1] != 0x03)&&(message[1] != 0x06)&&(message[1] != 0x10)){//功能码错误 
    Misdata_Dispose(message,0x01);// 非法功能处理过程
    return 1;
  }
  
  register_addr = (message[2]<<8)|message[3];//寄存器起始地址
  
  if(register_addr > HOLD_REGISTER_MAX){//寄存器地址大于寄存器最大地址 
    Misdata_Dispose(message,0x02); //非法数据地址处理过程
    return 1;
  }
  
  if((message[1] == 0x03)||(message[1] == 0x10)){//如果功能码为0x03或0x10
    if((register_addr + register_NUM) > HOLD_REGISTER_MAX){//如果读取的范围大于寄存器最大地址 
      Misdata_Dispose(message,0x02); //非法数据地址处理过程
      return 1;
    }
  }
  return 0;
}

/*非法数据的处理过程*/
static void Misdata_Dispose(uint8_t *message,uint8_t erro_code){
  
  uint8_t  err_tx[5] = {0};
  uint16_t err_tx_CRC = 0;
  
  err_tx[0] = message[0];
  err_tx[1] = message[1]|0x80;
  err_tx[2] = erro_code;//错误代码
  
  err_tx_CRC = Modbus_CRC16(err_tx,3);
  
  err_tx[3]= (uint8_t)((err_tx_CRC&0xFF00)>>8);    //CRC高八位
  err_tx[4] = (uint8_t)(err_tx_CRC&0x00FF);         //CRC低八位
  
  Modbus_Serial_SendData(err_tx,5);//发送反馈报文
  
}

/*串口DMA发送数据*/
void Modbus_Serial_SendData(uint8_t *buff,uint16_t length){
  memcpy(Tx_Buff,buff,length);
  
  DMA_Cmd(DMA1_Channel2, DISABLE); 
  DMA_SetCurrDataCounter(DMA1_Channel2, length); 
  DMA_Cmd(DMA1_Channel2, ENABLE); 
}

void USART3_IRQHandler(void){
  /*接收空闲中断*/
  if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET){
    
    USART3->SR;//进IDLED中断后要读SR,读DR寄存器后才能清除本次中断，不然下次进来还是这次中断
    USART3->DR;

    DisposeState = 1;
    
    DMA_Cmd(DMA1_Channel3,DISABLE);//关闭DMA结束 停止串口接收数据
    USART_ClearITPendingBit(USART3, USART_IT_IDLE);    /*清除接收超时标志*/
  }
}
/*CRC校验*/
static uint16_t Modbus_CRC16(uint8_t *arr_buff, uint8_t len){
  
  uint16_t crc=0xFFFF;
  
  for (uint8_t TX_Cnt=0; TX_Cnt<len;TX_Cnt++){
    crc=crc ^*arr_buff++;
    for (uint8_t i=0; i<8; i++){
      if( ( crc&0x0001) >0){
        crc=crc>>1;
        crc=crc^ 0xA001;
      }
      else
        crc=crc>>1;
    }
  }
  uint8_t t = (uint8_t)((crc&0xFF00)>>8);
  crc<<=8;
  crc|=t;
  return crc;
  
}




