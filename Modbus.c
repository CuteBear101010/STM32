#include "Modbus.h"
#include "stdlib.h"
#include "string.h"


static uint8_t DisposeState;
static uint8_t Tx_Buff[MASSAGE_MAX_LENGTH];
static uint8_t Rx_Buff[MASSAGE_MAX_LENGTH];
static uint16_t Device_Code;//�豸�ͺ�

uint16_t Device_Addr;//�ӻ���ַ

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
/*���ڳ�ʼ��*/
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
  USART_InitStruct.USART_Parity = USART_Parity_No;//��У��
  USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_Init(USART3, &USART_InitStruct);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;             //��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;                  //�����ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                     //IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);                                     //����ָ���Ĳ�����ʼ��VIC�Ĵ���  
   
  
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
    
  /*����DMA����*/
  DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)&(USART3->DR);          //DMA�����ַ
  DMA_InitStructure.DMA_MemoryBaseAddr        = (uint32_t)Rx_Buff;   //DMA�洢��0��ַ
  DMA_InitStructure.DMA_DIR                   = DMA_DIR_PeripheralSRC ;           //���赽�洢��ģʽ
  DMA_InitStructure.DMA_BufferSize            = MASSAGE_MAX_LENGTH;                //���ݴ�����
  DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;        //���������ģʽ
  DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Enable;             //�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_Byte;      //�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize        = DMA_MemoryDataSize_Byte;          //�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode                  = DMA_Mode_Normal;                //ʹ����ͨģʽ
  DMA_InitStructure.DMA_Priority              = DMA_Priority_Medium;              //�е����ȼ�
  DMA_InitStructure.DMA_M2M                   = DMA_M2M_Disable;                  //��ֹ�ڴ浽�ڴ�Ĵ���
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
  
  DMA_Cmd(DMA1_Channel3, ENABLE); /*ʹ��DMA*/
  
  USART_ClearFlag(USART3, USART_FLAG_TC); //������ͱ�־λ
  USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);/*ʹ�ܴ��ڽ��ճ�ʱ�ж�*/
  USART_Cmd(USART3, ENABLE);
 
  USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE); /*ʹ�ܴ��ڽ���DMA*/
 
}

void Task_Modbus(void){
  uint16_t rx_cnt  = 0;    
  uint16_t register_addr = 0;    //��ʼ�Ĵ�����ַ
  uint16_t register_NUM  = 0;    //�Ĵ���������
  uint16_t tx_CRC = 0;    //������Ϣ��CRCֵ
  uint16_t value_temp  = 0;    
  uint8_t  value_err   = 0;    //���ݴ��� ��־λ
  uint8_t  addr_exist  = 0;    //���ʼĴ�����ַ���� ��־λ
  
  if(DisposeState == 0)
    return;
  rx_cnt = MASSAGE_MAX_LENGTH - DMA_GetCurrDataCounter(DMA1_Channel3);//������󳤶� - �����ʣ�೤�� �������ó���
  
  if(!Data_Rough_Check(Rx_Buff,rx_cnt)){//���ͨ�������ݴּ�� 
    register_addr = (Rx_Buff[2]<<8)|Rx_Buff[3];//�Ĵ�����ʼ��ַ
    register_NUM = (Rx_Buff[4]<<8)|Rx_Buff[5];//��������ı��ּĴ�������
    addr_exist = 0;//���ʼĴ�����ַ��������־λ 0 ��ַ������ 1 ��ַ����
    value_err = 0;//���ݴ��� ��־λ 0 �����޴� 1 ���ݴ���
    switch(Rx_Buff[1]){//ѡ������
      
    case 0x03://�����ּĴ���        
      /*��д�����ķ�������*/
      Tx_Buff[0] = DEVICE_ADDR;
      Tx_Buff[1] = Rx_Buff[1];
      Tx_Buff[2] = register_NUM*2;
      
      for(uint16_t i = 0; i < register_NUM; i++){
        Tx_Buff[3+i*2]   = 0;
        Tx_Buff[3+i*2+1] = 0;
        for(uint16_t j = 0; j < Modbus_Data_Max; j++){
          if(Modbus_Data[j].Serial_Number == (i + register_addr)){//�����ѯ����Ŀ��Ĵ���
            if(((Modbus_Data[j].Type&0x0F) == UINT16_T)||((Modbus_Data[j].Type&0x0F) == INT16_T)){//������ݸ�ʽ��uint16_t ���� int16_t ����
              memcpy(&value_temp,Modbus_Data[j].Source,2);
              Tx_Buff[3+i*2]   = (value_temp&0xFF00)>>8;
              Tx_Buff[3+i*2+1] = (value_temp&0x00FF)>>0;
            }
          }
        }
      }
      tx_CRC = Modbus_CRC16(Tx_Buff,(Tx_Buff[2]+3));
      
      Tx_Buff[Tx_Buff[2]+3] = (tx_CRC&0xFF00)>>8;    //CRC�߰�λ
      Tx_Buff[Tx_Buff[2]+4] = (tx_CRC&0x00FF)>>0;    //CRC�Ͱ�λ
      
      Modbus_Serial_SendData(Tx_Buff,(Tx_Buff[2]+5));//���ͷ�������
      break; 
      
    case 0x06://д�������ּĴ���
      
      value_temp = (Rx_Buff[4]<<8)|Rx_Buff[5];
      for(uint16_t j = 0; j < Modbus_Data_Max; j++){
        if((Modbus_Data[j].Serial_Number == register_addr)&&((Modbus_Data[j].Type&0xF0) != RO)){//��ѯ����д�Ĵ������
          
          if((Modbus_Data[j].Type&0x0F) == UINT16_T){//uint16_t���� 
            if((value_temp >= Modbus_Data[j].MinValue)&&(value_temp <= Modbus_Data[j].MaxValue)){//�����ڷ�Χ֮��
              *(uint16_t *)Modbus_Data[j].Source = value_temp;
              Modbus_Function(Modbus_Data[j].Serial_Number,value_temp);
            }
            else{
              value_err = 1;//��ֵ����
            }
          }
          
          if((Modbus_Data[j].Type&0x0F) == INT16_T){//int16_t���� 
            if(((int16_t)value_temp >= Modbus_Data[j].MinValue)&&((int16_t)value_temp <= Modbus_Data[j].MaxValue)){//�����ڷ�Χ֮��
              *(int16_t *)Modbus_Data[j].Source = (int16_t)value_temp;
              Modbus_Function(Modbus_Data[j].Serial_Number,value_temp);//�ǵ��ڻص�����case�ｫvalue_tempǿ��ת��Ϊint16_t������
            }
            else{
              value_err = 1;//��ֵ����
            }
          }
          
          addr_exist = 1;//�Ĵ�������
          break;
        }
      }
      
      if(addr_exist == 0){//δ��ѯ����д��ļĴ���
        Misdata_Dispose(Rx_Buff,0x02); //�Ƿ����ݵ�ַ�������
        break;
      }
      
      if(value_err == 1){//д�����ֵ����
        Misdata_Dispose(Rx_Buff,0x03); //��ֵ���������
        break;
      }
      
      /*��д�����ķ�������*/
      Tx_Buff[0] = DEVICE_ADDR;
      for(uint8_t i = 1; i < 6; i++){
        Tx_Buff[i] = Rx_Buff[i];
      } 
      tx_CRC = Modbus_CRC16(Tx_Buff,6);
      Tx_Buff[6] = (tx_CRC&0xFF00)>>8; //CRC�߰�λ
      Tx_Buff[7] = (tx_CRC&0x00FF)>>0; //CRC�Ͱ�λ

      Modbus_Serial_SendData(Tx_Buff,8);//���ͷ�������
      break;
      
    case 0x10://д����������ּĴ���    
      for(uint16_t i = 0; i < register_NUM; i++){
        addr_exist = 0;
        value_temp = (Rx_Buff[i*2+7]<<8)|Rx_Buff[i*2+8];//����
        for(uint16_t j = 0; j < Modbus_Data_Max; j++){
          if((Modbus_Data[j].Serial_Number == (register_addr + i))&&(Modbus_Data[j].Type != RO)){//��ѯ����д�Ĵ������
            
            if((Modbus_Data[j].Type&0x0F) == UINT16_T){//uint16_t���� 
              if((value_temp < Modbus_Data[j].MinValue)||(value_temp > Modbus_Data[j].MaxValue)){
                value_err = 1;//��ֵ����
              }
            }
            
            if((Modbus_Data[j].Type&0x0F) == INT16_T){//int16_t���� 
              if(((int16_t)value_temp < Modbus_Data[j].MinValue)||((int16_t)value_temp > Modbus_Data[j].MaxValue)){
                value_err = 1;//��ֵ����
              }
            }
            
            addr_exist = 1;//�üĴ�������
            break;
          }  
        }
        if((addr_exist == 0)||(value_err == 1))//����мĴ��������ڣ������еļĴ���д����ֵ����ȷ
          break;
      }
      
      if(addr_exist == 0){//δ��ѯ����д��ļĴ���
        Misdata_Dispose(Rx_Buff,0x02); //�Ƿ����ݵ�ַ
        break;
      }
      
      if(value_err == 1){//д�����ֵ����
        Misdata_Dispose(Rx_Buff,0x03); //��ֵ����
        break;
      }
      
      for(uint16_t i = 0; i < register_NUM; i++){
        value_temp = (Rx_Buff[i*2+7]<<8)|Rx_Buff[i*2+8];//����
        for(uint16_t j = 0; j < Modbus_Data_Max; j++){
          if(Modbus_Data[j].Serial_Number == (register_addr + i)){//��ѯ����д�Ĵ������
            
            if(((Modbus_Data[j].Type&0x0F) == UINT16_T)||((Modbus_Data[j].Type&0x0F) == INT16_T)){//������ݸ�ʽ��uint16_t ���� int16_t ����
              memcpy(Modbus_Data[j].Source,&value_temp,2);
            }
            
            Modbus_Function(Modbus_Data[j].Serial_Number,value_temp);//�ǵ��ڻص�����case�ｫvalue_tempǿ��ת��Ϊint16_t������
          }
        }
      }
      
      Tx_Buff[0] = DEVICE_ADDR;
      /*��д�����ķ�������*/
      for(uint8_t i = 1; i < 6; i++){
        Tx_Buff[i] = Rx_Buff[i];
      } 
      tx_CRC = Modbus_CRC16(Tx_Buff,6);
      Tx_Buff[6] = (tx_CRC&0xFF00)>>8; //CRC�߰�λ
      Tx_Buff[7] = (tx_CRC&0x00FF)>>0; //CRC�Ͱ�λ
      
      Modbus_Serial_SendData(Tx_Buff,8);//���ͷ�������
      break;
    }
  }
  DisposeState = 0;
  DMA_SetCurrDataCounter(DMA1_Channel3,MASSAGE_MAX_LENGTH);//��������ʣ�೤��
  DMA_Cmd(DMA1_Channel3,ENABLE);//��������DMA ���½�������
}
/*�ص����� д���ض�ֵʱ ���ûص�����*/
static void Modbus_Function(uint16_t serial,uint16_t value){
  
  switch(serial){
  case 0:break;
  case 1:break;
  case 2:break;
  case 3:break;
  }
}
/*���д��Ե����ݼ��
���� 0 ���ݴּ� ��ȷ 
���� 1 ���ݴּ� ����*/
static uint8_t Data_Rough_Check(uint8_t *message,uint8_t length){
  uint16_t massageCRC = 0;    //��Ϣ�е�CRC
  uint16_t computeCRC = 0;    //�������CRC
  uint16_t register_addr = 0;    //��ʼ�Ĵ�����ַ
  uint16_t register_NUM  = 0;    //�Ĵ���������
  
  /*��ȡ��Ϣ�е�CRCУ����*/
  massageCRC = (message[length-2]<<8)|message[length-1];
  computeCRC = Modbus_CRC16(message,length - 2);
  
  if((massageCRC != computeCRC)||((message[0] != DEVICE_ADDR)&&(message[0] != 0x00)))//CRCУ��ʧ�ܻ�ӻ���ַ���� ֱ�Ӷ�������
    return 1;
  
  if((((message[1] == 0x03)||(message[1] == 0x06))&&(length != 8))||((message[1] == 0x10)&&(length != message[6] + 9)))//���ݳ��ȴ��� ֱ�Ӷ�������
    return 1;
  
  if((message[1] == 0x03)||(message[1] == 0x10)){
    register_NUM = (message[4]<<8)|message[5];//��������ı��ּĴ�������
    if(register_NUM == 0)//����Ĵ�������Ϊ0 ��Ϊ���ݴ���
      return 1;
  }
  
  if((message[1] == 0x10)&&(register_NUM*2 != message[6]))//���������Ϊ0x10 �� �Ĵ������������ݳ��ȼ���������� ��Ϊ���ݴ���
    return 1;  
  
  if((message[1] != 0x03)&&(message[1] != 0x06)&&(message[1] != 0x10)){//��������� 
    Misdata_Dispose(message,0x01);// �Ƿ����ܴ������
    return 1;
  }
  
  register_addr = (message[2]<<8)|message[3];//�Ĵ�����ʼ��ַ
  
  if(register_addr > HOLD_REGISTER_MAX){//�Ĵ�����ַ���ڼĴ�������ַ 
    Misdata_Dispose(message,0x02); //�Ƿ����ݵ�ַ�������
    return 1;
  }
  
  if((message[1] == 0x03)||(message[1] == 0x10)){//���������Ϊ0x03��0x10
    if((register_addr + register_NUM) > HOLD_REGISTER_MAX){//�����ȡ�ķ�Χ���ڼĴ�������ַ 
      Misdata_Dispose(message,0x02); //�Ƿ����ݵ�ַ�������
      return 1;
    }
  }
  return 0;
}

/*�Ƿ����ݵĴ������*/
static void Misdata_Dispose(uint8_t *message,uint8_t erro_code){
  
  uint8_t  err_tx[5] = {0};
  uint16_t err_tx_CRC = 0;
  
  err_tx[0] = message[0];
  err_tx[1] = message[1]|0x80;
  err_tx[2] = erro_code;//�������
  
  err_tx_CRC = Modbus_CRC16(err_tx,3);
  
  err_tx[3]= (uint8_t)((err_tx_CRC&0xFF00)>>8);    //CRC�߰�λ
  err_tx[4] = (uint8_t)(err_tx_CRC&0x00FF);         //CRC�Ͱ�λ
  
  Modbus_Serial_SendData(err_tx,5);//���ͷ�������
  
}

/*����DMA��������*/
void Modbus_Serial_SendData(uint8_t *buff,uint16_t length){
  memcpy(Tx_Buff,buff,length);
  
  DMA_Cmd(DMA1_Channel2, DISABLE); 
  DMA_SetCurrDataCounter(DMA1_Channel2, length); 
  DMA_Cmd(DMA1_Channel2, ENABLE); 
}

void USART3_IRQHandler(void){
  /*���տ����ж�*/
  if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET){
    
    USART3->SR;//��IDLED�жϺ�Ҫ��SR,��DR�Ĵ����������������жϣ���Ȼ�´ν�����������ж�
    USART3->DR;

    DisposeState = 1;
    
    DMA_Cmd(DMA1_Channel3,DISABLE);//�ر�DMA���� ֹͣ���ڽ�������
    USART_ClearITPendingBit(USART3, USART_IT_IDLE);    /*������ճ�ʱ��־*/
  }
}
/*CRCУ��*/
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




