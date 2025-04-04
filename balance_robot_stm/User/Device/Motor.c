#include "Motor.h"

// CAN1
DM_Motor_Info_Typedef DM_4310_Motor_leftfront = {
 
	 .CANFrameInfo = {
		.CAN_ID = 0x08, 
	  .Master_ID = 0x04, 
	 },
};

DM_Motor_Info_Typedef DM_4310_Motor_leftback = {
 
	 .CANFrameInfo = {
		.CAN_ID = 0x06, 
	  .Master_ID = 0x03, 
	 },
};

DM_Motor_Info_Typedef DM_6215_Motor_left = {
 
	 .CANFrameInfo = {
		.CAN_ID = 0x01, 
	  .Master_ID = 0x00, 
	 },
};

// CAN2
DM_Motor_Info_Typedef DM_4310_Motor_rightfront = {
 
	 .CANFrameInfo = {
		.CAN_ID = 0x06, 
	  .Master_ID = 0x03, 
	 },
};

DM_Motor_Info_Typedef DM_4310_Motor_rightback = {
 
	 .CANFrameInfo = {
		.CAN_ID = 0x08, 
	  .Master_ID = 0x04, 
	 },
};

DM_Motor_Info_Typedef DM_6215_Motor_right = {
 
	 .CANFrameInfo = {
		.CAN_ID = 0x01, 
	  .Master_ID = 0x00, 
	 },
};

// 电机参数
#define P_MAX 12.5f
#define V_MAX 45.f
#define T_MAX 10.f


DM_Motor_Control_Typedef DM_Motor_Control;

//uint -> float解码函数
//param: 原数据，映射范围min & max，位数
//以16位为例，x_int=32767时位于0点附近
static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

//同上
static int float_to_uint(float X_float, float X_min, float X_max, int bits){
    float span = X_max - X_min;
    float offset = X_min;
    return (int) ((X_float-offset)*((float)((1<<bits)-1))/span);
}

// 控制帧函数
// param: CAN发送结构体地址，发送的CAN_ID，控制命令
// 奇怪的是这里CMD类型为DM_Motor_CMD_e会报错
void DM_Motor_Command(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor ,uint8_t CMD){

	 TxFrame->Header.Identifier = DM_Motor->CANFrameInfo.CAN_ID;
  	
	 TxFrame->Data[0] = 0xFF;
   TxFrame->Data[1] = 0xFF;
 	 TxFrame->Data[2] = 0xFF;
	 TxFrame->Data[3] = 0xFF;
	 TxFrame->Data[4] = 0xFF;
	 TxFrame->Data[5] = 0xFF;
	 TxFrame->Data[6] = 0xFF;
	
	 switch(CMD){
		 
		  case Motor_Enable :
	        TxFrame->Data[7] = 0xFC; 
	    break;
      
			case Motor_Disable :
	        TxFrame->Data[7] = 0xFD; 
      break;
      
			case Motor_Save_Zero_Position :
	        TxFrame->Data[7] = 0xFE; 
			break;
			
			case DM_Motor_CMD_Type_Num :
					TxFrame->Data[7] = 0xFB;
			break;
			
			default:
	    break;   
	}
	
	// 发送
   HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);
}

// 获取系统参数
void DM_Motor_Read_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t RID){

   TxFrame->Header.Identifier = 0x7FF;
	 TxFrame->Data[0] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID);
	 TxFrame->Data[1] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID >> 8);
	 TxFrame->Data[2] = 0x33;
	 TxFrame->Data[3] = RID;
	 TxFrame->Data[4] = 0;
	 TxFrame->Data[5] = 0;
	 TxFrame->Data[6] = 0; 
	 TxFrame->Data[7] = 0;   
	 
 HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data); 
}

// 写入系统参数
void DM_Motor_Write_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t RID,uint8_t *Write_Param){

   TxFrame->Header.Identifier = 0x7FF;
	 TxFrame->Data[0] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID);
	 TxFrame->Data[1] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID >> 8);
	 TxFrame->Data[2] = 0x55;
	 TxFrame->Data[3] = RID;
	 TxFrame->Data[4] = Write_Param[0];
	 TxFrame->Data[5] = Write_Param[1];
	 TxFrame->Data[6] = Write_Param[2]; 
	 TxFrame->Data[7] = Write_Param[3];   
	 
 HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data); 
}

// 保存写入的系统参数
void DM_Motor_Save_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor){

   TxFrame->Header.Identifier = 0x7FF;
	 TxFrame->Data[0] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID);
	 TxFrame->Data[1] = (uint8_t)(DM_Motor->CANFrameInfo.CAN_ID >> 8);
	 TxFrame->Data[2] = 0xAA;
	 TxFrame->Data[3] = 1;
	 TxFrame->Data[4] = 0;
	 TxFrame->Data[5] = 0;
	 TxFrame->Data[6] = 0; 
	 TxFrame->Data[7] = 0;   
	 
 HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data); 
}

// 电机保护限幅，控制器中应该用等幅限制输出
static float DM_Motor_Limit_float(float input, float max)
{
	if(input > max)
		input = max;
	else if (input < -max)
		input = -max;
	return input;
}

// 电机控制
// param: Fdcan发送结构体，电机总结构体，电机模式，位置，速度，KP，KD，力矩
void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame, DM_Motor_Info_Typedef *DM_Motor, uint8_t Mode, float Postion, float Velocity, float KP, float KD, float Torque){

	  if(Mode > Velocity_Mode) Mode = MIT_Mode; //只有3种模式，标识默认为MIT模式
	
		// MIT模式
	  if(Mode == MIT_Mode) {
		
			 static uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
			 
			 //输出限幅：
			 Postion = DM_Motor_Limit_float(Postion, P_MAX);
			 Velocity = DM_Motor_Limit_float(Velocity, V_MAX);
			 Torque = DM_Motor_Limit_float(Torque, T_MAX);
			
			 Postion_Tmp  =  float_to_uint(Postion,-P_MAX,P_MAX,16) ;
			 Velocity_Tmp =  float_to_uint(Velocity,-V_MAX,V_MAX,12);
			 Torque_Tmp = float_to_uint(Torque,-T_MAX,T_MAX,12);
			 KP_Tmp = float_to_uint(KP,0,500,12);
			 KD_Tmp = float_to_uint(KD,0,5,12);

			 TxFrame->Header.Identifier = DM_Motor->CANFrameInfo.CAN_ID;
			 
			 TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8);
			 TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
			 TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);
			 TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (KP_Tmp>>8);
			 TxFrame->Data[4] = (uint8_t)(KP_Tmp);
			 TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);
			 TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (Torque_Tmp>>8);
			 TxFrame->Data[7] = (uint8_t)(Torque_Tmp);
			
			 HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);
	 
	 }else if(Mode == Position_Velocity_Mode){
	 
		   KP = 0; KD = 0; Torque = 0;
		 
       uint8_t *Postion_Tmp,*Velocity_Tmp;
		   
		   Postion_Tmp = (uint8_t *)&Postion; 
		   Velocity_Tmp = (uint8_t *)&Velocity; 
		 
	     TxFrame->Header.Identifier = DM_Motor->CANFrameInfo.CAN_ID + 0x100;
			 
			 TxFrame->Data[0] = *(Postion_Tmp);
			 TxFrame->Data[1] = *(Postion_Tmp + 1);
			 TxFrame->Data[2] = *(Postion_Tmp + 2);
			 TxFrame->Data[3] = *(Postion_Tmp + 3);
			 TxFrame->Data[4] = *(Velocity_Tmp);
			 TxFrame->Data[5] = *(Velocity_Tmp + 1);
			 TxFrame->Data[6] = *(Velocity_Tmp + 2);
			 TxFrame->Data[7] = *(Velocity_Tmp + 3);
			
			 HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);
	 
	 }else if(Mode == Velocity_Mode){
	 
	     Postion = 0;KP = 0; KD = 0; Torque = 0;
		 
       uint8_t *Velocity_Tmp;
		   
		   Velocity_Tmp = (uint8_t *)&Velocity; 
		 
	     TxFrame->Header.Identifier = DM_Motor->CANFrameInfo.CAN_ID + 0x200;
			 
			 TxFrame->Data[0] = *(Velocity_Tmp);
			 TxFrame->Data[1] = *(Velocity_Tmp + 1);
			 TxFrame->Data[2] = *(Velocity_Tmp + 2);
			 TxFrame->Data[3] = *(Velocity_Tmp + 3);
			 TxFrame->Data[4] = 0;
			 TxFrame->Data[5] = 0;
			 TxFrame->Data[6] = 0;
			 TxFrame->Data[7] = 0;
			
			 HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);
	 
	 }
	 
	 
}


// 电机反馈数据解码
// param: 8字节返回数据，电机总结构体地址
void DM_Motor_Info_Update(uint8_t *Data,DM_Motor_Info_Typedef *DM_Motor)
{
		DM_Motor->ID  = Data[0] & 0x0F;
	  DM_Motor->Data.State = Data[0]>>4;
		DM_Motor->Data.P_int = (uint16_t)(((Data[1]) <<8) | (Data[2]));
		DM_Motor->Data.V_int = (uint16_t)((Data[3]) <<4) | ((Data[4])>>4);
		DM_Motor->Data.T_int = (uint16_t)((Data[4]&0xF) <<8) | ((uint16_t)(Data[5]));
		DM_Motor->Data.Torque = uint_to_float(DM_Motor->Data.T_int,-T_MAX,T_MAX,12);
		DM_Motor->Data.Position = uint_to_float(DM_Motor->Data.P_int,-P_MAX,P_MAX,16);
    DM_Motor->Data.Velocity = uint_to_float(DM_Motor->Data.V_int,-V_MAX,V_MAX,12);
    DM_Motor->Data.Temperature_MOS   = (float)(Data[6]);
		DM_Motor->Data.Temperature_Rotor = (float)(Data[7]);

}






