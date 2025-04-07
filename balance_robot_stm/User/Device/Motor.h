#ifndef Motor_H
#define Motor_H

#include "bsp_can.h"
#include "stdbool.h"

#define P_MIN_4310 -12.5f
#define P_MAX_4310 12.5f
#define V_MIN_4310 -30.0f
#define V_MAX_4310 30.0f
#define KP_MIN_4310 0.0f
#define KP_MAX_4310 500.0f
#define KD_MIN_4310 0.0f
#define KD_MAX_4310 5.0f
#define T_MIN_4310 -10.0f
#define T_MAX_4310 10.0f

#define P_MIN_6215 -12.0f
#define P_MAX_6215 12.0f
#define V_MIN_6215 -45.0f
#define V_MAX_6215 45.0f
#define KP_MIN_6215 0.0f
#define KP_MAX_6215 500.0f
#define KD_MIN_6215 0.0f
#define KD_MAX_6215 5.0f
#define T_MIN_6215 -18.0f
#define T_MAX_6215 18.0f

//枚举4种特殊控制帧
typedef enum{

  Motor_Enable, //使能
  Motor_Disable, //失能
  Motor_Save_Zero_Position, //保存零点
  DM_Motor_CMD_Type_Num, //清除错误

}DM_Motor_CMD_e;

//枚举4种控制模式
typedef enum{

  MIT_Mode, //MIT模式
  Position_Velocity_Mode, //位置速度模式
  Velocity_Mode, //速度模式
  DM_Motor_Mode_Type_Num, //混合模式

}DM_Motor_Mode_e;

//电机反馈帧结构体
typedef struct 
{
  int16_t  State; 	
  uint16_t  P_int;
  uint16_t  V_int;
  uint16_t  T_int;
  float  Position;  
  float  Velocity;  
  float  Torque;  
  float  Temperature_MOS;   
  float  Temperature_Rotor;  
}DM_Motor_Data_Typedef;

//ID结构体
typedef struct
{
  uint32_t Master_ID; //接收电机发送来的信号的ID标识符，是电机向外发送时的ID号
  uint32_t CAN_ID; //向电机发送时的标识符，是电机接收外界的ID号
}Motor_CANFrameInfo_typedef;

//电机控制结构体
typedef struct
{
	float  KP;
	float  KD;
	float  Position; 
  float  Velocity;  	
  float  Torque;  
	
}DM_Motor_Control_Typedef;

//电机总结构体
typedef struct
{
	uint16_t ID; //不知道这啥ID
  Motor_CANFrameInfo_typedef CANFrameInfo; //电机发送ID和接收ID
	DM_Motor_Data_Typedef Data; //反馈数据
	DM_Motor_Control_Typedef Control; //输出力矩
}DM_Motor_Info_Typedef;




extern DM_Motor_Info_Typedef DM_4310_Motor_leftfront;
extern DM_Motor_Info_Typedef DM_4310_Motor_leftback;
extern DM_Motor_Info_Typedef DM_6215_Motor_left;
extern DM_Motor_Info_Typedef DM_4310_Motor_rightfront;
extern DM_Motor_Info_Typedef DM_4310_Motor_rightback;
extern DM_Motor_Info_Typedef DM_6215_Motor_right;

extern DM_Motor_Control_Typedef DM_Motor_Control;

extern void DM_Motor_Info_Update_4310(uint8_t *rxBuf,DM_Motor_Info_Typedef *DM_Motor);

extern void DM_Motor_Info_Update_6215(uint8_t *rxBuf,DM_Motor_Info_Typedef *DM_Motor);

extern void DM_Motor_Command(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor ,uint8_t CMD);

extern void DM_Motor_Read_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t RID);

extern void DM_Motor_Write_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t RID,uint8_t *Write_Param);
	
extern void DM_Motor_Save_Param(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor);

extern void DM_Motor_CAN_TxMessage_4310(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t Mode,
	                                             float Postion, float Velocity, float KP, float KD, float Torque);
extern void DM_Motor_CAN_TxMessage_6215(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t Mode,
	                                             float Postion, float Velocity, float KP, float KD, float Torque);
#endif