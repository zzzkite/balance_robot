#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32h7xx.h"


extern void BSP_FDCAN1_Init(void);
extern void BSP_FDCAN2_Init(void);

// 发送结构体
typedef struct {
		FDCAN_HandleTypeDef *hcan; //关联fdcan
    FDCAN_TxHeaderTypeDef Header; //库函数结构体，包含can发送的很多信息
    uint8_t				Data[8]; //发送的数据
}FDCAN_TxFrame_TypeDef;

// 接收结构体
typedef struct {
		FDCAN_HandleTypeDef *hcan; //同上
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			Data[8];
} FDCAN_RxFrame_TypeDef;

extern  FDCAN_TxFrame_TypeDef   FDCAN1_TxFrame;
	   
#endif