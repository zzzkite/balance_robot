#include "fdcan.h"
#include "bsp_can.h"
#include "Motor.h"

FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;

FDCAN_TxFrame_TypeDef FDCAN1_TxFrame = {
  .hcan = &hfdcan1, //关联Fdcan1
  .Header.IdType = FDCAN_STANDARD_ID, //标准ID 11位 
  .Header.TxFrameType = FDCAN_DATA_FRAME, //发送数据帧，而不是远程帧
  .Header.DataLength = 8, //数据长度8字节
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE, //错误状态启动，一般设置为FDCAN_ESI_ACTIVE
  .Header.BitRateSwitch = FDCAN_BRS_OFF, //数据帧和仲裁帧使用相同波特率，适用于经典Can
  .Header.FDFormat =  FDCAN_CLASSIC_CAN, //用于经典Can
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS, //不记录发送事件
  .Header.MessageMarker = 0, //消息标志
};

void BSP_FDCAN1_Init(void){

  FDCAN_FilterTypeDef FDCAN1_FilterConfig; //库函数结构体
	
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;//过滤标准ID，经典can只有标准ID
  FDCAN1_FilterConfig.FilterIndex = 0;           //过滤器编号
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK; //过滤器Mask模式
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//选择哪个FIFO接收
  FDCAN1_FilterConfig.FilterID1 = 0x00000000; // 这个都行，只要ID2配置0x00000000就不会滤掉任何ID
  FDCAN1_FilterConfig.FilterID2 = 0x00000000; 
  
  HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig); //将上述配置配置到can1
		
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); //开启过滤器（全局过滤）
 
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//打开FIFO0的中断接收，默认弱定义回调函数为：HAL_FDCAN_RxFifo0Callback
  
  HAL_FDCAN_Start(&hfdcan1);//使能Can1
	
}

void BSP_FDCAN2_Init(void){

  FDCAN_FilterTypeDef FDCAN2_FilterConfig; //库函数结构体
	
	FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;//过滤标准ID，经典can只有标准ID
  FDCAN2_FilterConfig.FilterIndex = 0;           //过滤器编号
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK; //过滤器Mask模式
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//选择哪个FIFO接收
  FDCAN2_FilterConfig.FilterID1 = 0x00000000; // 这个都行，只要ID2配置0x00000000就不会滤掉任何ID
  FDCAN2_FilterConfig.FilterID2 = 0x00000000; 
  
  HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig); //将上述配置配置到can2
		
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); //开启过滤器（全局过滤）
 
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//打开FIFO0的中断接收，默认弱定义回调函数为：HAL_FDCAN_RxFifo0Callback
  
  HAL_FDCAN_Start(&hfdcan2);//使能Can1
 	
	
}


static void FDCAN1_RxFifo0RxHandler(uint32_t *Master_ID,uint8_t Data[8])
{
   
  DM_Motor_Info_Update(Data,&DM_6220_Motor);

}

// 回调函数FIFO0
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  //从FIFO中获取数据
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxFrame.Header, FDCAN1_RxFrame.Data);
	//解析数据
  FDCAN1_RxFifo0RxHandler(&FDCAN1_RxFrame.Header.Identifier,FDCAN1_RxFrame.Data);
	 
}
	
// 回调函数FIFO1
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  //从FIFO中获取数据
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxFrame.Header, FDCAN1_RxFrame.Data);
	//解析数据
  FDCAN1_RxFifo0RxHandler(&FDCAN1_RxFrame.Header.Identifier,FDCAN1_RxFrame.Data);
	 
}
	
	
	
