/*
使用说明:
(0) 前提:
(大疆C板)
配置CAN通信:波特率1000000
CAN1_RX PD0
CAN1_TX PD1
CAN2_RX PB5
CAN2_TX PB6
(1) CAN_Init(CAN_HandleTypeDef *hcan);
初始化
(2) CAN_SEND_Data(CAN_HandleTypeDef *hcan,uint16_t ID,uint8_t *DATA,uint16_t Length);
发信息
*/
/*
fifo1和fifo0两个中断回调函数的模板，data用于接收数据
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef header;
	uint8_t data[8]; //用于接收数据(不知道为什么，这里不能是uint8_t* data;)(知道了，只定义指针没有开辟连续的数组空间)
	HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO1,&header,data);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef header;
	uint8_t data[8]; //用于接收数据(不知道为什么，这里不能是uint8_t* data;)(知道了，只定义指针没有开辟连续的数组空间)
	HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&header,data);
}
*/
#ifndef MY_CAN_H_
#define MY_CAN_H_

#include "main.h"
#include "can.h"

#define CAN_FILTER(x)((x)<<3)
#define CAN_FIFO_0 (0<<2)
#define CAN_FIFO_1 (1<<2)
#define CAN_STDID (0<<1)
#define CAN_EXTID (1<<1)
#define CAN_DATA_TYPE (0<<0)
#define CAN_REMOTE_TYPE (1<<0)



void CAN_Init(CAN_HandleTypeDef *hcan);
uint8_t CAN_SEND_Data(CAN_HandleTypeDef *hcan,uint16_t ID,uint8_t *DATA,uint16_t Length);
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan,uint8_t object_param,uint32_t ID,uint32_t mask_ID);

#endif

