/*
(0)前提:
依赖drv_can
(1)send_to_motors(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4,uint16_t lable,CAN_HandleTypeDef* hcan_);
发送can报文给4个电机，lable是报文ID
(2)创建RMmotor结构体
RMmotor motor;
(3)RMmotor_init(RMmotor* motor,uint16_t label,uint16_t receivedID);
(4)receive_data(uint16_t receivedID,uint8_t* data,RMmotor* motor);


//收数据
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef header;
	uint8_t data[8]; //用于接收数据
	HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO1,&header,data);
    receive_data(header.StdId,data,RMmotor* motor);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef header;
	uint8_t data[8]; //用于接收数据
	HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&header,data);
    receive_data(header.StdId,data,RMmotor* motor);
}

!!!注意！！！
GM6020电压控制——负载扭矩一定电机转速与电机电压成正比 n=C1*U-C2*T (U为电机电压,T为负载扭矩)
电压控制不能用常规方法做PID控制，因为不能直接控制扭矩
M3508和M2006——电机扭矩和电机电流成正比

*/

#ifndef RMMOTOR_H
#define RMMOTOR_H

#include "main.h"
#include "drv_can.h"

typedef struct
{
    int16_t angle;
    int16_t speed;
    int16_t torque;
    int16_t temperature;
    uint16_t lable;
    uint16_t ID;
}RMmotor;

void send_to_motors(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4,uint16_t lable,CAN_HandleTypeDef* hcan_);
void receive_data(uint16_t receivedID,uint8_t* data,RMmotor* motor);
void RMmotor_init(RMmotor* motor,uint16_t label,uint16_t receivedID);
float GM6020_speedcontrol(float target,RMmotor* motor,float C1,float C2);

#endif
