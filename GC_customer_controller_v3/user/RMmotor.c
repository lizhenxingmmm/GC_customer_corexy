#include "RMmotor.h"

/**
 * @brief 发送can报文给4个电机(大疆RM电机的控制)
 * @param motor1 第一个电机要输出的电流或电压(m3508电流，gm6020电压)
 * @param motor2 第二个电机要输出的电流或电压(m3508电流，gm6020电压)
 * @param motor3 第三个电机要输出的电流或电压(m3508电流，gm6020电压)
 * @param motor4 第四个电机要输出的电流或电压(m3508电流，gm6020电压)
 * @param lable 报文ID或者说标识符
 * @param hcan_ 选择的can类型，这里是句柄类型的指针
*/
void send_to_motors(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4,uint16_t lable,CAN_HandleTypeDef* hcan_)
{
        uint8_t data[8];
        data[0]=(motor1>>8);
        data[1]=motor1;
        data[2]=(motor2>>8);
        data[3]=motor2;
        data[4]=(motor3>>8);
        data[5]=motor3;
        data[6]=(motor4>>8);
        data[7]=motor4;
        CAN_SEND_Data(hcan_,lable,data,8);
}
float GM6020_speedcontrol(float target,RMmotor* motor,float C1,float C2)
{
    return (target*C1+motor->torque*C2);
}
/**
 * @brief 接收can数据到该RMmotor结构体中，直接放在can的中断回调函数中使用
 * @param receivedID 该报文的ID
 * @param data 该报文数据的指针
*/
void receive_data(uint16_t receivedID,uint8_t* data,RMmotor* motor){
    if(receivedID==motor->ID){
        motor->angle=(((int16_t)data[0])<<8)+data[1];
        motor->speed=(int16_t)((((int16_t)data[2])<<8)+data[3]);
        motor->torque=(int16_t)((((int16_t)data[4])<<8)+data[5]);
        motor->temperature=data[7];
    }
}
/**
 * @brief 初始化RMmotor结构体
 * @param label 电机说明书上说的标识符
 * @param receivedID 电机发送的报文的ID
*/
void RMmotor_init(RMmotor* motor,uint16_t label,uint16_t receivedID)
{
    motor->angle=0;
    motor->speed=0;
    motor->torque=0;
    motor->temperature=0; 
    motor->lable=label;
    motor->ID=receivedID;  
}
