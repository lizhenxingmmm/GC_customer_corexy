#include "joint_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "drv_can.h"
#include "RMmotor.h"
#include "pid.h"
#include "judge_system.h"
#include "scara_kinematics.h"
#include <math.h>
#include <stdio.h>

customer_data_pack data_package={1,2,3,4};
slightly_controll_data slightly_controll_data_1;

RMmotor joint_motors[4]; //joint_motors[0]是corexy的A电机 joint_motors[1]是corexy的B电机
int16_t pre_angle[2];    //0是A电机 1是B电机 
pid_type_def joint_motors_speed[4];
pid_type_def joint_motors_angle[4];

 float delta_A=0;
 float delta_B=0;
 float delta_yaw=0;
 float delta_pitch=0;
static float circle_unit = 40;//unit:mm 单位:毫米
static float step_unit = 40.0f/8191;
static int16_t last_angle_A;
static int16_t current_angle_A;
static int16_t last_angle_B;
static int16_t current_angle_B;

static int16_t last_angle_yaw;
static int16_t current_angle_yaw;
static int16_t last_angle_pitch;
static int16_t current_angle_pitch;

 int16_t initial_angle_A;
 int16_t initial_angle_B;
 int16_t initial_angle_yaw;
 int16_t initial_angle_pitch;
 int16_t circle_flag_A=0;
 int16_t circle_flag_B=0;
 int16_t circle_flag_yaw=0;
 int16_t circle_flag_pitch=0;
float corexy_x=0; //启动处为原点
float corexy_y=0;
float res_xy[2]={PI/4,PI/4};//初始机械臂关节姿态
float result_scara_angle[2]={0.0f,0.0f};
float result_yaw = 0;
float result_pitch = 0;
float derivative_angleA=0;
float derivative_angleB=0;//角度导数
float last_delta_A;
float last_delta_B;//用于计算角度导数


void joint_task(void const * argument)
{
    CAN_Init(&hcan);
    for(int i=0;i<4;i++)
    {
        RMmotor_init(&(joint_motors[i]),0x200,0x201+i);
    }
    initial_angle_A = joint_motors[0].angle;
    initial_angle_B = joint_motors[1].angle;
    initial_angle_yaw = joint_motors[2].angle;
    initial_angle_pitch = joint_motors[3].angle;
    //直到收到角度数据，初始化初始角度
    while(initial_angle_A==0)
    {
        initial_angle_A = joint_motors[0].angle;
		osDelay(100);	
    }
    while(initial_angle_B==0)
    {
        initial_angle_B = joint_motors[1].angle;
        osDelay(100);	
    }
    while(initial_angle_yaw==0)
    {
        initial_angle_yaw = joint_motors[2].angle;
        osDelay(100);	
    }
    while(initial_angle_pitch==0)
    {
        initial_angle_pitch = joint_motors[3].angle;
        osDelay(100);	
    }
    last_angle_A = initial_angle_A;
    last_angle_B = initial_angle_B;
	last_angle_yaw = initial_angle_yaw;
    last_angle_pitch = initial_angle_pitch;
		
    for(;;)
    {///////////////////////////////////////////////////////////
        current_angle_A = joint_motors[0].angle;
        if(joint_motors[0].speed>0)
        {
            if(current_angle_A-last_angle_A<-4000)
            {
                circle_flag_A++;
            }
        }
        else
        {
            if(current_angle_A-last_angle_A>4000)
            {
                circle_flag_A--;
            }
        }
        delta_A = circle_flag_A*circle_unit+(current_angle_A-initial_angle_A)*step_unit;
        last_angle_A = current_angle_A;
///////////////////////////////////////////////////////////////////////////////////////////
        current_angle_B = joint_motors[1].angle;
         if(joint_motors[1].speed>0)
        {
            if(current_angle_B-last_angle_B<-4000)
            {
                circle_flag_B++;
            }
        }
        else
        {
            if(current_angle_B-last_angle_B>4000)
            {
                circle_flag_B--;
            }
        }
        delta_B = circle_flag_B*circle_unit+(current_angle_B-initial_angle_B)*step_unit;
        last_angle_B = current_angle_B;
//////////////////////////////////////////////////////////////////////////////////////////////////
        current_angle_yaw = joint_motors[2].angle;
         if(joint_motors[2].speed>0)
        {
            if(current_angle_yaw-last_angle_yaw<-4000)
            {
                circle_flag_yaw++;
            }
        }
        else
        {
            if(current_angle_yaw-last_angle_yaw>4000)
            {
                circle_flag_yaw--;
            }
        }
        result_yaw=-((current_angle_yaw-initial_angle_yaw)+circle_flag_yaw*8191)*2*PI/8191;
        delta_yaw = circle_flag_yaw*circle_unit+(current_angle_yaw-initial_angle_yaw)*step_unit;
        last_angle_yaw = current_angle_yaw;
////////////////////////////////////////////////////////////////////////////////////////////////////
    current_angle_pitch = joint_motors[3].angle;
         if(joint_motors[3].speed>0)
        {
            if(current_angle_pitch-last_angle_pitch<-4000)
            {
                circle_flag_pitch++;
            }
        }
        else
        {
            if(current_angle_pitch-last_angle_pitch>4000)
            {
                circle_flag_pitch--;
            }
        }
        result_pitch=((current_angle_pitch-initial_angle_pitch)+circle_flag_pitch*8191)*2*PI/8191;
        delta_pitch = circle_flag_pitch*circle_unit+(current_angle_pitch-initial_angle_pitch)*step_unit;
        last_angle_pitch = current_angle_pitch;
//////////////////////////////////////////////////////////////////////////////////////////////////////////

corexy_y = -0.5f*(delta_A-delta_B)*2.5;
corexy_x = -0.5f*(delta_A+delta_B)*5.2;

//暂时用于调试的微调模式数据
slightly_controll_data_1.delta_x=corexy_x/10;
slightly_controll_data_1.delta_y=corexy_y/10;
slightly_controll_data_1.delta_yaw=result_yaw;
slightly_controll_data_1.delta_pitch=result_pitch;

check_boundary_scara(corexy_x,corexy_y,res_xy);

        scara_inverse_kinematics(res_xy[0],res_xy[1],ARMLENGHT1,ARMLENGHT1,1,result_scara_angle);
          data_package.angle1=result_scara_angle[0];
          data_package.angle2=result_scara_angle[1];
          data_package.angle3=result_yaw;
          data_package.angle4=result_pitch;

        derivative_angleA=delta_A-last_delta_A;
        derivative_angleB=delta_B-last_delta_B;
        last_delta_A=delta_A;
        last_delta_B=delta_B;

        int16_t target_current_A;
        int16_t target_current_B;
//补偿摩擦力
        if(derivative_angleA>0.04)
        {target_current_A=500;}
        else if(derivative_angleA<-0.04)
        {target_current_A=-500;}
        else
        {target_current_A=0;}
        if(derivative_angleB>0.04)
        {target_current_B=500;}
        else if(derivative_angleB<-0.04)
        {target_current_B=-500;}
        else
        {target_current_B=0;}

        send_to_motors(target_current_A,target_current_B,0,0,0x200,&hcan);
        osDelay(2);

    }
    
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef header;
	uint8_t data[8]; //用于接收数据
	HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO1,&header,data);
    for(int i=0;i<4;i++)
    {
        receive_data(header.StdId,data,&joint_motors[i]);
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef header;
	uint8_t data[8]; //用于接收数据
	HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&header,data);
    for(int i=0;i<4;i++)
    {
        receive_data(header.StdId,data,&joint_motors[i]);
    }
}

