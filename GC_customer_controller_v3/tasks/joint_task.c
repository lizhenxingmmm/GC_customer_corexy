#include "joint_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "drv_can.h"
#include "RMmotor.h"
#include "pid.h"
#include "judge_system.h"

#include <math.h>
#include <stdio.h>

//matlab拟合scara作用域边缘曲线
//theta1_range = linspace(-0.26, -0.26+2.79, 300); % theta1的取值范围
//theta2_range = linspace(-pi+0.7, pi-0.7, 300); % theta2的取值范围
double poly1_param[5]={-3.87211898502825e-09,	1.49680164193646e-08,	-0.000952131361654781,	-0.000706867847757277,	438.179252629610};
double poly2_param[5]={3.130140147185706e-07,3.262060398771530e-04,0.127545147828127,21.756961942241873,1.262333839890743e+03};
double poly3_param[5]={9.10125329974849e-08	,-7.61060290839952e-05	,0.0235002401159757,	-3.22019298191863,	-98.2853696478014};
double poly4_param[5]={-6.02330231685224e-08	,5.41553239452551e-05	,-0.0195591064553661,	3.25973061635365,	-45.2812881113085};
double inner_circle_radius=155.0;

customer_data_pack data_package={1,2,3,4};

double polyval_calc(double p[5],double x);
void check_boundary(float x,float y,float res_xy[2]);
void check_boundary_rightHand(float x,float y,float res_xy[2]);
static void scara_inverse_kinematics(float x,float y,float L1,float L2,uint8_t handcoor,float angles[2]);

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
float x_limit[2]={-390.0f,405.0f};

float res_xy[2]={PI/4,PI/4};//初始机械臂关节姿态

float result_scara_angle[2]={0.0f,0.0f};
float result_yaw = 0;
float result_pitch = 0;

float derivative_angleA=0;
float derivative_angleB=0;//角度导数

float last_delta_A;
float last_delta_B;//用于计算角度导数

float debug=40.0f/8191;

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
    
//    while(initial_angle_yaw==0)
//    initial_angle_yaw = joint_motors[2].angle;
//    while(initial_angle_pitch==0)
//    initial_angle_pitch = joint_motors[3].angle;
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

corexy_y = -0.5f*(delta_A-delta_B)*3.2;
corexy_x = -0.5f*(delta_A+delta_B)*5.2;

if(corexy_x<x_limit[0])
{
    corexy_x=x_limit[0];
}
if(corexy_x>x_limit[1])
{
    corexy_x=x_limit[1];
}
check_boundary_rightHand(corexy_x,corexy_y,res_xy);
//-109.86,426.085
        scara_inverse_kinematics(res_xy[0],res_xy[1],ARMLENGHT1,ARMLENGHT1,1,result_scara_angle);
          //scara_inverse_kinematics(220,220,ARMLENGHT1,ARMLENGHT1,1,result_scara_angle);
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

        if(derivative_angleA>0.03)
        {target_current_A=500;}
        else if(derivative_angleA<-0.03)
        {target_current_A=-500;}
        else
        {target_current_A=0;}
        if(derivative_angleB>0.03)
        {target_current_B=500;}
        else if(derivative_angleB<-0.03)
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

/**
 * @brief two dimensional scara
 * @param handcoor 1:right hand 2:left hand
 * @param angles rad 0~2pi
*/
void scara_inverse_kinematics(float x,float y,float L1,float L2,uint8_t handcoor,float angles[2])
{
    if(pow(x,2)+pow(y,2)>pow(L1+L2,2))
    {
        x=(L1+L2)*cos(atan2(y,x));
        y=(L1+L2)*sin(atan2(y,x));
    }
    float cos_beta = (pow(x,2)+pow(y,2)-pow(L1,2)-pow(L2,2))/(2*L1*L2);
    float sin_beta = 0.0f;
    float temp = 1-pow(cos_beta,2);
    float calc_error = 0.1f;
    if(temp<0)
    {
        if(temp>-calc_error)
        {
            temp = 0;
        }
        else
        {
            debug = -1;
            return;
        }
    }
    if(handcoor==1)
    {
        sin_beta = sqrt(temp);
    }
    else if(handcoor==2)
    {
        sin_beta = -sqrt(temp);
    }
    else
    {
        debug=-2;
    }
    angles[1] = atan2(sin_beta, cos_beta);
    angles[0] = atan2(y, x) - atan2(L2 * sin_beta, L1 + L2 * cos_beta);
    // if(angles[1]<0)
    // {
    //     angles[1]+=2*3.14159f;
    // }
    // if(angles[0]<0)
    // {
    //     angles[0]+=2*3.14159f;
    // }
}
/**
 * @brief 越界检查并转化到最短边界点
 * @param res_xy 输出的结果xy坐标，第一个为x，第二个为y
*/
void check_boundary(float x,float y,float res_xy[2])
{
    double temp_angle = atan2(y,x);
    double polyval_p1 = polyval_calc(poly1_param,x);
    double polyval_p2 = polyval_calc(poly2_param,x);
    double polyval_p3 = polyval_calc(poly3_param,x);
    double slope = y/x;
    debug = polyval_p1;
    if(pow(x,2)+pow(y,2)<pow(inner_circle_radius,2))
    {    
        res_xy[0] = inner_circle_radius*cos(temp_angle);
        res_xy[1] = inner_circle_radius*sin(temp_angle);
    }
    else if((polyval_p1<y)&&(x>-390)&&(x<405))
    {
        double temp_delta_x=(y-polyval_p1)/slope;
        double target_boundary_x=x-temp_delta_x;
        double bound_x1 = target_boundary_x;
        double bound_x2 = x;
        double error_set=0.1;
        //二分法
        for(int i=0;i<100;i++)
        {
            // if((bound_x1*slope)<polyval_calc(poly1_param,bound_x1) &&
            // (bound_x2*slope)>polyval_calc(poly1_param,bound_x2))
            if(fabs(((bound_x1+bound_x2)*0.5*slope)-polyval_calc(poly1_param,(bound_x1+bound_x2)*0.5))<error_set)
            {
                target_boundary_x = (bound_x1+bound_x2)*0.5;
                break;
            }
            if(((bound_x1+bound_x2)*0.5*slope)<polyval_calc(poly1_param,(bound_x1+bound_x2)*0.5))
            {
                bound_x1 = (bound_x1+bound_x2)*0.5;
            }
            else
            {
                bound_x2 = (bound_x1+bound_x2)*0.5;
            }
        }
        res_xy[0]=target_boundary_x;
        res_xy[1]=target_boundary_x*slope;
    }
    else if((x>-390)&&(x<-140)&&(polyval_p2>y))
    {
        res_xy[0]=x;
        res_xy[1]=polyval_p2;
    }
    else if((x>17)&&(x<405)&&(polyval_p3>y))
    {
        res_xy[0]=x;
        res_xy[1]=polyval_p3;
    }
    else if((x>=-140)&&(x<=17))
    {
        res_xy[0]=x;
        res_xy[1]=sqrt(pow(inner_circle_radius,2)-pow(x,2));
    }
    else
    {
        res_xy[0]=x;
        res_xy[1]=y;
    }
}

double polyval_calc(double p[5],double x)
{
    double y=0;
    for(int i=0;i<5;i++)
    {
        y+=pow(x,4-i)*p[i];
    }
    return y;
}

/**
 * @brief 越界检查并转化到最短边界点(右手系)
 * @param res_xy 输出的结果xy坐标，第一个为x，第二个为y
*/
void check_boundary_rightHand(float x,float y,float res_xy[2])
{
    double temp_angle = atan2(y,x);
    double polyval_p1 = polyval_calc(poly1_param,x);
    double polyval_p2 = polyval_calc(poly2_param,x);
    double polyval_p4 = polyval_calc(poly4_param,x);
    double slope = y/x;
    if(pow(x,2)+pow(y,2)<pow(inner_circle_radius,2))
    {    
        // res_xy[0] = inner_circle_radius*cos(temp_angle);
        // res_xy[1] = inner_circle_radius*sin(temp_angle);
        res_xy[0] = x;
        res_xy[1] =sqrt(pow(inner_circle_radius,2)-pow(x,2));
    }
    else if((polyval_p1<y)&&(x>-390)&&(x<405))
    {
        double temp_delta_x=(y-polyval_p1)/slope;
        double target_boundary_x=x-temp_delta_x;
        double bound_x1 = target_boundary_x;
        double bound_x2 = x;
        double error_set=0.1;
        //二分法
        for(int i=0;i<100;i++)
        {
            // if((bound_x1*slope)<polyval_calc(poly1_param,bound_x1) &&
            // (bound_x2*slope)>polyval_calc(poly1_param,bound_x2))
            if(fabs(((bound_x1+bound_x2)*0.5*slope)-polyval_calc(poly1_param,(bound_x1+bound_x2)*0.5))<error_set)
            {
                target_boundary_x = (bound_x1+bound_x2)*0.5;
                break;
            }
            if(((bound_x1+bound_x2)*0.5*slope)<polyval_calc(poly1_param,(bound_x1+bound_x2)*0.5))
            {
                bound_x1 = (bound_x1+bound_x2)*0.5;
            }
            else
            {
                bound_x2 = (bound_x1+bound_x2)*0.5;
            }
        }
        res_xy[0]=target_boundary_x;
        res_xy[1]=target_boundary_x*slope;
    }
    else if((x>-390)&&(x<-140)&&(polyval_p2>y))
    {
        res_xy[0]=x;
        res_xy[1]=polyval_p2;
    }
    else if((x>=-140)&&(x<=86)&&(y<sqrt(pow(inner_circle_radius,2)-pow(x,2))))
    {
        res_xy[0]=x;
        res_xy[1]=sqrt(pow(inner_circle_radius,2)-pow(x,2));
    }
    else if((x>86)&&(x<405)&&(polyval_p4>y))
    {
        res_xy[0]=x;
        res_xy[1]=polyval_p4;
    }
    else
    {
        res_xy[0]=x;
        res_xy[1]=y;
    }
}
