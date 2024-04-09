#ifndef PID_H
#define PID_H
#include "main.h"

//two mode of pid controller
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA = 10
};

typedef struct
{
    uint8_t mode;
    float Kp;
    float Ki;
    float Kd;

    float max_out;  
    float max_iout; 

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //used in Delta mode
    float error[3]; 

} pid_type_def;
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data pointer
  * @param[in]      mode: PID_POSITION: normal pid
  *                       PID_DELTA: delta pid
  * @param          Kp
  * @param          Ki
  * @param          Kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, float Kp,float Ki,float Kd, float max_out, float max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
extern float PID_calc(pid_type_def *pid, float ref, float set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

float PID_calc_GM6020_angle(pid_type_def *pid, float ref, float set);

#endif
