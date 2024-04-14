#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "senddatatask.h"
#include "string.h"
#include "judge_system.h"
#include "message_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"
#include "NRF24L01.h"
#include <string.h>
#include "scara_kinematics.h"
uint8_t TxData[32] = {0,0,0,0};

uint32_t analog_data=0;
uint8_t button1_flag=0;
uint8_t button2_flag=0;

float debug2=0;
float debug3=0;
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth);
extern customer_data_pack data_package;
extern slightly_controll_data slightly_controll_data_1;
Controller_t tx_data; 

void StartSendDataTask(void const *argument)
{
       Init_NRF24L01();
   if(NRF24L01_Check())
	{
		
	}
	TX_Mode();
    gpio_init_for_this();
    HAL_ADC_Start_DMA(&hadc1,&analog_data,1);
    // uint8_t index = 0;
    uint32_t wait_time = xTaskGetTickCount();
    for (;;)
    {
        if(HAL_GPIO_ReadPin(BUTTON_PORT,BUTTON1_PIN) == GPIO_PIN_RESET)
       {
           button1_flag=1;
       }
       else
       {
           button1_flag=0;
       }
       if(HAL_GPIO_ReadPin(BUTTON_PORT,BUTTON2_PIN) == GPIO_PIN_RESET)
       {
           button2_flag=1;
       }
       else
       {
           button2_flag=0;
       }
       TxData[0] = 0xa5;
        memcpy((void*)(&TxData[1]),(const void*)(&data_package.angle1),4);
        memcpy((void*)(&TxData[5]),(const void*)(&data_package.angle2),4);
        memcpy((void*)(&TxData[9]),(const void*)(&data_package.angle3),4);
        memcpy((void*)(&TxData[13]),(const void*)(&data_package.angle4),4);
        debug3=sizeof(customer_data_pack);
 if(NRF24L01_TxPacket(TxData)==TX_OK)		
		{
			debug2=666;
		}

        uint8_t data[DATA_LENGTH] = {0x40};
        data_package.angle1=data_package.angle1-0.96;
        data_package.angle2=data_package.angle2+0.43;
        data_package.angle3=data_package.angle3+0.03;
        data_package.angle4=data_package.angle4+0.66;//和工程实车编码值对应
//0x1a:全控制模式
//0x2a:微调模式
data[0]=0x2a;
if(data[0]==0x1a)
{
    memcpy((void*)(&data[1]),(const void*)(&data_package.angle1),4);
        memcpy((void*)(&data[5]),(const void*)(&data_package.angle2),4);
        memcpy((void*)(&data[9]),(const void*)(&data_package.angle3),4);
        memcpy((void*)(&data[13]),(const void*)(&data_package.angle4),4);
}
if(data[0]==0x2a)
{
    memcpy((void*)(&data[1]),(const void*)(&slightly_controll_data_1),sizeof(slightly_controll_data));
}

        debug3=sizeof(slightly_controll_data);
        //memcpy((void*)&data[0],(const void*)&data_package,sizeof(customer_data_pack));
        Data_Concatenation(data, DATA_LENGTH);
        HAL_UART_Transmit(&huart3, (uint8_t *)(&tx_data), sizeof(tx_data), 50);
        //osDelayUntil(&wait_time, 500);
				osDelay(10);
    }
}

static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    
    tx_data.frame_header.sof = 0xA5;                              
    tx_data.frame_header.data_length = data_lenth;                
    tx_data.frame_header.seq = seq++;                             
    append_CRC8_check_sum((uint8_t *)(&tx_data.frame_header), 5); 
    
    tx_data.cmd_id = CONTROLLER_CMD_ID;
    
    memcpy(tx_data.data, data, data_lenth);
    
    append_CRC16_check_sum((uint8_t *)(&tx_data), DATA_FRAME_LENGTH);
}
