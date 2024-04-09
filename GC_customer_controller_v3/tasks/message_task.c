#include "message_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "RMmotor.h"
#include "NRF24L01.h"





void message_task(void const * argument)
{
//    gpio_init_for_this();
//    Init_NRF24L01();
//    if(NRF24L01_Check())
//	{
//		
//	}
//	TX_Mode();
    //HAL_ADC_Start_DMA(&hadc1,&analog_data,1);
    for(;;)
    {
//        if(HAL_GPIO_ReadPin(BUTTON_PORT,BUTTON1_PIN) == GPIO_PIN_RESET)
//        {
//            button1_flag=1;
//        }
//        else
//        {
//            button1_flag=0;
//        }
//        if(HAL_GPIO_ReadPin(BUTTON_PORT,BUTTON2_PIN) == GPIO_PIN_RESET)
//        {
//            button2_flag=1;
//        }
//        else
//        {
//            button2_flag=0;
//        }
//        memcpy((void*)(&TxData[1]),(const void*)(&(joint_motors[0].angle)),2);
//        memcpy((void*)(&TxData[3]),(const void*)(&(joint_motors[1].angle)),2);
//        memcpy((void*)(&TxData[5]),(const void*)(&(joint_motors[2].angle)),2);
//        memcpy((void*)(&TxData[7]),(const void*)(&(joint_motors[3].angle)),2);
    //    if(NRF24L01_TxPacket(TxData)==TX_OK)		
	// 	{
			
	// 	}
		osDelay(10);
    }
}


void gpio_init_for_this(void)
{
    GPIO_InitTypeDef this_gpio={0};
    this_gpio.Pin = BUTTON1_PIN | BUTTON2_PIN | STICKto2_PIN;
    this_gpio.Mode = GPIO_MODE_INPUT;
    this_gpio.Pull = GPIO_PULLUP;
    this_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUTTON_PORT,&this_gpio);

    GPIO_InitTypeDef this_gpio_2={0};
    this_gpio_2.Pin = STICKto1_PIN;
    this_gpio_2.Mode = GPIO_MODE_INPUT;
    this_gpio_2.Pull = GPIO_PULLUP;
    this_gpio_2.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STICKto1_PORT,&this_gpio_2);

}

