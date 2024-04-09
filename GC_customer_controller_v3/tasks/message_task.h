#ifndef MESSAGE_TASK_H
#define MESSAGE_TASK_H

#include "gpio.h"

#define     BUTTON1_PIN     GPIO_PIN_1
#define     BUTTON2_PIN     GPIO_PIN_2
#define     BUTTON_PORT     GPIOB

#define     STICKto1_PIN    GPIO_PIN_7
#define     STICKto1_PORT   GPIOA
#define     STICKto2_PIN    GPIO_PIN_2
#define     STICKto2_PORT   GPIOB

void gpio_init_for_this(void);

#endif
