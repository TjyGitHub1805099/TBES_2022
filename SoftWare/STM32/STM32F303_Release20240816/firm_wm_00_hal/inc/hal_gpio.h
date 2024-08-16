/**
  *******************************************************************
  * @file    hal_gpio.h
  * @author  MKdriver
  * @version V1.0.0
  * @date    9-Oct-2015
  * @brief   GPIO头文件(抽象层)
  *******************************************************************
  * @attention
  *
  * 
  *
  * <h2><center>&copy; COPYRIGHT 2015 MKdriver</center></h2>
  *******************************************************************
  */
#ifndef __HAL_GPIO_H__
#define __HAL_GPIO_H__

#include "drv_gpio.h"


#define MOTOR_PULSE_CTR_BY_PWM 		(1)//1:用timer做PWM输出 0:普通IO口

//=============================================DO 配置 (uodate to :2022.04.30)
//LED_RUN
#define DO0_GPIO_PORT				GPIOA
#define DO0_GPIO_CLK				RCC_AHBENR_GPIOAEN
#define DO0_GPIO_PIN				GPIO_PIN_4

//ZGM0_RST
#define DO1_GPIO_PORT				GPIOA
#define DO1_GPIO_CLK				RCC_AHBENR_GPIOAEN
#define DO1_GPIO_PIN				GPIO_PIN_7

//STM32_Motor_En
#define DO2_GPIO_PORT				GPIOB
#define DO2_GPIO_CLK				RCC_AHBENR_GPIOBEN
#define DO2_GPIO_PIN				GPIO_PIN_12

//STM32_Motor_Dir
#define DO3_GPIO_PORT				GPIOB
#define DO3_GPIO_CLK				RCC_AHBENR_GPIOBEN
#define DO3_GPIO_PIN				GPIO_PIN_13

//STM32_Dcf_Ctrl 虽然连的PB15 ，但是PB15是MOTER的PWM
#define DO4_GPIO_PORT				GPIOB
#define DO4_GPIO_CLK				RCC_AHBENR_GPIOBEN
#define DO4_GPIO_PIN				GPIO_PIN_14

//M_HX711_CLK
#define DO5_GPIO_PORT				GPIOA
#define DO5_GPIO_CLK				RCC_AHBENR_GPIOAEN
#define DO5_GPIO_PIN				GPIO_PIN_11

//STM32_I2C1_WP
#define DO6_GPIO_PORT				GPIOB
#define DO6_GPIO_CLK				RCC_AHBENR_GPIOBEN
#define DO6_GPIO_PIN				GPIO_PIN_7

#if(0 == MOTOR_PULSE_CTR_BY_PWM)
//STM32_Motor_Pwm 原理图连的是PB14 但是实际控制输出是PB15
#define DO7_GPIO_PORT				GPIOB
#define DO7_GPIO_CLK				RCC_AHBENR_GPIOBEN
#define DO7_GPIO_PIN				GPIO_PIN_15
#endif
//=============================================DI 配置 (uodate to :2022.04.30)
//STM32_POS_1
#define DI2_GPIO_PORT				GPIOC
#define DI2_GPIO_CLK				RCC_AHBENR_GPIOCEN
#define DI2_GPIO_PIN				GPIO_PIN_8

//STM32_POS_2
#define DI0_GPIO_PORT				GPIOC
#define DI0_GPIO_CLK				RCC_AHBENR_GPIOCEN
#define DI0_GPIO_PIN				GPIO_PIN_6

//STM32_POS_3
#define DI1_GPIO_PORT				GPIOC
#define DI1_GPIO_CLK				RCC_AHBENR_GPIOCEN
#define DI1_GPIO_PIN				GPIO_PIN_7

//STM32_KEY_1
#define DI3_GPIO_PORT				GPIOC
#define DI3_GPIO_CLK				RCC_AHBENR_GPIOCEN
#define DI3_GPIO_PIN				GPIO_PIN_9

//STM32_KEY_2
#define DI4_GPIO_PORT				GPIOA
#define DI4_GPIO_CLK				RCC_AHBENR_GPIOAEN
#define DI4_GPIO_PIN				GPIO_PIN_8

//STM32_KEY_3
#define DI5_GPIO_PORT				GPIOA
#define DI5_GPIO_CLK				RCC_AHBENR_GPIOAEN
#define DI5_GPIO_PIN				GPIO_PIN_9

//STM32_KEY_4
#define DI6_GPIO_PORT				GPIOA
#define DI6_GPIO_CLK				RCC_AHBENR_GPIOAEN
#define DI6_GPIO_PIN				GPIO_PIN_10

//M_HX711_DATA
#define DI7_GPIO_PORT				GPIOA
#define DI7_GPIO_CLK				RCC_AHBENR_GPIOAEN
#define DI7_GPIO_PIN				GPIO_PIN_12

//output gpio type (uodate to :2022.04.30)
typedef enum DoLineType
{
	LED_RUN=0,
	ZGM0_RST,
	STM32_Motor_En,
	STM32_Motor_Dir,
	STM32_Dcf_Ctrl,
	HX711_CLK_1,
	STM32_I2C1_WP,
#if(0 == MOTOR_PULSE_CTR_BY_PWM)
	STM32_Motor_Pwm,
#endif
	DO_GPIO_NUMBER
}enumDoLineType;

//input gpio type (uodate to :2022.04.30)
typedef enum DiLineType
{
	STM32_POS_1=0,
	STM32_POS_2,
	STM32_POS_3,
	STM32_KEY_1,
	STM32_KEY_2,
	STM32_KEY_3,
	STM32_KEY_4,
	HX711_DATA_1,
	DI_GPIO_NUMBER
}enumDiLineType;

typedef struct
{
    UINT16 PulseModeL : 1;
    UINT16 PulseModeH : 1;
    UINT16 ExtPulseDir : 1;
    UINT16 SelectUartCan : 1;
    UINT16 SelectRs485Rs422 : 1;
    UINT16 SelectEzEqepZ : 1;
    UINT16 EqeqDir : 1;
    UINT16 Warn : 1;
    UINT16 Fan : 1;
    UINT16 Res : 7;
}structDoStatusType;

typedef union
{
    structDoStatusType bits;
    UINT16 all;
}DoStatusType;


extern void hal_gpio_init( void );
extern UINT8 hal_di_get( UINT8 offset );
extern void hal_gpio_set_do_high( enumDoLineType Do );
extern void hal_gpio_set_do_low( enumDoLineType Do );
extern void hal_gpio_set_do_toggle( enumDoLineType Do );

#endif
