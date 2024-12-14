/**
 *******************************************************************
 * @file    hal_pd_pulse.h
 * @author  MKdriver
 * @version V1.0.0
 * @date    9-Oct-2015
 * @brief   STM32F303 发送PD脉冲程序头文件
 *******************************************************************
 * @attention
 *
 *
 *
 * <h2><center>&copy; COPYRIGHT 2015 MKdriver</center></h2>
 *******************************************************************
 */
 
#ifndef __HAL_PD_OUTPUT_H__
#define __HAL_PD_OUTPUT_H__


//#ifndef PROJ_BOOTLOADER

#include "drv_rcc.h"
#include "drv_nvic.h"
#include "drv_gpio.h"
#include "drv_dbgmcu.h"
#include "drv_syscfg.h"
#include "drv_exti.h"
#include "drv_timer.h" 

// PD P = Timer15 CH2
#define PD_TIMER_BASE				    TIM15
#define PD_TIMER_CLK					RCC_APB2ENR_TIM15EN
#define PD_TIMER_DEBUG_STOP			    DBGMCU_TIM15_STOP
#define PD_TIMER_IRQn			        TIM1_BRK_TIM15_IRQn
#define PD_GPIO_AF					    GPIO_AF1_TIM15

// PD P = GPIO PB15
#define PD_PULSE_GPIO_PORT		        GPIOB
#define PD_PULSE_GPIO_CLK		        RCC_AHBENR_GPIOBEN
#define PD_PULSE_GPIO_PIN		        GPIO_PIN_15
#define PD_PULSE_GPIO_SOURCE		    GPIO_PINSOURCE15

// PD D = GPIO PB13
#define PD_DIR_GPIO_PORT		        GPIOB
#define PD_DIR_GPIO_CLK		            RCC_AHBENR_GPIOBEN
#define PD_DIR_GPIO_PIN		            GPIO_PIN_13

//#define hal_pd_send_finish_irq          TIM1_BRK_TIM15_IRQnHandler

typedef struct PdData
{
    UINT8   ValidFlag;          // 有效标志，1表示有效
    UINT8   Dir;                // 方向，1为高电平
    UINT16  msTime;             // 毫秒级时间
    UINT32  PulseNumber;        // 要发送脉冲的数量
}structPdDataType;

extern UINT16 g_PdSendPulseDelayCounter;

extern void hal_pd_init( void );
//extern void hal_pd_counter_init( void );
extern void hal_pd_stop( void );
extern void hal_pd_set_output( UINT8 Dir, UINT16 msTime, UINT32 PulseNumber );
extern CCMRAM void hal_pd_output( void );
extern void motor_pulse_init(void);
extern void motor_pulse_dir_set(UINT8 dir);
extern void motor_pulse_frequency_set(UINT16 fre);
extern void motor_PulseHandle(UINT8 en ,UINT8 dir, UINT16 fre, UINT32 num);
extern void motor_pulse_output_set(UINT8 en);
extern void motor_TimerCnt_set(UINT16 vlu);
//#endif

#endif

