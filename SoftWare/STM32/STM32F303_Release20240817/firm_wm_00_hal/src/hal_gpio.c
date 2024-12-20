/**
  *******************************************************************
  * @file    hal_gpio.c
  * @author  MKdriver
  * @version V1.0.0
  * @date    9-Oct-2015
  * @brief   GPIO程序C文件(抽象层)
  *******************************************************************
  * @attention
  *
  *
  * <h2><center>&copy; COPYRIGHT 2015 MKdriver</center></h2>
  *******************************************************************
  */
  
#include "typedefine.h"
#include "Stm32F303xC_Vect_Tab.h"
#include "hal_irq_priority.h"
#include "drv_exti.h"
#include "drv_syscfg.h"
#include "drv_rcc.h"
#include "drv_nvic.h"
#include "hal_gpio.h"

/*
	OUTPUT
*/
GpioType* 		DO_GPIO_PORT[] = { \
DO0_GPIO_PORT,
DO1_GPIO_PORT,
DO2_GPIO_PORT,
DO3_GPIO_PORT,
DO4_GPIO_PORT, 
DO5_GPIO_PORT,
DO6_GPIO_PORT,
#if(0 == MOTOR_PULSE_CTR_BY_PWM)
DO7_GPIO_PORT, 
#endif
};
const UINT32  DO_GPIO_CLK[]  = { \
DO0_GPIO_CLK,
DO1_GPIO_CLK,
DO2_GPIO_CLK,
DO3_GPIO_CLK,
DO4_GPIO_CLK,
DO5_GPIO_CLK,
DO6_GPIO_CLK, 
#if(0 == MOTOR_PULSE_CTR_BY_PWM)
DO7_GPIO_CLK, 
#endif
};
const UINT16  DO_GPIO_PIN[]  = { \
DO0_GPIO_PIN,
DO1_GPIO_PIN,
DO2_GPIO_PIN,
DO3_GPIO_PIN,
DO4_GPIO_PIN,
DO5_GPIO_PIN,
DO6_GPIO_PIN,
#if(0 == MOTOR_PULSE_CTR_BY_PWM)
DO7_GPIO_PIN,
#endif
};
/*
	INPUT
*/
GpioType* 		DI_GPIO_PORT[] = { \
DI0_GPIO_PORT, \
DI1_GPIO_PORT, \
DI2_GPIO_PORT, \
DI3_GPIO_PORT, \
DI4_GPIO_PORT, \
DI5_GPIO_PORT, \
DI6_GPIO_PORT, \
DI7_GPIO_PORT, \
};
const UINT32  DI_GPIO_CLK[]  = { \
DI0_GPIO_CLK, \
DI1_GPIO_CLK, \
DI2_GPIO_CLK, \
DI3_GPIO_CLK, \
DI4_GPIO_CLK, \
DI5_GPIO_CLK, \
DI6_GPIO_CLK, \
DI7_GPIO_CLK, \
};
const UINT16  DI_GPIO_PIN[]  = { \
DI0_GPIO_PIN, \
DI1_GPIO_PIN, \
DI2_GPIO_PIN, \
DI3_GPIO_PIN, \
DI4_GPIO_PIN, \
DI5_GPIO_PIN, \
DI6_GPIO_PIN, \
DI7_GPIO_PIN, \
};

//gpio init
void hal_gpio_init()
{
	UINT32 i;

	// 逐位输入口线初始化
	for ( i = 0; i < DI_GPIO_NUMBER; i++ )
	{
		drv_rcc_ahb1_clk_enable( DI_GPIO_CLK[i] );
    	drv_gpio_input_init( DI_GPIO_PORT[i], DI_GPIO_PIN[i], GPIO_PUPD_NOPULL );
	}

	// 逐位输出口线初始化
	for ( i = 0; i < DO_GPIO_NUMBER; i++ )
	{
		drv_rcc_ahb1_clk_enable( DO_GPIO_CLK[i] );
    drv_gpio_output_init( DO_GPIO_PORT[i], DO_GPIO_PIN[i], GPIO_PUPD_PULL, GPIO_OTYPE_PP, GPIO_SPEED_LOW );
	}
	
}

/**
* @brief  DI输人采样
* @param  offset: 要采样的口线
* @retval UINT8 : 高/低
*/
UINT8 hal_di_get( UINT8 offset )
{
		if ( 0 == drv_gpio_get_pin_status( DI_GPIO_PORT[ offset ], DI_GPIO_PIN[ offset ] ) )
		{
			return 0;
		}
		else
		{
			return 1;
		}
}

/**
* @brief  设置指定端口的某些位为高电平.
* @param  pPort: 可以是 GPIOA ~ GPIOH
* @param  Pins: 要操作的位 GPIO_PIN_0 ~ GPIO_PIN_15 的组合
* @retval 无
*/
void drv_gpio_set_pins_high( GpioType *pPort, UINT16 Pins )
{
	pPort->BSRRL.all |= Pins;
}

/**
* @brief  DO输出高电平
* @param  Do: 要设置的口线
* @retval 无
*/
void hal_gpio_set_do_high( enumDoLineType Do )
{
    drv_gpio_set_pins_high( DO_GPIO_PORT[Do], DO_GPIO_PIN[Do] );
}

/**
* @brief  DO输出低电平
* @param  Do: 要设置的口线
* @retval 无
*/
void hal_gpio_set_do_low( enumDoLineType Do )
{
    drv_gpio_set_pins_low( DO_GPIO_PORT[Do], DO_GPIO_PIN[Do] );
}

/**
* @brief  DO输出电平反转
* @param  Do: 要设置的口线
* @retval 无
*/
void hal_gpio_set_do_toggle( enumDoLineType Do )
{
    drv_gpio_toggle_pin( DO_GPIO_PORT[Do], DO_GPIO_PIN[Do] );
}

