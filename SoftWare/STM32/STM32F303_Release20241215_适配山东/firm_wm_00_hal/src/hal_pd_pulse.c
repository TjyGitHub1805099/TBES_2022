/**
  *******************************************************************
  * @file    hal_pd_pulse.c
  * @author  MKdriver
  * @version V1.0.0
  * @date    9-Oct-2015
  * @brief   STM32F303 发送PD脉冲程序C文件
  *******************************************************************
  * @attention
  *
  *
  *
  * <h2><center>&copy; COPYRIGHT 2015 MKdriver</center></h2>
  *******************************************************************
  */

#include "hal_pd_pulse.h"
#include "hal_gpio.h"
#include "hal_delay.h"


//电机PWM初始化
void motor_pulse_init(void)
{
	TimerType *pTimer=PD_TIMER_BASE;
	// 配置PB15端口时钟 定时器TIM15时钟
	drv_rcc_ahb1_clk_enable( PD_PULSE_GPIO_CLK );
	drv_rcc_apb2_clk_enable(PD_TIMER_CLK);

	// 配置管脚PB15
	drv_gpio_alternate_function_init( PD_PULSE_GPIO_PORT, PD_PULSE_GPIO_PIN, GPIO_PUPD_PULL, GPIO_OTYPE_PP, GPIO_SPEED_MID );
	// 复用管脚
	drv_gpio_alternate_function_connect( PD_PULSE_GPIO_PORT, PD_PULSE_GPIO_SOURCE, PD_GPIO_AF );

	// 定时器模块先复位
	drv_timer_deinit( PD_TIMER_BASE );

	//使能定时器时钟
	//定时器DEBUG模式停止计数
	if( ( PD_TIMER_BASE == TIM1 ) || ( PD_TIMER_BASE == TIM8 ) || ( PD_TIMER_BASE == TIM15 ) || ( PD_TIMER_BASE == TIM16 ) || ( PD_TIMER_BASE == TIM17 ) || ( PD_TIMER_BASE == TIM20 ) )
	{
		drv_rcc_apb2_clk_enable( PD_TIMER_CLK );
		drv_debug_apb2_stop( PD_TIMER_DEBUG_STOP );
	}
	else
	{
		drv_rcc_apb1_clk_enable( PD_TIMER_CLK );
		drv_debug_apb1_stop( PD_TIMER_DEBUG_STOP );
	}


	pTimer->ARR = 100;//PWM频率 = 1/100*TIM的时钟输入 = 10kHz
	pTimer->PSC = (64 - 1);//TIM的时钟输入 = 1/64*系统时钟的64Mhz = 1MHz
	pTimer->CR1.bit.CKD = 0 ;//时钟分频因子

	
	pTimer->CR1.bit.DIR = 0 ;//向上计数
	pTimer->CR1.bit.ARPE = 1;//自动重载预装载允许位

	
	
	pTimer->SR.bit.UIF = 0 ;//更新中断 由软件清零
	
	pTimer->CCER.bit.CC2P = 0;//0C2高电平有效
	pTimer->CCER.bit.CC2E = 1;//OC2输出使能到对应的输出引脚

	pTimer->CCMR1.bit_o.CC2S = 1 ;//输出比较2 通道2被配置成输出
	pTimer->CCMR1.bit_o.OC2M = 6;//PWM模式1
	pTimer->CCMR1.bit_o.OC2PE = 1 ;//输出比较2预装载使能，开启TIM15 CCR2寄存器预装载	功能


	pTimer->CCR2 = pTimer->ARR / 2;//占空比


	pTimer->BDTR.bit.MOE = 1 ;//主输出使能

	pTimer->CR1.bit.CEN = 1 ;//开启计数器

}


void motor_pulse_output_set(UINT8 en)
{
	TimerType *pTimer=PD_TIMER_BASE;
	pTimer->CR1.bit.CEN = en ;//开启计数器
}



//电机控制使能设定
void motor_pulse_en_set(UINT8 en)
{
	TimerType *pTimer=PD_TIMER_BASE;

	if(0 == en)
	{
		hal_gpio_set_do_low( (enumDoLineType)(STM32_Motor_En) );//电机不使能
		pTimer->CR1.bit.CEN = 0 ;//关闭计数器
	}
	else
	{
		hal_gpio_set_do_high( (enumDoLineType)(STM32_Motor_En) );//电机使能
		hal_delay_us(50);
		pTimer->CR1.bit.CEN = 1 ;//打开计数器
	}
}


//电机控制方向设定
void motor_pulse_dir_set(UINT8 dir)
{
	if(0 == dir)
	{
		hal_gpio_set_do_low( (enumDoLineType)(STM32_Motor_Dir) );	
	}
	else
	{
		hal_gpio_set_do_high( (enumDoLineType)(STM32_Motor_Dir) );
	}
}

//电机控制PWM频率设定
void motor_pulse_frequency_set(UINT16 fre)
{
	TimerType *pTimer=PD_TIMER_BASE;

	if(0 != fre)
	{
		pTimer->ARR = 1000000/fre;//PWM频率 = 1/100*TIM的时钟输入
		pTimer->CCR2 = pTimer->ARR / 2;//占空比
		//
		pTimer->CNT.all = 0 ;
	}
}

//电机控制 TIMER CNT 清零 频率设定
void motor_TimerCnt_set(UINT16 vlu)
{
	TimerType *pTimer=PD_TIMER_BASE;
	pTimer->CNT.all = vlu ;
}


//电机控制脉冲数量设定
void motor_pulse_num_set(UINT8 dir,UINT16 fre,UINT32 num)
{
	//计算脉冲数量
	
}


void motor_PulseHandle(UINT8 en ,UINT8 dir, UINT16 fre, UINT32 num)
{
	static UINT8 en_pre=0xFF , dir_pre=0xFF;
	static UINT16 fre_pre=0XFFFF;
	//en = 1:电机使能,en = 0:电机不使能
	if(en_pre != en)
	{
		en_pre = en;
		motor_pulse_en_set(en);
	}
	//dir = 1:电机正传,en = 0:电机反正传
	if(dir_pre != dir)
	{
		dir_pre = dir;
		motor_pulse_dir_set(dir);
	}
	//fre = 电机脉冲频率
	if(fre_pre != fre)
	{
		fre_pre = fre;
		motor_pulse_frequency_set(fre);
	}
}





#if 0
/**
 * @brief  PD 发送完成中断处理函数
 * @retval 无
 */
CCMRAM void pd_send_finish_irq( void )
{
    drv_timer_set_compara1( PD_TIMER_BASE, 0xFFFF );
    drv_timer_clear_it_pending_bit( EQEP2_TIMER_BASE, TIM_IT_CC1 );
    drv_timer_it_disable( EQEP2_TIMER_BASE, TIM_IT_CC1 );

}
#endif

//#endif
