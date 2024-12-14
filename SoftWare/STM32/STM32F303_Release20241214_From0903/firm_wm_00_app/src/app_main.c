/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "typedefine.h"
#include "hal_clock.h"
#include "hal_delay.h"
#include "hal_timer.h"
#include "hal_gpio.h"
#include "hal_uart.h"
#include "drv_cpu.h"
#include "app_main_task.h"
#include "app_led_ctrl.h"
#include "app_sdwe_ctrl.h"
#include "app_hx711_ctrl.h"
#include "app_key_ctrl.h"
#include "app_modbus_rtu_ctrl.h"
#include "app_password.h"
#include "app_syspara.h"
#include "app_t5l_ctrl.h"
#include "drv_iwdg.h"
#include "hal_pd_pulse.h"
#include "app_motor_ctrl.h"

/*******************************************************************************
 * Functions
 ******************************************************************************/
/**
 * @brief  系统初始化
 * @retval 无
 */
void system_init( void )
{
    UINT32 l_VectStartAddress = FLASH_BASE, l_VectOffset = 0;

#ifdef DEBUG_RAM
    l_VectStartAddress = SRAM_BASE;
#endif	

#ifdef PROJ_BOOTLOADER
    l_VectOffset = 0x4000;				// 16KB
#endif

    hal_clock_init( l_VectStartAddress + l_VectOffset );
    hal_delay_init( );
	
	//weight mode gpio init
	hal_gpio_init();
	
    hal_timer_init( 1000 );

	drv_iwdg_init(1000);
    //wdg_init( 4000 );
}

/**
 * @brief  系统主函数
 * @retval 无
 */
int main(void)
{
	system_init();
	key_init();
	//app position init
	pos_init();

	hx711_init();
	screenT5L_Init();
	#if(0 == CURRENT_PRODUCT)
	ModbusRtu_init();
	#endif
	readSysDataFromFlash();
	readSysDataFromFlash_3030();
	STM32MCUIDGet(&STM32McuId[0],STM32F1);
	STM32CheckPassWord(g_passWordStore);
	
	motor_pulse_init();
	app_MotorContrl_Init();

	app_gjf_handle(SDWeFaKaiFaGuan_FaKai);

  	while(1)
	{}
}

