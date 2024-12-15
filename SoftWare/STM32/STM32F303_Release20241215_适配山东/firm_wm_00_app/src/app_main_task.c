/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "hal_gpio.h"
#include "app_led_ctrl.h"
#include "app_hx711_ctrl.h"
#include "app_main_task.h"
#include "app_key_ctrl.h"
#include "app_sdwe_ctrl.h"
#include "app_modbus_rtu_ctrl.h"
#include "app_t5l_ctrl.h"
#include "drv_iwdg.h"
#include "app_syspara.h"
#include "hal_pd_pulse.h"
#include "app_motor_ctrl.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//sys main task status
UINT32 g_sys_ms_tick = 0 ;


/*******************************************************************************
 * Function
 ******************************************************************************/
//==sys main init function
void app_main_init(void)
{
	//after power up 3.3 seconds clear all weight
	static UINT8 removeWeight = TRUE;
	if((TRUE == removeWeight)&&(g_sys_ms_tick >= SYS_REMOVE_WEIGHT_TIME))
	{
		removeWeight = FALSE;
		hx711_setAllRemoveWeight();
	}
}

/*******************************************************************************
 * Function
 ******************************************************************************/
//==sys main function
void app_main_task()
{
	UINT8 hx711DataUpgrade = 0 ;
	(void)hx711DataUpgrade;
	
	//sys tick add
	g_sys_ms_tick++;

	//feed watch dog
	drv_iwdg_feed();

	//app running indicate led 
	app_LED_RUN(1000);
	
	//app power on init
	app_main_init();

	//KEY sample and filter
	key_MainFunction();
	
	//POS sample and filter
	pos_MainFunction();
	
	//HX711 sanple and calculate avgSampleValue and weight
	if(1 == hx711_SenserCheck())
	{
		hx711DataUpgrade = hx711_MainFunction();
	}

	//SDWe MainFunction
	sreenSDWe_MainFunction();

	//电机控制
	app_MotorContrl();
}

