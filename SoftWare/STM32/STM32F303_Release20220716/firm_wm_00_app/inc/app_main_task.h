#ifndef __APP_MAIN_TASK_H__
#define __APP_MAIN_TASK_H__

#include "typedefine.h"
#include "app_led_ctrl.h"
#include "app_key_ctrl.h"

#define CURRENT_PRODUCT		(1)//0:weight mode , 1:TBES


#define COLOR_ALT_20210328_DEFINE	(FALSE)
#define COLOR_ALT_20210414_DEFINE	(FALSE)
#define COLOR_ALT_20210427_DEFINE	(FALSE)
#define COLOR_ALT_20210606_DEFINE	(FALSE)

#define SYS_HX711_ONLINE_CHECK_TIME	(2000)//when power on 2000ms start check HX711
#define SYS_REMOVE_WEIGHT_TIME		(3300)//when power on 3300ms remove weight
#define SYS_POWER_REDAY_TIME		(3500)//when power on 3500ms send data to T5L , do not change
#define SYS_SCREEN_BOOT_ANIMATION_DELAY	(60)
#define MCU_VERSION			(35)//2022.07.16
#define DIWEN_VERSION		(33)//2022.01.27
//#define MCU_VERSION		(33)//2022.01.27
//#define DIWEN_VERSION		(33)//2022.01.27

#define MOTOR_POS_Left		STM32_POS_3
#define MOTOR_POS_Middle	STM32_POS_1
#define MOTOR_POS_Right		STM32_POS_2

#define TBES_200_MODLE_CHOICE	(STM32_KEY_1)
#define TBES_300_MODLE_CHOICE	(STM32_KEY_2)
#define TBES_400_MODLE_CHOICE	(STM32_KEY_3)
#define TBES_START_MODLE_CHOICE	(STM32_KEY_4)





//================================================================================================
/*20220716 change list
1、用第二版硬件
  1.1、包含一个位置传感器：PM-F25(5-24V,≥8mA)，接白色线（常闭）
  1.2、驱动器换成：雷赛智能DM422S
  1.3、电机换成：雷赛智能35CM04，1.4A，0.4N.M
2、电机控制
  2.1、摇摆时，电机一直转即可
  2.2、停水平时，通过检测到中间的位置传感器后延迟150ms停止
3、上电3.5秒后电机停水平
*/
//================================================================================================




//================================================================================================
/*
1.Screen Page describe
Page1~48:log printf from min to max
Page49:Balancing page , six block display , two group of help data display
Page50:number input page
Page51:text input page
Page52:system param set page
Page53:system caculate page
Page54:second sheet , used for enter Page52 or Page49 or Page53
Page55:Balancing page , 12 block display
Page56:sys password
Page57:Balancing page , six block display , logo and unit and err display
Page58:Balancing page , 12 block display , home enter and remove enter
Page59:help page , six group help data display , logo and unit and err display

2.Page jump logic
2.1.1 when sys powerup , entry Page49
2.1.2 when 0x1102 write 0x1102 , Page49  -> Page57 (single module) ,or Page55  -> Page58 (cascatde module) 
2.1.3 when 0x1101 write 0x1101 , Page57  -> Page49 (single module) ,or Page58  -> Page55 (cascatde module)
2.1.4 when 0x2104 write   1010 , Page54  -> Page52
2.1.5 when 0x2103 write   2021 , Page54  -> Page53
2.1.6 when 0x2103 write   1202 , Page54  -> Page56
2.1.7 when home key enter 		 Page54 <-> Page49(single module) , or Page49 <-> Page55 / Page59 (cascatde module)

*/

//================================================================================================
/*20220119 change list
1.FunctionA Module
1.1.when gSystemPara.isCascade = ModbusFuncA_Slave
    1.1.1.need send the screen block display num as 7~12
    1.1.2.send it's self weight data to screen
    1.1.3.send color data from master to screen
    1.1.4.send caculate help data from master to screen
    1.1.5.screen exchange only six block page to sys para page
1.2.when gSystemPara.isCascade = ModbusFuncA_Master
	1.1.1.need send the screen block display num as 1~6 ; [don't change]
	1.1.2.send it's self weight data to screen ; [don't change]
	1.1.3.send color data from master to screen ; [don't change]
	1.1.4.send caculate help data from master to screen ; [don't change]
	1.1.5.screen exchange only six block page to sys para page

2.1 send weight and color date to screen
	
*/
#endif
