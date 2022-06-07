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
#define MCU_VERSION			(33)//2022.01.27
#define DIWEN_VERSION		(33)//2022.01.27

#define MOTOR_POS_Left		STM32_POS_3
#define MOTOR_POS_Middle	STM32_POS_1
#define MOTOR_POS_Right		STM32_POS_2

#define TBES_200_MODLE_CHOICE	(STM32_KEY_1)
#define TBES_300_MODLE_CHOICE	(STM32_KEY_2)
#define TBES_400_MODLE_CHOICE	(STM32_KEY_3)
#define TBES_START_MODLE_CHOICE	(STM32_KEY_4)


typedef enum
{
	MotorCtrlStatus_DISEN=0,
	MotorCtrlStatus_EN_NormalRun_WitMaxPos,
	MotorCtrlStatus_EN_NormalRun_WitMaxPos1,
	MotorCtrlStatus_EN_NormalRun_WitTMode,
	MotorCtrlStatus_EN_StopToMiddle,
	MotorCtrlStatus_EN_StopToMiddle1,
	MotorCtrlStatus_MAX
}enumMotorCtrlStatusType;




typedef struct structMotorContrlType
{
	enumMotorCtrlStatusType motorCtrlStatus;

	UINT8 findStatus;

	float motorJianSuBi;//减速比 = 13.7641
	UINT16 motorXiFen;//细分比例 = 10000
	INT32 motorAge;//运行角度 ， 单边 = 20°
	float motorTotalPulse;//单边行程 总脉冲数 = motorAge/360 * motorXiFen * motorJianSuBi

	//
	float tfAB;//单边行程 B点 时间 百分比 0.3
	float tfAC;//单边行程 C点 时间 百分比 0.6
	INT32 tAD;//单边行程总时间 ms
	UINT16 tAB;//单边行程 B点 时间 ms = tAD * tfAB
	UINT16 tAC;//单边行程 C点 时间 ms = tAD * tfAC

	float pwmMAX_ms;//脉冲频率 每ms 个数
	
	float kAB;//单边行程 A->B点 斜率 (ms) f=kABx+bAB
	float kCD;//单边行程 C->D点 斜率 (ms) f=-kABx+bCD

	UINT8  leftPosVlu;
	UINT8  rightPosVlu;
	UINT8  runEn;
	UINT8  runDir;
	UINT16 runPwmFre;
	UINT16 runPwmFreFixed;
	UINT16 runArrivedFlag;
	UINT32 curStep_Ms;
	UINT32 atMiddleStepOffset;//运行它以追快的速度多走100ms
}MotorContrlType;


#define MotorContrlDefault   { \
	MotorCtrlStatus_DISEN,\
	0,\
	13.7641,\
	10000,\
	10,\
	3823, \
	0.33,\
	0.66,\
	2000,\
	660, \
	1320, \
	2.87,\
	0.004355618,\
	0.004227511,\
	FALSE,\
	FALSE,\
	FALSE,\
	FALSE,\
	0,\
	8000,\
	FALSE,\
	0,\
	100,\
	}

extern MotorContrlType motorCtl;

extern void app_MotorContrl_Init(void);
void app_SetMotorContrlMode(enumMotorCtrlStatusType setStatus);

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
