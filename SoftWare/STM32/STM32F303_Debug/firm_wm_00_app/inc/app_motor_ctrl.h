#ifndef __APP_TEBS_MOTOR_CTRL_H__
#define __APP_TEBS_MOTOR_CTRL_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum
{
	MotorCtrlStatus_DISEN=0,
	MotorCtrlStatus_EN,
	MotorCtrlStatus_EN_NormalRun_WitMaxPos,
	MotorCtrlStatus_EN_NormalRun_WitMaxPos1,
	MotorCtrlStatus_EN_NormalRun_WitTMode,
	MotorCtrlStatus_EN_NormalRun_WitTMode1,
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
	//
	INT32 tAD;//单边行程总时间 ms
	UINT16 tAB;//单边行程 B点 时间 ms = tAD * tfAB
	UINT16 tAC;//单边行程 C点 时间 ms = tAD * tfAC

	float pwmMAX_ms;//脉冲频率 每ms 个数
	
	float kAB;//单边行程 A->B点 斜率 (ms) f=kABx+bAB
	float bAB;//单边行程 A->B点 斜率 (ms) f=kABx+bAB

	UINT16 tBC_t_pulse[2][3];

	float kCD;//单边行程 C->D点 斜率 (ms) f=-kABx+bCD
	float bCD;//单边行程 C->D点 斜率 (ms) f=-kABx+bCD

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
	660,\
	1320,\
	2.87,\
	0.004355618,\
	0,\
	{{0,0,0},{0,0,0}},\
	0.004227511,\
	8.45502236,\
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
extern void app_MotorContrl(void);


#endif

