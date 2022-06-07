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

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//sys main task status
UINT32 g_sys_ms_tick = 0 ;
MotorContrlType motorCtl = MotorContrlDefault;


/*******************************************************************************
 * Functions
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
		t5lDisPlayDataClear();
	}
}

//==gjf motor handle
void app_gjf_handle(enumSDWeFaKaiFaGuanType status)
{
	if(status == SDWeFaKaiFaGuan_FaKai)
	{
		hal_gpio_set_do_low( (enumDoLineType)(STM32_Dcf_Ctrl) );
	}
	else
	{
		hal_gpio_set_do_high( (enumDoLineType)(STM32_Dcf_Ctrl) );
	}
	g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FK_FG] = status;

	if(TRUE != gSystemPara.u16_kaiqi_guanjiafa_gongneng)
	{
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FK_FG] = (UINT16)SDWeFaKaiFaGuan_KongBai;//阀开/关 = 空白
	}
}


//==电机控制 初始化
void app_MotorContrl_Init(void)
{
	//单边行程总脉冲数
	motorCtl.motorTotalPulse = motorCtl.motorJianSuBi;
	motorCtl.motorTotalPulse *= motorCtl.motorXiFen;
	motorCtl.motorTotalPulse /= 360;
	motorCtl.motorTotalPulse *= motorCtl.motorAge;

	//梯形图 时间点
	//motorCtl.tAD = 500;//单边行程 总时间 ms
	motorCtl.tAB = motorCtl.tAD * motorCtl.tfAB;
	motorCtl.tAC = motorCtl.tAD * motorCtl.tfAC;
	
	//每ms脉冲数
	motorCtl.pwmMAX_ms = motorCtl.motorTotalPulse * 2;
	motorCtl.pwmMAX_ms /= (motorCtl.tAD + (motorCtl.tAC - motorCtl.tAB));

	//行程A->B 的斜率
	motorCtl.kAB = motorCtl.pwmMAX_ms / (motorCtl.tAB);
	
	//行程C->D 的斜率
	motorCtl.kCD = -(motorCtl.pwmMAX_ms / (motorCtl.tAD - motorCtl.tAC));

	motorCtl.runEn = FALSE;
	motorCtl.runDir = 0 ;
	motorCtl.runPwmFre = 50;

	motor_PulseHandle(motorCtl.runEn,motorCtl.runDir,motorCtl.runPwmFre,0);
}


//==设置电机运行模式
void app_SetMotorContrlMode(enumMotorCtrlStatusType setStatus)
{
	motorCtl.motorCtrlStatus = setStatus;
}


//==电机实时运行控制
void app_MotorContrl(void)
{
	static UINT16 middleAppear = FALSE , middleStopedEn = FALSE;
	switch(motorCtl.motorCtrlStatus)
	{
		case MotorCtrlStatus_DISEN://电机不使能
			motorCtl.curStep_Ms = 0;
			//
			motorCtl.runEn = FALSE;
		break;




		
		case MotorCtrlStatus_EN_NormalRun_WitMaxPos://电机以左右限位运行
			middleStopedEn = FALSE;
			
			motor_pulse_output_set(TRUE);
			motorCtl.motorCtrlStatus = MotorCtrlStatus_EN_NormalRun_WitMaxPos1;
		break;
		case MotorCtrlStatus_EN_NormalRun_WitMaxPos1://电机以左右限位运行
			motorCtl.curStep_Ms = 0;
			//
			motorCtl.runEn = TRUE;
			
			//左限位检测 ： 直接反向 且清零 curStep_Ms
			if(TRUE == pos_EventGet(MOTOR_POS_Left))//通过限位开关判断
			{
				pos_EventClear(MOTOR_POS_Left);
				if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Left))
				{	
					motorCtl.runDir = 0;
				}
			}
			//右限位检测 ： 直接反向 且清零 curStep_Ms
			else if(TRUE == pos_EventGet(MOTOR_POS_Right))//通过限位开关判断
			{
				pos_EventClear(MOTOR_POS_Right);
				if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Right))
				{	
					motorCtl.runDir = 1;
				}
			}
			else if(TRUE == pos_EventGet(MOTOR_POS_Middle))
			{
				pos_EventClear(MOTOR_POS_Middle);
				if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Middle))
				{
					middleAppear = TRUE;
				}
				else
				{
					middleAppear = FALSE;
				}
			}
			//
			motorCtl.runPwmFre = motorCtl.runPwmFreFixed;


			//==实时采集界面
			if(SDWeCurPage_ShiShiJieMian == g_T5L.curPage)
			{
				//==采集到xx% ~100% 且水平位置出现
				if((g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] >= gSystemPara.u16_fmqkqsj)
					&& (g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] < 100)
					&& (TRUE == middleAppear))
				{
					middleStopedEn = TRUE;
				}				
			}


			if(TRUE == middleStopedEn)
			{
				if(g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] >= 100)
				{
					middleStopedEn = FALSE;
					motor_pulse_output_set(TRUE);
				}
				else
				{
					motor_pulse_output_set(FALSE);
					//motorCtl.runPwmFre = 50;//电机运行较慢 = 锁轴
				}
			}

		break;
		case MotorCtrlStatus_EN_NormalRun_WitTMode://电机以T形图运行
			motorCtl.curStep_Ms++;
			//
			motorCtl.runEn = TRUE;

			//计算每 ms 的PWM频率
			if(motorCtl.curStep_Ms <= motorCtl.tAB)
			{
				motorCtl.runPwmFre = 1000 * (motorCtl.kAB*motorCtl.curStep_Ms );
			}
			else if((motorCtl.curStep_Ms > motorCtl.tAB) && (motorCtl.curStep_Ms <= motorCtl.tAC))
			{
				motorCtl.runPwmFre = 1000 * motorCtl.pwmMAX_ms;
			}else if((motorCtl.curStep_Ms > motorCtl.tAC) && (motorCtl.curStep_Ms <= motorCtl.tAD))
			{
				motorCtl.runPwmFre = 1000 * (motorCtl.pwmMAX_ms +  motorCtl.kCD*(motorCtl.curStep_Ms-motorCtl.tAC));
			}
			else if(motorCtl.curStep_Ms >= motorCtl.tAD)//T形图 走到最大时间 就反向
			{
				motorCtl.curStep_Ms = 0 ;
				motorCtl.runDir = ~(motorCtl.runDir);
				motorCtl.runPwmFre = 50;
			}


			//左限位检测 ： 直接反向 且清零 curStep_Ms
			if(TRUE == pos_EventGet(MOTOR_POS_Left))//通过限位开关判断
			{
				pos_EventClear(MOTOR_POS_Left);
				if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Left))
				{	
					motorCtl.runDir = 0;
					motorCtl.curStep_Ms = 0 ;
				}
			}
			//右限位检测 ： 直接反向 且清零 curStep_Ms
			else if(TRUE == pos_EventGet(MOTOR_POS_Right))//通过限位开关判断
			{
				pos_EventClear(MOTOR_POS_Right);
				if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Right))
				{	
					motorCtl.runDir = 1;
					motorCtl.curStep_Ms = 0 ;
				}
			}
			//中间位检测 ： 判断 校准 curStep_Ms 为过半
			else if(TRUE == pos_EventGet(MOTOR_POS_Middle))
			{
				pos_EventClear(MOTOR_POS_Middle);
				if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Middle))
				{
					motorCtl.curStep_Ms = motorCtl.tAD / 2 - motorCtl.atMiddleStepOffset;
				}
			}
			
		break;
			
		case MotorCtrlStatus_EN_StopToMiddle://电机停止在中间
			middleAppear = FALSE;
			middleStopedEn = FALSE;

			g_T5L.sampleComplet = FALSE;
			
			//开管夹阀
			app_gjf_handle(SDWeFaKaiFaGuan_FaKai);

			motor_pulse_output_set(TRUE);
			
			motorCtl.motorCtrlStatus = MotorCtrlStatus_EN_StopToMiddle1;
		break;
		case MotorCtrlStatus_EN_StopToMiddle1://电机停止在中间
			motorCtl.curStep_Ms = 0;
			//
			motorCtl.runEn = TRUE;
			
			//左限位检测 ： 直接反向 且清零 curStep_Ms
			if(TRUE == pos_EventGet(MOTOR_POS_Left))//通过限位开关判断
			{
				pos_EventClear(MOTOR_POS_Left);
				if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Left))
				{	
					motorCtl.runDir = 0;
				}
			}
			//右限位检测 ： 直接反向 且清零 curStep_Ms
			else if(TRUE == pos_EventGet(MOTOR_POS_Right))//通过限位开关判断
			{
				pos_EventClear(MOTOR_POS_Right);
				if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Right))
				{	
					motorCtl.runDir = 1;
				}
			}
			else if(TRUE == pos_EventGet(MOTOR_POS_Middle))
			{
				pos_EventClear(MOTOR_POS_Middle);
				if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Middle))
				{
					middleAppear = TRUE;
				}
				else
				{
					middleAppear = FALSE;
				}
			}
			//
			motorCtl.runPwmFre = motorCtl.runPwmFreFixed;

			if(TRUE == middleAppear)
			{
				middleStopedEn = TRUE;
			}

			if(TRUE == middleStopedEn)
			{
				motor_pulse_output_set(FALSE);
				//motorCtl.runPwmFre = 50;//电机运行较慢 = 锁轴
			}

		break;
		default:
		break;
	}









	//设置驱动器 ： 使能 方向 脉冲频率
	motor_PulseHandle(motorCtl.runEn,motorCtl.runDir,motorCtl.runPwmFre,0);
}













//==电机控制 实时运行
void app_MotorContrl_Run(UINT8 sampleCplt)
{
	static UINT32  ADD = 0;
	
	(void)ADD;
	
	//电机使能
	if(TRUE == motorCtl.runEn)
	{
		motorCtl.curStep_Ms++;
		motorCtl.runArrivedFlag = FALSE;
	}
	else
	{
		motorCtl.curStep_Ms = 0;
		motorCtl.runArrivedFlag = FALSE;
	}


	//左限位检测 ： 直接反向 且清零 curStep_Ms
	if(TRUE == pos_EventGet(MOTOR_POS_Left))//通过限位开关判断
	{
		pos_EventClear(MOTOR_POS_Left);
		if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Left))
		{	
			motorCtl.runDir = 0;
			motorCtl.curStep_Ms = 0 ;
			motorCtl.runArrivedFlag = TRUE;
		}
	}

	if(TRUE == pos_EventGet(MOTOR_POS_Middle))//middle stop
	{
		pos_EventClear(MOTOR_POS_Middle);
		if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Middle))
		{
			//motorCtl.curStep_Ms = motorCtl.tAD / 2;
		}
	}

	
	//右限位检测 ： 直接反向 且清零 curStep_Ms
	if(TRUE == pos_EventGet(MOTOR_POS_Right))//通过限位开关判断
	{
		pos_EventClear(MOTOR_POS_Right);
		if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Right))
		{	
			motorCtl.runDir = 1;
			motorCtl.curStep_Ms = 0 ;
			motorCtl.runArrivedFlag = TRUE;
		}
	}
	//当单边行程时间超过设定时间时 直接反向
	
	if(motorCtl.curStep_Ms >= (motorCtl.tAD))
	{
		motorCtl.runDir = ~motorCtl.runDir;
		motorCtl.curStep_Ms = 0 ;
		motorCtl.runArrivedFlag = TRUE;
	}


/*
	if(TRUE == pos_EventGet(MOTOR_POS_Middle))//middle stop
	{
		pos_EventClear(MOTOR_POS_Middle);
		if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Middle))
		{	
			motorCtl.curStep_Ms = motorCtl.tAD/2;
		}
	}

*/

	//计算每 ms 的PWM频率
	if(motorCtl.curStep_Ms <= motorCtl.tAB)
	{
		motorCtl.runPwmFre = 1000 * (motorCtl.kAB*motorCtl.curStep_Ms );
	}
	else if((motorCtl.curStep_Ms > motorCtl.tAB) && (motorCtl.curStep_Ms <= motorCtl.tAC))
	{
		motorCtl.runPwmFre = 1000 * motorCtl.pwmMAX_ms;
	}else if((motorCtl.curStep_Ms > motorCtl.tAC) && (motorCtl.curStep_Ms <= motorCtl.tAD))
	{
		motorCtl.runPwmFre = 1000 * (motorCtl.pwmMAX_ms +  motorCtl.kCD*(motorCtl.curStep_Ms-motorCtl.tAC));
	}
	else
	{
		motorCtl.runPwmFre = 50;//50Hz
	}

	//采样完成 水平位置         保持不运动	一直清CNT
	if(TRUE == sampleCplt)
	{
		motorCtl.curStep_Ms =  motorCtl.tAD/2;
		motorCtl.runPwmFre = 50;//50Hz
		motor_TimerCnt_set(0);
	}
	
	//设置驱动器 ： 使能 方向 脉冲频率
	motor_PulseHandle(motorCtl.runEn,motorCtl.runDir,motorCtl.runPwmFre,0);
}





//电机控制 上电检测 左中右位置 
void app_MotorContrl_FindPos(UINT16 findFre)
{
	//8000HZ �� Ƶ�� ������ 900ms
	static INT32 oneSided_Left = 0 , oneSided_Middle = 0 , oneSided_Right = 0;
	static INT32 oneSided_LeftToRight = 0, oneSided_RightToLeft = 0;
	static UINT32 findPosOneSideTicks , findPosTotalTicksMax = 30000;//2倍周期

	(void)oneSided_Middle;
	
	
	findPosOneSideTicks++;

	if(0xFF != motorCtl.findStatus)
	{
		switch(motorCtl.findStatus)
		{
			case 0://查找 左 右 限位
				motorCtl.runEn = TRUE;
				motorCtl.runPwmFre = findFre;
				//左限位检测 ： 直接反向 且清零 curStep_Ms
				if(TRUE == pos_EventGet(MOTOR_POS_Left))//通过限位开关判断
				{
					pos_EventClear(MOTOR_POS_Left);
					if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Left))
					{	
						motorCtl.runDir = 0;
						//
						findPosOneSideTicks = 0;
						oneSided_Left = g_sys_ms_tick;
						if(0 != oneSided_Right)
						{
							oneSided_RightToLeft = oneSided_Left - oneSided_Right;
						}
					}
				}
				else if(TRUE == pos_EventGet(MOTOR_POS_Middle))//middle stop
				{
					pos_EventClear(MOTOR_POS_Middle);
					if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Middle))
					{	
						oneSided_Middle = g_sys_ms_tick;
						//左边 右边 都找到了
						if((0 != oneSided_RightToLeft) && (oneSided_LeftToRight != 0))
						{
							motorCtl.runPwmFre = 50;
							//判断
							motorCtl.findStatus = 1;
						}
					}
				}
				//右限位检测 ： 直接反向 且清零 curStep_Ms
				else if(TRUE == pos_EventGet(MOTOR_POS_Right))//通过限位开关判断
				{
					pos_EventClear(MOTOR_POS_Right);
					if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Right))
					{	
						motorCtl.runDir = 1;
						//
						findPosOneSideTicks = 0;
						oneSided_Right = g_sys_ms_tick;
						if(0 != oneSided_Left)
						{
							oneSided_LeftToRight = oneSided_Right - oneSided_Left;
						}
					}
				}
				//堵转情况
				if(findPosOneSideTicks >= (findPosTotalTicksMax/2))
				{
					motorCtl.findStatus = 1;
				}
				motor_PulseHandle(motorCtl.runEn,motorCtl.runDir,motorCtl.runPwmFre,0);
			break;
			case 1://左 中 右都找到了	或则时间超了
				motorCtl.findStatus = 0XFF;
				if(findPosOneSideTicks < (findPosTotalTicksMax/2))
				{
					if(oneSided_LeftToRight <= (findPosTotalTicksMax/2))
					{
						if(oneSided_RightToLeft <= (findPosTotalTicksMax/2))
						{
							//success
							motorCtl.findStatus = 0xFE;
							//
							motorCtl.runEn = TRUE;
							motorCtl.runPwmFre = 50;
						}
						else//右边到左边时间不满足
						{
							motorCtl.runEn = TRUE;
							motorCtl.runPwmFre = 50;
						}
					}
					else//左边到右边时间不满足
					{
						motorCtl.runEn = TRUE;
						motorCtl.runPwmFre = 50;
					}
				}
				else//堵转
				{
					motorCtl.runEn = FALSE;
					motorCtl.runPwmFre = 50;
				}
				//
				motor_PulseHandle(motorCtl.runEn,motorCtl.runDir,motorCtl.runPwmFre,0);
			break;
			case 0xFE://success
				
			break;
			
			default:
				motorCtl.findStatus = 0XFF;
				motorCtl.runEn = FALSE;
				motorCtl.runPwmFre = 50;
				//
				motor_PulseHandle(motorCtl.runEn,motorCtl.runDir,motorCtl.runPwmFre,0);
			break;
		}
	}
}



////////////////////


void app_MotorContrl_FindPos111111(UINT16 findFre)
{
	//8000HZ �� Ƶ�� ������ 900ms
	static INT32 oneSided_Left = 0 , oneSided_Middle = 0 , oneSided_Right = 0;
	static INT32 oneSided_LeftToRight = 0, oneSided_RightToLeft = 0;
	static UINT32 findPosOneSideTicks , findPosTotalTicksMax = 30000;//2倍周期

	(void)oneSided_Middle;
	
	findPosOneSideTicks++;

	if(0xFF != motorCtl.findStatus)
	{
		switch(motorCtl.findStatus)
		{
			case 0://查找 左 右 限位
				motorCtl.runEn = TRUE;
				motorCtl.runPwmFre = findFre;
				//左限位检测 ： 直接反向 且清零 curStep_Ms
				if(TRUE == pos_EventGet(MOTOR_POS_Left))//通过限位开关判断
				{
					pos_EventClear(MOTOR_POS_Left);
					if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Left))
					{	
						motorCtl.runDir = 0;
						//
						findPosOneSideTicks = 0;
						oneSided_Left = g_sys_ms_tick;
						if(0 != oneSided_Right)
						{
							oneSided_RightToLeft = oneSided_Left - oneSided_Right;
						}
					}
				}
				else if(TRUE == pos_EventGet(MOTOR_POS_Middle))//middle stop
				{
					pos_EventClear(MOTOR_POS_Middle);
					if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Middle))
					{	
						oneSided_Middle = g_sys_ms_tick;
						//左边 右边 都找到了
						if((0 != oneSided_RightToLeft) && (oneSided_LeftToRight != 0))
						{
							//motorCtl.runPwmFre = 50;
							//判断
							//motorCtl.findStatus = 1;
						}
					}
				}
				//右限位检测 ： 直接反向 且清零 curStep_Ms
				else if(TRUE == pos_EventGet(MOTOR_POS_Right))//通过限位开关判断
				{
					pos_EventClear(MOTOR_POS_Right);
					if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Right))
					{	
						motorCtl.runDir = 1;
						//
						findPosOneSideTicks = 0;
						oneSided_Right = g_sys_ms_tick;
						if(0 != oneSided_Left)
						{
							oneSided_LeftToRight = oneSided_Right - oneSided_Left;
						}
					}
				}
				//堵转情况
				if(findPosOneSideTicks >= (findPosTotalTicksMax/2))
				{
					motorCtl.findStatus = 1;
				}
				motor_PulseHandle(motorCtl.runEn,motorCtl.runDir,motorCtl.runPwmFre,0);
			break;
			case 1://左 中 右都找到了	或则时间超了
				motorCtl.findStatus = 0XFF;
				if(findPosOneSideTicks < (findPosTotalTicksMax/2))
				{
					if(oneSided_LeftToRight <= (findPosTotalTicksMax/2))
					{
						if(oneSided_RightToLeft <= (findPosTotalTicksMax/2))
						{
							//success
							motorCtl.findStatus = 0xFE;
							//
							motorCtl.runEn = TRUE;
							motorCtl.runPwmFre = 50;
						}
						else//右边到左边时间不满足
						{
							motorCtl.runEn = TRUE;
							motorCtl.runPwmFre = 50;
						}
					}
					else//左边到右边时间不满足
					{
						motorCtl.runEn = TRUE;
						motorCtl.runPwmFre = 50;
					}
				}
				else//堵转
				{
					motorCtl.runEn = FALSE;
					motorCtl.runPwmFre = 50;
				}
				//
				motor_PulseHandle(motorCtl.runEn,motorCtl.runDir,motorCtl.runPwmFre,0);
			break;
			case 0xFE://success
				
			break;
			
			default:
				motorCtl.findStatus = 0XFF;
				motorCtl.runEn = FALSE;
				motorCtl.runPwmFre = 50;
				//
				motor_PulseHandle(motorCtl.runEn,motorCtl.runDir,motorCtl.runPwmFre,0);
			break;
		}
	}
}


















////////////////////


static UINT8 sample_complete = 0 ;

void app_MotorTestByKey(void)
{
	static UINT8 enPre = TRUE,dirPre=TRUE;
	static UINT32 Cycle[3][2];
	

	//dir == 1 ��������
	//dir == 0 ��������
	if(TRUE == pos_EventGet(MOTOR_POS_Left))//move to right
	{
		pos_EventClear(MOTOR_POS_Left);
		if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Left))
		{	
			motor_PulseHandle(1,0,gSystemPara.motorFre,0);
			Cycle[0][0] = Cycle[0][1] ;
			Cycle[0][1] = g_sys_ms_tick;
		}
	}
	if(TRUE == pos_EventGet(MOTOR_POS_Middle))//middle stop
	{
		pos_EventClear(MOTOR_POS_Middle);
		if((SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Middle)) && (1 == sample_complete))
		{	
			motor_PulseHandle(0,0,gSystemPara.motorFre,0);
			Cycle[1][0] = Cycle[1][1] ;
			Cycle[1][1] = g_sys_ms_tick;
		}
	}
	if(TRUE == pos_EventGet(MOTOR_POS_Right))//move left  MOTOR_POS_Right   MOTOR_POS_Middle  MOTOR_POS_Left
	{
		pos_EventClear(MOTOR_POS_Right);
		if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Right))
		{	
			motor_PulseHandle(1,1,gSystemPara.motorFre,0);
			Cycle[2][0] = Cycle[2][1] ;
			Cycle[2][1] = g_sys_ms_tick;
		}
	}










	if(TRUE == key_EventGet(STM32_KEY_1))//ʹ�� or ��ʹ��
	{
		key_EventClear(STM32_KEY_1);
		if(SYS_KEY_VALUED == key_FilterGet(STM32_KEY_1))
		{	
			if(enPre == TRUE)
			{
				motor_PulseHandle(1,0,10000,0);
			}
			else
			{
				motor_PulseHandle(0,0,10000,0);
			}
			enPre = ~enPre;
		}
	}
	if(TRUE == key_EventGet(STM32_KEY_2))// fre++
	{
		key_EventClear(STM32_KEY_2);
		if(SYS_KEY_VALUED == key_FilterGet(STM32_KEY_2))
		{	
			if(gSystemPara.motorFre >= 1000)
			{
				gSystemPara.motorFre -=1000;
			}
			motor_PulseHandle(1,0,gSystemPara.motorFre,0);
		}
	}
	if(TRUE == key_EventGet(STM32_KEY_3))// fre--
	{
		key_EventClear(STM32_KEY_3);
		if(SYS_KEY_VALUED == key_FilterGet(STM32_KEY_3))
		{	
			if(gSystemPara.motorFre >= 1000)
			{
				gSystemPara.motorFre +=1000;
			}
			motor_PulseHandle(1,0,gSystemPara.motorFre,0);
		}
	}
	if(TRUE == key_EventGet(STM32_KEY_4))//dir
	{
		key_EventClear(STM32_KEY_4);
		if(SYS_KEY_VALUED == key_FilterGet(STM32_KEY_4))
		{			
			if(dirPre == TRUE)
			{
				motor_PulseHandle(1,0,gSystemPara.motorFre,0);
			}
			else
			{
				motor_PulseHandle(1,1,gSystemPara.motorFre,0);
			}

			dirPre = ~dirPre;
		}
	}
}
/*
void app_MotorTest_ByScreen(void)
{
	static UINT16 motor_dir_pre=0;
	static UINT16 motor_dir_ctl_cnt=0,motor_dir_ctl_fre=0;
	static UINT32 diff=100;
	
	(void)motor_dir_ctl_fre;
	
	if(motor_dir_pre != gSystemPara.motorDir)
	{
		motor_dir_ctl_cnt++;
		if(motor_dir_ctl_cnt == 1 )
		{
			motor_pulse_output_set(0);
		}else if(motor_dir_ctl_cnt == diff )
		{
			motor_pulse_output_set(1);
			motor_PulseHandle(gSystemPara.motorEn,gSystemPara.motorDir,(gSystemPara.motorFre),0);
		}
		else if(motor_dir_ctl_cnt > diff)
		{
			motor_dir_ctl_cnt = 0 ;
			motor_dir_pre = gSystemPara.motorDir;
		}


	
	#if 0
		motor_dir_ctl_cnt++;
		if(motor_dir_ctl_cnt%diff == 0)
		{
			motor_dir_ctl_fre++;
			if(motor_dir_ctl_fre < 100)
			{
				motor_PulseHandle(gSystemPara.motorEn,motor_dir_pre,(((100-motor_dir_ctl_fre)/100)*gSystemPara.motorFre),0);
			}
			else if((motor_dir_ctl_fre >= 100) && (motor_dir_ctl_fre < 200))
			{
				motor_PulseHandle(gSystemPara.motorEn,gSystemPara.motorDir,(((motor_dir_ctl_fre-99)/100)*gSystemPara.motorFre),0);
			}
			else if(motor_dir_ctl_fre >= 200)
			{
				motor_dir_pre = gSystemPara.motorDir;
				motor_dir_ctl_fre = 0 ;
			}
		}
		#endif
	}
}



*/




UINT8 en,fre,dir;
UINT8 en_pre,fre_pre,dir_pre;

//==sys main function
void app_main_task()
{
	UINT8 hx711DataUpgrade = 0 ;
	static float weight = 0 ;
	static UINT16 FRE = 8000;
	(void)hx711DataUpgrade;
	(void)FRE;
	(void)weight;
	drv_iwdg_feed();

	//sys tick add
	g_sys_ms_tick++;

	//app run led
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

	//T5L contrl mainfunction
	sreenT5L_MainFunction();

	//电机控制
	app_MotorContrl();
}

