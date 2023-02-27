/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "typedefine.h"
#include "app_motor_ctrl.h"
#include "hal_pd_pulse.h"

#include "app_key_ctrl.h"
#include "app_t5l_ctrl.h"

#include "app_syspara.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
MotorContrlType motorCtl = MotorContrlDefault;

/*******************************************************************************
 * Functions : 管夹阀 开关控制
 ******************************************************************************/
void app_gjf_handle(enumSDWeFaKaiFaGuanType status)
{
	if(TRUE != gSystemPara.u16_kaiqi_guanjiafa_gongneng)//管夹阀功能无效
	{
		hal_gpio_set_do_low( (enumDoLineType)(STM32_Dcf_Ctrl) );//阀开
		SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FK_FG,SDWeFaKaiFaGuan_KongBai);//阀开/关 显示 = 空白
	}
	else//管夹阀功能有效
	{
		if(status == SDWeFaKaiFaGuan_FaKai)//阀开处理
		{
			hal_gpio_set_do_low( (enumDoLineType)(STM32_Dcf_Ctrl) );
			SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FK_FG,SDWeFaKaiFaGuan_FaKai);
		}
		else//否则阀关处理
		{
			hal_gpio_set_do_high( (enumDoLineType)(STM32_Dcf_Ctrl) );
			SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FK_FG,SDWeFaKaiFaGuan_FaGuan);
		}
	}
}


 /*******************************************************************************
  * Function : 电机控制 初始化
  ******************************************************************************/
 void app_MotorContrl_Init(void)
 { 
	//T形面积 = (上底 + 下底) * 高 / 2

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
	//0 = kAB * 0 + bAB
	//max = kAB * tAB + bAB
	motorCtl.kAB = motorCtl.pwmMAX_ms / (motorCtl.tAB);
	motorCtl.bAB = 0 ;


	// total_Midle = motorCtl.pwmMAX_ms * (motorCtl.tAC - motorCtl.tAB);
	// x * motorCtl.tBC_t_pulse[0][1] + (motorCtl.tAC - motorCtl.tAB - x) * motorCtl.tBC_t_pulse[1][1] = total_Midle
	// motorCtl.tBC_t_pulse[0][1] = (motorCtl.tAC - motorCtl.tAB)*motorCtl.tBC_t_pulse[1][1] - motorCtl.pwmMAX_ms * (motorCtl.tAC - motorCtl.tAB);

	motorCtl.tBC_t_pulse[0][0] = motorCtl.tAB;
	motorCtl.tBC_t_pulse[0][1] = (UINT16)motorCtl.pwmMAX_ms;
	motorCtl.tBC_t_pulse[0][2] = ((float)((UINT16)motorCtl.pwmMAX_ms+1) - motorCtl.pwmMAX_ms) * (motorCtl.tAC - motorCtl.tAB);

	motorCtl.tBC_t_pulse[1][0] = motorCtl.tAB + motorCtl.tBC_t_pulse[0][2];
	motorCtl.tBC_t_pulse[1][1] = (UINT16)motorCtl.pwmMAX_ms + 1;
	motorCtl.tBC_t_pulse[1][2] = (motorCtl.tAC - motorCtl.tAB) -  motorCtl.tBC_t_pulse[0][2];

	//行程C->D 的斜率
	//max = kCD * tAC + bCD
	//0 = kCD * tAD + bCD
	motorCtl.kCD = -(motorCtl.pwmMAX_ms / (motorCtl.tAD - motorCtl.tAC));
	motorCtl.bCD = -motorCtl.kCD * motorCtl.tAD;

	motorCtl.runEn = FALSE;
	motorCtl.runDir = 0 ;
	motorCtl.runPwmFre = 50;

	motor_PulseHandle(motorCtl.runEn,motorCtl.runDir,motorCtl.runPwmFre,0);
 }
 
 
 /*******************************************************************************
  * Function : 设置电机运行模式
  ******************************************************************************/
 void app_SetMotorContrlMode(enumMotorCtrlStatusType setStatus)
 {
	 motorCtl.motorCtrlStatus = setStatus;
 }
 
 
 /*******************************************************************************
  * Function : 电机实时运行控制
  ******************************************************************************/
void app_MotorContrl(void)
{
	static UINT8 middleAppear = FALSE , middleAppearDelay = 150;
	static UINT16 stopMiddleAtStart = TRUE,stopMiddleAtStartCnt=0;

	//==上电3.5秒后 电机停中间
	if((TRUE == stopMiddleAtStart) && (stopMiddleAtStartCnt++ >= SYS_POWER_REDAY_TIME))
	{
		stopMiddleAtStart = FALSE;
		motorCtl.motorCtrlStatus = MotorCtrlStatus_EN_StopToMiddle;//电机停中间
	}

	switch(motorCtl.motorCtrlStatus)
	{
		//==================================================================================
		//                                  《电机不使能》 	   
		//==================================================================================
		case MotorCtrlStatus_DISEN:
			motor_pulse_output_set(FALSE);//停止PWM timer 停止PWM 电机停止
			motorCtl.runEn = FALSE;
		break;
		//==================================================================================
		// 								《电机使能》		
		//==================================================================================
		case MotorCtrlStatus_EN:
			motor_pulse_output_set(FALSE);//停止PWM timer 停止PWM 电机停止
			motorCtl.runEn = TRUE;
		break;
		//==================================================================================
		// 								《电机停止在中间：开始》		
		//==================================================================================
		case MotorCtrlStatus_EN_StopToMiddle:
			motor_pulse_output_set(TRUE);//开始PWM的timer计数
			motorCtl.runEn = TRUE;
			motorCtl.runPwmFre = motorCtl.runPwmFreFixed / 6;//以1/6的固定频率转动 找中间限位
			middleAppear = FALSE;
			//
			motorCtl.motorCtrlStatus = MotorCtrlStatus_EN_StopToMiddle1;
		break;
		//==================================================================================
		//								 《电机停止在中间：检测》		 
		//==================================================================================
		case MotorCtrlStatus_EN_StopToMiddle1:	  
			if((FALSE == middleAppear) && (TRUE == pos_EventGet(MOTOR_POS_Middle)))//通过限位开关判断
			{
				pos_EventClear(MOTOR_POS_Middle);				  
				if(SYS_POS_VALUED_1 == pos_FilterGet(MOTOR_POS_Middle))//中间限位
				{
					middleAppear = TRUE;
					middleAppearDelay = 150;
				}
			}else if((TRUE == middleAppear) && (--middleAppearDelay == 0))
			{
				motorCtl.motorCtrlStatus = MotorCtrlStatus_EN;//电机只使能
			}
		break;
		//==================================================================================
		//								 《电机左右限位运行：开始》		 
		//==================================================================================
		case MotorCtrlStatus_EN_NormalRun_WitMaxPos:
			motor_pulse_output_set(TRUE);//开始PWM的timer计数
			motorCtl.runEn = TRUE;
			motorCtl.runPwmFre = gSystemPara.u32_MotorRockFre;
			//
			motorCtl.motorCtrlStatus = MotorCtrlStatus_EN_NormalRun_WitMaxPos1;
		break;
		//==================================================================================
		// 								《电机左右限位运行：执行》 		
		//==================================================================================
		case MotorCtrlStatus_EN_NormalRun_WitMaxPos1: 
			//==实时采集界面
			if(SDWeCurPage_ShiShiJieMian == g_T5L.curPage)
			{
				//==采集到xx% ~100% 且水平位置出现
				if((GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) >= gSystemPara.u16_fmqkqsj)
					&& (GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) < 100)	)
				{
					motorCtl.motorCtrlStatus = MotorCtrlStatus_EN_StopToMiddle;//电机停中间
				}				 
			}
		break;
		default:
		break;
	}

	//==================================================================================
	//1.左限位开关检测时	： 反向 ，清零curStep_Ms
	//2.右限位开关检测时	： 反向 ，清零curStep_Ms
	//3.周期设置驱动器		: 使能 ，方向 ，脉冲频率
	//==================================================================================
	if(TRUE == pos_EventGet(MOTOR_POS_Left))//左限位开关事件获取
	{
		pos_EventClear(MOTOR_POS_Left);
		if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Left))//左限位：有效
		{	 
			motorCtl.runDir = 0;//立即反向
			motorCtl.curStep_Ms = 0 ;
		}
	}
	else if(TRUE == pos_EventGet(MOTOR_POS_Right))//右限位开关事件获取
	{
		pos_EventClear(MOTOR_POS_Right);
		if(SYS_POS_VALUED == pos_FilterGet(MOTOR_POS_Right))//右限位：有效
		{	 
			motorCtl.runDir = 1;//立即反向
			motorCtl.curStep_Ms = 0 ;
		}
	}
	motor_PulseHandle(motorCtl.runEn,motorCtl.runDir,motorCtl.runPwmFre,0); 
}
 
