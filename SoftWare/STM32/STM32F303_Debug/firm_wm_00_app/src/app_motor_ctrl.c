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
	if(status == SDWeFaKaiFaGuan_FaKai)
	{
		hal_gpio_set_do_low( (enumDoLineType)(STM32_Dcf_Ctrl) );
	}
	else
	{
		hal_gpio_set_do_high( (enumDoLineType)(STM32_Dcf_Ctrl) );
	}
	//
	g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FK_FG] = status;
	//当取消管夹阀功能时 取消显示管夹阀图标
	if(TRUE != gSystemPara.u16_kaiqi_guanjiafa_gongneng)
	{
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FK_FG] = (UINT16)SDWeFaKaiFaGuan_KongBai;//阀开/关 = 空白
	}
}


 /*******************************************************************************
  * Function : 电机控制 初始化
  ******************************************************************************/
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
				 motorCtl.runPwmFre = 1000 * (motorCtl.pwmMAX_ms +	motorCtl.kCD*(motorCtl.curStep_Ms-motorCtl.tAC));
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
 
			 //g_T5L.sampleComplet = FALSE;
			 
			 //开管夹阀
			 //app_gjf_handle(SDWeFaKaiFaGuan_FaKai);
 
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
			 motorCtl.runPwmFre = motorCtl.runPwmFreFixed / 6;
 
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
 
