#ifndef __APP_T5L_CTRL_H__
#define __APP_T5L_CTRL_H__
#include "app_sdwe_ctrl.h"


#include "hal_uart.h"
#include "app_hx711_ctrl.h"


#define T5L_DMG_UART_DATA_LEN	(0X100)
#define SDWE_CRC16_EN 			(1)
//==(update:20220609):SDWE reserve
#define DMG_MIN_DIFF_OF_TWO_SEND_ORDER			(50)//50ms 

//================================================================================================

//==(update:20220609):address of set chanel number : 0->all chanel set  ; (1~8)->single chanel set
#define SCREEN_FUNC_SET_CHANEL_NUM				(0X2100)

//==(update:20210328):address of reset calibration of choice chanel number : 0->all chanel set  ; (1~x)->single chanel set
#define SCREEN_RESET_CALIBRATION_ADDRESS		(0X2101)

//==(update:20210328):address of remove weight
#define SCREEN_REMOVE_WEIGHT_ADDRESS			(0X2102)
#define SCREEN_REMOVE_WEIGHT_TRIG_VAL			(0XA55A)

//==(update:20210428):address of remove weight
#define SCREEN_JUNPTO_CALIBRATION_ADDRESS		(0X2103)
#define SCREEN_JUNPTO_CALIBRATION_TRIG_VAL		(2021)
#define SCREEN_JUNPTO_ACTIVE_TRIG_VAL			(1202)


//==(update:20210328):address of set point(weight value) of chanel : (0~9)-> point of chanel set (:g)
#define SCREEN_SET_CHANEL_POINT_ADDRESS						(0X2200)//0x2200~0x2209

//==(update:20210328):address of triger COLOR back to DMG : (0~9)-> COLOR of point of chanel set triger(val=0x00:white(not triger),val=0x01green(triger))
#define SCREEN_ASK_CHANEL_POINT_TRIG_BACK_COLOR_ADDRESS		(0X2300)//0x2300~0x2309

//==(update:20210328):address of triger sample back to DMG : (0~9)-> COLOR of point of chanel set triger(val=0x00:white(not triger),val=0x01green(triger))
#define SCREEN_ASK_CHANEL_POINT_TRIG_SAMPLE_DATA_ADDRESS	(0X2400)//0x2400~0x2409


//==(update:20210328):address of set point of chanel triger : (0~9)-> point of chanel set triger(val=0x12FE(DMG triger MCU))
#define SCREEN_SET_CHANEL_POINT_TRIG_ADDRESS				(0X2500)//0x2500~0x2509
#define SCREEN_SET_CHANEL_POINT_TRIG_VAL					(0X12FE)



//==(update:20220720):address of set voice printf en
#define SDWE_50_VOICE_PRINTF_CHOICE							(0X2600)
#define SDWE_50_VOICE_PRINTF_CHOICE_EXCHANGE_VLU			(0X2600)
#define SDWE_80_VOICE_PRINTF_CHOICE							(0X2601)
#define SDWE_80_VOICE_PRINTF_CHOICE_EXCHANGE_VLU			(0X2601)
#define SDWE_90_VOICE_PRINTF_CHOICE							(0X2602)
#define SDWE_90_VOICE_PRINTF_CHOICE_EXCHANGE_VLU			(0X2602)
#define SDWE_508090_VOICE_PRINTF_CHOICE_NUM					(3)

//==(update:20220720):address of min and max weight of each level
#define SDWE_LEVEL_200_MIN_WEIGHT							(0X2700)
#define SDWE_LEVEL_200_MAX_WEIGHT							(0X2701)
#define SDWE_LEVEL_300_MIN_WEIGHT							(0X2702)
#define SDWE_LEVEL_300_MAX_WEIGHT							(0X2703)
#define SDWE_LEVEL_400_MIN_WEIGHT							(0X2704)
#define SDWE_LEVEL_400_MAX_WEIGHT							(0X2705)

//==(update:20220721):address of trigger stop or run at real sample page
#define SDWE_TRIGGER_STOP_ROCK_ADDRESS						(0X2800)


//
#define SDWE_CYCLE_DATA_START_ADDRESS		(0X3000)//MCU -> SDWe

//
#define SDWE_BOOT_ANIMATION_ADDRESS			(0X3500)//开机动画 图标变量 vlu=0~16
#define SDWE_BACKANDSET_A_CONTRL_ADDRESS	(0X3501)//返回.结束.设置 开关指令 触发状态跳变 默认=0：返回 1：结束 2：设置
#define SDWE_BACKANDSET_B_CONTRL_ADDRESS	(0X3502)//返回.结束.设置 开关指令 触发状态跳变 默认=0：返回 1：结束 2：设置
#define SDWE_GJF_CONTRL_ADDRESS				(0X3503)//管夹阀 开关指令 触发状态跳变 默认=0：管夹阀开 1：管夹阀开
#define SDWE_RUN_CONTRL_ADDRESS				(0X3504)//运行.暂停 开关指令 触发状态跳变 默认=1：运行 0：暂停
#define SDWE_ENTER_REALTIME_CONTRL_ADDRESS	(0X3505)//进入实时界面 开关指令 200/300/400

//
#define SDWE_YUSHEZHONGLIANG_200_ADDRESS 	(0X3600)//SDWE->MCU
#define SDWE_YUSHEZHONGLIANG_300_ADDRESS 	(0X3601)//SDWE->MCU
#define SDWE_YUSHEZHONGLIANG_400_ADDRESS 	(0X3602)//SDWE->MCU
#define SDWE_YAOBAISHIJIAN_ADDRESS			(0X3603)//SDWE->MCU
#define SDWE_YAOBAIJIAODU_ADDRESS			(0X3604)//SDWE->MCU
#define SDWE_ML_G_BILU_ADDRESS 				(0X3605)//SDWE->MCU
#define SDWE_FMQKQD_ADDRESS 				(0X3606)//SDWE->MCU
#define SDWE_LIUSUDIAN_GAO_BAOJIN_ADDRESS 	(0X3607)//SDWE->MCU
#define SDWE_LIUSUDIAN_DI_BAOJIN_ADDRESS 	(0X3608)//SDWE->MCU
#define DMG_FUNC_SET_VOICE_NUM_ADDRESS		(0X3609)//SDWE->MCU
#define DMG_FUNC_SET_SCREEN_LIGHT_ADDRESS	(0X360A)//SDWE->MCU
#define DMG_FUNC_SET_ERR_RANGE_ADDRESS		(0X360B)//SDWE->MCU
#define DMG_FUNC_SET_MAX_RANGE_ADDRESS		(0X360C)//SDWE->MCU
#define DMG_FUNC_SET_MIN_RANGE_ADDRESS		(0X360D)//SDWE->MCU
#define DMG_FUNC_SET_ZERO_RANGE_ADDRESS		(0X360E)//SDWE->MCU
#define SDWE_GUANJIAFAGONGNENGKAIQI_ADDRESS (0X360F)//SDWE->MCU

//add at 20230206:cause team sample
#define SDWE_LIUSU_WINDOWN_RANGE_ADDRESS		(0X3610)//SDWE->MCU 流速窗口
#define SDWE_TUANCAI_YAOBAI_VLU_RANGE_ADDRESS	(0X3611)//SDWE->MCU 团采摇摆值
#define SDWE_TUANCAI_YAOBAI_TIME_RANGE_ADDRESS 	(0X3612)//SDWE->MCU 团才摇摆时间

//==是否开启管夹阀功能
#define SDWE_GUANJIAFAGONGNENGKAIQI_EN		(1)

//==硬件 主控 屏幕版本信息
#define SDWE_VERSION_ADDRERSS				(0X3700)
#define SDWE_VERSION_LENGTH					(3)


//
#define SDWE_RTC_TIME_SET_HH_ADDRESS		(0X3005)//SDWE->MCU
#define SDWE_RTC_TIME_SET_MM_ADDRESS		(0X3006)//SDWE->MCU
#define SDWE_RTC_TIME_SET_SS_ADDRESS		(0X3007)//SDWE->MCU



//例如把RTC设置为2015-06-01星期一18:56:00，串口发送以下指令： 
//A5 5A 0A 80 1F 5A 15 06 01 00 18 56 00 VGUS屏会自动换算星期，改写时间时星期可以写任意值。
//读取日历（YY:MM:DD:WW:HH:MM:SS）：A5 5A 03 81 20 07
#define SDWE_RTC_TIMER_ADDRESS				(0X20)//后面7个字节      	A5 5A 03 81 20 07   
#define SDWE_RTC_TIMER_LEN					(0X07)//后面7个字节      	A5 5A 03 81 20 07
#define RTC_TIME_HH_OFFSET					(0X04)
#define RTC_TIME_MM_OFFSET					(0X05)
#define RTC_TIME_SS_OFFSET					(0X06)

//语音音量地址 连续5个
#define SDWE_VOICE_PRINTF_SET_REG_ADDRESS	(0X50)
#define SDWE_VOICE_PRINTF_SET_REG_LEN		(5)

//语音音量地址 连续2个
#define SDWE_VOICE_SET_REG_ADDRESS			(0X53)
#define SDWE_VOICE_SET_REG_LEN				(2)

//蜂鸣器地址
#define SDWE_FENGMINGQI_SET_REG_ADDRESS		(0X02)
#define SDWE_FENGMINGQI_SET_REG_LEN			(1)


//页面跳转指令
#define SYS_SDWE_PAGE_REG_INDEX	(0X03)
#define SYS_SDWE_PAGE_REG_NUM	(0X02)//0X03 and 0X04







//==(update:20210328):value of reset calibration of choice chanel number:2021 reset calibration
#define DMG_FUNC_RESET_CALIBRATION_VAL	 		(2021)



//==(update:20211119):address of syspara entry
#define DMG_FUNC_JUNPTO_SYSPAR_ADDRESS		(0X2104)
#define DMG_FUNC_JUNPTO_SYSPAR_VAL			(1010)

//==(update:20220725):address of back to home page //num = 11
#define DMG_FUNC_JUNPTO_HOMEPAGE_ADDRESS	(0X2105)
#define DMG_FUNC_JUNPTO_HOMEPAGE_VAL		(0X2105)




//==(update:20210328):address of color back to DMG : (0~5)-> color of chanel(val:g)
#define DMG_FUNC_ASK_CHANEL_COLOR_ADDRESS		(0X3100)//0x3100~0x3105


//==(update:20210411):address of unit min max ...
#define DMG_FUNC_SET_UNIT_ADDRESS			(0X1000)//0x1000

//#define DMG_FUNC_SET_MIN_RANGE_ADDRESS		(0X100A)//0x100A
//#define DMG_FUNC_SET_MAX_RANGE_ADDRESS		(0X100B)//0x100B
//#define DMG_FUNC_SET_ERR_RANGE_ADDRESS		(0X100C)//0x100C
#define DMG_FUNC_SET_isCascade_ADDRESS		(0X100D)//0x100D
#define DMG_FUNC_SET_isLedIndicate_ADDRESS	(0X100E)//0x100E
#define DMG_FUNC_SET_COLOR_START_ADDRESS	(0X100F)//0x100F
#define DMG_FUNC_SET_COLOR_END_ADDRESS		(0X1012)//0x1012
//#define DMG_FUNC_SET_ZERO_RANGE_ADDRESS		(0X1013)//0x1013
//#define DMG_FUNC_SET_SCREEN_LIGHT_ADDRESS	(0X1014)//0x1014

#define DMG_FUNC_SET_VOICE_NUM_TOUCH_ADDRESS		(0X1015)//0x1015

//#define DMG_FUNC_SET_VOICE_NUM_ADDRESS		(0X1016)//0x1016

#define DMG_FUNC_SET_VOICE_SWITCH_ADDRESS	(0X1017)//0x1017
#define DMG_FUNC_SET_CAST_SWITCH_ADDRESS	(0X1018)//0x1018
#define DMG_FUNC_SET_FLASH_ERASEE_TIMES_ADDRESS	(0X1019)//0x1019

#define DMG_FUNC_SET_CAST_SWITCH_ADDRESS	(0X1018)//0x1018
#define DMG_FUNC_SET_FLASH_ERASEE_TIMES_ADDRESS	(0X1019)//0x1019

#define DMG_FUNC_MCU_VERSION_ADDRESS	(0X101A)//0x101A
#define DMG_FUNC_DIWEN_VERSION_ADDRESS	(0X101B)//0x101B


#define DMG_FUNC_MCUID_ADDRESS				(0X1500)//0x1500

#define DMG_FUNC_PASSORD_SET_ADDRESS		(0X1510)//0x1510


#define DMG_FUNC_Balancing_SET_ADDRESS		(0X1101)//0x1101
#define DMG_FUNC_Balancing_SET_VALUE		(0X1101)//0x1101

#define DMG_FUNC_Balancing_HOME_SET_ADDRESS	(0X1102)//0x1102
#define DMG_FUNC_Balancing_HOME_SET_VALUE	(0X1102)//0x1102

#define DMG_FUNC_Balancing_CLEARPAGE_SET_ADDRESS	(0X1103)//0x1103
#define DMG_FUNC_Balancing_CLEARPAGE_SET_VALUE		(0X1103)//0x1103

#define DMG_FUNC_HELP_TO_JUDGE_SET_ADDRESS	(0X1201)//0x1201

#define DMG_SYS_STATUS_OF_VOICE_PRINTF_00A1	(0X00A1)



//=============================================================SDWE address design ： 20220520

#define SYS_CTL_EVENT_INVALID		(0)
#define SYS_CTL_EVENT_VALID			(1)

#define SYS_CTL_REG_STATUS_INDEX	(0)//状态位置
#define SYS_CTL_REG_EVENT_INDEX 	(1)//事件位置
#define SYS_CTL_REG_NUM				(2)






typedef enum
{
	cmdWaitVoivePrint_forceRead  = 0 ,
	cmdWaitVoivePrint_waitResult = 1 ,	
	cmdWaitVoivePrint_max,
}enumSDWEcmdWaitVoivePrintType;


//DMG PageType
typedef enum DMGPageType
{
	DMG_FUNC_Balancing_6_PAGE = 49,
	DMG_FUNC_Balancing_6_HOME_PAGE = 57,
	DMG_FUNC_Balancing_12_PAGE = 55,
	DMG_FUNC_Balancing_12_HOME_PAGE =58,
	DMG_FUNC_Help_PAGE =59
}enumDMGPageType;

#define T5L_VOICE_MAX_PRINTF_NUM	6

typedef enum VoinceType
{
	T5L_VoiceTypeNum_0 = 0,
	T5L_VoiceTypeNum_1 = 1,
	T5L_VoiceTypeNum_2 = 2,
	T5L_VoiceTypeNum_3 = 3,
	T5L_VoiceTypeNum_4 = 4,
	T5L_VoiceTypeNum_5 = 5,
	T5L_VoiceTypeNum_6 = 6,
	T5L_VoiceTypeNum_7 = 7,
	T5L_VoiceTypeNum_8 = 8,
	T5L_VoiceTypeNum_9 = 9,
	T5L_VoiceTypeNum_10 = 10,
	T5L_VoiceTypeNum_11 = 11,
	T5L_VoiceTypeNum_12 = 12,
	VoiceTypeYu_13 = 13,
	VoiceTypePeiPin_14 = 14,
	VoiceTypeMax,
}tT5LVoinceType;


//ask calibration page data
typedef enum CalibrationAskParaType
{
	DMG_TRIGER_SAMPLE_OF_STATUS = 0 ,		/* trigerStarus */
	DMG_TRIGER_SAMPLE_OF_ASK_COLOR = 1 ,	/* back color of point*/
	DMG_TRIGER_SAMPLE_OF_AVG_SAMPLE = 2 ,	/* avg sample of point*/
	DMG_TRIGER_SAMPLE_OF_ASK_WEIGHT = 3 ,	/* set weight of point */
	DMG_TRIGER_SAMPLE_MAX_NUM
}enumCalibrationAskParaType;
	

typedef enum sdweRxFuncIdType
{
	/**< SDWE_RX_0X83 举例
	1：A5 5A 06 83 01 FF 01 00 01 ; 代表 add = 0x01ff(校准通道号选择) , len = 0x01 , data = 0x0001 
	解释：校准通道选择 (add=0x01ff , len = 1), data=0:所有通道 data=1~8:代表具体通道
	2:A5 5A 06 83 03 00 01 00 0A
	解释：对于通道下的校准点选择 (add=0x0300 , len = 1), data=1~11:具体点(十段总共11点)
	*/
	T5L_RX_FUN_HEAD2 = 0X5A, /**< SDWE HEAD1*/
	T5L_RX_FUN_HEAD1 = 0XA5, /**< SDWE HEAD2*/
	SDWE_RX_FUN_0X83 = 0X83, /**< SDWE 设置变量 下发给MCU*/
	SDWE_RX_FUN_NUM	 		 /**< SDWE 总数量*/
}enumsdweRxFuncIdType;


typedef enum
{
	cmdWriteSWDERegister = 0x80 ,
	cmdReadSWDERegister = 0x81 ,
	cmdWriteSWDEVariable = 0x82 ,
	cmdReadSWDEVariable = 0x83 ,
}enumSDWEcmdType;

typedef enum
{
	cmdPosHead1  = 0 ,//A5
	cmdPosHead2  = 1 ,//5A
	cmdPosDataLen= 2 ,//last data len
	cmdPosCommand= 3 ,//command position

	//=======MCU->SDWE order
	//read register 
	cmdPosRegReadAddress= 4 ,//reg address one byte position
	cmdPosRegReadLen= 5 ,//reg address one byte position
	//write register 
	cmdPosRegWriteAddress= 4 ,//reg address one byte position
	cmdPosRegWritesData= 5 ,//reg address one byte position

	//read varible 
	cmdPosVarReadAddress1= 4 ,//val address two byte position
	cmdPosVarReadAddress2= 5 ,//val address two byte position
	cmdPosVarReadLen= 6 ,//val address two byte position
	//write varible 
	cmdPosVarWriteAddress1= 4 ,//val address two byte position
	cmdPosVarWriteAddress2= 5 ,//val address two byte position
	cmdPosVarWriteData= 6 ,//val address two byte position

	//=======SDWE->MCU order
	//read register
	cmdPosRegAddress= 4 ,//reg address one byte position
	cmdPosReadRegAskLen= 5 ,//when read data ask data len position
	cmdPosRegData= 6 ,//reg address one byte position
	//read varible
	cmdPosVarAddress1= 4 ,//val address two byte position
	cmdPosVarAddress2= 5 ,//val address two byte position
	cmdPosReadVarAskLen= 6 ,//when read data ask data len position
	cmdPosVarData1= 7 ,//val address two byte position
}enumSDWEcmdPosType;


typedef enum
{
	SDWE_CYCLE_DATA_TOTAL_WEIGHT=0,
	SDWE_CYCLE_DATA_CUN_WEIGHT=1,
	SDWE_CYCLE_DATA_PERCENT=2,
	SDWE_CYCLE_DATA_PERCENT_TUBIAO=3,
	SDWE_CYCLE_DATA_LIUSU=4,
	SDWE_CYCLE_DATA_HH_1=5,
	SDWE_CYCLE_DATA_MM_1=6,
	SDWE_CYCLE_DATA_SS_1=7,
	SDWE_CYCLE_DATA_HH_2=8,
	SDWE_CYCLE_DATA_MM_2=9,
	SDWE_CYCLE_DATA_SS_2=10,
	SDWE_CYCLE_DATA_YUSE_WEIGHT=11,
	SDWE_CYCLE_DATA_KONGDAI_WEIGHT=12,
	SDWE_CYCLE_DATA_STATUS_FJS_A=13,
	SDWE_CYCLE_DATA_STATUS_FJS_B=14,
	SDWE_CYCLE_DATA_STATUS_FK_FG=15,
	SDWE_CYCLE_DATA_STATUS_ZT_YX=16,
	SDWE_CYCLE_DATA_STATUS_MAX
}enumSDWeCycleDataType;



typedef enum
{
	SDWeCurPage_ZhuYeJieMian=0,
	SDWeCurPage_KaiShiJieMian=11,
	SDWeCurPage_ShiShiJieMian=14,
	SDWeCurPage_CanShuSetJieMian=15,
	SDWeCurPage_JiaoZhunJieMianMian=17,
}enumSDWeCurPageType;


typedef enum
{
	SDWeFanHuiJieShuSeZhi_FanHui=0,
	SDWeFanHuiJieShuSeZhi_JieShu=1,
	SDWeFanHuiJieShuSeZhi_SheZhi=2,
	SDWeFanHuiJieShuSeZhi_KongBai=3,
}enumSDWeFanHuiJieShuSheZhiType;

typedef enum
{
	SDWeFaKaiFaGuan_FaGuan=0,
	SDWeFaKaiFaGuan_FaKai=1,
	SDWeFaKaiFaGuan_KongBai=2,
}enumSDWeFaKaiFaGuanType;

typedef enum
{
	SDWeZhanTingYuXing_ZhanTing=0,
	SDWeZhanTingYuXing_YunXing=1,
}enumSDWeZhanTingYuXingType;

typedef enum
{
	SDWeCaiJiDangWei_200=0,
	SDWeCaiJiDangWei_300,
	SDWeCaiJiDangWei_400,
	SDWeCaiJiDangWei_MAX,
}enumSDWeCaiJiDangWeiType;


typedef enum
{
	SDWeCtlStep_bootAnimation=0,
	SDWeCtlStep_GetRTC,
	SDWeCtlStep_RecvRTC,	
	SDWeCtlStep_SendDataToSDWE,	
	SDWeCtlStep_CycleHandle,
	SDWeCtlStep_MAX,
}enumSDWeCtlStepType;

//语音播报
typedef enum
{
	SDWEVoicePrintf_CaiJi_KaiShi=31,//采集开始
	SDWEVoicePrintf_CaiJi_ZanTing=32,//采集暂停
	SDWEVoicePrintf_CaiJi_50=33,//采集百分之五十
	SDWEVoicePrintf_CaiJi_80=34,//采集百分之八十
	SDWEVoicePrintf_CaiJi_90=35,//采集百分之九十
	SDWEVoicePrintf_DiDi_DiDi=36,	 //滴滴滴滴滴滴 ，大于95%时播报
	SDWEVoicePrintf_CaiJi_WanCheng=37,//采集完成
	
	SDWEVoicePrintf_Warn_LiuSu_High=38,//流速过快
	SDWEVoicePrintf_Warn_LiuSu_Low=39, //流速过慢
	
	SDWEVoicePrintf_QinFangDai=40,		 //请放袋
	SDWEVoicePrintf_KongDaiZhengChang=41,//空袋正常
	SDWEVoicePrintf_KongDaiYiChang=42,   //空袋异常
	
	SDWEVoicePrintf_MAX,
}enumSDWEVoicePrintfType;

#define T5L_VOICE_PRINTF_QUEUE_MAX		(10)

#define SDWE_CYCLE_DATA_ARR_INDEX_PRE	(0)
#define SDWE_CYCLE_DATA_ARR_INDEX_CUR	(1)
#define SDWE_CYCLE_DATA_ARR_NUM			(2)


/** 定义从机串口设备类型 */
typedef struct structSdweType
{
	UartDeviceType *pUartDevice;        	/**< 串口设备 */
	UINT8 	rxData[T5L_DMG_UART_DATA_LEN];	/**< 串口接收 */
	UINT8 	txData[T5L_DMG_UART_DATA_LEN];	/**< 串口发送 */
	UINT16	RxLength;						/**< 接收字节数 */
	UINT8 	RxFinishFlag;					/**< 接收完成标志 */

	//与屏幕通信
	UINT16  SetAdd;	/**< 地址 */
	INT16  	DataLen;/**< 数据长度 */
	INT16  	SetData;/**< 数据 */

	//应用参数
	UINT16 	sdweRemoveWeightTriger;	/**< 去皮标志 */
	UINT16 	sdwePointTriger;		/**< 点触发校准标志 */
	UINT16 	sdweResetTriger;		/**< 重新校准标志 */
	UINT16 	ResetTrigerValid;		/**< 重新校准有效 */
	UINT16 	sdweChanelChanged;		/**< 通道改变 */
	UINT16 	ColorClen;				/**< 通道改变时清颜色 */
	UINT16 	CalibrateChanel;		/**< 通道 */
	UINT16 	CalibratePoint;			/**< 校准点 */
	INT32 	CalibratePointArry[CHANEL_POINT_NUM];/**< 校准点数组 */

	//用于间隔两条指令间的延时
	UINT32	CurTick;		/**<  当前系统Tick*/
	UINT32	LastSendTick;	/**<  上次串口发送Tick*/

	//
	UINT16 	sdweJumpToCalitrationPage;/**< 跳转至校准页面 */
	UINT16	sdweJumpToHomePage;/**< 跳转至主页页面 */
	UINT16	sdweJumpToBanlingPage;/**< 跳转至配平页面 */
	UINT16 	sdweJumpActivePage;/**< 跳转至激活页面 */
	UINT16 	sdweJumpBalancing;/**< 跳转至配平页面 */
	UINT16 	sdweJumpBalancing_home;/**< 跳转至配平页面可去皮 */
	UINT16 	sdweJumpBalancing_cleanpagee;/**< 跳转至配平清爽页面 */
	UINT16 	sdweJumpToSysParaPage;/**< 跳转至参数页面 */
	UINT16 	sdweFreshScreenLight;/**< 刷新背光亮度 */


	//SDWE相关
	enumSDWeCtlStepType sdweCtlStep;/**< SDWE状态机 */
	UINT8   bootAnimation;/**< 开机动画 */

	enumSDWeCurPageType curPage;/**< 屏当前页面 */
	UINT16 takeDownOccure;/**< 拿下动作产生 */
	UINT16 putBackOccure;/**< 放回动作产生 */
	UINT16 curCaiJiDangWeiWeight;/**< 当前挡位 重量 */

	//event
	UINT16 backReturnASetContrl[SYS_CTL_REG_NUM];	/**< status , backReturnASetContrl event */
	UINT16 backReturnBSetContrl[SYS_CTL_REG_NUM];	/**< status , backReturnBSetContrl event */
	UINT16 gjfContrl[SYS_CTL_REG_NUM];				/**< status , gjfContrl event */
	UINT16 runContrl[SYS_CTL_REG_NUM];				/**< status , runContrl event */
	UINT16 enterRealTimeSetContrl[SYS_CTL_REG_NUM];	/**< status , enterRealTimeSetContrl event */
	UINT16 u16_508090SetPercentEvent;				/**< status , u16_508090SetPercentEvent event */
	UINT16 u16_TriggerStopRock;						/**< status , u16_TriggerStopRock event */
	
	//cycle data
	INT16 cycleData[SDWE_CYCLE_DATA_ARR_NUM][SDWE_CYCLE_DATA_STATUS_MAX];/**< 周期数据 */

	//rtc相关
	UINT8   rtcSet;			/**< RTC设置事件标志 */
	UINT8   getRTC;			/**< 获取屏幕RTC时间 标志*/
	UINT32  rtcTime_ms;		/**< RTC实时时间 ms */
	UINT32  rtcTimeSample_ms;/**< 采集时间 */
	UINT8  rtcTime[SDWE_RTC_TIMER_LEN];		/**< 当前RTC时间 十进制 */
	UINT8  rtcSetTime[SDWE_RTC_TIMER_LEN];	/**< 设置RTC时间 十进制 */

	//
	UINT8 cjyyts[SDWEVoicePrintf_MAX-SDWEVoicePrintf_CaiJi_KaiShi];/**< 采集语音 播报标志位 */
	UINT8 voicePrintf[T5L_VOICE_PRINTF_QUEUE_MAX];/*< 语音播报队列 */
	UINT8 voicePrintfOffset;/*< 语音播报队列 下标 */

	//流速
	INT32 liusuAvg;/*< 平均流速 */
	INT32 liusuCur;/*< 当前流速 */
	UINT8 liusuError;/*< 流速异常 */

	//采集完成标志
	UINT8 sampleComplete;/*< 采集完成标志 */
}T5LType;
extern T5LType g_T5L;

/** ModbusRtu设备默认配置 */
#define SDWeDataDefault   { \
	&g_UartDevice[UART_ZXP], /**< 串口设备 */\
	{0}, /**< 串口接收 */\
	{0}, /**< 串口发送 */\
	0,/**< 接收字节数 */\
	FALSE,/**< 接收完成标志 */\
	\
	0XFFFF,/**< 地址 */\
	0,/**< 数据长度 */\
	0,/**< 数据 */\
	\
	0,/**< 去皮标志 */\
	0,/**< 点触发校准标志 */\
	0,/**< 重新校准标志 */\
	0,/**< 重新校准有效 */\
	0,/**< 通道改变 */\
	0,/**< 通道改变时清颜色 */\
	88,/**< 通道 */\
	0,/**< 校准点 */\
	{0},/**< 校准点数组 */\
	\
	0,/**<  当前系统Tick*/\
	0,/**<  上次串口发送Tick*/\
	\
	0,/**< 跳转至校准页面 */\
	0,/**< 跳转至主页页面 */\
	0,/**< 跳转至配平页面 */\
	0,/**< 跳转至激活页面 */\
	0,/**< 跳转至配平页面 */\
	0,/**< 跳转至配平页面可去皮 */\
	0,/**< 跳转至配平清爽页面 */\
	0,/**< 跳转至参数页面 */\
	0,/**< 刷新背光亮度 */\
	\
	SDWeCtlStep_bootAnimation,/**< SDWE状态机 */\
	FALSE,/**< 开机动画 */\
	\
	SDWeCurPage_KaiShiJieMian,/**< 屏当前页面 */\
	FALSE,/**< 拿下动作产生 */\
	FALSE,/**< 放回动作产生 */\
	200,/**< 当前挡位 重量 */\
	\
	{0,0},/**< backReturnASetContrl event */\
	{0,0},/**< backReturnBSetContrl event */\
	{0,0},/**< gjfContrl event*/\
	{0,0},/**< runContrl event*/\
	{0,0},/**< enterRealTimeSetContrl event */\
	FALSE,/**< u16_508090SetPercentEvent event */\
	FALSE,/**< u16_TriggerStopRock event */\
	\
	{{0},{0}},/**< 周期数据*/\
	\
	FALSE,/**< RTC设置事件标志*/\
	FALSE,/**< 获取屏幕RTC时间 标志*/\
	0,/**< RTC实时时间 ms*/\
	0,/**< 采集时间*/\
	{0},/**< 当前RTC时间 十进制*/\
	{0},/**< 设置RTC时间 十进制*/\
	\
	{0},/**< 采集语音 播报标志位*/\
	{0},/**< 语音播报队列*/\
	0,/**< 语音播报队列 下标*/\
	\
	0,/**< 平均流速*/\
	0,/**< 当前流速*/\
	FALSE,/*< 流速异常 */\
	\
	FALSE,/*< 采集完成标志 */\
	}
#define T5L_INITIAL_COMPLETE		(0X12)
#define T5L_MAX_CHANEL_LEN			(HX711_CHANEL_NUM+HX711_CHANEL_NUM)
#define T5L_CHANEL_WEIGHT_NOT_EQUAL	(0XFF)
		
extern INT16 g_i16ColorOtherChanel[T5L_MAX_CHANEL_LEN];//T5L_CHANEL_WEIGHT_NOT_EQUAL:invalid
	
extern void app_gjf_handle(enumSDWeFaKaiFaGuanType status);
extern void color_clearAllColor(void);
extern void screenT5L_Init(void);
extern void sdweSetWeightBackColor(UINT8 seq,UINT8 color);
extern void pointSampleTrigerDataSet(UINT8 localChanel , UINT8 point , INT16 value);
extern void pointWeightTrigerDataSet(UINT8 localChanel , UINT8 point , INT16 value);
extern void pointWeightTrigerDataSet(UINT8 localChanel , UINT8 point , INT16 value);
extern void sreenSDWe_MainFunction(void);



extern T5LType g_T5L;


#define GET_SDWE_CYCLE_DATA_PTR()		(g_T5L.cycleData)
#define GET_SDWE_CUR_CYCLE_DATA_PTR()	(GET_SDWE_CYCLE_DATA_PTR()[SDWE_CYCLE_DATA_ARR_INDEX_CUR])
#define GET_SDWE_CUR_CYCLE_DATA(index)	GET_SDWE_CUR_CYCLE_DATA_PTR()[index]
#define SET_SDWE_CUR_CYCLE_DATA(index,data)	(GET_SDWE_CUR_CYCLE_DATA_PTR()[index] = data)

#define GET_SDWE_PRE_CYCLE_DATA_PTR()	(GET_SDWE_CYCLE_DATA_PTR()[SDWE_CYCLE_DATA_ARR_INDEX_PRE])
#define GET_SDWE_PRE_CYCLE_DATA(index)	GET_SDWE_PRE_CYCLE_DATA_PTR()[index]
#define SET_SDWE_PRE_CYCLE_DATA(index,data)	(GET_SDWE_PRE_CYCLE_DATA_PTR()[index] = data)


#endif