#ifndef __APP_T5L_CTRL_H__
#define __APP_T5L_CTRL_H__
#include "app_sdwe_ctrl.h"


#include "hal_uart.h"
#include "app_hx711_ctrl.h"


#define T5L_DMG_UART_DATA_LEN	(0X100)
#define SDWE_CRC16_EN 			(1)
//================================================================================================
//==(update:20210328):DIWEN reserve (uodate to v3:2021.03.26)
#define DMG_MIN_DIFF_OF_TWO_SEND_ORDER			(25)//25ms 
#define DMG_DATA_HOLD_TIME						(250)//250ms

//==(update:20210328):address of set chanel number : 0->all chanel set  ; (1~8)->single chanel set
#define DMG_FUNC_SET_CHANEL_NUM					(0X2100)
//==(update:20210328):address of reset calibration of choice chanel number : 0->all chanel set  ; (1~x)->single chanel set
#define DMG_FUNC_RESET_CALIBRATION_ADDRESS		(0X2101)
//==(update:20210328):value of reset calibration of choice chanel number:2021 reset calibration
#define DMG_FUNC_RESET_CALIBRATION_VAL	 		(2021)

//==(update:20210328):address of remove weight
#define DMG_FUNC_REMOVE_WEIGHT_ADDRESS			(0X2102)
#define DMG_FUNC_REMOVE_WEIGHT_VAL				(0XA55A)

//==(update:20210428):address of remove weight
#define DMG_FUNC_JUNPTO_CALIBRATION_ADDRESS		(0X2103)
#define DMG_FUNC_JUNPTO_CALIBRATION_VAL			(2021)
#define DMG_FUNC_JUNPTO_ACTIVE_VAL				(1202)

//==(update:20211119):address of syspara entry
#define DMG_FUNC_JUNPTO_SYSPAR_ADDRESS		(0X2104)
#define DMG_FUNC_JUNPTO_SYSPAR_VAL			(1010)

//==(update:20210328):address of set point(weight value) of chanel : (0~9)-> point of chanel set (:g)
#define DMG_FUNC_SET_CHANEL_POINT_ADDRESS		(0X2200)//0x2200~0x2209

//==(update:20210328):address of set point of chanel triger : (0~9)-> point of chanel set triger(val=0x12FE(DMG triger MCU))
#define DMG_FUNC_SET_CHANEL_POINT_TRIG_ADDRESS	(0X2500)//0x2500~0x2509
#define DMG_FUNC_SET_CHANEL_POINT_TRIG_VAL		(0X12FE)


//==(update:20210328):address of triger COLOR back to DMG : (0~9)-> COLOR of point of chanel set triger(val=0x00:white(not triger),val=0x01green(triger))
#define DMG_FUNC_ASK_CHANEL_POINT_TRIG_BACK_COLOR_ADDRESS	(0X2300)//0x2300~0x2309

//==(update:20210328):address of triger sample back to DMG : (0~9)-> COLOR of point of chanel set triger(val=0x00:white(not triger),val=0x01green(triger))
#define DMG_FUNC_ASK_CHANEL_POINT_TRIG_SAMPLE_DATA_ADDRESS	(0X2400)//0x2400~0x2409

//==(update:20210328):address of weight back to DMG : (0~5)-> weight of chanel(val:g)
#define DMG_FUNC_ASK_CHANEL_WEIGHT_ADDRESS		(0X3000)//0x3000~0x3005
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

//at BALANCING Page , auto to judge the remaining chanel weight minus
//to help user to caculate
//1.find out the remaining chanel
//2.find out the closed group(minus was smallest)
//3.send to DIWEN Screen to display
#define DIFF_JUDGE_GROUP_NUM	(2)//2 group display 
#define DIFF_JUDGE_DATA_NUM		(3)//num1 num2 minus


#define DIFF_JUDGE_GROUP_NUM_SLAVE1	(6)//2 group display 
#define DIFF_JUDGE_DATA_NUM_SLAVE1	(3)//num1 num2 minus

#define DIFF_TO_DIWEN_DATA_LEN		(DIFF_JUDGE_GROUP_NUM_SLAVE1*DIFF_JUDGE_DATA_NUM_SLAVE1)




//=============================================================SDWE address design ??? 20220520

#define SYS_CTL_EVENT_INVALID		(0)
#define SYS_CTL_EVENT_VALID			(1)

#define SYS_CTL_REG_STATUS_INDEX	(0)//????????????
#define SYS_CTL_REG_EVENT_INDEX 	(1)//????????????
#define SYS_CTL_REG_NUM				(2)

//MCU -> SDWe
#define SDWE_CYCLE_DATA_START_ADDRESS		(0X3000)
#define SDWE_BOOT_ANIMATION_ADDRESS			(0X3500)//???????????? ???????????? vlu=0~16


//SDWe -> MCU
#define SDWE_BACKANDSET_A_CONTRL_ADDRESS	(0X3501)//??????.??????.?????? ???????????? ?????????????????? ??????=0????????? 1????????? 2?????????
#define SDWE_BACKANDSET_B_CONTRL_ADDRESS	(0X3502)//??????.??????.?????? ???????????? ?????????????????? ??????=0????????? 1????????? 2?????????
#define SDWE_GJF_CONTRL_ADDRESS				(0X3503)//????????? ???????????? ?????????????????? ??????=0??????????????? 1???????????????
#define SDWE_RUN_CONTRL_ADDRESS				(0X3504)//??????.?????? ???????????? ?????????????????? ??????=1????????? 0?????????
#define SDWE_ENTER_REALTIME_CONTRL_ADDRESS	(0X3505)//?????????????????? ???????????? 200/300/400


#define SDWE_RTC_TIME_SET_HH_ADDRESS		(0X3703)
#define SDWE_RTC_TIME_SET_MM_ADDRESS		(0X3704)
#define SDWE_RTC_TIME_SET_SS_ADDRESS		(0X3705)



#define SDWE_WEIGHT_JIAOZHUN_ADDRESS		(0X2103)


//?????????RTC?????????2015-06-01?????????18:56:00?????????????????????????????? 
//A5 5A 0A 80 1F 5A 15 06 01 00 18 56 00 VGUS?????????????????????????????????????????????????????????????????????
//???????????????YY:MM:DD:WW:HH:MM:SS??????A5 5A 03 81 20 07
#define SDWE_RTC_TIMER_ADDRESS				(0X20)//??????7?????????      	A5 5A 03 81 20 07   
#define SDWE_RTC_TIMER_LEN					(0X07)//??????7?????????      	A5 5A 03 81 20 07   

#define SDWE_VOICE_PRINTF_SET_REG_ADDRESS	(0X50)
#define SDWE_VOICE_PRINTF_SET_REG_LEN		(5)

#define SDWE_VOICE_SET_REG_ADDRESS			(0X53)
#define SDWE_VOICE_SET_REG_LEN				(2)


#define SDWE_FENGMINGQI_SET_REG_ADDRESS		(0X02)
#define SDWE_FENGMINGQI_SET_REG_LEN			(1)


#define SDWE_YUSHEZHONGLIANG_200_ADDRESS 	(0X3600)
#define SDWE_YUSHEZHONGLIANG_300_ADDRESS 	(0X3601)
#define SDWE_YUSHEZHONGLIANG_400_ADDRESS 	(0X3602)

#define SDWE_YAOBAISHIJIAN_ADDRESS			(0X3603)
#define SDWE_YAOBAIJIAODU_ADDRESS			(0X3604)

#define SDWE_ML_G_BILU_ADDRESS 				(0X3605)
#define SDWE_FMQKQD_ADDRESS 				(0X3606)

#define SDWE_LIUSUDIAN_GAO_BAOJIN_ADDRESS 	(0X3607)
#define SDWE_LIUSUDIAN_DI_BAOJIN_ADDRESS 	(0X3608)

#define DMG_FUNC_SET_VOICE_NUM_ADDRESS		(0X3609)

#define DMG_FUNC_SET_SCREEN_LIGHT_ADDRESS	(0X360A)

#define DMG_FUNC_SET_ERR_RANGE_ADDRESS		(0X360B)

#define DMG_FUNC_SET_MAX_RANGE_ADDRESS		(0X360C)
#define DMG_FUNC_SET_MIN_RANGE_ADDRESS		(0X360D)
#define DMG_FUNC_SET_ZERO_RANGE_ADDRESS		(0X360E)

#define SDWE_GUANJIAFAGONGNENGKAIQI_ADDRESS (0X360F)
#define SDWE_GUANJIAFAGONGNENGKAIQI_EN		(1)



#define SYS_SDWE_PAGE_REG_INDEX	(0X03)
#define SYS_SDWE_PAGE_REG_NUM	(0X02)//0X03 and 0X04



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
	/**< SDWE_RX_0X83 ??????
	1???A5 5A 06 83 01 FF 01 00 01 ; ?????? add = 0x01ff(?????????????????????) , len = 0x01 , data = 0x0001 
	??????????????????????????? (add=0x01ff , len = 1), data=0:???????????? data=1~8:??????????????????
	2:A5 5A 06 83 03 00 01 00 0A
	?????????????????????????????????????????? (add=0x0300 , len = 1), data=1~11:?????????(????????????11???)
	*/
	T5L_RX_FUN_HEAD2 = 0X5A, /**< SDWE HEAD1*/
	T5L_RX_FUN_HEAD1 = 0XA5, /**< SDWE HEAD2*/
	SDWE_RX_FUN_0X83 = 0X83, /**< SDWE ???????????? ?????????MCU*/
	SDWE_RX_FUN_NUM	 		 /**< SDWE ?????????*/
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
	SDWE_CYCLE_DATA_CUN_WEIGHT,
	SDWE_CYCLE_DATA_PERCENT,
	SDWE_CYCLE_DATA_PERCENT_TUBIAO,
	SDWE_CYCLE_DATA_LIUSU,
	SDWE_CYCLE_DATA_HH_1,
	SDWE_CYCLE_DATA_MM_1,
	SDWE_CYCLE_DATA_SS_1,
	SDWE_CYCLE_DATA_HH_2,
	SDWE_CYCLE_DATA_MM_2,
	SDWE_CYCLE_DATA_SS_2,
	SDWE_CYCLE_DATA_YUSE_WEIGHT,
	SDWE_CYCLE_DATA_KONGDAI_WEIGHT,
	SDWE_CYCLE_DATA_STATUS_FJS_A,
	SDWE_CYCLE_DATA_STATUS_FJS_B,
	SDWE_CYCLE_DATA_STATUS_FK_FG,
	SDWE_CYCLE_DATA_STATUS_ZT_YX,
	SDWE_CYCLE_DATA_STATUS_MAX
}enumSDWeCycleDataType;



typedef enum
{
	SDWeCurPage_ZhuYeJieMian=0,
	SDWeCurPage_KaiShiJieMian=11,
	SDWeCurPage_ShiShiJieMian=14,
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

//????????????
typedef enum
{
	SDWEVoicePrintf_CaiJi_KaiShi=31,//1???????????????
	SDWEVoicePrintf_CaiJi_80=32,//2????????????????????????
	SDWEVoicePrintf_CaiJi_90=33,//3????????????????????????
	SDWEVoicePrintf_CaiJi_WanCheng=34,//4???????????????
	SDWEVoicePrintf_Warn_LiuSu_High=35,//5???????????????????????????
	SDWEVoicePrintf_Warn_LiuSu_Low=36,//6???????????????????????????
	SDWEVoicePrintf_DiDi_DiDi=37,//7??????????????????????????????????????????
	SDWEVoicePrintf_QinFangDai=38,//7??????????????????????????????????????????
	SDWEVoicePrintf_MAX,
}enumSDWEVoicePrintfType;



#define SDWE_CYCLE_DATA_ARR_INDEX_PRE	(0)
#define SDWE_CYCLE_DATA_ARR_INDEX_CUR	(1)
#define SDWE_CYCLE_DATA_ARR_NUM			(2)


/** ?????????????????????????????? */
typedef struct structSdweType
{
	enumSDWeCtlStepType sdweCtlStep;
	UINT8   rtcSet;
	UINT32  rtcTime_ms;
	UINT32  rtcTime_s;
	UINT8   getRTC;
	UINT8   bootAnimation;
	UINT8   sendParaToSDWE;
	UINT8 	sendSdweInit;
	UINT8 	readSdweInit;
	UartDeviceType *pUartDevice;        /**< ???????????? */
	UINT8 	version;//SDWE version
	UINT8 	allowCompare;
	UINT8 	rxData[T5L_DMG_UART_DATA_LEN];
	UINT8 	txData[T5L_DMG_UART_DATA_LEN];
	UINT16	RxLength;					/**< ??????????????? */
	UINT8 	RxFinishFlag;				/**< ?????????????????? */
	
	UINT16  SetAdd;/**< ?????? */
	INT16  	DataLen;/**< ???????????? */
	INT16  	SetData;/**< ?????? */

	UINT16 	sdweRemoveWeightTriger;/**< ?????? */
	UINT16 	sdwePointTriger;/**< ??????????????? */
	UINT16 	sdweResetTriger;/**< ???????????? */
	UINT16 	ResetTrigerValid;/**< ?????????????????? */
	UINT16 	sdweChanelChanged;/**< ???????????? */
	UINT16 	ColorClen;/**< ???????????????????????? */
	UINT16 	CalibrateChanel;/**< ?????? */
	UINT16 	CalibratePoint;/**< ????????? */
	INT32 	CalibratePointArry[CHANEL_POINT_NUM];/**< ??????????????? */
	UINT32	CurTick;
	UINT32	LastSendTick;
	UINT16 	sdweJumpToCalitrationPage;/**< ????????????????????? */
	UINT16	sdweJumpToHomePage;
	UINT16	sdweJumpToBanlingPage;
	UINT16 	sdweJumpActivePage;/**< ????????????????????? */
	UINT16 	sdweJumpBalancing;/**< ????????????????????? */
	UINT16 	sdweJumpBalancing_home;/**< ?????????????????????????????? */
	UINT16 	sdweJumpBalancing_cleanpagee;/**< ??????????????????????????? */
	UINT16 	sdweJumpToSysParaPage;/**< ????????????????????? */
	UINT16 	sdweFreshScreenLight;/**< ?????????????????? */

	enumSDWeCurPageType curPage;/**< ??????????????? */
	UINT16 sampleComplet;/**< ???????????? */
	UINT16 curCaiJiDangWeiWeight;/**< ???????????? ?????? */

	UINT16 backReturnASetContrl[SYS_CTL_REG_NUM];//status , event
	UINT16 backReturnBSetContrl[SYS_CTL_REG_NUM];//status , event
	UINT16 gjfContrl[SYS_CTL_REG_NUM];//status , event
	UINT16 runContrl[SYS_CTL_REG_NUM];//status , event
	UINT16 enterRealTimeSetContrl[SYS_CTL_REG_NUM];//status , event

	UINT16 cycleData[SDWE_CYCLE_DATA_ARR_NUM][SDWE_CYCLE_DATA_STATUS_MAX];
	UINT8  rtcTime[7];
	UINT8 cjyyts[SDWEVoicePrintf_MAX-SDWEVoicePrintf_CaiJi_KaiShi];//???????????? ???????????????
}T5LType;
extern T5LType g_T5L;

/** ModbusRtu?????????????????? */
#define T5LDataDefault   { \
	SDWeCtlStep_bootAnimation,\
	FALSE,\
	0,\
	0,\
	FALSE,\
	FALSE,\
	0,\
	0,\
	0,\
	&g_UartDevice[UART_ZXP], \
	0,\
	FALSE,\
	{0}, \
	{0}, \
	0,\
	0,\
	0XFFFF,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	88,\
	0,\
	{0},\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	SDWeCurPage_KaiShiJieMian,\
	FALSE,\
	200,\
	{0,0},\
	{0,0},\
	{0,0},\
	{0,0},\
	{0,0},\
	{{0},{0}},\
	{0},\
	{0},\
	}
#define T5L_INITIAL_COMPLETE		(0X12)
#define T5L_MAX_CHANEL_LEN			(HX711_CHANEL_NUM+HX711_CHANEL_NUM)
#define T5L_CHANEL_WEIGHT_NOT_EQUAL	(0XFF)
	
	extern void app_gjf_handle(enumSDWeFaKaiFaGuanType status);

	
extern INT16 g_i16ColorOtherChanel[T5L_MAX_CHANEL_LEN];//T5L_CHANEL_WEIGHT_NOT_EQUAL:invalid
	
extern void color_clearAllColor(void);
extern void screenT5L_Init(void);
extern void sdweSetWeightBackColor(UINT8 seq,UINT8 color);
extern void pointSampleTrigerDataSet(UINT8 localChanel , UINT8 point , INT16 value);
extern void pointWeightTrigerDataSet(UINT8 localChanel , UINT8 point , INT16 value);
extern void sdwe_VoicePrintfPush(tT5LVoinceType u8Voice1 ,tT5LVoinceType u8Voice2);
extern void pointWeightTrigerDataSet(UINT8 localChanel , UINT8 point , INT16 value);
extern void sreenT5L_MainFunction(void);
extern void writeHelpDataFromCom(UINT8 *pHelpData,UINT8 len);
extern void readHelpDataFromSys(UINT8 *pHelpData,UINT8 len);
extern void t5lDisPlayDataClear(void);
extern void readWeightDataFromSys(UINT8 *pWeightData,UINT8 len);
extern void readColorDataFromSys(UINT8 *pColorData,UINT8 len);
extern void writeWeightDataFromCom(UINT8 *pWeightData,UINT8 len);
extern void writeColorDataFromCom(UINT8 *pColorData,UINT8 len);
extern UINT8 screenT5L_OutputVoice(UINT8 voiceId);
#endif
