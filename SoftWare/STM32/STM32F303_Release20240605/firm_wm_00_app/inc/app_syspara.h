#ifndef __APP_SYSPARA_H__
#define __APP_SYSPARA_H__
#include "typedefine.h"
#include "app_led_ctrl.h"
#include "app_sdwe_ctrl.h"
#include "app_t5l_ctrl.h"

//==(update:20210606):the SYS data need store : SECTOR1 and SECTOR2
#define DMG_TRIGER_SAVE_SECOTOR_1			(0X01)
#define DMG_TRIGER_SAVE_SECOTOR_2			(0X02)


//system parameter
typedef enum HX711SystemParaType
{
	HX711SystemPara_UNIT = 0,  		/**< HX711  系统设置-单位 */
	HX711SystemPara_MIN_RANGE = 1,  /**< HX711  系统设置-最小量程 */
	HX711SystemPara_MAX_RANGE = 2,  /**< HX711  系统设置-最大量程 */
	HX711SystemPara_ERR_RANGE = 3,	/**< HX711	系统设置-误差 */
	HX711SystemPara_CASCADE = 4,  	/**< HX711  系统设置-级联 */	
	HX711SystemPara_LED_DIS_EN = 5,	/**< HX711	系统设置-LED指示 */
	HX711SystemPara_COLOR1 = 6,		/**< HX711	系统设置-颜色1 */
	HX711SystemPara_COLOR2 = 7,		/**< HX711	系统设置-颜色2 */
	HX711SystemPara_COLOR3 = 8,		/**< HX711	系统设置-颜色3 */
	HX711SystemPara_COLOR4 = 9,		/**< HX711	系统设置-颜色4 */	
	HX711SystemPara_ZERO_RANGE = 10,/**< HX711	零点范围 */ 
	HX711SystemPara_ScreenVoiceSwitch = 11,/**< HX711	语音开关 */ 
	HX711SystemPara_ScreenCastMode = 12,/**< HX711	级联模式 */ 
	HX711SystemPara_FlashEraseTimes = 13,/**< HX711	FLASH擦写次数 */ 
	HX711SystemPara_McuVersion = 14,/**< MCU 软件版本 */ 
	HX711SystemPara_DiVenVersion = 15,/**< 迪文屏 软件版本 */ 
	HX711SystemPara_VoiceNum = 16,/**< 音量参数 */ 
	HX711SystemPara_VoiceNumTouch = 17,/**< 音量参数 触控*/ 
	HX711SystemPara_ScreenLight = 18,/**< 屏幕背光亮度 */	

	//
	HX711SystemPara_yusheWeight_200 = 19,/**< 200 档预设重量 */
	HX711SystemPara_yusheWeight_300 = 20,/**< 300 档预设重量 */
	HX711SystemPara_yusheWeight_400 = 21,/**< 400 档预设重量 */
	HX711SystemPara_u16_ml_g_bilv = 22,//ml与g比率
	HX711SystemPara_u16_fmqkqsj= 23,//蜂鸣器开始时间
	HX711SystemPara_u16_liusu_high_warn= 24,//流速高 报警点
	HX711SystemPara_u16_liusu_low_warn= 25,//流速低 报警点
	HX711SystemPara_motorAge = 26,//摇摆角度
	HX711SystemPara_tAD = 27,//摇摆时间
	HX711SystemPara_KaiQiGuanJiaFaGongNeng = 28,//开启管夹阀功能

	//==20220720添加9个参数：各挡位最小值，最大值；语音开关使能
	//各挡位最小值，最大值
	HX711SystemPara_200LevelWeight_Min=29,//200档最小值
	HX711SystemPara_200LevelWeight_Max=30,//200档最大值
	HX711SystemPara_300LevelWeight_Min=31,//300档最小值
	HX711SystemPara_300LevelWeight_Max=32,//300档最大值
	HX711SystemPara_400LevelWeight_Min=33,//400档最小值
	HX711SystemPara_400LevelWeight_Max=34,//400档最大值
	//语音开关使能
	HX711SystemPara_50PercentVoicePrintf=35,//50%语音播报
	HX711SystemPara_80PercentVoicePrintf=36,//80%语音播报
	HX711SystemPara_90PercentVoicePrintf=37,//90%语音播报
	//电机摇摆频率
	HX711SystemPara_MotorRockFre=38,//电机摇摆频率
	
	//流速窗口
	HX711SystemPara_u32_LiuSu_Window=39,//流速窗口
	//摇摆值（团采）
	HX711SystemPara_u32_YaoBaiVlu_TuanCai=40,//摇摆值（团采）
	//摇摆时间（团采）low
	HX711SystemPara_u32_YaoBaiastTime_TuanCai=41,//摇摆时间（团采）
	//流速报警滤波时间
	HX711SystemPara_u32_FastWarnFilterTime=42,//快速滤波时间
	//流速报警滤波时间
	HX711SystemPara_u32_SlowWarnFilterTime=43,//慢速滤波时间
	//
	HX711SystemPara_NUM  			/**< HX711  系统设置-最大长度 */
}enumHX711SystemParaType;


//==========================================================================================================================
//==================code area
//start of on board flash store address
//0x0800 0000 ~ 0x0803 0000

//===================================important:each need store data need 4 byte=============================================
//==================SECTOR1:system control of unit , min , max , err , cascade
//start of on board flash store address
//0X0803E000 ~ 0X0803E7FF
//start of on board sys para flash store address
#define FLASH_SYS_PARA_STORE_ADDRESS_START	(0X0803E000)
#define FLASH_SYS_PASSWORD_ADDRESS_START	FLASH_SYS_PARA_STORE_ADDRESS_START
#define FLASH_SYS_PASSWORD_ADDRESS_LED		(4)
#define FLASH_SYS_PASSWORD_ADDRESS_END		(FLASH_SYS_PASSWORD_ADDRESS_START+FLASH_SYS_PASSWORD_ADDRESS_LED)
//unit:g or ml
#define FLASH_SYS_UNIT_ADDRESS_START	FLASH_SYS_PASSWORD_ADDRESS_END
#define FLASH_SYS_UNIT_LEN				(HX711SystemPara_NUM*4)
#define FLASH_SYS_UNIT_ADDRESS_END		(FLASH_SYS_UNIT_ADDRESS_START+FLASH_SYS_UNIT_LEN)

//end of on board sys para flash store address
#define FLASH_SYS_PARA_STORE_ADDRESS_END		(FLASH_SYS_UNIT_ADDRESS_END)

//store flash data : PASSWORD unit , min , max , cascade ,... , crc
#define FLASH_SYS_PARA_STORE_MAX_LEN			(((FLASH_SYS_PARA_STORE_ADDRESS_END-FLASH_SYS_PARA_STORE_ADDRESS_START)/4)+1)

//==========================================================================================================================
//==================SECTOR2:HX711 point sample , weight value , K ,B , weightRemove , weightDir
//start of on board flash store address
//0X0803F000 ~ 0X0803F7FF
//start of on board flash store address
#define FLASH_STORE_ADDRESS_START				(0X0803F000)

//each chanel have 10 point : HX711_CHANEL_NUM*10*8(sample + weight value) byte
#define FLASH_CHANEL_POINT_ADDRESS_START		(FLASH_STORE_ADDRESS_START)
#define FLASH_CHANEL_POINT_LEN					(HX711_CHANEL_NUM*CHANEL_POINT_NUM*8)
#define FLASH_CHANEL_POINT_ADDRESS_END			(FLASH_CHANEL_POINT_ADDRESS_START+FLASH_CHANEL_POINT_LEN)

//each chanel have 10 point -> 11 KB : HX711_CHANEL_NUM*11*8(K + B) byte
#define FLASH_CHANEL_POINT_KB_ADDRESS_START		(FLASH_CHANEL_POINT_ADDRESS_END)
#define FLASH_CHANEL_POINT_KB_LEN				(HX711_CHANEL_NUM*(CHANEL_POINT_NUM+1)*8)
#define FLASH_CHANEL_POINT_KB_ADDRESS_END		(FLASH_CHANEL_POINT_KB_ADDRESS_START+FLASH_CHANEL_POINT_KB_LEN)

//each chanel remove weight:HX711_CHANEL_NUM*4
#define FLASH_CHANEL_POINT_RMW_ADDRESS_START	(FLASH_CHANEL_POINT_KB_ADDRESS_END)
#define FLASH_CHANEL_POINT_RMW_LEN				(HX711_CHANEL_NUM*4)
#define FLASH_CHANEL_POINT_RMW_ADDRESS_END		((FLASH_CHANEL_POINT_RMW_ADDRESS_START)+FLASH_CHANEL_POINT_RMW_LEN)

//each chanel sensor direction :HX711_CHANEL_NUM*4
#define FLASH_CHANEL_SERNSER_DIR_ADDRESS_START	(FLASH_CHANEL_POINT_RMW_ADDRESS_END)
#define FLASH_CHANEL_SERNSER_DIR_LEN			(HX711_CHANEL_NUM*4)
#define FLASH_CHANEL_SERNSER_DIR_ADDRESS_END	((FLASH_CHANEL_SERNSER_DIR_ADDRESS_START)+FLASH_CHANEL_SERNSER_DIR_LEN)

//end of on board flash store address
#define FLASH_STORE_ADDRESS_END					(FLASH_CHANEL_SERNSER_DIR_ADDRESS_END)

//store flash data : HX711_CHANEL_NUM * (sample value , weight value , k , b , remove value , weightDir ) , crc
#define FLASH_STORE_MAX_LEN						(((FLASH_STORE_ADDRESS_END-FLASH_STORE_ADDRESS_START)/4)+1)
//==========================================================================================================================











//
typedef struct SystemParaType
{
	//store in flash
	INT32	uint;/**< 单位 */
	INT32	minWeight;/**< 最小量程 */
	INT32	maxWeight;/**< 最大量程 */
	float	errRange;/**< 误差范围 */
	INT32	isCascade;/**< 是否级联 0:不级联 1:master 2~n:slave*/
	INT32	isLedIndicate;/**< 是否LED指示 */
	INT32	userColorSet[SYS_COLOR_GROUP_NUM];/**< 配平色1~4 */
	float	zeroRange;/**< 零点范围 */
	
	INT32	ScreenVoiceSwitch;/**< HX711	语音开关 */ 
	INT32 	ScreenCastMode;/**< HX711	级联显示模式 */ 
	INT32 	FlashEraseTimes;/**< HX711	FLASH 擦写次数 */ 

	INT32 	McuVersion;/**< MCU	软件版本号 */ 
	INT32 	DiWenVersion;/**< 迪文	软件版本号 */ 
	INT32 	VoiceNum;/**< 语音大小 */ 
	INT32 	VoiceNumTouch;/**< 语音大小 触控 */ 
	INT32	ScreenLight;/**< 屏幕背光亮度 */
	//sys used flag
	UINT16	userColorUsed[SYS_COLOR_GROUP_NUM];/**< chanel_a<<8 + chanel_b*/
	
	UINT16 motorEn;//电机运转使能
	UINT16 motorDir;//电机运转方向
	UINT16 motorFre;//电机PWM频率
	
	INT32 yusheWeight[SDWeCaiJiDangWei_MAX];//各挡位的预设重量

	INT32 u16_ml_g_bilv;//ml与g比率
	INT32 u16_fmqkqsj;//蜂鸣器开始时间
	INT32 u16_liusu_high_warn;//流速高 报警点
	INT32 u16_liusu_low_warn;//流速低 报警点
	INT32 u16_kaiqi_guanjiafa_gongneng;//开启管夹阀功能
	UINT32 u32_PercentReceipt;//屏幕百分比显示的回滞区间
	
	//==20220720添加9个参数：各挡位最小值，最大值；语音开关使能
	UINT32 u32_LevelMinAndMax[SDWeCaiJiDangWei_MAX][2];//各挡位的最小值 最大值
	UINT32 u32_50PerVoicePrintf;//50%语音播报
	UINT32 u32_80PerVoicePrintf;//80%语音播报
	UINT32 u32_90PerVoicePrintf;//90%语音播报
	UINT32 u32_MotorRockFre;/**< 电机摇摆频率*/
	//==20220226添加3个参数：流速窗口 摇摆值（团采） 摇摆时间（团采）
	UINT32 u32_LiuSu_Window;//流速窗口：窗口采样宽度
	UINT32 u32_YaoBaiVlu_TuanCai;//摇摆值（团采）
	UINT32 u32_YaoBaiTime_TuanCai;//摇摆时间（团采）
	UINT32 u32_FastWarnFilterTime;//快速滤波时间
	UINT32 u32_SlowWarnFilterTime;//慢速滤波时间
} gSystemParaType;
//
#define gSystemParaDefault {\
0, \
0, \
5000, \
(float)(2.0), \
0, \
TRUE, \
{LED_COLOR_GREEN, LED_COLOR_NONE, LED_COLOR_NONE, LED_COLOR_NONE },\
(float)(5.0), \
0,\
0,\
0,\
100,\
30,\
30,\
30,\
50,\
{FALSE, FALSE, FALSE, FALSE },\
\
0,/*电机运转使能*/\
0,/*电机运转方向*/\
1000,/*电机PWM频率*/\
\
{210,310,410},/*各挡位的预设重量*/\
\
1050,/*ml与g比率*/\
0,/*蜂鸣器开始时间*/\
1800,/*流速高 报警点*/\
120,/*流速低 报警点*/\
TRUE,/*是否：开启管夹阀功能*/\
5,/*屏幕百分比显示的回滞区间*/\
\
{{200,450},{300,650},{400,850}},/*各挡位的最小值 最大值*/\
\
FALSE,/*50%语音播报*/\
FALSE,/*80%语音播报*/\
FALSE,/*90%语音播报*/\
\
FALSE,/*90%电机摇摆频率*/\
2,/*流速窗口：窗口采样宽度*/\
80,/*摇摆值（团采）*/\
5,/*摇摆时间（团采）*/\
3000,/*快速滤波时间*/\
3000/*慢速滤波时间*/\
}
//
extern gSystemParaType gSystemPara;

//===================sys para functions
//DMG_TRIGER_SAVE_SECOTOR_1
extern void readSysDataFromFlash(void);
extern void storeSysDataToFlash(void);

//DMG_TRIGER_SAVE_SECOTOR_2
extern void readSysDataFromFlash_3030(void);
extern void storeSysDataToFlash_3030(void);

#endif
