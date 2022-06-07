/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "hal_gpio.h"
#include "app_led_ctrl.h"
#include "app_hx711_ctrl.h"
#include "app_sdwe_ctrl.h"
#include "app_main_task.h"
#include "app_crc.h"
#include "app_syspara.h"
#include "app_t5l_ctrl.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
UINT8 g_led_ctrl_data[LED_CTRL_DATA_LEN]={0};

/*******************************************************************************
 * Functions
 ******************************************************************************/
//==led delay
void Led_delay( UINT32 TIME )
{
	while(TIME>0)
	{
		TIME--;
	}
}
//==led send pulse
void LedCtrlSendPulse(enumDoLineType offset,UINT8 type)
{
		UINT8 TIME = 100;
		if(1==type)//rising
		{
			hal_gpio_set_do_low( offset );
			Led_delay( TIME );
			hal_gpio_set_do_high( offset );
			Led_delay( TIME );
			hal_gpio_set_do_low( offset );
			Led_delay( TIME );
		}
		else//falling
		{
			hal_gpio_set_do_high( offset );
			Led_delay( TIME );
			hal_gpio_set_do_low( offset );
			Led_delay( TIME );
			hal_gpio_set_do_high( offset );
			Led_delay( TIME );
		}
}
//==led mode init
void led_init(void)
{
	#if 0
	//default LED output
	hal_gpio_set_do_high( LED_DO_SER0 );
	hal_gpio_set_do_high( LED_DO_OE );//disable LED OUT
	hal_gpio_set_do_low( LED_DO_RCLK );//init CLK = high
	hal_gpio_set_do_low( LED_DO_SRCLK );//init LOCK CLK = high
	hal_gpio_set_do_low( LED_DO_SRCLR );//clear ALL LED shift regester

	//1rd:disable LED output
	hal_gpio_set_do_high( LED_DO_OE );

	//2nd:clear all shift reg = 0
	LedCtrlSendPulse(LED_DO_SRCLR,0);

	//2rd:lock data form shift reg to store reg
	LedCtrlSendPulse(LED_DO_RCLK,1);

	//4th:enable LED output
	hal_gpio_set_do_low( LED_DO_OE );
	#endif
}
//==set LED light 
UINT8 LedDataSet(enumLedSeqType seq , enumLedColorType color)
{	
	UINT8 ret = 0 ;//1:success
	UINT8 cloorData = 0 ;
	UINT8 offset = 0 ;//数组偏移
	UINT8 lsb_flag = 0;//低4位 or 高4位
	UINT8 l_data = 0;
	
	//color judge
	switch(color)
	{
		case LED_COLOR_REG:		cloorData = 0x01;/**< LED 红 控制 */
		break;
		case LED_COLOR_WHITE:	cloorData = 0x08;/**< LED 白 控制 */
		break;
		case LED_COLOR_BLUE:	cloorData = 0x04;/**< LED 蓝 控制 */
		break;
		case LED_COLOR_GREEN:	cloorData = 0x02;/**< LED 绿 控制 */	
		break;
		default :			 	cloorData = 0x00;/**< LED    控制 */
		break;
	}
	
	//data change
	if(seq<LED_SEQ_NUM)
	{
		//arry offset
		offset = (LED_SEQ_NUM - 1 - seq)/2;
		l_data = g_led_ctrl_data[offset];
		
		if( offset < LED_CTRL_DATA_LEN )
		{
			//lsb or msb
			lsb_flag = 	(LED_SEQ_NUM - 1 - seq)%2;

			if(0 == lsb_flag)//low 4 bits
			{
				l_data &= 0xf0;
				l_data |= cloorData;
			}
			else//high 4 bits
			{
				l_data &= 0x0f;
				l_data |= ((cloorData<<4)&0xf0);
			}
			
			//
			g_led_ctrl_data[offset] = l_data;
			ret = 1;
		}
	}
	return ret;
}
void balaningColorClear(void)
{
	UINT8 color_i = 0 ;
	for(color_i=0;color_i<LED_SEQ_NUM;color_i++)
	{
		LedDataSet((enumLedSeqType)color_i,LED_COLOR_NONE);
	}
}

//===================v3.0 color compare start
typedef struct WeightUpdateColorType
{
	enumHX711ChanelType curChanel;
	enumHX711ChanelType lockedChanel;//used for if locked not push compare arr
	UINT8	lockedColor;
	float 	lockedWeight;//recode locked weight , used for 
	float	curWeight;
}tWeightUpdateColorType;
static tWeightUpdateColorType tWeightUpdateColorArr[HX711_CHANEL_NUM];


#define COLOR_LOCKED_GROUP_NUM	(HX711_CHANEL_NUM/2)
#define COLOR_LOCKED_ERR_RANGE	(2)



void useWeightUpdataOutColor_3030(UINT8 hx711DataUpgrade)
{
	enumHX711ChanelType chanel = HX711Chanel_1,chanel_a,chanel_b;

	UINT8 sortArry_num = 0;
	enumHX711ChanelType sortArry[HX711_CHANEL_NUM];
	float sortWeight[HX711_CHANEL_NUM]={0.0};
	UINT8 compare_i = 0 ,compare_a = 0 ,compare_b = 0 ;
	float curWeightBuf,lockedWeightBuf;
	
	UINT8 colorGet_i;
	enumLedColorType colorGet_color = LED_COLOR_NONE;
	tWeightUpdateColorType *pWeightUpdateColorArr = &tWeightUpdateColorArr[0];

	
	if((TRUE == hx711DataUpgrade) && (0 != pWeightUpdateColorArr))
	{
		//1.get current weight
		for(chanel = HX711Chanel_1;chanel<HX711_CHANEL_NUM;chanel++)
		{
			pWeightUpdateColorArr[chanel].curChanel = chanel;
			pWeightUpdateColorArr[chanel].curWeight = hx711_getWeight(chanel);
		}

		//2.already locked judge if not remove lock
		for(chanel = HX711Chanel_1;chanel<HX711_CHANEL_NUM;chanel++)
		{
			//2.1
			if(HX711_CHANEL_LOCKED == pWeightUpdateColorArr[chanel].lockedChanel)
			{
				curWeightBuf = pWeightUpdateColorArr[chanel].curWeight;
				lockedWeightBuf = pWeightUpdateColorArr[chanel].lockedWeight;
				//2.1.1.at zero range , such as take out
				if(( curWeightBuf >= -gSystemPara.zeroRange) && ( curWeightBuf <= gSystemPara.zeroRange))
				{
					pWeightUpdateColorArr[chanel].lockedChanel = HX711_CHANEL_UNLOCKED;//unlock
				}//2.1.2.cur weight and pre weight out of errRange , such as weight changed
				else if(((lockedWeightBuf - curWeightBuf) > gSystemPara.errRange ) || ((lockedWeightBuf - curWeightBuf) < -gSystemPara.errRange ))
				{
					pWeightUpdateColorArr[chanel].lockedChanel = HX711_CHANEL_UNLOCKED;//unlock
				}
			}
			//2.2if judge locked is not locked ,prepare Sort
			if(HX711_CHANEL_LOCKED != pWeightUpdateColorArr[chanel].lockedChanel)
			{
				//clear color
				LedDataSet((enumLedSeqType)chanel, LED_COLOR_NONE);
				sdweSetWeightBackColor(chanel, LED_COLOR_NONE);
				//push to sortArry
				sortArry[sortArry_num] = pWeightUpdateColorArr[chanel].curChanel;
				sortWeight[sortArry_num] = pWeightUpdateColorArr[chanel].curWeight;
				sortArry_num++;	
			}
		}

		//3.Sort
		BubbleSort(sortWeight,(INT16 *)sortArry,sortArry_num);
		
		//4.compare
		for(compare_i=0;compare_i<(sortArry_num-1);compare_i++)
		{
			//4.1.get used color
			for(colorGet_i=0;colorGet_i<SYS_COLOR_GROUP_NUM;colorGet_i++)
			{
				//4.1.1.already used color , check if not release
				if(FALSE != gSystemPara.userColorUsed[colorGet_i])
				{
					//4.1.1.1.get locked chanel
					chanel_a = (enumHX711ChanelType)((gSystemPara.userColorUsed[colorGet_i]>>8)&0xff);
					chanel_b = (enumHX711ChanelType)((gSystemPara.userColorUsed[colorGet_i]>>0)&0xff);
					//4.1.1.2.if 2 chanel not locked
					if((HX711_CHANEL_LOCKED != pWeightUpdateColorArr[chanel_a].lockedChanel)
						&& (HX711_CHANEL_LOCKED != pWeightUpdateColorArr[chanel_b].lockedChanel))
					{
						gSystemPara.userColorUsed[colorGet_i] = FALSE;//clear color locked
					}
				}
				//4.1.2.if color is not none and not used
				if((FALSE == gSystemPara.userColorUsed[colorGet_i]) 
					&& (LED_COLOR_NONE != gSystemPara.userColorSet[colorGet_i]))
				{
					//colorGet_color = gSystemPara.userColorSet[colorGet_i];
					break;
				}
			}

			//4.2.not avaliable color
			if(colorGet_i >= SYS_COLOR_GROUP_NUM)
			{
				break;
			}
			//4.3.get avaliable color
			colorGet_color = (enumLedColorType)gSystemPara.userColorSet[colorGet_i];

			//4.4 get compare chanel
			compare_a = compare_i;
			compare_b = compare_i+1;
			//4.5.1.compare_i out of zeroRange
			if((sortWeight[compare_a] < -gSystemPara.zeroRange) || (sortWeight[compare_a] > gSystemPara.zeroRange))
			{
				//4.5.2.compare_i+1 out of zeroRange
				if((sortWeight[compare_b] < -gSystemPara.zeroRange) || (sortWeight[compare_b] > gSystemPara.zeroRange))
				{
					//4.5.3.compare_i+1 - compare_i at of errRange
					if(((sortWeight[compare_a] - sortWeight[compare_b]) > -gSystemPara.errRange) 
						&& ((sortWeight[compare_a] - sortWeight[compare_b]) < gSystemPara.errRange) )
					{
						//4.2.3.1.compare success,ger chanel
						chanel_a = sortArry[compare_a];
						chanel_b = sortArry[compare_b];
						//4.2.3.2.get avaliable color
						colorGet_color = (enumLedColorType)gSystemPara.userColorSet[colorGet_i];
						gSystemPara.userColorUsed[colorGet_i] = ((chanel_a<<8)&0xff00)+chanel_b;
						//4.2.3.3.set same color
						LedDataSet((enumLedSeqType)chanel_a, colorGet_color);//led
						sdweSetWeightBackColor(chanel_a, colorGet_color);//screen
						LedDataSet((enumLedSeqType)chanel_b, colorGet_color);//led
						sdweSetWeightBackColor(chanel_b, colorGet_color);//screen	
						//4.2.3.4.set lockedChanel flag and recode locked weight
						pWeightUpdateColorArr[chanel_a].lockedChanel = HX711_CHANEL_LOCKED;
						pWeightUpdateColorArr[chanel_a].lockedWeight = sortWeight[chanel_a];
						pWeightUpdateColorArr[chanel_b].lockedChanel = HX711_CHANEL_LOCKED;
						pWeightUpdateColorArr[chanel_b].lockedWeight = sortWeight[chanel_b];

						//
						compare_i++;
					}
				}
			}
		}
	}
}
//===================v3.0 color compare end







#define CHANEL_COMPARED_FLAG_MASK	0XF000
#define CHANEL_COMPARED_OTHER_MASK	0X0F00
#define CHANEL_COMPARED_COLOR_MASK	0X00F0

#define CHANEL_COMPARED_FLAG_BIT	(12)
#define CHANEL_COMPARED_OTHER_BIT	(8)
#define CHANEL_COMPARED_COLOR_BIT	(4)

//==update color
void useWeightUpdateLedAndSdweColor(UINT8 hx711DataUpgrade)
{
	enumHX711ChanelType chanel = HX711Chanel_1;
	enumHX711ChanelType arry[HX711_CHANEL_NUM];
	enumLedSeqType ledSeq = LED_SEQ_1; 	
	enumLedColorType color = LED_COLOR_REG ;
	float weight[HX711_CHANEL_NUM]={0.0};
	//static float preWeight[HX711_CHANEL_NUM]={0.0};
	//if weight data changed
	if(1 == hx711DataUpgrade)
	{
		//get each chanel weight
		for(chanel = HX711Chanel_1;chanel<HX711_CHANEL_NUM;chanel++)
		{
			arry[chanel] = chanel;
			weight[chanel] = hx711_getWeight(chanel);
		}
		//sequence
		BubbleSort(weight,(INT16 *)arry,HX711_CHANEL_NUM);
		//
		for(ledSeq = LED_SEQ_1;ledSeq<(LED_SEQ_NUM-1);ledSeq++)
		{
			if(((weight[ledSeq+1] - weight[ledSeq]) < gSystemPara.errRange) &&
				((weight[ledSeq+1] - weight[ledSeq]) > -gSystemPara.errRange) &&
				((weight[ledSeq] < -gSystemPara.errRange) || (weight[ledSeq] > gSystemPara.errRange)) &&
				((weight[ledSeq+1] < -gSystemPara.errRange) || (weight[ledSeq+1] > gSystemPara.errRange)) )
			{
				LedDataSet((enumLedSeqType)arry[ledSeq], color);//light same color
				sdweSetWeightBackColor(arry[ledSeq], color);
				LedDataSet((enumLedSeqType)(arry[ledSeq+1]), color);//light same color
				sdweSetWeightBackColor((enumLedSeqType)(arry[ledSeq+1]), color);//light same color
				ledSeq++;
				color++;
			}
			else
			{
				LedDataSet((enumLedSeqType)arry[ledSeq], LED_COLOR_NONE);//not light
				sdweSetWeightBackColor(arry[ledSeq], LED_COLOR_NONE);//not light
				if((LED_SEQ_NUM-2) == ledSeq)
				{
					LedDataSet((enumLedSeqType)(arry[ledSeq+1]), LED_COLOR_NONE);//not light
					sdweSetWeightBackColor((enumLedSeqType)(arry[ledSeq+1]), LED_COLOR_NONE);//not light
				}
			}
		}
	}
}





#if(TRUE == COLOR_ALT_20210427_DEFINE)
//num of need balaning
#define BALANCING_NUM	(6)
//balaning judge status
typedef enum
{
	eBalaningJudgeStatus_FAILD = 0 ,
	eBalaningJudgeStatus_SUCCESS = 1 ,
	eBalaningJudgeStatus_Curent_MOVED = 2 ,
	eBalaningJudgeStatus_Other_MOVED = 3 ,
	eBalaningJudgeStatus_MAX = 4
}eBalaningJudgeStatus_t;

//each need balaning info
typedef struct
{
	float 	preWight;//pre weight
	float 	curWight;//current weight
	enumLedColorType disColor;//current dis color
	UINT16	otherChanel;//balaning chanel
	eBalaningJudgeStatus_t balaningStatus;
}tBalaningInfo_t;
//balaning management
typedef struct
{
	tBalaningInfo_t tBalaningInfo[BALANCING_NUM];
	INT32	userColorSet[SYS_COLOR_GROUP_NUM];/**< 配平色1~4 */
	UINT16	userColorUsed[SYS_COLOR_GROUP_NUM];/**< */
	float 	fWightA;//weight A
	float 	fWightB;//weight B
}tBalaningManager_t;
//
static tBalaningManager_t tBalaningManager;
//get balaning weight
float balaningWeightGet(UINT8 chanel)
{
	float weight;
	tBalaningManager_t *pBalaning=&tBalaningManager;
	if(chanel < BALANCING_NUM)
	{
		weight = pBalaning->tBalaningInfo[chanel].curWight;
	}
	return weight;
}
//set balaning weight
void balaningWeightSet(UINT8 chanel,float weight)
{
	tBalaningManager_t *pBalaning=&tBalaningManager;
	if(chanel < BALANCING_NUM)
	{
		pBalaning->tBalaningInfo[chanel].preWight = pBalaning->tBalaningInfo[chanel].curWight;
		pBalaning->tBalaningInfo[chanel].curWight = weight;
	}
}
//get balaning color
enumLedColorType balaningColorGet(UINT8 chanel)
{
	enumLedColorType disColor;
	tBalaningManager_t *pBalaning=&tBalaningManager;
	if(chanel < BALANCING_NUM)
	{
		disColor = pBalaning->tBalaningInfo[chanel].disColor;
	}
	return disColor;
}
//set balaning weight
void balaningColorSet(UINT8 chanel,enumLedColorType disColor)
{
	tBalaningManager_t *pBalaning=&tBalaningManager;
	if(chanel < BALANCING_NUM)
	{
		pBalaning->tBalaningInfo[chanel].disColor = disColor;
	}
}
//get balaning other chanel
UINT16 balaningOtherChanelGet(UINT8 chanel)
{
	UINT16 otherChanel=BALANCING_NUM;
	tBalaningManager_t *pBalaning=&tBalaningManager;
	if(chanel < BALANCING_NUM)
	{
		otherChanel = pBalaning->tBalaningInfo[chanel].otherChanel;
	}
	return otherChanel;
}
//set balaning other chanel
void balaningOtherChanelSet(UINT8 chanel,UINT16 otherChanel)
{
	tBalaningManager_t *pBalaning=&tBalaningManager;
	if(chanel < BALANCING_NUM)
	{
		pBalaning->tBalaningInfo[chanel].otherChanel = otherChanel;
	}
}
//get balaning status
eBalaningJudgeStatus_t balaningChanelStatusGet(UINT8 chanel)
{
	eBalaningJudgeStatus_t balaningStatus;
	tBalaningManager_t *pBalaning=&tBalaningManager;
	if(chanel < BALANCING_NUM)
	{
		balaningStatus = pBalaning->tBalaningInfo[chanel].balaningStatus;
	}
	return balaningStatus;
}
//set balaning status
void balaningChanelStatusSet(UINT8 chanel,eBalaningJudgeStatus_t balaningStatus)
{
	tBalaningManager_t *pBalaning=&tBalaningManager;
	if(chanel < BALANCING_NUM)
	{
		pBalaning->tBalaningInfo[chanel].balaningStatus = balaningStatus;
	}
}

//balaning init
void balaningInit()
{
	UINT8 chanel;
	enumLedColorType color = LED_COLOR_NONE;
	UINT8 user_i=0;
	gSystemParaType *pSysPara = &gSystemPara;
	for(chanel=0;chanel<BALANCING_NUM;chanel++)
	{
		balaningWeightSet(chanel,0);
		balaningColorSet(chanel,LED_COLOR_NONE);
		balaningOtherChanelSet(chanel,BALANCING_NUM);
		balaningChanelStatusSet(chanel,eBalaningJudgeStatus_FAILD);
	}

	//
	tBalaningManager.fWightA = 0 ;
	tBalaningManager.fWightB = 0 ;
	for(user_i=0;user_i<SYS_COLOR_GROUP_NUM;user_i++)
	{
		tBalaningManager.userColorUsed = 0;//not used
		tBalaningManager.userColorSet[user_i] = pSysPara->userColorSet[user_i];
	}
}
//balaning
eBalaningJudgeStatus_t balaningJudge(tBalaningInfo_t *pA,tBalaningInfo_t *pB)
{
	eBalaningJudgeStatus_t balaningStatus = eBalaningJudgeStatus_FAILD;
	float curWeight,otherWeight;

	if((NULL != pA) && (NULL != pB)) 
	{
		curWeight = pA->curWight;
		otherWeight = pB->curWight;
		//
		if((curWeight < -gSystemPara.zeroRange) || (curWeight > gSystemPara.zeroRange))
		{
			//4.5.2.compare_i+1 out of zeroRange
			if((otherWeight < -gSystemPara.zeroRange) || (otherWeight > gSystemPara.zeroRange))
			{
				//4.5.3.compare_i+1 - compare_i at of errRange
				if(((curWeight - otherWeight) > -gSystemPara.errRange) 
					&& ((curWeight - otherWeight) < gSystemPara.errRange) )
				{
					balaningStatus = eBalaningJudgeStatus_SUCCESS;
				}
				else
				{
					balaningStatus = eBalaningJudgeStatus_FAILD;
				}
			}
			else
			{
				balaningStatus = eBalaningJudgeStatus_Other_MOVED;
			}
		}
		else
		{
			balaningStatus = eBalaningJudgeStatus_Curent_MOVED;
		}
	}

	return balaningStatus;
}


//balanind mainfunction
void balaningMainfuction()
{
	tBalaningManager_t *pBalaningManager = &tBalaningManager;
	UINT8 curChanel,otherChanel;
	enumLedColorType disColor,otherDisColor;
	float curWeight,otherWeight;
	eBalaningJudgeStatus_t balaningStatus = eBalaningJudgeStatus_FAILD;
	for(curChanel=0;curChanel<BALANCING_NUM;curChanel++)
	{
		//chanel was at dis color status
		disColor = balaningColorGet(curChanel);
		curWeight = balaningWeightGet(curChanel);
		if(LED_COLOR_NONE != disColor)
		{
			//judge weight if not exchanegd
			otherChanel = balaningOtherChanelGet(curChanel);
			if(otherChanel < BALANCING_NUM)
			{
				//normal:other chanel was exist
				otherDisColor = balaningColorGet(otherChanel);
				otherWeight = balaningWeightGet(otherChanel);
				if((LED_COLOR_NONE != otherDisColor) && (disColor == otherDisColor))
				{
					//normal:color was the same color
					//balaning juge again
					balaningStatus = balaningJudge(&pBalaningManager->tBalaningInfo[curChanel],&pBalaningManager->tBalaningInfo[otherChanel])		
					


				}
				else
				{
					//clear chanel color
					balaningColorSet(curChanel,LED_COLOR_NONE);
					balaningColorSet(otherChanel,LED_COLOR_NONE);
					//clear chanel other chanel
					balaningOtherChanelSet(curChanel,BALANCING_NUM);
					balaningOtherChanelSet(otherChanel,BALANCING_NUM);
				}
			}
			else//err:if curent chanel have dis color but other chanel not exist
			{
				//clear chanel color
				balaningColorSet(curChanel,LED_COLOR_NONE);
				//clear chanel other chanel
				balaningOtherChanelSet(curChanel,BALANCING_NUM);
			}
			
		}
	}
}

enumLedColorType balaningFinoutUseableColor()
{
	enumLedColorType color = LED_COLOR_NONE;
	gSystemParaType *pSystemPara = &gSystemPara;
	UINT8	i = 0 ;
	for(i=0;i<SYS_COLOR_GROUP_NUM;i++)
	{
		if(FALSE == pSystemPara->userColorUsed[i])
		{
			color = pSystemPara->userColorSet[i];
			pSystemPara->userColorUsed[i] = TRUE;
			break;
		}
	}
	return color;
}


//balanind mainfunction
void balaningMainfuction222()
{
	tBalaningManager_t *pBalaningManager = &tBalaningManager;
	UINT8 curChanel,otherChanel;
	enumLedColorType disColor,otherDisColor;
	float curWeight,otherWeight;
	eBalaningJudgeStatus_t balaningStatus = eBalaningJudgeStatus_FAILD;
	for(curChanel=0;curChanel<BALANCING_NUM;curChanel++)
	{	
		for(otherChanel=(curChanel+1);otherChanel<BALANCING_NUM;otherChanel++)
		{
			balaningStatus = balaningJudge(&pBalaningManager->tBalaningInfo[curChanel],&pBalaningManager->tBalaningInfo[otherChanel]);
			if(eBalaningJudgeStatus_SUCCESS == balaningStatus)
			{
				//clear chanel color
				
				balaningColorSet(curChanel,LED_COLOR_NONE);
				//update balaning chanel
				balaningOtherChanelSet(curChanel,otherChanel);
				balaningOtherChanelSet(otherChanel,curChanel);
			}
			else
			{
				//clear color
			}
		}

	}


}



#endif



//led program indicate
void app_LED_RUN(UINT16 cycle)
{
	static UINT32 ledRunTick=0;
	if(ledRunTick++%cycle == 0)
	{
		hal_gpio_set_do_toggle( (enumDoLineType)(LED_RUN) );
	}
}




