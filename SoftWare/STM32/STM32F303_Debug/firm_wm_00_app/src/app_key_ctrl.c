/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "app_main_task.h"
#include "app_key_ctrl.h"
#include "app_hx711_ctrl.h"
#include "app_sdwe_ctrl.h"
#include "app_t5l_ctrl.h"
#include "app_modbus_rtu_ctrl.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
SysInputFilterType SysKey[SYS_KEY_NUM];
SysInputFilterType SysPos[SYS_POS_NUM];

/*******************************************************************************
 * Functions
 ******************************************************************************/
//==key init
void key_init(void)
{
	UINT8 i = 0 ;
	SysInputFilterType *pSysKeyType = &SysKey[0];
	for(i=0;i<SYS_KEY_NUM;i++)
	{
		pSysKeyType[i].type = (enumDiLineType)(STM32_KEY_1+i);
		pSysKeyType[i].count = 0;
		pSysKeyType[i].preSample = SYS_KEY_INVALUED ;
		pSysKeyType[i].curSample = SYS_KEY_INVALUED ;
		pSysKeyType[i].filterOutput = SYS_KEY_INVALUED;
		pSysKeyType[i].initFlag = TRUE;
	}
}
//==key filter
void key_filter()
{
	UINT8 i = 0;
	SysInputFilterType *pSysKeyType = &SysKey[0];
	//
	for(i=0;i<SYS_KEY_NUM;i++)
	{
		pSysKeyType[i].curSample = hal_di_get(pSysKeyType[i].type);
		if( pSysKeyType[i].preSample == pSysKeyType[i].curSample)
		{
			pSysKeyType[i].count++;
			//output
			if( pSysKeyType[i].count >= SYS_KEY_FILTER_NUM )
			{
				pSysKeyType[i].count = SYS_KEY_FILTER_NUM;
				pSysKeyType[i].filterOutput = pSysKeyType[i].curSample;
			}
		}
		else
		{
			pSysKeyType[i].count = 0 ;
		}
		pSysKeyType[i].preSample = pSysKeyType[i].curSample ;
		
		//
		if(pSysKeyType[i].filterOutputPre != pSysKeyType[i].filterOutput)
		{
			pSysKeyType[i].filterOutputPre = pSysKeyType[i].filterOutput;
			pSysKeyType[i].eventGenerate = TRUE ;
		}
	}
}
//==key filter out get
UINT8 key_FilterGet(enumDiLineType type)
{
	UINT8 value = SYS_KEY_INVALUED;
	if( (type >= STM32_KEY_1) && (type < (STM32_KEY_1+SYS_KEY_NUM) ) )
	{
		value =  SysKey[type-STM32_KEY_1].filterOutput;
	}
	return value;
}
//==key event out get
UINT8 key_EventGet(enumDiLineType type)
{
	UINT8 value = 0;//not event
	if( (type >= STM32_KEY_1) && (type < (STM32_KEY_1+SYS_KEY_NUM) ) )
	{
		value =  SysKey[type-STM32_KEY_1].eventGenerate;
	}
	return value;
}
//==key event out clear
void key_EventClear(enumDiLineType type)
{
	if( (type >= STM32_KEY_1) && (type < (STM32_KEY_1+SYS_KEY_NUM) ) )
	{
		SysKey[type-STM32_KEY_1].eventGenerate = FALSE;
	}
}

//==key main function
void key_MainFunction(void)
{
	//SysInputFilterType *sysInput = &SysKey[0];
	key_filter();
}


//==pos init
void pos_init(void)
{
	UINT8 i = 0 ;
	SysInputFilterType *pSysPosType = &SysPos[0];
	for(i=0;i<SYS_POS_NUM;i++)
	{
		pSysPosType[i].type = (enumDiLineType)(STM32_POS_1+i);
		pSysPosType[i].count = 0;
		pSysPosType[i].preSample = SYS_POS_INVALUED ;
		pSysPosType[i].curSample = SYS_POS_INVALUED ;
		pSysPosType[i].filterOutput = SYS_POS_INVALUED;
		pSysPosType[i].initFlag = TRUE;
	}
}
//==pos filter
void pos_filter()
{
	UINT8 i = 0;
	SysInputFilterType *pSysPosType = &SysPos[0];
	//
	for(i=0;i<SYS_POS_NUM;i++)
	{
		pSysPosType[i].curSample = hal_di_get(pSysPosType[i].type);
		if( pSysPosType[i].preSample == pSysPosType[i].curSample)
		{
			pSysPosType[i].count++;
			//output
			if( pSysPosType[i].count >= SYS_POS_FILTER_NUM )
			{
				pSysPosType[i].count = SYS_POS_FILTER_NUM;
				pSysPosType[i].filterOutput = pSysPosType[i].curSample;
			}
		}
		else
		{
			pSysPosType[i].count = 0 ;
		}
		pSysPosType[i].preSample = pSysPosType[i].curSample ;

		//
		if(pSysPosType[i].filterOutputPre != pSysPosType[i].filterOutput)
		{
			pSysPosType[i].filterOutputPre = pSysPosType[i].filterOutput;
			pSysPosType[i].eventGenerate = TRUE ;
		}
	}
}
//==pos filter out get
UINT8 pos_FilterGet(enumDiLineType type)
{
	UINT8 value = SYS_POS_INVALUED;
	if( (type >= (enumDiLineType)STM32_POS_1) && (type < (STM32_POS_1+SYS_POS_NUM) ))
	{
		value =  SysPos[type-STM32_POS_1].filterOutput;
	}
	return value;
}
//==pos main function
void pos_MainFunction(void)
{
	pos_filter();
}
//==key event out get
UINT8 pos_EventGet(enumDiLineType type)
{
	UINT8 value = 0;//not event
	if( (type >= STM32_POS_1) && (type < (STM32_POS_1+SYS_POS_NUM) ) )
	{
		value =  SysPos[type-STM32_POS_1].eventGenerate;
	}
	return value;
}
//==key event out clear
void pos_EventClear(enumDiLineType type)
{
	if( (type >= STM32_POS_1) && (type < (STM32_POS_1+SYS_POS_NUM) ) )
	{
		SysPos[type-STM32_POS_1].eventGenerate = FALSE;
	}
}




UINT8 getKeyValuedEvent_OfTBES(enumDiLineType tbes_Key)
{
	UINT8 ret = SYS_KEY_INVALUED;
	if(TRUE == key_EventGet(tbes_Key))
	{
		//key_EventClear(tbes_Key);
		if(SYS_KEY_VALUED == key_FilterGet(tbes_Key))
		{	
			ret = SYS_KEY_VALUED;
		}
	}
	return ret;
}


