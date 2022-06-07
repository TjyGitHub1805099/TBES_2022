#ifndef __APP_KEY_CTRL_H__
#define __APP_KEY_CTRL_H__

#include "typedefine.h"
#include "hal_gpio.h"

#define SYS_KEY_NUM			(STM32_KEY_4-STM32_KEY_1+1)	
#define SYS_KEY_FILTER_NUM	(50)//50ms
#define SYS_KEY_VALUED		(0)
#define SYS_KEY_INVALUED	(1)

#define SYS_POS_NUM			(STM32_POS_3-STM32_POS_1+1)	
#define SYS_POS_VALUED		(0)
#define SYS_POS_INVALUED	(1)
#define SYS_POS_FILTER_NUM	(20)//20ms

typedef struct
{
	enumDiLineType type;
	UINT8 	initFlag;
	UINT8	count;
	UINT8	preSample;
	UINT8	curSample;
	UINT8	filterOutput;
	//
	UINT8 	filterOutputPre;
	UINT8 	eventGenerate;
} SysInputFilterType;

void key_init(void);
void key_MainFunction(void);
UINT8 key_FilterGet(enumDiLineType type);
UINT8 key_EventGet(enumDiLineType type);
void key_EventClear(enumDiLineType type);

void pos_init(void);
void pos_MainFunction(void);
UINT8 pos_FilterGet(enumDiLineType type);
UINT8 pos_EventGet(enumDiLineType type);
void pos_EventClear(enumDiLineType type);




extern UINT8 getKeyValuedEvent_OfTBES(enumDiLineType tbes_Key);
#endif
