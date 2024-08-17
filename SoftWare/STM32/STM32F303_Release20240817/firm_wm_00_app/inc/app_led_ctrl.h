#ifndef __APP_LED_CTRL_H__
#define __APP_LED_CTRL_H__

#include "typedefine.h"
#include "app_main_task.h"

#define LED_CTRL_DATA_LEN 	(3)

#define SYS_COLOR_GROUP_NUM		(4)//sys max color num
#define SYS_COLOR_USED_FLAG		(0X123)//color used
#define SYS_COLOR_NOT_USED_FLAG		(0)//color not used

//main task status
typedef enum LedSeqType
{
	LED_SEQ_1 = 0,    /**< LED 1控制 */
	LED_SEQ_2 ,       /**< LED 2控制 */
	LED_SEQ_3 ,       /**< LED 3控制 */
	LED_SEQ_4 ,       /**< LED 4控制 */
	LED_SEQ_5 ,       /**< LED 5控制 */
	LED_SEQ_6 ,       /**< LED 6控制 */
	LED_SEQ_NUM
}enumLedSeqType;

typedef enum LedColorType
{
	LED_COLOR_NONE = 0,		/**< LED 无 控制 */
	LED_COLOR_REG ,	/**< LED 红 控制 */
	LED_COLOR_GREEN,	/**< LED 绿 控制 */
	LED_COLOR_BLUE, 	/**< LED 蓝 控制 */
	LED_COLOR_WHITE,	/**< LED 白 控制 */
	LED_COLOR_NUM,
	LED_COLOR_LOCK=0X80,
}enumLedColorType;







 //led program indicate
extern void app_LED_RUN(UINT16 cycle);


#endif
