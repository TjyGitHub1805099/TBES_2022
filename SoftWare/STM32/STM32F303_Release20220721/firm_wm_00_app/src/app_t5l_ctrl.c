/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "drv_flash.h"
#include "app_main_task.h"
#include "app_sdwe_ctrl.h"
#include "app_crc.h"
#include "app_hx711_ctrl.h"
#include "app_crc.h"
#include "hal_delay.h"
#include "app_modbus_rtu_ctrl.h"
#include "app_syspara.h"
#include "app_password.h"
#include "app_t5l_ctrl.h"
#include "app_motor_ctrl.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
T5LType g_T5L = SDWeDataDefault;
//sdwe 8 weight data + 8 color data	
INT16 g_t5l_dis_data[T5L_MAX_CHANEL_LEN]={0};
INT16 g_t5l_dis_data_buff[T5L_MAX_CHANEL_LEN]={0};

//1.chanel num :0~x HX711_CHANEL_NUM
//2.trigerStarus , back color , point avg Sample , point set weight
//3.point num
static INT16 g_t5l_triger_data[HX711_CHANEL_NUM+1][DMG_TRIGER_SAMPLE_MAX_NUM][CHANEL_POINT_NUM];

//voice printf buff
tT5LVoinceType g_T5L_VoiceBuff[T5L_VOICE_MAX_PRINTF_NUM][3];
UINT8 u8T5LVoiceBuffPush_i = 0 ,u8T5LVoiceBuffPop_i = 0 , u8T5LVoiceBuffStoreNum = 0;

//data send to DIWEN
INT16 g_i16DataBuff[T5L_MAX_CHANEL_LEN]={0};
INT16 g_i16DataBuffPre[T5L_MAX_CHANEL_LEN]={0};
//color send to DIWEN
INT16 g_i16ColorBuff[T5L_MAX_CHANEL_LEN]={0};
INT16 g_i16ColorBuffPre[T5L_MAX_CHANEL_LEN]={0};
INT16 g_i16ColorOtherChanel[T5L_MAX_CHANEL_LEN]={0};//T5L_WEIGHT_CHANEL_INVALID:invalid
//
float g_fDataBuffCaculate[T5L_MAX_CHANEL_LEN]={0};
INT16 g_i16OtherChanelCaculate[T5L_MAX_CHANEL_LEN]={0};//other chanel need +1 , chanel = 1~x

float g_i16HelpDataSort[T5L_MAX_CHANEL_LEN]={0};
INT16 g_i16HelpDataChnSort[T5L_MAX_CHANEL_LEN]={0};

UINT8 g_u8Read00A1_Data = 0XFF;


extern UINT32 g_sys_ms_tick;

/*******************************************************************************
 * Functions
 ******************************************************************************/
//==sdwe->mcu rx callback ,not used
void app_uart_extern_msg_packet_process( UartDeviceType *pUartDevice )
{
	//not used
}

//==sdwe initial
void screenT5L_Init(void)
{
	UINT8 i = 0 ;
	//
	g_T5L.pUartDevice = &g_UartDevice[UART_ZXP];
	//
	g_T5L.pUartDevice->pRxLength = &g_T5L.RxLength;
	g_T5L.pUartDevice->pRxFinishFlag = &g_T5L.RxFinishFlag;
	g_T5L.pUartDevice->pTxBuffer = &g_T5L.rxData[0];
	g_T5L.pUartDevice->pRxBuffer = &g_T5L.rxData[0];
	//
	//
	g_T5L.RxLength = 0;					/**< ??????????????? */
	g_T5L.RxFinishFlag = FALSE;			/**< ?????????????????? */
	//
	g_T5L.SetAdd = 0XFFFF;/**< ?????? */
	g_T5L.DataLen = 0;/**< ???????????? */
	g_T5L.SetData = 0;/**< ?????? */

	g_T5L.ColorClen=FALSE;/**< ????????????SDWE???????????? */
	g_T5L.CalibrateChanel=88;/**< ?????? */
	g_T5L.CalibratePoint=0;/**< ????????? */


	g_T5L.ResetTrigerValid = FALSE;
	//
	for(i=0;i<CHANEL_POINT_NUM;i++)
	{
		g_T5L.CalibratePointArry[i] = defaultChanelSamplePoint[i];/**< ??????????????? */
	}
	for(i=0;i<T5L_MAX_CHANEL_LEN;i++)
	{
		g_i16ColorOtherChanel[i]=T5L_CHANEL_WEIGHT_NOT_EQUAL;
	}
	//
	g_T5L.pUartDevice->init(g_T5L.pUartDevice);
}


//========================================================================================write varible to SDWe : check:20220526
//==write varible data to SDWE thought UART
UINT8 screenSDWeWriteVarible(UINT16 varAdd, UINT16 *pData ,UINT16 varlen ,UINT8 crcEn)
{
	//???????????????0x0003????????????0x00 ???0x01
	//?????????0xA5 0x5A 0x05 0x82 0x00 0x03 0x00 0x01
	UINT16 i = 0 ,l_data = 0 , total_len = 0 , crc = 0 , ret = FALSE;
	if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
		((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
	{
		if(varAdd < 0xFFFF)
		{
			if(((varAdd+varlen)>0)&&((varAdd+varlen)<0xFFFF))
			{
				//head
				g_T5L.txData[cmdPosHead1]=T5L_RX_FUN_HEAD1;
				g_T5L.txData[cmdPosHead2]=T5L_RX_FUN_HEAD2;
				//data len
				if(TRUE == crcEn)
				{
					g_T5L.txData[cmdPosDataLen]=0X03+2*varlen+2;//CRC
				}
				else
				{
					g_T5L.txData[cmdPosDataLen]=0X03+2*varlen;
				}
				//order:write
				g_T5L.txData[cmdPosCommand]=cmdWriteSWDEVariable;
				//varAdd
				g_T5L.txData[cmdPosVarWriteAddress1]=0xff&(varAdd>>8);
				g_T5L.txData[cmdPosVarWriteAddress2]=0xff&(varAdd>>0);
				//data
				for(i=0;i<varlen;i++)
				{
					l_data = *pData++;
					g_T5L.txData[cmdPosVarWriteData+2*i+0] = 0xff&(l_data>>8);
					g_T5L.txData[cmdPosVarWriteData+2*i+1] = 0xff&(l_data>>0);
				}
				//crc
				if(TRUE == crcEn)
				{
					crc = cal_crc16(&g_T5L.txData[cmdPosCommand],(3+2*varlen));
					g_T5L.txData[cmdPosVarWriteData+2*varlen+0] = 0xff&(crc>>0);
					g_T5L.txData[cmdPosVarWriteData+2*varlen+1] = 0xff&(crc>>8);
					//total len
					total_len = 6+2*varlen+2;//crc
				}
				else
				{
					//total len
					total_len = 6+2*varlen;
				}
				//send
				g_T5L.pUartDevice->tx_bytes(g_T5L.pUartDevice,&g_T5L.txData[0],total_len);
				g_T5L.LastSendTick = g_T5L.CurTick;
				//
				//hal_delay_ms(1);
			}
		}
		ret = TRUE;
	}
	return ret;
}

//========================================================================================read varible from SDWe : check:20220526
//==read varible data from SDWE thought UART
UINT8 screenSDWeReadVarible(UINT16 varAdd,UINT16 varlen ,UINT8 crcEn)
{
	//???????????????????????????0x0003???0x0004??????
	//?????????0xA5 0x5A 0x04 0x83 0x00 0x03 0x02
	//?????????0xA5 0x5A 0x08 0x83 0x00 0x03 0x02 0x00 0x01 0xff 0xff
	UINT16 total_len = 0 , crc = 0 , ret = FALSE;
	if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
		((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
	{
		if(varAdd < 0xFFFF)
		{
			if(((varAdd+varlen)>0)&&((varAdd+varlen)<0xFFFF))
			{
				//head
				g_T5L.txData[cmdPosHead1]=T5L_RX_FUN_HEAD1;
				g_T5L.txData[cmdPosHead2]=T5L_RX_FUN_HEAD2;
				//data len
				if(TRUE == crcEn)
				{
					g_T5L.txData[cmdPosDataLen]=0X04+2;//CRC
				}
				else
				{
					g_T5L.txData[cmdPosDataLen]=0X04;
				}
				//order:write
				g_T5L.txData[cmdPosCommand]=cmdReadSWDEVariable;
				//varAdd
				g_T5L.txData[cmdPosVarReadAddress1]=0xff&(varAdd>>8);
				g_T5L.txData[cmdPosVarReadAddress2]=0xff&(varAdd>>0);
				//len
				g_T5L.txData[cmdPosVarReadLen]=varlen&0xff;
				//crc
				if(TRUE == crcEn)
				{
					crc = cal_crc16(&g_T5L.txData[cmdPosCommand],4);
					g_T5L.txData[cmdPosVarReadLen+1+0] = 0xff&(crc>>0);
					g_T5L.txData[cmdPosVarReadLen+1+1] = 0xff&(crc>>8);
					//total len
					total_len = 9;
				}
				else
				{
					//total len
					total_len = 7;
				}
				//send
				g_T5L.pUartDevice->tx_bytes(g_T5L.pUartDevice,&g_T5L.txData[0],total_len);
				g_T5L.LastSendTick = g_T5L.CurTick;
				//
				//hal_delay_ms(1);
			}
		}
		ret = TRUE;
	}
	return ret;
}

//========================================================================================write reg to SDWe : check:20220526
//==write reg data to screen , have delay contrl
UINT8 screenSDWeWriteReg(UINT8 regAdd, UINT8 *pData ,UINT8 reglen ,UINT8 crcEn)
{
	//?????????0x03???0x04??????????????????0x00 ???0x01
	//0xA5 0x5A 0x04 0x80 0x03 0x00 0x01
	//??????CRC   0x06
	UINT8 i = 0 ,l_data = 0 , total_len = 0 , ret = FALSE;
	UINT16 crc = 0;
	if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
		((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
	{
		if(regAdd <= 0xFF)
		{
			if(((regAdd+reglen)>0)&&((regAdd+reglen)<=0xFF))
			{
				//head
				g_T5L.txData[cmdPosHead1]=T5L_RX_FUN_HEAD1;
				g_T5L.txData[cmdPosHead2]=T5L_RX_FUN_HEAD2;
				//data len
				if(TRUE == crcEn)
				{
					g_T5L.txData[cmdPosDataLen]=2+reglen+2;//CRC
				}
				else
				{
					g_T5L.txData[cmdPosDataLen]=2+reglen;
				}
				//order:write
				g_T5L.txData[cmdPosCommand]=cmdWriteSWDERegister;
				//regAdd
				g_T5L.txData[cmdPosRegWriteAddress]=regAdd;
				//data
				for(i=0;i<reglen;i++)
				{
					l_data = *pData++;
					g_T5L.txData[cmdPosRegWritesData+i] = l_data;
				}
				//crc
				if(TRUE == crcEn)
				{
					crc = cal_crc16(&g_T5L.txData[cmdPosCommand],(2+reglen));
					g_T5L.txData[cmdPosRegWritesData+reglen+0] = 0xff&(crc>>0);
					g_T5L.txData[cmdPosRegWritesData+reglen+1] = 0xff&(crc>>8);
					//total len
					total_len =  5 + reglen + 2 ;
				}
				else
				{
					//total len
					total_len = 5+reglen;
				}
				//send
				g_T5L.pUartDevice->tx_bytes(g_T5L.pUartDevice,&g_T5L.txData[0],total_len);
				g_T5L.LastSendTick = g_T5L.CurTick;
			}
		}
		//
		ret = TRUE;
	}
	return ret;
}
//========================================================================================read reg from SDWe : check:20220526
//==read reg data from SDWE thought UART
UINT8 screenSDWeReadReg(UINT8 varAdd,UINT8 varlen ,UINT8 crcEn)
{
	//??????????????????????????????0x03???0x04??????
	//?????????0xA5 0x5A 0x03 0x81 0x03 0x02 
	//?????????0xA5 0x5A 0x05 0x81 0x03 0x02 0x00 0x01
	UINT16 total_len = 0 , crc = 0 , ret = FALSE;
	if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
		((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
	{
		if(varAdd <= 0xFF)
		{
			if(((varAdd+varlen)>0)&&((varAdd+varlen)<=0xFF))
			{
				//head
				g_T5L.txData[cmdPosHead1]=T5L_RX_FUN_HEAD1;
				g_T5L.txData[cmdPosHead2]=T5L_RX_FUN_HEAD2;
				//data len
				//data len
				if(TRUE == crcEn)
				{
					g_T5L.txData[cmdPosDataLen]=0X03+02;
				}
				else
				{
					g_T5L.txData[cmdPosDataLen]=0X03;
				}
				//order:write
				g_T5L.txData[cmdPosCommand]=cmdReadSWDERegister;
				//varAdd
				g_T5L.txData[cmdPosRegReadAddress]=0xff&(varAdd);
				//len
				g_T5L.txData[cmdPosRegReadLen]=varlen&0xff;
				//crc
				if(TRUE == crcEn)
				{
					crc = cal_crc16(&g_T5L.txData[cmdPosCommand],3);
					g_T5L.txData[cmdPosRegReadLen+1+0] = 0xff&(crc>>0);
					g_T5L.txData[cmdPosRegReadLen+1+1] = 0xff&(crc>>8);
					//total len
					total_len = 6+2;
				}
				else
				{
					//total len
					total_len = 6;
				}
				//send
				g_T5L.pUartDevice->tx_bytes(g_T5L.pUartDevice,&g_T5L.txData[0],total_len);
				g_T5L.LastSendTick = g_T5L.CurTick;
				//
				//hal_delay_ms(1);
			}
		}
		//
		ret = TRUE;
	}
	return ret;
}

//======???????????? checked:20220526
UINT8 screenSDWe_JumpToPage(UINT16 pageIndex)
{
	//??????????????????2??????????????????A5 5A 04 80 03 00 02
	UINT8 pageVlu[SYS_SDWE_PAGE_REG_NUM];
	pageVlu[0] = (pageIndex>>8)&0xff;
	pageVlu[1] = (pageIndex>>0)&0xff;
	return screenSDWeWriteReg(SYS_SDWE_PAGE_REG_INDEX,&pageVlu[0],SYS_SDWE_PAGE_REG_NUM,SDWE_CRC16_EN);
}

//========recv sdwe register ask deal
UINT8 sdweAskRegData(UINT8 regAdd, UINT8 regData)
{
	UINT8 needStore = FALSE ;
	T5LType *pSdwe = &g_T5L;
	//
	pSdwe->SetAdd = regAdd ;
	pSdwe->SetData = regData ;
	switch(pSdwe->SetAdd)
	{
		case SDWE_RTC_TIMER_ADDRESS://???
			pSdwe->rtcTime[0] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
		break;
		case SDWE_RTC_TIMER_ADDRESS+1://???
			pSdwe->rtcTime[1] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
		break;
		case SDWE_RTC_TIMER_ADDRESS+2://???
			pSdwe->rtcTime[2] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
		break;
		case SDWE_RTC_TIMER_ADDRESS+3://??????
			pSdwe->rtcTime[3] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
		break;
		case SDWE_RTC_TIMER_ADDRESS+4://???
			pSdwe->rtcTime[RTC_TIME_HH_OFFSET] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
		break;
		case SDWE_RTC_TIMER_ADDRESS+5://???
			pSdwe->rtcTime[RTC_TIME_MM_OFFSET] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
		break;
		case SDWE_RTC_TIMER_ADDRESS+6://???
			pSdwe->rtcTime[RTC_TIME_SS_OFFSET] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
			//??????RTC??????
			pSdwe->getRTC = TRUE;
		break;
		default:
		break;
	}
	//
	return needStore;
}




































































//========================================================================================check:20220517
//==write varible data to SDWE thought UART
void t5lWriteVarible(UINT16 varAdd, INT16 *pData ,UINT16 varlen ,UINT8 crcEn)
{
	//A5 5A 05 82 00 03 00 01:???0x0003??????????????????0x0001
	UINT16 i = 0 ,l_data = 0 , total_len = 0 , crc = 0;

	#if(1 == SDWE_CRC16_EN)
		crcEn = 1;
	#endif
	if(varAdd < 0xFFFF)
	{
		if(((varAdd+varlen)>0)&&((varAdd+varlen)<0xFFFF))
		{
			//head
			g_T5L.txData[cmdPosHead1]=T5L_RX_FUN_HEAD1;
			g_T5L.txData[cmdPosHead2]=T5L_RX_FUN_HEAD2;
			//data len
			if(TRUE == crcEn)
			{
				g_T5L.txData[cmdPosDataLen]=0X03+2*varlen+2;//CRC
			}
			else
			{
				g_T5L.txData[cmdPosDataLen]=0X03+2*varlen;
			}
			//order:write
			g_T5L.txData[cmdPosCommand]=cmdWriteSWDEVariable;
			//varAdd
			g_T5L.txData[cmdPosVarWriteAddress1]=0xff&(varAdd>>8);
			g_T5L.txData[cmdPosVarWriteAddress2]=0xff&(varAdd>>0);
			//data
			for(i=0;i<varlen;i++)
			{
				l_data = *pData++;
				g_T5L.txData[cmdPosVarWriteData+2*i+0] = 0xff&(l_data>>8);
				g_T5L.txData[cmdPosVarWriteData+2*i+1] = 0xff&(l_data>>0);
			}
			//crc
			if(TRUE == crcEn)
			{
				crc = cal_crc16(&g_T5L.txData[cmdPosCommand],(3+2*varlen));
				g_T5L.txData[cmdPosVarWriteData+2*varlen+0] = 0xff&(crc>>0);
				g_T5L.txData[cmdPosVarWriteData+2*varlen+1] = 0xff&(crc>>8);
				//total len
				total_len = cmdPosVarWriteData+2*varlen+2;
			}
			else
			{
				//total len
				total_len = cmdPosVarWriteData+2*varlen;
			}
			//send
			g_T5L.pUartDevice->tx_bytes(g_T5L.pUartDevice,&g_T5L.txData[0],total_len);
			g_T5L.LastSendTick = g_T5L.CurTick;
			//
			hal_delay_ms(1);
		}
	}
}

//========================================================================================check:20220517
//==read varible data to SDWE thought UART
void t5lReadVarible(UINT16 varAdd,UINT16 varlen ,UINT8 crcEn)
{
	//A5 5A 04 83 00 A1 01:???????????????0x00A1?????????1
	UINT16 total_len = 0 , crc = 0;
	
	#if(1 == SDWE_CRC16_EN)
		crcEn = 1;
	#endif
	if(varAdd < 0xFFFF)
	{
		if(((varAdd+varlen)>0)&&((varAdd+varlen)<0xFFFF))
		{
			//head
			g_T5L.txData[cmdPosHead1]=T5L_RX_FUN_HEAD1;
			g_T5L.txData[cmdPosHead2]=T5L_RX_FUN_HEAD2;
			//data len
			if(TRUE == crcEn)
			{
				g_T5L.txData[cmdPosDataLen]=0X04+2;//CRC
			}
			else
			{
				g_T5L.txData[cmdPosDataLen]=0X04;
			}
			//order:write
			g_T5L.txData[cmdPosCommand]=cmdReadSWDEVariable;
			//varAdd
			g_T5L.txData[cmdPosVarWriteAddress1]=0xff&(varAdd>>8);
			g_T5L.txData[cmdPosVarWriteAddress2]=0xff&(varAdd>>0);
			//len
			g_T5L.txData[cmdPosVarReadLen]=varlen&0xff;
			//crc
			if(TRUE == crcEn)
			{
				crc = cal_crc16(&g_T5L.txData[cmdPosCommand],4);
				g_T5L.txData[cmdPosVarReadLen+1+0] = 0xff&(crc>>0);
				g_T5L.txData[cmdPosVarReadLen+1+1] = 0xff&(crc>>8);
				//total len
				total_len = cmdPosVarReadLen+1+2;
			}
			else
			{
				//total len
				total_len = cmdPosVarReadLen+1;
			}
			//send
			g_T5L.pUartDevice->tx_bytes(g_T5L.pUartDevice,&g_T5L.txData[0],total_len);
			g_T5L.LastSendTick = g_T5L.CurTick;
			//
			hal_delay_ms(1);
		}
	}
}
//========================================================================================check:20220517
//==write reg data to screen , have delay contrl
UINT8 t5lWriteReg(UINT8 regAdd, INT8 *pData ,UINT8 reglen ,UINT8 crcEn)
{
	UINT8 ret = FALSE;
	//A5 5A 05 82 00 03 00 01:???0x0003??????????????????0x0001
	UINT8 i = 0 ,l_data = 0 , total_len = 0 , crc = 0;
	
	#if(1 == SDWE_CRC16_EN)
		crcEn = 1;
	#endif
	if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
		((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
	{
		if(regAdd <= 0xFF)
		{
			if(((regAdd+reglen)>0)&&((regAdd+reglen)<=0xFF))
			{
				//head
				g_T5L.txData[cmdPosHead1]=T5L_RX_FUN_HEAD1;
				g_T5L.txData[cmdPosHead2]=T5L_RX_FUN_HEAD2;
				//data len
				if(TRUE == crcEn)
				{
					g_T5L.txData[cmdPosDataLen]=cmdPosDataLen+1*reglen+2;//CRC
				}
				else
				{
					g_T5L.txData[cmdPosDataLen]=cmdPosDataLen+1*reglen;
				}
				//order:write
				g_T5L.txData[cmdPosCommand]=cmdWriteSWDERegister;
				//regAdd
				g_T5L.txData[cmdPosRegWriteAddress]=regAdd;
				//data
				for(i=0;i<reglen;i++)
				{
					l_data = *pData++;
					g_T5L.txData[cmdPosRegWritesData+i] = l_data;
				}
				//crc
				if(TRUE == crcEn)
				{
					crc = cal_crc16(&g_T5L.txData[cmdPosCommand],(2+reglen));
					g_T5L.txData[cmdPosRegWritesData+i+0] = 0xff&(crc>>0);
					g_T5L.txData[cmdPosRegWritesData+i+1] = 0xff&(crc>>8);
					//total len
					total_len =  g_T5L.txData[cmdPosDataLen] + 3 ;
				}
				else
				{
					//total len
					total_len = g_T5L.txData[cmdPosDataLen] + 3;
				}
				//send
				g_T5L.pUartDevice->tx_bytes(g_T5L.pUartDevice,&g_T5L.txData[0],total_len);
				g_T5L.LastSendTick = g_T5L.CurTick;
				//
				ret = TRUE;
			}
		}
	}
	return ret;
}

//==read varible data to SDWE thought UART
void t5lReadReg(UINT8 varAdd,UINT8 varlen ,UINT8 crcEn)
{
	//A5 5A 04 83 00 A1 01:???????????????0x00A1?????????1
	UINT16 total_len = 0 , crc = 0;
	
#if(1 == SDWE_CRC16_EN)
		crcEn = 1;
#endif
	if(varAdd <= 0xFF)
	{
		if(((varAdd+varlen)>0)&&((varAdd+varlen)<=0xFF))
		{
			//head
			g_T5L.txData[cmdPosHead1]=T5L_RX_FUN_HEAD1;
			g_T5L.txData[cmdPosHead2]=T5L_RX_FUN_HEAD2;
			//data len
			g_T5L.txData[cmdPosDataLen]=0X04;
			//order:write
			g_T5L.txData[cmdPosCommand]=cmdReadSWDERegister;
			//varAdd
			g_T5L.txData[cmdPosRegReadAddress]=0xff&(varAdd);
			//len
			g_T5L.txData[cmdPosRegReadLen]=varlen&0xff;
			//crc
			if(TRUE == crcEn)
			{
				crc = cal_crc16(&g_T5L.txData[cmdPosCommand],3);
				g_T5L.txData[cmdPosRegReadLen+1+0] = 0xff&(crc>>0);
				g_T5L.txData[cmdPosRegReadLen+1+1] = 0xff&(crc>>8);
				//total len
				total_len = cmdPosRegReadLen+1+2;
			}
			else
			{
				//total len
				total_len = cmdPosRegReadLen+1;
			}
			//send
			g_T5L.pUartDevice->tx_bytes(g_T5L.pUartDevice,&g_T5L.txData[0],total_len);
			g_T5L.LastSendTick = g_T5L.CurTick;
			//
			hal_delay_ms(1);
		}
	}
}



















//==write data to screen , have delay contrl
UINT8 t5lWriteData(UINT16 varAdd, INT16 *pData ,UINT16 varlen ,UINT8 crcEn)
{
	UINT8 ret = FALSE;
	//A5 5A 05 82 00 03 00 01:???0x0003??????????????????0x0001
	UINT16 i = 0 ,l_data = 0 , total_len = 0 , crc = 0;
	if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
		((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
	{
		if(varAdd < 0xFFFF)
		{
			if(((varAdd+varlen)>0)&&((varAdd+varlen)<0xFFFF))
			{
				//head
				g_T5L.txData[cmdPosHead1]=T5L_RX_FUN_HEAD1;
				g_T5L.txData[cmdPosHead2]=T5L_RX_FUN_HEAD2;
				//data len
				g_T5L.txData[cmdPosDataLen]=0X03+2*varlen;
				//order:write
				g_T5L.txData[cmdPosCommand]=cmdWriteSWDEVariable;
				//varAdd
				g_T5L.txData[cmdPosVarWriteAddress1]=0xff&(varAdd>>8);
				g_T5L.txData[cmdPosVarWriteAddress2]=0xff&(varAdd>>0);
				//data
				for(i=0;i<varlen;i++)
				{
					l_data = *pData++;
					g_T5L.txData[cmdPosVarWriteData+2*i+0] = 0xff&(l_data>>8);
					g_T5L.txData[cmdPosVarWriteData+2*i+1] = 0xff&(l_data>>0);
				}
				//crc
				if(TRUE == crcEn)
				{
					crc = cal_crc16(&g_T5L.txData[cmdPosCommand],(3+2*varlen));
					g_T5L.txData[cmdPosVarWriteData+2*varlen+0] = 0xff&(crc>>8);
					g_T5L.txData[cmdPosVarWriteData+2*varlen+1] = 0xff&(crc>>0);
					//total len
					total_len = cmdPosVarWriteData+2*varlen+2;
				}
				else
				{
					//total len
					total_len = cmdPosVarWriteData+2*varlen;
				}
				//send
				g_T5L.pUartDevice->tx_bytes(g_T5L.pUartDevice,&g_T5L.txData[0],total_len);
				g_T5L.LastSendTick = g_T5L.CurTick;
				//
				ret = TRUE;
			}
		}
	}
	return ret;
}


//if screen at calibration page point trigerd
void pointWeightTrigerDataSet(UINT8 localChanel , UINT8 point , INT16 value)
{
	if(localChanel > HX711_CHANEL_NUM)
		return ;

	if(point < CHANEL_POINT_NUM)
	{
		g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_ASK_WEIGHT][point] = value;//point weight triger	
	}
}
//if screen at calibration page point trigerd
void pointSampleTrigerDataSet(UINT8 localChanel , UINT8 point , INT16 value)
{
	if(localChanel > HX711_CHANEL_NUM)
		return ;

	if(point < CHANEL_POINT_NUM)
	{
		g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_AVG_SAMPLE][point] = value;//point sample triger	
	}
}

//if screen at calibration page point trigerd
void pointTrigerDataSet(UINT8 localChanel , UINT8 point , UINT8 value ,INT16 avgSampleValue)
{	
	if(localChanel > HX711_CHANEL_NUM)
		return ;

	if(point < CHANEL_POINT_NUM)
	{
		g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_STATUS][point] = TRUE;//point triger need answer flag	
		g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_ASK_COLOR][point] = value;//point triger color answer	
		g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_AVG_SAMPLE][point] = avgSampleValue;//point triger avg sample value answer
	}
}

//if sreen calibtion point triger
UINT8 pointTrigerDeal()
{
	static UINT8 inerStatus = 0 , localChanel = 0 ;	
	INT16 *pSendData= 0;
	UINT8 result = 0 ;

	if(g_T5L.CalibrateChanel > HX711_CHANEL_NUM)
		return 0 ;

	//chanel get
	if(0 == g_T5L.CalibrateChanel)
	{
		localChanel = HX711_CHANEL_NUM ;
	}
	else if(g_T5L.CalibrateChanel <= HX711_CHANEL_NUM)
	{
		localChanel = g_T5L.CalibrateChanel - 1 ;
	}

	//status
	switch(inerStatus)
	{
		case 0://send Color
			if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
				((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
			{
				pSendData= &g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_ASK_COLOR][0];//color:1 green 0:white
				t5lWriteVarible(SCREEN_ASK_CHANEL_POINT_TRIG_BACK_COLOR_ADDRESS,pSendData,(CHANEL_POINT_NUM),1);
				//
				inerStatus++ ;
			}
		break;
		case 1://send sample data
			{
				if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
					((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
				{
					pSendData= &g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_AVG_SAMPLE][0];//data
					t5lWriteVarible(SCREEN_ASK_CHANEL_POINT_TRIG_SAMPLE_DATA_ADDRESS,pSendData,(CHANEL_POINT_NUM),1);
					//
					inerStatus++ ;
				}
			}
			break;
		default:
			inerStatus = 0 ;
			result = 1 ;
			break;
	}

	return result;
}








//==updata sdwe weight color
void sdweSetWeightBackColor(UINT8 seq,UINT8 color)
{
	if(seq < HX711_CHANEL_NUM)
	{
		//0~HX711_CHANEL_NUM:weight
		//HX711_CHANEL_NUM~2*HX711_CHANEL_NUM:color
		g_t5l_dis_data[HX711_CHANEL_NUM+seq] = color;
	}
}
void color_clearAllColor(void)
{
	UINT8 seq = HX711Chanel_1;
	for(seq=HX711Chanel_1;seq<HX711_CHANEL_NUM;seq++)
	{
		g_t5l_dis_data[HX711_CHANEL_NUM+seq] = LED_COLOR_NONE;
	}
}

//clear data to screen at calibration page
void clearLocalCalibrationRecordData(UINT8 sreen_chanel)
{
	UINT8 chane_i = 0 , point_j = 0 ;
	
	if(0 == sreen_chanel)//clear all
	{
		for(chane_i=0;chane_i<(HX711_CHANEL_NUM+1);chane_i++)
		{
			//==1:clear sreen needs back color and sample data
			for(point_j=0;point_j<CHANEL_POINT_NUM;point_j++)
			{
				//back color
				g_t5l_triger_data[chane_i][DMG_TRIGER_SAMPLE_OF_ASK_COLOR][point_j] = 0 ;//color:1 green 0:white
				//data
				g_t5l_triger_data[chane_i][DMG_TRIGER_SAMPLE_OF_AVG_SAMPLE][point_j] = 0 ;//sample data = 0
			}
		}
	}
	else if(sreen_chanel <= HX711_CHANEL_NUM)
	{
		chane_i = sreen_chanel - 1 ;
		//==1:clear sreen needs back color and sample data
		for(point_j=0;point_j<CHANEL_POINT_NUM;point_j++)
		{
			//back color
			g_t5l_triger_data[chane_i][DMG_TRIGER_SAMPLE_OF_ASK_COLOR][point_j] = 0 ;//color:1 green 0:white
			//data
			g_t5l_triger_data[chane_i][DMG_TRIGER_SAMPLE_OF_AVG_SAMPLE][point_j] = 0 ;//sample data = 0
		}
	}
}

//clear local recode k and b and sample data
void clearLocalCalibrationKAndBAndSample(UINT8 sreen_chanel)
{
	UINT8 chane_i = 0 , point_j = 0 ;
	ChanelType *pChanel=0;
	
	if(0 == sreen_chanel)//clear all
	{
		for(chane_i=0;chane_i<HX711_CHANEL_NUM;chane_i++)
		{
			//==1:clear local sample data and k and b
			//get chanel
			pChanel = getChanelStruct(chane_i);
			//clear local sample point
			for(point_j=0;point_j<CHANEL_POINT_NUM;point_j++)
			{
				pChanel->section_PointSample[point_j] = 0 ;
			}
			//clear local k & b
			for(point_j=0;point_j<(CHANEL_POINT_NUM+1);point_j++)
			{
				pChanel->section_K[point_j] = 0.0 ;
				pChanel->section_B[point_j] = 0.0 ;
			}
		}
	}
	else if(sreen_chanel <= HX711_CHANEL_NUM)
	{
		chane_i = sreen_chanel - 1 ;

		//==1:clear local sample data and k and b
		//get chanel
		pChanel = getChanelStruct(chane_i);
		//clear local sample point
		for(point_j=0;point_j<CHANEL_POINT_NUM;point_j++)
		{
			pChanel->section_PointSample[point_j] = 0 ;
		}
		//clear local k & b
		for(point_j=0;point_j<(CHANEL_POINT_NUM+1);point_j++)
		{
			pChanel->section_K[point_j] = 0.0 ;
			pChanel->section_B[point_j] = 0.0 ;
		}
	}	
}

void storeSysPara_3030(UINT16 varAdd, UINT16 varData)
{
	switch(varAdd)
	{
		case DMG_FUNC_SET_UNIT_ADDRESS://		(0X1000)//0x1000
			gSystemPara.uint = varData;
		break;
		case DMG_FUNC_SET_MIN_RANGE_ADDRESS://		(0X100A)//0x100A
		break;
		case  DMG_FUNC_SET_MAX_RANGE_ADDRESS://		(0X100B)//0x100B
		break;
		case DMG_FUNC_SET_ERR_RANGE_ADDRESS://		(0X100C)//0x100C
		break;
		case  DMG_FUNC_SET_isCascade_ADDRESS://		(0X100D)//0x100D
		break;
		case  DMG_FUNC_SET_isLedIndicate_ADDRESS://	(0X100E)//0x100E
		break;
		case  DMG_FUNC_SET_COLOR_START_ADDRESS://	(0X100F)//0x100F
		break;
		case DMG_FUNC_SET_COLOR_END_ADDRESS://		(0X1012)//0x1012
		break;
		case DMG_FUNC_SET_ZERO_RANGE_ADDRESS://		(0X1013)//0x1013
		break;
		default:
		break;
	}

}

//==recv sdwe variable ask deal
UINT8 sdweAskVaribleData(UINT16 varAdd, UINT16 varData)
{
	UINT8 needStore = FALSE ;
	UINT8 i = 0 , point = 0;
	INT32 weight=0,avgSampleValue=0;
	T5LType *pSdwe = &g_T5L;
	//
	pSdwe->SetAdd = varAdd ;
	pSdwe->SetData = varData ;
	//receive address from SDWE
	if(0xffff != pSdwe->SetAdd)
	{
		if((SDWE_TRIGGER_STOP_ROCK_ADDRESS == pSdwe->SetAdd) && (SDWE_TRIGGER_STOP_ROCK_ADDRESS == pSdwe->SetData))
		{
			g_T5L.u16_TriggerStopRock = TRUE;
		}
		else if((SDWE_50_VOICE_PRINTF_CHOICE == pSdwe->SetAdd) && (SDWE_50_VOICE_PRINTF_CHOICE_EXCHANGE_VLU == pSdwe->SetData))
		{
			if(0 == gSystemPara.u32_50PerVoicePrintf)
			{
				gSystemPara.u32_50PerVoicePrintf = 1;
			}
			else
			{
				gSystemPara.u32_50PerVoicePrintf = 0 ;
			}
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
			g_T5L.u16_508090SetPercentEvent = SYS_CTL_EVENT_VALID;
		}
		else if((SDWE_80_VOICE_PRINTF_CHOICE == pSdwe->SetAdd) && (SDWE_80_VOICE_PRINTF_CHOICE_EXCHANGE_VLU == pSdwe->SetData))
		{
			if(0 == gSystemPara.u32_80PerVoicePrintf)
			{
				gSystemPara.u32_80PerVoicePrintf = 1;
			}
			else
			{
				gSystemPara.u32_80PerVoicePrintf = 0 ;
			}
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
			g_T5L.u16_508090SetPercentEvent = SYS_CTL_EVENT_VALID;
		}
		else if((SDWE_90_VOICE_PRINTF_CHOICE == pSdwe->SetAdd) && (SDWE_90_VOICE_PRINTF_CHOICE_EXCHANGE_VLU == pSdwe->SetData))
		{
			if(0 == gSystemPara.u32_90PerVoicePrintf)
			{
				gSystemPara.u32_90PerVoicePrintf = 1;
			}
			else
			{
				gSystemPara.u32_90PerVoicePrintf = 0 ;
			}
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
			g_T5L.u16_508090SetPercentEvent = SYS_CTL_EVENT_VALID;
		}
		else if(SDWE_LEVEL_200_MIN_WEIGHT == pSdwe->SetAdd)
		{
			gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_200][0] = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_LEVEL_200_MAX_WEIGHT == pSdwe->SetAdd)
		{
			gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_200][1] = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_LEVEL_300_MIN_WEIGHT == pSdwe->SetAdd)
		{
			gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_300][0] = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_LEVEL_300_MAX_WEIGHT == pSdwe->SetAdd)
		{
			gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_300][1] = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_LEVEL_400_MIN_WEIGHT == pSdwe->SetAdd)
		{
			gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_400][0] = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_LEVEL_400_MAX_WEIGHT == pSdwe->SetAdd)
		{
			gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_400][1] = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if((SDWE_BACKANDSET_A_CONTRL_ADDRESS == pSdwe->SetAdd) && (SDWE_BACKANDSET_A_CONTRL_ADDRESS == pSdwe->SetData))
		{
			//g_T5L.backReturnASetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_VALID;//?????????
		}
		else if((SDWE_BACKANDSET_B_CONTRL_ADDRESS == pSdwe->SetAdd) && (SDWE_BACKANDSET_B_CONTRL_ADDRESS == pSdwe->SetData))
		{
			g_T5L.backReturnBSetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_VALID;
		}
		else if((SDWE_GJF_CONTRL_ADDRESS == pSdwe->SetAdd) && (SDWE_GJF_CONTRL_ADDRESS == pSdwe->SetData))
		{
			//???????????????????????? ????????? ???/??? ??? ???????????????????????????
			if((SDWeCurPage_KaiShiJieMian == g_T5L.curPage) && (TRUE == gSystemPara.u16_kaiqi_guanjiafa_gongneng))
			{
				g_T5L.gjfContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_VALID;
			}
		}
		else if((SDWE_RUN_CONTRL_ADDRESS == pSdwe->SetAdd) && (SDWE_RUN_CONTRL_ADDRESS == pSdwe->SetData))
		{
			g_T5L.runContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_VALID;
		}
		else if(SDWE_ENTER_REALTIME_CONTRL_ADDRESS == pSdwe->SetAdd)
		{
			g_T5L.enterRealTimeSetContrl[SYS_CTL_REG_STATUS_INDEX] = pSdwe->SetData;
			g_T5L.enterRealTimeSetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_VALID;
		}else if(SDWE_RTC_TIME_SET_HH_ADDRESS == pSdwe->SetAdd)
		{	
			g_T5L.rtcSetTime[RTC_TIME_HH_OFFSET] = pSdwe->SetData%24;
			g_T5L.rtcSetTime[RTC_TIME_MM_OFFSET] = g_T5L.rtcTime[RTC_TIME_MM_OFFSET];
			g_T5L.rtcSetTime[RTC_TIME_SS_OFFSET] = g_T5L.rtcTime[RTC_TIME_SS_OFFSET];

			g_T5L.rtcSet = TRUE;
		}else if(SDWE_RTC_TIME_SET_MM_ADDRESS == pSdwe->SetAdd)
		{
			g_T5L.rtcSetTime[RTC_TIME_HH_OFFSET] = g_T5L.rtcTime[RTC_TIME_HH_OFFSET];
			g_T5L.rtcSetTime[RTC_TIME_MM_OFFSET] = pSdwe->SetData%60;
			g_T5L.rtcSetTime[RTC_TIME_SS_OFFSET] = g_T5L.rtcTime[RTC_TIME_SS_OFFSET];

			g_T5L.rtcSet = TRUE;
		}else if(SDWE_RTC_TIME_SET_SS_ADDRESS == pSdwe->SetAdd)
		{
			g_T5L.rtcSetTime[RTC_TIME_HH_OFFSET] = g_T5L.rtcTime[RTC_TIME_HH_OFFSET];
			g_T5L.rtcSetTime[RTC_TIME_MM_OFFSET] = g_T5L.rtcTime[RTC_TIME_MM_OFFSET];
			g_T5L.rtcSetTime[RTC_TIME_SS_OFFSET] = pSdwe->SetData%60;

			g_T5L.rtcSet = TRUE;
		}
		else if(SDWE_YUSHEZHONGLIANG_200_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.yusheWeight[SDWeCaiJiDangWei_200] = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_YUSHEZHONGLIANG_300_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.yusheWeight[SDWeCaiJiDangWei_300] = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_YUSHEZHONGLIANG_400_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.yusheWeight[SDWeCaiJiDangWei_400] = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_YAOBAIJIAODU_ADDRESS == pSdwe->SetAdd)
		{
			motorCtl.motorAge = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_YAOBAISHIJIAN_ADDRESS == pSdwe->SetAdd)
		{
			motorCtl.tAD = pSdwe->SetData;
			gSystemPara.u32_MotorRockFre = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_ML_G_BILU_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.u16_ml_g_bilv = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_FMQKQD_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.u16_fmqkqsj = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_LIUSUDIAN_GAO_BAOJIN_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.u16_liusu_high_warn = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_LIUSUDIAN_DI_BAOJIN_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.u16_liusu_low_warn = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_VOICE_NUM_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.ScreenVoiceSwitch = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_SCREEN_LIGHT_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.ScreenLight = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_ERR_RANGE_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.errRange = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_MAX_RANGE_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.maxWeight = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_MIN_RANGE_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.minWeight = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_ZERO_RANGE_ADDRESS == pSdwe->SetAdd)
		{
			gSystemPara.zeroRange = pSdwe->SetData;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SCREEN_REMOVE_WEIGHT_ADDRESS == pSdwe->SetAdd)
		{
			if(SCREEN_REMOVE_WEIGHT_TRIG_VAL == (UINT16)pSdwe->SetData)
			{
				pSdwe->sdweRemoveWeightTriger = TRUE;
			}
		}
		else if(DMG_FUNC_SET_MIN_RANGE_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.minWeight = pSdwe->SetData;/**< ???????????? */
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_MAX_RANGE_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.maxWeight = pSdwe->SetData;/**< ???????????? */
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_ERR_RANGE_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.errRange = pSdwe->SetData;/**< ???????????? */
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_ZERO_RANGE_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.zeroRange = pSdwe->SetData;/**< ???????????? */
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_SCREEN_LIGHT_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.ScreenLight = pSdwe->SetData;/**< ?????????????????? */
			g_T5L.sdweFreshScreenLight = TRUE;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_VOICE_NUM_TOUCH_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.VoiceNumTouch = pSdwe->SetData;/**< ???????????? ??????*/
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_VOICE_NUM_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.VoiceNum = pSdwe->SetData;/**< ???????????? */
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_GUANJIAFAGONGNENGKAIQI_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.u16_kaiqi_guanjiafa_gongneng = pSdwe->SetData;/**< ????????????????????? */	
			if(1 != pSdwe->SetData)
			{
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FK_FG,SDWeFaKaiFaGuan_KongBai);//??????/??? = ??????
			}
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		//==(update:20210515):Balancing JUMP CLEAN PAGE
		else if(DMG_FUNC_Balancing_CLEARPAGE_SET_ADDRESS == pSdwe->SetAdd)
		{
			if(DMG_FUNC_Balancing_CLEARPAGE_SET_VALUE == (UINT16)pSdwe->SetData)
			{
				pSdwe->sdweJumpBalancing_cleanpagee = TRUE;
			}
		}
		//==(update:20210515):Balancing JUMP
		else if(DMG_FUNC_Balancing_HOME_SET_ADDRESS == pSdwe->SetAdd)
		{
			if(DMG_FUNC_Balancing_HOME_SET_VALUE == (UINT16)pSdwe->SetData)
			{
				pSdwe->sdweJumpBalancing_home = TRUE;
			}
		}
		//==(update:20210515):Balancing JUMP
		else if(DMG_FUNC_Balancing_SET_ADDRESS == pSdwe->SetAdd)
		{
			if(DMG_FUNC_Balancing_SET_VALUE == (UINT16)pSdwe->SetData)
			{
				pSdwe->sdweJumpBalancing = TRUE;
			}
		}
		//==(update:20210328):chanel choice:0->all chanel , 1~8:single chanel
		else if(SCREEN_FUNC_SET_CHANEL_NUM == pSdwe->SetAdd)
		{
			pSdwe->ResetTrigerValid = FALSE;/*??????????????????*/
			if(pSdwe->CalibrateChanel != pSdwe->SetData)
			{
				pSdwe->sdweChanelChanged = TRUE;
				if(pSdwe->SetData <= HX711_CHANEL_NUM)
				{
					pSdwe->CalibrateChanel = pSdwe->SetData;//chanel
				}
			}
		}//==(update:20210328):reset calibration
		else if(SCREEN_RESET_CALIBRATION_ADDRESS == pSdwe->SetAdd)
		{
			if(DMG_FUNC_RESET_CALIBRATION_VAL == (UINT16)pSdwe->SetData)
			{
				pSdwe->sdweResetTriger = TRUE;
				pSdwe->ResetTrigerValid = TRUE;
				clearLocalCalibrationRecordData(pSdwe->CalibrateChanel);
				clearLocalCalibrationKAndBAndSample(pSdwe->CalibrateChanel);
			}
		}
		//==(update:20210428):reset calibration
		else if(SCREEN_JUNPTO_CALIBRATION_ADDRESS == pSdwe->SetAdd)
		{
			if(SCREEN_JUNPTO_CALIBRATION_TRIG_VAL == (UINT16)pSdwe->SetData)
			{
				pSdwe->sdweJumpToCalitrationPage = TRUE;
			}
			else if(SCREEN_JUNPTO_ACTIVE_TRIG_VAL == (UINT16)pSdwe->SetData)
			{
				pSdwe->sdweJumpActivePage = TRUE;
			}
		}//==(update:20211119):address of syspara entry
		else if(DMG_FUNC_JUNPTO_SYSPAR_ADDRESS == pSdwe->SetAdd)
		{
			if(DMG_FUNC_JUNPTO_SYSPAR_VAL == (UINT16)pSdwe->SetData)
			{
				pSdwe->sdweJumpToSysParaPage = TRUE;
			}
		}//==(update:20210328):remove all weight value
		else if(SCREEN_REMOVE_WEIGHT_ADDRESS == pSdwe->SetAdd)
		{
			if(SCREEN_REMOVE_WEIGHT_TRIG_VAL == (UINT16)pSdwe->SetData)
			{
				pSdwe->sdweRemoveWeightTriger = TRUE;
				//
				#if(0 == CURRENT_PRODUCT)
				#endif
			}
		}//==(update:20210328):chanel point weight value set
		else if((pSdwe->SetAdd >= SCREEN_SET_CHANEL_POINT_ADDRESS)&&(pSdwe->SetAdd < (SCREEN_SET_CHANEL_POINT_ADDRESS + CHANEL_POINT_NUM )))
		{
			needStore = DMG_TRIGER_SAVE_SECOTOR_1 ;
			//point
			pSdwe->CalibratePoint = (pSdwe->SetAdd -SCREEN_SET_CHANEL_POINT_ADDRESS) ;//point
			point = pSdwe->CalibratePoint;
			pSdwe->CalibratePointArry[point] = pSdwe->SetData;
			//weight
			weight = pSdwe->SetData;
		
			if(0 == pSdwe->CalibrateChanel)//all chanel point weight value set
			{
				for(i=0;i<HX711_CHANEL_NUM;i++)//8??????
				{
					setSampleWeightValue(i,point,weight);
					pointWeightTrigerDataSet(i,point,weight);
				}
				pointWeightTrigerDataSet(i,point,weight);
			}
			else//single chanel point weight value set
			{
				setSampleWeightValue((pSdwe->CalibrateChanel-1),point,weight);
				pointWeightTrigerDataSet((pSdwe->CalibrateChanel-1),point,weight);
			}
		}//triger calculate
		else if((TRUE == pSdwe->ResetTrigerValid)&&(pSdwe->SetAdd >= SCREEN_SET_CHANEL_POINT_TRIG_ADDRESS)&&(pSdwe->SetAdd < (SCREEN_SET_CHANEL_POINT_TRIG_ADDRESS + CHANEL_POINT_NUM )))
		{
			//value = 0x12fe
			if(SCREEN_SET_CHANEL_POINT_TRIG_VAL == pSdwe->SetData)
			{
				//	
				pSdwe->sdwePointTriger = TRUE;
				//
				needStore = DMG_TRIGER_SAVE_SECOTOR_1 ;
				point = ( pSdwe->SetAdd - SCREEN_SET_CHANEL_POINT_TRIG_ADDRESS );
				
				if(0 == pSdwe->CalibrateChanel)//all chanel caculate	K & B
				{
					//avgSampleValue = hx711_getAvgSample(pSdwe->CalibrateChanel)/512;
					for(i=0;i<HX711_CHANEL_NUM;i++)//eight chanel
					{
						avgSampleValue = hx711_getAvgSample((enumHX711ChanelType)i)/512;
						trigerCalcKB(i,point);
						pointTrigerDataSet(i,point,1,avgSampleValue);
					}
					pointTrigerDataSet(HX711_CHANEL_NUM,point,1,avgSampleValue);
					
				}
				else if(HX711_CHANEL_NUM >= pSdwe->CalibrateChanel)//single chanel caculate  K & B
				{
					avgSampleValue = hx711_getAvgSample((enumHX711ChanelType)(pSdwe->CalibrateChanel-1))/512;
					trigerCalcKB((pSdwe->CalibrateChanel-1),point);
					pointTrigerDataSet((pSdwe->CalibrateChanel-1),point,1,avgSampleValue);
				}
				//sdwePointTrigerUpdata(point,1,avgSampleValue);
			}
		}
		else if(pSdwe->SetAdd == DMG_SYS_STATUS_OF_VOICE_PRINTF_00A1)
		{
			g_u8Read00A1_Data = pSdwe->SetData;
		}
		//clr address
		pSdwe->SetAdd = 0xffff;
	}
	return needStore;
}






//if need jump to active page 
UINT8 jumpToActivePage()
{
	UINT8 result = 0 ;
	//5A A5 07 82 0084 5A01 page
	INT16 pageChangeOrderAndData[2]={0x5A01,56};//56 page
	if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
		((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
	{
		t5lWriteVarible((0X0084),pageChangeOrderAndData,2,0);
		result = 1;
	}
	return result;
}
//if need jump to Syspara page 
UINT8 jumpToSysparaPage()
{
	UINT8 result = 0 ;
	//5A A5 07 82 0084 5A01 page
	INT16 pageChangeOrderAndData[2]={0x5A01,52};//52 page
	if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
		((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
	{
		t5lWriteVarible((0X0084),pageChangeOrderAndData,2,0);
		result = 1;
	}
	return result;
}





//if need jump to calibration page 
UINT8 jumpToCalibrationPage()
{
	UINT8 result = 0 ;
	if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
		((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
	{
		screenSDWe_JumpToPage(SDWeCurPage_JiaoZhunJieMianMian);
		result = 1;
	}
	return result;
}


//send screen light 
UINT8 sendScreenLight()
{
	UINT8 result = 0 ;
	//5A A5 05 82 0082 5A01 page 0XEA60
	INT16 pageChangeOrderAndData[2]={0x6423,0XEA60};//0x64:black light ,0x23:standby light ,entry standby x*0.01S
	pageChangeOrderAndData[0] &= 0x00ff;
	pageChangeOrderAndData[0] |= (0xff00&(gSystemPara.ScreenLight<<8));
	if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
		((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
	{
		t5lWriteVarible((0X0082),pageChangeOrderAndData,2,0);
		result = 1;
	}
	return result;
}

//if reset calibration valid 
//prepare DMG display of color and sample avg data
UINT8 resetCalibrationTrigerDeal()
{
	static UINT8 inerStatus = 0 , localChanel = 0 ;	

	INT16 *pSendData= 0;
	UINT8 result = 0 ;

	if(g_T5L.CalibrateChanel > HX711_CHANEL_NUM)
		return 0 ;

	//chanel get
	if(0 == g_T5L.CalibrateChanel)
	{
		localChanel = 0 ;
	}
	else if(g_T5L.CalibrateChanel <= HX711_CHANEL_NUM)
	{
		localChanel = g_T5L.CalibrateChanel ;
	}
	//status
	switch(inerStatus)
	{
		case 0://send Color
			if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
				((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
			{
				pSendData= &g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_ASK_COLOR][0];//color:1 green 0:white
				t5lWriteVarible(SCREEN_ASK_CHANEL_POINT_TRIG_BACK_COLOR_ADDRESS,pSendData,(CHANEL_POINT_NUM),0);
				//
				inerStatus++ ;
			}
		break;
		case 1://send data
			{
				if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
					((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
				{
					pSendData= &g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_AVG_SAMPLE][0];//data
					t5lWriteVarible(SCREEN_ASK_CHANEL_POINT_TRIG_SAMPLE_DATA_ADDRESS,pSendData,(CHANEL_POINT_NUM),0);
					//
					inerStatus++ ;
				}
			}
			break;
		default:
			inerStatus = 0 ;
			result = 1 ;
			break;
	}

	return result;
}


//if sreen chanel changed
UINT8 chanelChangedTrigerDeal()
{
	static UINT8 inerStatus = 0 , localChanel = 0 ;	

	INT16 *pSendData= 0 ;
	UINT8 result = 0 ;
	
	if(g_T5L.CalibrateChanel > HX711_CHANEL_NUM)
		return 0 ;

	//chanel get
	if(0 == g_T5L.CalibrateChanel)
	{
		localChanel = HX711_CHANEL_NUM ;
	}
	else if(g_T5L.CalibrateChanel <= HX711_CHANEL_NUM)
	{
		localChanel = g_T5L.CalibrateChanel - 1 ;
	}
	
	//status
	switch(inerStatus)
	{
		case 0://send back Color
			if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
				((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
			{
				pSendData= &g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_ASK_COLOR][0];//color:1 green 0:white
				t5lWriteVarible(SCREEN_ASK_CHANEL_POINT_TRIG_BACK_COLOR_ADDRESS,pSendData,(CHANEL_POINT_NUM),0);
				//
				inerStatus++ ;
			}
		break;
		case 1://send avg sample data
			{
				if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
					((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
				{
					pSendData= &g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_AVG_SAMPLE][0];//avg sample data
					t5lWriteVarible(SCREEN_ASK_CHANEL_POINT_TRIG_SAMPLE_DATA_ADDRESS,pSendData,(CHANEL_POINT_NUM),0);
					//
					inerStatus++ ;
				}
			}
			break;
		case 2://send weight point
			{
				if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
					((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
				{
					pSendData= &g_t5l_triger_data[localChanel][DMG_TRIGER_SAMPLE_OF_ASK_WEIGHT][0];//weight point data
					t5lWriteVarible(SCREEN_SET_CHANEL_POINT_ADDRESS,pSendData,(CHANEL_POINT_NUM),0);
					//
					inerStatus++ ;
				}
			}
			break;
		case 3://send chanel
			{
				if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
					((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
				{

					pSendData = (INT16 *)&(g_T5L.CalibrateChanel);		
					t5lWriteVarible(SCREEN_FUNC_SET_CHANEL_NUM,pSendData,1,0);
					//
					inerStatus++ ;
				}
			}
			break;
		default:
			inerStatus = 0 ;
			result = 1 ;
			break;
	}

	return result;

}






//RTC????????????
void screenSDWe_RTCTimeCaculate(void)
{
	//????????????????????????RTC?????????
	//if(TRUE == g_T5L.getRTC)
	{
		if(g_sys_ms_tick - g_T5L.rtcTime_ms >= 1000 )//??????1s????????? rtcTime
		{
			g_T5L.rtcTime_ms = g_sys_ms_tick;
			//
			g_T5L.rtcTime[RTC_TIME_SS_OFFSET]++;
			if(g_T5L.rtcTime[RTC_TIME_SS_OFFSET] >= 60)//???
			{
				g_T5L.rtcTime[RTC_TIME_MM_OFFSET]++;
				g_T5L.rtcTime[RTC_TIME_SS_OFFSET] = 0;
			}
			if(g_T5L.rtcTime[RTC_TIME_MM_OFFSET] >= 60)//???
			{
				g_T5L.rtcTime[RTC_TIME_HH_OFFSET]++;
				g_T5L.rtcTime[RTC_TIME_MM_OFFSET] = 0;
			}
			if(g_T5L.rtcTime[RTC_TIME_HH_OFFSET] >= 24)//???
			{
				g_T5L.rtcTime[RTC_TIME_MM_OFFSET] = 0;
			}
			SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_HH_1,g_T5L.rtcTime[RTC_TIME_HH_OFFSET]);
			SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_MM_1,g_T5L.rtcTime[RTC_TIME_MM_OFFSET]);
			SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_SS_1,g_T5L.rtcTime[RTC_TIME_SS_OFFSET]);	
		}
	}
}

//==??????????????????
void screenSDWe_CaiJiYuYin_Set(enumSDWEVoicePrintfType voicePrintf,UINT8 setVlu)
{
	g_T5L.cjyyts[voicePrintf - SDWEVoicePrintf_CaiJi_KaiShi] = setVlu;
}
//==??????????????????
UINT8 screenSDWe_CaiJiYuYin_Get(enumSDWEVoicePrintfType voicePrintf)
{
	return g_T5L.cjyyts[voicePrintf - SDWEVoicePrintf_CaiJi_KaiShi];
}


//???????????????????????????
UINT8 screenVoicePrintf_Handle(UINT16 eventVlu)
{
	//????????????????????????????????????????????????????????????????????????6.wav?????????100%?????????????????????????????? 
	//A5 5A 07 80 50 5b 00 06 5A 40 
	//???????????????????????????????????????A5 5A 05 80 50 5c 00 06 

	UINT8 ret = FALSE;
	static UINT8 regData[SDWE_VOICE_PRINTF_SET_REG_LEN]={0X5B,00,00,0X5A,0X40};
	static UINT8 status = 0;
	
	(void)eventVlu;
	switch(status)
	{
		case 0://????????????
			status = 1;
		break;
		case 1://???????????? ?????????
			regData[0] = 0X5B;
			eventVlu = eventVlu%4096;
			regData[1] = (eventVlu>>8)&0XFF;
			regData[2] = (eventVlu>>0)&0XFF;
			regData[3] = 0X5A;//????????????
			regData[4] = gSystemPara.ScreenVoiceSwitch;//    0X30;//????????????
			
			if(TRUE == screenSDWeWriteReg(SDWE_VOICE_PRINTF_SET_REG_ADDRESS,&regData[0],SDWE_VOICE_PRINTF_SET_REG_LEN,SDWE_CRC16_EN))
			{
				status = 0 ;
				ret = TRUE;
			}
		break;
		case 2://????????????
		break;
		default:
			status = 0 ;
		break;
	}

	//
	if(ret == TRUE)
	{
		status = 0 ;
	}
	//
	return ret;
}


#define SDWE_CAIJI_WANCHENG_DELAY	(1000)
#define SDWE_CAIJI_WEIWANCHENG_DELAY	(1000)

void screenSDWe_CaiJiWanCheng_Handle(void)
{
	//?????????????????? ??????????????????
	if(SDWeCurPage_ShiShiJieMian == g_T5L.curPage)
	{
		if((FALSE == g_T5L.sampleComplete) && (GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) >= 100))
		{
			g_T5L.sampleComplete = TRUE;//????????????
			
			//????????????
			app_gjf_handle(SDWeFaKaiFaGuan_FaGuan);

			//??????????????????
			app_SetMotorContrlMode(MotorCtrlStatus_EN_NormalRun_WitMaxPos);//??????????????????????????????
			
			//????????????/?????? = ?????? ??? ????????????/?????? = ??????
			SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX,SDWeZhanTingYuXing_YunXing);
			SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_A,SDWeFanHuiJieShuSeZhi_KongBai);//??????????????????A = ??????
			SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_B,SDWeFanHuiJieShuSeZhi_JieShu); //??????????????????B = ??????
		}
		else if((TRUE == g_T5L.sampleComplete) && ((GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) + gSystemPara.u32_PercentReceipt)  < 100))
		{
			g_T5L.sampleComplete = FALSE;//???????????????
			
			//????????????
			app_gjf_handle(SDWeFaKaiFaGuan_FaKai);
			//????????????/?????? = ?????? ??? ????????????/?????? = ??????
			SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX,SDWeZhanTingYuXing_ZhanTing);
			SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_A,SDWeFanHuiJieShuSeZhi_KongBai);//A = ??????
			SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_B,SDWeFanHuiJieShuSeZhi_FanHui);//B = ??????
		}
	}
	else
	{
		g_T5L.sampleComplete = FALSE;//???????????????
	}
}


#define SCREEN_SDWE_SAMPLE_COMPLETE_VOICE_PRINTF_OFFSET		(10000)//10S ????????????
#define SCREEN_SDWE_SAMPLE_COMPLETE_VOICE_PRINTF_NUM		(5)//10S ??????5???
#define SCREEN_SDWE_VOICE_PRINTF_DIDID_EN					(FALSE)
//==????????????
void screenSDWe_CaiJiYuYin_Handle(void)
{
	static UINT32 sampleCompletVoiceCyclePrintfTick = 0 ,sampleCompletVoiceCyclePrintfCnt = 0;

	//??????????????????????????????
	if(TRUE == screenSDWe_CaiJiYuYin_Get(SDWEVoicePrintf_QinFangDai))
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_QinFangDai))
		{
			screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_QinFangDai,FALSE);
		}
	}
	//?????????????????????????????????
	else if(TRUE == screenSDWe_CaiJiYuYin_Get(SDWEVoicePrintf_KongDaiZhengChang))
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_KongDaiZhengChang))
		{
			screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_KongDaiZhengChang,FALSE);
		}
	}	
	//?????????????????????????????????
	else if(TRUE == screenSDWe_CaiJiYuYin_Get(SDWEVoicePrintf_KongDaiYiChang))
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_KongDaiYiChang))
		{
			screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_KongDaiYiChang,FALSE);
		}
	}
	//?????????????????????????????????
	else if(TRUE == screenSDWe_CaiJiYuYin_Get(SDWEVoicePrintf_CaiJi_KaiShi))
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_CaiJi_KaiShi))
		{
			screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_KaiShi,FALSE);
		}
	}
	//?????????????????????????????????
	else if(TRUE == screenSDWe_CaiJiYuYin_Get(SDWEVoicePrintf_CaiJi_ZanTing))
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_CaiJi_ZanTing))
		{
			screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_ZanTing,FALSE);
		}
	}
	//?????????????????????????????????
	else if(1 == g_T5L.liusuError)
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_Warn_LiuSu_High))
		{
			screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_Warn_LiuSu_High,FALSE);
			g_T5L.liusuError = 0 ;
		}
	}
	//?????????????????????????????????
	else if(2 == g_T5L.liusuError)
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_Warn_LiuSu_Low))
		{
			screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_Warn_LiuSu_Low,FALSE);
			g_T5L.liusuError = 0 ;
		}
	}
	//?????????????????????????????????????????????
	else if(SDWeCurPage_ShiShiJieMian == g_T5L.curPage)
	{
		//1.???????????????100%???(??????+??????)??????????????????????????? 
		if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) >= 100)
		{
			if(TRUE == screenSDWe_CaiJiYuYin_Get(SDWEVoicePrintf_CaiJi_WanCheng))
			{
				if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_CaiJi_WanCheng))
				{
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_WanCheng,FALSE);
					sampleCompletVoiceCyclePrintfTick = 0 ;//?????????????????????????????????5????????????
				}
			}
			else
			{
				//??????????????????????????????????????????0????????????????????????
				#if(SCREEN_SDWE_SAMPLE_COMPLETE_VOICE_PRINTF_OFFSET > 0)
					if(sampleCompletVoiceCyclePrintfTick++ >= SCREEN_SDWE_SAMPLE_COMPLETE_VOICE_PRINTF_OFFSET)//??????????????????
					{
						if(sampleCompletVoiceCyclePrintfCnt < SCREEN_SDWE_SAMPLE_COMPLETE_VOICE_PRINTF_NUM)
						{
							screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_WanCheng,TRUE);
							sampleCompletVoiceCyclePrintfCnt++;
						}
					}
				#endif
			}
		}
		else
		{
			//???????????????????????????100%????????????
			if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) < (100-gSystemPara.u32_PercentReceipt))
			{
				screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_WanCheng,TRUE);
				sampleCompletVoiceCyclePrintfCnt = 0;//??????????????????????????????
			}
			//
			//2.????????????95%??????????????????????????????
			if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) >= 95)
			{
				if((SCREEN_SDWE_VOICE_PRINTF_DIDID_EN == TRUE)&&(TRUE == screenSDWe_CaiJiYuYin_Get(SDWEVoicePrintf_DiDi_DiDi)))
				{
					if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_DiDi_DiDi))
					{
						screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_DiDi_DiDi,FALSE);
					}
				}
			}
			else
			{
				//???????????????????????????95%????????????
				if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) < (95-gSystemPara.u32_PercentReceipt))
				{
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_DiDi_DiDi,TRUE);
				}
				//
				//3.????????????90%???????????????????????????????????????
				if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) >= 90)
				{
					if((TRUE == screenSDWe_CaiJiYuYin_Get(SDWEVoicePrintf_CaiJi_90))
						&&(TRUE == gSystemPara.u32_90PerVoicePrintf))
					{
						if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_CaiJi_90))
						{
							screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_90,FALSE);
						}
					}
				}
				else
				{
					//???????????????????????????90%????????????
					if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) < (90-gSystemPara.u32_PercentReceipt))
					{
						screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_90,TRUE);
					}
					//
					//4.????????????80%???????????????????????????????????????
					if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) >= 80)
					{
						if((TRUE == screenSDWe_CaiJiYuYin_Get(SDWEVoicePrintf_CaiJi_80))
							&&(TRUE == gSystemPara.u32_80PerVoicePrintf))
						{
							if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_CaiJi_80))
							{
								screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_80,FALSE);
							}	
						}
					}
					else
					{
						//???????????????????????????80%????????????
						if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) < (80-gSystemPara.u32_PercentReceipt))
						{
							screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_80,TRUE);
						}
						//5.????????????50%???????????????????????????????????????
						if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) >= 50)
						{
							if((TRUE == screenSDWe_CaiJiYuYin_Get(SDWEVoicePrintf_CaiJi_50))
								&&(TRUE == gSystemPara.u32_50PerVoicePrintf))
							{
								if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_CaiJi_50))
								{
									screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_50,FALSE);
								}	
							}
						}
						else
						{
							//???????????????????????????50%????????????
							if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) < (50-gSystemPara.u32_PercentReceipt))
							{
								screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_50,TRUE);
							}
						}
					}
				}
			}
		}
	}
}


//??????????????????
void screenSDWe_SampleTimeCaculate(void)
{
	UINT32 rtcTimeSample_s = 0 ;
	//?????????????????????????????????
	if(SDWeCurPage_ShiShiJieMian == g_T5L.curPage)
	{
		//??????/???????????? = ??????????????????????????????
		if(SDWeZhanTingYuXing_ZhanTing == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX))
		{
			g_T5L.rtcTimeSample_ms++;
			if(g_T5L.rtcTimeSample_ms%1000 == 0)
			{
				rtcTimeSample_s = g_T5L.rtcTimeSample_ms/1000;
				//
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_HH_2, ((rtcTimeSample_s/3600)%24) );
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_MM_2, (((rtcTimeSample_s%3600)/60)%60) );
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_SS_2, ((rtcTimeSample_s%3600)%60) );
			}
		}
	}
	else
	{
		g_T5L.rtcTimeSample_ms = 0 ;
	}
}
void screenSDWe_LiuSuErrorCaculate(UINT16 liusu)
{
	//1?????????????????????
	//2?????????/?????? = ??????
	if((SDWeCurPage_ShiShiJieMian == g_T5L.curPage) &&
		(SDWeZhanTingYuXing_ZhanTing == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX)))
	{
		//?????????????????????????????????
		if((0 != gSystemPara.u16_liusu_high_warn) &&
			(liusu >= gSystemPara.u16_liusu_high_warn))
		{
			g_T5L.liusuError = 1;
		}
		//?????????????????????????????????
		if((0 != gSystemPara.u16_liusu_low_warn) &&
			(liusu <= gSystemPara.u16_liusu_low_warn) )
		{
			g_T5L.liusuError = 2;
		}
	}
}


//????????????
void screenSDWe_LiuSuCaculate(UINT32 cycleAvg,UINT32 cycleCur)
{
	static UINT32 tervalVelocity_PreWeight = 0 ;//????????????
	if((SDWeCurPage_ShiShiJieMian == g_T5L.curPage) && (FALSE == g_T5L.sampleComplete))
	{
		if(0 != cycleAvg)//?????? cycleAvg ??????????????????
		{
			if(0 == (g_sys_ms_tick % cycleAvg ))
			{
				if(0 != g_T5L.rtcTimeSample_ms)
				{
					g_T5L.liusuAvg = 0 ;
					g_T5L.liusuAvg = GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_CUN_WEIGHT);
					g_T5L.liusuAvg *= 60000;//?????????ml/min
					g_T5L.liusuAvg /= g_T5L.rtcTimeSample_ms;
					//
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_LIUSU,g_T5L.liusuAvg);
				}
			}
		}
		if(0 != cycleCur)//?????? cycleCur ??????????????????
		{
			if(0 == (g_sys_ms_tick % cycleCur ))
			{
				g_T5L.liusuCur = 0 ;
				g_T5L.liusuCur = GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_CUN_WEIGHT) - tervalVelocity_PreWeight;
				g_T5L.liusuCur *= 60000;//?????????ml/min
				g_T5L.liusuCur /= cycleCur;

				//??????????????????????????????????????? ??????????????????
				screenSDWe_LiuSuErrorCaculate(g_T5L.liusuCur);
				
				//????????????????????????
				tervalVelocity_PreWeight = GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_CUN_WEIGHT);
			}
		}
	}
	else
	{
		g_T5L.liusuCur = 0 ;
		g_T5L.liusuAvg = 0 ;
		tervalVelocity_PreWeight = GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_CUN_WEIGHT);
		SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_LIUSU,g_T5L.liusuAvg);
	}
}



//======?????????????????? ??????????????????
void screenSDWe_TakeDownCheck(UINT16 congJi_LiuSu_CheckPoint)
{
	static UINT16 takeDownOccureDelay = 0 ,takeDownOccureDelayCheck = 1500, takeDownOccureEvent = FALSE ,takeDownOccureMotorCtl = FALSE;
	static UINT16 putBackOccureDelay = 0 ,putBackOccureDelayCheck = 1500, putBackOccureEvent = FALSE,putBackOccureMotorCtl = FALSE;


	(void)takeDownOccureEvent;
	(void)putBackOccureEvent;

	//?????????????????? ??????????????????
	if(SDWeCurPage_ShiShiJieMian == g_T5L.curPage)
	{
		//????????????????????????????????? ??????	  ???????????? ??????????????????????????????
		if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT) < GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_KONGDAI_WEIGHT))
		{
			putBackOccureDelay = 0 ;
			putBackOccureMotorCtl = FALSE;
			//
			if(takeDownOccureDelay++ > takeDownOccureDelayCheck)//1.5???
			{
				takeDownOccureDelay = takeDownOccureDelayCheck;
				g_T5L.takeDownOccure = TRUE;
				g_T5L.putBackOccure = FALSE;
				if(FALSE == takeDownOccureMotorCtl)//???????????????
				{
					takeDownOccureMotorCtl = TRUE;
					app_SetMotorContrlMode(MotorCtrlStatus_EN_StopToMiddle);//????????????
				}
			}
		}
		else if(TRUE == g_T5L.takeDownOccure)//?????????????????????????????? ??????????????? ????????????
		{
			takeDownOccureDelay = 0 ;
			takeDownOccureMotorCtl = FALSE;
			//
			if(putBackOccureDelay++ > putBackOccureDelayCheck)//1.5???
			{
				putBackOccureDelay = putBackOccureDelayCheck;
				g_T5L.takeDownOccure = FALSE;
				g_T5L.putBackOccure = TRUE;
				if(FALSE == putBackOccureMotorCtl)//???????????????
				{
					putBackOccureMotorCtl = TRUE;
					app_SetMotorContrlMode(MotorCtrlStatus_EN_NormalRun_WitMaxPos);//???????????????????????????
				}
			}
		}
	}
	else
	{
		g_T5L.takeDownOccure = FALSE;//?????? ????????????
		takeDownOccureDelay = 0 ;
		takeDownOccureMotorCtl = FALSE;

		g_T5L.putBackOccure = FALSE;//?????? ????????????
		putBackOccureDelay = 0 ;
		putBackOccureMotorCtl = FALSE;	
	}
}



//======???????????? ?????????????????? ??????
void screenSDWe_CycleDataSend(UINT16 cycle)
{
	static UINT16 needSend=FALSE;
	INT16 i=0,*preData=&g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_PRE][0],*curData=&g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][0];
	static float weight = 0 ,weight_pre = 0;
 	static UINT32 cycleSendPreTick = 0 ;
	UINT16 samplePercent = 0 ;	
	(void)weight_pre;

	//????????????????????? ?????????g
	weight = hx711_getWeight(HX711Chanel_1);

	//????????????????????? ?????????ml
	if(gSystemPara.u16_ml_g_bilv !=0 )
	{
		SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT,(weight *1000 / gSystemPara.u16_ml_g_bilv));
	}
	else
	{
		SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT,weight);
	}

	//?????????????????? ??????????????????
	if(SDWeCurPage_ShiShiJieMian == g_T5L.curPage)
	{
		//============================================================================================
		//???????????????
		SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_CUN_WEIGHT,
			(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT) - GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_KONGDAI_WEIGHT)));
		//??????????????????????????????
		#if 0		
			//????????????
			g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] = 
				(100*g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_CUN_WEIGHT]) / g_T5L.curCaiJiDangWeiWeight;
			
			
			//???????????????
			g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] = 
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT];

			if(g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] >= 100)
			{
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] = 100;//?????????????????????
			}else if (g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] < 0)
			{
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] = 0;//?????????????????????
			}
		#else
			//?????????????????????
			//?????????????????????
			if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_CUN_WEIGHT) >= 0)
			{
				samplePercent = (100*GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_CUN_WEIGHT)) / g_T5L.curCaiJiDangWeiWeight;
				//
				if(((samplePercent + gSystemPara.u32_PercentReceipt) < GET_SDWE_PRE_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT)) ||
					(samplePercent >= GET_SDWE_PRE_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT)))
				{
					//?????? ????????? ?????? ???????????????-???????????? ?????????
					//?????? ????????? ?????? ??????????????? ?????????
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT,samplePercent);
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT_TUBIAO,samplePercent);
				}
			}
			else
			{
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_CUN_WEIGHT,0);//???????????????0??? ????????? = 0 
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT,0);//???????????????0??? ???????????? = 0
			}

			//????????????????????????????????????
			if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT_TUBIAO) >= 100)
			{
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT_TUBIAO,100);
			}
			else if(GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT_TUBIAO) < 0)
			{
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT_TUBIAO,0);
			}
		#endif

	}
	//??????????????????????????????
	else
	{
		SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_CUN_WEIGHT,0);
		SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT,0);
		SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT_TUBIAO,0);
	}

	//===-????????????????????????????????? ??? ??????????????????????????????
	if((g_sys_ms_tick - cycleSendPreTick) >= cycle)
	{
		cycleSendPreTick = g_sys_ms_tick;
		//
		for(i=0;i<SDWE_CYCLE_DATA_STATUS_MAX;i++)
		{
			if(preData[i] != curData[i])
			{
				needSend = TRUE;
			}
			preData[i] = curData[i];
		}
	}
	
	//2.if need send
	if(TRUE == needSend)
	{
		//3.wait send ok
		if(TRUE == screenSDWeWriteVarible(SDWE_CYCLE_DATA_START_ADDRESS,(UINT16 *)preData,SDWE_CYCLE_DATA_STATUS_MAX,SDWE_CRC16_EN))
		{
			//4.clear need send flag
			needSend = FALSE;
		}	
	}
}





//???????????????
UINT8 XXX_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	switch(status)
	{
		case 0://????????????
		break;
		case 1://???????????? ?????????
		break;
		case 2://????????????
		break;
		default:
			status = 0 ;
		break;
	}

	//
	if(ret == TRUE)
	{
		status = 0 ;
	}
	//
	return ret;
}










//======????????????????????????????????? checked:20220526
UINT8 screenVoice_Handle(UINT16 eventVlu)
{
	//????????????????????????????????????????????????????????????????????????6.wav?????????100%?????????????????????????????? 
	//A5 5A 07 80 50 5b 00 06 5A 40 
	//???????????????????????????????????????A5 5A 05 80 50 5c 00 06

	UINT8 ret = FALSE,regData[2];
	static UINT8 status = 0;
	
	(void)eventVlu;
	switch(status)
	{
		case 0://????????????
			status = 1;
		break;
		case 1://???????????? ?????????
			regData[0] = 0X5A;
			regData[1] = eventVlu%101;
			if(TRUE == screenSDWeWriteReg(SDWE_VOICE_SET_REG_ADDRESS,&regData[0],SDWE_VOICE_SET_REG_LEN,SDWE_CRC16_EN))
			{
				status = 0 ;
				ret = TRUE;
			}
		break;
		case 2://????????????
		break;
		default:
			status = 0 ;
		break;
	}

	//
	if(ret == TRUE)
	{
		status = 0 ;
	}
	//
	return ret;
}


//======???????????????????????? checked:20220526
UINT8 screenFengMingQi_Handle(UINT16 eventVlu)
{
	//??????????????????????????????2s???????????????A5 5A 03 80 02 C8
	//??????10ms

	UINT8 ret = FALSE,regData[1];
	static UINT8 status = 0;
	
	(void)eventVlu;//??????10ms
	switch(status)
	{
		case 0://????????????
			status = 1;
		break;
		case 1://???????????? ?????????
			regData[0] = eventVlu/10;
			if(TRUE == screenSDWeWriteReg(SDWE_FENGMINGQI_SET_REG_ADDRESS,&regData[0],SDWE_FENGMINGQI_SET_REG_LEN,SDWE_CRC16_EN))
			{
				status = 0 ;
				ret = TRUE;
			}
		break;
		case 2://????????????
		break;
		default:
			status = 0 ;
		break;
	}

	//
	if(ret == TRUE)
	{
		status = 0 ;
	}
	//
	return ret;
}



//======?????????????????????????????????A        	?????????
UINT8 backReturnASetContrl_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	switch(status)
	{
		case 0://????????????
			if(SDWeFanHuiJieShuSeZhi_SheZhi == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_A))
			{
			}
		break;
		case 1://???????????? ?????????
		break;
		case 2://????????????
		break;
		default:
			status = 0 ;
		break;
	}

	//
	if(ret == TRUE)
	{
		status = 0 ;
	}
	//
	return ret;
}


//======?????????????????????????????????B checked:20220526
UINT8 backReturnBSetContrl_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	switch(status)
	{
		case 0://????????????
			status =1;
		break;
		case 1://???????????? ?????????
			//if(SDWeZhanTingYuXing_YunXing == g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_ZT_YX])
			if(SDWeZhanTingYuXing_YunXing == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX))
			{
				status = 2;
			}
			else
			{
				status = 0XFF;
				ret = TRUE;
			}
		
		break;
		case 2://????????????
			if(TRUE == screenSDWe_JumpToPage(SDWeCurPage_KaiShiJieMian))
			{
				g_T5L.curPage = SDWeCurPage_KaiShiJieMian;
				status = 3;
			}
		break;

		case 3://???????????????????????????????????????		
			//???????????????
			app_SetMotorContrlMode(MotorCtrlStatus_EN_StopToMiddle);//??????????????? ??????200/300/400??? ?????? ????????????
			
			//????????????
			app_gjf_handle(SDWeFaKaiFaGuan_FaKai);
		
			status = 4;
		break;
		case 4://???????????? ?????????
			if(TRUE == screenSDWeWriteVarible(SDWE_CYCLE_DATA_START_ADDRESS,(UINT16 *)GET_SDWE_CUR_CYCLE_DATA_PTR(),SDWE_CYCLE_DATA_STATUS_MAX,SDWE_CRC16_EN))
			{
				status = 0;
				ret = TRUE;
			}
		break;
		default:
			status = 0 ;
		break;
	}

	//
	if(ret == TRUE)
	{
		status = 0 ;
	}
	//
	return ret;
}

//======?????????????????????????????? ?????? checked:20220526
UINT8 gjfContrl_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	switch(status)
	{
		case 0://????????????
			if(FALSE == gSystemPara.u16_kaiqi_guanjiafa_gongneng)
			{
				//????????????????????? doing nothing
			}
			else
			{
				if(SDWeFaKaiFaGuan_FaKai == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FK_FG))
				{
					app_gjf_handle(SDWeFaKaiFaGuan_FaGuan);
				}
				else if(SDWeFaKaiFaGuan_FaGuan == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FK_FG))
				{
					app_gjf_handle(SDWeFaKaiFaGuan_FaKai);
				}			
			}
			status = 0;
			ret = TRUE;
		break;
		case 1://???????????? ?????????
		break;
		case 2://????????????
		break;
		default:
			status = 0 ;
		break;
	}

	//
	if(ret == TRUE)
	{
		status = 0 ;
	}
	//
	return ret;
}


//======????????????????????????????????? ?????????RUN checked:??????
UINT8 runContrl_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	switch(status)
	{
		case 0://????????????
			if(SDWeZhanTingYuXing_YunXing == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX))
			{
				if( GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) < 100 )
				{
					//??????cycleData
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX,SDWeZhanTingYuXing_ZhanTing);//????????????/?????? = ??????
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_A,SDWeFanHuiJieShuSeZhi_KongBai);//??????????????????A = ??????
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_B,SDWeFanHuiJieShuSeZhi_KongBai);//??????????????????B = ??????

					//????????????
					app_gjf_handle(SDWeFaKaiFaGuan_FaKai);
					
					//??????????????????????????????????????????
					app_SetMotorContrlMode(MotorCtrlStatus_EN_NormalRun_WitMaxPos);

					//?????????????????????????????????
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_KaiShi,TRUE);
				}
			}
			else if(SDWeZhanTingYuXing_ZhanTing == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX))
			{
				//??????cycleData
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX,SDWeZhanTingYuXing_YunXing);//????????????/?????? = ??????
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_A,SDWeFanHuiJieShuSeZhi_KongBai);//??????????????????A = ??????
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_B,SDWeFanHuiJieShuSeZhi_JieShu);//??????????????????B = ??????

				//????????????
				app_gjf_handle(SDWeFaKaiFaGuan_FaGuan);
	
				//????????????????????????????????????
				app_SetMotorContrlMode(MotorCtrlStatus_EN_StopToMiddle);
				
				//?????????????????????????????????
				screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_ZanTing,TRUE);
			}
			status = 1;
		break;
		case 1://???????????? ?????????
			if(TRUE == screenSDWeWriteVarible(SDWE_CYCLE_DATA_START_ADDRESS,(UINT16 *)GET_SDWE_CUR_CYCLE_DATA_PTR(),SDWE_CYCLE_DATA_STATUS_MAX,SDWE_CRC16_EN))
			{
				status = 0;
				ret = TRUE;
			}
		break;
		case 2://????????????
		break;
		default:
			status = 0 ;
			ret = TRUE;
		break;
	}

	//
	if(ret == TRUE)
	{
		status = 0 ;
	}
	//
	return ret;
}








//======?????????????????????200 300 400?????????????????? checked:20220526
UINT8 enterRealTimeSet_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	UINT8 kongDaiWeight_Matched=FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	//????????????
	if(((TRUE == gSystemPara.u16_kaiqi_guanjiafa_gongneng) && (SDWeFaKaiFaGuan_FaGuan == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FK_FG)))
		|| (TRUE != gSystemPara.u16_kaiqi_guanjiafa_gongneng) )
	{
		switch(status)
		{
			case 0://????????????
				//1?????????????????? ?????????~????????? ??????
				if((200 == g_T5L.enterRealTimeSetContrl[SYS_CTL_REG_STATUS_INDEX]) || (SYS_KEY_VALUED == key_FilterGet(TBES_200_MODLE_CHOICE)))
				{
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_YUSE_WEIGHT,gSystemPara.yusheWeight[SDWeCaiJiDangWei_200]);
					g_T5L.curCaiJiDangWeiWeight = gSystemPara.yusheWeight[SDWeCaiJiDangWei_200];
				
					//??????????????????
					#if(TRUE == KONGDAI_WEIGHT_MATCH_JUDGE)		
						if((GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT) < gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_200][0])
							|| (GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT) > gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_200][1]))
						{
							kongDaiWeight_Matched = FALSE;
						}
						else
						{
							kongDaiWeight_Matched = TRUE;
						}
						//?????? ????????????0??????????????????
						if((0 == gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_200][0])
							&& (0 == gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_200][1]))
						{
							kongDaiWeight_Matched = TRUE;
						}				
					#else
						kongDaiWeight_Matched = TRUE;
					#endif
				}
				else if((300 == g_T5L.enterRealTimeSetContrl[SYS_CTL_REG_STATUS_INDEX]) || (SYS_KEY_VALUED == key_FilterGet(TBES_300_MODLE_CHOICE)))
				{
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_YUSE_WEIGHT,gSystemPara.yusheWeight[SDWeCaiJiDangWei_300]);
					g_T5L.curCaiJiDangWeiWeight = gSystemPara.yusheWeight[SDWeCaiJiDangWei_300];
					//??????????????????
					#if(TRUE == KONGDAI_WEIGHT_MATCH_JUDGE)
						if((GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT) < gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_300][0])
							|| (GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT) > gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_300][1]))
						{
							kongDaiWeight_Matched = FALSE;
						}
						else
						{
							kongDaiWeight_Matched = TRUE;
						}
						//?????? ????????????0??????????????????
						if((0 == gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_300][0])
							&& (0 == gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_300][1]))
						{
							kongDaiWeight_Matched = TRUE;
						}
					#else
						kongDaiWeight_Matched = TRUE;
					#endif					
				}
				else if((400 == g_T5L.enterRealTimeSetContrl[SYS_CTL_REG_STATUS_INDEX]) || (SYS_KEY_VALUED == key_FilterGet(TBES_400_MODLE_CHOICE)))
				{
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_YUSE_WEIGHT,gSystemPara.yusheWeight[SDWeCaiJiDangWei_400]);
					g_T5L.curCaiJiDangWeiWeight = gSystemPara.yusheWeight[SDWeCaiJiDangWei_400];
					//??????????????????
					#if(TRUE == KONGDAI_WEIGHT_MATCH_JUDGE)
						if((GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT) < gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_400][0])
							|| (GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT) > gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_400][1]))
						{
							kongDaiWeight_Matched = FALSE;
						}
						else
						{
							kongDaiWeight_Matched = TRUE;
						}
						//?????? ????????????0??????????????????
						if((0 == gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_400][0])
							&& (0 == gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_400][1]))
						{
							kongDaiWeight_Matched = TRUE;
						}
					#else
						kongDaiWeight_Matched = TRUE;
					#endif					
				}
				else
				{
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_YUSE_WEIGHT,888);
					g_T5L.curCaiJiDangWeiWeight = gSystemPara.yusheWeight[SDWeCaiJiDangWei_200];
				
					//??????????????????
					#if(TRUE == KONGDAI_WEIGHT_MATCH_JUDGE)
						kongDaiWeight_Matched = FALSE;
					#else
						kongDaiWeight_Matched = FALSE;
					#endif
				}

				//??????????????? ????????????
				if(TRUE == kongDaiWeight_Matched)
				{
					//?????????????????????????????????????????????
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_KongDaiZhengChang,TRUE);
					
					//1????????????????????????
					//SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT,);
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_CUN_WEIGHT,0);
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT,0);
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT_TUBIAO,0);
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_LIUSU,0);
					//??????
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_HH_1,g_T5L.rtcTime[RTC_TIME_HH_OFFSET]);
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_MM_1,g_T5L.rtcTime[RTC_TIME_MM_OFFSET]);
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_SS_1,g_T5L.rtcTime[RTC_TIME_SS_OFFSET]);
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_HH_2,0);
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_MM_2,0);
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_SS_2,0);
	
					//2??????????????????????????????????????????????????????????????????
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_KONGDAI_WEIGHT,GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_TOTAL_WEIGHT));

					//3??????????????????A??????/?????? B??????/?????? ??????
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_A,SDWeFanHuiJieShuSeZhi_KongBai);//A = ??????
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_B,SDWeFanHuiJieShuSeZhi_FanHui);//B = ??????

					//4???????????????????????????/?????? ??????
					if(TRUE != gSystemPara.u16_kaiqi_guanjiafa_gongneng)
					{
						SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FK_FG,SDWeFaKaiFaGuan_KongBai);//??????/??? = ??????
					}
					else
					{
						SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FK_FG,SDWeFaKaiFaGuan_FaGuan);//??????/??? = ?????? 
					}

					//5??????????????????/?????? ??????
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX,SDWeZhanTingYuXing_YunXing);//??????
	
					//6??????????????????
					status = 0x10;//????????????
				}
				else//?????????????????????
				{
					//????????????????????????????????????????????????
					status = 0x20;
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_KongDaiYiChang,TRUE);					
				}
			break;
				
			case 0x10://????????????????????????
				if(TRUE == screenSDWe_JumpToPage(SDWeCurPage_ShiShiJieMian))
				{
					g_T5L.curPage = SDWeCurPage_ShiShiJieMian;
					status = 0;
					ret = TRUE;
				}
			break;
		
			default :
				status = 0 ;
				ret = TRUE;
			break;
		}
	}
	else
	{
		ret = TRUE;
	}
	//
	if(ret == TRUE)
	{
		status = 0 ;
	}
	//
	return ret;
}


//======?????????????????????RTC??????
UINT8 rtcSet_Handle(void)
{
	//?????????RTC?????????2015-06-01?????????18:56:00?????????????????????????????? 
	//A5 5A 0A 80 1F    5A     15 06 01 00 18 56 00 VGUS??????????????????????????????????????????????????????????????????

	UINT8 ret = FALSE,i=0;
	UINT8 dataRtc[8]={0X5A,0X22,0X05,0X25,0X03,0X18,0X56,0X00};//YY:MM:DD:WW:HH:MM:SS
	static UINT8 status = 0;
	switch(status)
	{
		case 0://????????????
			status = 1;
		break;
		case 1://???????????? ?????????
			dataRtc[5] = 0xf0&((g_T5L.rtcSetTime[RTC_TIME_HH_OFFSET]/10)<<4);//???
			dataRtc[5] += 0x0f&((g_T5L.rtcSetTime[RTC_TIME_HH_OFFSET]%10)<<0);//???

			dataRtc[6] = 0xf0&((g_T5L.rtcSetTime[RTC_TIME_MM_OFFSET]/10)<<4);//???
			dataRtc[6] += 0x0f&((g_T5L.rtcSetTime[RTC_TIME_MM_OFFSET]%10)<<0);//???

			dataRtc[7] = 0xf0&((g_T5L.rtcSetTime[RTC_TIME_SS_OFFSET]/10)<<4);//???
			dataRtc[7] += 0x0f&((g_T5L.rtcSetTime[RTC_TIME_SS_OFFSET]%10)<<0);//???

			if(TRUE == screenSDWeWriteReg((SDWE_RTC_TIMER_ADDRESS-1),&dataRtc[0],(SDWE_RTC_TIMER_LEN+1),SDWE_CRC16_EN))
			{
				status = 0;
				ret = TRUE;
				//
				for(i=0;i<SDWE_RTC_TIMER_LEN;i++)
				{
					g_T5L.rtcTime[i] = g_T5L.rtcSetTime[i];
				}
			}
		break;
		case 2://????????????
		break;
		default:
			status = 0 ;
		break;
	}

	//
	if(ret == TRUE)
	{
		status = 0 ;
	}
	//
	return ret;

}

//======???????????? ????????????????????? checked:20220526
void sreenT5L_BootAnimation(UINT8 pageStart,UINT8 pageEnd,UINT16 pageDelay)
{
	static UINT16 bootAnimationPage = 0,offset = 0 , trigger = FALSE;
	if(FALSE == g_T5L.bootAnimation)
	{
		if(g_T5L.CurTick%pageDelay == 0)
		{
			bootAnimationPage= pageStart+offset;
			trigger = TRUE;
		}
		if(TRUE == trigger)
		{
			if(bootAnimationPage <= pageEnd) 
			{
				if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
					((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
				{
					if(TRUE == screenSDWeWriteVarible((SDWE_BOOT_ANIMATION_ADDRESS),&bootAnimationPage,1,SDWE_CRC16_EN))
					{
						trigger = FALSE;
						offset++;
					}
				}
			}
			else if(bootAnimationPage > pageEnd) 
			{
				if(((g_T5L.LastSendTick > g_T5L.CurTick)&&((g_T5L.LastSendTick-g_T5L.CurTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER))||
					((g_T5L.LastSendTick < g_T5L.CurTick)&&((g_T5L.CurTick - g_T5L.LastSendTick) >= DMG_MIN_DIFF_OF_TWO_SEND_ORDER)))
				{
					if(TRUE == screenSDWe_JumpToPage(11))
					{
						trigger = FALSE;
						offset = 0 ;
						g_T5L.bootAnimation = TRUE;
					}
				}
			}
		}
	}
}

//======?????????????????????????????? checked:20220526
UINT8 screenSendDataToSDWE(void)
{
	UINT8 ret = FALSE;
	UINT16 i=0,yinyong[17];//0x3600
	static UINT8 status = 0;
	switch(status)
	{
		case 0://????????????
			status = 1;
		break;
		case 1://???????????? ?????????
			i=0;
			yinyong[i++] = gSystemPara.yusheWeight[SDWeCaiJiDangWei_200];
			yinyong[i++] = gSystemPara.yusheWeight[SDWeCaiJiDangWei_300];
			yinyong[i++] = gSystemPara.yusheWeight[SDWeCaiJiDangWei_400];
			yinyong[i++] = motorCtl.tAD;
			yinyong[i++] = motorCtl.motorAge;
			yinyong[i++] = gSystemPara.u16_ml_g_bilv;
			yinyong[i++] = gSystemPara.u16_fmqkqsj;
			yinyong[i++] = gSystemPara.u16_liusu_high_warn;
			yinyong[i++] = gSystemPara.u16_liusu_low_warn;
			yinyong[i++] = gSystemPara.ScreenVoiceSwitch;
			yinyong[i++] = gSystemPara.ScreenLight;
			yinyong[i++] = gSystemPara.errRange;
			yinyong[i++] = gSystemPara.maxWeight;
			yinyong[i++] = gSystemPara.minWeight;
			yinyong[i++] = gSystemPara.zeroRange;
			yinyong[i++] = gSystemPara.u16_kaiqi_guanjiafa_gongneng;
			if(TRUE == screenSDWeWriteVarible((SDWE_YUSHEZHONGLIANG_200_ADDRESS),&yinyong[0],i,SDWE_CRC16_EN))
			{
				status = 2;
			}
		break;
			
		case 2://?????????????????????
			i=0;
			yinyong[i++] = HW_VERSION;
			yinyong[i++] = MCU_VERSION;
			yinyong[i++] = DIWEN_VERSION;
			if(TRUE == screenSDWeWriteVarible((SDWE_VERSION_ADDRERSS),&yinyong[0],i,SDWE_CRC16_EN))
			{
				status = 3;
			}
		break;
			
		case 3://????????????????????????????????????????????? ?????????
			i=0;
			yinyong[i++] = gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_200][0];
			yinyong[i++] = gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_200][1];
			yinyong[i++] = gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_300][0];
			yinyong[i++] = gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_300][1];
			yinyong[i++] = gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_400][0];
			yinyong[i++] = gSystemPara.u32_LevelMinAndMax[SDWeCaiJiDangWei_400][1];
			if(TRUE == screenSDWeWriteVarible((SDWE_LEVEL_200_MIN_WEIGHT),&yinyong[0],i,SDWE_CRC16_EN))
			{
				status = 4;
			}
		break;
			
		case 4://????????????????????????
			i=0;
			yinyong[i++] = gSystemPara.u32_50PerVoicePrintf;
			yinyong[i++] = gSystemPara.u32_80PerVoicePrintf;
			yinyong[i++] = gSystemPara.u32_90PerVoicePrintf;
			if(TRUE == screenSDWeWriteVarible((SDWE_50_VOICE_PRINTF_CHOICE),&yinyong[0],i,SDWE_CRC16_EN))
			{
				status = 5;
				ret = TRUE;
			}
		break;
		
		default:
			status = 0 ;
			ret = TRUE;
		break;
	}

	//
	if(ret == TRUE)
	{
		status = 0 ;
	}
	//
	return ret;
}

//======prepare TX data checked:20220526
void screenSDWe_TxFunction(void)
{
	T5LType *pScreen = &g_T5L;
	//
	static UINT32 maxWaitTime;
	UINT16 localVlu[0x10];
	UINT8 i=0;
	static UINT8 s_u16_TriggerStopRock = FALSE;
	switch(pScreen->sdweCtlStep)
	{
		//????????????
		case SDWeCtlStep_bootAnimation:
			if(FALSE == pScreen->bootAnimation)
			{
				sreenT5L_BootAnimation(1,15,100);
			}
			else
			{
				pScreen->sdweCtlStep = SDWeCtlStep_GetRTC;
			}
		break;
		
		//??????RTC??????
		case SDWeCtlStep_GetRTC:
			if(FALSE == pScreen->getRTC)
			{
				if(TRUE == screenSDWeReadReg(SDWE_RTC_TIMER_ADDRESS,SDWE_RTC_TIMER_LEN,SDWE_CRC16_EN))
				{
					maxWaitTime = 0 ;
					pScreen->sdweCtlStep = SDWeCtlStep_RecvRTC;
				}
			}
		break;
		//??????RTC??????
		case SDWeCtlStep_RecvRTC:
			if(TRUE == pScreen->getRTC)
			{
				maxWaitTime = 0 ;
				pScreen->sdweCtlStep = SDWeCtlStep_SendDataToSDWE;
			}
			else if(maxWaitTime < 2000)//2s ??????
			{
				maxWaitTime++;
			}
			else
			{
				maxWaitTime = 0 ;
				pScreen->sdweCtlStep = SDWeCtlStep_SendDataToSDWE;
			}
		break;

		//?????????????????????SDWe
		case SDWeCtlStep_SendDataToSDWE:
			if(TRUE == screenSendDataToSDWE())
			{
				pScreen->sdweCtlStep = SDWeCtlStep_CycleHandle;

			}
		break;
		
		//????????????	
		case SDWeCtlStep_CycleHandle:
			//??????????????? ???????????? ???????????????????????????????????? -> ????????????/?????? ????????????
			if((TRUE == g_T5L.u16_TriggerStopRock) && (SDWeCurPage_ShiShiJieMian == g_T5L.curPage) && (TRUE == g_T5L.sampleComplete))
			{
				if(FALSE == s_u16_TriggerStopRock)
				{
					s_u16_TriggerStopRock = TRUE;
					app_SetMotorContrlMode(MotorCtrlStatus_EN_StopToMiddle);
				}
				else
				{
					s_u16_TriggerStopRock = FALSE;
					app_SetMotorContrlMode(MotorCtrlStatus_EN_NormalRun_WitMaxPos);
				}
				//
				g_T5L.u16_TriggerStopRock = FALSE;
			}
			//==50% 80% 90%??????
			else if(SYS_CTL_EVENT_VALID == pScreen->u16_508090SetPercentEvent)
			{
				localVlu[0] = gSystemPara.u32_50PerVoicePrintf;
				localVlu[1] = gSystemPara.u32_80PerVoicePrintf;
				localVlu[2] = gSystemPara.u32_90PerVoicePrintf;
				if(TRUE == screenSDWeWriteVarible(SDWE_50_VOICE_PRINTF_CHOICE,(UINT16 *)(&localVlu[0]),SDWE_508090_VOICE_PRINTF_CHOICE_NUM,SDWE_CRC16_EN))
				{
					pScreen->u16_508090SetPercentEvent = SYS_CTL_EVENT_INVALID;
				}
			}
			//A?????????????????????{?????? ?????? ??????}
			else if(SYS_CTL_EVENT_VALID == pScreen->backReturnASetContrl[SYS_CTL_REG_EVENT_INDEX])
			{
				//if(TRUE == backReturnASetContrl_Handle(pScreen->backReturnASetContrl[SYS_CTL_REG_STATUS_INDEX]))
				{
					pScreen->backReturnASetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_INVALID;
				}
			}
			//B?????????????????????{?????? ?????? ??????}
			else if(SYS_CTL_EVENT_VALID == pScreen->backReturnBSetContrl[SYS_CTL_REG_EVENT_INDEX])
			{
				if(TRUE == backReturnBSetContrl_Handle(pScreen->backReturnBSetContrl[SYS_CTL_REG_STATUS_INDEX]))
				{
					pScreen->backReturnBSetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_INVALID;
					//
					for(i=SDWEVoicePrintf_CaiJi_KaiShi;i<SDWEVoicePrintf_MAX;i++)
					{
						screenSDWe_CaiJiYuYin_Set((enumSDWEVoicePrintfType)(SDWEVoicePrintf_CaiJi_KaiShi),FALSE);
					}
				}
			}
			else if(SYS_CTL_EVENT_VALID == pScreen->gjfContrl[SYS_CTL_REG_EVENT_INDEX])
			{
				if(TRUE == gjfContrl_Handle(pScreen->gjfContrl[SYS_CTL_REG_STATUS_INDEX]))
				{
					pScreen->gjfContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_INVALID;
				}
			}
			else if(((SYS_CTL_EVENT_VALID == pScreen->runContrl[SYS_CTL_REG_EVENT_INDEX]) ||	(SYS_KEY_VALUED == getKeyValuedEvent_OfTBES(TBES_START_MODLE_CHOICE)))
				&& (SDWeCurPage_ShiShiJieMian == pScreen->curPage) )
			{
				if(TRUE == runContrl_Handle(pScreen->runContrl[SYS_CTL_REG_STATUS_INDEX]))
				{
					pScreen->runContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_INVALID;				
					key_EventClear(TBES_START_MODLE_CHOICE);
				}
			}
			else if(((SYS_CTL_EVENT_VALID == pScreen->enterRealTimeSetContrl[SYS_CTL_REG_EVENT_INDEX]) 	||	(SYS_KEY_VALUED == key_FilterGet(TBES_200_MODLE_CHOICE))	||	(SYS_KEY_VALUED == key_FilterGet(TBES_300_MODLE_CHOICE)) ||	(SYS_KEY_VALUED == key_FilterGet(TBES_400_MODLE_CHOICE)))
				&& (SDWeCurPage_KaiShiJieMian == pScreen->curPage) )
			{
				//==?????????????????????????????????????????????
				if(pScreen->cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_TOTAL_WEIGHT] > gSystemPara.zeroRange)
				{
					if(TRUE == enterRealTimeSet_Handle(pScreen->enterRealTimeSetContrl[SYS_CTL_REG_STATUS_INDEX]))
					{
						pScreen->enterRealTimeSetContrl[SYS_CTL_REG_STATUS_INDEX] = 0XFFFF;
						pScreen->enterRealTimeSetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_INVALID;
					}
				}
				else//==??????????????????????????????????????????
				{
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_QinFangDai,TRUE);
					pScreen->enterRealTimeSetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_INVALID;
				}
			}
			else if((TRUE == pScreen->sdweRemoveWeightTriger))
			{
				hx711_setAllRemoveWeight();
				pScreen->sdweRemoveWeightTriger = FALSE;
			}
			else if((TRUE == pScreen->sdweJumpToSysParaPage))//?????????????????????????????????
			{
				if(TRUE == screenSDWe_JumpToPage(SDWeCurPage_CanShuSetJieMian))
				{
					g_T5L.curPage = SDWeCurPage_CanShuSetJieMian;
					pScreen->sdweJumpToSysParaPage = FALSE;
				}
			}
			else if(TRUE == pScreen->sdweJumpToCalitrationPage)
			{
				if(0 != jumpToCalibrationPage())
				{
					pScreen->sdweJumpToCalitrationPage = FALSE;
				}
			}
			else if(TRUE == pScreen->rtcSet)
			{
				if(0 != rtcSet_Handle())
				{
					pScreen->rtcSet = FALSE;
				}
			}
			//==C1 event arrive:At Calibration Page , chanel changed trigerd
			else if(TRUE == pScreen->sdweChanelChanged)
			{
				if(0 != chanelChangedTrigerDeal())
				{
					 pScreen->sdweChanelChanged = FALSE;
				}
			}
			//==C2 event arrive:At Calibration Page , calibration reset trigerd 
			else if(TRUE == pScreen->sdweResetTriger)
			{
				if(0 != resetCalibrationTrigerDeal())
				{
					pScreen->sdweResetTriger = FALSE;
				}
			}
			//==C3 event arrive:At Calibration Page , point trigerd
			else if(TRUE == pScreen->sdwePointTriger)
			{
				if(0 != pointTrigerDeal())
				{
					pScreen->sdwePointTriger = FALSE;
				}
			}
			else//????????????
			{
				screenSDWe_CycleDataSend(500);//???500ms ???????????????????????????
			}
		break;
		default:
			pScreen->sdweCtlStep = SDWeCtlStep_bootAnimation;
		break;
	}
	return;
}



//======SDWE UART data deal checked:20220526
void screenSDWe_RxFunction(void)
{
	//???????????????????????????????????????0x03???0x04?????? 
	//?????????0xA5 0x5A     0x03       0x81   0x03     0x02 
	//		  header   order_len  order	 address  len

	//?????????0xA5 0x5A     0x05       0x81   0x03     0x02    0x00    0x01
	//		  header   order_len  order	 address  len     data[3] data[4]


	//????????????????????????????????????0x0003???0x0004?????? 
	//?????????0xA5 0x5A      0x04     0x83  0x00 0x03   0x02 
	//        header  order_len  order  address    len

	//?????????0xA5 0x5A      0x08     0x83  0x00 0x03   0x02    0x00 0x01   0xff 0xff 
	//        header  order_len  order  address    len     data[3]     data[4]

	UINT8 needStore = FALSE ;
	UINT16 regLen = 0 , reg_i = 0 , regAdd = 0 , regData = 0;
	UINT16 varLen = 0 , var_i = 0 , varAdd = 0 , varData = 0;
	UINT16 crc16Rcv=0,crc16;
	T5LType *pScreen = &g_T5L;
	
	if(TRUE == pScreen->RxFinishFlag)
	{
		//A5 5A
		if((T5L_RX_FUN_HEAD1 == pScreen->rxData[cmdPosHead1]) && (T5L_RX_FUN_HEAD2 == pScreen->rxData[cmdPosHead2]))
		{
			//2 head + 1 len + last 3(cmd:1 add:1-2 data:1-n) data 
			if(( pScreen->RxLength >= 6 ) && ((pScreen->RxLength-3) == pScreen->rxData[cmdPosDataLen]) )
			{
				//caculate CRC
				#if(1 == SDWE_CRC16_EN)
				crc16Rcv =(pScreen->rxData[pScreen->RxLength-1]<<8)&0xff00;
				crc16Rcv +=pScreen->rxData[pScreen->RxLength-2];
				crc16=cal_crc16(&pScreen->rxData[cmdPosCommand],(pScreen->RxLength- cmdPosCommand -2));
				if(crc16Rcv == crc16)
				{
				#endif
				
					switch(pScreen->rxData[cmdPosCommand])
					{
						case cmdWriteSWDERegister:
						break;
						case cmdReadSWDERegister://each register is 8 bits
							//send:A5 5A 03 cmdReadSWDERegister XX YY (XX:address YY:len)
							//rec :A5 5A (03+YY) cmdReadSWDERegister XX YY DD^YY (XX:address YY:len DD:data)
							//if((pScreen->RxLength-3) == pScreen->rxData[cmdPosDataLen])//remove 2 head + 1 data len
							{
								regLen = pScreen->rxData[cmdPosReadRegAskLen];
								if(((pScreen->rxData[cmdPosDataLen]-3-2)/1) == regLen)
								{
									regAdd = 0 ;
									regAdd = pScreen->rxData[cmdPosRegAddress];
									//mult varible deal
									for(reg_i = 0 ; reg_i < regLen ;reg_i++)
									{
										regData = 0 ;
										regData = pScreen->rxData[cmdPosRegData+reg_i];
										//deal
										needStore |= sdweAskRegData((regAdd+reg_i),regData);
									}
								}
							}
						break;
						case cmdWriteSWDEVariable:
						break;
						case cmdReadSWDEVariable://each variable is 16 bits
							//send:A5 5A 04 cmdReadSWDEVariable XX XX YY (XX XX:address YY:len)
							//rec :A5 5A (04+2*YY) cmdReadSWDEVariable XX XX YY DD DD^YY (XX XX:address YY:len DD DD:data)
							//if((pScreen->RxLength-3) == pScreen->rxData[cmdPosDataLen])//remove 2 head + 1 data len
							{
								varLen = pScreen->rxData[cmdPosReadVarAskLen];
								if(((pScreen->rxData[cmdPosDataLen]-4-2)/2) == varLen)
								{
									varAdd = 0 ;
									varAdd = pScreen->rxData[cmdPosVarAddress1];					
									varAdd <<= 8 ;
									varAdd &= 0xff00;
									varAdd += pScreen->rxData[cmdPosVarAddress2];
									//mult varible deal
									for(var_i = 0 ; var_i < varLen ;var_i++)
									{
										varData = 0 ;
										varData = pScreen->rxData[cmdPosVarData1+2*var_i+0];					
										varData <<= 8 ;
										varData &= 0xff00;
										varData += pScreen->rxData[cmdPosVarData1+2*var_i+1];
										//deal
										needStore |= sdweAskVaribleData((varAdd+var_i),varData);
									}
								}
							}						
						break;
						default:
						break;
					}
				#if(1 == SDWE_CRC16_EN)
				}
				#endif
			}

			//store in flash
			if(pScreen->CurTick > 5000)
			{
				if(0 != (DMG_TRIGER_SAVE_SECOTOR_1&needStore))
				{
					storeSysDataToFlash();
					storeSysDataToFlash_3030();//Flash Erase Times Manager
				}
				//store in flash
				else if(0 != (DMG_TRIGER_SAVE_SECOTOR_2&needStore))
				{
					storeSysDataToFlash_3030();
				}
			}
		}
		//
		pScreen->RxFinishFlag = FALSE;
	}
}














































/*******************************************************************************
 * Functions : SDWe??? ????????? ??????
 ******************************************************************************/
void screenSDWe_SubFunctionTest(void)
{
	static UINT8 testVoiceEn=FALSE,testVoiceVlu=0; 
	static UINT16 testFengMingQiEn=FALSE,testFengMingQiVlu=0; 
	static UINT16 testVoicePrintfEn=FALSE,testVoicePrintfVlu=0; 
	//
	if(TRUE == testVoiceEn)
	{
		if(TRUE == screenVoice_Handle(testVoiceVlu))
		{
			testVoiceEn = FALSE;
		}
	}
	else if(TRUE == testFengMingQiEn)
	{
		if(TRUE == screenFengMingQi_Handle(testFengMingQiVlu))
		{
			testFengMingQiEn = FALSE;
		}
	}
	else if(TRUE == testVoicePrintfEn)
	{
		if(TRUE == screenVoicePrintf_Handle(testVoicePrintfVlu))
		{
			testVoicePrintfEn = FALSE;
		}
	}
}


/*******************************************************************************
 * Functions : SDWe??? ?????????
 ******************************************************************************/
void sreenSDWe_MainFunction(void)
{
	T5LType *pScreen = &g_T5L;
	//
	pScreen->CurTick++;
	if(pScreen->CurTick > SYS_POWER_REDAY_TIME)//this time not overview !!!
	{
		//SDWe ??????????????????
		screenSDWe_SubFunctionTest();
		
		//deal rx data from SDWe
		screenSDWe_RxFunction();
		
		//prepare data and send to SDWe
		screenSDWe_TxFunction();
		
		//???????????? 1???????????????
		screenSDWe_LiuSuCaculate(1000,3000);
		
		//??????????????????
		screenSDWe_SampleTimeCaculate();
		
		//RTC?????? ??????
		screenSDWe_RTCTimeCaculate();
		
		//==????????????
		screenSDWe_CaiJiYuYin_Handle();

		//==??????????????????
		screenSDWe_CaiJiWanCheng_Handle();

		//==???????????? ?????? ?????? ??????
		screenSDWe_TakeDownCheck(2000);//2000ml/min = 
	}
}
