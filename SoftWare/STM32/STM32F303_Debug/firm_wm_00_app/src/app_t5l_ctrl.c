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
	g_T5L.readSdweInit = FALSE;
	g_T5L.pUartDevice = &g_UartDevice[UART_ZXP];
	g_T5L.version = 0;//SDWE version
	g_T5L.allowCompare = FALSE;
	//
	g_T5L.pUartDevice->pRxLength = &g_T5L.RxLength;
	g_T5L.pUartDevice->pRxFinishFlag = &g_T5L.RxFinishFlag;
	g_T5L.pUartDevice->pTxBuffer = &g_T5L.rxData[0];
	g_T5L.pUartDevice->pRxBuffer = &g_T5L.rxData[0];
	//
	//
	g_T5L.RxLength = 0;					/**< 接收字节数 */
	g_T5L.RxFinishFlag = FALSE;			/**< 接收完成标志 */
	//
	g_T5L.SetAdd = 0XFFFF;/**< 地址 */
	g_T5L.DataLen = 0;/**< 数据长度 */
	g_T5L.SetData = 0;/**< 数据 */

	g_T5L.ColorClen=FALSE;/**< 通道切换SDWE颜色清除 */
	g_T5L.CalibrateChanel=88;/**< 通道 */
	g_T5L.CalibratePoint=0;/**< 校准点 */


	g_T5L.ResetTrigerValid = FALSE;
	//
	for(i=0;i<CHANEL_POINT_NUM;i++)
	{
		g_T5L.CalibratePointArry[i] = defaultChanelSamplePoint[i];/**< 校准点数组 */
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
	//变量存储器0x0003单元写入0x00 、0x01
	//发送：0xA5 0x5A 0x05 0x82 0x00 0x03 0x00 0x01
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
				hal_delay_ms(1);
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
	//连续读取变量存储器0x0003和0x0004单元
	//发送：0xA5 0x5A 0x04 0x83 0x00 0x03 0x02
	//返回：0xA5 0x5A 0x08 0x83 0x00 0x03 0x02 0x00 0x01 0xff 0xff
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
				hal_delay_ms(1);
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
	//寄存器0x03和0x04单元连续写入0x00 、0x01
	//0xA5 0x5A 0x04 0x80 0x03 0x00 0x01
	//若有CRC   0x06
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
	//连续读取寄存器寄存器0x03和0x04单元
	//发送：0xA5 0x5A 0x03 0x81 0x03 0x02 
	//返回：0xA5 0x5A 0x05 0x81 0x03 0x02 0x00 0x01
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
				hal_delay_ms(1);
			}
		}
		//
		ret = TRUE;
	}
	return ret;
}

//======页面跳转 checked:20220526
UINT8 screenSDWe_JumpToPage(UINT16 pageIndex)
{
	//例如，切换到2号界面的指令A5 5A 04 80 03 00 02
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
		case SDWE_RTC_TIMER_ADDRESS:
			pSdwe->rtcTime[0] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
		break;
		case SDWE_RTC_TIMER_ADDRESS+1:
			pSdwe->rtcTime[1] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
		break;
		case SDWE_RTC_TIMER_ADDRESS+2:
			pSdwe->rtcTime[2] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
		break;
		case SDWE_RTC_TIMER_ADDRESS+3:
			pSdwe->rtcTime[3] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
		break;
		case SDWE_RTC_TIMER_ADDRESS+4://时
			pSdwe->cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_1] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
			pSdwe->rtcTime[4] = pSdwe->cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_1];
		break;
		case SDWE_RTC_TIMER_ADDRESS+5://分
			pSdwe->cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_1] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
			pSdwe->rtcTime[5] = pSdwe->cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_1];
		break;
		case SDWE_RTC_TIMER_ADDRESS+6://秒
			pSdwe->cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_1] = 10*((pSdwe->SetData>>4)&0xf) + (pSdwe->SetData&0xf);
			//
			pSdwe->rtcTime_ms = 0 ;
			pSdwe->rtcTime_ms += pSdwe->cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_1]*3600;
			pSdwe->rtcTime_ms += pSdwe->cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_1]*60;
			pSdwe->rtcTime_ms += pSdwe->cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_1];
			pSdwe->rtcTime_ms *=1000;//ms
			pSdwe->getRTC = TRUE;
			pSdwe->rtcTime[6] = pSdwe->cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_1];
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
	//A5 5A 05 82 00 03 00 01:向0x0003地址写入数据0x0001
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
	//A5 5A 04 83 00 A1 01:读系统变量0x00A1长度为1
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
	//A5 5A 05 82 00 03 00 01:向0x0003地址写入数据0x0001
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
	//A5 5A 04 83 00 A1 01:读系统变量0x00A1长度为1
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
	//A5 5A 05 82 00 03 00 01:向0x0003地址写入数据0x0001
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
		if((SDWE_BACKANDSET_A_CONTRL_ADDRESS == pSdwe->SetAdd) && (SDWE_BACKANDSET_A_CONTRL_ADDRESS == pSdwe->SetData))
		{
			g_T5L.backReturnASetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_VALID;
		}
		else if((SDWE_BACKANDSET_B_CONTRL_ADDRESS == pSdwe->SetAdd) && (SDWE_BACKANDSET_B_CONTRL_ADDRESS == pSdwe->SetData))
		{
			g_T5L.backReturnBSetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_VALID;
		}
		else if((SDWE_GJF_CONTRL_ADDRESS == pSdwe->SetAdd) && (SDWE_GJF_CONTRL_ADDRESS == pSdwe->SetData))
		{
			g_T5L.gjfContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_VALID;
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
			g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_1] = pSdwe->SetData%24;

			g_T5L.rtcTime_s = 0 ;
			g_T5L.rtcTime_s += g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_1]*3600;
			g_T5L.rtcTime_s += g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_1]*60;
			g_T5L.rtcTime_s += g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_1];
			g_T5L.rtcTime_ms = g_T5L.rtcTime_s*1000;
			g_T5L.rtcSet = TRUE;
		}else if(SDWE_RTC_TIME_SET_MM_ADDRESS == pSdwe->SetAdd)
		{
			g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_1] = pSdwe->SetData%60;

			g_T5L.rtcTime_s = 0 ;
			g_T5L.rtcTime_s += g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_1]*3600;
			g_T5L.rtcTime_s += g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_1]*60;
			g_T5L.rtcTime_s += g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_1];
			g_T5L.rtcTime_ms = g_T5L.rtcTime_s*1000;
			g_T5L.rtcSet = TRUE;
		}else if(SDWE_RTC_TIME_SET_SS_ADDRESS == pSdwe->SetAdd)
		{
			g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_1] = pSdwe->SetData%60;

			g_T5L.rtcTime_s = 0 ;
			g_T5L.rtcTime_s += g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_1]*3600;
			g_T5L.rtcTime_s += g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_1]*60;
			g_T5L.rtcTime_s += g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_1];
			g_T5L.rtcTime_ms = g_T5L.rtcTime_s*1000;
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
			gSystemPara.minWeight = pSdwe->SetData;/**< 最小量程 */
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_MAX_RANGE_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.maxWeight = pSdwe->SetData;/**< 最大量程 */
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_ERR_RANGE_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.errRange = pSdwe->SetData;/**< 误差范围 */
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_ZERO_RANGE_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.zeroRange = pSdwe->SetData;/**< 零点范围 */
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_SCREEN_LIGHT_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.ScreenLight = pSdwe->SetData;/**< 屏幕背光亮度 */
			g_T5L.sdweFreshScreenLight = TRUE;
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_VOICE_NUM_TOUCH_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.VoiceNumTouch = pSdwe->SetData;/**< 语音大小 触控*/
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(DMG_FUNC_SET_VOICE_NUM_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.VoiceNum = pSdwe->SetData;/**< 语音大小 */
			needStore |= DMG_TRIGER_SAVE_SECOTOR_2 ;
		}
		else if(SDWE_GUANJIAFAGONGNENGKAIQI_ADDRESS== pSdwe->SetAdd)
		{
			gSystemPara.u16_kaiqi_guanjiafa_gongneng = pSdwe->SetData;/**< 开启管夹阀功能 */
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
			pSdwe->ResetTrigerValid = FALSE;/*重新校准取消*/
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
		}//==(update:20210428):reset calibration
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
				for(i=0;i<HX711_CHANEL_NUM;i++)//8通道
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






//RTC时长计算
void screenSDWe_RTCTimeCaculate(void)
{
	if(TRUE == g_T5L.getRTC)
	{
		g_T5L.rtcTime_ms++;
		g_T5L.rtcTime_s = g_T5L.rtcTime_ms/1000;
	}
}

//==使能播报语音
void screenSDWe_CaiJiYuYin_Set(enumSDWEVoicePrintfType voicePrintf,UINT8 setVlu)
{
	g_T5L.cjyyts[voicePrintf - SDWEVoicePrintf_CaiJi_KaiShi] = setVlu;
}


//事件处理：语音播报
UINT8 screenVoicePrintf_Handle(UINT16 eventVlu)
{
	//举例，一段语音（比如“欢迎光临武汉中显”）保存为6.wav，要以100%音量播放，串口下发： 
	//A5 5A 07 80 50 5b 00 06 5A 40 
	//要停止语音播放，串口下发：A5 5A 05 80 50 5c 00 06 

	UINT8 ret = FALSE;
	static UINT8 regData[SDWE_VOICE_PRINTF_SET_REG_LEN]={0X5B,00,00,0X5A,0X40};
	static UINT8 status = 0;
	
	(void)eventVlu;
	switch(status)
	{
		case 0://条件判断
			status = 1;
		break;
		case 1://数据准备 及发送
			regData[0] = 0X5B;
			eventVlu = eventVlu%4096;
			regData[1] = (eventVlu>>8)&0XFF;
			regData[2] = (eventVlu>>0)&0XFF;
			regData[3] = 0X5A;//设置音量
			regData[4] = gSystemPara.ScreenVoiceSwitch;//    0X30;//设置音量
			
			if(TRUE == screenSDWeWriteReg(SDWE_VOICE_PRINTF_SET_REG_ADDRESS,&regData[0],SDWE_VOICE_PRINTF_SET_REG_LEN,SDWE_CRC16_EN))
			{
				status = 0 ;
				ret = TRUE;
			}
		break;
		case 2://页面切换
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
	static UINT16 caiJiWanChengCnt = 0;
	static UINT16 caiJiWeiWanChengCnt = 0;

	(void)caiJiWeiWanChengCnt;

	//实时采集页面 计算百分比等
	if(SDWeCurPage_ShiShiJieMian == g_T5L.curPage)
	{
		if(g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] >= 100)
		{
			caiJiWeiWanChengCnt = 0 ;
			if(caiJiWanChengCnt++ >= SDWE_CAIJI_WANCHENG_DELAY)
			{
				caiJiWanChengCnt = SDWE_CAIJI_WANCHENG_DELAY;
				//
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_ZT_YX] = SDWeZhanTingYuXing_YunXing;
				app_gjf_handle(SDWeFaKaiFaGuan_FaGuan);
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FJS_A] = SDWeFanHuiJieShuSeZhi_KongBai;//设置返回结束A = 空白
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FJS_B] = SDWeFanHuiJieShuSeZhi_JieShu;//设置返回结束B = 结束
			}
		}
		else 
		{
			caiJiWanChengCnt = 0 ;
			/*
			if(caiJiWeiWanChengCnt++ >= SDWE_CAIJI_WEIWANCHENG_DELAY)
			{
				caiJiWeiWanChengCnt = SDWE_CAIJI_WEIWANCHENG_DELAY;
				//
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_ZT_YX] = SDWeZhanTingYuXing_ZhanTing;
				app_gjf_handle(SDWeFaKaiFaGuan_FaGuan);
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FJS_A] = SDWeFanHuiJieShuSeZhi_KongBai;//设置返回结束A = 空白
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FJS_B] = SDWeFanHuiJieShuSeZhi_KongBai;//设置返回结束B = 空白

			}
			*/
		}
	}
	else
	{
		caiJiWanChengCnt = 0 ;
		caiJiWeiWanChengCnt = 0 ;
	}
}



//==语音提示
void screenSDWe_CaiJiYuYin_Handle(void)
{
	static UINT32 didiTick = 0 ;
	static UINT32 caiYangWanChengCnt = 0 ;
	
	//采样完成 = 100% 后 间隔5秒 提示“采集完成”
	if(g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] >= 100)
	{
		if(FALSE == g_T5L.cjyyts[SDWEVoicePrintf_CaiJi_WanCheng - SDWEVoicePrintf_CaiJi_KaiShi])
		{
			didiTick++;
			if(didiTick >= 2000)//2000 + 3000
			{
				didiTick = 0 ;
				g_T5L.cjyyts[SDWEVoicePrintf_CaiJi_WanCheng - SDWEVoicePrintf_CaiJi_KaiShi] = TRUE;
			}
		}
	}
	else
	{
		didiTick = 0;
		caiYangWanChengCnt = 0 ;
	}

	//==具体语音播报
	if((TRUE == g_T5L.cjyyts[SDWEVoicePrintf_CaiJi_KaiShi - SDWEVoicePrintf_CaiJi_KaiShi]) 
		&& (g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] < 10))
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_CaiJi_KaiShi))
		{
			g_T5L.cjyyts[SDWEVoicePrintf_CaiJi_KaiShi - SDWEVoicePrintf_CaiJi_KaiShi] = FALSE;
		}	
	}
	else if((TRUE == g_T5L.cjyyts[SDWEVoicePrintf_CaiJi_80 - SDWEVoicePrintf_CaiJi_KaiShi]) 
		&& (g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] >= 80))
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_CaiJi_80))
		{
			g_T5L.cjyyts[SDWEVoicePrintf_CaiJi_80 - SDWEVoicePrintf_CaiJi_KaiShi] = FALSE;
		}	
	}
	else if((TRUE == g_T5L.cjyyts[SDWEVoicePrintf_CaiJi_90 - SDWEVoicePrintf_CaiJi_KaiShi]) 
			&& (g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] >= 90))
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_CaiJi_90))
		{
			g_T5L.cjyyts[SDWEVoicePrintf_CaiJi_90 - SDWEVoicePrintf_CaiJi_KaiShi] = FALSE;
		}	
	}
	else if((TRUE == g_T5L.cjyyts[SDWEVoicePrintf_DiDi_DiDi - SDWEVoicePrintf_CaiJi_KaiShi]) 
			&& (g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] >= 95))
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_DiDi_DiDi))
		{
			g_T5L.cjyyts[SDWEVoicePrintf_DiDi_DiDi - SDWEVoicePrintf_CaiJi_KaiShi] = FALSE;
		}	
	}
	else if((TRUE == g_T5L.cjyyts[SDWEVoicePrintf_CaiJi_WanCheng - SDWEVoicePrintf_CaiJi_KaiShi]) 
			&& (g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] >= 100))
	{
		caiYangWanChengCnt++;
		if((caiYangWanChengCnt >= 3000) && (TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_CaiJi_WanCheng)))
		{
			caiYangWanChengCnt = 0 ;
			g_T5L.cjyyts[SDWEVoicePrintf_CaiJi_WanCheng - SDWEVoicePrintf_CaiJi_KaiShi] = FALSE;
		}	
	}
	else if(TRUE == g_T5L.cjyyts[SDWEVoicePrintf_QinFangDai - SDWEVoicePrintf_CaiJi_KaiShi])
	{
		if(TRUE == screenVoicePrintf_Handle(SDWEVoicePrintf_QinFangDai))
		{
			g_T5L.cjyyts[SDWEVoicePrintf_QinFangDai - SDWEVoicePrintf_CaiJi_KaiShi] = FALSE;
		}	

	}
}






//采样时长计算
void screenSDWe_SampleTimeCaculate(void)
{
	static UINT32 ticksPre,tickTotal=0,tickTotalCaculate=0,tickTotal_s=0;
	if((SDWeCurPage_ShiShiJieMian == g_T5L.curPage))
	{
		if(SDWeZhanTingYuXing_ZhanTing == g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_ZT_YX])
		{
			tickTotal += (g_sys_ms_tick - ticksPre);
			ticksPre = g_sys_ms_tick;
			if((tickTotal - tickTotalCaculate) >= 1000)//trriger use tickTotal caculate times
			{
				tickTotalCaculate = tickTotal;
				tickTotal_s = tickTotal/1000;
				
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_2] = (tickTotal_s)%60 ;
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_2] = (tickTotal_s/60)%60;
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_2] = (tickTotal_s/3600)%60;
			}
		}
		else
		{
			ticksPre = g_sys_ms_tick;
			//tickTotal = 0 ;
			//tickTotalCaculate = 0 ;
			//tickTotal_s = 0 ;
			//g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_2] = 0 ;
			//g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_2] = 0 ;
			//g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_2] = 0 ;
		}
	}
	else
	{
		ticksPre = g_sys_ms_tick;
		tickTotal = 0 ;
		tickTotalCaculate = 0 ;
		tickTotal_s = 0 ;
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_2] = 0 ;
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_2] = 0 ;
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_2] = 0 ;
	}
}


//流速计算
void screenSDWe_LiuSuCaculate(UINT32 cycle)
{
	static UINT32 ticksPre,weightCunPre=0;
	float liusu = 0 ;
	if((cycle != 0) && (SDWeCurPage_ShiShiJieMian == g_T5L.curPage))
	{
		if(g_sys_ms_tick - ticksPre >= cycle)//3秒计算一次
		{
			//流速 = weight / time(s)
			if(g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_CUN_WEIGHT] <= weightCunPre)
			{
				liusu = 0 ;
			}
			else
			{
				liusu = g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_CUN_WEIGHT] - weightCunPre;
				liusu /= (g_sys_ms_tick - ticksPre);
				liusu *= 60000;
			}
			g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_LIUSU] = liusu;
			
			//
			ticksPre = g_sys_ms_tick;
			weightCunPre = g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_CUN_WEIGHT];
		}
	}
	else
	{
		ticksPre = g_sys_ms_tick;
		weightCunPre = g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_CUN_WEIGHT];
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_LIUSU] = 0 ;
	}

}



//======实时采样界面 拿下事件判断
void screenSDWe_TakeDownCheck(void)
{
	static UINT16 takeDownOccureDelay = 0 ,takeDownOccureDelayCheck = 1500, takeDownOccureEvent = FALSE ,takeDownOccureMotorCtl = FALSE;
	static UINT16 putBackOccureDelay = 0 ,putBackOccureDelayCheck = 1500, putBackOccureEvent = FALSE,putBackOccureMotorCtl = FALSE;

	(void)takeDownOccureEvent;
	(void)putBackOccureEvent;

	//实时采集页面 计算百分比等
	if(SDWeCurPage_ShiShiJieMian == g_T5L.curPage)
	{
		//当实时采集界面的总重量 小于	  空袋重量 ，认为袋子发生了取出
		if(g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_TOTAL_WEIGHT] < (g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_KONGDAI_WEIGHT] - gSystemPara.zeroRange))
		{
			putBackOccureDelay = 0 ;
			putBackOccureMotorCtl = FALSE;
			if(takeDownOccureDelay++ > takeDownOccureDelayCheck)//1.5秒
			{
				takeDownOccureDelay = takeDownOccureDelayCheck;
				g_T5L.takeDownOccure = TRUE;
				if(FALSE == takeDownOccureMotorCtl)
				{
					takeDownOccureMotorCtl = TRUE;
					app_SetMotorContrlMode(MotorCtrlStatus_EN_StopToMiddle);//停在水平
				}
			}
		}
		else 
		{
			takeDownOccureDelay = 0 ;
			takeDownOccureMotorCtl = FALSE;
			if(putBackOccureDelay++ > putBackOccureDelayCheck)//1.5秒
			{
				putBackOccureDelay = putBackOccureDelayCheck;
				g_T5L.takeDownOccure = FALSE;
				if(FALSE == putBackOccureMotorCtl)
				{
					putBackOccureMotorCtl = TRUE;
					app_SetMotorContrlMode(MotorCtrlStatus_EN_NormalRun_WitMaxPos);//电机按限位开关运行
				}
			}
		}
	}
	else
	{
		takeDownOccureDelay = 0 ;
		takeDownOccureMotorCtl = FALSE;

		putBackOccureDelay = 0 ;
		putBackOccureMotorCtl = FALSE;

		g_T5L.takeDownOccure = FALSE;
	}
}



//======采集重量 周期发送数据 给屏
void screenSDWe_CycleDataSend(UINT16 cycle)
{
	static UINT16 needSend=FALSE;
	INT16 i=0,*preData,*curData;
	static float weight = 0 ,weight_pre = 0;
 	static UINT32 cycleSendCnt = 0 ;
		
	(void)weight_pre;

	//获取当前总重量 单位：g
	weight = hx711_getWeight(HX711Chanel_1);

	//计算当前总重量 单位：ml
	if(gSystemPara.u16_ml_g_bilv !=0 )
	{
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_TOTAL_WEIGHT] = weight *1000 / gSystemPara.u16_ml_g_bilv;
	}
	else
	{
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_TOTAL_WEIGHT] = weight;
	}

	//实时采集页面 计算百分比等
	if(SDWeCurPage_ShiShiJieMian == g_T5L.curPage)
	{
		//============================================================================================
		//计算纯重量
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_CUN_WEIGHT] = 
			g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_TOTAL_WEIGHT] - 
				g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_KONGDAI_WEIGHT]; 
		
		//百分比值
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] = 
			(100*g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_CUN_WEIGHT]) / g_T5L.curCaiJiDangWeiWeight;
		
		
		//百分比图标
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] = 
			g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT];

		if(g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] >= 100)
		{
			g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] = 100;//避免屏显示错误
		}else if (g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] < 0)
		{
			g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] = 0;//避免屏显示错误
		}		
	}
	//非实时页面：清零操作
	else
	{
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_CUN_WEIGHT] = 0 ;
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] = 0 ;
		g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] = 0;
	}


	//===-比对当前数据和上次数据 ， 判断是否需要发送给屏
	if((cycleSendCnt++%cycle) == 0 )
	{
		preData = &g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_PRE][0];
		curData = &g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][0];
		//1.check if need send
		if(FALSE == needSend)
		{
			for(i=0;i<SDWE_CYCLE_DATA_STATUS_MAX;i++)
			{
				if(preData[i] != curData[i])
				{
					needSend = TRUE;
					break;
				}
			}
			for(i=0;i<SDWE_CYCLE_DATA_STATUS_MAX;i++)
			{
				preData[i] = curData[i];
			}
		}
	}

	//2.if need send
	if(TRUE == needSend)
	{
		//3.wait send ok
		if(TRUE == screenSDWeWriteVarible(SDWE_CYCLE_DATA_START_ADDRESS,(UINT16 *)preData,SDWE_CYCLE_DATA_STATUS_MAX,1))
		{
			//4.clear need send flag
			needSend = FALSE;
		}	
	}
}





//事件处理：
UINT8 XXX_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	switch(status)
	{
		case 0://条件判断
		break;
		case 1://数据准备 及发送
		break;
		case 2://页面切换
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










//======事件处理：语音播报音量 checked:20220526
UINT8 screenVoice_Handle(UINT16 eventVlu)
{
	//举例，一段语音（比如“欢迎光临武汉中显”）保存为6.wav，要以100%音量播放，串口下发： 
	//A5 5A 07 80 50 5b 00 06 5A 40 
	//要停止语音播放，串口下发：A5 5A 05 80 50 5c 00 06

	UINT8 ret = FALSE,regData[2];
	static UINT8 status = 0;
	
	(void)eventVlu;
	switch(status)
	{
		case 0://条件判断
			status = 1;
		break;
		case 1://数据准备 及发送
			regData[0] = 0X5A;
			regData[1] = eventVlu%101;
			if(TRUE == screenSDWeWriteReg(SDWE_VOICE_SET_REG_ADDRESS,&regData[0],SDWE_VOICE_SET_REG_LEN,SDWE_CRC16_EN))
			{
				status = 0 ;
				ret = TRUE;
			}
		break;
		case 2://页面切换
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


//======事件处理：蜂鸣器 checked:20220526
UINT8 screenFengMingQi_Handle(UINT16 eventVlu)
{
	//例如，控制蜂鸣器鸣叫2s，发送指令A5 5A 03 80 02 C8
	//间隔10ms

	UINT8 ret = FALSE,regData[1];
	static UINT8 status = 0;
	
	(void)eventVlu;//单位10ms
	switch(status)
	{
		case 0://条件判断
			status = 1;
		break;
		case 1://数据准备 及发送
			regData[0] = eventVlu/10;
			if(TRUE == screenSDWeWriteReg(SDWE_FENGMINGQI_SET_REG_ADDRESS,&regData[0],SDWE_FENGMINGQI_SET_REG_LEN,SDWE_CRC16_EN))
			{
				status = 0 ;
				ret = TRUE;
			}
		break;
		case 2://页面切换
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



//======事件处理：按下设置返回A checked:??????
UINT8 backReturnASetContrl_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	switch(status)
	{
		case 0://条件判断
			if(SDWeFanHuiJieShuSeZhi_SheZhi == g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FJS_A])
			{

			}
		break;
		case 1://数据准备 及发送
		break;
		case 2://页面切换
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


//======事件处理：按下设置返回B checked:20220526
UINT8 backReturnBSetContrl_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	switch(status)
	{
		case 0://条件判断
			status =1;
		break;
		case 1://数据准备 及发送
			//if(SDWeZhanTingYuXing_YunXing == g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_ZT_YX])
			if(SDWeZhanTingYuXing_YunXing == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX))
			{
				status = 2;
				app_SetMotorContrlMode(MotorCtrlStatus_EN_StopToMiddle);//返回主界面 选择200/300/400时 电机 停在水平
			}
			else
			{
				status = 0XFF;
				ret = TRUE;
			}
		
		break;
		case 2://页面切换
			if(TRUE == screenSDWe_JumpToPage(SDWeCurPage_KaiShiJieMian))
			{
				g_T5L.curPage = SDWeCurPage_KaiShiJieMian;

				//开管夹阀
				app_gjf_handle(SDWeFaKaiFaGuan_FaKai);
				
				status = 3;
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

//======事件处理：按下管夹阀 开关 checked:20220526
UINT8 gjfContrl_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	switch(status)
	{
		case 0://条件判断
			if(SDWeFaKaiFaGuan_FaKai == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FK_FG))
			{
				app_gjf_handle(SDWeFaKaiFaGuan_FaGuan);
			}
			else if(SDWeFaKaiFaGuan_FaGuan == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FK_FG))
			{
				app_gjf_handle(SDWeFaKaiFaGuan_FaKai);
			}			
			status = 0;
			ret = TRUE;
		break;
		case 1://数据准备 及发送
		break;
		case 2://页面切换
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


//======事件处理：按下实时界面 里面的RUN checked:??????
UINT8 runContrl_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	switch(status)
	{
		case 0://条件判断
			if(SDWeZhanTingYuXing_YunXing == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX))
			{
				if( GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_PERCENT) < 100 )
				{
					//设置cycleData
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX,SDWeZhanTingYuXing_ZhanTing);//设置运行/暂停 = 暂停
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_A,SDWeFanHuiJieShuSeZhi_KongBai);//设置返回结束A = 空白
					SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_B,SDWeFanHuiJieShuSeZhi_KongBai);//设置返回结束B = 空白

					//开管夹阀
					app_gjf_handle(SDWeFaKaiFaGuan_FaKai);
					
					//设置电机控制进入限位开关控制
					app_SetMotorContrlMode(MotorCtrlStatus_EN_NormalRun_WitMaxPos);

					//打开语音：“采集开始”
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_KaiShi,TRUE);
				}
			}
			else if(SDWeZhanTingYuXing_ZhanTing == GET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX))
			{
				//设置cycleData
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_ZT_YX,SDWeZhanTingYuXing_YunXing);//设置运行/暂停 = 运行
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_A,SDWeFanHuiJieShuSeZhi_KongBai);//设置返回结束A = 空白
				SET_SDWE_CUR_CYCLE_DATA(SDWE_CYCLE_DATA_STATUS_FJS_B,SDWeFanHuiJieShuSeZhi_JieShu);//设置返回结束B = 结束

				//关管夹阀
				app_gjf_handle(SDWeFaKaiFaGuan_FaGuan);
			}
			status = 1;
		break;
		case 1://数据准备 及发送
			if(TRUE == screenSDWeWriteVarible(SDWE_CYCLE_DATA_START_ADDRESS,(UINT16 *)GET_SDWE_CUR_CYCLE_DATA_PTR(),SDWE_CYCLE_DATA_STATUS_MAX,SDWE_CRC16_EN))
			{
				status = 0;
				ret = TRUE;
			}
		break;
		case 2://页面切换
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








//======事件处理：按下200 300 400进入实时界面 checked:20220526
UINT8 enterRealTimeSet_Handle(UINT16 eventVlu)
{
	UINT8 ret = FALSE;
	static UINT8 status = 0;
	(void)eventVlu;
	//条件判断
	if(((TRUE == gSystemPara.u16_kaiqi_guanjiafa_gongneng) && (SDWeFaKaiFaGuan_FaGuan == g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FK_FG]))
		|| (TRUE != gSystemPara.u16_kaiqi_guanjiafa_gongneng) )
	{
		switch(status)
		{
			case 0://数据准备
					//g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_TOTAL_WEIGHT] = (UINT16)hx711_getWeight(HX711Chanel_1);
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_CUN_WEIGHT] = 0;
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT] = 0 ;
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_PERCENT_TUBIAO] = 0 ;
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_LIUSU] = 0 ;
					//时间
					g_T5L.rtcTime_s = g_T5L.rtcTime_ms/1000;
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_1] = (g_T5L.rtcTime_s/3600)%60;
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_1] = (g_T5L.rtcTime_s/60)%60;
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_1] = (g_T5L.rtcTime_s)%60;
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_HH_2] = 0 ;
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_MM_2] = 0 ;
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_SS_2] = 0 ;
					//
					if((200 == g_T5L.enterRealTimeSetContrl[SYS_CTL_REG_STATUS_INDEX]) || (SYS_KEY_VALUED == key_FilterGet(TBES_200_MODLE_CHOICE)))
					{
						g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_YUSE_WEIGHT] = gSystemPara.yusheWeight[SDWeCaiJiDangWei_200];
						g_T5L.curCaiJiDangWeiWeight = gSystemPara.yusheWeight[SDWeCaiJiDangWei_200];
						
					}
					else if((300 == g_T5L.enterRealTimeSetContrl[SYS_CTL_REG_STATUS_INDEX]) || (SYS_KEY_VALUED == key_FilterGet(TBES_300_MODLE_CHOICE)))
					{
						g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_YUSE_WEIGHT] = gSystemPara.yusheWeight[SDWeCaiJiDangWei_300];
						g_T5L.curCaiJiDangWeiWeight = gSystemPara.yusheWeight[SDWeCaiJiDangWei_300];
					}else if((400 == g_T5L.enterRealTimeSetContrl[SYS_CTL_REG_STATUS_INDEX]) || (SYS_KEY_VALUED == key_FilterGet(TBES_400_MODLE_CHOICE)))
					{
						g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_YUSE_WEIGHT] = gSystemPara.yusheWeight[SDWeCaiJiDangWei_400];
						g_T5L.curCaiJiDangWeiWeight = gSystemPara.yusheWeight[SDWeCaiJiDangWei_400];
					}
					else
					{
						g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_YUSE_WEIGHT] = 999;
						g_T5L.curCaiJiDangWeiWeight = gSystemPara.yusheWeight[SDWeCaiJiDangWei_200];
					}

					//当选择挡位进入时：空袋总量等于实际采样重量
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_KONGDAI_WEIGHT] = 
						g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_TOTAL_WEIGHT];
					
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FJS_A] = (UINT16)SDWeFanHuiJieShuSeZhi_KongBai;//A = 空白
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FJS_B] = (UINT16)SDWeFanHuiJieShuSeZhi_FanHui;//B = 返回
					if(TRUE != gSystemPara.u16_kaiqi_guanjiafa_gongneng)
					{
						g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FK_FG] = (UINT16)SDWeFaKaiFaGuan_KongBai;//阀开/关 = 空白
					}
					else
					{
						g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_FK_FG] = (UINT16)SDWeFaKaiFaGuan_FaGuan;//阀开/关 = 阀关 
					}
					g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_STATUS_ZT_YX] = (UINT16)SDWeZhanTingYuXing_YunXing;//运行
			
					if(TRUE == screenSDWeWriteVarible(SDWE_CYCLE_DATA_START_ADDRESS,(UINT16 *)&g_T5L.cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][0],SDWE_CYCLE_DATA_STATUS_MAX,SDWE_CRC16_EN))
					{
						status = 1;
					}

					g_T5L.takeDownOccure = FALSE;
			break;
			case 1://切页面
				if(TRUE == screenSDWe_JumpToPage(SDWeCurPage_ShiShiJieMian))
				{
					g_T5L.curPage = SDWeCurPage_ShiShiJieMian;
					status = 2;
					ret = TRUE;
				}
			break;

			default :
				status = 0 ;
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









//======事件处理：设置RTC时间 checked:？？？
UINT8 rtcSet_Handle(void)
{
	UINT8 ret = FALSE;
	UINT8 dataRtc[8]={0X5A,0X22,0X05,0X25,0X03,0X18,0X56,0X00};//YY:MM:DD:WW:HH:MM:SS
	static UINT8 status = 0;
	switch(status)
	{
		case 0://条件判断
			status = 1;
		break;
		case 1://数据准备 及发送
			dataRtc[5] = (g_T5L.rtcTime_s /3600)%24;
			dataRtc[6] = ((g_T5L.rtcTime_s %3600)/60)%24;
			dataRtc[7] = (g_T5L.rtcTime_s %60);
			if(TRUE == screenSDWeWriteReg((SDWE_RTC_TIMER_ADDRESS-1),&dataRtc[0],(SDWE_RTC_TIMER_LEN+1),SDWE_CRC16_EN))
			{
				status = 0;
				ret = TRUE;
			}
		break;
		case 2://页面切换
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

//======开机动画 采用的图标变量 checked:20220526
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

//======上电发送初始数据给屏 checked:20220526
UINT8 screenSendDataToSDWE(void)
{
	UINT8 ret = FALSE;
	UINT16 i=0,yinyong[17];//0x3600
	static UINT8 status = 0;
	switch(status)
	{
		case 0://条件判断
			status = 1;
		break;
		case 1://数据准备 及发送
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
				status = 0;
				ret = TRUE;
			}
		break;
		case 2://页面切换
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

//======prepare TX data checked:20220526
void screenSDWe_TxFunction(void)
{
	T5LType *pScreen = &g_T5L;
	//
	static UINT32 maxWaitTime;
	switch(pScreen->sdweCtlStep)
	{
		//开机动画
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
		
		//获取RTC时间
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
		//收到RTC时间
		case SDWeCtlStep_RecvRTC:
			if(TRUE == pScreen->getRTC)
			{
				maxWaitTime = 0 ;
				pScreen->sdweCtlStep = SDWeCtlStep_SendDataToSDWE;
			}
			else if(maxWaitTime < 2000)//2s 超时
			{
				maxWaitTime++;
			}
			else
			{
				maxWaitTime = 0 ;
				pScreen->sdweCtlStep = SDWeCtlStep_SendDataToSDWE;
			}
		break;

		//发送系统参数给SDWe
		case SDWeCtlStep_SendDataToSDWE:
			if(TRUE == screenSendDataToSDWE())
			{
				pScreen->sdweCtlStep = SDWeCtlStep_CycleHandle;

			}
		break;
		
		//周期处理	
		case SDWeCtlStep_CycleHandle:
			//A按键事件处理：{设置 返回 退出}
			if(SYS_CTL_EVENT_VALID == pScreen->backReturnASetContrl[SYS_CTL_REG_EVENT_INDEX])
			{
				//if(TRUE == backReturnASetContrl_Handle(pScreen->backReturnASetContrl[SYS_CTL_REG_STATUS_INDEX]))
				{
					pScreen->backReturnASetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_INVALID;
				}
			}
			//B按键事件处理：{设置 返回 退出}
			else if(SYS_CTL_EVENT_VALID == pScreen->backReturnBSetContrl[SYS_CTL_REG_EVENT_INDEX])
			{
				if(TRUE == backReturnBSetContrl_Handle(pScreen->backReturnBSetContrl[SYS_CTL_REG_STATUS_INDEX]))
				{
					pScreen->backReturnBSetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_INVALID;
					//
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_KaiShi,FALSE);
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_80,FALSE);
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_90,FALSE);
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_WanCheng,FALSE);
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_Warn_LiuSu_High,FALSE);
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_Warn_LiuSu_Low,FALSE);
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_DiDi_DiDi,FALSE);
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
				//==袋已放置
				if(pScreen->cycleData[SDWE_CYCLE_DATA_ARR_INDEX_CUR][SDWE_CYCLE_DATA_TOTAL_WEIGHT] > gSystemPara.zeroRange)
				{
					if(TRUE == enterRealTimeSet_Handle(pScreen->enterRealTimeSetContrl[SYS_CTL_REG_STATUS_INDEX]))
					{
						pScreen->enterRealTimeSetContrl[SYS_CTL_REG_STATUS_INDEX] = 0XFFFF;
						pScreen->enterRealTimeSetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_INVALID;
						//
						screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_KaiShi,FALSE);
						screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_80,TRUE);
						screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_90,TRUE);
						screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_CaiJi_WanCheng,TRUE);
						screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_Warn_LiuSu_High,TRUE);
						screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_Warn_LiuSu_Low,TRUE);
						screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_DiDi_DiDi,TRUE);
					}
				}
				else//==袋未放置
				{
					pScreen->enterRealTimeSetContrl[SYS_CTL_REG_EVENT_INDEX] = SYS_CTL_EVENT_INVALID;
					screenSDWe_CaiJiYuYin_Set(SDWEVoicePrintf_QinFangDai,TRUE);
				}
			}
			else if((TRUE == pScreen->sdweRemoveWeightTriger))
			{
				hx711_setAllRemoveWeight();
				pScreen->sdweRemoveWeightTriger = FALSE;
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
			else 
			{
				screenSDWe_CycleDataSend(500);
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
	//举例：连续读取寄存器寄存器0x03和0x04单元 
	//发送：0xA5 0x5A     0x03       0x81   0x03     0x02 
	//		  header   order_len  order	 address  len

	//返回：0xA5 0x5A     0x05       0x81   0x03     0x02    0x00    0x01
	//		  header   order_len  order	 address  len     data[3] data[4]


	//举例：连续读取变量存储器0x0003和0x0004单元 
	//发送：0xA5 0x5A      0x04     0x83  0x00 0x03   0x02 
	//        header  order_len  order  address    len

	//返回：0xA5 0x5A      0x08     0x83  0x00 0x03   0x02    0x00 0x01   0xff 0xff 
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
 * Functions : SDWe屏 子功能 测试
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
 * Functions : SDWe屏 主函数
 ******************************************************************************/
void sreenSDWe_MainFunction(void)
{
	T5LType *pScreen = &g_T5L;
	//
	pScreen->CurTick++;
	if(pScreen->CurTick > SYS_POWER_REDAY_TIME)//this time not overview !!!
	{
		//SDWe 基本功能测试
		screenSDWe_SubFunctionTest();
		
		//deal rx data from SDWe
		screenSDWe_RxFunction();
		
		//prepare data and send to SDWe
		screenSDWe_TxFunction();
		
		//流速计算 3秒计算一次
		screenSDWe_LiuSuCaculate(2000);
		
		//采样时间计算
		screenSDWe_SampleTimeCaculate();
		
		//RTC时间 ms++ s++
		screenSDWe_RTCTimeCaculate();
		
		//==语音提示
		screenSDWe_CaiJiYuYin_Handle();

		//==采集完成处理
		screenSDWe_CaiJiWanCheng_Handle();

		//==实时界面 拿下 放回 处理
		screenSDWe_TakeDownCheck();
	}
}


