#include "typedefine.h"
#include "app_crc.h"
#include "drv_flash.h"
#include "app_syspara.h"
#include "app_password.h"
#include "app_hx711_ctrl.h"
#include "app_t5l_ctrl.h"
#include "app_motor_ctrl.h"

//store flash data : 8 * (sample value , weight value , k , b , remove value ) , last one is crc
unionFloatInt32 flashStoreDataBuf[FLASH_STORE_MAX_LEN]={0};
unionFloatInt32 flashStoreDataBuf_3030[FLASH_SYS_PARA_STORE_MAX_LEN]={0};

//=======================sys parameter read :unit min max....
gSystemParaType gSystemPara = gSystemParaDefault;

//==read data from flash
void readSysDataFromFlash(void)
{
	ChanelType *pChanel = 0;	
	unionFloatInt32 readflashDataBuf[FLASH_STORE_MAX_LEN]={0};
	INT32 *pInt32 = 0;
	float *pFloat = 0;
	UINT32 crc = 0 ;
	UINT16 chanel_i = 0 ,start_i = 0 , end_i = 0;
	UINT8 point_i = 0 ;
	//read data from flash
	drv_flash_read_words( FLASH_STORE_ADDRESS_START, (UINT32 *)(&readflashDataBuf[0].i_value), FLASH_STORE_MAX_LEN);

	//crc
	crc = readflashDataBuf[FLASH_STORE_MAX_LEN-1].i_value;
	if(crc == cal_crc16(((UINT8 *)&readflashDataBuf[0].u_value),(4*(FLASH_STORE_MAX_LEN-1)))) 
	{
		start_i = 0 ;
		end_i = 0 ;
		//get ram buf
		for(chanel_i=0;chanel_i<HX711_CHANEL_NUM;chanel_i++)
		{
				//get chanel
				pChanel = getChanelStruct(chanel_i);
				//==point sample value
				point_i = 0 ;
				start_i = end_i ;
				end_i = start_i + CHANEL_POINT_NUM;
				pInt32 = &(pChanel->section_PointSample[0]);
				for(;start_i<end_i;start_i++)
				{
					*pInt32++ = readflashDataBuf[start_i].i_value;
					pointSampleTrigerDataSet(chanel_i,(point_i++),(readflashDataBuf[start_i].i_value/512));
				}
				//==point weight value
				point_i = 0;
				start_i = end_i ;
				end_i = start_i + CHANEL_POINT_NUM;
				pInt32 = &(pChanel->section_PointWeight[0]);
				for(;start_i<end_i;start_i++)
				{
					*pInt32++ = readflashDataBuf[start_i].i_value;
					pointWeightTrigerDataSet(chanel_i,(point_i++),readflashDataBuf[start_i].i_value);
				}	
				//==point K value
				start_i = end_i ;
				end_i = start_i + CHANEL_POINT_NUM + 1;
				pFloat = &(pChanel->section_K[0]);
				for(;start_i<end_i;start_i++)
				{
					*pFloat++ = readflashDataBuf[start_i].f_value;
				}
				//==point B value
				start_i = end_i ;
				end_i = start_i + CHANEL_POINT_NUM + 1;
				pFloat = &(pChanel->section_B[0]);
				for(;start_i<end_i;start_i++)
				{
					*pFloat++ = readflashDataBuf[start_i].f_value;
				}
				//==point remove value
				start_i = end_i ;
				end_i = start_i + 1;
				pFloat = &(pChanel->weightRemove);
				for(;start_i<end_i;start_i++)
				{
					*pFloat++ = readflashDataBuf[start_i].f_value;
				}
				//==point weight direction
				start_i = end_i ;
				end_i = start_i + 1;
				pInt32 = &(pChanel->weightDir);
				for(;start_i<end_i;start_i++)
				{
					*pInt32++ = readflashDataBuf[start_i].i_value;
				}
			}
		}
}
//==store set data to flash
void storeSysDataToFlash(void)
{
	static UINT16 storeTick = 0 ; 
	ChanelType *pChanel = 0;	
	unionFloatInt32 *pWordInt32Float=&flashStoreDataBuf[0];
	UINT8 *pChar = 0 ;
	INT32 *pInt32 = 0 ;
	float *pFloat = 0;
	UINT32 crc = 0 ;
	UINT16 chanel_i = 0 ,start_i = 0 , end_i = 0;
	//get ram buf
	for(chanel_i=0;chanel_i<HX711_CHANEL_NUM;chanel_i++)
	{
		//get chanel
		pChanel = getChanelStruct(chanel_i);
		//==point sample value value
		start_i = end_i ;
		end_i = start_i+CHANEL_POINT_NUM;
		pInt32 = (INT32 *)&(pChanel->section_PointSample[0]);
		for(;start_i<end_i;start_i++)
		{
			if(start_i < (FLASH_STORE_MAX_LEN - 1))
			{
				pWordInt32Float[start_i].i_value = *pInt32++;
			}
		}
		//==point weight value
		start_i = end_i ;
		end_i = start_i+CHANEL_POINT_NUM;
		pInt32 = (INT32 *)&(pChanel->section_PointWeight[0]);
		for(;start_i<end_i;start_i++)
		{
			if(start_i < (FLASH_STORE_MAX_LEN - 1))
			{
				pWordInt32Float[start_i].i_value = *pInt32++;
			}
		}
		//==point K
		start_i = end_i ;
		end_i = start_i+CHANEL_POINT_NUM+1;
		pFloat = (float *)&(pChanel->section_K[0]);
		for(;start_i<end_i;start_i++)
		{
			if(start_i < (FLASH_STORE_MAX_LEN - 1))
			{
				pWordInt32Float[start_i].f_value = *pFloat++;
			}
		}
		//==point B
		start_i = end_i ;
		end_i = start_i+CHANEL_POINT_NUM+1;
		pFloat = (float *)&(pChanel->section_B[0]);
		for(;start_i<end_i;start_i++)
		{
			if(start_i < (FLASH_STORE_MAX_LEN - 1))
			{
				pWordInt32Float[start_i].f_value = *pFloat++;
			}
		}
		//==point remove weight
		start_i = end_i ;
		end_i = start_i+1;
		pFloat = (float *)&(pChanel->weightRemove);
		for(;start_i<end_i;start_i++)
		{
			if(start_i < (FLASH_STORE_MAX_LEN - 1))
			{
				pWordInt32Float[start_i].f_value = *pFloat++;
			}
		}
		//==point weight dirction
		start_i = end_i ;
		end_i = start_i+1;
		pInt32 = (INT32 *)&(pChanel->weightDir);
		for(;start_i<end_i;start_i++)
		{
			if(start_i < (FLASH_STORE_MAX_LEN - 1))
			{
				pWordInt32Float[start_i].i_value = *pInt32++;
			}
		}
	}
	//
	pChar = (UINT8 *)(&pWordInt32Float[0].u_value[0]);
	crc = cal_crc16(pChar,(4*start_i));
	pWordInt32Float[start_i].i_value = crc;
	start_i++;
	//write flash
	if(start_i <= FLASH_STORE_MAX_LEN)
	{	
		storeTick++;
		drv_flash_unlock();
		drv_flash_erase_sector(FLASH_STORE_ADDRESS_START);
		drv_flash_write_words( FLASH_STORE_ADDRESS_START, (UINT32 *)(&pWordInt32Float[0].i_value), (start_i) );
		drv_flash_lock();
	}
}

//=======================v3.0
void readSysDataFromFlash_3030(void)
{
	unionFloatInt32 readflashDataBuf[FLASH_SYS_PARA_STORE_MAX_LEN]={0};
	UINT32 crc = 0 ;
	UINT16 start_i = 0 , end_i = 0;
	UINT8 point_i = 0 ;
	//read data from flash
	drv_flash_read_words( FLASH_SYS_PARA_STORE_ADDRESS_START, (UINT32 *)(&readflashDataBuf[0].i_value), FLASH_SYS_PARA_STORE_MAX_LEN);

	//crc
	crc = readflashDataBuf[FLASH_SYS_PARA_STORE_MAX_LEN-1].i_value;
	if(crc == cal_crc16(((UINT8 *)&readflashDataBuf[0].u_value),(4*(FLASH_SYS_PARA_STORE_MAX_LEN-1)))) 
	{
		start_i = 0 ;
		g_passWordStore = readflashDataBuf[start_i++].i_value;/**< ?????? */
		gSystemPara.uint = readflashDataBuf[start_i++].i_value;/**< ?????? */
		gSystemPara.minWeight = readflashDataBuf[start_i++].i_value;/**< ???????????? */
		gSystemPara.maxWeight = readflashDataBuf[start_i++].i_value;/**< ???????????? */
		gSystemPara.errRange = readflashDataBuf[start_i++].f_value;/**< ???????????? */
		gSystemPara.isCascade = readflashDataBuf[start_i++].i_value;/**< ???????????? */
		gSystemPara.isLedIndicate = readflashDataBuf[start_i++].i_value;/**< ??????LED?????? */
		end_i = start_i+SYS_COLOR_GROUP_NUM;
		for(point_i=0;start_i<end_i;start_i++)
		{
			gSystemPara.userColorSet[point_i++] = readflashDataBuf[start_i].i_value;/**< ?????????1~4 */
		}
		gSystemPara.zeroRange = readflashDataBuf[start_i++].f_value;/**< ???????????? */

		gSystemPara.ScreenVoiceSwitch = readflashDataBuf[start_i++].i_value;/**< HX711	???????????? */ 
		gSystemPara.ScreenCastMode = readflashDataBuf[start_i++].i_value;/**< HX711	?????????????????? */ 
		gSystemPara.FlashEraseTimes = readflashDataBuf[start_i++].i_value;/**< HX711 Flash ???????????? */ 

		gSystemPara.McuVersion = readflashDataBuf[start_i++].i_value;/**< MCU ??????????????? */  
		gSystemPara.DiWenVersion = readflashDataBuf[start_i++].i_value;/**< ??????	??????????????? */ 
		gSystemPara.VoiceNum = readflashDataBuf[start_i++].i_value;/**< ???????????? */ 
		gSystemPara.VoiceNumTouch = readflashDataBuf[start_i++].i_value;/**< ???????????? ??????*/ 
		gSystemPara.ScreenLight = readflashDataBuf[start_i++].i_value;/**< ?????????????????? */	



///

		gSystemPara.yusheWeight[SDWeCaiJiDangWei_200] = readflashDataBuf[start_i++].i_value;/**< 200 ??????????????? */
		gSystemPara.yusheWeight[SDWeCaiJiDangWei_300] = readflashDataBuf[start_i++].i_value;/**< 300 ??????????????? */
		gSystemPara.yusheWeight[SDWeCaiJiDangWei_400] = readflashDataBuf[start_i++].i_value;/**< 400 ??????????????? */


		gSystemPara.u16_ml_g_bilv= readflashDataBuf[start_i++].i_value;//ml???g??????
		gSystemPara.u16_fmqkqsj= readflashDataBuf[start_i++].i_value;//?????????????????????
		gSystemPara.u16_liusu_high_warn= readflashDataBuf[start_i++].i_value;//????????? ?????????
		gSystemPara.u16_liusu_low_warn= readflashDataBuf[start_i++].i_value;//????????? ?????????

		motorCtl.motorAge = readflashDataBuf[start_i++].i_value;//????????????
		motorCtl.tAD = readflashDataBuf[start_i++].i_value;//????????????
		gSystemPara.u16_kaiqi_guanjiafa_gongneng = readflashDataBuf[start_i++].i_value;//?????????????????????
	}
}

//==store set data to flash 3030
void storeSysDataToFlash_3030(void)
{
	static UINT16 storeTick = 0 ; 
	unionFloatInt32 *pWordInt32Float=&flashStoreDataBuf_3030[0];
	UINT8 *pChar = 0 ;
	INT32 *pInt32 = 0 ;
	float *pFloat = 0;
	UINT32 crc = 0 ;
	UINT16 start_i = 0 , end_i = 0;

	//Flash Erase Times
	gSystemPara.FlashEraseTimes++;

	//0
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(g_passWordStore);/**< ?????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//1
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.uint);/**< ?????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//2
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.minWeight);/**< ???????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//3
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.maxWeight);/**< ???????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//4
	start_i = end_i ;
	end_i = start_i+1;
	pFloat = (float *)&(gSystemPara.errRange);/**< ???????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].f_value = *pFloat++;
	}
	//5
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.isCascade);/**< ???????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//6
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.isLedIndicate);/**< ??????LED?????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//7~10
	start_i = end_i ;
	end_i = start_i+SYS_COLOR_GROUP_NUM;
	pInt32 = (INT32 *)&(gSystemPara.userColorSet[0]);/**< ?????????1~4 */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//11
	start_i = end_i ;
	end_i = start_i+1;
	pFloat = (float *)&(gSystemPara.zeroRange);/**< ???????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].f_value = *pFloat++;
	}
	//12
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.ScreenVoiceSwitch);/**< HX711	???????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//13
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.ScreenCastMode);/**< HX711	?????????????????? */ 
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//14
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.FlashEraseTimes);/**< HX711	FLASH???????????? */ 
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//15
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.McuVersion);/**< MCU	??????????????? */ 
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//16
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.DiWenVersion);/**< ??????	??????????????? */ 
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//17
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.VoiceNum);/**< ???????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//18
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.VoiceNumTouch);/**< ???????????? ??????*/
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//19
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.ScreenLight);/**< ?????????????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}




	
	//20 21 22
	start_i = end_i ;
	end_i = start_i+3;
	pInt32 = (INT32 *)&(gSystemPara.yusheWeight[SDWeCaiJiDangWei_200]);/**< 200 300 400 ??????????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//23
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.u16_ml_g_bilv);/**< ml???g?????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//24
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.u16_fmqkqsj);/**< ????????????????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//25
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.u16_liusu_high_warn);/**< ????????? ????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//26
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.u16_liusu_low_warn);/**< ????????? ????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//27
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(motorCtl.motorAge);/**< ???????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//28
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(motorCtl.tAD);/**< ???????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}
	//29
	start_i = end_i ;
	end_i = start_i+1;
	pInt32 = (INT32 *)&(gSystemPara.u16_kaiqi_guanjiafa_gongneng);/**< ????????????????????? */
	for(;start_i<end_i;start_i++)
	{
		pWordInt32Float[start_i].i_value = *pInt32++;
	}


	


	
	//
	pChar = (UINT8 *)(&pWordInt32Float[0].u_value[0]);
	crc = cal_crc16(pChar,(4*start_i));
	pWordInt32Float[start_i].i_value = crc;
	start_i++;
	//write flash
	if(start_i <= FLASH_SYS_PARA_STORE_MAX_LEN)
	{	
		storeTick++;
		drv_flash_unlock();
		drv_flash_erase_sector(FLASH_SYS_PARA_STORE_ADDRESS_START);
		drv_flash_write_words( FLASH_SYS_PARA_STORE_ADDRESS_START, (UINT32 *)(&pWordInt32Float[0].i_value), (start_i) );
		drv_flash_lock();
	}
}

