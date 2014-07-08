/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *                                        
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#ifndef __HAL_COM_PHYCFG_H__
#define __HAL_COM_PHYCFG_H__

#define		PathA                     			0x0	// Useless
#define		PathB                     			0x1
#define		PathC                     			0x2
#define		PathD                     			0x3

typedef enum _RATE_SECTION {
	CCK = 0,
	OFDM,
	HT_MCS0_MCS7,
	HT_MCS8_MCS15,	
	VHT_1SSMCS0_1SSMCS9,
	VHT_2SSMCS0_2SSMCS9,
} RATE_SECTION;

typedef enum _RF_TX_NUM {
	RF_1TX = 0,
	RF_2TX,
	RF_MAX_TX_NUM,
	RF_TX_NUM_NONIMPLEMENT,
} RF_TX_NUM;

#define MAX_POWER_INDEX 		0x3F

typedef enum _REGULATION_TXPWR_LMT {
	TXPWR_LMT_FCC = 0,
	TXPWR_LMT_MKK = 1,
	TXPWR_LMT_ETSI = 2,
	TXPWR_LMT_WW = 3,	

	TXPWR_LMT_MAX_REGULATION_NUM = 4
} REGULATION_TXPWR_LMT;

/*------------------------------Define structure----------------------------*/ 
typedef struct _BB_REGISTER_DEFINITION{
	u32 rfintfs;			// set software control: 
						//		0x870~0x877[8 bytes]
							
	u32 rfintfo; 			// output data: 
						//		0x860~0x86f [16 bytes]
							
	u32 rfintfe; 			// output enable: 
						//		0x860~0x86f [16 bytes]
							
	u32 rf3wireOffset;	// LSSI data:
						//		0x840~0x84f [16 bytes]

	u32 rfHSSIPara2; 	// wire parameter control2 : 
						//		0x824~0x827,0x82c~0x82f, 0x834~0x837, 0x83c~0x83f [16 bytes]
								
	u32 rfLSSIReadBack; 	//LSSI RF readback data SI mode
						//		0x8a0~0x8af [16 bytes]

	u32 rfLSSIReadBackPi; 	//LSSI RF readback data PI mode 0x8b8-8bc for Path A and B

}BB_REGISTER_DEFINITION_T, *PBB_REGISTER_DEFINITION_T;

#ifndef CONFIG_EMBEDDED_FWIMG
int phy_ConfigMACWithParaFile(IN PADAPTER	Adapter, IN s8*	pFileName);

int phy_ConfigBBWithParaFile(IN PADAPTER	Adapter, IN s8*	pFileName);

int phy_ConfigBBWithPgParaFile(IN PADAPTER	Adapter, IN s8*	pFileName);

int phy_ConfigBBWithMpParaFile(IN PADAPTER	Adapter, IN s8*	pFileName);

int PHY_ConfigBBWithCustomPgParaFile(IN PADAPTER	Adapter, IN s8*	pFileName, IN BOOLEAN	checkInit);

int PHY_ConfigRFWithParaFile(IN	PADAPTER	Adapter, IN s8*	pFileName, IN u8	eRFPath);

int PHY_ConfigRFWithPowerLimitTableParaFile(IN PADAPTER	Adapter, IN s8*	pFileName);

int PHY_ConfigRFWithCustomPowerLimitTableParaFile(IN PADAPTER	Adapter,IN s8*	pFileName, IN BOOLEAN	checkInit);

int PHY_ConfigRFWithTxPwrTrackParaFile(IN PADAPTER	Adapter, IN s8*	pFileName);
#endif

//----------------------------------------------------------------------
s32
phy_TxPwrIdxToDbm(
	IN	PADAPTER		Adapter,
	IN	WIRELESS_MODE	WirelessMode,
	IN	u8				TxPwrIdx	
	);

VOID
PHY_StoreTxPowerByRateBase(	
	IN	PADAPTER	pAdapter
	);

u8
PHY_GetRateSectionIndexOfTxPowerByRate(
	IN	PADAPTER	pAdapter,
	IN	u32			RegAddr,
	IN	u32			BitMask
	);

VOID
PHY_InitTxPowerByRate(
	IN	PADAPTER	pAdapter
	);

VOID
PHY_StoreTxPowerByRate(
	IN	PADAPTER	pAdapter,
	IN	u32			Band,
	IN	u32			RfPath,
	IN	u32			TxNum,
	IN	u32			RegAddr,
	IN	u32			BitMask,
	IN	u32			Data
	);

u8
PHY_GetTxPowerByRateNew(
	IN	PADAPTER	pAdapter,
	IN	u8			Band,
	IN	u8			RfPath,
	IN	u8			TxNum,
	IN	u8			Rate
	);

void 
PHY_SetTxPowerByRateOld(
	IN	PADAPTER	pAdapter,
	IN	u32			RegAddr,
	IN	u32			BitMask,
	IN	u32			Data
	);

VOID
PHY_TxPowerByRateConfiguration(
	IN  PADAPTER			pAdapter
	);

VOID 
PHY_SetTxPowerIndexByRateSection(
	IN	PADAPTER		pAdapter,
	IN	u8				RFPath,	
	IN	u8				Channel,
	IN	u8				RateSection
	);

s8
PHY_GetTxPowerLimit(
	IN	PADAPTER		Adapter,
	IN	u32				RegPwrTblSel,
	IN	BAND_TYPE		Band,
	IN	CHANNEL_WIDTH	Bandwidth,
	IN	u8				RfPath,
	IN	u8				DataRate,
	IN	u8				Channel
	);

VOID
PHY_SetTxPowerLimit(
	IN	PADAPTER			Adapter,
	IN	u8					*Regulation,
	IN	u8					*Band,
	IN	u8					*Bandwidth,
	IN	u8					*RateSection,
	IN	u8					*RfPath,
	IN	u8					*Channel,
	IN	u8					*PowerLimit
	);

VOID 
PHY_ConvertTxPowerLimitToPowerIndex(
	IN	PADAPTER			Adapter
	);

VOID
PHY_InitTxPowerLimit(
	IN	PADAPTER			Adapter
	);

VOID
PHY_SetTxPowerByRateBase(
	IN	PADAPTER			Adapter,
	IN	u8					Band,
	IN	u8					RfPath,
	IN	RATE_SECTION		RateSection,
	IN	u8					TxNum,
	IN	u8					Value
	);

VOID 
PHY_ConvertTxPowerByRateInDbmToRelativeValues(
	IN	u32		*pData,
	IN	u8		Start,
	IN	u8		End,
	IN	u8		BaseValue
	);

u8
PHY_GetTxPowerByRateBase(
	IN	PADAPTER		Adapter,
	IN	u8				Band,
	IN	u8				RfPath,
	IN	u8				TxNum,
	IN	RATE_SECTION	RateSection
	);

VOID
Hal_ChannelPlanToRegulation(
	IN	PADAPTER		Adapter,
	IN	u16				ChannelPlan
	);

#endif //__HAL_COMMON_H__

