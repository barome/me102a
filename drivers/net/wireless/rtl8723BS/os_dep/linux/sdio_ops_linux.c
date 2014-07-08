/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
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
 *******************************************************************************/
#define _SDIO_OPS_LINUX_C_

#include <drv_types.h>

#define SD_IO_TRY_CNT (8)
//#define SDIO_DEBUG_IO

static bool rtw_sdio_claim_host_needed(struct sdio_func *func)
{
	struct dvobj_priv *dvobj = sdio_get_drvdata(func);
	PSDIO_DATA sdio_data = &dvobj->intf_data;

	if (sdio_data->sys_sdio_irq_thd && sdio_data->sys_sdio_irq_thd == current)
		return _FALSE;
	return _TRUE;
}

inline void rtw_sdio_set_irq_thd(struct dvobj_priv *dvobj, _thread_hdl_ thd_hdl)
{
	PSDIO_DATA sdio_data = &dvobj->intf_data;

	sdio_data->sys_sdio_irq_thd = thd_hdl;
}

u8 sd_f0_read8(PSDIO_DATA psdio, u32 addr, s32 *err)
{
	u8 v;
	struct sdio_func *func;
	bool claim_needed;

_func_enter_;

	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
	v = sdio_f0_readb(func, addr, err);
	if (claim_needed)
		sdio_release_host(func);
	if (err && *err)
		DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x\n", __func__, *err, addr);

_func_exit_;

	return v;
}

void sd_f0_write8(PSDIO_DATA psdio, u32 addr, u8 v, s32 *err)
{
	struct sdio_func *func;
	bool claim_needed;
	
_func_enter_;

	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
	sdio_f0_writeb(func, v, addr, err);
	if (claim_needed)
		sdio_release_host(func);
	if (err && *err)
		DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x val=0x%02x\n", __func__, *err, addr, v);

_func_exit_;
}

/*
 * Return:
 *	0		Success
 *	others	Fail
 */
s32 _sd_cmd52_read(PSDIO_DATA psdio, u32 addr, u32 cnt, u8 *pdata)
{
	int err, i;
	struct sdio_func *func;

_func_enter_;

	err = 0;
	func = psdio->func;

	for (i = 0; i < cnt; i++) {
#ifdef SDIO_DEBUG_IO
		DBG_871X("%s: sdio_readb addr=0x%05x\n", __FUNCTION__, addr+i);
#endif
		pdata[i] = sdio_readb(func, addr+i, &err);
		if (err) {
			DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x\n", __func__, err, addr+i);
			break;
		}
	}

_func_exit_;

	return err;
}

/*
 * Return:
 *	0		Success
 *	others	Fail
 */
s32 sd_cmd52_read(PSDIO_DATA psdio, u32 addr, u32 cnt, u8 *pdata)
{
	int err, i;
	struct sdio_func *func;
	bool claim_needed;

_func_enter_;

	err = 0;
	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
	err = _sd_cmd52_read(psdio, addr, cnt, pdata);
	if (claim_needed)
		sdio_release_host(func);

_func_exit_;

	return err;
}

/*
 * Return:
 *	0		Success
 *	others	Fail
 */
s32 _sd_cmd52_write(PSDIO_DATA psdio, u32 addr, u32 cnt, u8 *pdata)
{
	int err, i;
	struct sdio_func *func;

_func_enter_;

	err = 0;
	func = psdio->func;

	for (i = 0; i < cnt; i++) {
#ifdef SDIO_DEBUG_IO
		DBG_871X("%s: sdio_writeb addr=0x%05x val=0x%02x\n", __FUNCTION__, addr+i, pdata[i]);
#endif
		sdio_writeb(func, pdata[i], addr+i, &err);
		if (err) {
			DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x val=0x%02x\n", __func__, err, addr+i, pdata[i]);
			break;
		}
	}

_func_exit_;

	return err;
}

/*
 * Return:
 *	0		Success
 *	others	Fail
 */
s32 sd_cmd52_write(PSDIO_DATA psdio, u32 addr, u32 cnt, u8 *pdata)
{
	int err, i;
	struct sdio_func *func;
	bool claim_needed;

_func_enter_;

	err = 0;
	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
	err = _sd_cmd52_write(psdio, addr, cnt, pdata);
	if (claim_needed)
		sdio_release_host(func);

_func_exit_;

	return err;
}

u8 _sd_read8(PSDIO_DATA psdio, u32 addr, s32 *err)
{
	u8 v;
	struct sdio_func *func;

_func_enter_;

	func = psdio->func;

#ifdef SDIO_DEBUG_IO
	DBG_871X("%s: sdio_readb addr=0x%05x\n", __FUNCTION__, addr);
#endif
	v = sdio_readb(func, addr, err);

	if (err && *err)
		DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x\n", __func__, *err, addr);

_func_exit_;

	return v;
}

u8 sd_read8(PSDIO_DATA psdio, u32 addr, s32 *err)
{
	u8 v;
	struct sdio_func *func;
	bool claim_needed;

_func_enter_;

	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
#ifdef SDIO_DEBUG_IO
	DBG_871X("%s: sdio_readb addr=0x%05x\n", __FUNCTION__, addr);
#endif
	v = sdio_readb(func, addr, err);
	if (claim_needed)
		sdio_release_host(func);
	if (err && *err)
		DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x\n", __func__, *err, addr);

_func_exit_;

	return v;
}

u16 sd_read16(PSDIO_DATA psdio, u32 addr, s32 *err)
{
	u16 v;
	struct sdio_func *func;
	bool claim_needed;

_func_enter_;

	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
#ifdef SDIO_DEBUG_IO
	DBG_871X("%s: sdio_readw addr=0x%05x\n", __FUNCTION__, addr);
#endif
	v = sdio_readw(func, addr, err);
	if (claim_needed)
		sdio_release_host(func);
	if (err && *err)
		DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x\n", __func__, *err, addr);

_func_exit_;

	return  v;
}

u32 _sd_read32(PSDIO_DATA psdio, u32 addr, s32 *err)
{
	u32 v;
	struct sdio_func *func;

_func_enter_;

	func = psdio->func;

#ifdef SDIO_DEBUG_IO
	DBG_871X("%s: sdio_readl addr=0x%05x\n", __FUNCTION__, addr);
#endif
	v = sdio_readl(func, addr, err);

	if (err && *err)
	{
		int i;

		DBG_871X(KERN_ERR "%s: (%d) addr=0x%05x, val=0x%x\n", __func__, *err, addr, v);

		*err = 0;
		for(i=0; i<SD_IO_TRY_CNT; i++)
		{
			//sdio_claim_host(func);
#ifdef SDIO_DEBUG_IO
			DBG_871X("%s: sdio_readl addr=0x%05x\n", __FUNCTION__, addr);
#endif
			v = sdio_readl(func, addr, err);
			//sdio_release_host(func);
			if (*err == 0)
				break;
		}

		if (i==SD_IO_TRY_CNT)
			DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x, val=0x%x, try_cnt=%d\n", __func__, *err, addr, v, i);
		else
			DBG_871X(KERN_ERR "%s: (%d) addr=0x%05x, val=0x%x, try_cnt=%d\n", __func__, *err, addr, v, i);

	}

_func_exit_;

	return  v;
}

u32 sd_read32(PSDIO_DATA psdio, u32 addr, s32 *err)
{
	u32 v;
	struct sdio_func *func;
	bool claim_needed;

_func_enter_;

	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
#ifdef SDIO_DEBUG_IO
	DBG_871X("%s: sdio_readl addr=0x%05x\n", __FUNCTION__, addr);
#endif
	v = sdio_readl(func, addr, err);
	if (claim_needed)
		sdio_release_host(func);

	if (err && *err)
	{
		int i;

		DBG_871X(KERN_ERR "%s: (%d) addr=0x%05x, val=0x%x\n", __func__, *err, addr, v);

		*err = 0;
		for(i=0; i<SD_IO_TRY_CNT; i++)
		{
			if (claim_needed) sdio_claim_host(func);
#ifdef SDIO_DEBUG_IO
//			DBG_871X("%s: sdio_readl addr=0x%05x\n", __FUNCTION__, addr);
#endif
			v = sdio_readl(func, addr, err);
			if (claim_needed) sdio_release_host(func);
			if (*err == 0)
				break;
		}

		if (i==SD_IO_TRY_CNT)
			DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x, val=0x%x, try_cnt=%d\n", __func__, *err, addr, v, i);
		else
			DBG_871X(KERN_ERR "%s: (%d) addr=0x%05x, val=0x%x, try_cnt=%d\n", __func__, *err, addr, v, i);

	}

_func_exit_;

	return  v;
}

void sd_write8(PSDIO_DATA psdio, u32 addr, u8 v, s32 *err)
{
	struct sdio_func *func;
	bool claim_needed;

_func_enter_;

	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
#ifdef SDIO_DEBUG_IO
	DBG_871X("%s: sdio_writeb addr=0x%05x val=0x%02x\n", __FUNCTION__, addr, v);
#endif
	sdio_writeb(func, v, addr, err);
	if (claim_needed)
		sdio_release_host(func);
	if (err && *err)
		DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x val=0x%02x\n", __func__, *err, addr, v);

_func_exit_;
}

void sd_write16(PSDIO_DATA psdio, u32 addr, u16 v, s32 *err)
{
	struct sdio_func *func;
	bool claim_needed;

_func_enter_;

	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
#ifdef SDIO_DEBUG_IO
	DBG_871X("%s: sdio_writew addr=0x%05x val=0x%04x\n", __FUNCTION__, addr, v);
#endif
	sdio_writew(func, v, addr, err);
	if (claim_needed)
		sdio_release_host(func);
	if (err && *err)
		DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x val=0x%04x\n", __func__, *err, addr, v);

_func_exit_;
}

void _sd_write32(PSDIO_DATA psdio, u32 addr, u32 v, s32 *err)
{
	struct sdio_func *func;

_func_enter_;

	func = psdio->func;

#ifdef SDIO_DEBUG_IO
	DBG_871X("%s: sdio_writel addr=0x%05x val=0x%08x\n", __FUNCTION__, addr, v);
#endif
	sdio_writel(func, v, addr, err);

	if (err && *err)
	{
		int i;

		DBG_871X(KERN_ERR "%s: (%d) addr=0x%05x val=0x%08x\n", __func__, *err, addr, v);

		*err = 0;
		for(i=0; i<SD_IO_TRY_CNT; i++)
		{
//			DBG_871X("%s: sdio_writel addr=0x%05x val=0x%08x\n", __FUNCTION__, addr, v);
			sdio_writel(func, v, addr, err);
			if (*err == 0)
				break;
		}

		if (i==SD_IO_TRY_CNT)
			DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x val=0x%08x, try_cnt=%d\n", __func__, *err, addr, v, i);
		else
			DBG_871X(KERN_ERR "%s: (%d) addr=0x%05x val=0x%08x, try_cnt=%d\n", __func__, *err, addr, v, i);

	}

_func_exit_;
}

void sd_write32(PSDIO_DATA psdio, u32 addr, u32 v, s32 *err)
{
	struct sdio_func *func;
	bool claim_needed;

_func_enter_;

	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
#ifdef SDIO_DEBUG_IO
	DBG_871X("%s: sdio_writel addr=0x%05x val=0x%08x\n", __FUNCTION__, addr, v);
#endif
	sdio_writel(func, v, addr, err);
	if (claim_needed)
		sdio_release_host(func);

	if (err && *err)
	{
		int i;

		DBG_871X(KERN_ERR "%s: (%d) addr=0x%05x val=0x%08x\n", __func__, *err, addr, v);

		*err = 0;
		for(i=0; i<SD_IO_TRY_CNT; i++)
		{
			if (claim_needed) sdio_claim_host(func);
#ifdef SDIO_DEBUG_IO
//			DBG_871X("%s: sdio_writel addr=0x%05x val=0x%08x\n", __FUNCTION__, addr, v);
#endif
			sdio_writel(func, v, addr, err);
			if (claim_needed) sdio_release_host(func);
			if (*err == 0)
				break;
		}

		if (i==SD_IO_TRY_CNT)
			DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x val=0x%08x, try_cnt=%d\n", __func__, *err, addr, v, i);
		else
			DBG_871X(KERN_ERR "%s: (%d) addr=0x%05x val=0x%08x, try_cnt=%d\n", __func__, *err, addr, v, i);
	}

_func_exit_;
}

/*
 * Use CMD53 to read data from SDIO device.
 * This function MUST be called after sdio_claim_host() or
 * in SDIO ISR(host had been claimed).
 *
 * Parameters:
 *	psdio	pointer of SDIO_DATA
 *	addr	address to read
 *	cnt		amount to read
 *	pdata	pointer to put data, this should be a "DMA:able scratch buffer"!
 *
 * Return:
 *	0		Success
 *	others	Fail
 */
s32 _sd_read(PSDIO_DATA psdio, u32 addr, u32 cnt, void *pdata)
{
	int err;
	struct sdio_func *func;

_func_enter_;

	func = psdio->func;

	if (unlikely((cnt==1) || (cnt==2)))
	{
		int i;
		u8 *pbuf = (u8*)pdata;

		for (i = 0; i < cnt; i++)
		{
#ifdef SDIO_DEBUG_IO
			DBG_871X("%s: sdio_readb addr=0x%05x\n", __FUNCTION__, addr+i);
#endif
			*(pbuf+i) = sdio_readb(func, addr+i, &err);

			if (err) {
				DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x\n", __func__, err, addr);
				break;
			}
		}
		return err;
	}

#ifdef SDIO_DEBUG_IO
	DBG_871X("%s: sdio_memcpy_fromio addr=0x%05x cnt=%d\n", __FUNCTION__, addr, cnt);
#endif
	err = sdio_memcpy_fromio(func, pdata, addr, cnt);
	if (err) {
		DBG_871X(KERN_ERR "%s: FAIL(%d)! ADDR=%#x Size=%d\n", __func__, err, addr, cnt);
#if 0
0x04~0x7
 
0x100~0x103
 
0x200~0x203
0x204~0x207
0x210~0x213
 
0xf6
0x3a
0xc0
0x8a
 
sdio local 0x14~0x1d
#endif
		{
		u32 addr_dbg, val_dbg;
		addr_dbg = 0x10004;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10005;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10006;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10007;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);

		addr_dbg = 0x10100;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10101;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10102;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10103;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);

		addr_dbg = 0x10200;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10201;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10202;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10203;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10204;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10205;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10206;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10207;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);

		addr_dbg = 0x10210;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10211;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10212;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%02X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x10213;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%02X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);

		addr_dbg = 0x100f6;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x1003a;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x100c0;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%02X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		addr_dbg = 0x1008a;
		val_dbg = _sd_read8(psdio, addr_dbg, &err);
		DBG_871X("%s: 0x%02X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);

		for (addr_dbg = 0x14; addr_dbg <= 0x1d; addr_dbg++)
		{
			val_dbg = _sd_read8(psdio, addr_dbg, &err);
			DBG_871X("%s: 0x%05X=0x%02X\n", __FUNCTION__, addr_dbg, val_dbg);
		}
		}
	}

_func_exit_;

	return err;
}

/*
 * Use CMD53 to read data from SDIO device.
 *
 * Parameters:
 *	psdio	pointer of SDIO_DATA
 *	addr	address to read
 *	cnt		amount to read
 *	pdata	pointer to put data, this should be a "DMA:able scratch buffer"!
 *
 * Return:
 *	0		Success
 *	others	Fail
 */
s32 sd_read(PSDIO_DATA psdio, u32 addr, u32 cnt, void *pdata)
{
	s32 err;
	struct sdio_func *func;
	bool claim_needed;

	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
	err = _sd_read(psdio, addr, cnt, pdata);
	if (claim_needed)
		sdio_release_host(func);

	return err;
}

/*
 * Use CMD53 to write data to SDIO device.
 * This function MUST be called after sdio_claim_host() or
 * in SDIO ISR(host had been claimed).
 *
 * Parameters:
 *	psdio	pointer of SDIO_DATA
 *	addr	address to write
 *	cnt		amount to write
 *	pdata	data pointer, this should be a "DMA:able scratch buffer"!
 *
 * Return:
 *	0		Success
 *	others	Fail
 */
s32 _sd_write(PSDIO_DATA psdio, u32 addr, u32 cnt, void *pdata)
{
	int err;
	struct sdio_func *func;
	u32 size;

_func_enter_;

	func = psdio->func;
//	size = sdio_align_size(func, cnt);

	if (unlikely((cnt==1) || (cnt==2)))
	{
		int i;
		u8 *pbuf = (u8*)pdata;

		for (i = 0; i < cnt; i++)
		{
#ifdef SDIO_DEBUG_IO
			DBG_871X("%s: sdio_writeb addr=0x%05x val=0x%02x\n", __FUNCTION__, addr+i, pbuf[i]);
#endif
			sdio_writeb(func, *(pbuf+i), addr+i, &err);
			if (err) {
				DBG_871X(KERN_ERR "%s: FAIL!(%d) addr=0x%05x val=0x%02x\n", __func__, err, addr, *(pbuf+i));
				break;
			}
		}

		return err;
	}

	size = cnt;
#ifdef SDIO_DEBUG_IO
	DBG_871X("%s: sdio_memcpy_toio addr=0x%05x cnt=%d\n", __FUNCTION__, addr, cnt);
#endif
	err = sdio_memcpy_toio(func, addr, pdata, size);
	if (err) {
		DBG_871X(KERN_ERR "%s: FAIL(%d)! ADDR=%#x Size=%d(%d)\n", __func__, err, addr, cnt, size);
	}

_func_exit_;

	return err;
}

/*
 * Use CMD53 to write data to SDIO device.
 *
 * Parameters:
 *  psdio	pointer of SDIO_DATA
 *  addr	address to write
 *  cnt		amount to write
 *  pdata	data pointer, this should be a "DMA:able scratch buffer"!
 *
 * Return:
 *  0		Success
 *  others	Fail
 */
s32 sd_write(PSDIO_DATA psdio, u32 addr, u32 cnt, void *pdata)
{
	s32 err;
	struct sdio_func *func;
	bool claim_needed;

	func = psdio->func;
	claim_needed = rtw_sdio_claim_host_needed(func);

	if (claim_needed)
		sdio_claim_host(func);
	err = _sd_write(psdio, addr, cnt, pdata);
	if (claim_needed)
		sdio_release_host(func);

	return err;
}

