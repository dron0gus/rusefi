/**
 * @file  port_flash.cpp
 *
 * @date Jun 18, 2025
 * @author Andrey Gusakov
 */

#include "pch.h"

#if !defined(EFI_BOOTLOADER) && (EFI_STORAGE_MFS == TRUE)

#include "hal_mfs.h"

/* 1Mb device, single bank */
static const MFSConfig mfscfg_1m_1b = {
	.flashp           = (BaseFlash *)&EFLD1,
	.erased           = 0xFFFFFFFFU,
	.bank_size        = 256 * 1024U,
	.bank0_start      = 6U,
	.bank0_sectors    = 1U,
	.bank1_start      = 7U,
	.bank1_sectors    = 1U
};

/* 1Mb device, dual bank */
static const MFSConfig mfscfg_1m_2b = {
	.flashp           = (BaseFlash *)&EFLD1,
	.erased           = 0xFFFFFFFFU,
	.bank_size        = 128U * 1024U,
	.bank0_start      = 8 + 6U,
	.bank0_sectors    = 1U,
	.bank1_start      = 8 + 7U,
	.bank1_sectors    = 1U
};

/* 2Mb device, single bank */
static const MFSConfig mfscfg_2m_1b = {
	.flashp           = (BaseFlash *)&EFLD1,
	.erased           = 0xFFFFFFFFU,
	.bank_size        = 256 * 1024U,
	.bank0_start      = 10U,
	.bank0_sectors    = 1U,
	.bank1_start      = 11U,
	.bank1_sectors    = 1U
};

/* 2Mb device, dual bank */
static const MFSConfig mfscfg_2m_2b = {
	.flashp           = (BaseFlash *)&EFLD1,
	.erased           = 0xFFFFFFFFU,
	.bank_size        = 128 * 1024U,
	.bank0_start      = 12 + 10U,
	.bank0_sectors    = 1U,
	.bank1_start      = 12 + 11U,
	.bank1_sectors    = 1U
};

static const MFSConfig *mfscfgs[2][2] = {
	{&mfscfg_1m_1b, &mfscfg_1m_2b},
	{&mfscfg_2m_1b, &mfscfg_2m_2b}
};

PUBLIC_API_WEAK void portInitMfs()
{
	/* Starting EFL driver.*/
	eflStart(&EFLD1, NULL);
}

const MFSConfig *portGetMfsConfig()
{
	return mfscfgs[flashSize() >= 2048][isDualBank()];
}

#endif
