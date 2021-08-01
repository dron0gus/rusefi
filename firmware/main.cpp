/**
 * @file	main.cpp
 * @brief C++ main entry point
 *
 * @date Nov 29, 2012
 * @author Andrey Belomutskiy, (c) 2012-2020
 *      http://rusefi.com/
 */

#include "global.h"
#include "os_access.h"
#include "rusefi.h"
#include "mpu_util.h"
#include "version_check.h"

#ifdef EFI_SERIAL_SHELL_SD
#include "shell.h"

static const SerialConfig shell_sd_config =
{
	.speed	= 115200,
	.cr1	= 0,
	.cr2	= USART_CR2_STOP1_BITS,
	.cr3	= 0
};

#define SHELL_WA_SIZE	THD_WORKING_AREA_SIZE(2048)
static THD_WORKING_AREA(shellStack, SHELL_WA_SIZE);

extern "C" {
	void rusefi_trace_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
	void rusefi_trace_trigger(BaseSequentialStream *chp, int argc, char *argv[]);
}

static const ShellCommand shell_commands[] = {
	{"trace", rusefi_trace_cmd},
	{"trigger", rusefi_trace_trigger},
	{NULL, NULL}
};

static const ShellConfig shell_cfg = {
	(BaseSequentialStream *)&EFI_SERIAL_SHELL_SD,
	shell_commands
};

systime_t last_print_tm = 0;

#endif

void mpu_debug(BaseSequentialStream *chp)
{
	int i;

	/* protect second banck of flash */
	ARM_MPU_SetRegionEx(6,
		0x08100000,
		(1 << 28) | /* no instruction fetching */
		(0 << 24) | /* no access */
		(0 << 19) | /* TEX */
		(0 << 17) | /* C */
		(0 << 16) | /* B */
		(0 << 8)  | /* enable all sub-regs */
		(ARM_MPU_REGION_SIZE_1MB << 1) | /* size */
		(1 << 0)  | /* Region enable */
		0);
	/* protect information bock */
	ARM_MPU_SetRegionEx(5,
		0x1FF00000,
		(1 << 28) | /* no instruction fetching */
		(0 << 24) | /* no access */
		(0 << 19) | /* TEX */
		(0 << 17) | /* C */
		(0 << 16) | /* B */
		(0 << 8)  | /* enable all sub-regs */
		(ARM_MPU_REGION_SIZE_64KB << 1) | /* size */
		(1 << 0)  | /* Region enable */
		0);

	for (i = 0; i < 10; i++)
		chprintf(chp, "Fu\r\n");
	chprintf(chp, "MPU type: 0x%08x, ctrl 0x%08x\r\n", MPU->TYPE, MPU->CTRL);
	for (i = 0; i < 8; i++) {
		MPU->RNR = i;

		chprintf(chp, "MPU%02d: RBAR 0x%08x RASR 0x%08x\r\n",
			i, MPU->RBAR, MPU->RASR);
	}
}

int main(void) {
	/* Disable ITCM */
	SCB->ITCMCR = 0;
	__DSB();

	/*
	 * ChibiOS/RT initialization
	 */
	halInit();
	chSysInit();

#ifdef EFI_SERIAL_SHELL_SD
	/*
	 * Shell manager initialization.
	 */
	shellInit();
	/*
	 * Activates the serial driver 6 using the driver default configuration.
	 */
	sdStart(&EFI_SERIAL_SHELL_SD, &shell_sd_config);

	dbgprintf("Wellcome to RusEFI serial console\r\nType 'help' for list of support commands\r\n\r\n");

	chThdCreateStatic(shellStack, sizeof(shellStack), NORMALPRIO + 1, shellThread, (void *)&shell_cfg);
#endif

	mpu_debug((BaseSequentialStream *)&EFI_SERIAL_SHELL_SD);

	/**
	 * most basic MCU initialization - no configuration access, no external hardware access
	 */
	baseMCUInit();

	runRusEfi();
	return 0;
}

