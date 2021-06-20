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

static const ShellCommand shell_commands[] = {
	{NULL, NULL}
};

static const ShellConfig shell_cfg = {
	(BaseSequentialStream *)&EFI_SERIAL_SHELL_SD,
	shell_commands
};

systime_t last_print_tm = 0;

#endif

int main(void) {
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

	/**
	 * most basic MCU initialization - no configuration access, no external hardware access
	 */
	baseMCUInit();

	runRusEfi();
	return 0;
}

