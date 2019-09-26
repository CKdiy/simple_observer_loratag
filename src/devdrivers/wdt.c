/*
  Filename:       wdt.c
  Revised:        $Date: 2019-03-13  $
  Revision:       $Revision:  $
 */
#ifdef IWDG_ENABLE

#include <xdc/std.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Error.h>
#include <string.h>
#include "Board.h"

#include <ti/drivers/Watchdog.h>
#include <inc/hw_wdt.h>

#include "wdt.h"

//Watchdog_Params params;
Watchdog_Handle watchdog;

void wdtClear(void)
{
	Watchdog_clear(watchdog);
}

void wdtCallback(UArg handle) 
{
	Watchdog_clear((Watchdog_Handle)handle);
}

void wdtInitFxn(void) 
{
	Watchdog_Params wp;
	
	Watchdog_init();
	Watchdog_Params_init(&wp);
	wp.callbackFxn    = (Watchdog_Callback)wdtCallback;
	wp.debugStallMode = Watchdog_DEBUG_STALL_ON;
	wp.resetMode      = Watchdog_RESET_ON;
 
	watchdog = Watchdog_open(CC2650_WATCHDOG0, &wp);
	Watchdog_setReload(watchdog, 1500000); // 1sec (WDT runs always at 48MHz/32)
}

#endif 
