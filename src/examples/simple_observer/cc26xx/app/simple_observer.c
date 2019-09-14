/******************************************************************************

 @file  simple_observer.c

 @brief This file contains the Simple BLE Observer sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2011-2018, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_02_25
 Release Date: 2018-04-02 18:03:35
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#include "observer.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "board_key.h"
#include "board.h"

#include "simple_observer.h"

#include "ibeaconinf.h"
#include "mems.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  20

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 800

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// Task configuration
#define SBO_TASK_PRIORITY                     1

#ifndef SBO_TASK_STACK_SIZE
#define SBO_TASK_STACK_SIZE                   660
#endif

// Internal Events for RTOS application
#define SBO_KEY_CHANGE_EVT                    0x0001
#define SBO_STATE_CHANGE_EVT                  0x0002
#define SBP_OBSERVER_PERIODIC_EVT             0x0004
#define SBO_MEMS_ACTIVE_EVT                   0x0008

#define RCOSC_CALIBRATION_PERIOD_1s           1000
#define RCOSC_CALIBRATION_PERIOD_3s           3000

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} sboEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to signal timeout
static Clock_Struct keyChangeClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sboTask;
Char sboTaskStack[SBO_TASK_STACK_SIZE];

// Number of scan results and scan result index
static uint8 scanRes;
static uint8 scanResult_write;
static uint8 scanResult_read;
static uint8 scanTimetick;

//mems
static memsmgr_t memsMgr;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];
static ibeaconInf_t ibeaconInfList[DEFAULT_MAX_SCAN_RES + 1];
static scanResult_t scanResultList[BUFFER_SCANRESULT_MAX_NUM];

static Clock_Struct userProcessClock;

//The UUID of the bluetooth beacon 
//const uint8_t ibeaconUuid[6]={0x20,0x19,0x01,0x10,0x09,0x31};
const uint8_t ibeaconUuid[6]={0xFD,0xA5,0x06,0x93,0xA4,0xE2};
//const uint8_t ibeaconUuid[6]={0xAB,0x81,0x90,0xD5,0xD1,0x1E};
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleBLEObserver_init(void);
static void SimpleBLEObserver_taskFxn(UArg a0, UArg a1);

static void SimpleBLEObserver_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLEObserver_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLEObserver_processAppMsg(sboEvt_t *pMsg);
static void SimpleBLEObserver_processRoleEvent(gapObserverRoleEvent_t *pEvent);
static void SimpleBLEObserver_addDeviceInfo(uint8 *pAddr, uint8 *pData,
                                            uint8 datalen, uint8 rssi);

static uint8_t SimpleBLEObserver_eventCB(gapObserverRoleEvent_t *pEvent);

static uint8_t SimpleBLEObserver_enqueueMsg(uint8_t event, uint8_t status,
                                            uint8_t *pData);

void SimpleBLEObserver_initKeys(void);

void SimpleBLEObserver_keyChangeHandler(uint8 keys);
static void SimpleBLEObserver_userClockHandler(UArg arg);

static uint8_t sort_ibeaconInf_By_Rssi(void);
static bool UserProcess_MemsInterrupt_Mgr( uint8_t status );
void SimpleBLEObserver_memsActiveHandler(uint8 pins);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapObserverRoleCB_t simpleBLERoleCB =
{
  SimpleBLEObserver_eventCB  // Event callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEObserver_createTask
 *
 * @brief   Task creation function for the Simple BLE Observer.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLEObserver_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sboTaskStack;
  taskParams.stackSize = SBO_TASK_STACK_SIZE;
  taskParams.priority = SBO_TASK_PRIORITY;

  Task_construct(&sboTask, SimpleBLEObserver_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEObserver_init
 *
 * @brief   Initialization function for the Simple BLE Observer App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLEObserver_init(void)
{
	// ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the DB Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0x44, 0x44, 0x44, 0x44, 0x44, 0x44 };
  //HCI_EXT_SetBDADDRCmd(bdAddress);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  Board_initKeys(SimpleBLEObserver_keyChangeHandler);
  
  if( MemsOpen() )
  {
	 memsMgr.status = MEMS_ACTIVE;
	 memsMgr.old_tick = Clock_getTicks();
	 memsMgr.new_tick = memsMgr.old_tick;
	 memsMgr.interval = 0;
    UserProcess_MemsInterrupt_Mgr( ENABLE );  
  }

  // Setup Observer Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPObserverRole_SetParameter(GAPOBSERVERROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                 &scanRes );
  }

  // Setup GAP
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);

  // Start the Device
  VOID GAPObserverRole_StartDevice((gapObserverRoleCB_t *)&simpleBLERoleCB);

  Util_constructClock(&userProcessClock, SimpleBLEObserver_userClockHandler,
                          RCOSC_CALIBRATION_PERIOD_1s, 0, false, SBP_OBSERVER_PERIODIC_EVT);
  
  Util_startClock(&userProcessClock);
}

/*********************************************************************
 * @fn      SimpleBLEObserver_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Observer.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEObserver_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEObserver_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBLEObserver_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    // If RTOS queue is not empty, process app message
    while (!Queue_empty(appMsgQueue))
    {
      sboEvt_t *pMsg = (sboEvt_t *)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message
        SimpleBLEObserver_processAppMsg(pMsg);

        // Free the space from the message
        ICall_free(pMsg);
      }
    }
	
	if (events & SBP_OBSERVER_PERIODIC_EVT)
	{	
		events &= ~SBP_OBSERVER_PERIODIC_EVT;
		
		memsMgr.new_tick = Clock_getTicks();
		
		if( (  MEMS_ACTIVE == memsMgr.status ) && 
		    ( memsMgr.new_tick - memsMgr.old_tick < NOACTIVE_TIME_OF_DURATION ) )
		{
		    // Perform periodic application task
		    GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                        DEFAULT_DISCOVERY_WHITE_LIST );	
			
		    Util_startClock(&userProcessClock);
		}
		else if( ( MEMS_ACTIVE == memsMgr.status ) && 
			     ( memsMgr.new_tick - memsMgr.old_tick >= NOACTIVE_TIME_OF_DURATION ) )
		{
			memsMgr.status = MEMS_SLEEP;	
			
			Util_restartClock(&userProcessClock, RCOSC_CALIBRATION_PERIOD_3s);	
		}
		else if( (MEMS_SLEEP == memsMgr.status ) &&
			     ( memsMgr.interval < 2) )
		{
		    if( memsMgr.new_tick - memsMgr.old_tick < ACTIVE_TIME_OF_DURATION) //3s
			    memsMgr.interval ++; 
			else
			    memsMgr.interval = 0;
			
			if( memsMgr.interval >= 2 )
			{
			    memsMgr.interval = 0;
				
			    memsMgr.status = MEMS_ACTIVE;
			
			    Util_restartClock(&userProcessClock, RCOSC_CALIBRATION_PERIOD_1s);
				
			    memsMgr.old_tick = memsMgr.new_tick + COMPENSATOR_TICK_500ms;
			}
			else
			{
			    Util_startClock(&userProcessClock);
			}
		}
		else
		{
			memsMgr.interval = 0;
		}
	}		
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLEObserver_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleBLEObserver_processRoleEvent((gapObserverRoleEvent_t *)pMsg);
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEObserver_processAppMsg(sboEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBO_STATE_CHANGE_EVT:
      SimpleBLEObserver_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case SBO_KEY_CHANGE_EVT:
      SimpleBLEObserver_handleKeys(0, pMsg->hdr.state);
      break;
	  
    case SBO_MEMS_ACTIVE_EVT:
	  memsMgr.old_tick = Clock_getTicks();

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SimpleBLEObserver_handleKeys(uint8 shift, uint8 keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_UP)
  {
    return;
  }

  if (keys & KEY_LEFT)
  {
    return;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_processRoleEvent
 *
 * @brief   Observer role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEObserver_processRoleEvent(gapObserverRoleEvent_t *pEvent)
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
		GAPObserverRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
									   DEFAULT_DISCOVERY_ACTIVE_SCAN,
									   DEFAULT_DISCOVERY_WHITE_LIST);
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
		  if(pEvent->deviceInfo.eventType != GAP_ADRPT_SCAN_RSP)
		  {
			  SimpleBLEObserver_addDeviceInfo(pEvent->deviceInfo.addr,
                                              pEvent->deviceInfo.pEvtData,
                                              pEvent->deviceInfo.dataLen,
                                              pEvent->deviceInfo.rssi);
		  }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
		   // discovery complete
		   scanTimetick ++;	
		  
		   GAPObserverRole_CancelDiscovery();
		  
		   scanResultList[scanResult_write].timertick = scanTimetick;
		   scanResultList[scanResult_write].numdevs = sort_ibeaconInf_By_Rssi();
		   memcpy( &scanResultList[scanResult_write].bleinfbuff, ibeaconInfList, 
					sizeof(ibeaconInf_t)*scanResultList[scanResult_write].numdevs);
		  
		  scanResult_write ++;
		  if( scanResult_write >= BUFFER_SCANRESULT_MAX_NUM )
			scanResult_write = 0;
		  
		  scanRes = 0;
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_eventCB
 *
 * @brief   Observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLEObserver_eventCB(gapObserverRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleBLEObserver_enqueueMsg(SBO_STATE_CHANGE_EVT,
                                   SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}


/*********************************************************************
 * @fn      SimpleBLEObserver_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLEObserver_addDeviceInfo(uint8 *pAddr, uint8 *pData, uint8 datalen, uint8 rssi)
{
  uint8 i;
  uint8 minoroffset;
  uint8 uuidoffset;
  uint8 scannum;
  uint8 *ptr;

  if( (pAddr == NULL) || (pData == NULL) )
	return;
  
  if( IBEACON_ADVDATA_LEN != datalen)
	return;
  
  ptr = pData;
  uuidoffset = IBEACON_ADVUUID_OFFSET;
  
  if( memcmp(&ptr[uuidoffset], ibeaconUuid, sizeof(ibeaconUuid)) != 0)
	return;  
  
  scannum = scanRes;
  // If result count not at max
  if ( scannum < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < scanRes; i++ )
    {
      if (memcmp(pAddr, devList[i].addr, B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy( devList[scannum].addr, pAddr, B_ADDR_LEN );
    
    minoroffset = IBEACON_ADVMINOR_OFFSET;
    memcpy((void *)(&ibeaconInfList[scannum].minor[0]), 
		   (void *)&ptr[minoroffset], sizeof(uint16));	
	
    ibeaconInfList[scannum].rssi = 0xFF - rssi;

    // Increment scan result count
    scanRes ++;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys pressed
 *
 * @return  none
 */
void SimpleBLEObserver_keyChangeHandler(uint8 keys)
{
  SimpleBLEObserver_enqueueMsg(SBO_KEY_CHANGE_EVT, keys, NULL);
}

void SimpleBLEObserver_memsActiveHandler(uint8 pins)
{
  SimpleBLEObserver_enqueueMsg(SBO_MEMS_ACTIVE_EVT, pins, NULL);
  
  Semaphore_post(sem);
}

static void SimpleBLEObserver_userClockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
  
  PowerCC26XX_injectCalibration(); 
}

/*********************************************************************
 * @fn      SimpleBLEObserver_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLEObserver_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sboEvt_t *pMsg;

  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(sboEvt_t)))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }

  return FALSE;
}

static uint8_t sort_ibeaconInf_By_Rssi(void)
{
  uint8_t i=0,j=0;
  uint8 scannum;
  
  scannum = scanRes;

  for(i=0; i<scannum - 1; i++)
  {
	for(j=0; j<scannum-i-1; j++)
	{
		if(ibeaconInfList[j].rssi > ibeaconInfList[j+1].rssi)
		{
			memcpy(&ibeaconInfList[scannum], &ibeaconInfList[j], sizeof(ibeaconInf_t));
			memcpy(&ibeaconInfList[j], &ibeaconInfList[j+1], sizeof(ibeaconInf_t));
			memcpy(&ibeaconInfList[j+1], &ibeaconInfList[scannum], sizeof(ibeaconInf_t));															
		}												
	}				
  }
  
  if( scannum > BUFFER_IBEACONINF_NUM)
    scannum = BUFFER_IBEACONINF_NUM;	
  
  return scannum;
}

static bool UserProcess_MemsInterrupt_Mgr( uint8_t status )
{
    bool res = FALSE;
	
  	if( status == DISABLE )
	{
		Mems_ActivePin_Disable();
		memsMgr.interruptE = DISABLE;
		res = TRUE;
	}
	else
	{
		res = Mems_ActivePin_Enable(SimpleBLEObserver_memsActiveHandler);
		if(!res)
		{
			HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);							
		}
		
		memsMgr.interruptE = ENABLE;
	}
	
    return res;
}
/*********************************************************************
*********************************************************************/
