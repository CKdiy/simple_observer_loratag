
#ifndef IBEACONINF_H
#define IBEACONINF_H

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"

/*********************************************************************
 * CONSTANTS
 */
#define IBEACON_ADVDATA_LEN       30 
#define IBEACON_ADVUUID_OFFSET    9
#define IBEACON_ADVMINOR_OFFSET   27
  
#define BUFFER_IBEACONINF_NUM     4  
#define BUFFER_SCANRESULT_MAX_NUM 16
/*********************************************************************
 * MACROS
 */

typedef struct
{
	uint8_t rssi;	     	
	uint8_t minor[2];
}ibeaconInf_t;

typedef struct
{
    uint8_t timertick;
	uint8_t numdevs;
	ibeaconInf_t bleinfbuff[BUFFER_IBEACONINF_NUM]; 
}scanResult_t;

#endif 