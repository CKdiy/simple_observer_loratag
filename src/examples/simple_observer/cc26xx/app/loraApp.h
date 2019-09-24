/******************************************************************************

 @file  lora.h

 @brief ...

 ******************************************************************************/
 

#ifndef LORAAPP_H
#define LORAAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "osal.h"
#include "sx1278_lora.h"
/*********************************************************************
 * CONSTANTS
 */
 
#define LORA_SADDR_LEN                      6 
#define LORA_SEND_LEN_MAX                   64
   
#define LORA_HW_STATUS_OK                  0x0  
#define LORA_HW_STATUS_ERROR               0x1  
   
#define LORA_RF_MODE_STANDBY               0x0  
#define LORA_RF_MODE_SLEEP                 0x1    
#define LORA_RF_MODE_RX                    0x2  
#define LORA_RF_MODE_TX                    0x3  
//  
//#define LORA_DEVICE_INIT_DONE_EVENT        0x00
//#define LORA_DEVICE_TX_DONE_EVENT          0x01
//#define LORA_DEVICE_RX_DONE_EVENT          0x02
//#define LORA_DEVICE_RX_TIMEOUT_EVENT       0x03  
//#define LORA_DEVICE_ERROR_EVENT            0x04 
   
#define LORA_TXPKT_FIX     		0xFE    
#define LORA_RXPKT_FIX     		0xBE   
/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */
enum
{    
	TYPE_LORA_BEACONINF_UP          = 0x00,   
};

typedef struct
{
	uint8_t hw;
	uint8_t status;
}loraRole_Status_t;

typedef union 
{
	struct
	{
	  	uint8_t		pkt_len    		:6;      // Bit5~ Bit0  Payload length
		uint8_t		pkt_type		:2;      // Bit7~ Bit6  Package type     
    }bit_t; 
	
	uint8_t payloadHead;
}payloadHead_t;

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*-------------------------------------------------------------------
 * Observer Profile Public APIs
 */
bStatus_t loraRole_SetRFMode( uint8_t param);
bStatus_t loraRole_GetRFMode( uint8_t *param);
bStatus_t loraRole_StartDevice( LoraRFStatusCB_t rfstatusCB, uint8_t *para);
bStatus_t loraRole_MacSend( uint8_t *payload, uint8_t len);
bStatus_t loraRole_MacRecv(void);
uint8_t loraRole_GetRand(void);
/*-------------------------------------------------------------------
 * TASK API - These functions must only be called by OSAL.
 */
/**
 * @brief       Task creation function for the lora.
 *
 * @param       none
 *
 * @return      none
 */

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* LORAAPP_H */
