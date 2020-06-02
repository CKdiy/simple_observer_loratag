/******************************************************************************

 @file  lora.c

 @brief Lora role for RTOS Applications

 ******************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/knl/Clock.h>
   
#include <inc/hw_types.h>
#include <inc/hw_fcfg1.h>
#include "board.h"
   
#include "loraApp.h"
#include "ibeaconinf.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

static uint8_t dev_Saddr[LORA_SADDR_LEN];
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

loraRole_Status_t loraRole_Status;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

//void loraRole_SaddrGet( uint8_t *mac_addr);
bStatus_t loraRole_SetRFMode( uint8_t param);
bStatus_t loraRole_GetRFMode( uint8_t *param);
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

bStatus_t loraRole_SetRFMode( uint8_t param)
{
	bStatus_t ret = SUCCESS;
	bool      E_Ok = TRUE;
    
	if( LORA_HW_STATUS_OK != loraRole_Status.hw )
	  return FAILURE;
	  
	loraRole_Status.status = sx1278_GetStatus();
	if( (loraRole_Status.status == LORA_RF_MODE_SLEEP) && 
	    ( param !=  LORA_RF_MODE_SLEEP ) )
	{
		sx1278_Pin_OutLowPower();
	}
	else if( loraRole_Status.status ==  param)
	{
		E_Ok = FALSE;
	}
	
	if( E_Ok )
	{
	  switch( param )
	  {
		  case LORA_RF_MODE_STANDBY:
		    sx1278_SetStandby();		
		    break;

	      case LORA_RF_MODE_SLEEP:
			sx1278_SetStandby();
			sx1278DelayMs();
		    sx1278_SetSleep();
			sx1278_Pin_SetLowPower();
		    break;
		  
		  case LORA_RF_MODE_RX:
		    sx1278_EnterRx();
		    break;
		  
		  default: ret = FAILURE;
		    break;	
	  }
	}
	else
	{
	  ret = FAILURE;
	}
	
	return ret;
}

bStatus_t loraRole_GetRFMode( uint8_t *param)
{
	bStatus_t ret = SUCCESS;	
	
	if( LORA_HW_STATUS_OK != loraRole_Status.hw )
	  return FAILURE;
	
    loraRole_Status.status = sx1278_GetStatus();	
	switch( loraRole_Status.status )
	{
		case RF_STANDBY:
		  *param = LORA_RF_MODE_STANDBY;
		  break;
		  
		case RF_SLEEP:  
		  *param = LORA_RF_MODE_SLEEP;
		  break;
		  
		case RF_RX_RUNNING: 
		  *param = LORA_RF_MODE_RX;
		  break;
		  
		case RF_TX_RUNNING: 
		  *param = LORA_RF_MODE_TX;
		  break;
		  
		default: ret = FAILURE;
		  break;		  
	}
	
	return ret;
}

bStatus_t loraRole_StartDevice(LoraRFStatusCB_t rfstatusCB, uint8_t *para)
{
  bStatus_t ret = SUCCESS;
  
  if( rfstatusCB != NULL)
  {	
  	if( sx1278Init() )
  	{
  		sx1278_SetLoraPara( (LoRaSettings_t *)para );
	
		sx1278_StatusPin_Enable(rfstatusCB, TRUE);
	
		loraRole_Status.hw = LORA_HW_STATUS_OK;
		
		loraRole_SaddrGet( dev_Saddr );
		
		loraRole_SetRFMode( LORA_RF_MODE_SLEEP );
  	}
  	else
  	{
  		ret = FAILURE;
		loraRole_Status.hw = LORA_HW_STATUS_ERROR;
  	}
  }
  else
  {
  	ret = FAILURE;
  }
	 
  return ret;
}

bStatus_t loraRole_MacSend( uint8_t *payload, uint8_t len, uint8_t status)
{
  bStatus_t ret = SUCCESS;
  
  uint8_t crc_and;
  uint8_t *ptr;
  uint8_t i;
  
  payloadHead_t payload_Head;
	
  uint8_t *txbuf = (uint8_t *)ICall_malloc(LORA_SEND_LEN_MAX); 
  
  if( txbuf != NULL)
	 ptr = txbuf;
  else
	return FAILURE;
  
  crc_and = 0;
  *ptr++ = LORA_TXPKT_FIX;
  
  payload_Head.bit_t.pkt_type = TYPE_LORA_BEACONINF_UP;
  payload_Head.bit_t.pkt_len  = len + LORA_SADDR_LEN;
  *ptr++ = payload_Head.payloadHead;
  
  if( status )
  	dev_Saddr[0] |= 1<<7;
  else
	dev_Saddr[0] &= ~(1<<7);
  
  memcpy( ptr, dev_Saddr, LORA_SADDR_LEN);
  ptr+= LORA_SADDR_LEN;

  memcpy( ptr, payload, len);
  ptr+= len;
  
  for(i = 0; i< ptr - txbuf - 1; i++)
	crc_and += *( txbuf + i + 1);
  
  *ptr++ = crc_and;
   
  sx1278_SendBuf( txbuf, ptr - txbuf);

  ICall_free(txbuf);
  
  return ret;
}

bStatus_t loraRole_MacRecv(void)
{
  bStatus_t ret = SUCCESS;
  
  uint8_t *ptr;
  uint8_t rx_size;
  uint8_t crc_and;
  uint8_t i;
  uint8_t index;
  
  payloadHead_t payload_Head;
  
  ptr = sx1278_ReadRxPkt(&rx_size);
  
  if( ptr == NULL)
	return FAILURE;
  
  i = 0;
  while(i < rx_size)
  {
    if(*(ptr + i) == LORA_RXPKT_FIX)
	  break;
	else
	  i ++;
  }
  
  index = i + sizeof(uint8_t);
  payload_Head.payloadHead = *(ptr + index);
  
  if( (rx_size - index) < payload_Head.bit_t.pkt_len + sizeof(crc_and))
	return FAILURE;
  
  crc_and = 0;
  for(i=0; i< payload_Head.bit_t.pkt_len + sizeof(payloadHead_t); i++)
    crc_and += *(ptr + index + i);
  
  if( crc_and != *(ptr + index + sizeof(payloadHead_t) + payload_Head.bit_t.pkt_len) )
	return FAILURE;
  
//  if( memcmp((void *)(ptr + index + sizeof(payloadHead_t)), dev_Saddr, LORA_SADDR_LEN) != 0)
//	return FAILURE;
  
  if( memcmp((void *)(ptr + index + sizeof(payloadHead_t) + sizeof(uint8_t)), &dev_Saddr[1], LORA_SADDR_LEN - 1) != 0)
	return FAILURE;  
	  
  switch( payload_Head.bit_t.pkt_type )
  {
	case TYPE_LORA_BEACONINF_UP :
	  break;
		  
    default:
      break;	  
  }
  
  return ret; 
}

void loraRole_SaddrGet( uint8_t *mac_addr)
{
	uint32_t mac0 = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_0);    
	uint32_t mac1 = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_1);    
   
	*mac_addr++ = HI_UINT16(mac1) & (~(1<<7));  
	*mac_addr++ = LO_UINT16(mac1);  
	*mac_addr++ = BREAK_UINT32(mac0, 3);  
	*mac_addr++ = BREAK_UINT32(mac0, 2);  
	*mac_addr++ = BREAK_UINT32(mac0, 1);  
	*mac_addr++ = BREAK_UINT32(mac0, 0); 
	
	
}

static uint32_t s_rand;
uint8_t loraRole_GetRand(void)
{
    int res;
	uint8_t seed;
	
	s_rand += dev_Saddr[LORA_SADDR_LEN - 1];
	s_rand += Clock_getTicks();
	
	srand(s_rand);
	res = rand() + 1;	
	s_rand = ((res << 8) | (res >> 8)) >> 16;
	seed =  s_rand % 20; 
	  
    return seed;
}

/*********************************************************************
*********************************************************************/
