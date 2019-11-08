/**************************************************************************************************
  Filename:       sx1278.h
  Revised:        $Date: 2018-10-25  $
  Revision:       $Revision:   $

**************************************************************************************************/

#ifndef SX1278_HAL_H
#define SX1278_HAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <inc/hw_ints.h>
#include <inc/hw_types.h>

/*********************************************************************
 * CONSTANTS
 */
 
/*********************************************************************
 * MACROS
 */
  
typedef void (*LoraRFStatusCB_t)(uint8_t pinID);

/*********************************************************************
 * FUNCTIONS
 */
void Close_sx1278_SPI(void);   
bool Open_sx1278_SPI(void);
bool sx1278_SPI_Read(const uint8_t *rxbuf, size_t rxlen);
bool sx1278_SPI_Write(const uint8_t *txbuf, size_t txlen); 
void sx1278_SPI_CSN(bool enable);
uint8_t Read_sx1278Dio0_Pin(void);
bool sx1278_Reset(void);
void sx1278_Pin_OutLowPower(void);
void sx1278_Pin_SetLowPower(void);
bool sx1278_StatusPin_Enable(LoraRFStatusCB_t loraRFStatusCB , uint8_t powerFlg);
void sx1278_StatusPin_Disable(void);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SX1278_H */
