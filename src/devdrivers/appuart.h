/**************************************************************************************************
  Filename:       hal_uart.h
  Revised:        $Date: 2019-03-13  $
  Revision:       $Revision:   $

**************************************************************************************************/

#ifndef DEMO

#ifndef APPUART_H
#define APPUART_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <ti/drivers/UART.h>
/*********************************************************************
 * DEFINE
 */

#define UART0_RECEIVE_BUFF_SIZE 1<<4
#define RX_BUFF_SIZE 16
  
#define DEFAULT_UART_AT_TEST_LEN              4
#define DEFAULT_UART_AT_MAC_LEN               8   
#define DEFAULT_UART_AT_CMD_LEN               10
#define DEFAULT_UART_AT_RSP_LEN               6
/*********************************************************************
 * MACROS
 */
  

/*********************************************************************
 * FUNCTIONS
 */
void Close_uart0(void);
bool Open_uart0(UART_Callback appuartCB);
bool Uart0_Write(const uint8_t *wbuf, size_t wlen);
uint8_t strCompara( uint8_t *str1, uint8_t *str2, uint8_t len);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_H */
#endif