/*
 * snv.h
 */
#ifndef __SNV_H__
#define __SNV_H__

#include <xdc/std.h>
#include "crc.h"

/* Customer NV Items - Range 0x80 - 0x8F - This must match the number of Bonding entries */
#define USER_NVID_CUST_START 0x80 //!< Start of the Customer's NV IDs

#define USER_NVID_DEVINF_START 	USER_NVID_CUST_START 

#define USER_INF_BLOCK_SIZE   10

typedef struct
{
    uint8_t NVid;
    uint8_t size;
} block_info_t;

/**
 *  lora union
 */
typedef union 
{
	struct
	{    
		uint8_t		channel   		:8;      // Bit15~Bit8 channel
		uint8_t     power           :4;      // Bit3~Bit0  Lora Power 
		uint8_t     rate            :4;      // Bit7~Bit4  rate
	}bit_t;
	
	uint16_t    loraPara;
}lora_Para_N;

typedef struct
{
    uint32_t            crc32;
	lora_Para_N			loraPara;
	uint8_t             blescan_time;		 //1~9
	uint8_t             atflag;
}snv_device_inf_t;

void Nvram_Init(void);
int Nvram_ReadNv_Inf(uint8_t nvid, uint8_t *readbuf);
int Nvram_WriteNv_Inf(uint8_t nvid, uint8_t *writebuf);

#endif  /* SNV_H */