/*
 * snv.c
 * Description: nvram managerment for system
 */

#include <string.h>
#include "osal_snv.h"
#include "snv.h"
#include "bcomdef.h"

static const block_info_t fb[] =
{
    /* block ID (must < 0x8f)    size */
	{USER_NVID_DEVINF_START,       USER_INF_BLOCK_SIZE},
};

#define FB_SIZE         (sizeof(fb)/sizeof(block_info_t))

uint8_t nvBuf[USER_INF_BLOCK_SIZE] = {0};

static int Nvram_Block_Init(uint8_t nvid, uint8_t len);
static int Nvram_Block_Check(uint8_t nvid, uint8_t len);
static int DevInf_Snv_Init(uint8_t len);

/* Init nvram */
void Nvram_Init(void)
{  
	 uint8_t i;
	 
	 for(i=0; i<FB_SIZE; i++)
	 {
    	if (Nvram_Block_Check(fb[i].NVid, fb[i].size) != 0)
		{
			Nvram_Block_Init(fb[i].NVid, fb[i].size);
		}
	 }
}

static int Nvram_Block_Check(uint8_t nvid, uint8_t len)
{
    uint32_t	crc = 0;
	uint8_t		status;
	
    if(nvid > BLE_NVID_CUST_END)
	 	return -1;
	
	memset((void *)nvBuf, 0, sizeof(nvBuf));
	
	status = osal_snv_read(nvid, len, (void *)nvBuf);
	if(status != SUCCESS)
	 	return -1;
	
	memcpy((void *)&crc, nvBuf, sizeof(crc));
    if (crc != crc32(0, (uint8_t *)&nvBuf[sizeof(crc)], len - sizeof(crc)))
        return -1;
    
    return 0;
}

static int Nvram_Block_Init(uint8_t nvid, uint8_t len)
{	
	if(nvid > BLE_NVID_CUST_END)
	 	return -1;
	
	if(USER_NVID_DEVINF_START == nvid)
	{
		return DevInf_Snv_Init(len);
	}
	 
    return 0;
}

static int DevInf_Snv_Init(uint8_t len)
{
 	uint8_t		status;
	snv_device_inf_t *ptr = (snv_device_inf_t *)nvBuf;
	
	memset((void *)nvBuf, 0, sizeof(nvBuf));
	
	/* LORA Para */
	ptr->loraPara.bit_t.channel = 0;
	ptr->loraPara.bit_t.power   = 1;
	ptr->loraPara.bit_t.rate    = 1;
	
	/* BLE Para */
    ptr->blescan_time = 8;
	
	ptr->atflag       = 0;
	
	ptr->crc32 = crc32(0, (uint8_t *)(&ptr->loraPara), len - sizeof(uint32_t));
	
	status = osal_snv_write(USER_NVID_DEVINF_START, len,  ptr);
	if(status != SUCCESS)
		return -1;
	
	return 0;
}

int Nvram_ReadNv_Inf(uint8_t nvid, uint8_t *readbuf)
{
	uint8_t		status;
	uint8_t     i;
	uint8_t     len;
	uint32_t	crc = 0;
	
	if(nvid > BLE_NVID_CUST_END)
		return -1;
	
	for(i=0; i<FB_SIZE; i++)
	{
		if(fb[i].NVid == nvid)
			len =  fb[i].size; 
	}
	
	status = osal_snv_read(nvid, len, (void *)nvBuf);	
	if(status != SUCCESS)
		return -1;
	
	memcpy(&crc, nvBuf, sizeof(crc));
	if( crc != crc32(0, (void *)(nvBuf+sizeof(crc)), len - sizeof(uint32_t)))
		return -1;
	
	memcpy(readbuf, nvBuf+sizeof(crc), len - sizeof(uint32_t));
	return 0;
}

int Nvram_WriteNv_Inf(uint8_t nvid, uint8_t *writebuf)
{
	uint8_t		status;
	uint8_t     i;
	uint8_t     len;
	uint32_t	crc = 0;    
	
	if(nvid > BLE_NVID_CUST_END)
		return -1;
	
	for(i=0; i<FB_SIZE; i++)
	{
		if(fb[i].NVid == nvid)
			len =  fb[i].size; 
	}
	
	memset((void *)nvBuf, 0, sizeof(nvBuf));
	
	memcpy((void *)(nvBuf+sizeof(crc)), writebuf, len-sizeof(crc));
	
	crc = crc32(0, (void *)(nvBuf+sizeof(crc)), len - sizeof(crc));
	memcpy((void *)nvBuf, (void *)&crc, sizeof(crc));
	  
	status = osal_snv_write(nvid, len, (void *)nvBuf);	
	if(status != SUCCESS)
		return -1;
	
	return 0;
}