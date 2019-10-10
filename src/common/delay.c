
#include "delay.h"
#include <cpu.h>

void delayMs( uint16_t time)
{
	uint32_t us;
	
	us = time * 12000;
	
	CPUdelay( us );
}