#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void XianWei_Init(void) //ÏÞÎ»Æ÷ PG10 PG11
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 ;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
}	

