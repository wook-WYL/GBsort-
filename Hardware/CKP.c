#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

#include <string.h>

#include "Serial_5.h"
#include "Serial.h"

uint8_t Usart_hmi_Trasmt[20]={0}; //同样用于发送串口屏
char tjcstr[200]; //每次新识别的字符串
char tjcstr2[200]; //旧的累加的
char zhongjian[100];

#define STR_LENGTH 100

//初始化串口
void SKP_init(void)
{
	
	usart5_init();//usart5 串口初始化

}


//通过串口发送数据
void CKPSends(char *buf1)		  
{
	u16 i=0;
	while(1)
	{
		if(buf1[i] != 0)
	 	{
			USART_SendData(UART5,buf1[i]);  //发送一个字节
			while(USART_GetFlagStatus(UART5,USART_FLAG_TXE)==RESET){};//等待发送结束
		 	i++;
		}
		else
		{
			return ;
		}
	}
}

void TJCPrintf(const char *str, ...)
{


	uint8_t end = 0xff;
	char buffer[STR_LENGTH+1];  // 数据长度
	va_list arg_ptr;
	va_start(arg_ptr, str);
	int len = vsnprintf(buffer, STR_LENGTH+1, str, arg_ptr);
	va_end(arg_ptr);
	for(int i = 0; i < len; i++)
	{
		USART_SendData(UART5, buffer[i]);
		while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);	//等待发送完毕
	}

	USART_SendData(UART5, end);			//这个函数改为你的单片机的串口发送单字节函数
	while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);	//等待发送完毕
	USART_SendData(UART5, end);			//这个函数改为你的单片机的串口发送单字节函数
	while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);	//等待发送完毕
	USART_SendData(UART5, end);			//这个函数改为你的单片机的串口发送单字节函数
	while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);	//等待发送完毕

}





void SendDaiJi(void) //待机播放动画
{
			Serial_Printf(UART5,"n7.val=0\xff\xff\xff");
}


void SendManZai(void) //满载警告
{
			Serial_Printf(UART5,"n7.val=4\xff\xff\xff");
}

void SendView(void) //垃圾信息显示页面
{
			Serial_Printf(UART5,"n0.val=1\xff\xff\xff");
}


//可回收 1  厨余 2  有害 3   其它 4
void SendSort(uint16_t num)
{
			sprintf(zhongjian,"n6.val=%d\xff\xff\xff",num);
			Serial_Printf(UART5,zhongjian);
	

}

void ChuShiHua(void){
	
			strcpy(tjcstr, "view.va1.txt=\"");
			sprintf(zhongjian, "slt0.txt=\"\"");
			CKPSends(zhongjian);
			//Optimize_Send();
	
			sprintf(zhongjian, "va1.txt=\"\"");
			CKPSends(zhongjian);
			//Optimize_Send();
	
			sprintf(zhongjian, "va2.txt=\"\"");
     	CKPSends(zhongjian);
			//Optimize_Send();
			
			sprintf(zhongjian, "va3.txt=\"\"");
     	CKPSends(zhongjian);
			//Optimize_Send();
			
	    sprintf(zhongjian,"n0.val=0");
			CKPSends(zhongjian);
			//Optimize_Send();
			
			sprintf(zhongjian,"n1.val=0");
			CKPSends(zhongjian);
			//Optimize_Send();

			sprintf(zhongjian,"n2.val=0");
			CKPSends(zhongjian);
			//Optimize_Send();
			
			sprintf(zhongjian,"n3.val=0");
			CKPSends(zhongjian);
			//Optimize_Send();	

}

void SendT1(uint16_t hao) //发送可回收垃圾总数
{
	    sprintf(zhongjian,"n8.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();	
}

void SendT2(uint16_t hao) //发送有害垃圾总数
{
	    sprintf(zhongjian,"n9.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();	
}

void SendT3(uint16_t hao) //发送厨余垃圾总数
{
	    sprintf(zhongjian,"n10.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();	
}

void SendT4(uint16_t hao) //发送其他垃圾总数
{
	    sprintf(zhongjian,"n11.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();
}

void SendAllTrashCounts(uint16_t recyclable, uint16_t hazardous, uint16_t kitchen, uint16_t other)
{
    // 发送可回收垃圾数量
    SendT1(recyclable);

    // 发送有害垃圾数量
    SendT2(hazardous);

    // 发送厨余垃圾数量
    SendT3(kitchen);

    // 发送其他垃圾数量
    SendT4(other);
}


void SendSuccess(void)
{
		sprintf(zhongjian,"n7.val=1");
		CKPSends(zhongjian);

}

void Send_Init(void)
{
		sprintf(zhongjian,"n7.val=3");
		CKPSends(zhongjian);

}


void Send_NUM_s(uint16_t hao) //发送序号
{
	    sprintf(zhongjian,"n4.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();
}

void Send_amount(uint16_t hao) //发送单次数量
{
	    sprintf(zhongjian,"n5.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();
}




