#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

#include <string.h>

#include "Serial_5.h"
#include "Serial.h"

uint8_t Usart_hmi_Trasmt[20]={0}; //ͬ�����ڷ��ʹ�����
char tjcstr[200]; //ÿ����ʶ����ַ���
char tjcstr2[200]; //�ɵ��ۼӵ�
char zhongjian[100];

#define STR_LENGTH 100

//��ʼ������
void SKP_init(void)
{
	
	usart5_init();//usart5 ���ڳ�ʼ��

}


//ͨ�����ڷ�������
void CKPSends(char *buf1)		  
{
	u16 i=0;
	while(1)
	{
		if(buf1[i] != 0)
	 	{
			USART_SendData(UART5,buf1[i]);  //����һ���ֽ�
			while(USART_GetFlagStatus(UART5,USART_FLAG_TXE)==RESET){};//�ȴ����ͽ���
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
	char buffer[STR_LENGTH+1];  // ���ݳ���
	va_list arg_ptr;
	va_start(arg_ptr, str);
	int len = vsnprintf(buffer, STR_LENGTH+1, str, arg_ptr);
	va_end(arg_ptr);
	for(int i = 0; i < len; i++)
	{
		USART_SendData(UART5, buffer[i]);
		while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);	//�ȴ��������
	}

	USART_SendData(UART5, end);			//���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
	while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);	//�ȴ��������
	USART_SendData(UART5, end);			//���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
	while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);	//�ȴ��������
	USART_SendData(UART5, end);			//���������Ϊ��ĵ�Ƭ���Ĵ��ڷ��͵��ֽں���
	while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);	//�ȴ��������

}





void SendDaiJi(void) //�������Ŷ���
{
			Serial_Printf(UART5,"n7.val=0\xff\xff\xff");
}


void SendManZai(void) //���ؾ���
{
			Serial_Printf(UART5,"n7.val=4\xff\xff\xff");
}

void SendView(void) //������Ϣ��ʾҳ��
{
			Serial_Printf(UART5,"n0.val=1\xff\xff\xff");
}


//�ɻ��� 1  ���� 2  �к� 3   ���� 4
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

void SendT1(uint16_t hao) //���Ϳɻ�����������
{
	    sprintf(zhongjian,"n8.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();	
}

void SendT2(uint16_t hao) //�����к���������
{
	    sprintf(zhongjian,"n9.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();	
}

void SendT3(uint16_t hao) //���ͳ�����������
{
	    sprintf(zhongjian,"n10.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();	
}

void SendT4(uint16_t hao) //����������������
{
	    sprintf(zhongjian,"n11.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();
}

void SendAllTrashCounts(uint16_t recyclable, uint16_t hazardous, uint16_t kitchen, uint16_t other)
{
    // ���Ϳɻ�����������
    SendT1(recyclable);

    // �����к���������
    SendT2(hazardous);

    // ���ͳ�����������
    SendT3(kitchen);

    // ����������������
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


void Send_NUM_s(uint16_t hao) //�������
{
	    sprintf(zhongjian,"n4.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();
}

void Send_amount(uint16_t hao) //���͵�������
{
	    sprintf(zhongjian,"n5.val=%d\xff\xff\xff",hao);
			Serial_Printf(UART5,zhongjian);
			//Optimize_Send();
}




