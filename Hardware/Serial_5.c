#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdbool.h>


#define PIN_USART1_TXD    GPIO_Pin_9   // PA9 �� USART1 �� TXD
#define PORT_USART1_TXD   GPIOA        // USART1 TXD �Ķ˿��� GPIOA
#define PIN_USART1_RXD    GPIO_Pin_10  // PA10 �� USART1 �� RXD+++++++++++++++
#define PORT_USART1_RXD   GPIOA        // USART1 RXD �Ķ˿��� GPIOA

#define PIN_USART2_TXD    GPIO_Pin_2   // PA2 �� USART2 �� TXD
#define PORT_USART2_TXD   GPIOA        // USART2 TXD �Ķ˿��� GPIOA
#define PIN_USART2_RXD    GPIO_Pin_3  // PA3 �� USART2 �� RXD
#define PORT_USART2_RXD   GPIOA        // USART2 RXD �Ķ˿��� GPIOA

#define PIN_UART3_TXD    GPIO_Pin_10   // PB10 �� USART3 �� TXD
#define PORT_UART3_TXD   GPIOB        // USART3 TXD �Ķ˿��� GPIOB
#define PIN_UART3_RXD    GPIO_Pin_11  // PB11 �� USART3 �� RXD
#define PORT_UART3_RXD   GPIOB        // USART3 RXD �Ķ˿��� GPIOB

#define PIN_UART4_TXD    GPIO_Pin_10   // PC10 �� USART4 �� TXD
#define PORT_UART4_TXD   GPIOC        // USART1 TXD �Ķ˿��� GPIOC
#define PIN_UART4_RXD    GPIO_Pin_11  // PC11 �� USART4 �� RXD
#define PORT_UART4_RXD   GPIOC        // USART1 RXD �Ķ˿��� GPIOC

#define PIN_UART5_TXD    GPIO_Pin_12   // PC12 �� USART5 �� TXD
#define PORT_UART5_TXD   GPIOC        // USART5 TXD �Ķ˿��� GPIOC
#define PIN_UART5_RXD    GPIO_Pin_2  // PD10 �� USART5 �� RXD
#define PORT_UART5_RXD   GPIOD        // USART5 RXD �Ķ˿��� GPIOD


typedef enum
{
    UartPort1,
    UartPort2,
    UartPort3,
    UartPort4,
    UartPort5,
    UartPort_USB,
}UARTPORT;

#define   RXTIMEOUT    10         // ���ճ�ʱʱ��,��10mSec��δ�����κ�����,����һ�����հ�
#define   UART_TXBUFFERLEN  0x100         //���ͻ�������С
#define   UART_RXBUFFERLEN  0x400          //�ӽӻ�������С

typedef struct
{
    //����
    volatile uint32_t       rxHead;
    volatile uint32_t       rxTail;
    volatile uint8_t      rxBuf[UART_RXBUFFERLEN];
    volatile uint8_t      rxTimeOut_Base;
    volatile uint8_t      rxTimeOut;

    //����
    volatile uint32_t       txHead;
    volatile uint32_t       txTail;
    volatile bool    			txEmpty;
    volatile uint8_t      baudrate;
    volatile uint8_t      txIdleTimeOut;
    volatile uint8_t      txIdleTimeOut_Base;
    volatile uint8_t      txBuf[UART_TXBUFFERLEN];
}UART_STRUCT;

static UART_STRUCT uart1;
static UART_STRUCT uart2;
static UART_STRUCT uart3;
static UART_STRUCT uart4;
static UART_STRUCT uart5;


void usart1_init(void)
{
	/*����ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//����USART1��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//����GPIOA��ʱ��
	
	/*GPIO��ʼ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//��PA9���ų�ʼ��Ϊ�����������
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//��PA10���ų�ʼ��Ϊ��������
	
	/*USART��ʼ��*/
	USART_InitTypeDef USART_InitStructure;					//����ṹ�����
	USART_InitStructure.USART_BaudRate = 9600;				//������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//Ӳ�������ƣ�����Ҫ
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//ģʽ������ģʽ�ͽ���ģʽ��ѡ��
	USART_InitStructure.USART_Parity = USART_Parity_No;		//��żУ�飬����Ҫ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//ֹͣλ��ѡ��1λ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//�ֳ���ѡ��8λ
	USART_Init(USART1, &USART_InitStructure);				//���ṹ���������USART_Init������USART1
	
	/*�ж��������*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//�������ڽ������ݵ��ж�
	
	/*NVIC�жϷ���*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//����NVICΪ����2
	
	/*NVIC����*/
	NVIC_InitTypeDef NVIC_InitStructure;					//����ṹ�����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//ѡ������NVIC��USART1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ָ��NVIC��·ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		//ָ��NVIC��·����ռ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//ָ��NVIC��·����Ӧ���ȼ�Ϊ1
	NVIC_Init(&NVIC_InitStructure);							//���ṹ���������NVIC_Init������NVIC����
	
	/*USARTʹ��*/
	USART_Cmd(USART1, ENABLE);								//ʹ��USART1�����ڿ�ʼ����

}


void usart2_init(void)
{
    USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    /* ���� USART2 �� GPIOA ʱ�ӣ�ȷ�� GPIO �˿ڿ��� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* ���� USART2 Tx Ϊ����������� */
    GPIO_InitStruct.GPIO_Pin = PIN_USART2_TXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_USART2_TXD, &GPIO_InitStruct);

    /* ���� USART2 Rx Ϊ�������룬��ǿ���������� */
    GPIO_InitStruct.GPIO_Pin = PIN_USART2_RXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PORT_USART2_RXD, &GPIO_InitStruct);

    /* USART ��ʼ������ */
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = 115200; /* ���ò����� */
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);

    /* ʹ�� USART2 */
    USART_Cmd(USART2, ENABLE);
}


void usart3_init(void)
{
    USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* ���� USART3 �� GPIOC ʱ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* ���� USART3 Tx */
    GPIO_InitStruct.GPIO_Pin = PIN_UART3_TXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_UART3_TXD, &GPIO_InitStruct);

    /* ���� USART3 Rx */
    GPIO_InitStruct.GPIO_Pin = PIN_UART3_RXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PORT_UART3_RXD, &GPIO_InitStruct);

    /* ��ʼ�� USART3 */
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStruct);

    /* ʹ�� USART3 */
    USART_Cmd(USART3, ENABLE);
}



void usart4_init(void)
{
    USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    /* ���� USART4 �� GPIOC ʱ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* ���� USART4 Tx Ϊ����������� */
    GPIO_InitStruct.GPIO_Pin = PIN_UART4_TXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_UART4_TXD, &GPIO_InitStruct);

    /* ���� USART4 Rx Ϊ�������룬��ǿ���������� */
    GPIO_InitStruct.GPIO_Pin = PIN_UART4_RXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PORT_UART4_RXD, &GPIO_InitStruct);

    /* USART ��ʼ������ */
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART4, &USART_InitStruct);

    /* ʹ�� USART4 */
    USART_Cmd(UART4, ENABLE);
}

void usart5_init(void)
{
    USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    /* ���� USART5 ����Ӧ GPIO �˿ڵ�ʱ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    /* ���� USART5 Tx Ϊ����������� */
    GPIO_InitStruct.GPIO_Pin = PIN_UART5_TXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_UART5_TXD, &GPIO_InitStruct);

    /* ���� USART5 Rx Ϊ�������룬��ǿ���������� */
    GPIO_InitStruct.GPIO_Pin = PIN_UART5_RXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PORT_UART5_RXD, &GPIO_InitStruct);

    /* USART ��ʼ������ */
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART5, &USART_InitStruct);

    /* ʹ�� USART5 */
    USART_Cmd(UART5, ENABLE);
}





void USART_Irq_Function(UART_STRUCT *uart,USART_TypeDef *uartdef)
{
    //���յ�����USART_ GetITStatus
    while (USART_GetITStatus(uartdef, USART_IT_RXNE))
    {
        uint8_t temp;

        USART_ClearFlag(uartdef, USART_FLAG_RXNE | USART_FLAG_ORE);
        temp = USART_ReceiveData(uartdef);
        if (((uart->rxHead + 1) % UART_RXBUFFERLEN) != uart->rxTail)
        {
            uart->rxBuf[uart->rxHead] = temp;
            uart->rxHead = (uart->rxHead + 1) % UART_RXBUFFERLEN;
            uart->rxTimeOut = uart->rxTimeOut_Base;
        }
    }
    //��������
    if (USART_GetITStatus(uartdef,USART_IT_TXE))
    {
        // USART_ClearFlag(uartdef, USART_FLAG_TXE);
        if (uart->txHead == uart->txTail)
        {
            uart->txEmpty = 1;
            USART_ITConfig(uartdef, USART_IT_TXE, DISABLE);
        }
        else
        {
            USART_SendData(uartdef, uart->txBuf[uart->txTail]);
            uart->txTail = (uart->txTail + 1) & (UART_TXBUFFERLEN - 1);
            uart->txIdleTimeOut = uart->txIdleTimeOut_Base;
        }
    }

}










