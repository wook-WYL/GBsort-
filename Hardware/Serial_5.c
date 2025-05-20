#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdbool.h>


#define PIN_USART1_TXD    GPIO_Pin_9   // PA9 是 USART1 的 TXD
#define PORT_USART1_TXD   GPIOA        // USART1 TXD 的端口是 GPIOA
#define PIN_USART1_RXD    GPIO_Pin_10  // PA10 是 USART1 的 RXD+++++++++++++++
#define PORT_USART1_RXD   GPIOA        // USART1 RXD 的端口是 GPIOA

#define PIN_USART2_TXD    GPIO_Pin_2   // PA2 是 USART2 的 TXD
#define PORT_USART2_TXD   GPIOA        // USART2 TXD 的端口是 GPIOA
#define PIN_USART2_RXD    GPIO_Pin_3  // PA3 是 USART2 的 RXD
#define PORT_USART2_RXD   GPIOA        // USART2 RXD 的端口是 GPIOA

#define PIN_UART3_TXD    GPIO_Pin_10   // PB10 是 USART3 的 TXD
#define PORT_UART3_TXD   GPIOB        // USART3 TXD 的端口是 GPIOB
#define PIN_UART3_RXD    GPIO_Pin_11  // PB11 是 USART3 的 RXD
#define PORT_UART3_RXD   GPIOB        // USART3 RXD 的端口是 GPIOB

#define PIN_UART4_TXD    GPIO_Pin_10   // PC10 是 USART4 的 TXD
#define PORT_UART4_TXD   GPIOC        // USART1 TXD 的端口是 GPIOC
#define PIN_UART4_RXD    GPIO_Pin_11  // PC11 是 USART4 的 RXD
#define PORT_UART4_RXD   GPIOC        // USART1 RXD 的端口是 GPIOC

#define PIN_UART5_TXD    GPIO_Pin_12   // PC12 是 USART5 的 TXD
#define PORT_UART5_TXD   GPIOC        // USART5 TXD 的端口是 GPIOC
#define PIN_UART5_RXD    GPIO_Pin_2  // PD10 是 USART5 的 RXD
#define PORT_UART5_RXD   GPIOD        // USART5 RXD 的端口是 GPIOD


typedef enum
{
    UartPort1,
    UartPort2,
    UartPort3,
    UartPort4,
    UartPort5,
    UartPort_USB,
}UARTPORT;

#define   RXTIMEOUT    10         // 接收超时时间,如10mSec内未接收任何数据,当作一个接收包
#define   UART_TXBUFFERLEN  0x100         //发送缓冲区大小
#define   UART_RXBUFFERLEN  0x400          //接接缓冲区大小

typedef struct
{
    //接收
    volatile uint32_t       rxHead;
    volatile uint32_t       rxTail;
    volatile uint8_t      rxBuf[UART_RXBUFFERLEN];
    volatile uint8_t      rxTimeOut_Base;
    volatile uint8_t      rxTimeOut;

    //发送
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
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//开启USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA9引脚初始化为复用推挽输出
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA10引脚初始化为上拉输入
	
	/*USART初始化*/
	USART_InitTypeDef USART_InitStructure;					//定义结构体变量
	USART_InitStructure.USART_BaudRate = 9600;				//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式和接收模式均选择
	USART_InitStructure.USART_Parity = USART_Parity_No;		//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位
	USART_Init(USART1, &USART_InitStructure);				//将结构体变量交给USART_Init，配置USART1
	
	/*中断输出配置*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//配置NVIC为分组2
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//选择配置NVIC的USART1线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*USART使能*/
	USART_Cmd(USART1, ENABLE);								//使能USART1，串口开始运行

}


void usart2_init(void)
{
    USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 开启 USART2 和 GPIOA 时钟，确保 GPIO 端口可用 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* 配置 USART2 Tx 为复用推挽输出 */
    GPIO_InitStruct.GPIO_Pin = PIN_USART2_TXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_USART2_TXD, &GPIO_InitStruct);

    /* 配置 USART2 Rx 为上拉输入，增强抗干扰能力 */
    GPIO_InitStruct.GPIO_Pin = PIN_USART2_RXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PORT_USART2_RXD, &GPIO_InitStruct);

    /* USART 初始化参数 */
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = 115200; /* 设置波特率 */
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);

    /* 使能 USART2 */
    USART_Cmd(USART2, ENABLE);
}


void usart3_init(void)
{
    USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* 开启 USART3 和 GPIOC 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* 配置 USART3 Tx */
    GPIO_InitStruct.GPIO_Pin = PIN_UART3_TXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_UART3_TXD, &GPIO_InitStruct);

    /* 配置 USART3 Rx */
    GPIO_InitStruct.GPIO_Pin = PIN_UART3_RXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PORT_UART3_RXD, &GPIO_InitStruct);

    /* 初始化 USART3 */
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStruct);

    /* 使能 USART3 */
    USART_Cmd(USART3, ENABLE);
}



void usart4_init(void)
{
    USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 开启 USART4 和 GPIOC 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* 配置 USART4 Tx 为复用推挽输出 */
    GPIO_InitStruct.GPIO_Pin = PIN_UART4_TXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_UART4_TXD, &GPIO_InitStruct);

    /* 配置 USART4 Rx 为上拉输入，增强抗干扰能力 */
    GPIO_InitStruct.GPIO_Pin = PIN_UART4_RXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PORT_UART4_RXD, &GPIO_InitStruct);

    /* USART 初始化参数 */
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART4, &USART_InitStruct);

    /* 使能 USART4 */
    USART_Cmd(UART4, ENABLE);
}

void usart5_init(void)
{
    USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 开启 USART5 和相应 GPIO 端口的时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    /* 配置 USART5 Tx 为复用推挽输出 */
    GPIO_InitStruct.GPIO_Pin = PIN_UART5_TXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_UART5_TXD, &GPIO_InitStruct);

    /* 配置 USART5 Rx 为上拉输入，增强抗干扰能力 */
    GPIO_InitStruct.GPIO_Pin = PIN_UART5_RXD;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PORT_UART5_RXD, &GPIO_InitStruct);

    /* USART 初始化参数 */
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART5, &USART_InitStruct);

    /* 使能 USART5 */
    USART_Cmd(UART5, ENABLE);
}





void USART_Irq_Function(UART_STRUCT *uart,USART_TypeDef *uartdef)
{
    //接收到数据USART_ GetITStatus
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
    //发送数据
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










