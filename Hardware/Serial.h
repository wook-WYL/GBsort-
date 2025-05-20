#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

extern uint8_t Serial_TxPacket[];
extern uint8_t Serial_RxPacket_USART1[];
extern uint8_t Serial_RxPacket_USART2[];
extern uint8_t garbageTypes[]; // �洢������������
extern uint8_t isSameType; // �Ƿ�ͬ���࣬λ�����ݰ��ĵ�6���ֽ�
extern uint16_t garbagePositionX ;// ����λ��X��16λ����˱�ʾ��
extern uint16_t garbagePositionY; // ����λ��Y��16λ����˱�ʾ��
extern uint8_t garbageCount[4]; // �洢��������������



void Serial_Init_1(void);
void Serial_Init_2(void);

void Serial_SendByte(USART_TypeDef* USARTx, uint8_t Byte);
void Serial_SendArray(USART_TypeDef* USARTx, uint8_t *Array, uint16_t Length);
void Serial_SendString(USART_TypeDef* USARTx, char *String);
void Serial_SendNumber(USART_TypeDef* USARTx, uint32_t Number, uint8_t Length);
void Serial_Printf(USART_TypeDef* USARTx, char *format, ...);
void Serial_SendPacket(USART_TypeDef* USARTx);
void Serial_SendSuccess(USART_TypeDef* USARTx);
void Serial_SendFailure(USART_TypeDef* USARTx); 
void Serial_SendRestart(USART_TypeDef* USARTx);

void USART1_IRQHandler(void);
//void USART2_IRQHandler(void);
void USART_IRQHandler(USART_TypeDef* USARTx, uint8_t* RxPacket, uint8_t* RxFlag);
void ProcessReceivedData(USART_TypeDef* USARTx, uint8_t* RxPacket);


void HandleGarbage_not_single(int8_t garbageType, int16_t garbagePositionX,int16_t garbagePositionY);
void HandleGarbage_single(uint8_t garbageType);
void Grab_main(int16_t x, int16_t y, int16_t Angle, int16_t objectWidth, int8_t Type);


int fputc(int ch, FILE *f);
uint32_t Serial_Pow(uint32_t X, uint32_t Y);
uint8_t Serial_GetRxFlag(USART_TypeDef* USARTx);


//void Grab_main(int x, int y, int Angle, int objectWidth, Type Type);

#endif
