#ifndef __CKP_H
#define __CKP_H

/*
uint8_t Usart_hmi_Trasmt[20]={0}; //同样用于发送串口屏
char tjcstr[200]; //每次新识别的字符串
char tjcstr2[200]; //旧的累加的
char zhongjian[200];

#define STR_LENGTH 100
*/

void SKP_init(void);
void CKPSends(char *buf1);
void TJCPrintf(const char *str, ...);
void SendDaiJi(void);
void SendManZai(void);
void SendView(void);
void ChuShiHua(void);
void SendT1(uint16_t hao); //发送可回收垃圾总数
void SendT2(uint16_t hao); //发送有害垃圾总数
void SendT3(uint16_t hao) ;//发送厨余垃圾总数
void SendT4(uint16_t hao) ;//发送其他垃圾总数

void SendSort(uint16_t num);
void SendAllTrashCounts(uint16_t recyclable, uint16_t hazardous, uint16_t kitchen, uint16_t other);
void SendSuccess(void);
void Send_Init(void);
void Send_NUM_s(uint16_t hao);
void Send_amount(uint16_t hao);

#endif


