#ifndef __CKP_H
#define __CKP_H

/*
uint8_t Usart_hmi_Trasmt[20]={0}; //ͬ�����ڷ��ʹ�����
char tjcstr[200]; //ÿ����ʶ����ַ���
char tjcstr2[200]; //�ɵ��ۼӵ�
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
void SendT1(uint16_t hao); //���Ϳɻ�����������
void SendT2(uint16_t hao); //�����к���������
void SendT3(uint16_t hao) ;//���ͳ�����������
void SendT4(uint16_t hao) ;//����������������

void SendSort(uint16_t num);
void SendAllTrashCounts(uint16_t recyclable, uint16_t hazardous, uint16_t kitchen, uint16_t other);
void SendSuccess(void);
void Send_Init(void);
void Send_NUM_s(uint16_t hao);
void Send_amount(uint16_t hao);

#endif


