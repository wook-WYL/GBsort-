#include "stm32f10x.h"
#include "delay.h"
#include "OLED.h"
#include <stdint.h>
#include <stdbool.h>
#include "CKP.h"

#define SAMPLE_COUNT 10                // ��������
#define FILTER_WINDOW_SIZE 5           // �˲����ڴ�С
#define STABLE_COUNT_THRESHOLD 10       // �ȶ������ֵ
#define FULL_LOAD_DISTANCE_MM 200      // ���ؾ�����ֵ�����ף�



/*
����ʹ��˵���뵽�Ҳ��ͣ�// https://blog.zeruns.tech
*/

#define Echo GPIO_Pin_6		//HC-SR04ģ���Echo�Ž�GPIOB6
#define Trig GPIO_Pin_5		//HC-SR04ģ���Trig�Ž�GPIOB5

uint64_t time=0;			//����������������ʱ
uint64_t time_end=0;		//�����������洢�ز��ź�ʱ��

void TIM5_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // **1. ���� TIM5 ʱ��**
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    // **2. ���� TIM5 ��ʱ��**
    TIM_TimeBaseStructure.TIM_Period = 99;   // ���� 100 �������10��s x 100 = 1ms��
    TIM_TimeBaseStructure.TIM_Prescaler = 71; // 72MHz / (71+1) = 1MHz���� 1��s ����һ�Σ�
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    // **3. ʹ�� TIM5 �����ж�**
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    // **4. ���� NVIC**
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // **5. ���� TIM5**
    TIM_Cmd(TIM5, ENABLE);
}

void HC_SR04_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	//����GPIOB������ʱ��	
	GPIO_InitTypeDef GPIO_InitStructure;					//����ṹ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//����GPIO��Ϊ�������
	GPIO_InitStructure.GPIO_Pin = Trig;						//����GPIO��5
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//����GPIO���ٶ�50Mhz
	GPIO_Init(GPIOB,&GPIO_InitStructure);					//��ʼ��GPIOB
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;			//����GPIO��Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = Echo;						//����GPIO��6
	GPIO_Init(GPIOB,&GPIO_InitStructure);					//��ʼ��GPIOB
	GPIO_WriteBit(GPIOB,GPIO_Pin_5,0);						//����͵�ƽ
	delay_us(15);											//��ʱ15΢��
	
	//TIM5_Init();
}



/*
int16_t sonar_mm(void)									//��ಢ���ص�λΪ���׵ľ�����
{
	OLED_ShowString(2, 1, "Wait1");
	uint32_t Distance,Distance_mm = 0;
	GPIO_WriteBit(GPIOB,Trig,1);						//����ߵ�ƽ
	delay_us(15);										//��ʱ15΢��
	GPIO_WriteBit(GPIOB,Trig,0);						//����͵�ƽ
	while(GPIO_ReadInputDataBit(GPIOB,Echo)==0);		//�ȴ��͵�ƽ����
	time=0;												//��ʱ����
	while(GPIO_ReadInputDataBit(GPIOB,Echo)==1);		//�ȴ��ߵ�ƽ����
	time_end=time;										//��¼����ʱ��ʱ��
	if(time_end/100<38)									//�ж��Ƿ�С��38���룬����38����ľ��ǳ�ʱ��ֱ�ӵ������淵��0
	{
		Distance=(time_end*346)/2;						//������룬25��C�����е�����Ϊ346m/s
		Distance_mm=Distance/100;						//��Ϊ�����time_end�ĵ�λ��10΢�룬����Ҫ�ó���λΪ���׵ľ����������ó���100
	}
	OLED_ShowString(2, 1, "Wait2");
	return Distance_mm;									//���ز����
}

*/


int16_t sonar_mm(void) {
    uint32_t Distance, Distance_mm = 0;
    uint32_t timeout = 0;

    GPIO_WriteBit(GPIOB, Trig, 1);  // ���ʹ����ź�
    delay_us(15);
    GPIO_WriteBit(GPIOB, Trig, 0);

    OLED_Clear();
    OLED_ShowString(1, 1, "Waiting Echo...");

    // **�ȴ� Echo ��ߣ����볬ʱ**
    timeout = 0;
    while (GPIO_ReadInputDataBit(GPIOB, Echo) == 0) {
        timeout++;
        if (timeout > 10000) {  // **��ʱ�˳�**
            OLED_Clear();
            OLED_ShowString(1, 1, "Echo HIGH Timeout");
            return -1;
        }
    }

    time = 0;  // ��ʱ����

    // **�ȴ� Echo ��ͣ����볬ʱ**
    timeout = 0;
    while (GPIO_ReadInputDataBit(GPIOB, Echo) == 1) {
        timeout++;
        if (timeout > 30000) {  // **��ʱ�˳�**
            OLED_Clear();
            OLED_ShowString(2, 1, "Echo LOW Timeout");
            return -1;
        }
    }

    time_end = time;  // ��¼ʱ��

    OLED_Clear();
    OLED_ShowString(2, 1, "time_end:");
    OLED_ShowNum(2, 10, time_end, 5);  // **�鿴 time_end ֵ**

    // **�ж��Ƿ�ʱ**
    if (time_end / 100 < 38) {
        Distance = (time_end * 346) / 2;
        Distance_mm = Distance / 100;

        OLED_Clear();
        OLED_ShowString(3, 1, "Dist(mm):");
        OLED_ShowNum(3, 11, Distance_mm, 5);
    } else {
        OLED_Clear();
        OLED_ShowString(4, 1, "OUT OF RANGE");
        return -1;
    }

    return Distance_mm;
}

/*
int16_t sonar_mm(void) {
    uint32_t Distance, Distance_mm = 0;
    uint32_t timeout = 0;  // ��ʱ����
    GPIO_WriteBit(GPIOB, Trig, 1);  // ���ʹ����ź�
    delay_us(15);
    GPIO_WriteBit(GPIOB, Trig, 0);

    // **�ȴ� Echo ��ߣ����볬ʱ**
    timeout = 0;
    while (GPIO_ReadInputDataBit(GPIOB, Echo) == 0) {
        timeout++;
        if (timeout > 10000) {  // **��ʱ�˳�**
            OLED_Clear();
            OLED_ShowString(1, 1, "Echo HIGH Timeout");
            return -1;  // **���ش�����**
        }
    }

    time = 0;  // ��ʱ����

    // **�ȴ� Echo ��ͣ����볬ʱ**
    timeout = 0;
    while (GPIO_ReadInputDataBit(GPIOB, Echo) == 1) {
        timeout++;
        if (timeout > 30000) {  // **��ʱ�˳�**
            OLED_Clear();
            OLED_ShowString(2, 1, "Echo LOW Timeout");
            return -1;
        }
    }

    time_end = time;  // ��¼ʱ��

    // **�ж��Ƿ�ʱ**
    if (time_end / 100 < 38) {
        Distance = (time_end * 346) / 2;
        Distance_mm = Distance / 100;

        // **�� OLED ����ʾ�����**
        OLED_Clear();
        OLED_ShowString(1, 1, "Dist(mm):");
        OLED_ShowNum(1, 11, Distance_mm, 5);
    } else {
        OLED_Clear();
        OLED_ShowString(3, 1, "OUT OF RANGE");
        return -1;
    }

    return Distance_mm;  // ���ز����
}
*/


void TIM5_IRQHandler(void) 
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) 
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  // **һ��Ҫ����жϱ�־**
        time++;  
    }
}


/***************************************************************************************************/

// �ж��Ƿ�����
int IsFullLoad(int distance_mm) {
    return distance_mm < FULL_LOAD_DISTANCE_MM;
}


int GetAverageDistance(void) {
    int sum = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sum += sonar_mm();  // ��ȡ����
        delay_ms(10);                // ÿ�β������10ms
    }
    return sum / SAMPLE_COUNT;       // ����ƽ��ֵ
}


// �˲���������ֵ�˲���
int FilterDistance(int new_distance) {
    static int buffer[FILTER_WINDOW_SIZE] = {0};  // �˲�����
    static int index = 0;                         // ��ǰ����
    static int sum = 0;                           // �����������ܺ�

    sum -= buffer[index];           // ��ȥ��ɵ�����
    buffer[index] = new_distance;   // ����Ϊ������
    sum += buffer[index];           // ����������
    index = (index + 1) % FILTER_WINDOW_SIZE;  // ��������

    return sum / FILTER_WINDOW_SIZE;  // ����ƽ��ֵ
}


void OLED_DisplayDistance(int distance_mm) {
    int Distance_m = distance_mm / 1000;  // ת��Ϊ�ף�m��Ϊ��λ�����������ַ���Distance_m
    int Distance_m_p = distance_mm % 1000;  // ת��Ϊ�ף�m��Ϊ��λ����С�����ַ���Distance_m_p

    /*OLED_Clear_Part(2, 1, 16);  // ��OLED����2������
    OLED_ShowNum(2, 1, Distance_m, numlen(Distance_m));  // ��ʾ�����������������
    OLED_ShowChar(2, 1 + numlen(Distance_m), '.');  // ��ʾС����

    if (Distance_m_p < 100) {  // �ж��Ƿ�С��100����
        OLED_ShowChar(2, 1 + numlen(Distance_m) + 1, '0');  // ��Ϊ��λ���ף�����С��10cmʱҪ��0
        OLED_ShowNum(2, 1 + numlen(Distance_m) + 2, Distance_m_p, numlen(Distance_m_p));  // ��ʾ���������С������
        OLED_ShowChar(2, 1 + numlen(Distance_m) + 2 + numlen(Distance_m_p), 'm');  // ��ʾ��λ
    } else {
        OLED_ShowNum(2, 1 + numlen(Distance_m) + 1, Distance_m_p, numlen(Distance_m_p));  // ��ʾ���������С������
        OLED_ShowChar(2, 1 + numlen(Distance_m) + 1 + numlen(Distance_m_p), 'm');  // ��ʾ��λ
    }
	*/

    //OLED_Clear_Part(2, 1, 16);  // ��OLED����3������
    OLED_ShowNum(2, 1, distance_mm, 5);  // ��ʾ��λΪ���׵ľ�����
    //OLED_ShowString(2, 1 + numlen(distance_mm), "mm");
}


// ��ʾ������ʾ
void OLED_DisplayFullLoad(void) {
		
    //OLED_Clear_Part(3, 1, 16);  // ��OLED����3������
    OLED_ShowString(3, 1, "Full Load!");  // ��ʾ������ʾ
    //OLED_Clear_Part(3, 1, 16);  // ��OLED����3������
    //OLED_ShowString(3, 1, "Distance < 50mm");  // ��ʾ����С��50mm����ʾ
	
}


// ��������
void TriggerAlarm(void) {
    // ִ�б���������������� LED�����������
    // ������Ը���ʵ��Ӳ��ʵ��
		SendManZai();
}

void FullLoadDetection(void) {
    static uint8_t stable_count = 0;      // �ȶ�����
    static bool full_load_detected = false;  // ���ر�־

    while (1) {
        int distance_mm = GetAverageDistance();  // **ʵʱ��ȡ����**
        
        // �� OLED ����ʾ��ǰ����
        OLED_Clear();
        OLED_ShowString(1, 1, "Dist: ");
        OLED_ShowNum(1, 7, distance_mm, 5);  // ��ʾ���루��λ mm��

        if (IsFullLoad(distance_mm)) {  
            stable_count++;  

            // **��ʾ�ȶ�����**
            OLED_ShowString(2, 1, "Stable:");
            OLED_ShowNum(2, 9, stable_count, 3);

            if (stable_count >= STABLE_COUNT_THRESHOLD && !full_load_detected) {
                // **���������߼�**
                OLED_ShowString(3, 1, "FULL LOAD!!!");
                TriggerAlarm();
                full_load_detected = true;
            }
        } else {
            stable_count = 0; // ��������

            if (full_load_detected) {
                // **�ȴ������ʾ**
                OLED_ShowString(3, 1, "WAITING...");
                full_load_detected = false;
                break;  // **�����ȴ�ѭ�����ָ�����ִ��**
            }
        }

        delay_ms(100);  // **���� CPU ��ռ��**
    }
}




