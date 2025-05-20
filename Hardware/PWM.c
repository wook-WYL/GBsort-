#include "stm32f10x.h"                  // Device header



/**
  * ��    ����PWM��ʼ��
  * ��    ������
  * �� �� ֵ����
  */
//��ά��̨�����ʼ��
void PWM_Init_PA1(void)
{
	/*����ʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			//����TIM2��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//����GPIOA��ʱ��
	
	/*GPIO��ʼ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);							//��PA1���ų�ʼ��Ϊ�����������	
																	//��������Ƶ����ţ�����Ҫ����Ϊ����ģʽ
	
	/*����ʱ��Դ*/
	TIM_InternalClockConfig(TIM2);		//ѡ��TIM2Ϊ�ڲ�ʱ�ӣ��������ô˺�����TIMĬ��ҲΪ�ڲ�ʱ��
	
	/*ʱ����Ԫ��ʼ��*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//����ṹ�����
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //������ģʽ��ѡ�����ϼ���
	TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;				//�������ڣ���ARR��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;				//Ԥ��Ƶ������PSC��ֵ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //�ظ����������߼���ʱ���Ż��õ�
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);             //���ṹ���������TIM_TimeBaseInit������TIM2��ʱ����Ԫ
	
	/*����Ƚϳ�ʼ��*/ 
	TIM_OCInitTypeDef TIM_OCInitStructure;							//����ṹ�����
	TIM_OCStructInit(&TIM_OCInitStructure);                         //�ṹ���ʼ�������ṹ��û��������ֵ
	                                                                //�����ִ�д˺��������ṹ�����г�Ա����һ��Ĭ��ֵ
	                                                                //����ṹ���ֵ��ȷ��������
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               //����Ƚ�ģʽ��ѡ��PWMģʽ1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //������ԣ�ѡ��Ϊ�ߣ���ѡ����Ϊ�ͣ�������ߵ͵�ƽȡ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //���ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;								//��ʼ��CCRֵ
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);                        //���ṹ���������TIM_OC2Init������TIM2������Ƚ�ͨ��2
	
	/*TIMʹ��*/
	TIM_Cmd(TIM2, ENABLE);			//ʹ��TIM2����ʱ����ʼ����
}

//��ά��̨�����ʼ�� ʹ����PA6 PA7����
void PWM_Init_PA6_PA7(void)
{
    /* ����ʱ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    // ����TIM3ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // ����GPIOAʱ��

    /* GPIO��ʼ�� */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // ��ʼ��PA6��TIM3_CH1��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // ��ʼ��PA7��TIM3_CH2��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ����ʱ��Դ */
    TIM_InternalClockConfig(TIM3);       // ʹ��TIM3�ڲ�ʱ��

    /* ʱ����Ԫ��ʼ�� */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;       // ARR�����ڣ�
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;       // PSC��Ԥ��Ƶ��
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;    // �߼���ʱ��ר��
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    /* ����Ƚϳ�ʼ����ͨ��1��ͨ��2�� */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);                 // ��ʼ��Ĭ��ֵ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    
    // ����ͨ��1��PA6��
    TIM_OCInitStructure.TIM_Pulse = 0;                      // ��ʼCCR1ֵ
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    
    // ����ͨ��2��PA7��
    TIM_OCInitStructure.TIM_Pulse = 0;                      // ��ʼCCR2ֵ
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    /* ʹ��TIM3 */
    TIM_Cmd(TIM3, ENABLE);
}


//��еצ�����pwm��ʼ��
void PWM_Init_PB0_PB1_PC6(void)
{
    /*����ʱ��*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);            //����TIM3��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE); //����GPIOB��GPIOC��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);            //����TIM8��ʱ��

    /*GPIO��ʼ��*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // ��ʼ��PB0 (TIM3_CH3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // ��ʼ��PB1 (TIM3_CH4)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // ��ʼ��PC6 (TIM8_CH1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*����ʱ��Դ*/
    TIM_InternalClockConfig(TIM3);                                  //ѡ��TIM3Ϊ�ڲ�ʱ��
    TIM_InternalClockConfig(TIM8);                                  //ѡ��TIM8Ϊ�ڲ�ʱ��

    /*ʱ����Ԫ��ʼ��*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;               //�������ڣ���ARR��ֵ
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;               //Ԥ��Ƶ������PSC��ֵ
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);             //����TIM3��ʱ����Ԫ
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);             //����TIM8��ʱ����Ԫ

    /*����Ƚϳ�ʼ��*/
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;                              //��ʼ��CCRֵ

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);                        //����TIM3������Ƚ�ͨ��3 (PB0)
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);               //����TIM3ͨ��3��Ԥװ�ؼĴ���

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);                        //����TIM3������Ƚ�ͨ��4 (PB1)
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);               //����TIM3ͨ��4��Ԥװ�ؼĴ���

    TIM_OC1Init(TIM8, &TIM_OCInitStructure);                        //����TIM8������Ƚ�ͨ��1 (PC6)
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);               //����TIM8ͨ��1��Ԥװ�ؼĴ���

    /*TIMʹ��*/
    TIM_Cmd(TIM3, ENABLE);                                          //ʹ��TIM3����ʱ����ʼ����
    TIM_Cmd(TIM8, ENABLE);                                          //ʹ��TIM8����ʱ����ʼ����
    TIM_CtrlPWMOutputs(TIM8, ENABLE);                               //ʹ��TIM8��PWM���
}






/**
  * ��    ����PWM����CCR
  * ��    ����Compare Ҫд���CCR��ֵ����Χ��0~100
  * �� �� ֵ����
  * ע�����CCR��ARR��ͬ����ռ�ձȣ��˺���������CCR��ֵ������ֱ����ռ�ձ�
  *           ռ�ձ�Duty = CCR / (ARR + 1)
  */

void PWM_SetCompare_PA1(uint16_t Compare)
{
	TIM_SetCompare2(TIM2, Compare);		//����CCR2��ֵ
}


void PWM_SetCompare_PA6(uint16_t Compare)
{
    TIM_SetCompare1(TIM3, Compare);  // ����TIM3��CCR1ֵ��PA6��
}


void PWM_SetCompare_PA7(uint16_t Compare)
{
    TIM_SetCompare2(TIM3, Compare);  // ����TIM3��CCR2ֵ��PA7��
}

void PWM_SetCompare_PB0(uint16_t Compare)
{
    TIM_SetCompare3(TIM3, Compare);  // ����TIM3��CCR2ֵ��PA7��
}

void PWM_SetCompare_PB1(uint16_t Compare)
{
    TIM_SetCompare4(TIM3, Compare);  // ����TIM3��CCR2ֵ��PA7��
}

void PWM_SetCompare_PC6(uint16_t Compare)
{
    TIM_SetCompare1(TIM8, Compare);  // ����TIM8��CCR1ֵ��PC6��
}