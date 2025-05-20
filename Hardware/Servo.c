#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "delay.h"
#include "OLED.h"
#include <stdio.h>
 
//������ýǶ�
//���ڶ����̨�ĵ���
void Servo_SetAngle_270(float Angle)
{
	
	PWM_SetCompare_PA1(Angle / 270 * 2000 + 500);	//����ռ�ձ�
												//���Ƕ����Ա任����Ӧ�����Ҫ���ռ�ձȷ�Χ��
}


//���ڶ����̨����ת��
void Servo_SetAngle_180(float Angle)
{
	PWM_SetCompare_PA6((Angle) / 180 * 2000 + 500);	//����ռ�ձ�
												//���Ƕ����Ա任����Ӧ�����Ҫ���ռ�ձȷ�Χ��
}
 


void Servo_SetAngle_test(float Angle)
{
	PWM_SetCompare_PA7((Angle) / 180 * 2000 + 500);	//����ռ�ձ�
												//���Ƕ����Ա任����Ӧ�����Ҫ���ռ�ձȷ�Χ��
}

//���ڻ�еצ��˿�����ת��
void Servo_SetAngle_Zhua_F(float Angle)//PB0
{
	PWM_SetCompare_PB0((Angle) / 180 * 2000 + 500);	//����ռ�ձ�
												//���Ƕ����Ա任����Ӧ�����Ҫ���ռ�ձȷ�Χ��
}

//���ڻ�еצ�в�����Z��������
void Servo_SetAngle_Zhua_M(float Angle)//PB1
{
	PWM_SetCompare_PB1((Angle) / 180 * 2000 + 500);	//����ռ�ձ�
												//���Ƕ����Ա任����Ӧ�����Ҫ���ռ�ձȷ�Χ��
}

//���ڻ�еצĩ�˿��ƿ��ϵ�
void Servo_SetAngle_Zhua_E(float Angle)//PB13
{
	PWM_SetCompare_PC6((Angle) / 180 * 2000 + 500);	//����ռ�ձ�
												//���Ƕ����Ա任����Ӧ�����Ҫ���ռ�ձȷ�Χ��
}
 
 
//��    ���������ʼ��
void Servo_Init(void)
{
		PWM_Init_PA1();									//��ʼ������ĵײ�PWM
		PWM_Init_PA6_PA7();
		PWM_Init_PB0_PB1_PC6();
}
 

/*********************************************************************************/
//��ά�����̨

// ������б����
typedef enum {
    DIRECTION_45 = 45,
    DIRECTION_135 = 135,
    DIRECTION_225 = 225,
    DIRECTION_315 = 315
} TiltDirection;



// ��б���̺���
void TiltTray(TiltDirection direction) {
    float baseAngle, armAngle;

    switch (direction) {
        case DIRECTION_45:
            baseAngle = 45.0;  // �ײ������ת45��
            armAngle = 45.0;   // �ϱ۶����б45��
            break;
        case DIRECTION_135:
            baseAngle = 135.0; // �ײ������ת135��
            armAngle = 45.0;   // �ϱ۶����б45��
            break;
        case DIRECTION_225:
            baseAngle = 225.0; // �ײ������ת225��
            armAngle = 45.0;   // �ϱ۶����б45��
            break;
        case DIRECTION_315:
            baseAngle = 315.0; // �ײ������ת315�ȣ�����270�ȷ�Χ����Ҫ������
            baseAngle = baseAngle - 360.0; // ����Ϊ-45�ȣ���Ч��315�ȣ�
            armAngle = 45.0;   // �ϱ۶����б45��
            break;
        default:
            return;
    }

    // ���ƶ��
		Servo_SetAngle_270(baseAngle);
		 delay_ms(1000);  // ��ʱ500ms��ȷ���ײ�����˶����
		delay_ms(1000);
		Servo_SetAngle_180(armAngle);
		 delay_ms(500);  // ��ʱ500ms��ȷ���ײ�����˶����
		
		delay_ms(1000);
		delay_ms(1000);
	
		Servo_SetAngle_180(90);
		delay_ms(500);
		Servo_SetAngle_270(0);
		delay_ms(500);
		
}



// ������б����
typedef enum {
    DIRECTION_open = 45,
    DIRECTION_close= 135,
} TestDirection;


//��еצ���ϲ��Ժ���
void test(TestDirection direction){
		float Angle;

    switch (direction) {
        case DIRECTION_open:
            Angle = 45.0;  // �ײ������ת45��
            break;
        case DIRECTION_close:
            Angle = 0.0; // �ײ������ת135��
            break;
        default:
            return;
    }

    // ���ƶ��
		Servo_SetAngle_test(Angle);
		 delay_ms(1000);  // ��ʱ500ms��ȷ���ײ�����˶����
}

/******************************************************************************************/
//�������еצ

// �����еצ�ĳ�ʼ״̬
#define INITIAL_ANGLE 90  // ��ʼ�Ƕ�
#define INITIAL_EXTEND 90 // ��ʼ����λ��
#define INITIAL_OPEN 90   // ��ʼ����λ��

// �����еצ��������С���ϽǶ�
#define MIN_OPEN_ANGLE 0
#define MAX_OPEN_ANGLE 180

// �����еצ��������С����λ��
#define MIN_EXTEND_ANGLE 0
#define MAX_EXTEND_ANGLE 180

// �����еצ��������С��ת�Ƕ�
#define MIN_ROTATE_ANGLE 0
#define MAX_ROTATE_ANGLE 180


void InitializeMechanicalClaw() {
    // ���û�еצ����ת�Ƕ�Ϊ��ʼֵ
    Servo_SetAngle_Zhua_F(INITIAL_ANGLE);
    delay_ms(500); // �ȴ�500ms

    // ���û�еצ������λ��Ϊ��ʼֵ
    Servo_SetAngle_Zhua_M(INITIAL_EXTEND);
    delay_ms(500); // �ȴ�500ms

    // ���û�еצ�Ŀ��ϽǶ�Ϊ��ʼֵ
    Servo_SetAngle_Zhua_E(INITIAL_OPEN);
    delay_ms(500); // �ȴ�500ms

    // OLED��ʾ��ʼ�������Ϣ
    OLED_ShowString(1, 1, "ZhuaZi Success!"); // �ڵ�1�е�1����ʾ��Ϣ
}


// ������������������ֵ�ķ�Χ
int constrain(int value, int minValue, int maxValue) {
    if (value < minValue) {
        return minValue;
    } else if (value > maxValue) {
        return maxValue;
    } else {
        return value;
    }
}

// ������������һ��ֵ��һ����Χӳ�䵽��һ����Χ
int map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}



// ��еצ���ƺ���
void ControlMechanicalClaw(int rotateAngle, int extendLength, int objectSize) {
    // ������������ķ�Χ
    rotateAngle = constrain(rotateAngle, MIN_ROTATE_ANGLE, MAX_ROTATE_ANGLE);
    extendLength = constrain(extendLength, MIN_EXTEND_ANGLE, MAX_EXTEND_ANGLE);
    objectSize = constrain(objectSize, MIN_OPEN_ANGLE, MAX_OPEN_ANGLE);

    // ���û�еצ����ת�Ƕ�
    Servo_SetAngle_Zhua_F(rotateAngle);

    // ���û�еצ������λ��
    Servo_SetAngle_Zhua_M(extendLength);

    // ���������С���û�еצ�Ŀ��ϽǶ�
    // ���������СԽ�󣬿��ϽǶ�ԽС
    int openAngle = map(objectSize, 0, 100, MAX_OPEN_ANGLE, MIN_OPEN_ANGLE);
    Servo_SetAngle_Zhua_E(openAngle);
}


// ץȡ����
void GrabObject(int rotateAngle, int extendLength, int objectSize) {
    // �ƶ���Ŀ��λ��
    ControlMechanicalClaw(rotateAngle, extendLength, objectSize);

    // ץȡ����
    Servo_SetAngle_Zhua_E(MIN_OPEN_ANGLE); // �պϻ�еצ
    delay_ms(1000); // �ȴ�1�룬ȷ��ץȡ���
}


// ��������
void ReleaseObject(int rotateAngle, int extendLength) {
    // �ƶ���Ŀ��λ��
    ControlMechanicalClaw(rotateAngle, extendLength, MAX_OPEN_ANGLE);

    // ��������
    Servo_SetAngle_Zhua_E(MAX_OPEN_ANGLE); // �򿪻�еצ
    delay_ms(1000); // �ȴ�1�룬ȷ���������
}


void ControlZAxis(int extendLength) {
    // ������������ķ�Χ
    extendLength = constrain(extendLength, MIN_EXTEND_ANGLE, MAX_EXTEND_ANGLE);

    // ���� Z ������λ��
    Servo_SetAngle_Zhua_M(extendLength);

    // ��ѡ����ʾ��ǰ Z ��λ��
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "Z: %d", extendLength);
    OLED_ShowString(2, 1, buffer); // �ڵ�2�е�1����ʾ Z ��λ��
}


void ControlRotation(int rotateAngle) {
    // ������������ķ�Χ
    rotateAngle = constrain(rotateAngle, MIN_ROTATE_ANGLE, MAX_ROTATE_ANGLE);

    // ������ת�Ƕ�
    Servo_SetAngle_Zhua_F(rotateAngle);

    // ��ѡ����ʾ��ǰ��ת�Ƕ�
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "Rot: %d", rotateAngle);
    OLED_ShowString(3, 1, buffer); // �ڵ�3�е�1����ʾ��ת�Ƕ�
}





