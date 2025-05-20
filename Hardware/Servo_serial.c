#include <stdio.h>
#include "stm32f10x.h"                  // Device header

#include "delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Serial_5.h"

uint16_t TIME = 500;
/*
���ļ�����ͨ������USART2��24·���������ͨѶ����
���������̨���������
��еצ������pwm�����һ�����ڿ��ƻ�еץ���ϵ����߶��

���ƶ����ͨѶЭ������

���Ƶ������
#IndexPpwmTtime!
�������ָ�Index Ϊ 3 λ��000-254��pwm
Ϊ 4 λ��0500-2500��time Ϊ 4 λ��0000-9999��
��λ���룬�ܹ� 15 λ���ݣ������λ���� 0

{#000P1500T1000!#001P0900T1000!}
������ָ�����������ָ�����һ����{}����������

���ƶ�����
$DGS:0!
���ö��� G0000��ǰ���Ƕ��� G0000 �Ѿ��洢

Ԥ�ƽ���̨��������ֲ�ͬ�㵹��ʽ����Ϊ��ͬ�Ķ�����
�ֱ�Ϊ
G0000		�ɻ�������
G0001		�к�����
G0002		��������
G0003		��������

��еצ�ĸ�����Ŀ��ƣ���Ϊ����

������ת�Ķ������180�ȷ�Ϊ6�ݣ���ʮ��һ�飬��ǰ���
����Z�����������Ԥ������ָ߶ȣ�һ��Ϊ��ʼ�߶ȣ�һ��Ϊ��ȡ�߶ȣ�һ��Ԥ���߶ȡ�
ͬʱz���������������ͬʱ��������Ҫͬ�����ơ�

צ�ӿ��϶���ҲҪ���ú�Ԥ��������

�ܹ���Ҫ���¼��ֺ���

1.ͨ���������ݶ������ƶ���ĺ��������
2.���ڷ����趨�ö�����ĺ�����һ������ǰ����ͬ�����鶨��ã�ֻʹ��һ�����������Ͷ�����
*/

/*******************************************************************************************/
//�������ƺ���
typedef enum {
    ACTION_GROUP_1 = 0,  // ������ 1
    ACTION_GROUP_2 = 1,  // ������ 2
    ACTION_GROUP_3 = 2,  // ������ 3
    ACTION_GROUP_4 = 3,  // ������ 4
} ActionGroup;

uint16_t angle_to_pwm_180(uint16_t angle) {
    // PWM ��Χ��500 (0��) �� 2500 (180��)
    uint16_t pwm = 500 + (angle * 2000) / 180;
    return (pwm < 500) ? 500 : (pwm > 2500) ? 2500 : pwm;  // ��������Ч��Χ��
}

// ���Ƕ�ת��Ϊ PWM ֵ��270�� �����
uint16_t angle_to_pwm_270(uint16_t angle) {
    // PWM ��Χ��500 (0��) �� 2500 (270��)
    uint16_t pwm = 500 + (angle * 2000) / 270;
    return (pwm < 500) ? 500 : (pwm > 2500) ? 2500 : pwm;  // ��������Ч��Χ��
}

/*

uint8_t index, uint16_t angle, uint16_t time, uint8_t is_270_servo
������				��ת�Ƕ�						ת��ʱ��				�Ƿ�Ϊ270�ȶ��1Ϊ��

*/
// ���Ͷ������ָ��
void send_servo_command(uint8_t index, uint16_t angle, uint16_t time, uint8_t is_270_servo) {
    char command[16];  // ָ�����
    uint16_t pwm;

    // ���ݶ������ѡ��ת������
    if (is_270_servo) {
        pwm = angle_to_pwm_270(angle);
    } else {
        pwm = angle_to_pwm_180(angle);
    }

    // ��ʽ��ָ�#IndexPpwmTtime!
    snprintf(command, sizeof(command), "#%03dP%04dT%04d!", index, pwm, time);

    // ͨ�����ڷ���ָ��
    Serial_SendString(USART2, command);
}

// ���Ͷ�����ָ���װ�� {} �У�
void send_multiple_servo_commands(char *commands) {
    char buffer[64];  // ָ�����
    snprintf(buffer, sizeof(buffer), "{%s}", commands);  // �� {} ��װָ��
    Serial_SendString(USART2, buffer);
}


// ���Ͷ������������
void send_action_group_command(ActionGroup group) {
    char command[16];  // ָ�����

    // ��ʽ��ָ�$DGS:group!
    snprintf(command, sizeof(command), "$DGS:%d!", group);

    // ͨ�����ڷ���ָ��
    Serial_SendString(USART2, command);
}

/**********************************************************************************************/
//������װ����
/************************************************/
//��ά�����̨
// ������б����
typedef enum {
    DIRECTION_45 = 45,
    DIRECTION_135 = 135,
    DIRECTION_225 = 225,
    DIRECTION_315 = 315
} TiltDirection;

void TiltTray_Serial(TiltDirection direction) {
    float baseAngle, armAngle;

    switch (direction) {
        case DIRECTION_45:
            baseAngle = 115.0;  // �ײ������ת45��
            armAngle = 50.0;   // �ϱ۶����б45��
            break;
        case DIRECTION_135:
            baseAngle = 205.0; // �ײ������ת135��
            armAngle = 50.0;   // �ϱ۶����б45��
            break;
        case DIRECTION_225:
            baseAngle = 270.0; // �ײ������ת225��
            armAngle = 50.0;   // �ϱ۶����б45��
            break;
        case DIRECTION_315:
            baseAngle = 10; // �ײ������ת315�ȣ�����270�ȷ�Χ����Ҫ������
            //baseAngle = baseAngle - 360.0; // ����Ϊ-45�ȣ���Ч��315�ȣ�
            armAngle = 50.0;   // �ϱ۶����б45��
            break;
        default:
            return;
    }

    // ���Ƶײ������270�� �������� 000��
    send_servo_command(0, (uint16_t)baseAngle, TIME, 1);  // ʱ�� 1000ms
    delay_ms(1000);  // �ȴ� 1 �룬ȷ���ײ�����˶����

    // �����ϱ۶����180�� �������� 001��
    send_servo_command(2, (uint16_t)armAngle, TIME, 0);  // ʱ�� 1000ms
    delay_ms(1000);  // �ȴ� 1 �룬ȷ���ϱ۶���˶����
		delay_ms(1000);
    // ��λ���
    send_servo_command(2, 90, TIME, 0);  // �ϱ۶����λ�� 90��
    delay_ms(500);  // �ȴ� 500ms

    send_servo_command(0, 65, TIME, 1);  // �ײ������λ�� 12�����ʵ�����
    delay_ms(500);  // �ȴ� 500ms
}

typedef enum {
    DIRECTION_rec = 45,
    DIRECTION_kit = 135,
    DIRECTION_harm = 225,
    DIRECTION_other = 315
} type;

void TiltTray_Serial_zhua(int type) {
    float baseAngle, armAngle;

    switch (type) {
        case 0:
            baseAngle = 115.0;  // �ײ������ת45��

            break;
        case 1:
            baseAngle = 205.0; // �ײ������ת135��

            break;
        case 2:
            baseAngle = 270.0; // �ײ������ת225��

            break;
        case 3:
            baseAngle = 10; // �ײ������ת315�ȣ�����270�ȷ�Χ����Ҫ������

            break;
        default:
            return;
    }

    // ���Ƶײ������270�� �������� 000��
    send_servo_command(0, (uint16_t)baseAngle, TIME, 1);  // ʱ�� 1000ms
    delay_ms(1000);  // �ȴ� 1 �룬ȷ���ײ�����˶����
		
    delay_ms(1000);  // �ȴ� 1 �룬ȷ���ϱ۶���˶����
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
    // ��λ���
    send_servo_command(0, 65, TIME, 1);  // �ײ������λ�� 12�����ʵ�����
    delay_ms(500);  // �ȴ� 500ms
}

void RescueAction(){
		send_servo_command(0, 200, TIME, 1);
		send_servo_command(2, 20, TIME, 1);
		delay_ms(1000);
		delay_ms(500);
		send_servo_command(0, 100, TIME, 1);
		send_servo_command(2, 20, TIME, 1);
		delay_ms(1000);
		delay_ms(500);
		send_servo_command(0, 200, TIME, 1);
		send_servo_command(2, 20, TIME, 1);
		send_servo_command(0, 65, TIME, 1);
}

//��еצ
// �����еצ�ĳ�ʼ״̬
#define INITIAL_ANGLE 180  // ��ʼ�Ƕ�
#define INITIAL_EXTEND 0 // ��ʼ����λ��
#define INITIAL_OPEN 90   // ��ʼ����λ��

// �����еצ��������С���ϽǶ�
#define MIN_OPEN_ANGLE 72
#define MAX_OPEN_ANGLE 117

// �����еצ��������С����λ��
#define MIN_EXTEND_ANGLE 0
#define MAX_EXTEND_ANGLE 180

// �����еצ��������С��ת�Ƕ�
#define MIN_ROTATE_ANGLE 0
#define MAX_ROTATE_ANGLE 90



// ������������������ֵ�ķ�Χ
int constrain_serial(int value, int minValue, int maxValue) {
    if (value < minValue) {
        return minValue;
    } else if (value > maxValue) {
        return maxValue;
    } else {
        return value;
    }
}

// ������������һ��ֵ��һ����Χӳ�䵽��һ����Χ
int map_serial(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}




// ���� Z ������
void ControlZAxis_serial(int extendLength) {
    // ������������ķ�Χ
    extendLength = constrain_serial(extendLength, MIN_EXTEND_ANGLE, MAX_EXTEND_ANGLE);
		OLED_ShowString(2, 1, "Z");
		OLED_ShowNum(2, 3, extendLength, 5); 
    // ������������� PWM ֵ
    uint16_t pwm1 = 500 + (extendLength * 2000) / 180;  // 010 �Ŷ��
    uint16_t pwm2 = 500 + (extendLength * 2000)/ 180;  // 006 �Ŷ��

    // ���� PWM ֵ����Ч��Χ��
    pwm1 = (pwm1 < 500) ? 500 : (pwm1 > 2500) ? 2500 : pwm1;
    pwm2 = (pwm2 < 500) ? 500 : (pwm2 > 2500) ? 2500 : pwm2;
		OLED_ShowNum(3, 3, pwm1, 5); 
		OLED_ShowNum(4, 3, pwm2, 5); 
    // ��ʽ���������ָ��
    char command1[16], command2[16];
    snprintf(command1, sizeof(command1), "#010P%04dT1000!", pwm1);  // 0010 �Ŷ��
    snprintf(command2, sizeof(command2), "#006P%04dT1000!", pwm2);  // 006 �Ŷ��
		
    // ������ָ���װ�� {} �в�����
    char combined_command[32];
    snprintf(combined_command, sizeof(combined_command), "%s%s", command1, command2);
    send_multiple_servo_commands(combined_command);
		OLED_ShowString(3, 1, "S");
}

// ������ת�Ƕ�
void ControlRotation_serial(int rotateAngle) {
    // ������������ķ�Χ
    rotateAngle = constrain_serial(rotateAngle, MIN_ROTATE_ANGLE, MAX_ROTATE_ANGLE);

    // ������ת�Ƕȣ�003 �Ŷ����
    send_servo_command(4, rotateAngle, TIME, 0);  // 180�� ���
}


// ��еצ���ƺ���
void ControlMechanicalClaw_serial(int rotateAngle, int extendLength, int objectSize) {
    // ������������ķ�Χ
    rotateAngle = constrain_serial(rotateAngle, MIN_ROTATE_ANGLE, MAX_ROTATE_ANGLE);
    extendLength = constrain_serial(extendLength, MIN_EXTEND_ANGLE, MAX_EXTEND_ANGLE);
    objectSize = constrain_serial(objectSize, MIN_OPEN_ANGLE, MAX_OPEN_ANGLE);

    // ���û�еצ����ת�Ƕȣ�003 �Ŷ����
    send_servo_command(4, rotateAngle, TIME, 0);  // 180�� ���

    // ���û�еצ������λ�ã�002 �� 004 �Ŷ����
    ControlZAxis_serial(extendLength);

    // ���������С���û�еצ�Ŀ��ϽǶȣ�005 �Ŷ����
    int openAngle = map_serial(objectSize, 0, 100, MAX_OPEN_ANGLE, MIN_OPEN_ANGLE);
    send_servo_command(8, openAngle, 1000, 0);  // 180�� ���
}




// ץȡ����
void GrabObject_serial(int rotateAngle, int extendLength, int objectSize) {
    // �ƶ���Ŀ��λ��
    ControlMechanicalClaw_serial(rotateAngle, extendLength, objectSize);
    delay_ms(1000);  // �ȴ� 1 ��

    // ץȡ���壨�պϻ�еצ��
    send_servo_command(8, MIN_OPEN_ANGLE, TIME, 200);  // 180�� ���
    delay_ms(1000);  // �ȴ� 1 ��
		
}

// ��ʼ����еצ
void InitializeMechanicalClaw_serial(void) {
    // ���û�еצ����ת�Ƕ�Ϊ��ʼֵ��004 �Ŷ����
    send_servo_command(4, INITIAL_ANGLE, TIME, 0);  // 180�� ���
    delay_ms(500);  // �ȴ� 500ms

    // ���û�еצ������λ��Ϊ��ʼֵ��006 �� 010 �Ŷ����
    //send_servo_command(10, INITIAL_EXTEND, TIME, 0);  // 180�� ���
		//delay_ms(500);  // �ȴ� 500ms
    //send_servo_command(6, INITIAL_EXTEND, TIME, 0);  // 180�� ���
			ControlZAxis_serial(0);
    //delay_ms(500);  // �ȴ� 500ms

    // ���û�еצ�Ŀ��ϽǶ�Ϊ��ʼֵ��005 �Ŷ����
    send_servo_command(8, INITIAL_OPEN, TIME, 0);  // 180�� ���
    delay_ms(500);  // �ȴ� 500ms
}

// ��������+��ʼ��
void ReleaseObject_serial() {
    // �������壨�򿪻�еצ��
    send_servo_command(8, MAX_OPEN_ANGLE, TIME, 200);  // 180�� ���
    delay_ms(1000);  // �ȴ� 1 ��
		InitializeMechanicalClaw_serial();
}







