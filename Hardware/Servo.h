#ifndef __SERVO_H
#define __SERVO_H

typedef enum {
    DIRECTION_45 = 45,   // 45�ȷ���
    DIRECTION_135 = 135, // 135�ȷ���
    DIRECTION_225 = 225, // 225�ȷ���
    DIRECTION_315 = 315  // 315�ȷ���
} TiltDirection;

typedef enum {
    DIRECTION_open = 45,
    DIRECTION_close= 135,
} TestDirection;



void Servo_Init(void);
void Servo_SetAngle_180(float Angle);
void Servo_SetAngle_270(float Angle);
void Servo_SetAngle_test(float Angle);

void Servo_SetAngle_Zhua_F(float Angle);
void Servo_SetAngle_Zhua_M(float Angle);
void Servo_SetAngle_Zhua_E(float Angle);

void TiltTray(TiltDirection direction);
void test(TestDirection direction);
#endif

