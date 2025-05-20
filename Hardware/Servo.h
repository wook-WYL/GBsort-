#ifndef __SERVO_H
#define __SERVO_H

typedef enum {
    DIRECTION_45 = 45,   // 45度方向
    DIRECTION_135 = 135, // 135度方向
    DIRECTION_225 = 225, // 225度方向
    DIRECTION_315 = 315  // 315度方向
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

