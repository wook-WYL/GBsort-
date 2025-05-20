#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "delay.h"
#include "OLED.h"
#include <stdio.h>
 
//舵机设置角度
//用于舵机云台的底座
void Servo_SetAngle_270(float Angle)
{
	
	PWM_SetCompare_PA1(Angle / 270 * 2000 + 500);	//设置占空比
												//将角度线性变换，对应到舵机要求的占空比范围上
}


//用于舵机云台上轴转动
void Servo_SetAngle_180(float Angle)
{
	PWM_SetCompare_PA6((Angle) / 180 * 2000 + 500);	//设置占空比
												//将角度线性变换，对应到舵机要求的占空比范围上
}
 


void Servo_SetAngle_test(float Angle)
{
	PWM_SetCompare_PA7((Angle) / 180 * 2000 + 500);	//设置占空比
												//将角度线性变换，对应到舵机要求的占空比范围上
}

//用于机械爪最顶端控制旋转的
void Servo_SetAngle_Zhua_F(float Angle)//PB0
{
	PWM_SetCompare_PB0((Angle) / 180 * 2000 + 500);	//设置占空比
												//将角度线性变换，对应到舵机要求的占空比范围上
}

//用于机械爪中部控制Z轴伸缩的
void Servo_SetAngle_Zhua_M(float Angle)//PB1
{
	PWM_SetCompare_PB1((Angle) / 180 * 2000 + 500);	//设置占空比
												//将角度线性变换，对应到舵机要求的占空比范围上
}

//用于机械爪末端控制开合的
void Servo_SetAngle_Zhua_E(float Angle)//PB13
{
	PWM_SetCompare_PC6((Angle) / 180 * 2000 + 500);	//设置占空比
												//将角度线性变换，对应到舵机要求的占空比范围上
}
 
 
//函    数：舵机初始化
void Servo_Init(void)
{
		PWM_Init_PA1();									//初始化舵机的底层PWM
		PWM_Init_PA6_PA7();
		PWM_Init_PB0_PB1_PC6();
}
 

/*********************************************************************************/
//二维舵机云台

// 定义倾斜方向
typedef enum {
    DIRECTION_45 = 45,
    DIRECTION_135 = 135,
    DIRECTION_225 = 225,
    DIRECTION_315 = 315
} TiltDirection;



// 倾斜托盘函数
void TiltTray(TiltDirection direction) {
    float baseAngle, armAngle;

    switch (direction) {
        case DIRECTION_45:
            baseAngle = 45.0;  // 底部舵机旋转45度
            armAngle = 45.0;   // 上臂舵机倾斜45度
            break;
        case DIRECTION_135:
            baseAngle = 135.0; // 底部舵机旋转135度
            armAngle = 45.0;   // 上臂舵机倾斜45度
            break;
        case DIRECTION_225:
            baseAngle = 225.0; // 底部舵机旋转225度
            armAngle = 45.0;   // 上臂舵机倾斜45度
            break;
        case DIRECTION_315:
            baseAngle = 315.0; // 底部舵机旋转315度（超出270度范围，需要调整）
            baseAngle = baseAngle - 360.0; // 调整为-45度（等效于315度）
            armAngle = 45.0;   // 上臂舵机倾斜45度
            break;
        default:
            return;
    }

    // 控制舵机
		Servo_SetAngle_270(baseAngle);
		 delay_ms(1000);  // 延时500ms，确保底部舵机运动完成
		delay_ms(1000);
		Servo_SetAngle_180(armAngle);
		 delay_ms(500);  // 延时500ms，确保底部舵机运动完成
		
		delay_ms(1000);
		delay_ms(1000);
	
		Servo_SetAngle_180(90);
		delay_ms(500);
		Servo_SetAngle_270(0);
		delay_ms(500);
		
}



// 定义倾斜方向
typedef enum {
    DIRECTION_open = 45,
    DIRECTION_close= 135,
} TestDirection;


//机械爪开合测试函数
void test(TestDirection direction){
		float Angle;

    switch (direction) {
        case DIRECTION_open:
            Angle = 45.0;  // 底部舵机旋转45度
            break;
        case DIRECTION_close:
            Angle = 0.0; // 底部舵机旋转135度
            break;
        default:
            return;
    }

    // 控制舵机
		Servo_SetAngle_test(Angle);
		 delay_ms(1000);  // 延时500ms，确保底部舵机运动完成
}

/******************************************************************************************/
//三舵机机械爪

// 定义机械爪的初始状态
#define INITIAL_ANGLE 90  // 初始角度
#define INITIAL_EXTEND 90 // 初始伸缩位置
#define INITIAL_OPEN 90   // 初始开合位置

// 定义机械爪的最大和最小开合角度
#define MIN_OPEN_ANGLE 0
#define MAX_OPEN_ANGLE 180

// 定义机械爪的最大和最小伸缩位置
#define MIN_EXTEND_ANGLE 0
#define MAX_EXTEND_ANGLE 180

// 定义机械爪的最大和最小旋转角度
#define MIN_ROTATE_ANGLE 0
#define MAX_ROTATE_ANGLE 180


void InitializeMechanicalClaw() {
    // 设置机械爪的旋转角度为初始值
    Servo_SetAngle_Zhua_F(INITIAL_ANGLE);
    delay_ms(500); // 等待500ms

    // 设置机械爪的伸缩位置为初始值
    Servo_SetAngle_Zhua_M(INITIAL_EXTEND);
    delay_ms(500); // 等待500ms

    // 设置机械爪的开合角度为初始值
    Servo_SetAngle_Zhua_E(INITIAL_OPEN);
    delay_ms(500); // 等待500ms

    // OLED显示初始化完成信息
    OLED_ShowString(1, 1, "ZhuaZi Success!"); // 在第1行第1列显示消息
}


// 辅助函数：限制输入值的范围
int constrain(int value, int minValue, int maxValue) {
    if (value < minValue) {
        return minValue;
    } else if (value > maxValue) {
        return maxValue;
    } else {
        return value;
    }
}

// 辅助函数：将一个值从一个范围映射到另一个范围
int map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}



// 机械爪控制函数
void ControlMechanicalClaw(int rotateAngle, int extendLength, int objectSize) {
    // 限制输入参数的范围
    rotateAngle = constrain(rotateAngle, MIN_ROTATE_ANGLE, MAX_ROTATE_ANGLE);
    extendLength = constrain(extendLength, MIN_EXTEND_ANGLE, MAX_EXTEND_ANGLE);
    objectSize = constrain(objectSize, MIN_OPEN_ANGLE, MAX_OPEN_ANGLE);

    // 设置机械爪的旋转角度
    Servo_SetAngle_Zhua_F(rotateAngle);

    // 设置机械爪的伸缩位置
    Servo_SetAngle_Zhua_M(extendLength);

    // 根据物体大小设置机械爪的开合角度
    // 假设物体大小越大，开合角度越小
    int openAngle = map(objectSize, 0, 100, MAX_OPEN_ANGLE, MIN_OPEN_ANGLE);
    Servo_SetAngle_Zhua_E(openAngle);
}


// 抓取物体
void GrabObject(int rotateAngle, int extendLength, int objectSize) {
    // 移动到目标位置
    ControlMechanicalClaw(rotateAngle, extendLength, objectSize);

    // 抓取物体
    Servo_SetAngle_Zhua_E(MIN_OPEN_ANGLE); // 闭合机械爪
    delay_ms(1000); // 等待1秒，确保抓取完成
}


// 放下物体
void ReleaseObject(int rotateAngle, int extendLength) {
    // 移动到目标位置
    ControlMechanicalClaw(rotateAngle, extendLength, MAX_OPEN_ANGLE);

    // 放下物体
    Servo_SetAngle_Zhua_E(MAX_OPEN_ANGLE); // 打开机械爪
    delay_ms(1000); // 等待1秒，确保放下完成
}


void ControlZAxis(int extendLength) {
    // 限制输入参数的范围
    extendLength = constrain(extendLength, MIN_EXTEND_ANGLE, MAX_EXTEND_ANGLE);

    // 设置 Z 轴伸缩位置
    Servo_SetAngle_Zhua_M(extendLength);

    // 可选：显示当前 Z 轴位置
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "Z: %d", extendLength);
    OLED_ShowString(2, 1, buffer); // 在第2行第1列显示 Z 轴位置
}


void ControlRotation(int rotateAngle) {
    // 限制输入参数的范围
    rotateAngle = constrain(rotateAngle, MIN_ROTATE_ANGLE, MAX_ROTATE_ANGLE);

    // 设置旋转角度
    Servo_SetAngle_Zhua_F(rotateAngle);

    // 可选：显示当前旋转角度
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "Rot: %d", rotateAngle);
    OLED_ShowString(3, 1, buffer); // 在第3行第1列显示旋转角度
}





