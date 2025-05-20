#include <stdio.h>
#include "stm32f10x.h"                  // Device header

#include "delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Serial_5.h"

uint16_t TIME = 500;
/*
该文件用于通过串口USART2与24路舵机驱动板通讯控制
包括舵机云台的两个舵机
机械爪的三个pwm舵机和一个用于控制机械抓开合的总线舵机

控制舵机的通讯协议如下

控制单个舵机
#IndexPpwmTtime!
单个舵机指令，Index 为 3 位，000-254；pwm
为 4 位，0500-2500；time 为 4 位，0000-9999，
单位毫秒，总共 15 位数据，不足的位数补 0

{#000P1500T1000!#001P0900T1000!}
多个舵机指令，将多个单舵机指令放在一起，用{}封起来即可

控制动作组
$DGS:0!
调用动作 G0000，前提是动作 G0000 已经存储

预计将云台舵机的四种不同倾倒方式设置为不同的动作组
分别为
G0000		可回收垃圾
G0001		有害垃圾
G0002		厨余垃圾
G0003		其它垃圾

机械爪四个舵机的控制，分为三组

负责旋转的舵机，将180度分为6份，三十度一组，提前编好
负责Z轴纵向深入的预设好三种高度，一种为初始高度，一组为夹取高度，一种预备高度。
同时z轴纵深由两个电机同时驱动，需要同步控制。

爪子开合动作也要设置好预备动作。

总过需要以下几种函数

1.通过接收数据独立控制舵机的函数，五个
2.用于发送设定好动作组的函数，一个，提前将不同动作组定义好，只使用一个函数来发送动作组
*/

/*******************************************************************************************/
//基本控制函数
typedef enum {
    ACTION_GROUP_1 = 0,  // 动作组 1
    ACTION_GROUP_2 = 1,  // 动作组 2
    ACTION_GROUP_3 = 2,  // 动作组 3
    ACTION_GROUP_4 = 3,  // 动作组 4
} ActionGroup;

uint16_t angle_to_pwm_180(uint16_t angle) {
    // PWM 范围：500 (0°) 到 2500 (180°)
    uint16_t pwm = 500 + (angle * 2000) / 180;
    return (pwm < 500) ? 500 : (pwm > 2500) ? 2500 : pwm;  // 限制在有效范围内
}

// 将角度转换为 PWM 值（270° 舵机）
uint16_t angle_to_pwm_270(uint16_t angle) {
    // PWM 范围：500 (0°) 到 2500 (270°)
    uint16_t pwm = 500 + (angle * 2000) / 270;
    return (pwm < 500) ? 500 : (pwm > 2500) ? 2500 : pwm;  // 限制在有效范围内
}

/*

uint8_t index, uint16_t angle, uint16_t time, uint8_t is_270_servo
舵机标号				旋转角度						转动时间				是否为270度舵机1为是

*/
// 发送舵机控制指令
void send_servo_command(uint8_t index, uint16_t angle, uint16_t time, uint8_t is_270_servo) {
    char command[16];  // 指令缓冲区
    uint16_t pwm;

    // 根据舵机类型选择转换函数
    if (is_270_servo) {
        pwm = angle_to_pwm_270(angle);
    } else {
        pwm = angle_to_pwm_180(angle);
    }

    // 格式化指令：#IndexPpwmTtime!
    snprintf(command, sizeof(command), "#%03dP%04dT%04d!", index, pwm, time);

    // 通过串口发送指令
    Serial_SendString(USART2, command);
}

// 发送多个舵机指令（封装在 {} 中）
void send_multiple_servo_commands(char *commands) {
    char buffer[64];  // 指令缓冲区
    snprintf(buffer, sizeof(buffer), "{%s}", commands);  // 用 {} 封装指令
    Serial_SendString(USART2, buffer);
}


// 发送动作组控制命令
void send_action_group_command(ActionGroup group) {
    char command[16];  // 指令缓冲区

    // 格式化指令：$DGS:group!
    snprintf(command, sizeof(command), "$DGS:%d!", group);

    // 通过串口发送指令
    Serial_SendString(USART2, command);
}

/**********************************************************************************************/
//动作封装函数
/************************************************/
//二维舵机云台
// 定义倾斜方向
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
            baseAngle = 115.0;  // 底部舵机旋转45度
            armAngle = 50.0;   // 上臂舵机倾斜45度
            break;
        case DIRECTION_135:
            baseAngle = 205.0; // 底部舵机旋转135度
            armAngle = 50.0;   // 上臂舵机倾斜45度
            break;
        case DIRECTION_225:
            baseAngle = 270.0; // 底部舵机旋转225度
            armAngle = 50.0;   // 上臂舵机倾斜45度
            break;
        case DIRECTION_315:
            baseAngle = 10; // 底部舵机旋转315度（超出270度范围，需要调整）
            //baseAngle = baseAngle - 360.0; // 调整为-45度（等效于315度）
            armAngle = 50.0;   // 上臂舵机倾斜45度
            break;
        default:
            return;
    }

    // 控制底部舵机（270° 舵机，标号 000）
    send_servo_command(0, (uint16_t)baseAngle, TIME, 1);  // 时间 1000ms
    delay_ms(1000);  // 等待 1 秒，确保底部舵机运动完成

    // 控制上臂舵机（180° 舵机，标号 001）
    send_servo_command(2, (uint16_t)armAngle, TIME, 0);  // 时间 1000ms
    delay_ms(1000);  // 等待 1 秒，确保上臂舵机运动完成
		delay_ms(1000);
    // 复位舵机
    send_servo_command(2, 90, TIME, 0);  // 上臂舵机复位到 90°
    delay_ms(500);  // 等待 500ms

    send_servo_command(0, 65, TIME, 1);  // 底部舵机复位到 12°根据实物调节
    delay_ms(500);  // 等待 500ms
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
            baseAngle = 115.0;  // 底部舵机旋转45度

            break;
        case 1:
            baseAngle = 205.0; // 底部舵机旋转135度

            break;
        case 2:
            baseAngle = 270.0; // 底部舵机旋转225度

            break;
        case 3:
            baseAngle = 10; // 底部舵机旋转315度（超出270度范围，需要调整）

            break;
        default:
            return;
    }

    // 控制底部舵机（270° 舵机，标号 000）
    send_servo_command(0, (uint16_t)baseAngle, TIME, 1);  // 时间 1000ms
    delay_ms(1000);  // 等待 1 秒，确保底部舵机运动完成
		
    delay_ms(1000);  // 等待 1 秒，确保上臂舵机运动完成
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
    // 复位舵机
    send_servo_command(0, 65, TIME, 1);  // 底部舵机复位到 12°根据实物调节
    delay_ms(500);  // 等待 500ms
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

//机械爪
// 定义机械爪的初始状态
#define INITIAL_ANGLE 180  // 初始角度
#define INITIAL_EXTEND 0 // 初始伸缩位置
#define INITIAL_OPEN 90   // 初始开合位置

// 定义机械爪的最大和最小开合角度
#define MIN_OPEN_ANGLE 72
#define MAX_OPEN_ANGLE 117

// 定义机械爪的最大和最小伸缩位置
#define MIN_EXTEND_ANGLE 0
#define MAX_EXTEND_ANGLE 180

// 定义机械爪的最大和最小旋转角度
#define MIN_ROTATE_ANGLE 0
#define MAX_ROTATE_ANGLE 90



// 辅助函数：限制输入值的范围
int constrain_serial(int value, int minValue, int maxValue) {
    if (value < minValue) {
        return minValue;
    } else if (value > maxValue) {
        return maxValue;
    } else {
        return value;
    }
}

// 辅助函数：将一个值从一个范围映射到另一个范围
int map_serial(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}




// 控制 Z 轴伸缩
void ControlZAxis_serial(int extendLength) {
    // 限制输入参数的范围
    extendLength = constrain_serial(extendLength, MIN_EXTEND_ANGLE, MAX_EXTEND_ANGLE);
		OLED_ShowString(2, 1, "Z");
		OLED_ShowNum(2, 3, extendLength, 5); 
    // 计算两个舵机的 PWM 值
    uint16_t pwm1 = 500 + (extendLength * 2000) / 180;  // 010 号舵机
    uint16_t pwm2 = 500 + (extendLength * 2000)/ 180;  // 006 号舵机

    // 限制 PWM 值在有效范围内
    pwm1 = (pwm1 < 500) ? 500 : (pwm1 > 2500) ? 2500 : pwm1;
    pwm2 = (pwm2 < 500) ? 500 : (pwm2 > 2500) ? 2500 : pwm2;
		OLED_ShowNum(3, 3, pwm1, 5); 
		OLED_ShowNum(4, 3, pwm2, 5); 
    // 格式化两个舵机指令
    char command1[16], command2[16];
    snprintf(command1, sizeof(command1), "#010P%04dT1000!", pwm1);  // 0010 号舵机
    snprintf(command2, sizeof(command2), "#006P%04dT1000!", pwm2);  // 006 号舵机
		
    // 将两个指令封装在 {} 中并发送
    char combined_command[32];
    snprintf(combined_command, sizeof(combined_command), "%s%s", command1, command2);
    send_multiple_servo_commands(combined_command);
		OLED_ShowString(3, 1, "S");
}

// 控制旋转角度
void ControlRotation_serial(int rotateAngle) {
    // 限制输入参数的范围
    rotateAngle = constrain_serial(rotateAngle, MIN_ROTATE_ANGLE, MAX_ROTATE_ANGLE);

    // 设置旋转角度（003 号舵机）
    send_servo_command(4, rotateAngle, TIME, 0);  // 180° 舵机
}


// 机械爪控制函数
void ControlMechanicalClaw_serial(int rotateAngle, int extendLength, int objectSize) {
    // 限制输入参数的范围
    rotateAngle = constrain_serial(rotateAngle, MIN_ROTATE_ANGLE, MAX_ROTATE_ANGLE);
    extendLength = constrain_serial(extendLength, MIN_EXTEND_ANGLE, MAX_EXTEND_ANGLE);
    objectSize = constrain_serial(objectSize, MIN_OPEN_ANGLE, MAX_OPEN_ANGLE);

    // 设置机械爪的旋转角度（003 号舵机）
    send_servo_command(4, rotateAngle, TIME, 0);  // 180° 舵机

    // 设置机械爪的伸缩位置（002 和 004 号舵机）
    ControlZAxis_serial(extendLength);

    // 根据物体大小设置机械爪的开合角度（005 号舵机）
    int openAngle = map_serial(objectSize, 0, 100, MAX_OPEN_ANGLE, MIN_OPEN_ANGLE);
    send_servo_command(8, openAngle, 1000, 0);  // 180° 舵机
}




// 抓取物体
void GrabObject_serial(int rotateAngle, int extendLength, int objectSize) {
    // 移动到目标位置
    ControlMechanicalClaw_serial(rotateAngle, extendLength, objectSize);
    delay_ms(1000);  // 等待 1 秒

    // 抓取物体（闭合机械爪）
    send_servo_command(8, MIN_OPEN_ANGLE, TIME, 200);  // 180° 舵机
    delay_ms(1000);  // 等待 1 秒
		
}

// 初始化机械爪
void InitializeMechanicalClaw_serial(void) {
    // 设置机械爪的旋转角度为初始值（004 号舵机）
    send_servo_command(4, INITIAL_ANGLE, TIME, 0);  // 180° 舵机
    delay_ms(500);  // 等待 500ms

    // 设置机械爪的伸缩位置为初始值（006 和 010 号舵机）
    //send_servo_command(10, INITIAL_EXTEND, TIME, 0);  // 180° 舵机
		//delay_ms(500);  // 等待 500ms
    //send_servo_command(6, INITIAL_EXTEND, TIME, 0);  // 180° 舵机
			ControlZAxis_serial(0);
    //delay_ms(500);  // 等待 500ms

    // 设置机械爪的开合角度为初始值（005 号舵机）
    send_servo_command(8, INITIAL_OPEN, TIME, 0);  // 180° 舵机
    delay_ms(500);  // 等待 500ms
}

// 放下物体+初始化
void ReleaseObject_serial() {
    // 放下物体（打开机械爪）
    send_servo_command(8, MAX_OPEN_ANGLE, TIME, 200);  // 180° 舵机
    delay_ms(1000);  // 等待 1 秒
		InitializeMechanicalClaw_serial();
}







