#include "PCA9685.h"
#include "I2C.h"
#include "delay.h"
#include <math.h>
#include "usart.h"

#define PCA_Addr 0x80
#define PCA_Model 0x00
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define PCA_Pre 0xFE

// 初始化 PCA9685
void PCA9685_Init(float hz, u16 angle)
{
    u32 off = 0;
    IIC_Init();
    PCA9685_Write(PCA_Model, 0x00);
    PCA9685_setFreq(hz);

    // 根据新的占空比范围和角度计算 off 值
    off = (u32)(145+angle*2.4);
    for (u8 i = 0; i < 16; i++) {
        PCA9685_setPWM(i, 0, off);
    }
    delay_ms(100);
}

// 向 PCA9685 写入数据
void PCA9685_Write(u8 addr, u8 data)    //   addr 表示要写入数据的寄存器地址，data 表示要写入的数据
{
    IIC_Start();                         //   发送 I2C 起始信号，开始 I2C 通信。

    IIC_Send_Byte(PCA_Addr);             //   发送 PCA9685 的地址，告诉设备我们要和 PCA9685 进行通信。
    IIC_NAck();                          //   发送不应答信号，表示主控器不需要从设备接收更多数据。

    IIC_Send_Byte(addr);                 //   发送要写入数据的寄存器地址。
    IIC_NAck();                          //   发送不应答信号。

    IIC_Send_Byte(data);                 //   发送要写入的数据。
    IIC_NAck();                          //   发送不应答信号。

    IIC_Stop();                          //   发送 I2C 停止信号，结束本次通信。
}

// 从 PCA9685 读取数据
u8 PCA9685_Read(u8 addr)                //   addr 表示要读取数据的寄存器地址
{
    u8 data;                              //   声明一个无符号 8 位整数变量 data，用于存储读取到的数据。

    IIC_Start();                          //   发送 I2C 起始信号，开始 I2C 通信。

    IIC_Send_Byte(PCA_Addr);              //   发送 PCA9685 的地址，告诉设备我们要和 PCA9685 进行通信。
    IIC_NAck();                           //   发送不应答信号，表示主控器不需要从设备接收更多数据。

    IIC_Send_Byte(addr);                  //   发送要读取数据的寄存器地址。
    IIC_NAck();                           //   发送不应答信号。

    IIC_Stop();                           //   发送 I2C 停止信号，结束本次通信。

    delay_us(10);                         //   延时 10 微秒，等待芯片准备好数据。


    IIC_Start();                          //   发送 I2C 起始信号，开始另一次 I2C 通信。

    IIC_Send_Byte(PCA_Addr | 0x01);         //   发送 PCA9685 的地址，并设置最低位为 1，表示要进行读取操作。
    IIC_NAck();                           //   发送不应答信号。

    data = IIC_Read_Byte(0);              //   通过 I2C 从 PCA9685 读取一个字节的数据，并存储到变量 data 中。

    IIC_Stop();                           //   发送 I2C 停止信号，结束本次通信。

    return data;                          //   返回读取到的数据。
}

// 设置 PCA9685 指定通道的 PWM 参数
void PCA9685_setPWM(u8 num, u32 on, u32 off)   //num 表示 PWM 通道号，on 表示 PWM 的起始位置，off 表示 PWM 的结束位置（即从高电平切换到低电平的时刻）
{
    IIC_Start();                              //发送 I2C 起始信号，开始 I2C 通信。

    IIC_Send_Byte(PCA_Addr);                  //发送 PCA9685 的地址，告诉设备我们要和 PCA9685 进行通信。
    IIC_Wait_Ack();                           //等待应答信号，确保设备准备好接收数据。

    IIC_Send_Byte(LED0_ON_L + 4 * num);           //发送 LED 寄存器的地址，根据 PWM 通道号计算出相应的寄存器地址。
    IIC_Wait_Ack();                           //

    IIC_Send_Byte(on & 0xFF);                   //发送 PWM 的起始位置低 8 位。
    IIC_Wait_Ack();                           //等待应答信号。

    IIC_Send_Byte(on >> 8);                     //发送 PWM 的起始位置高 8 位。
    IIC_Wait_Ack();                           //等待应答信号。

    IIC_Send_Byte(off & 0xFF);                  //发送 PWM 的结束位置低 8 位。
    IIC_Wait_Ack();                           //等待应答信号。

    IIC_Send_Byte(off >> 8);                    //发送 PWM 的结束位置高 8 位。
    IIC_Wait_Ack();                           //等待应答信号。

    IIC_Stop();                              //发送 I2C 停止信号，结束本次通信。
}

// 设置 PCA9685 的 PWM 频率
void PCA9685_setFreq(float freq)
{
    u8 prescale, oldmode, newmode;              //定义了三个无符号 8 位整型变量 用于存储预分频器值、旧的模式寄存器值和新的模式寄存器值
    double prescaleval;                       //定义了一个双精度浮点型变量 prescaleval，用于计算预分频器的值。
    freq *= 0.98;                              //将传入的频率值乘以 0.98，这是为了微调频率值以适应 PCA9685 的实际需求
    prescaleval = 25000000;                   //这是 PCA9685 内部振荡器的频率
    prescaleval /= 4096;                      //每个周期从0计数到4095，除以 4096，得到每个计数器周期的时间，
    prescaleval /= freq;                      //除以所需的频率值，得到预分频器的值。
    prescaleval -= 1;                         //减去 1，得到最终的预分频器值
    prescale = floor(prescaleval + 0.5f);       //将计算得到的预分频器值四舍五入取整，并将其赋值给 prescale 变量。
    oldmode = PCA9685_Read(PCA_Model);        //通过调用 PCA9685_Read 函数读取当前 PCA9685 寄存器中的模式值，并将其存储在 oldmode 变量中。

    newmode = (oldmode & 0x7F) | 0x10;            //根据旧的模式值计算出新的模式值，将最高位清零（bit 7）并将第 5 位设为1（bit 4），表示将 PCA9685 设置为睡眠模式。
    PCA9685_Write(PCA_Model, newmode);         //将新的模式值写入 PCA9685 的模式寄存器。
    PCA9685_Write(PCA_Pre, prescale);          //将计算得到的预分频器值写入 PCA9685 的预分频器寄存器。
    PCA9685_Write(PCA_Model, oldmode);         //恢复旧的模式值。
    delay_ms(5);                              // 延时 5 毫秒，等待 PCA9685 完全启动。
    PCA9685_Write(PCA_Model, oldmode | 0xa1);    //将模式值的最高位和第 1 位设为1，表示将 PCA9685 设置为正常工作模式。
}

// 根据角度设置指定通道的舵机占空比
void set180DegreeServoAngle(u8 num, u16 angle)
{
    u32 off = 0;
    // 180 度舵机占空比范围 250 - 1250
    off = (u32)(158+angle*2.2);
    PCA9685_setPWM(num, 0, off);
}


void set270DegreeServoAngle(u8 num, u16 angle)
{
    u32 off = 0;
    // 270 度舵机占空比范围 250 - 1250
    off = (u32)(250 + (1250 - 250) * angle / 270);
    PCA9685_setPWM(num, 0, off);
}



