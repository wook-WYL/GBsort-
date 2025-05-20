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

// ��ʼ�� PCA9685
void PCA9685_Init(float hz, u16 angle)
{
    u32 off = 0;
    IIC_Init();
    PCA9685_Write(PCA_Model, 0x00);
    PCA9685_setFreq(hz);

    // �����µ�ռ�ձȷ�Χ�ͽǶȼ��� off ֵ
    off = (u32)(145+angle*2.4);
    for (u8 i = 0; i < 16; i++) {
        PCA9685_setPWM(i, 0, off);
    }
    delay_ms(100);
}

// �� PCA9685 д������
void PCA9685_Write(u8 addr, u8 data)    //   addr ��ʾҪд�����ݵļĴ�����ַ��data ��ʾҪд�������
{
    IIC_Start();                         //   ���� I2C ��ʼ�źţ���ʼ I2C ͨ�š�

    IIC_Send_Byte(PCA_Addr);             //   ���� PCA9685 �ĵ�ַ�������豸����Ҫ�� PCA9685 ����ͨ�š�
    IIC_NAck();                          //   ���Ͳ�Ӧ���źţ���ʾ����������Ҫ���豸���ո������ݡ�

    IIC_Send_Byte(addr);                 //   ����Ҫд�����ݵļĴ�����ַ��
    IIC_NAck();                          //   ���Ͳ�Ӧ���źš�

    IIC_Send_Byte(data);                 //   ����Ҫд������ݡ�
    IIC_NAck();                          //   ���Ͳ�Ӧ���źš�

    IIC_Stop();                          //   ���� I2C ֹͣ�źţ���������ͨ�š�
}

// �� PCA9685 ��ȡ����
u8 PCA9685_Read(u8 addr)                //   addr ��ʾҪ��ȡ���ݵļĴ�����ַ
{
    u8 data;                              //   ����һ���޷��� 8 λ�������� data�����ڴ洢��ȡ�������ݡ�

    IIC_Start();                          //   ���� I2C ��ʼ�źţ���ʼ I2C ͨ�š�

    IIC_Send_Byte(PCA_Addr);              //   ���� PCA9685 �ĵ�ַ�������豸����Ҫ�� PCA9685 ����ͨ�š�
    IIC_NAck();                           //   ���Ͳ�Ӧ���źţ���ʾ����������Ҫ���豸���ո������ݡ�

    IIC_Send_Byte(addr);                  //   ����Ҫ��ȡ���ݵļĴ�����ַ��
    IIC_NAck();                           //   ���Ͳ�Ӧ���źš�

    IIC_Stop();                           //   ���� I2C ֹͣ�źţ���������ͨ�š�

    delay_us(10);                         //   ��ʱ 10 ΢�룬�ȴ�оƬ׼�������ݡ�


    IIC_Start();                          //   ���� I2C ��ʼ�źţ���ʼ��һ�� I2C ͨ�š�

    IIC_Send_Byte(PCA_Addr | 0x01);         //   ���� PCA9685 �ĵ�ַ�����������λΪ 1����ʾҪ���ж�ȡ������
    IIC_NAck();                           //   ���Ͳ�Ӧ���źš�

    data = IIC_Read_Byte(0);              //   ͨ�� I2C �� PCA9685 ��ȡһ���ֽڵ����ݣ����洢������ data �С�

    IIC_Stop();                           //   ���� I2C ֹͣ�źţ���������ͨ�š�

    return data;                          //   ���ض�ȡ�������ݡ�
}

// ���� PCA9685 ָ��ͨ���� PWM ����
void PCA9685_setPWM(u8 num, u32 on, u32 off)   //num ��ʾ PWM ͨ���ţ�on ��ʾ PWM ����ʼλ�ã�off ��ʾ PWM �Ľ���λ�ã����Ӹߵ�ƽ�л����͵�ƽ��ʱ�̣�
{
    IIC_Start();                              //���� I2C ��ʼ�źţ���ʼ I2C ͨ�š�

    IIC_Send_Byte(PCA_Addr);                  //���� PCA9685 �ĵ�ַ�������豸����Ҫ�� PCA9685 ����ͨ�š�
    IIC_Wait_Ack();                           //�ȴ�Ӧ���źţ�ȷ���豸׼���ý������ݡ�

    IIC_Send_Byte(LED0_ON_L + 4 * num);           //���� LED �Ĵ����ĵ�ַ������ PWM ͨ���ż������Ӧ�ļĴ�����ַ��
    IIC_Wait_Ack();                           //

    IIC_Send_Byte(on & 0xFF);                   //���� PWM ����ʼλ�õ� 8 λ��
    IIC_Wait_Ack();                           //�ȴ�Ӧ���źš�

    IIC_Send_Byte(on >> 8);                     //���� PWM ����ʼλ�ø� 8 λ��
    IIC_Wait_Ack();                           //�ȴ�Ӧ���źš�

    IIC_Send_Byte(off & 0xFF);                  //���� PWM �Ľ���λ�õ� 8 λ��
    IIC_Wait_Ack();                           //�ȴ�Ӧ���źš�

    IIC_Send_Byte(off >> 8);                    //���� PWM �Ľ���λ�ø� 8 λ��
    IIC_Wait_Ack();                           //�ȴ�Ӧ���źš�

    IIC_Stop();                              //���� I2C ֹͣ�źţ���������ͨ�š�
}

// ���� PCA9685 �� PWM Ƶ��
void PCA9685_setFreq(float freq)
{
    u8 prescale, oldmode, newmode;              //�����������޷��� 8 λ���ͱ��� ���ڴ洢Ԥ��Ƶ��ֵ���ɵ�ģʽ�Ĵ���ֵ���µ�ģʽ�Ĵ���ֵ
    double prescaleval;                       //������һ��˫���ȸ����ͱ��� prescaleval�����ڼ���Ԥ��Ƶ����ֵ��
    freq *= 0.98;                              //�������Ƶ��ֵ���� 0.98������Ϊ��΢��Ƶ��ֵ����Ӧ PCA9685 ��ʵ������
    prescaleval = 25000000;                   //���� PCA9685 �ڲ�������Ƶ��
    prescaleval /= 4096;                      //ÿ�����ڴ�0������4095������ 4096���õ�ÿ�����������ڵ�ʱ�䣬
    prescaleval /= freq;                      //���������Ƶ��ֵ���õ�Ԥ��Ƶ����ֵ��
    prescaleval -= 1;                         //��ȥ 1���õ����յ�Ԥ��Ƶ��ֵ
    prescale = floor(prescaleval + 0.5f);       //������õ���Ԥ��Ƶ��ֵ��������ȡ���������丳ֵ�� prescale ������
    oldmode = PCA9685_Read(PCA_Model);        //ͨ������ PCA9685_Read ������ȡ��ǰ PCA9685 �Ĵ����е�ģʽֵ��������洢�� oldmode �����С�

    newmode = (oldmode & 0x7F) | 0x10;            //���ݾɵ�ģʽֵ������µ�ģʽֵ�������λ���㣨bit 7�������� 5 λ��Ϊ1��bit 4������ʾ�� PCA9685 ����Ϊ˯��ģʽ��
    PCA9685_Write(PCA_Model, newmode);         //���µ�ģʽֵд�� PCA9685 ��ģʽ�Ĵ�����
    PCA9685_Write(PCA_Pre, prescale);          //������õ���Ԥ��Ƶ��ֵд�� PCA9685 ��Ԥ��Ƶ���Ĵ�����
    PCA9685_Write(PCA_Model, oldmode);         //�ָ��ɵ�ģʽֵ��
    delay_ms(5);                              // ��ʱ 5 ���룬�ȴ� PCA9685 ��ȫ������
    PCA9685_Write(PCA_Model, oldmode | 0xa1);    //��ģʽֵ�����λ�͵� 1 λ��Ϊ1����ʾ�� PCA9685 ����Ϊ��������ģʽ��
}

// ���ݽǶ�����ָ��ͨ���Ķ��ռ�ձ�
void set180DegreeServoAngle(u8 num, u16 angle)
{
    u32 off = 0;
    // 180 �ȶ��ռ�ձȷ�Χ 250 - 1250
    off = (u32)(158+angle*2.2);
    PCA9685_setPWM(num, 0, off);
}


void set270DegreeServoAngle(u8 num, u16 angle)
{
    u32 off = 0;
    // 270 �ȶ��ռ�ձȷ�Χ 250 - 1250
    off = (u32)(250 + (1250 - 250) * angle / 270);
    PCA9685_setPWM(num, 0, off);
}



