#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include <OLED.h>
#include "Servo.h"
#include "delay.h"
#include "Servo_serial.h"
#include "CKP.h"
#include "Serial.h"
#include "Emm.h"
#include <stdlib.h>
#include <math.h> 
#define REQUEST_SIGNAL  0x01  // 请求信号：STM32请求树莓派发送数据
#define ACK_SIGNAL      0x02  // 确认信号：树莓派确认数据已接收

#define HEADER 0xFF  // 定义包头
#define FOOTER 0xFE  // 定义包尾
#define MAX_PACKET_SIZE_USART1 13  // 假设数据包最大长度为13字节（根据实际情况调整）
#define MAX_PACKET_SIZE_USART2 13

TiltDirection direction_A = DIRECTION_45;
TiltDirection direction_B = DIRECTION_135;
TiltDirection direction_C = DIRECTION_225;
TiltDirection direction_D = DIRECTION_315;


uint8_t Serial_TxPacket[13];				//定义发送数据包数组，数据包格式：FF 01 02 03 04 FE
uint8_t Serial_RxPacket_USART1[MAX_PACKET_SIZE_USART1];				//定义接收数据包数组
uint8_t Serial_RxPacket_USART2[MAX_PACKET_SIZE_USART2];
uint8_t Serial_RxFlag;					//定义接收数据包标志位

uint8_t Serial_RxFlag_USART1 = 0;  // USART1 接收标志位
uint8_t Serial_RxFlag_USART2 = 0;  // USART2 接收标志位

//uint16_t garbageCount[4];

uint16_t NUM_s =0;

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
//进行了整理，在Serial_5中

/**
  * 函    数：串口发送一个字节
  * 参    数：USARTx 串口实例（如USART1、USART2）
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(USART_TypeDef* USARTx, uint8_t Byte)
{
    USART_SendData(USARTx, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);	//等待发送完成
    /*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口发送一个数组
  * 参    数：USARTx 串口实例（如USART1、USART2）
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(USART_TypeDef* USARTx, uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i ++)		//遍历数组
    {
        Serial_SendByte(USARTx, Array[i]);		//依次调用Serial_SendByte发送每个字节数据
    }
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：USARTx 串口实例（如USART1、USART2）
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(USART_TypeDef* USARTx, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
    {
        Serial_SendByte(USARTx, String[i]);		//依次调用Serial_SendByte发送每个字节数据
    }
}




/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;	//设置结果初值为1
    while (Y --)			//执行Y次
    {
        Result *= X;		//将X累乘到结果
    }
    return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：USARTx 串口实例（如USART1、USART2）
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(USART_TypeDef* USARTx, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
    {
        Serial_SendByte(USARTx, Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
    }
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：USARTx 串口实例（如USART1、USART2）
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
    // 默认重定向到USART1
    Serial_SendByte(USART1, ch);			//将printf的底层重定向到自己的发送字节函数
    return ch;
}

/**
  * 函    数：自己封装的prinf函数
  * 参    数：USARTx 串口实例（如USART1、USART2）
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(USART_TypeDef* USARTx, char *format, ...)
{
    char String[100];				//定义字符数组
    va_list arg;					//定义可变参数列表数据类型的变量arg
    va_start(arg, format);			//从format开始，接收参数列表到arg变量
    vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
    va_end(arg);					//结束变量arg
    Serial_SendString(USARTx, String);		//串口发送字符数组（字符串）
}

/**
  * 函    数：串口发送数据包
  * 参    数：USARTx 串口实例（如USART1、USART2）
  * 参    数：无
  * 返 回 值：无
  * 说    明：调用此函数后，Serial_TxPacket数组的内容将加上包头（FF）包尾（FE）后，作为数据包发送出去
  */
void Serial_SendPacket(USART_TypeDef* USARTx)
{
    Serial_SendByte(USARTx, 0xFF);
    Serial_SendArray(USARTx, Serial_TxPacket, 12);
    Serial_SendByte(USARTx, 0xFE);
}

/**
  * 函    数：获取串口接收数据包标志位
  * 参    数：USARTx 串口实例（如 USART1、USART2）
  * 返 回 值：串口接收数据包标志位，范围：0~1，接收到数据包后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial_GetRxFlag(USART_TypeDef* USARTx)
{
    if (USARTx == USART1)
    {
        if (Serial_RxFlag_USART1 == 1)  // 如果 USART1 的标志位为1
        {
            Serial_RxFlag_USART1 = 0;  // 清零标志位
            return 1;                  // 返回1
        }
    }
    else if (USARTx == USART2)
    {
        if (Serial_RxFlag_USART2 == 1)  // 如果 USART2 的标志位为1
        {
            Serial_RxFlag_USART2 = 0;  // 清零标志位
            return 1;                  // 返回1
        }
    }
    return 0;  // 如果标志位为0，则返回0
}



/**
  * 函    数：串口发送成功消息
  * 参    数：USARTx 串口实例（如USART1、USART2）
  * 返 回 值：无
  */
void Serial_SendSuccess(USART_TypeDef* USARTx)
{
    uint8_t successMessage[] = {0xFF, 0x01, 0x00, 0x00, 0xFE}; // 成功消息
    Serial_SendArray(USARTx, successMessage, sizeof(successMessage)); // 发送成功消息
}

/**
  * 函    数：串口发送失败消息
  * 参    数：USARTx 串口实例（如USART1、USART2）
  * 返 回 值：无
  */
void Serial_SendFailure(USART_TypeDef* USARTx)
{
    uint8_t failureMessage[] = {0xFF, 0x00, 0x00,0x03, 0xFE}; // 失败消息
    Serial_SendArray(USARTx, failureMessage, sizeof(failureMessage)); // 发送失败消息
}



/**
  * 函    数：串口发送开始发送的消息
  * 参    数：USARTx 串口实例（如USART1、USART2）
  * 返 回 值：无
  */
void Serial_SendRestart(USART_TypeDef* USARTx)
{
    uint8_t failureMessage[] = {0xFF, 0x00, 0x02, 0x00, 0xFE}; // 再次发送
    Serial_SendArray(USARTx, failureMessage, sizeof(failureMessage)); // 发送下一次开始消息
}





void USART_IRQHandler(USART_TypeDef* USARTx, uint8_t* RxPacket, uint8_t* RxFlag)
{
    static uint8_t RxState = 0;      // 定义表示当前状态机状态的静态变量
    static uint8_t pRxPacket = 0;    // 定义表示当前接收数据位置的静态变量
    uint8_t RxData;

    if (USART_GetITStatus(USARTx, USART_IT_RXNE) == SET) // 判断是否是接收事件触发的中断
    {
        RxData = USART_ReceiveData(USARTx); // 读取数据寄存器，存放在接收的数据变量

        /* 使用状态机的思路，依次处理数据包的不同部分 */
        switch (RxState)
        {
            case 0: // 接收包头
                if (RxData == 0xFF) // 如果数据确实是包头
                {
                    RxState = 1; // 置下一个状态
                    pRxPacket = 0; // 数据包的位置归零
                }
                else
                {
                    // 如果不是包头，复位状态机
                    RxState = 0;
                    pRxPacket = 0;
                }
                break;

            case 1: // 接收数据
                RxPacket[pRxPacket++] = RxData; // 将数据存入数据包数组的指定位置
                if (pRxPacket >= 13) // 如果收够13字节
                {
                    RxState = 2; // 置下一个状态
                }
                break;

            case 2: // 接收包尾
                if (RxData == 0xFE) // 如果数据确实是包尾
                {
                    RxState = 0; // 状态归0
                    *RxFlag = 1; // 接收数据包标志位置1，成功接收一个数据包

                    // 打印完整数据包
                    for (int i = 0; i < 13; i++) // 修改为实际数据长度
                    {
                        OLED_ShowHexNum(4, i * 3, RxPacket[i], 2);
                    }

                    // 发送成功消息
                    Serial_SendSuccess(USART1);

                    // 增加延时或标志位检查，防止重复处理
                    while (USART_GetITStatus(USARTx, USART_IT_RXNE) == SET)
                    {
                        // 等待直到接收缓冲区为空
                    }
                }
                else
                {
                    // 如果不是包尾，复位状态机
                    RxState = 0;
                    pRxPacket = 0;
                }
                break;

            default:
                RxState = 0; // 未知状态，复位状态机
                pRxPacket = 0;
                break;
        }

        USART_ClearITPendingBit(USARTx, USART_IT_RXNE); // 清除接收中断标志位
    }
}




/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  */
void USART1_IRQHandler(void)
{
		//OLED_ShowString(3, 1, "RECEVIE_Q");
    USART_IRQHandler(USART1, Serial_RxPacket_USART1, &Serial_RxFlag_USART1);
		//OLED_ShowString(4, 1, "RECEVIE_H");
}

/**
  * 函    数：USART2中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  */


/**
  * 函    数：通用USART中断处理函数
  * 参    数：USARTx 串口实例（如 USART1、USART2）
  * 参    数：RxPacket 接收数据包缓冲区
  * 参    数：RxFlag 接收标志位指针
  * 返 回 值：无
  */








/******************************************************************************************************/
// 定义的不同中垃圾对应的脉冲值
typedef enum {
    xy_rec_0,       // 可回收()
    xy_kic_1,    // 厨余(100, 100)
    xy_harm_2,  // 有害 (-100, -100)
    xy_other_3,        // 其它()
} CoordinateType;


typedef struct {
    int x_p;
    int y_p;
		int size;
} Coordinate;

// 根据枚举类型返回对应的坐标
Coordinate get_coordinate(uint8_t type) {
    Coordinate coord;
    switch (type) {
        case 0:
            coord.x_p = 0;
            coord.y_p = 0;
            break;
        case 1:
            coord.x_p = 850;
            coord.y_p = 0;
            break;
        case 2:
            coord.x_p = 0;
            coord.y_p = 17000;
            break;
        case 3:
            coord.x_p = 850;  // 
            coord.y_p = 17000;   // 
            break;
        default:
            break;
    }
    return coord;
}

int map_serial_1(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

int* map_camera_real_phase(int x_c, int y_c) {
    // 动态分配内存给数组
		int x_r=0;
		int y_r=0;
		int x_p=0;
		int y_p=0;
    int* xy_p = (int*)malloc(2 * sizeof(int));
		if (xy_p == NULL) {
        // 处理内存分配失败的情况
        return NULL;
    }
		x_r = map_serial_1(x_c,0,300,0,250);
		y_r = map_serial_1(y_c,0,270,0,200);
		//调试偏差值
		x_r+=50;
		y_r+=50;
	
		
    
	//映射关系，脉冲需要每次都从初始位置开始记录，并且要将测得的坐标偏差加入到脉冲中
     xy_p[0]=RealToPulse(x_r,0);
		 xy_p[1]=RealToPulse(y_r,1);
		
    // 返回数组的指针
    return xy_p;
}


//机械爪运行主程序
void Grab_main(int16_t x, int16_t y, int16_t Angle, int16_t objectWidth, int8_t Type) {
	//夹爪z轴移动量	
	uint16_t z=0;
	//将摄像头坐标转换为脉冲	
		uint16_t x_p=map_camera_real_phase(x,y)[0];
	
		uint16_t y_p=map_camera_real_phase(x,y)[1];
		
	//将装置移动到摄像头识别物品位置处
	XY_GO(-x_p,-y_p);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(500);
		
	//进行抓取（使用角度，物体大小类型）
	ControlMechanicalClaw_serial(Angle,z,objectWidth);
	delay_ms(1000);
	delay_ms(500);
	//进行抬升
	ControlZAxis_serial(0);
	delay_ms(1000);
	delay_ms(500);
	//先回到初始化位置
	XY_GO_0();
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(500);
	//再移动到对应垃圾位置
	Coordinate xy;
	xy = get_coordinate(Type);
	XY_GO(xy.x_p,xy.y_p);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(500);
	//松开夹爪+自动初始化夹爪
	ReleaseObject_serial();
	//恢复初始位置
	XY_GO_0();
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	
}


// 
//一次投入多个垃圾，涉及多垃圾分类，通过机械夹爪处理
void HandleGarbage_not_single(int8_t garbageType, int16_t garbagePositionX,int16_t garbagePositionY)
{
    // 根据垃圾种类进行相应的处理
	OLED_ShowNum(1,6,garbageType,2);	
    switch (garbageType)
    {
        case 1:
            // 处理垃圾种类1的逻辑
				SendSort(1);
				//TiltTray_Serial(DIRECTION_45);
				Grab_main(garbagePositionX,garbagePositionY,45,80,garbageType);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				TiltTray_Serial_zhua(0);
				SendSuccess();
				//delay_ms(1000);
				OLED_ShowString(2,1,"1");
            break;
        case 2:
            // 处理垃圾种类2的逻辑
				SendSort(2);
				Grab_main(garbagePositionX,garbagePositionY,45,80,garbageType);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				OLED_ShowString(2,1,"2");
				SendSuccess();
				TiltTray_Serial_zhua(0);
				//TiltTray_Serial(DIRECTION_135);
            break;
        case 3:
            // 处理垃圾种类3的逻辑
				SendSort(3);
				Grab_main(garbagePositionX,garbagePositionY,45,80,garbageType);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				OLED_ShowString(2,1,"3");
				//TiltTray_Serial(DIRECTION_225);
				TiltTray_Serial_zhua(0);
				SendSuccess();
            break;
        default:
            // 处理未知垃圾种类的逻辑
				SendSort(4);
				Grab_main(garbagePositionX,garbagePositionY,45,80,garbageType);
				OLED_ShowString(2,1,"4");
				//TiltTray_Serial(DIRECTION_315);
				TiltTray_Serial_zhua(0);
				
            break;
    }
		//Serial_SendRestart(USART1);
		SendSuccess();
		//Serial_SendRestart(USART1);
}




//处理单个垃圾不涉及多垃圾分类
void HandleGarbage_single(uint8_t garbageType){
	
	
	OLED_ShowNum(1,6,garbageType,2);	
	
	switch (garbageType)
    {
				
        case 1:
            // 处理垃圾种类1的逻辑
				SendSort(1);
				TiltTray_Serial(DIRECTION_45);
				
				SendSuccess();
				//delay_ms(1000);
				OLED_ShowString(2,1,"1");
            break;
        case 2:
            // 处理垃圾种类2的逻辑
				SendSort(2);
				OLED_ShowString(2,1,"2");
				SendSuccess();
				TiltTray_Serial(DIRECTION_135);
            break;
        case 3:
            // 处理垃圾种类3的逻辑
				SendSort(3);
				OLED_ShowString(2,1,"3");
				TiltTray_Serial(DIRECTION_225);
				SendSuccess();
            break;
        default:
            // 处理未知垃圾种类的逻辑
				SendSort(4);
				OLED_ShowString(2,1,"4");
				TiltTray_Serial(DIRECTION_315);
				
            break;
    }
		//Serial_SendRestart(USART1);
		SendSuccess();
		//Serial_SendRestart(USART1);
		
		
}


/**
  * 函    数：处理接收到的数据
  * 参    数：USARTx 串口实例（如 USART1、USART2）
  * 参    数：RxPacket 接收数据包缓冲区
  * 返 回 值：无
  */
void ProcessReceivedData(USART_TypeDef* USARTx, uint8_t* RxPacket)
{
    // 1. 提取数据包中的各个部分
		OLED_ShowString(3, 1, "PRO ");
    uint8_t garbageTypes[4]; // 存储四种垃圾类型
    uint16_t garbagePositionX ; // 垃圾位置X，16位（大端表示）
    uint16_t garbagePositionY ; // 垃圾位置Y，16位（大端表示）
    static uint16_t garbageCount[4];
    uint8_t i;
    uint8_t isSameType = RxPacket[4]; // 是否同种类，位于数据包的第5个字节
		uint16_t NUM_single=0;
		uint8_t flag = 0 ;
		
	 // 提取垃圾种类
    for (i = 0; i < 4; i++) {
        garbageTypes[i] = RxPacket[i];  // 垃圾种类在第2至5个字节中
				if(garbageTypes[i]!=0){
				flag = 1;
			}
    }
		
		if(flag==1){
				NUM_s++;
		//发送序号
				Send_NUM_s(NUM_s);
			
		}
		else{
			//循环播放
			SendDaiJi();
		
		}
			
		
			
    // 提取垃圾数量
    for (i = 0; i < 4; i++) {
        //garbageCount[i] = RxPacket[8 + i];  // 垃圾数量在第10至13个字节中
			
			if(garbageTypes[i]!=0){
			
					garbageCount[i]++;
			}		
    }
		
		garbagePositionX=RxPacket[6];
		garbagePositionY=RxPacket[7];
		
		//串口屏显示种类
		SendAllTrashCounts(garbageCount[0],garbageCount[1],garbageCount[2],garbageCount[3]);
		
		
		
    // 2. 处理垃圾种类、数量和位置
    // 判断是否为同种类垃圾
			// 全局变量或静态变量，用于记录上一次检测到的位置、种类和计数器
		static float lastGarbagePositionX = -1; // 上一次检测到的 X 坐标
		static float lastGarbagePositionY = -1; // 上一次检测到的 Y 坐标
		static int lastGarbageType = -1;       // 上一次检测到的垃圾种类
		static int stuckCounter = 0;           // 卡住计数器
		const float POSITION_THRESHOLD = 10.0; // 位置差异阈值，小于此值认为位置变化不大
		const int STUCK_THRESHOLD = 3;         // 卡住阈值，连续检测到位置变化不大的次数

		if (isSameType == 0x00) {
				// 所有垃圾都是同种类，根据具体的种类进行处理
				for (i = 0; i < 4; i++) {
						if (garbageTypes[i] != 0x00) {
								NUM_single = 1;
								Send_amount(NUM_single);

								// 判断当前位置与上一次位置的差异
								float deltaX = fabs(garbagePositionX - lastGarbagePositionX);
								float deltaY = fabs(garbagePositionY - lastGarbagePositionY);

								if (lastGarbagePositionX == -1 && lastGarbagePositionY == -1) {
										// 第一次检测到位置，直接更新位置和种类，不进行卡住判断
										lastGarbagePositionX = garbagePositionX;
										lastGarbagePositionY = garbagePositionY;
										lastGarbageType = garbageTypes[i];
										HandleGarbage_single(garbageTypes[i]);
										break;
								}

								// 判断是否为同种类垃圾且位置变化不大
								if (garbageTypes[i] == lastGarbageType && deltaX < POSITION_THRESHOLD && deltaY < POSITION_THRESHOLD) {
										// 同种类且位置变化不大，增加卡住计数器
										stuckCounter++;
								} else {
										// 种类不同或位置变化较大，重置卡住计数器
										stuckCounter = 0;
								}

								// 更新上一次检测到的位置和种类
								lastGarbagePositionX = garbagePositionX;
								lastGarbagePositionY = garbagePositionY;
								lastGarbageType = garbageTypes[i];

								// 判断是否触发解救动作
								if (stuckCounter >= STUCK_THRESHOLD) {
										// 触发解救动作
										RescueAction();
										stuckCounter = 0; // 重置计数器
								} else {
										// 正常处理垃圾
										HandleGarbage_single(garbageTypes[i]);
								}
								break;
						}
				}
		}
			else {
				// 如果不是同种类，可以按每种垃圾类型分别处理
				static int notSingleCounter = 0; // 计数器，记录连续使用 HandleGarbage_not_single 的次数

				for (i = 0; i < 4; i++) {
						if (garbageTypes[i] != 0x00) {
								if (notSingleCounter >= 2) {
										// 如果连续使用 HandleGarbage_not_single 超过2次，则第四次使用 HandleGarbage_single
										HandleGarbage_single(garbageTypes[i]);
										notSingleCounter = 0; // 重置计数器
								} else {
										// 否则正常使用 HandleGarbage_not_single
										Send_amount(1);
										HandleGarbage_not_single(garbageTypes[i], garbagePositionX, garbagePositionY);
										notSingleCounter++; // 计数器加 1
								}
						}
				}
			}

    // 3. 向树莓派发送当前垃圾处理成功的消息
		Serial_SendRestart(USART1);
    // SendResponseToRaspberry(USARTx);
}



//机械爪数据定义












