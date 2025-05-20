#include "stm32f10x.h"
#include "delay.h"
#include "OLED.h"
#include <stdint.h>
#include <stdbool.h>
#include "CKP.h"

#define SAMPLE_COUNT 10                // 采样次数
#define FILTER_WINDOW_SIZE 5           // 滤波窗口大小
#define STABLE_COUNT_THRESHOLD 10       // 稳定检测阈值
#define FULL_LOAD_DISTANCE_MM 200      // 满载距离阈值（毫米）



/*
具体使用说明请到我博客：// https://blog.zeruns.tech
*/

#define Echo GPIO_Pin_6		//HC-SR04模块的Echo脚接GPIOB6
#define Trig GPIO_Pin_5		//HC-SR04模块的Trig脚接GPIOB5

uint64_t time=0;			//声明变量，用来计时
uint64_t time_end=0;		//声明变量，存储回波信号时间

void TIM5_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // **1. 开启 TIM5 时钟**
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    // **2. 配置 TIM5 定时器**
    TIM_TimeBaseStructure.TIM_Period = 99;   // 计数 100 次溢出（10μs x 100 = 1ms）
    TIM_TimeBaseStructure.TIM_Prescaler = 71; // 72MHz / (71+1) = 1MHz（即 1μs 计数一次）
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    // **3. 使能 TIM5 更新中断**
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    // **4. 配置 NVIC**
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // **5. 启动 TIM5**
    TIM_Cmd(TIM5, ENABLE);
}

void HC_SR04_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	//启用GPIOB的外设时钟	
	GPIO_InitTypeDef GPIO_InitStructure;					//定义结构体
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//设置GPIO口为推挽输出
	GPIO_InitStructure.GPIO_Pin = Trig;						//设置GPIO口5
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//设置GPIO口速度50Mhz
	GPIO_Init(GPIOB,&GPIO_InitStructure);					//初始化GPIOB
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;			//设置GPIO口为下拉输入模式
	GPIO_InitStructure.GPIO_Pin = Echo;						//设置GPIO口6
	GPIO_Init(GPIOB,&GPIO_InitStructure);					//初始化GPIOB
	GPIO_WriteBit(GPIOB,GPIO_Pin_5,0);						//输出低电平
	delay_us(15);											//延时15微秒
	
	//TIM5_Init();
}



/*
int16_t sonar_mm(void)									//测距并返回单位为毫米的距离结果
{
	OLED_ShowString(2, 1, "Wait1");
	uint32_t Distance,Distance_mm = 0;
	GPIO_WriteBit(GPIOB,Trig,1);						//输出高电平
	delay_us(15);										//延时15微秒
	GPIO_WriteBit(GPIOB,Trig,0);						//输出低电平
	while(GPIO_ReadInputDataBit(GPIOB,Echo)==0);		//等待低电平结束
	time=0;												//计时清零
	while(GPIO_ReadInputDataBit(GPIOB,Echo)==1);		//等待高电平结束
	time_end=time;										//记录结束时的时间
	if(time_end/100<38)									//判断是否小于38毫秒，大于38毫秒的就是超时，直接调到下面返回0
	{
		Distance=(time_end*346)/2;						//计算距离，25°C空气中的音速为346m/s
		Distance_mm=Distance/100;						//因为上面的time_end的单位是10微秒，所以要得出单位为毫米的距离结果，还得除以100
	}
	OLED_ShowString(2, 1, "Wait2");
	return Distance_mm;									//返回测距结果
}

*/


int16_t sonar_mm(void) {
    uint32_t Distance, Distance_mm = 0;
    uint32_t timeout = 0;

    GPIO_WriteBit(GPIOB, Trig, 1);  // 发送触发信号
    delay_us(15);
    GPIO_WriteBit(GPIOB, Trig, 0);

    OLED_Clear();
    OLED_ShowString(1, 1, "Waiting Echo...");

    // **等待 Echo 变高，加入超时**
    timeout = 0;
    while (GPIO_ReadInputDataBit(GPIOB, Echo) == 0) {
        timeout++;
        if (timeout > 10000) {  // **超时退出**
            OLED_Clear();
            OLED_ShowString(1, 1, "Echo HIGH Timeout");
            return -1;
        }
    }

    time = 0;  // 计时清零

    // **等待 Echo 变低，加入超时**
    timeout = 0;
    while (GPIO_ReadInputDataBit(GPIOB, Echo) == 1) {
        timeout++;
        if (timeout > 30000) {  // **超时退出**
            OLED_Clear();
            OLED_ShowString(2, 1, "Echo LOW Timeout");
            return -1;
        }
    }

    time_end = time;  // 记录时间

    OLED_Clear();
    OLED_ShowString(2, 1, "time_end:");
    OLED_ShowNum(2, 10, time_end, 5);  // **查看 time_end 值**

    // **判断是否超时**
    if (time_end / 100 < 38) {
        Distance = (time_end * 346) / 2;
        Distance_mm = Distance / 100;

        OLED_Clear();
        OLED_ShowString(3, 1, "Dist(mm):");
        OLED_ShowNum(3, 11, Distance_mm, 5);
    } else {
        OLED_Clear();
        OLED_ShowString(4, 1, "OUT OF RANGE");
        return -1;
    }

    return Distance_mm;
}

/*
int16_t sonar_mm(void) {
    uint32_t Distance, Distance_mm = 0;
    uint32_t timeout = 0;  // 超时计数
    GPIO_WriteBit(GPIOB, Trig, 1);  // 发送触发信号
    delay_us(15);
    GPIO_WriteBit(GPIOB, Trig, 0);

    // **等待 Echo 变高，加入超时**
    timeout = 0;
    while (GPIO_ReadInputDataBit(GPIOB, Echo) == 0) {
        timeout++;
        if (timeout > 10000) {  // **超时退出**
            OLED_Clear();
            OLED_ShowString(1, 1, "Echo HIGH Timeout");
            return -1;  // **返回错误码**
        }
    }

    time = 0;  // 计时清零

    // **等待 Echo 变低，加入超时**
    timeout = 0;
    while (GPIO_ReadInputDataBit(GPIOB, Echo) == 1) {
        timeout++;
        if (timeout > 30000) {  // **超时退出**
            OLED_Clear();
            OLED_ShowString(2, 1, "Echo LOW Timeout");
            return -1;
        }
    }

    time_end = time;  // 记录时间

    // **判断是否超时**
    if (time_end / 100 < 38) {
        Distance = (time_end * 346) / 2;
        Distance_mm = Distance / 100;

        // **在 OLED 上显示测距结果**
        OLED_Clear();
        OLED_ShowString(1, 1, "Dist(mm):");
        OLED_ShowNum(1, 11, Distance_mm, 5);
    } else {
        OLED_Clear();
        OLED_ShowString(3, 1, "OUT OF RANGE");
        return -1;
    }

    return Distance_mm;  // 返回测距结果
}
*/


void TIM5_IRQHandler(void) 
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) 
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  // **一定要清除中断标志**
        time++;  
    }
}


/***************************************************************************************************/

// 判断是否满载
int IsFullLoad(int distance_mm) {
    return distance_mm < FULL_LOAD_DISTANCE_MM;
}


int GetAverageDistance(void) {
    int sum = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sum += sonar_mm();  // 获取距离
        delay_ms(10);                // 每次采样间隔10ms
    }
    return sum / SAMPLE_COUNT;       // 返回平均值
}


// 滤波函数（均值滤波）
int FilterDistance(int new_distance) {
    static int buffer[FILTER_WINDOW_SIZE] = {0};  // 滤波窗口
    static int index = 0;                         // 当前索引
    static int sum = 0;                           // 窗口内数据总和

    sum -= buffer[index];           // 减去最旧的数据
    buffer[index] = new_distance;   // 更新为新数据
    sum += buffer[index];           // 加上新数据
    index = (index + 1) % FILTER_WINDOW_SIZE;  // 更新索引

    return sum / FILTER_WINDOW_SIZE;  // 返回平均值
}


void OLED_DisplayDistance(int distance_mm) {
    int Distance_m = distance_mm / 1000;  // 转换为米（m）为单位，将整数部分放入Distance_m
    int Distance_m_p = distance_mm % 1000;  // 转换为米（m）为单位，将小数部分放入Distance_m_p

    /*OLED_Clear_Part(2, 1, 16);  // 将OLED屏第2行清屏
    OLED_ShowNum(2, 1, Distance_m, numlen(Distance_m));  // 显示测量结果的整数部分
    OLED_ShowChar(2, 1 + numlen(Distance_m), '.');  // 显示小数点

    if (Distance_m_p < 100) {  // 判断是否小于100毫米
        OLED_ShowChar(2, 1 + numlen(Distance_m) + 1, '0');  // 因为单位是米，所以小于10cm时要加0
        OLED_ShowNum(2, 1 + numlen(Distance_m) + 2, Distance_m_p, numlen(Distance_m_p));  // 显示测量结果的小数部分
        OLED_ShowChar(2, 1 + numlen(Distance_m) + 2 + numlen(Distance_m_p), 'm');  // 显示单位
    } else {
        OLED_ShowNum(2, 1 + numlen(Distance_m) + 1, Distance_m_p, numlen(Distance_m_p));  // 显示测量结果的小数部分
        OLED_ShowChar(2, 1 + numlen(Distance_m) + 1 + numlen(Distance_m_p), 'm');  // 显示单位
    }
	*/

    //OLED_Clear_Part(2, 1, 16);  // 将OLED屏第3行清屏
    OLED_ShowNum(2, 1, distance_mm, 5);  // 显示单位为毫米的距离结果
    //OLED_ShowString(2, 1 + numlen(distance_mm), "mm");
}


// 显示满载提示
void OLED_DisplayFullLoad(void) {
		
    //OLED_Clear_Part(3, 1, 16);  // 将OLED屏第3行清屏
    OLED_ShowString(3, 1, "Full Load!");  // 显示满载提示
    //OLED_Clear_Part(3, 1, 16);  // 将OLED屏第3行清屏
    //OLED_ShowString(3, 1, "Distance < 50mm");  // 显示距离小于50mm的提示
	
}


// 报警函数
void TriggerAlarm(void) {
    // 执行报警操作，例如点亮 LED、蜂鸣器响等
    // 这里可以根据实际硬件实现
		SendManZai();
}

void FullLoadDetection(void) {
    static uint8_t stable_count = 0;      // 稳定计数
    static bool full_load_detected = false;  // 满载标志

    while (1) {
        int distance_mm = GetAverageDistance();  // **实时获取距离**
        
        // 在 OLED 上显示当前距离
        OLED_Clear();
        OLED_ShowString(1, 1, "Dist: ");
        OLED_ShowNum(1, 7, distance_mm, 5);  // 显示距离（单位 mm）

        if (IsFullLoad(distance_mm)) {  
            stable_count++;  

            // **显示稳定计数**
            OLED_ShowString(2, 1, "Stable:");
            OLED_ShowNum(2, 9, stable_count, 3);

            if (stable_count >= STABLE_COUNT_THRESHOLD && !full_load_detected) {
                // **触发满载逻辑**
                OLED_ShowString(3, 1, "FULL LOAD!!!");
                TriggerAlarm();
                full_load_detected = true;
            }
        } else {
            stable_count = 0; // 计数清零

            if (full_load_detected) {
                // **等待解除提示**
                OLED_ShowString(3, 1, "WAITING...");
                full_load_detected = false;
                break;  // **跳出等待循环，恢复正常执行**
            }
        }

        delay_ms(100);  // **避免 CPU 高占用**
    }
}




