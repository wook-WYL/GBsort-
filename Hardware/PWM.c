#include "stm32f10x.h"                  // Device header



/**
  * 函    数：PWM初始化
  * 参    数：无
  * 返 回 值：无
  */
//二维云台舵机初始化
void PWM_Init_PA1(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			//开启TIM2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);							//将PA1引脚初始化为复用推挽输出	
																	//受外设控制的引脚，均需要配置为复用模式
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM2);		//选择TIM2为内部时钟，若不调用此函数，TIM默认也为内部时钟
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;				//计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;				//预分频器，即PSC的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM2的时基单元
	
	/*输出比较初始化*/ 
	TIM_OCInitTypeDef TIM_OCInitStructure;							//定义结构体变量
	TIM_OCStructInit(&TIM_OCInitStructure);                         //结构体初始化，若结构体没有完整赋值
	                                                                //则最好执行此函数，给结构体所有成员都赋一个默认值
	                                                                //避免结构体初值不确定的问题
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               //输出比较模式，选择PWM模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //输出极性，选择为高，若选择极性为低，则输出高低电平取反
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;								//初始的CCR值
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);                        //将结构体变量交给TIM_OC2Init，配置TIM2的输出比较通道2
	
	/*TIM使能*/
	TIM_Cmd(TIM2, ENABLE);			//使能TIM2，定时器开始运行
}

//二维云台舵机初始化 使用了PA6 PA7空闲
void PWM_Init_PA6_PA7(void)
{
    /* 开启时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    // 开启TIM3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // 开启GPIOA时钟

    /* GPIO初始化 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 初始化PA6（TIM3_CH1）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 初始化PA7（TIM3_CH2）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 配置时钟源 */
    TIM_InternalClockConfig(TIM3);       // 使用TIM3内部时钟

    /* 时基单元初始化 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;       // ARR（周期）
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;       // PSC（预分频）
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;    // 高级定时器专用
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    /* 输出比较初始化（通道1和通道2） */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);                 // 初始化默认值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    
    // 配置通道1（PA6）
    TIM_OCInitStructure.TIM_Pulse = 0;                      // 初始CCR1值
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    
    // 配置通道2（PA7）
    TIM_OCInitStructure.TIM_Pulse = 0;                      // 初始CCR2值
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    /* 使能TIM3 */
    TIM_Cmd(TIM3, ENABLE);
}


//机械爪三舵机pwm初始化
void PWM_Init_PB0_PB1_PC6(void)
{
    /*开启时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);            //开启TIM3的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE); //开启GPIOB和GPIOC的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);            //开启TIM8的时钟

    /*GPIO初始化*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 初始化PB0 (TIM3_CH3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始化PB1 (TIM3_CH4)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始化PC6 (TIM8_CH1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*配置时钟源*/
    TIM_InternalClockConfig(TIM3);                                  //选择TIM3为内部时钟
    TIM_InternalClockConfig(TIM8);                                  //选择TIM8为内部时钟

    /*时基单元初始化*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;               //计数周期，即ARR的值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;               //预分频器，即PSC的值
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);             //配置TIM3的时基单元
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);             //配置TIM8的时基单元

    /*输出比较初始化*/
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;                              //初始的CCR值

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);                        //配置TIM3的输出比较通道3 (PB0)
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);               //启用TIM3通道3的预装载寄存器

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);                        //配置TIM3的输出比较通道4 (PB1)
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);               //启用TIM3通道4的预装载寄存器

    TIM_OC1Init(TIM8, &TIM_OCInitStructure);                        //配置TIM8的输出比较通道1 (PC6)
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);               //启用TIM8通道1的预装载寄存器

    /*TIM使能*/
    TIM_Cmd(TIM3, ENABLE);                                          //使能TIM3，定时器开始运行
    TIM_Cmd(TIM8, ENABLE);                                          //使能TIM8，定时器开始运行
    TIM_CtrlPWMOutputs(TIM8, ENABLE);                               //使能TIM8的PWM输出
}






/**
  * 函    数：PWM设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~100
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */

void PWM_SetCompare_PA1(uint16_t Compare)
{
	TIM_SetCompare2(TIM2, Compare);		//设置CCR2的值
}


void PWM_SetCompare_PA6(uint16_t Compare)
{
    TIM_SetCompare1(TIM3, Compare);  // 设置TIM3的CCR1值（PA6）
}


void PWM_SetCompare_PA7(uint16_t Compare)
{
    TIM_SetCompare2(TIM3, Compare);  // 设置TIM3的CCR2值（PA7）
}

void PWM_SetCompare_PB0(uint16_t Compare)
{
    TIM_SetCompare3(TIM3, Compare);  // 设置TIM3的CCR2值（PA7）
}

void PWM_SetCompare_PB1(uint16_t Compare)
{
    TIM_SetCompare4(TIM3, Compare);  // 设置TIM3的CCR2值（PA7）
}

void PWM_SetCompare_PC6(uint16_t Compare)
{
    TIM_SetCompare1(TIM8, Compare);  // 设置TIM8的CCR1值（PC6）
}