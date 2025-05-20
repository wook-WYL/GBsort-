#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include "Emm.h"
#include "XianWei.h"
#include "Delay.h"
#include "OLED.h"

#define X_DaoWei while(GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_11)==0) //检测x轴电机是否运动到规定位置，接步进电机dir
#define Y_DaoWei while(GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_10)==0) //检测y轴电机是否运动到规定位置
	
#define MOTOR_RANGE 825		//电机脉冲范围
#define CAMERA_WIDTH 640	//摄像头识别到的图像位置 宽
#define CAMERA_HEIGHT 640	//摄像头识别到的图像位置 高

#define MOTOR_PULSE_RANGE 15000	//	从最一端到另一端的最大脉冲数

// 定义脉冲范围

#define X_PULSE 850

#define Y_PULSE 17000

// 定义真实坐标范围

#define X_REAL 125

#define Y_REAL 100

int16_t X=0,Y=0;


uint8_t XianWei_Flag=0;//限位成功标志位

void Emm_Init(void)// PD10 PD11
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);  // 使能 GPIOG 时钟

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;  // 配置 PG10 和 PG11
    GPIO_Init(GPIOG, &GPIO_InitStructure);
}


// 定义限位开关按下的计数器
uint16_t X_LimitCount = 0;
uint16_t Y_LimitCount = 0;
void TestLimitSwitches(void)
{
    // 初始化限位开关引脚
    Emm_Init();

    while (1)
    {
        // 检测X轴限位开关
        if (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) == 0)
        {
            X_LimitCount++;  // 增加X轴限位开关按下计数
            OLED_ShowNum(1, 7, X_LimitCount, 5);  // 在OLED上显示X轴限位开关按下次数
            while (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) == 0);  // 等待限位开关释放
        }

        // 检测Y轴限位开关
        if (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11) == 0)
        {
            Y_LimitCount++;  // 增加Y轴限位开关按下计数
            OLED_ShowNum(2, 7, Y_LimitCount, 5);  // 在OLED上显示Y轴限位开关按下次数
            while (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11) == 0);  // 等待限位开关释放
        }

        // 添加适当的延时，避免过于频繁的检测
        delay_ms(10);  // 假设你有一个延时函数
    }
}

/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0是直接启动
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial_SendArray(UART4,cmd, 8);
}


/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, uint8_t snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  Serial_SendArray(UART4,cmd, 13);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial_SendArray(UART4,cmd, 5);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_En_Control(uint8_t addr, uint8_t state, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial_SendArray(UART4,cmd, 6);
}


/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0; 
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  Serial_SendArray(UART4,cmd, i);
}


/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  Serial_SendArray(UART4,cmd, 4);
}




uint32_t MeasurePulseToLimit(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint16_t limitPin)
{
    uint32_t pulseCount = 0;  // 脉冲计数器
    uint32_t stepSize = 100;  // 每次增加的脉冲数
    uint8_t limitHit = 0;     // 限位开关触发标志

    OLED_Clear();  // 清屏
    OLED_ShowString(1, 1, "Measuring Pulses");  // 显示提示信息

    while (!limitHit)
    {
        // 发送位置控制命令
        Emm_V5_Pos_Control(addr, dir, vel, acc, pulseCount, 0, 0);

        // 延时等待电机运动
        delay_ms(50);

        // 检测限位开关
        if (GPIO_ReadInputDataBit(GPIOG, limitPin) == 0)  // 低电平触发
        {
            limitHit = 1;  // 限位开关触发
            Emm_V5_Stop_Now(addr, 0);  // 停止电机
            OLED_ShowString(2, 1, "Limit Hit!");  // 显示限位触发信息
        }
        else
        {
            pulseCount += stepSize;  // 增加脉冲数
            OLED_ShowNum(3, 1, pulseCount, 10);  // 显示当前脉冲数
        }
    }

    // 显示最终脉冲数
    OLED_ShowString(4, 1, "Final Pulses:");
    OLED_ShowNum(4, 14, pulseCount, 10);

    return pulseCount;  // 返回所需的脉冲数
}



void TestMeasurePulse(void)
{
    // 测量X轴电机从一端走到限位器所需的脉冲数
    uint32_t xPulses = MeasurePulseToLimit(1, 1, 100, 10, GPIO_Pin_10);

    // 测量Y轴电机从一端走到限位器所需的脉冲数
    //uint32_t yPulses = MeasurePulseToLimit(2, 1, 100, 10, GPIO_Pin_11);

    // 显示结果
    OLED_ShowString(5, 1, "X Pulses:");
    OLED_ShowNum(5, 10, xPulses, 10);
    OLED_ShowString(6, 1, "Y Pulses:");
    //OLED_ShowNum(6, 10, yPulses, 10);
}




//以下需要根据实际测试更改	
/**************************************************************************************************************************************/

/*void XY_GO_0(void) //上电后的默认操作，先让x,y电机归位
{
	// 初始运动：让电机向限位开关方向运动
    Emm_V5_Vel_Control(1, 0, 100, 10, 0); // 速度模式，电机1，方向0，速度100，加速度10
    Emm_V5_Vel_Control(2, 0, 100, 10, 0); // 速度模式，电机2，方向0，速度100，加速度10
    delay_ms(100); // 延时等待电机启动

    // 限位标志位清零
    XianWei_Flag = 0;

    // 检测限位开关
    while (!XianWei_Flag)
    {
        uint8_t x_xw = 0, y_xw = 0;

        // 检测X轴限位开关
        if (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) == 0) // 低电平触发
        {
            Emm_V5_Stop_Now(1, 0); // 电机1立刻停止
            delay_ms(20);
            x_xw = 1; // X轴限位标志位置1
        }

        // 检测Y轴限位开关
        if (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11) == 0) // 低电平触发
        {
            Emm_V5_Stop_Now(2, 0); // 电机2立刻停止
            delay_ms(20);
            y_xw = 1; // Y轴限位标志位置1
        }

        // 如果X轴和Y轴都触发限位开关，设置成功标志位
        if (x_xw == 1 && y_xw == 1)
        {
            XianWei_Flag = 1; // 限位成功
        }
    }

    // 限位成功后，电机停止
    Emm_V5_Stop_Now(1, 0); // 停止电机1
    Emm_V5_Stop_Now(2, 0); // 停止电机2

    // 重置坐标
    X = 0;
    Y = 0;

    delay_ms(1000); // 延时等待
}
*/
// OLED 日志显示函数
void OLED_ShowLog(uint8_t line, char* message)
{
    OLED_ShowString(line, 1, message);  // 在指定行显示日志信息
    delay_ms(100);  // 延时，避免刷新过快
}

void XY_GO_0(void) // 上电后的默认操作，先让x,y电机归位
{
    // 初始化限位开关
    Emm_Init();
    OLED_ShowLog(1, "Init Limit SW");  // 显示日志：初始化限位开关

    // X 轴矫正
    OLED_ShowLog(2, "X Motor Start");  // 显示日志：X轴电机启动
    Emm_V5_Vel_Control(1, 1, 100, 10, 0); // 速度模式，电机1，方向1，速度100，加速度10
    delay_ms(100); // 延时等待电机启动

    // 检测X轴限位开关
    OLED_ShowLog(3, "Checking X Limit");  // 显示日志：开始检测X轴限位开关
    while (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11) != 0) // 等待X轴限位开关触发（低电平）
    {
        OLED_ShowLog(4, "X Moving...");  // 显示日志：X轴电机运动中
        delay_ms(10); // 延时，避免过于频繁的检测
    }

    // X轴限位开关触发
    Emm_V5_Stop_Now(1, 0); // 电机1立刻停止
    OLED_ShowLog(1, "X Limit Hit");  // 显示日志：X轴限位触发
    delay_ms(20);

    // Y 轴矫正
    OLED_ShowLog(2, "Y Motor Start");  // 显示日志：Y轴电机启动
    Emm_V5_Vel_Control(2, 1, 100, 10, 0); // 速度模式，电机2，方向1，速度100，加速度10
    delay_ms(100); // 延时等待电机启动

    // 检测Y轴限位开关
    OLED_ShowLog(3, "Checking Y Limit");  // 显示日志：开始检测Y轴限位开关
    while (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) != 0) // 等待Y轴限位开关触发（低电平）
    {
        OLED_ShowLog(4, "Y Moving...");  // 显示日志：Y轴电机运动中
        delay_ms(10); // 延时，避免过于频繁的检测
    }

    // Y轴限位开关触发
    Emm_V5_Stop_Now(2, 0); // 电机2立刻停止
    OLED_ShowLog(2, "Y Limit Hit");  // 显示日志：Y轴限位触发
    delay_ms(20);

    // 限位成功
    XianWei_Flag = 1; // 限位成功标志位置1
    OLED_ShowLog(3, "XianWei Success");  // 显示日志：限位成功

    // 限位成功后，电机停止
    Emm_V5_Stop_Now(1, 0); // 停止电机1
    Emm_V5_Stop_Now(2, 0); // 停止电机2
    OLED_ShowLog(4, "Motors Stopped");  // 显示日志：电机停止

    // 重置坐标
    X = 0;
    Y = 0;
    OLED_ShowLog(1, "Reset X=0, Y=0");  // 显示日志：坐标重置

    delay_ms(1000); // 延时等待
}

//脉冲从-850-850走全程
void X_GO(int16_t x) //x轴移动
{
	
	if(x>0)
	{
		Emm_V5_Pos_Control(1,1,x/8,100,x,0,0);//16  50   //位置模式 电机1，方向1，速度x/8 加速度100 脉冲数10*x 相对位置 取消多机同步
		delay_ms(50);
	}
	if(x<0)
	{
		Emm_V5_Pos_Control(1,0,(-x)/8,100,(-x),0,0);     //位置模式 电机1，方向0，速度-x/8 加速度100 脉冲数10*-x 相对位置 取消多机同步
		delay_ms(50);
	}
	
}

//脉冲从-17000-17000走全程
//
void Y_GO(int16_t x) //y轴移动
{
	if(x>0)
	{
		Emm_V5_Pos_Control(2,1,x/8,100,x,0,0);      //位置模式 电机2，方向1，速度x/8 加速度100 脉冲数10*x 相对位置 取消多机同步
		delay_ms(50);
	}
	if(x<0)
	{
		Emm_V5_Pos_Control(2,0,(-x)/8,100,(-x),0,0);//位置模式 电机2，方向0，速度-x/8 加速度100 脉冲数10*-x 相对位置 取消多机同步
		delay_ms(50);
	}
	
}

//脉冲从-850-850走全程
void X_GO_2(int16_t x) //x轴移动
{
	
	if(x>0)
	{
		Emm_V5_Pos_Control(1,1,x/8,100,x,0,1);//16  50   //位置模式 电机1，方向1，速度x/8 加速度100 脉冲数10*x 相对位置 多机同步
		delay_ms(50);
	}
	if(x<0)
	{
		Emm_V5_Pos_Control(1,0,(-x)/8,100,(-x),0,1);     //位置模式 电机1，方向0，速度-x/8 加速度100 脉冲数10*-x 相对位置 多机同步
		delay_ms(50);
	}
	
}

//脉冲从-17000-17000走全程
//
void Y_GO_2(int16_t x) //y轴移动
{
	if(x>0)
	{
		Emm_V5_Pos_Control(2,1,x/8,100,x,0,1);      //位置模式 电机2，方向1，速度x/8 加速度100 脉冲数10*x 相对位置 多机同步
		delay_ms(50);
	}
	if(x<0)
	{
		Emm_V5_Pos_Control(2,0,(-x)/8,100,(-x),0,1);//位置模式 电机2，方向0，速度-x/8 加速度100 脉冲数10*-x 相对位置 多机同步
		delay_ms(50);
	}
	
}

void XY_GO(int16_t x_want,int16_t y_want) //x轴和y轴一起运动
{
	//if(x_want<0) x_want=0; //如果x轴预期速度小于0，则让预期速度变为0，最小值为0
	//if(y_want<0) y_want=0; //如果y轴预期速度小于0，则让预期速度变为0，最小值为0
	//int16_t x_go,y_go;
	//x_go=x_want-X;//x轴的运行速度 = 预期速度 - x轴值
	X_GO_2(x_want);
	//y_go=y_want-Y;//y轴的运行速度 = 预期速度 - y轴值
	Y_GO_2(y_want);
	Emm_V5_Synchronous_motion(0);
	
}


/***********************************************************/

// 将摄像头坐标映射到电机脉冲数
void MapCameraToMotor(int16_t camera_x, int16_t camera_y, int16_t *motor_x, int16_t *motor_y)
{
    // 将摄像头坐标从0-640映射到0-15000
    *motor_x = (camera_x * MOTOR_PULSE_RANGE) / CAMERA_WIDTH;
    *motor_y = (camera_y * MOTOR_PULSE_RANGE) / CAMERA_HEIGHT;
}

// 将电机脉冲数映射回摄像头坐标
void MapMotorToCamera(int16_t motor_x, int16_t motor_y, int16_t *camera_x, int16_t *camera_y)
{
    // 将电机脉冲数从0-16500映射回0-640
    *camera_x = (motor_x * CAMERA_WIDTH) / MOTOR_PULSE_RANGE;
    *camera_y = (motor_y * CAMERA_HEIGHT) / MOTOR_PULSE_RANGE;
}


void MoveToCameraPosition(int16_t camera_x, int16_t camera_y)
{
    int16_t motor_x, motor_y;
    MapCameraToMotor(camera_x, camera_y, &motor_x, &motor_y);
    XY_GO(motor_x, motor_y);
}





void DebugMapping(int16_t camera_x, int16_t camera_y)
{
    int16_t motor_x, motor_y;
    int16_t mapped_camera_x, mapped_camera_y;
    char buf[20]; // 用于存储要显示的字符串

    // 映射摄像头坐标到电机脉冲数
    MapCameraToMotor(camera_x, camera_y, &motor_x, &motor_y);

    // 映射电机脉冲数回到摄像头坐标
    MapMotorToCamera(motor_x, motor_y, &mapped_camera_x, &mapped_camera_y);

    // 在 OLED 上显示调试信息
    OLED_Clear(); // 清屏，防止信息重叠

    sprintf(buf, "Cam: (%d,%d)", camera_x, camera_y);
    OLED_ShowString(0, 0, buf);

    sprintf(buf, "Mot: (%d,%d)", motor_x, motor_y);
    OLED_ShowString(0, 2, buf);

    sprintf(buf, "Map: (%d,%d)", mapped_camera_x, mapped_camera_y);
    OLED_ShowString(0, 4, buf);

    // 检查映射是否正确
    if (camera_x == mapped_camera_x && camera_y == mapped_camera_y)
    {
        OLED_ShowString(0, 6, "Map: OK");
    }
    else
    {
        OLED_ShowString(0, 6, "Map: Error!");
    }
}


//////////////////////////////////////////////////////////////////////
/**
  * @brief  将真实坐标转换为脉冲数
  * @param  realPos：真实坐标（单位：mm）
  * @param  axis：轴选择，0为X轴，1为Y轴
  * @retval 对应的脉冲数
  */
int16_t RealToPulse(float realPos, uint8_t axis)
{
    if (axis == 0)  // X轴
    {
        // X轴映射关系：真实坐标0 -> 0，真实坐标250 -> 850
        return (int16_t)((realPos / X_REAL) * X_PULSE);
    }
    else if (axis == 1)  // Y轴
    {
        // Y轴映射关系：真实坐标0 -> 0，真实坐标200 -> 17000
        return (int16_t)((realPos / Y_REAL) * Y_PULSE);
    }
    return 0;  // 默认返回0
}

void ShowSignedNum(uint8_t line, uint8_t column, int16_t value, uint8_t length)
{
    if (value < 0)  // 如果是负数
    {
        OLED_ShowChar(line, column, 'F');  // 显示负号 'F'
        value = -value;  // 将数字归正
        column++;  // 列位置右移一位
    }
    OLED_ShowNum(line, column, (uint32_t)value, length);  // 显示数字
}

/**
  * @brief  移动到指定真实坐标
  * @param  xPos：X轴目标坐标（单位：mm）
  * @param  yPos：Y轴目标坐标（单位：mm）
  */
/**********************************************************************/
void MoveToRealPosition(float xPos, float yPos)
{
    // 将真实坐标转换为脉冲数
    int16_t xPulse = RealToPulse(xPos, 0);
    int16_t yPulse = RealToPulse(yPos, 1);

    // 显示目标脉冲数和真实坐标（调试用）
    OLED_ShowString(1, 1, "X Pulse:");
    ShowSignedNum(1, 9, xPulse, 5);  // 显示带符号的 X 脉冲数

    OLED_ShowString(2, 1, "Y Pulse:");
    ShowSignedNum(2, 9, yPulse, 5);  // 显示带符号的 Y 脉冲数

    //OLED_ShowString(3, 1, "X Pos:");
    ShowSignedNum(3, 8, (int16_t)xPos, 5);  // 显示带符号的 X 坐标

    //OLED_ShowString(4, 1, "Y Pos:");
    ShowSignedNum(4, 8, (int16_t)yPos, 5);  // 显示带符号的 Y 坐标

    // 控制电机移动到目标位置
		xPulse = -xPulse;
		yPulse = -yPulse;
    X_GO_2(xPulse);
    Y_GO_2(yPulse);
    Emm_V5_Synchronous_motion(0);
}
/*****************************************************************************/

/**
  * @brief  调试映射关系
  */
void DebugMapping_real(void)
{
    // 测试X轴映射
    float xTestPos = 62.5f;  // 测试X轴坐标
    int16_t xTestPulse = RealToPulse(xTestPos, 0);
    //float xTestReal = PulseToReal(xTestPulse, 0);

    // 测试Y轴映射
    float yTestPos = -50.0f;  // 测试Y轴坐标
    int16_t yTestPulse = RealToPulse(yTestPos, 1);
    //float yTestReal = PulseToReal(yTestPulse, 1);

    // 显示测试结果
    OLED_ShowString(1, 1, "X Test:");
    OLED_ShowNum(1, 8, (int16_t)xTestPos, 5);
    OLED_ShowString(2, 1, "X Pulse:");
    OLED_ShowNum(2, 9, xTestPulse, 5);
    OLED_ShowString(3, 1, "X Real:");
    //OLED_ShowNum(3, 8, (int16_t)xTestReal, 5);

    OLED_ShowString(4, 1, "Y Test:");
    OLED_ShowNum(4, 8, (int16_t)yTestPos, 5);
    OLED_ShowString(5, 1, "Y Pulse:");
    OLED_ShowNum(5, 9, yTestPulse, 5);
    OLED_ShowString(6, 1, "Y Real:");
    //OLED_ShowNum(6, 8, (int16_t)yTestReal, 5);
}



