#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include "Emm.h"
#include "XianWei.h"
#include "Delay.h"
#include "OLED.h"

#define X_DaoWei while(GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_11)==0) //���x�����Ƿ��˶����涨λ�ã��Ӳ������dir
#define Y_DaoWei while(GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_10)==0) //���y�����Ƿ��˶����涨λ��
	
#define MOTOR_RANGE 825		//������巶Χ
#define CAMERA_WIDTH 640	//����ͷʶ�𵽵�ͼ��λ�� ��
#define CAMERA_HEIGHT 640	//����ͷʶ�𵽵�ͼ��λ�� ��

#define MOTOR_PULSE_RANGE 15000	//	����һ�˵���һ�˵����������

// �������巶Χ

#define X_PULSE 850

#define Y_PULSE 17000

// ������ʵ���귶Χ

#define X_REAL 125

#define Y_REAL 100

int16_t X=0,Y=0;


uint8_t XianWei_Flag=0;//��λ�ɹ���־λ

void Emm_Init(void)// PD10 PD11
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);  // ʹ�� GPIOG ʱ��

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // ��������ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;  // ���� PG10 �� PG11
    GPIO_Init(GPIOG, &GPIO_InitStructure);
}


// ������λ���ذ��µļ�����
uint16_t X_LimitCount = 0;
uint16_t Y_LimitCount = 0;
void TestLimitSwitches(void)
{
    // ��ʼ����λ��������
    Emm_Init();

    while (1)
    {
        // ���X����λ����
        if (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) == 0)
        {
            X_LimitCount++;  // ����X����λ���ذ��¼���
            OLED_ShowNum(1, 7, X_LimitCount, 5);  // ��OLED����ʾX����λ���ذ��´���
            while (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) == 0);  // �ȴ���λ�����ͷ�
        }

        // ���Y����λ����
        if (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11) == 0)
        {
            Y_LimitCount++;  // ����Y����λ���ذ��¼���
            OLED_ShowNum(2, 7, Y_LimitCount, 5);  // ��OLED����ʾY����λ���ذ��´���
            while (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11) == 0);  // �ȴ���λ�����ͷ�
        }

        // ����ʵ�����ʱ���������Ƶ���ļ��
        delay_ms(10);  // ��������һ����ʱ����
    }
}

/**
  * @brief    �ٶ�ģʽ
  * @param    addr�������ַ
  * @param    dir ������       ��0ΪCW������ֵΪCCW
  * @param    vel ���ٶ�       ����Χ0 - 5000RPM
  * @param    acc �����ٶ�     ����Χ0 - 255��ע�⣺0��ֱ������
  * @param    snF �����ͬ����־��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF)
{
  uint8_t cmd[16] = {0};

  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xF6;                       // ������
  cmd[2] =  dir;                        // ����
  cmd[3] =  (uint8_t)(vel >> 8);        // �ٶ�(RPM)��8λ�ֽ�
  cmd[4] =  (uint8_t)(vel >> 0);        // �ٶ�(RPM)��8λ�ֽ�
  cmd[5] =  acc;                        // ���ٶȣ�ע�⣺0��ֱ������
  cmd[6] =  snF;                        // ���ͬ���˶���־
  cmd[7] =  0x6B;                       // У���ֽ�
  
  // ��������
  Serial_SendArray(UART4,cmd, 8);
}


/**
  * @brief    λ��ģʽ
  * @param    addr�������ַ
  * @param    dir ������        ��0ΪCW������ֵΪCCW
  * @param    vel ���ٶ�(RPM)   ����Χ0 - 5000RPM
  * @param    acc �����ٶ�      ����Χ0 - 255��ע�⣺0��ֱ������
  * @param    clk ��������      ����Χ0- (2^32 - 1)��
  * @param    raF ����λ/���Ա�־��falseΪ����˶���trueΪ����ֵ�˶�
  * @param    snF �����ͬ����־ ��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, uint8_t snF)
{
  uint8_t cmd[16] = {0};

  // װ������
  cmd[0]  =  addr;                      // ��ַ
  cmd[1]  =  0xFD;                      // ������
  cmd[2]  =  dir;                       // ����
  cmd[3]  =  (uint8_t)(vel >> 8);       // �ٶ�(RPM)��8λ�ֽ�
  cmd[4]  =  (uint8_t)(vel >> 0);       // �ٶ�(RPM)��8λ�ֽ� 
  cmd[5]  =  acc;                       // ���ٶȣ�ע�⣺0��ֱ������
  cmd[6]  =  (uint8_t)(clk >> 24);      // ������(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // ������(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // ������(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // ������(bit0  - bit7 )
  cmd[10] =  raF;                       // ��λ/���Ա�־��falseΪ����˶���trueΪ����ֵ�˶�
  cmd[11] =  snF;                       // ���ͬ���˶���־��falseΪ�����ã�trueΪ����
  cmd[12] =  0x6B;                      // У���ֽ�
  
  // ��������
  Serial_SendArray(UART4,cmd, 13);
}

/**
  * @brief    ����ֹͣ�����п���ģʽ��ͨ�ã�
  * @param    addr  �������ַ
  * @param    snF   �����ͬ����־��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Stop_Now(uint8_t addr, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xFE;                       // ������
  cmd[2] =  0x98;                       // ������
  cmd[3] =  snF;                        // ���ͬ���˶���־
  cmd[4] =  0x6B;                       // У���ֽ�
  
  // ��������
  Serial_SendArray(UART4,cmd, 5);
}

/**
  * @brief    ʹ���źſ���
  * @param    addr  �������ַ
  * @param    state ��ʹ��״̬     ��trueΪʹ�ܵ����falseΪ�رյ��
  * @param    snF   �����ͬ����־ ��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_En_Control(uint8_t addr, uint8_t state, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xF3;                       // ������
  cmd[2] =  0xAB;                       // ������
  cmd[3] =  (uint8_t)state;             // ʹ��״̬
  cmd[4] =  snF;                        // ���ͬ���˶���־
  cmd[5] =  0x6B;                       // У���ֽ�
  
  // ��������
  Serial_SendArray(UART4,cmd, 6);
}


/**
  * @brief    ��ȡϵͳ����
  * @param    addr  �������ַ
  * @param    s     ��ϵͳ��������
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0; 
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[i] = addr; ++i;                   // ��ַ

  switch(s)                             // ������
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

  cmd[i] = 0x6B; ++i;                   // У���ֽ�
  
  // ��������
  Serial_SendArray(UART4,cmd, i);
}


/**
  * @brief    ���ͬ���˶�
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xFF;                       // ������
  cmd[2] =  0x66;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  Serial_SendArray(UART4,cmd, 4);
}




uint32_t MeasurePulseToLimit(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint16_t limitPin)
{
    uint32_t pulseCount = 0;  // ���������
    uint32_t stepSize = 100;  // ÿ�����ӵ�������
    uint8_t limitHit = 0;     // ��λ���ش�����־

    OLED_Clear();  // ����
    OLED_ShowString(1, 1, "Measuring Pulses");  // ��ʾ��ʾ��Ϣ

    while (!limitHit)
    {
        // ����λ�ÿ�������
        Emm_V5_Pos_Control(addr, dir, vel, acc, pulseCount, 0, 0);

        // ��ʱ�ȴ�����˶�
        delay_ms(50);

        // �����λ����
        if (GPIO_ReadInputDataBit(GPIOG, limitPin) == 0)  // �͵�ƽ����
        {
            limitHit = 1;  // ��λ���ش���
            Emm_V5_Stop_Now(addr, 0);  // ֹͣ���
            OLED_ShowString(2, 1, "Limit Hit!");  // ��ʾ��λ������Ϣ
        }
        else
        {
            pulseCount += stepSize;  // ����������
            OLED_ShowNum(3, 1, pulseCount, 10);  // ��ʾ��ǰ������
        }
    }

    // ��ʾ����������
    OLED_ShowString(4, 1, "Final Pulses:");
    OLED_ShowNum(4, 14, pulseCount, 10);

    return pulseCount;  // ���������������
}



void TestMeasurePulse(void)
{
    // ����X������һ���ߵ���λ�������������
    uint32_t xPulses = MeasurePulseToLimit(1, 1, 100, 10, GPIO_Pin_10);

    // ����Y������һ���ߵ���λ�������������
    //uint32_t yPulses = MeasurePulseToLimit(2, 1, 100, 10, GPIO_Pin_11);

    // ��ʾ���
    OLED_ShowString(5, 1, "X Pulses:");
    OLED_ShowNum(5, 10, xPulses, 10);
    OLED_ShowString(6, 1, "Y Pulses:");
    //OLED_ShowNum(6, 10, yPulses, 10);
}




//������Ҫ����ʵ�ʲ��Ը���	
/**************************************************************************************************************************************/

/*void XY_GO_0(void) //�ϵ���Ĭ�ϲ���������x,y�����λ
{
	// ��ʼ�˶����õ������λ���ط����˶�
    Emm_V5_Vel_Control(1, 0, 100, 10, 0); // �ٶ�ģʽ�����1������0���ٶ�100�����ٶ�10
    Emm_V5_Vel_Control(2, 0, 100, 10, 0); // �ٶ�ģʽ�����2������0���ٶ�100�����ٶ�10
    delay_ms(100); // ��ʱ�ȴ��������

    // ��λ��־λ����
    XianWei_Flag = 0;

    // �����λ����
    while (!XianWei_Flag)
    {
        uint8_t x_xw = 0, y_xw = 0;

        // ���X����λ����
        if (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) == 0) // �͵�ƽ����
        {
            Emm_V5_Stop_Now(1, 0); // ���1����ֹͣ
            delay_ms(20);
            x_xw = 1; // X����λ��־λ��1
        }

        // ���Y����λ����
        if (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11) == 0) // �͵�ƽ����
        {
            Emm_V5_Stop_Now(2, 0); // ���2����ֹͣ
            delay_ms(20);
            y_xw = 1; // Y����λ��־λ��1
        }

        // ���X���Y�ᶼ������λ���أ����óɹ���־λ
        if (x_xw == 1 && y_xw == 1)
        {
            XianWei_Flag = 1; // ��λ�ɹ�
        }
    }

    // ��λ�ɹ��󣬵��ֹͣ
    Emm_V5_Stop_Now(1, 0); // ֹͣ���1
    Emm_V5_Stop_Now(2, 0); // ֹͣ���2

    // ��������
    X = 0;
    Y = 0;

    delay_ms(1000); // ��ʱ�ȴ�
}
*/
// OLED ��־��ʾ����
void OLED_ShowLog(uint8_t line, char* message)
{
    OLED_ShowString(line, 1, message);  // ��ָ������ʾ��־��Ϣ
    delay_ms(100);  // ��ʱ������ˢ�¹���
}

void XY_GO_0(void) // �ϵ���Ĭ�ϲ���������x,y�����λ
{
    // ��ʼ����λ����
    Emm_Init();
    OLED_ShowLog(1, "Init Limit SW");  // ��ʾ��־����ʼ����λ����

    // X �����
    OLED_ShowLog(2, "X Motor Start");  // ��ʾ��־��X��������
    Emm_V5_Vel_Control(1, 1, 100, 10, 0); // �ٶ�ģʽ�����1������1���ٶ�100�����ٶ�10
    delay_ms(100); // ��ʱ�ȴ��������

    // ���X����λ����
    OLED_ShowLog(3, "Checking X Limit");  // ��ʾ��־����ʼ���X����λ����
    while (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11) != 0) // �ȴ�X����λ���ش������͵�ƽ��
    {
        OLED_ShowLog(4, "X Moving...");  // ��ʾ��־��X�����˶���
        delay_ms(10); // ��ʱ���������Ƶ���ļ��
    }

    // X����λ���ش���
    Emm_V5_Stop_Now(1, 0); // ���1����ֹͣ
    OLED_ShowLog(1, "X Limit Hit");  // ��ʾ��־��X����λ����
    delay_ms(20);

    // Y �����
    OLED_ShowLog(2, "Y Motor Start");  // ��ʾ��־��Y��������
    Emm_V5_Vel_Control(2, 1, 100, 10, 0); // �ٶ�ģʽ�����2������1���ٶ�100�����ٶ�10
    delay_ms(100); // ��ʱ�ȴ��������

    // ���Y����λ����
    OLED_ShowLog(3, "Checking Y Limit");  // ��ʾ��־����ʼ���Y����λ����
    while (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) != 0) // �ȴ�Y����λ���ش������͵�ƽ��
    {
        OLED_ShowLog(4, "Y Moving...");  // ��ʾ��־��Y�����˶���
        delay_ms(10); // ��ʱ���������Ƶ���ļ��
    }

    // Y����λ���ش���
    Emm_V5_Stop_Now(2, 0); // ���2����ֹͣ
    OLED_ShowLog(2, "Y Limit Hit");  // ��ʾ��־��Y����λ����
    delay_ms(20);

    // ��λ�ɹ�
    XianWei_Flag = 1; // ��λ�ɹ���־λ��1
    OLED_ShowLog(3, "XianWei Success");  // ��ʾ��־����λ�ɹ�

    // ��λ�ɹ��󣬵��ֹͣ
    Emm_V5_Stop_Now(1, 0); // ֹͣ���1
    Emm_V5_Stop_Now(2, 0); // ֹͣ���2
    OLED_ShowLog(4, "Motors Stopped");  // ��ʾ��־�����ֹͣ

    // ��������
    X = 0;
    Y = 0;
    OLED_ShowLog(1, "Reset X=0, Y=0");  // ��ʾ��־����������

    delay_ms(1000); // ��ʱ�ȴ�
}

//�����-850-850��ȫ��
void X_GO(int16_t x) //x���ƶ�
{
	
	if(x>0)
	{
		Emm_V5_Pos_Control(1,1,x/8,100,x,0,0);//16  50   //λ��ģʽ ���1������1���ٶ�x/8 ���ٶ�100 ������10*x ���λ�� ȡ�����ͬ��
		delay_ms(50);
	}
	if(x<0)
	{
		Emm_V5_Pos_Control(1,0,(-x)/8,100,(-x),0,0);     //λ��ģʽ ���1������0���ٶ�-x/8 ���ٶ�100 ������10*-x ���λ�� ȡ�����ͬ��
		delay_ms(50);
	}
	
}

//�����-17000-17000��ȫ��
//
void Y_GO(int16_t x) //y���ƶ�
{
	if(x>0)
	{
		Emm_V5_Pos_Control(2,1,x/8,100,x,0,0);      //λ��ģʽ ���2������1���ٶ�x/8 ���ٶ�100 ������10*x ���λ�� ȡ�����ͬ��
		delay_ms(50);
	}
	if(x<0)
	{
		Emm_V5_Pos_Control(2,0,(-x)/8,100,(-x),0,0);//λ��ģʽ ���2������0���ٶ�-x/8 ���ٶ�100 ������10*-x ���λ�� ȡ�����ͬ��
		delay_ms(50);
	}
	
}

//�����-850-850��ȫ��
void X_GO_2(int16_t x) //x���ƶ�
{
	
	if(x>0)
	{
		Emm_V5_Pos_Control(1,1,x/8,100,x,0,1);//16  50   //λ��ģʽ ���1������1���ٶ�x/8 ���ٶ�100 ������10*x ���λ�� ���ͬ��
		delay_ms(50);
	}
	if(x<0)
	{
		Emm_V5_Pos_Control(1,0,(-x)/8,100,(-x),0,1);     //λ��ģʽ ���1������0���ٶ�-x/8 ���ٶ�100 ������10*-x ���λ�� ���ͬ��
		delay_ms(50);
	}
	
}

//�����-17000-17000��ȫ��
//
void Y_GO_2(int16_t x) //y���ƶ�
{
	if(x>0)
	{
		Emm_V5_Pos_Control(2,1,x/8,100,x,0,1);      //λ��ģʽ ���2������1���ٶ�x/8 ���ٶ�100 ������10*x ���λ�� ���ͬ��
		delay_ms(50);
	}
	if(x<0)
	{
		Emm_V5_Pos_Control(2,0,(-x)/8,100,(-x),0,1);//λ��ģʽ ���2������0���ٶ�-x/8 ���ٶ�100 ������10*-x ���λ�� ���ͬ��
		delay_ms(50);
	}
	
}

void XY_GO(int16_t x_want,int16_t y_want) //x���y��һ���˶�
{
	//if(x_want<0) x_want=0; //���x��Ԥ���ٶ�С��0������Ԥ���ٶȱ�Ϊ0����СֵΪ0
	//if(y_want<0) y_want=0; //���y��Ԥ���ٶ�С��0������Ԥ���ٶȱ�Ϊ0����СֵΪ0
	//int16_t x_go,y_go;
	//x_go=x_want-X;//x��������ٶ� = Ԥ���ٶ� - x��ֵ
	X_GO_2(x_want);
	//y_go=y_want-Y;//y��������ٶ� = Ԥ���ٶ� - y��ֵ
	Y_GO_2(y_want);
	Emm_V5_Synchronous_motion(0);
	
}


/***********************************************************/

// ������ͷ����ӳ�䵽���������
void MapCameraToMotor(int16_t camera_x, int16_t camera_y, int16_t *motor_x, int16_t *motor_y)
{
    // ������ͷ�����0-640ӳ�䵽0-15000
    *motor_x = (camera_x * MOTOR_PULSE_RANGE) / CAMERA_WIDTH;
    *motor_y = (camera_y * MOTOR_PULSE_RANGE) / CAMERA_HEIGHT;
}

// �����������ӳ�������ͷ����
void MapMotorToCamera(int16_t motor_x, int16_t motor_y, int16_t *camera_x, int16_t *camera_y)
{
    // �������������0-16500ӳ���0-640
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
    char buf[20]; // ���ڴ洢Ҫ��ʾ���ַ���

    // ӳ������ͷ���굽���������
    MapCameraToMotor(camera_x, camera_y, &motor_x, &motor_y);

    // ӳ�����������ص�����ͷ����
    MapMotorToCamera(motor_x, motor_y, &mapped_camera_x, &mapped_camera_y);

    // �� OLED ����ʾ������Ϣ
    OLED_Clear(); // ��������ֹ��Ϣ�ص�

    sprintf(buf, "Cam: (%d,%d)", camera_x, camera_y);
    OLED_ShowString(0, 0, buf);

    sprintf(buf, "Mot: (%d,%d)", motor_x, motor_y);
    OLED_ShowString(0, 2, buf);

    sprintf(buf, "Map: (%d,%d)", mapped_camera_x, mapped_camera_y);
    OLED_ShowString(0, 4, buf);

    // ���ӳ���Ƿ���ȷ
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
  * @brief  ����ʵ����ת��Ϊ������
  * @param  realPos����ʵ���꣨��λ��mm��
  * @param  axis����ѡ��0ΪX�ᣬ1ΪY��
  * @retval ��Ӧ��������
  */
int16_t RealToPulse(float realPos, uint8_t axis)
{
    if (axis == 0)  // X��
    {
        // X��ӳ���ϵ����ʵ����0 -> 0����ʵ����250 -> 850
        return (int16_t)((realPos / X_REAL) * X_PULSE);
    }
    else if (axis == 1)  // Y��
    {
        // Y��ӳ���ϵ����ʵ����0 -> 0����ʵ����200 -> 17000
        return (int16_t)((realPos / Y_REAL) * Y_PULSE);
    }
    return 0;  // Ĭ�Ϸ���0
}

void ShowSignedNum(uint8_t line, uint8_t column, int16_t value, uint8_t length)
{
    if (value < 0)  // ����Ǹ���
    {
        OLED_ShowChar(line, column, 'F');  // ��ʾ���� 'F'
        value = -value;  // �����ֹ���
        column++;  // ��λ������һλ
    }
    OLED_ShowNum(line, column, (uint32_t)value, length);  // ��ʾ����
}

/**
  * @brief  �ƶ���ָ����ʵ����
  * @param  xPos��X��Ŀ�����꣨��λ��mm��
  * @param  yPos��Y��Ŀ�����꣨��λ��mm��
  */
/**********************************************************************/
void MoveToRealPosition(float xPos, float yPos)
{
    // ����ʵ����ת��Ϊ������
    int16_t xPulse = RealToPulse(xPos, 0);
    int16_t yPulse = RealToPulse(yPos, 1);

    // ��ʾĿ������������ʵ���꣨�����ã�
    OLED_ShowString(1, 1, "X Pulse:");
    ShowSignedNum(1, 9, xPulse, 5);  // ��ʾ�����ŵ� X ������

    OLED_ShowString(2, 1, "Y Pulse:");
    ShowSignedNum(2, 9, yPulse, 5);  // ��ʾ�����ŵ� Y ������

    //OLED_ShowString(3, 1, "X Pos:");
    ShowSignedNum(3, 8, (int16_t)xPos, 5);  // ��ʾ�����ŵ� X ����

    //OLED_ShowString(4, 1, "Y Pos:");
    ShowSignedNum(4, 8, (int16_t)yPos, 5);  // ��ʾ�����ŵ� Y ����

    // ���Ƶ���ƶ���Ŀ��λ��
		xPulse = -xPulse;
		yPulse = -yPulse;
    X_GO_2(xPulse);
    Y_GO_2(yPulse);
    Emm_V5_Synchronous_motion(0);
}
/*****************************************************************************/

/**
  * @brief  ����ӳ���ϵ
  */
void DebugMapping_real(void)
{
    // ����X��ӳ��
    float xTestPos = 62.5f;  // ����X������
    int16_t xTestPulse = RealToPulse(xTestPos, 0);
    //float xTestReal = PulseToReal(xTestPulse, 0);

    // ����Y��ӳ��
    float yTestPos = -50.0f;  // ����Y������
    int16_t yTestPulse = RealToPulse(yTestPos, 1);
    //float yTestReal = PulseToReal(yTestPulse, 1);

    // ��ʾ���Խ��
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



