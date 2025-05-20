#include "stm32f10x.h"
#include "PWM.h" // ????????????????????
#include "delay.h"
#include "OLED.h"
#include "Servo.h"
#include "Serial.h"
#include "ultrasonic.h"
#include "CKP.h"
#include "Serial.h"
#include "Serial_5.h"
#include "Emm.h"
#include "Servo_serial.h"
float distance;
uint8_t flag = 0;








int main(void)
{
	
	
	//????????
	//?????????????
	delay_init(); //????????????
	//????????
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	usart1_init(); //????????????
	usart2_init(); //???????????????
	usart3_init(); //?????????
	usart4_init(); //????????????
	usart5_init();
	//????????
	Servo_Init();	//???????????
	OLED_Init();	// OLED?????
	Emm_Init();		//????????????
	HC_SR04_Init(); //?????????????
	//	delay_ms(1000);

	//???��?��????
	OLED_ShowString(1, 1, "START1");
	send_servo_command(0, 65, 200, 1);//��ά��̨��ʼ��
	delay_ms(1000);
	OLED_ShowString(1, 1, "START2");
	InitializeMechanicalClaw_serial();//��צλ�ó�ʼ��
	delay_ms(1000);
	OLED_ShowString(1, 1, "START3");
	XY_GO_0();//���λ�ó�ʼ��
	delay_ms(1000);








	// Servo_SetAngle_180(90);
	// delay_ms(500);

	// Servo_SetAngle_270(0);
	// Servo_SetAngle_Zhua_F(90);
	// Servo_SetAngle_Zhua_M(180);
	// Servo_SetAngle_Zhua_E(0);
	// Servo_SetAngle_180(180);
	// delay_ms(500);
	//TIM5_Init();
	OLED_ShowString(1, 1, "START4");
	
	// OLED_ShowString(3, 1, "RxPacket");

	// SystemInit();
	// delay_init();
	// Ultrasonic_Init();
	// Serial_SendSuccess(USART1);
	// SendManZai();
	// Serial_SendSuccess(USART3);
	// XY_GO_0();
	// OLED_Init();
	// delay_ms(1000);
	// OLED_ShowString(2, 1, "YGO ");
	 //X_GO(-100);
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);
	//InitializeMechanicalClaw_serial();
	// OLED_ShowString(3, 1, "XGO ");
	//Y_GO(-100);
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);
	// OLED_ShowString(4, 1, "SERIAL ");
	// SendView();
	// ControlRotation_serial(90);
	/*TiltTray_Serial(DIRECTION_45);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	*/
	//FullLoadDetection(50);
	//sonar_mm();
	//XY_GO_0();
	//��λ���ز��Ժ���
	// ��ʾ��ʼ����
    //OLED_ShowString(1, 1, "X Count:");
    //OLED_ShowString(2, 1, "Y Count:");
    //OLED_ShowNum(1, 7, X_LimitCount, 5);
    //OLED_ShowNum(2, 7, Y_LimitCount, 5);

    // ��ʼ������λ����
    //TestLimitSwitches();
		//���Эͬ����
		
		//X_GO_2(-68);
    //Y_GO_2(-5000);
    //Emm_V5_Synchronous_motion(0);
		
		 //TestMeasurePulse();
		 OLED_Clear();
		 OLED_ShowString(1, 1, "START6");
			//MoveToRealPosition(62,50);
			//delay_ms(1000);
			//delay_ms(1000);
			//delay_ms(1000);
			//delay_ms(1000);
		 //ControlZAxis_serial(45);
		 //ControlRotation_serial(45);
		 //ControlMechanicalClaw_serial(45,45,90);
		 //GrabObject_serial(45,42,90);
		 
			//OLED_Clear();
			//Grab_main(400,8000,45,80,0);
	while (1)
	{
		//������
		OLED_ShowString(1, 1, "START7");
		if (Serial_GetRxFlag(USART1))	
		
			{
					SendView();
					OLED_ShowString(2, 1, "RECEVIE");
					ProcessReceivedData(USART1, Serial_RxPacket_USART1);
					//Serial_SendRestart(USART1);
					//int distance = sonar_mm();
					//OLED_Clear();
					//OLED_ShowString(1, 1, "Dist: ");
					//OLED_ShowNum(1, 7, distance, 5);  
					//OLED_ShowString(1, 1, "START4");
						//if (IsFullLoad(distance)) 
						//{  
						//		FullLoadDetection();  // **������أ�����ȴ�״̬**
						//		OLED_ShowString(1, 1, "START5");
						//}
					
					
				}
	
					
		
				
				//���ز��Դ���
			/*	int distance = sonar_mm();  // **�ɼ���ǰ����**
        
        // **��ʾ��ǰ����**
        OLED_Clear();
        OLED_ShowString(1, 1, "Dist: ");
        OLED_ShowNum(1, 7, distance, 5);  
        OLED_ShowString(1, 1, "START4");
        if (IsFullLoad(distance)) {  
            FullLoadDetection();  // **������أ�����ȴ�״̬**
						OLED_ShowString(1, 1, "START5");
        }
        
        delay_ms(100);
				*/
					

	}
}





