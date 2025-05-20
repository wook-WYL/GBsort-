#ifndef __EMM_H_
#define __EMM_H_




extern int16_t X,Y;
extern uint8_t XianWei_Flag;



typedef enum {
	S_VER   = 0,			/* ��ȡ�̼��汾�Ͷ�Ӧ��Ӳ���汾 */
	S_RL    = 1,			/* ��ȡ��ȡ���������� */
	S_PID   = 2,			/* ��ȡPID���� */
	S_VBUS  = 3,			/* ��ȡ���ߵ�ѹ */
	S_CPHA  = 5,			/* ��ȡ����� */
	S_ENCL  = 7,			/* ��ȡ�������Ի�У׼��ı�����ֵ */
	S_TPOS  = 8,			/* ��ȡ���Ŀ��λ�ýǶ� */
	S_VEL   = 9,			/* ��ȡ���ʵʱת�� */
	S_CPOS  = 10,			/* ��ȡ���ʵʱλ�ýǶ� */
	S_PERR  = 11,			/* ��ȡ���λ�����Ƕ� */
	S_FLAG  = 13,			/* ��ȡʹ��/��λ/��ת״̬��־λ */
	S_Conf  = 14,			/* ��ȡ�������� */
	S_State = 15,			/* ��ȡϵͳ״̬���� */
	S_ORG   = 16,     /* ��ȡ���ڻ���/����ʧ��״̬��־λ */
}SysParams_t;

void Emm_Init(void);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, uint8_t snF);
void Emm_V5_Stop_Now(uint8_t addr, uint8_t snF);
void Emm_V5_En_Control(uint8_t addr, uint8_t state, uint8_t snF);
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
uint32_t MeasurePulseToLimit(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint16_t limitPin);
void TestMeasurePulse(void);
void Emm_V5_Synchronous_motion(uint8_t addr);

void XY_GO_0(void);
void X_GO(int16_t x);
void Y_GO(int16_t x);
void XY_GO(int16_t x_want,int16_t y_want);
void Y_GO_2(int16_t x);
void X_GO_2(int16_t x);

void MapCameraToMotor(int16_t camera_x, int16_t camera_y, int16_t *motor_x, int16_t *motor_y);
void MapMotorToCamera(int16_t motor_x, int16_t motor_y, int16_t *camera_x, int16_t *camera_y);
void MoveToCameraPosition(int16_t camera_x, int16_t camera_y);
void TestLimitSwitches(void);

int16_t RealToPulse(float realPos, uint8_t axis);
void MoveToRealPosition(float xPos, float yPos);
void DebugMapping_real(void);
int16_t RealToPulse(float realPos, uint8_t axis);

#endif
