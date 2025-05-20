#ifndef __EMM_H_
#define __EMM_H_




extern int16_t X,Y;
extern uint8_t XianWei_Flag;



typedef enum {
	S_VER   = 0,			/* 读取固件版本和对应的硬件版本 */
	S_RL    = 1,			/* 读取读取相电阻和相电感 */
	S_PID   = 2,			/* 读取PID参数 */
	S_VBUS  = 3,			/* 读取总线电压 */
	S_CPHA  = 5,			/* 读取相电流 */
	S_ENCL  = 7,			/* 读取经过线性化校准后的编码器值 */
	S_TPOS  = 8,			/* 读取电机目标位置角度 */
	S_VEL   = 9,			/* 读取电机实时转速 */
	S_CPOS  = 10,			/* 读取电机实时位置角度 */
	S_PERR  = 11,			/* 读取电机位置误差角度 */
	S_FLAG  = 13,			/* 读取使能/到位/堵转状态标志位 */
	S_Conf  = 14,			/* 读取驱动参数 */
	S_State = 15,			/* 读取系统状态参数 */
	S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
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
