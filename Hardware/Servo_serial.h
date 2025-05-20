#ifndef __SERVO_SERIAL_H
#define __SERVO_SERIAL_H

typedef enum {
    ACTION_GROUP_1 = 0,  // 动作组 1
    ACTION_GROUP_2 = 1,  // 动作组 2
    ACTION_GROUP_3 = 2,  // 动作组 3
    ACTION_GROUP_4 = 3,  // 动作组 4
} ActionGroup;




uint16_t angle_to_pwm_180(uint16_t angle);
uint16_t angle_to_pwm_270(uint16_t angle);
void send_servo_command(uint8_t index, uint16_t angle, uint16_t time, uint8_t is_270_servo);
void send_action_group_command(ActionGroup group);
void send_multiple_servo_commands(char *commands);
void send_action_group_command(ActionGroup group);
void TiltTray_Serial(TiltDirection direction);
void TiltTray_Serial_zhua(int type);
void InitializeMechanicalClaw_serial();
int constrain_serial(int value, int minValue, int maxValue);
int map_serial(int value, int fromLow, int fromHigh, int toLow, int toHigh) ;
void RescueAction();
void ControlZAxis_serial(int extendLength);
void ControlRotation_serial(int rotateAngle);
void ControlMechanicalClaw_serial(int rotateAngle, int extendLength, int objectSize);
void GrabObject_serial(int rotateAngle, int extendLength, int objectSize);
void ReleaseObject_serial();


#endif

