#ifndef __PCA9685_H
#define __PCA9685_H
#include "sys.h"
 

 
void PCA9685_Init(float hz,u16 angle);
 
void PCA9685_Write(u8 addr,u8 data);
 
u8 PCA9685_Read(u8 addr);
 
void PCA9685_setPWM(u8 num,u32 on,u32 off);
 
void PCA9685_setFreq(float freq);
 
void set180DegreeServoAngle(u8 num,u16 angle);

void set270DegreeServoAngle(u8 num,u16 angle);

#endif
