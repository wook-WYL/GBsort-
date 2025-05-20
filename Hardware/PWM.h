#ifndef __PWM_H
#define __PWM_H

void PWM_Init_PA1(void);
void PWM_SetCompare2(uint16_t Compare);
void PWM_Init_PA6_PA7(void);
void PWM_Init_PB0_PB1_PC6(void);

void PWM_SetCompare_PA1(uint16_t Compare);
void PWM_SetCompare_PA6(uint16_t Compare);
void PWM_SetCompare_PA7(uint16_t Compare);
void PWM_SetCompare_PB0(uint16_t Compare);
void PWM_SetCompare_PB1(uint16_t Compare);
void PWM_SetCompare_PC6(uint16_t Compare);


#endif
