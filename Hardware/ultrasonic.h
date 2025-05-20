#ifndef __ultrasonic_H
#define __ultrasonic_H

void HC_SR04_Init(void);
int16_t sonar_mm(void);
float sonar(void);
void TI5_IRQHandler(void);
int IsFullLoad(int distance_mm);
int GetAverageDistance(void);
int FilterDistance(int new_distance);
void OLED_DisplayDistance(int distance_mm) ;
void OLED_DisplayFullLoad(void);
void TriggerAlarm(void) ;
void FullLoadDetection(void) ;
void TIM5_Init(void);
	
#endif
