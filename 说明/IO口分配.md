# I/O口分配

## 串口通讯

主控板和树莓派通讯



![image-20250221203515742](D:\ZILIAO\比赛文件夹\工训\比赛资料\文档\IO口分配.assets\image-20250221203515742.png)

 STM32						树莓派				

PA9	RX						   10	TX

PA10	TX						8	RX

​								串口屏 

PA2							

PA3

## 舵机控制

云台180度舵机				PA6

云台270度舵机				PA1

## 限位器

PG10	X轴限位器

PG11	Y轴限位器









# 新版

## 串口通讯分配

## usart1

用于和树莓派进行通讯。

## usart2

用于和舵机驱动板通讯。

## usart3

测试有问题)(备用)

## usart4

用于和电机通讯

## usart5

用于和串口屏通讯。3
