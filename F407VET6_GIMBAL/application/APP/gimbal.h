#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"
#include "usart_rc_dbus.h"
#include "pid.h"
#include "can_receive.h"
#include "math.h"
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"
#include "function.h"
#include "ins_task.h"
#include "USART_task.h"

#define Radius 1//车体机械半径
#define wheel_rpm_ratio 36.0//3508减速比
#define rotate_ratio_f 1
#define rotate_ratio_b 1

//云台机械数据结构体
typedef struct
{
	float pitch_H_angle;//pitch高端限位
	float pitch_L_angle;//
	
	float yaw_H_angle;//pitch高端限位
	float yaw_L_angle;//
}Mechanical_measure_t;
extern Mechanical_measure_t mechanical_data;

//舵轮底盘数据结构体
typedef struct
{
	
}ChassisHandle_t;
extern ChassisHandle_t ChassisHandle;


//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
double msp(double x, double min_in, double max_in, double min_out, double max_out);
//底盘参数初始化
void gimbal_init(void);
//底盘运动函数
void gimbal_task(void);
//麦轮电机速度解算函数
void Steer_Calculate(ChassisHandle_t *chassishandle,int16_t vx,int16_t vy,int16_t vw);

extern float target_angle[2];

#endif



