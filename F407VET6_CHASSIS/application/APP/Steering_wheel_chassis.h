#ifndef STEERING_WHEEL_CHASSIS_H
#define STEERING_WHEEL_CHASSIS_H

#include "main.h"
#include "usart_rc_dbus.h"
#include "pid.h"
#include "can_receive.h"
#include "math.h"
#include "usart.h"
#include "string.h"
#include "USART_task.h"

#define Radius 1//车体机械半径
#define wheel_rpm_ratio 36.0//3508减速比
#define rotate_ratio_f 1
#define rotate_ratio_b 1

//舵轮底盘机械数据结构体
typedef struct
{
	float wheel_perimeter;//轮毂周长
}MechanicalStructure_t;
extern MechanicalStructure_t structure;

//舵轮底盘机械数据结构体
typedef struct
{
	int8_t speed_direction;//速度方向
}Steer_Type;
extern Steer_Type steer_set[4];

//舵轮底盘数据结构体
typedef struct
{
	motor_measure_t motor_chassis[4];//轮毂电机数据返回结构体
	
	float vx; //Vx                      
	float vy; //Vy                   
	float vw; //Vw	
	float wheel_rpm[4]; //用于存储转速信息
	
	float steeringAngleTarget[4];//舵机的目标转角              
	float lastSteeringAngleTarget[4]; //上一次的目标转角
	float steeringAngle[4]; //当前的舵机转角                     
	float last_steeringAngle[4]; //上一次的舵机转角
	int32_t motor_circle[4]; //电机转过的圈数
	int32_t motor_target_count[4]; //电机旋转的次数
	Steer_Type steer_set[4]; //用于舵轮速度解算
	uint16_t turnFlag[4];  //舵轮角度解算标志位
	
}ChassisHandle_t;
extern ChassisHandle_t ChassisHandle;


//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
double msp(double x, double min_in, double max_in, double min_out, double max_out);
//底盘参数初始化
void chassis_init(void);
//底盘运动函数
void chassis_task(void);
//麦轮电机速度解算函数
void Steer_Calculate(ChassisHandle_t *chassishandle,int16_t vx,int16_t vy,int16_t vw);

#endif



