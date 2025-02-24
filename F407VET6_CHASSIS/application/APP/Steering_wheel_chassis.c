#include "Steering_wheel_chassis.h"

extern RC_ctrl_t rc_ctrl;//遥控器数据指针
extern motor_measure_t motor_chassis[4];//轮毂电机数据结构体
extern Communicate_measure_t Send_data;//双板通讯数据打包
ChassisHandle_t ChassisHandle;//底盘数据结构体

//底盘参数初始化
void chassis_init(void)
{
	//PID参数
	float hub_PID[4][3]={{6,0,0},{6,0,0},{6,0,0},{6,0,0}};
	
	//pid初始化
	for(int i=0;i<4;i++)
	{
		PID_init(&motor_chassis[i].hub_wheel_pid,hub_PID[i],0,10000);//轮毂电机速度环PID初始化
	}
}

//底盘运动函数
void chassis_task(void)
{
	chassis_init();
	if(Send_data.s_rx[0]==2||Send_data.s_rx[1]==2)
	{//失能模式
		for(int i=0;i<4;i++)
		{
			ChassisHandle.wheel_rpm[i]=0;
		}
	}
	else if(Send_data.s_rx[0]==1)
	{//使能模式（底盘跟随云台）
		//麦轮底盘解算
		Steer_Calculate(&ChassisHandle,Send_data.vx_rx/5,-Send_data.vy_rx/5,Send_data.vw_rx/5);
	}
	for(int i=0;i<4;i++)
	{//轮毂PID计算
		PID_calc(&motor_chassis[i].hub_wheel_pid,motor_chassis[i].speed_rpm,ChassisHandle.wheel_rpm[i]);//速度环
	}
	//电机输出赋值
	CAN_chassis_send(&hcan1,motor_chassis[0].hub_wheel_pid.out,motor_chassis[1].hub_wheel_pid.out,motor_chassis[2].hub_wheel_pid.out,motor_chassis[3].hub_wheel_pid.out);//轮毂输出
}

void Steer_Calculate(ChassisHandle_t *chassishandle,int16_t vx,int16_t vy,int16_t vw)
{
	chassishandle->wheel_rpm[0]=(vx-vy+vw*rotate_ratio_f)*wheel_rpm_ratio;
	chassishandle->wheel_rpm[1]=(vx+vy+vw*rotate_ratio_f)*wheel_rpm_ratio;
	chassishandle->wheel_rpm[2]=(-vx+vy+vw*rotate_ratio_b)*wheel_rpm_ratio;
  chassishandle->wheel_rpm[3]=(-vx-vy+vw*rotate_ratio_b)*wheel_rpm_ratio;
}
