  /*
   *__/\\\_______/\\\__/\\\\____________/\\\\__/\\\________/\\\______________/\\\\\\\\\____________/\\\\\\\\\_____/\\\\\\\\\\\___
   * _\///\\\___/\\\/__\/\\\\\\________/\\\\\\_\/\\\_______\/\\\____________/\\\///////\\\_______/\\\////////____/\\\/////////\\\_
   *  ___\///\\\\\\/____\/\\\//\\\____/\\\//\\\_\/\\\_______\/\\\___________\/\\\_____\/\\\_____/\\\/____________\//\\\______\///__
   *   _____\//\\\\______\/\\\\///\\\/\\\/_\/\\\_\/\\\_______\/\\\___________\/\\\\\\\\\\\/_____/\\\_______________\////\\\_________
   *    ______\/\\\\______\/\\\__\///\\\/___\/\\\_\/\\\_______\/\\\___________\/\\\//////\\\____\/\\\__________________\////\\\______
   *     ______/\\\\\\_____\/\\\____\///_____\/\\\_\/\\\_______\/\\\___________\/\\\____\//\\\___\//\\\____________________\////\\\___
   *      ____/\\\////\\\___\/\\\_____________\/\\\_\//\\\______/\\\____________\/\\\_____\//\\\___\///\\\___________/\\\______\//\\\__
   *       __/\\\/___\///\\\_\/\\\_____________\/\\\__\///\\\\\\\\\/_____________\/\\\______\//\\\____\////\\\\\\\\\_\///\\\\\\\\\\\/___
   *        _\///_______\///__\///_____________\///_____\/////////_______________\///________\///________\/////////____\///////////_____
  */

#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "can.h"
#include "usart.h"
#include "taskslist.h"
#include "tim.h"
#include "sysclk.h"
#include "delay.h"
#include "imu.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "judgement.h"
#include "led.h"
#include "HTmotor.h"
#include "Power_read.h"
#include "xuc.h"

Motor can1_motor[CAN1_MOTOR_NUM] = {
	Motor(M3508,SPD,shooter,ID2,PID(1.78f, 0.0f, 0.0f)),
	Motor(M3508,SPD,shooter,ID3,PID(1.78f, 0.0f, 0.0f)),
//	Motor(M2006,SPD,supply, ID7, PID(20.0f, 0.0f, 0.0f,0.f), PID(0.30f, 0.0f, 0.0f,0.f)),// 0x1FF 的 Data[0.1]
	Motor(M2006,SPD,supply, ID7, PID(20.0f, 0.0f, 0.0f,0.f), PID(0.30f, 0.0f, 0.0f,0.f)),
	Motor(M6020, POS, pantile, ID1, PID(80.0f, 0.08f, 0.0f), PID(2.0f, 0.0f, 0.0f))
//	Motor(M6020, POS, pantile, ID1, PID(80.0f, 0.08f, 0.0f), PID(2.0f, 0.5f, 50.0f))
};
Motor can2_motor[CAN2_MOTOR_NUM] = {
	Motor(M3508, SPD, chassis, ID1,PID(3.5f, 0.0f, 0.05f)),
	Motor(M3508, SPD, chassis, ID2,PID(3.5f, 0.0f, 0.05f)),
	Motor(M3508, SPD, chassis, ID3,PID(3.5f, 0.0f, 0.05f)),
	Motor(M3508, SPD, chassis, ID4,PID(3.5f, 0.0f, 0.05f)),
	
};


CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
TIM  timer;
IMU imu_big_pantile, imu_small_pantile;
DELAY delay;
RC rc;
POWER power;
LED led1, led2, led3, led4;
TASK task;
CONTROL ctrl;
Judgement judgement;
PARAMETER para;
XUC xuc;
//在can后定义
DMMOTOR DM_motorYaw(0x06,0x206,&can2,DMControlMode::Speed,-3.141593f,3.141593f);
DMMOTOR DM_motorPitch(0x09,0x109,&can1,DMControlMode::PositionSpeed,-12.56637f,12.56637f);
//达妙定义电机对象，反馈ID，控制ID，CAN对象，控制模式，位置反馈范围



int main(void)
{
	SystemClockConfig();
	delay.Init(168);
	HAL_Init();

	can1.Init(CAN1);
	can2.Init(CAN2);
	timer.Init(BASE, TIM3, 1000).BaseInit();

	// 大云台 IMU 使用 UART5，小 Yaw/Pitch IMU 使用 USART3。
	imu_big_pantile.Init(&uart5, UART5, 115200, CH010);
	imu_small_pantile.Init(&uart3, USART3, 921600, CH010);
	rc.Init(&uart1, USART1, 100000);
//	power.Init(&uart5,UART5,9600);
	xuc.Init(&uart6, USART6,460800);

	para.Init();

	ctrl.Init(std::vector<Motor*>{
			&can1_motor[0],//摩擦轮ID2
			& can1_motor[1],//摩擦轮ID3
			& can1_motor[2],//拨弹轮ID7
			& can1_motor[3],//小Yaw6020 ID1 反馈0x205
			/*	& can1_motor[3],
				& can1_motor[4],
				& can1_motor[5]*/
	});
	ctrl.Init(std::vector<Motor*>{
			&can2_motor[0],//底盘ID1
			& can2_motor[1],//底盘ID2
			& can2_motor[2],//底盘ID3
			& can2_motor[3],//底盘ID4
	
	});
	

	

	task.Init();
	for (;;)
		;
}





