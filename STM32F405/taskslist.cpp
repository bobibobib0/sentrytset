#include "label.h"
#include "taskslist.h"
#include "can.h"
#include "motor.h"
#include "imu.h"
#include "RC.h"
#include "tim.h"
#include "control.h"
#include "led.h"
#include "delay.h"
#include "HTmotor.h"
#include "Power_read.h"
#include "xuc.h"
//extern float Kp = 10;
//extern float Kd = 0.6;
extern int start_flag;


// 达妙临时通信调试开关，调试时只能打开其中一个。
// 防止两台云台电机同时产生意外动作。
//#define DM_YAW_TEST_ENABLE    0
//#define DM_PITCH_TEST_ENABLE  1

 // 达妙电机使能开关：1 为使能，0 为关闭。
//#define DM_YAW_ENABLE    1
//#define DM_PITCH_ENABLE  1

//#if DM_YAW_TEST_ENABLE && DM_PITCH_TEST_ENABLE
//#error "Only one DM motor may be tested at a time."
//#endif


void TASK::Init()
{
	// 创建启动任务。
	xTaskCreate((TaskFunction_t)start_task,            //任务函数
		(const char*)"start_task",          //任务名称
		(uint16_t)START_STK_SIZE,        //任务堆栈大小
		(void*)NULL,                  //传递给任务函数的参数
		(UBaseType_t)START_TASK_PRIO,       //任务优先级
		(TaskHandle_t*)&StartTask_Handler);   //任务句柄              
	vTaskStartScheduler();          //开启任务调度

}
/*
启动任务函数
*/
void start_task(void* pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	//创建任务

//// 请求使能大 Yaw 和 Pitch，实际使能帧由发送任务完成
//	DM_motorYaw.SetEnableRequest(DM_YAW_ENABLE != 0);
//	DM_motorPitch.SetEnableRequest(DM_PITCH_ENABLE != 0);
//
//	xTaskCreate((TaskFunction_t)ArmTask,
//		(const char*)"ArmTask",
//		(uint16_t)LED_STK_SIZE,
//		(void*)NULL,
//		(UBaseType_t)LED_TASK_PRIO,
//		(TaskHandle_t*)&LedTask_Handler);

	xTaskCreate((TaskFunction_t)DecodeTask,
		(const char*)"DecodeTask",
		(uint16_t)DECODE_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)DECODE_TASK_PRIO,
		(TaskHandle_t*)&DecodeTask_Handler);

	xTaskCreate((TaskFunction_t)MotorUpdateTask,
		(const char*)"MotorUpdateTask",
		(uint16_t)MOTOR_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)MOTOR_TASK_PRIO,
		(TaskHandle_t*)&MotorTask_Handler);

	xTaskCreate((TaskFunction_t)CanTransimtTask,
		(const char*)"CanTransimtTask",
		(uint16_t)CANTX_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CANTX_TASK_PRIO,
		(TaskHandle_t*)&CanTxTask_Handler);

	xTaskCreate((TaskFunction_t)ControlTask,
		(const char*)"ControlTask",
		(uint16_t)CONTROL_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CONTROL_TASK_PRIO,
		(TaskHandle_t*)&ControlTask_Handler);
//
//	vTaskDelete(StartTask_Handler); //删除启动任务
//	taskEXIT_CRITICAL();            //退出临界区
	
	// 先退出临界区，再删除当前启动任务。
	taskEXIT_CRITICAL();
	vTaskDelete(nullptr);
}
int CNT = 0;
void MotorUpdateTask(void* pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();

	while (true)
	{
		const uint32_t now_ms = HAL_GetTick();

		for (auto& motor : can1_motor)
		{
			motor.Ontimer(
				can1.data,
				can1.temp_data,
				can1.motor_feedback_count,
				can1.motor_last_feedback_tick,
				now_ms);
		}

		for (auto& motor : can2_motor)
		{
			motor.Ontimer(
				can2.data,
				can2.temp_data,
				can2.motor_feedback_count,
				can2.motor_last_feedback_tick,
				now_ms);
		}

		// 达妙数据流反馈解码→状态更新→指令编码
		DM_motorYaw.DecodeFeedback();
		DM_motorYaw.Update(now_ms);
		DM_motorYaw.EncodeCommand();

		DM_motorPitch.DecodeFeedback();
		DM_motorPitch.Update(now_ms);
		DM_motorPitch.EncodeCommand();

		vTaskDelayUntil(
			&last_wake_time,
			pdMS_TO_TICKS(2));
	}
}

void CanTransimtTask(void* pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();

	while (true)
	{
		switch ((timer.counter++) % 3)
		{
		case 0:
			{
				const uint32_t now_ms = HAL_GetTick();

				DM_motorYaw.Transmit(now_ms);
				DM_motorPitch.Transmit(now_ms);
				break;
			}

		case 1:
			can1.Transmit(0x1FF, can1.temp_data + 8);
			can2.Transmit(0x1FF, can2.temp_data + 8);
			break;

		case 2:
			can1.Transmit(0x200, can1.temp_data);
			can2.Transmit(0x200, can2.temp_data);
			break;

		default:
			break;
		}

		vTaskDelayUntil(
			&last_wake_time,
			pdMS_TO_TICKS(1));
	}
}

void ControlTask(void* pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();

	while (true)
	{
		// 输入层先更新，随后由控制层执行具体控制。
		rc.Update();
		ctrl.Update();

		vTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(5));
	}
}


void DecodeTask(void* pvParameters)
{
	while (true)
	{
		rc.Decode();
		imu_big_pantile.Decode();
		imu_small_pantile.Decode();
		
		xuc.Decode();
		xuc.Update();
	
		vTaskDelay(5);
	}
}

void ArmTask(void* pvParameters)
{
	while (true)
	{
		// 初始化达妙电机。
		/*DMmotor[0].DMmotorinit();*/
		power.Send();
		vTaskDelay(100);
	}
}





