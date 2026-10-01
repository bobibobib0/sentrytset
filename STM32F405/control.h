#pragma once
#include <vector>
#include <cmath>
#include "stm32f4xx.h"
#include "motor.h"
#include "imu.h"

class CONTROL final
{
public:
	uint8_t init_DM = 0;
	Motor* chassis_motor[CHASSIS_MOTOR_NUM]{};
	Motor* pantile_motor[PANTILE_MOTOR_NUM]{};
	Motor* shooter_motor[SHOOTER_MOTOR_NUM]{};
	Motor* supply_motor[SUPPLY_MOTOR_NUM]{};
	
	enum MODE { PC, RC, AUTOAIM, RESET, ROTATION, SPINNING, FOLLOW, SEPARATE, FIRE, FEED_TEST, STOP } mode=STOP;
	//
	struct CHASSIS
	{
		PID chassis_reset{};
		int32_t speedx{}, speedy{}, speedz{};
		
		bool direction_valid = false;

		float big_yaw_forward_pos = 1.393f;//人为设定的大YAW对底盘车头的达妙位置,电池在左边，小电脑在后
		float small_yaw_forward_count = 251.0f;//小 Yaw正对大 Yaw前方时的编码器值。
		float gimbal_to_chassis_yaw = 0.0f; //枪管相对底盘的偏角
		
		void Keep_Direction();

		void Update();
		float Ramp(float setval, float curval, uint32_t RampSlope);
	};

	struct PANTILE
	{
//		enum TYPE { YAW, PITCH };
//		float mark_pitch{}, mark_yaw{};
//		PID pantile_PID[3] = { {0.04f,0.f,0.f},{0.05f,0.f,0.f}, {0.f,0.f,0.f} };
//		const float sensitivity = 2.5f;
//		bool aim = false;
//		void Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference);
//		void Update();
		enum TYPE {YAW, PITCH};

		float mark_pitch = 0.0f;
		float mark_yaw = 0.0f;

		// 控制层保存归一化后的遥控输入，范围为 -1～1。
		float yaw_manual_ratio = 0.0f;
		float pitch_manual_ratio = 0.0f;

		// Pitch 目标值、运动限位和发射角度状态。
		float pitch_target = 0.0f;
		float pitch_test_min = 0.0f;
		float pitch_test_max = 0.0f;

		bool manual_enabled = false;
		bool pitch_target_ready = false;
		bool pitch_range_valid = false;
		bool pitch_fire_allowed = false;
		bool yaw_zero_speed_hold = false;
		bool pitch_hold_enabled = false;

		float pitch_hold_target = 0.0f;

		// 精调状态：小 Yaw 使用编码器计数，Pitch 使用弧度。
		bool fine_track_enabled = false;
		bool small_yaw_target_ready = false;
		bool small_yaw_hold_target_ready = false;
		bool coarse_yaw_active = false;
		float coarse_yaw_speed = 0.0f;
		float fine_yaw_ratio = 0.0f;
		float fine_pitch_ratio = 0.0f;
		float small_yaw_center = 0.0f;
		float small_yaw_target = 0.0f;
		float small_yaw_hold_target = 0.0f;
		float small_yaw_test_min = 0.0f;
		float small_yaw_test_max = 0.0f;

		PID pantile_PID[3] =
		{
			{0.04f, 0.0f, 0.0f},
			{0.05f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f}
		};

		const float sensitivity = 2.5f;
		bool aim = false;

		void SetManualInput(int32_t ch_yaw, int32_t ch_pitch);
		void SetFineInput(int32_t ch_yaw, int32_t ch_pitch);
		void EnterHold();
		void Stop();
		void Keep_Pantile(float angleKeep, TYPE type, IMU frameOfReference);
		void Update(Motor* small_yaw_motor);
	};

	struct SHOOTER
	{
		float now_bullet_speed = 0.0f;

		bool auto_shoot = false;
		bool openRub = false;
		bool supply_bullet = false;
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;

		bool friction_ready = false;
		bool feedback_fault = false;
		int32_t shoot_speed = 2500;//摩擦轮目标转速
		int32_t left_command_speed = 0;
		int32_t right_command_speed = 0;
		int32_t left_speed_error = 0;
		int32_t right_speed_error = 0;
		uint32_t ready_start_tick = 0;
		uint32_t ready_drop_start_tick = 0;
		uint32_t friction_ready_drop_count = 0;

		void StartFriction();
		void StopFriction();
		void Update(Motor* left_motor, Motor* right_motor);
	};

	struct FEEDER
	{
		enum STATE : uint8_t
		{
			STOPPED,
			WAIT_READY, //等待摩擦轮达到目标转速
			READY, //摩擦轮达到目标转速,可以接收发射请求
			SINGLE_FEED,
			CONTINUOUS_FEED,
			JAM_CONFIRM, //疑似堵转
			JAM_REVERSE, //确认堵转，反转尝试清除堵转
			RECOVER_WAIT, //反转结束后停顿再恢复发射
			TEST_FEED, //不启动摩擦轮，直接测试拨弹轮的堵转检测和堵转恢复
			FAULT
		};

		enum FAULT_REASON : uint8_t
		{
			NO_FAULT,
			FEEDBACK_LOST,
			MOTOR_OVER_TEMPERATURE,
			FRICTION_TIMEOUT,
			SINGLE_TIMEOUT,
			CONTINUOUS_TIMEOUT,
			REVERSE_TIMEOUT,
			JAM_RETRY_EXCEEDED,
			TEST_JAM,
			TEST_TIMEOUT
		};

		// 遥控器只写入请求，执行状态统一由控制层维护。
		bool single_request = false;
		bool continuous_request = false;
		bool single_armed = false;
		bool continuous_armed = false;
		bool single_release_timing = false;
		uint32_t single_release_tick = 0;

		STATE state = STOPPED;
		STATE resume_state = STOPPED;
		FAULT_REASON fault_reason = NO_FAULT;

		// LiveWatch 观测量。
		uint32_t state_start_tick = 0;
		uint32_t operation_start_tick = 0;
		uint32_t jam_start_tick = 0;
		uint8_t jam_retry_count = 0;
		uint32_t completed_single_count = 0;
		
		uint32_t single_start_ace_completion_count = 0;//记录单发开始时的ACE完成数
		uint32_t jam_count = 0;
		uint32_t recovery_count = 0;
		int32_t single_target_angle = 0;
		int32_t reverse_start_angle = 0;

		// 初始参数偏保守，需结合实车机构继续调节。
		int32_t single_speed = -800;
		int32_t continuous_speed = -1000;
		int32_t reverse_speed = 600;
		int32_t command_speed = 0;

		// 空仓测试参数及 LiveWatch 状态。
		int32_t test_speed = -500;
		bool test_requested = false;
		bool test_active = false;
		bool test_finished = false;
		bool jam_detected = false;
		bool feedback_fault = false;
		uint32_t test_start_tick = 0;

		void SetFireRequest(bool single, bool continuous);
		void ClearFireRequest();
		void StartTest();
		void Stop(Motor* motor);
		void Update(
			Motor* motor,
			bool fire_mode_enabled,
			bool fire_permitted,
			bool friction_ready,
			bool friction_feedback_ok,
			bool test_permitted);
	};
	
	CHASSIS chassis;
	PANTILE pantile;
	SHOOTER shooter;
	FEEDER feeder;
	
	static int16_t Setrange(const int16_t original, const int16_t range);
	void Chassis_Mecanum_Run(int16_t speedx, int16_t speedy, int16_t speedz);
	void Chassis_Stop();
//	void Control_Pantile(int32_t ch_yaw, int32_t ch_pitch);
	float GetDelta(float delta);
	void Init(std::vector<Motor*> motor);
	void init_dm();
	
	void SetMode(MODE new_mode);
	MODE GetMode() const;

	void Control_Pantile(int32_t ch_yaw, int32_t ch_pitch);
	void Control_FinePantile(int32_t ch_yaw, int32_t ch_pitch);
	void Hold_Pantile();
	void Stop_Pantile();

	// 遥控器只提出摩擦轮启停请求，具体 PID 由 SHOOTER 执行。
	void Start_Friction();
	void Stop_Friction();
	void Set_Fire_Request(bool single, bool continuous);
	void Clear_Fire_Request();

	// 遥控器只请求空仓拨弹测试，电机逻辑由 FEEDER 执行。
	void Start_Supply_Test();
	void Stop_Supply();

	// 供 ControlTask 统一调用。
	void Update();
	
private:

};

extern CONTROL ctrl;
