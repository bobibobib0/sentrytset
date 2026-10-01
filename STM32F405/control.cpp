#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include <algorithm>

namespace
{
	constexpr float RC_CHANNEL_MAX = 660.0f;
	constexpr int32_t RC_DEADBAND = 20;//遥控器死区限制
	constexpr int16_t CHASSIS_WHEEL_SPEED_LIMIT = 3000;
	constexpr int32_t CHASSIS_CURRENT_LIMIT = 10000;

	constexpr float YAW_TEST_SPEED_MAX = 0.80f;
	constexpr float PITCH_TARGET_RATE = 0.50f;
	constexpr float PITCH_MOVE_SPEED = 1.50f;
	constexpr float CONTROL_PERIOD_S = 0.005f;


// Pitch 抬头时位置减小，低头时位置增大。
	constexpr float PITCH_MOVE_MIN = -0.45f;
	constexpr float PITCH_MOVE_MAX =  0.72f;

	// 低头至0.48 rad时弹道开始受结构遮挡，因此发射上限限制为0.48 rad。
	constexpr float PITCH_FIRE_MIN = -0.45f;
	constexpr float PITCH_FIRE_MAX =  0.48f;

	// 如果实际方向相反，只修改这两个方向系数。
	constexpr float YAW_DIRECTION = -1.0f;
	constexpr float PITCH_DIRECTION = -1.0f;

	// 小 Yaw 以标定前向位置为基准使用非对称限位。
	constexpr float SMALL_YAW_DIRECTION = -1.0f;
	constexpr float SMALL_YAW_MIN_REL_COUNTS = -1158.0f;
	constexpr float SMALL_YAW_MAX_REL_COUNTS = 1706.67f;
	constexpr float SMALL_YAW_TARGET_RATE_COUNTS = 3000.0f;//目标角度最多每秒3000counts
	constexpr int16_t SMALL_YAW_SPEED_LIMIT_RPM = 100;//小Yaw电机初始测试速度限制，单位rpm
	constexpr int16_t SMALL_YAW_CURRENT_LIMIT = 3000;
	constexpr float PITCH_FINE_TARGET_RATE = 1.80f;
	
	constexpr float SMALL_YAW_COARSE_START_COUNTS = 682.67f; // 小yaw大yaw角度差30°就开始粗调
	constexpr float SMALL_YAW_COARSE_STOP_COUNTS = 113.78f;  // 回到5°停止粗调
	constexpr float BIG_YAW_COARSE_KP = 5.0f;
	constexpr float BIG_YAW_COARSE_SPEED_MAX = 2.0f;//大yaw粗调最大速度
	
	constexpr int32_t FRICTION_LEFT_DIRECTION = 1;
	constexpr int32_t FRICTION_RIGHT_DIRECTION = -1;
	constexpr int32_t FRICTION_RAMP_STEP = 10;
	constexpr int32_t FRICTION_READY_ERROR = 300;
	constexpr uint32_t FRICTION_READY_TIME_MS = 300U;
	// 摩擦轮就绪后允许弹丸通过造成的短时掉速；
	// 只有转速误差持续超限才暂停拨弹。
	constexpr int32_t FRICTION_READY_DROP_ERROR = 700;
	constexpr uint32_t FRICTION_READY_DROP_TIME_MS = 250U;

	constexpr int32_t SUPPLY_MAX_SPEED = 1200;
	constexpr int32_t SUPPLY_RAMP_STEP = 20;
	constexpr int32_t SUPPLY_CURRENT_LIMIT = 13000;
	constexpr int32_t SUPPLY_SINGLE_STEP_COUNTS = 42130;
	constexpr int32_t SUPPLY_SINGLE_TOLERANCE = 300;
	constexpr int32_t SUPPLY_REVERSE_COUNTS = 4096;

	constexpr int32_t SUPPLY_STALL_SPEED = 80;
	constexpr int32_t SUPPLY_STALL_CURRENT = 2500;

	constexpr uint32_t SUPPLY_SINGLE_REARM_MS = 100U;
	constexpr uint32_t SUPPLY_STARTUP_GRACE_MS = 250U;
	constexpr uint32_t SUPPLY_STALL_TIME_MS = 300U;
	constexpr uint32_t SUPPLY_FRICTION_TIMEOUT_MS = 4000U;
	constexpr uint32_t SUPPLY_SINGLE_TIMEOUT_MS = 6000U;
	constexpr uint32_t SUPPLY_CONTINUOUS_TIMEOUT_MS = 20000U;
	constexpr uint32_t SUPPLY_REVERSE_MIN_MS = 120U;
	constexpr uint32_t SUPPLY_REVERSE_TIMEOUT_MS = 800U;
	constexpr uint32_t SUPPLY_RECOVER_WAIT_MS = 150U;
	constexpr uint32_t SUPPLY_TEST_TIMEOUT_MS = 5000U;
	constexpr uint8_t SUPPLY_MAX_JAM_RETRIES = 3U;
	
	constexpr float BIG_YAW_DIRECTION = 1.0f;
	constexpr float SMALL_YAW_CHASSIS_DIRECTION = 1.0f;

	constexpr float ENCODER_TO_RAD =
	    2.0f * PI / 8192.0f;
	
	static float WrapPi(float angle)
	{
		while (angle > PI)
			angle -= 2.0f * PI;

		while (angle < -PI)
			angle += 2.0f * PI;

		return angle;
	}
	
	int32_t RampInt32(int32_t current, int32_t target, int32_t step)
	{
		if (current < target)
		{
			current += step;
			return current > target ? target : current;
		}

		if (current > target)
		{
			current -= step;
			return current < target ? target : current;
		}

		return current;
	}

	int32_t ClampCurrent(int32_t current, int32_t limit)
	{
		if (current > limit) return limit;
		if (current < -limit) return -limit;
		return current;
	}

	int32_t AbsInt32(int32_t value)
	{
		return value >= 0 ? value : -value;
	}

	float ClampFloat(float value, float minimum, float maximum)
	{
		if (value < minimum) return minimum;
		if (value > maximum) return maximum;
		return value;
	}

	float NormalizeRC(int32_t channel)
	{
		if (channel > -RC_DEADBAND && channel < RC_DEADBAND)
		{
			return 0.0f;
		}

		return ClampFloat(
			static_cast<float>(channel) / RC_CHANNEL_MAX,
			-1.0f,
			1.0f);
	}
}

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			if (num2 < PANTILE_MOTOR_NUM)
			{
				pantile_motor[num2] = motor[i];
				// 首次实车调试限制电流和转速，确认方向及限位后再调整。
				pantile_motor[num2]->maxspeed = SMALL_YAW_SPEED_LIMIT_RPM;
				pantile_motor[num2]->maxcurrent = SMALL_YAW_CURRENT_LIMIT;
				++num2;
			}
			break;
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			if (num4 < SUPPLY_MOTOR_NUM)
			{
				supply_motor[num4] = motor[i];// 保存供弹电机地址
				supply_motor[num4]->spinning = false;
				supply_motor[num4]->need_curcircle = 0;
				++num4;
			}
			break;
			
		default:
			break;
		}
	}
	/*pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;*/
	/*pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;*/
}


void CONTROL::SetMode(MODE new_mode)
{
	if (mode != new_mode)
	{
		chassis.direction_valid = false;
	}

	mode = new_mode;
}

CONTROL::MODE CONTROL::GetMode() const
{
	return mode;
}

void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)
{
	if (mode != SEPARATE)
	{
		pantile.Stop();
		return;
	}

	pantile.SetManualInput(ch_yaw, ch_pitch);
}

void CONTROL::Control_FinePantile(int32_t ch_yaw, int32_t ch_pitch)
{
	if (mode != AUTOAIM)
	{
		pantile.Stop();
		return;
	}

	// 用遥控器摇杆模拟视觉偏差
	pantile.SetFineInput(ch_yaw, ch_pitch);
}

void CONTROL::Hold_Pantile()
{
	pantile.EnterHold();
}

void CONTROL::Stop_Pantile()
{
	pantile.Stop();
}

void CONTROL::Start_Friction()
{
	if (feeder.state == FEEDER::FAULT)
	{
		shooter.StopFriction();
		return;
	}

	shooter.StartFriction();
}

void CONTROL::Stop_Friction()
{
	shooter.StopFriction();
}

void CONTROL::Set_Fire_Request(bool single, bool continuous)
{
	feeder.SetFireRequest(single, continuous);
}

void CONTROL::Clear_Fire_Request()
{
	feeder.ClearFireRequest();
}

void CONTROL::Start_Supply_Test()
{
	feeder.StartTest();
}

void CONTROL::Stop_Supply()
{
	feeder.Stop(supply_motor[0]);
}

void CONTROL::Update()
{
	chassis.Update();
	pantile.Update(pantile_motor[PANTILE::TYPE::YAW]);

	shooter.Update(shooter_motor[0],shooter_motor[1]);

	const bool fire_mode_enabled = (mode == FIRE);
	const bool fire_permitted =
		fire_mode_enabled &&
		pantile.pitch_range_valid &&
		pantile.pitch_fire_allowed;

	const bool feeder_test_permitted = (mode == FEED_TEST);

	feeder.Update(
		supply_motor[0],
		fire_mode_enabled,
		fire_permitted,
		shooter.friction_ready,
		!shooter.feedback_fault,
		feeder_test_permitted);

	// 拨弹故障保持锁定，退出当前模式后才复位
	if (feeder.state == FEEDER::FAULT)
	{
		shooter.StopFriction();

		for (Motor* motor : shooter_motor)
		{
			if (motor != nullptr)
			{
				motor->setspeed = 0;
				motor->current = 0;
			}
		}
	}
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)
{
	
}

void CONTROL::CHASSIS::Keep_Direction()
{
	direction_valid = false;

	Motor* small_yaw =
	    ctrl.pantile_motor[PANTILE::YAW];

	if (small_yaw == nullptr ||
	    !small_yaw->feedback_online_ ||
	    !DM_motorYaw.FeedbackFresh(
	        HAL_GetTick(),
		DMProtocol::FEEDBACK_TIMEOUT_MS))
	{
		return;
	}

	const float big_yaw_angle =
	    BIG_YAW_DIRECTION *
	    (DM_motorYaw.pos - big_yaw_forward_pos);

	float small_yaw_count =
	    small_yaw->angle[now] -
	    small_yaw_forward_count;

	while (small_yaw_count > 4096.0f)
		small_yaw_count -= 8192.0f;

	while (small_yaw_count < -4096.0f)
		small_yaw_count += 8192.0f;

	const float small_yaw_angle =
	    SMALL_YAW_CHASSIS_DIRECTION *
	    small_yaw_count *
	    ENCODER_TO_RAD;

	gimbal_to_chassis_yaw =
	    WrapPi(big_yaw_angle + small_yaw_angle);

	const float forward = static_cast<float>(speedx);
	const float right = static_cast<float>(speedy);
	const float angle = gimbal_to_chassis_yaw;

	speedx = static_cast<int32_t>(-forward * sinf(angle) +
	     right * cosf(angle));

	speedy = static_cast<int32_t>( -forward * cosf(angle) -
	     right * sinf(angle));

	direction_valid = true;
}

void CONTROL::CHASSIS::Update()
{
	for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
	{
		Motor* motor = ctrl.chassis_motor[i];
		if (motor == nullptr)
		{
			continue;
		}

		motor->mode = SPD;

		if (!motor->feedback_online_ || motor->temperature > 70)
		{
			motor->setspeed = 0;
			motor->current = 0;
			continue;
		}

		const int32_t speed_error = motor->setspeed - motor->curspeed;
		motor->current = ClampCurrent(
			static_cast<int32_t>(
				motor->pid[speed].Position(
					static_cast<float>(speed_error),
					static_cast<float>(CHASSIS_CURRENT_LIMIT))),
			CHASSIS_CURRENT_LIMIT);
	}
}

void CONTROL::Chassis_Mecanum_Run(int16_t speedx, int16_t speedy, int16_t speedz)
{
	Motor* left_front = nullptr;
	Motor* right_front = nullptr;
	Motor* right_rear = nullptr;
	Motor* left_rear = nullptr;

	for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
	{
		Motor* motor = chassis_motor[i];
		if (motor == nullptr)
		{
			continue;
		}

		switch (motor->ID)
		{
		case ID1: left_front = motor; break;
		case ID2: right_front = motor; break;
		case ID3: right_rear = motor; break;
		case ID4: left_rear = motor; break;
		default: break;
		}
	}

	if (left_front == nullptr || right_front == nullptr ||
		right_rear == nullptr || left_rear == nullptr)
	{
		Chassis_Stop();
		return;
	}

	chassis.speedx = speedx;
	chassis.speedy = speedy;
	chassis.speedz = speedz;

	const int16_t left_front_speed = Setrange(
		static_cast<int16_t>(0.707f * speedx + 0.707f * speedy + speedz),
		CHASSIS_WHEEL_SPEED_LIMIT);
	const int16_t right_front_speed = Setrange(
		static_cast<int16_t>(0.707f * speedx - 0.707f * speedy + speedz),
		CHASSIS_WHEEL_SPEED_LIMIT);
	const int16_t right_rear_speed = Setrange(
		static_cast<int16_t>(-0.707f * speedx - 0.707f * speedy + speedz),
		CHASSIS_WHEEL_SPEED_LIMIT);
	const int16_t left_rear_speed = Setrange(
		static_cast<int16_t>(-0.707f * speedx + 0.707f * speedy + speedz),
		CHASSIS_WHEEL_SPEED_LIMIT);

	left_front->mode = SPD;
	right_front->mode = SPD;
	right_rear->mode = SPD;
	left_rear->mode = SPD;

	left_front->setspeed = left_front_speed;
	right_front->setspeed = right_front_speed;
	right_rear->setspeed = right_rear_speed;
	left_rear->setspeed = left_rear_speed;
}

void CONTROL::Chassis_Stop()
{
	chassis.speedx = 0;
	chassis.speedy = 0;
	chassis.speedz = 0;

	for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
	{
		if (chassis_motor[i] != nullptr)
		{
			chassis_motor[i]->mode = SPD;
			chassis_motor[i]->setspeed = 0;
		}
	}
}

void CONTROL::PANTILE::SetManualInput(int32_t ch_yaw,int32_t ch_pitch)
{
	// 此处只接收原始遥控量，归一化和死区处理由控制层完成。
	yaw_manual_ratio = NormalizeRC(ch_yaw);
	pitch_manual_ratio = NormalizeRC(ch_pitch);

	fine_track_enabled = false;
	small_yaw_target_ready = false;
	small_yaw_hold_target_ready = false;
	pitch_hold_enabled = false;
	yaw_zero_speed_hold = false;
	manual_enabled = true;
}
void CONTROL::PANTILE::SetFineInput(int32_t ch_yaw, int32_t ch_pitch)
{
	if (!fine_track_enabled)
	{
		small_yaw_target_ready = false;
		pitch_target_ready = false;
		coarse_yaw_active = false;
		coarse_yaw_speed = 0.0f;
	}

	fine_yaw_ratio = NormalizeRC(ch_yaw);
	fine_pitch_ratio = NormalizeRC(ch_pitch);

	manual_enabled = false;
	small_yaw_hold_target_ready = false;
	pitch_hold_enabled = false;
	yaw_zero_speed_hold = true;
	fine_track_enabled = true;
}

void CONTROL::PANTILE::EnterHold()
{
	fine_track_enabled = false;
	small_yaw_target_ready = false;
	coarse_yaw_active = false;
	coarse_yaw_speed = 0.0f;
	fine_yaw_ratio = 0.0f;
	fine_pitch_ratio = 0.0f;

	if (pitch_hold_enabled)
	{
		return;
	}

	// 进入保持模式时仅记录一次小 Yaw 当前编码器位置。
	small_yaw_hold_target_ready = false;

	yaw_manual_ratio = 0.0f;
	pitch_manual_ratio = 0.0f;

	manual_enabled = false;
	pitch_hold_enabled = true;
	yaw_zero_speed_hold = true;

	// 下一控制周期记录 Pitch 实际位置。
	pitch_target_ready = false;
	pitch_range_valid = false;
	pitch_fire_allowed = false;
}
void CONTROL::PANTILE::Stop()
{
	yaw_manual_ratio = 0.0f;
	pitch_manual_ratio = 0.0f;
	fine_yaw_ratio = 0.0f;
	fine_pitch_ratio = 0.0f;
	coarse_yaw_speed = 0.0f;
	
	coarse_yaw_active = false;
	manual_enabled = false;
	fine_track_enabled = false;
	small_yaw_target_ready = false;
	small_yaw_hold_target_ready = false;
	pitch_hold_enabled = false;
	yaw_zero_speed_hold = false;

	pitch_target_ready = false;
	pitch_range_valid = false;
	pitch_fire_allowed = false;
}
void CONTROL::PANTILE::Update(Motor* small_yaw_motor)
{
	const uint32_t now_ms = HAL_GetTick();

	// 非精调模式下，云台保持时锁定小 Yaw 当前位置。
	if (!fine_track_enabled && small_yaw_motor != nullptr)
	{
		small_yaw_target_ready = false;

		if (pitch_hold_enabled && small_yaw_motor->feedback_online_)
		{
			if (!small_yaw_hold_target_ready)
			{
				small_yaw_hold_target = small_yaw_motor->angle[now];
				small_yaw_hold_target_ready = true;
			}

			small_yaw_motor->setangle = small_yaw_hold_target;
			small_yaw_motor->mode = POS;
		}
		else
		{
			small_yaw_motor->mode = IDLE;
			small_yaw_motor->setspeed = 0;
			small_yaw_motor->current = 0;
			small_yaw_hold_target_ready = false;
		}
	}

	if (fine_track_enabled)
	{
		// 此处保持大 Yaw 使能；仅当小 Yaw 超过下方粗调阈值时
		// 才允许大 Yaw 转动。
		DM_motorYaw.setSpeed = 0.0f;
		// 即使尚未收到反馈也持续请求使能，使达妙电机进入正常的
		// 反馈与控制流程。
		DM_motorYaw.SetEnableRequest(true);

		// M6020 沿用位置外环、速度内环的双环控制。
		if (small_yaw_motor == nullptr || !small_yaw_motor->feedback_online_)
		{
			if (small_yaw_motor != nullptr)
			{
				small_yaw_motor->mode = IDLE;
				small_yaw_motor->setspeed = 0;
				small_yaw_motor->current = 0;
			}
			small_yaw_target_ready = false;
			coarse_yaw_active = false;
			coarse_yaw_speed = 0.0f;
		}
		else
		{
			float current_relative =
				small_yaw_motor->angle[now] -
				ctrl.chassis.small_yaw_forward_count;

			while (current_relative > 4096.0f)
				current_relative -= 8192.0f;

			while (current_relative < -4096.0f)
				current_relative += 8192.0f;

			if (!small_yaw_target_ready)
			{
				small_yaw_center = ctrl.chassis.small_yaw_forward_count;
				small_yaw_target = small_yaw_center + current_relative;
				small_yaw_test_min =
					small_yaw_center + SMALL_YAW_MIN_REL_COUNTS;
				small_yaw_test_max =
					small_yaw_center + SMALL_YAW_MAX_REL_COUNTS;
				small_yaw_target_ready = true;
			}

			const float absolute_relative =
				current_relative >= 0.0f ? current_relative : -current_relative;

			// 使用启停回差，避免大 Yaw 在交接阈值附近
			// 反复启动和停止。
			if (!coarse_yaw_active)
			{
				if (absolute_relative >= SMALL_YAW_COARSE_START_COUNTS)
					coarse_yaw_active = true;
			}
			else if (absolute_relative <= SMALL_YAW_COARSE_STOP_COUNTS)
			{
				coarse_yaw_active = false;
			}

			coarse_yaw_speed = 0.0f;
			if (coarse_yaw_active &&
				DM_motorYaw.FeedbackFresh(now_ms, DMProtocol::FEEDBACK_TIMEOUT_MS))
			{
				const float relative_angle =
					SMALL_YAW_CHASSIS_DIRECTION *
					current_relative *
					ENCODER_TO_RAD;

				const float coarse_angle_rate = ClampFloat(
					BIG_YAW_COARSE_KP * relative_angle,
					-BIG_YAW_COARSE_SPEED_MAX,
					 BIG_YAW_COARSE_SPEED_MAX);

				coarse_yaw_speed =
					coarse_angle_rate / BIG_YAW_DIRECTION;
			}

			DM_motorYaw.setSpeed = coarse_yaw_speed;

			const float fine_target_rate =
				SMALL_YAW_DIRECTION *
				fine_yaw_ratio *
				SMALL_YAW_TARGET_RATE_COUNTS;

			// 大 Yaw 跟随时，小 Yaw 目标向相反方向移动，
			// 使小 Yaw 回中，同时不额外改变
			// 枪口的整体朝向。
			const float coarse_compensation_rate =
				BIG_YAW_DIRECTION * coarse_yaw_speed /
				(SMALL_YAW_CHASSIS_DIRECTION * ENCODER_TO_RAD);

			small_yaw_target +=
				(fine_target_rate - coarse_compensation_rate) *
				CONTROL_PERIOD_S;

			small_yaw_target = ClampFloat(
				small_yaw_target,
				small_yaw_test_min,
				small_yaw_test_max);

			small_yaw_motor->setangle = small_yaw_target;
			small_yaw_motor->mode = POS;
		}

		// 先请求使能；反馈有效性只决定是否下发运动指令。
		DM_motorPitch.SetEnableRequest(true);
		if (!DM_motorPitch.FeedbackFresh(now_ms, DMProtocol::FEEDBACK_TIMEOUT_MS))
		{
			DM_motorPitch.setSpeed = 0.0f;
			pitch_target_ready = false;
			pitch_range_valid = false;
			pitch_fire_allowed = false;
			return;
		}

		if (!pitch_target_ready)
		{
			pitch_target = ClampFloat(DM_motorPitch.pos, PITCH_MOVE_MIN, PITCH_MOVE_MAX);
			pitch_test_min = PITCH_MOVE_MIN;
			pitch_test_max = PITCH_MOVE_MAX;
			pitch_target_ready = true;
			pitch_range_valid = true;
		}

		pitch_target +=
			PITCH_DIRECTION *
			fine_pitch_ratio *
			PITCH_FINE_TARGET_RATE *
			CONTROL_PERIOD_S;
		pitch_target = ClampFloat(pitch_target, PITCH_MOVE_MIN, PITCH_MOVE_MAX);

		DM_motorPitch.setPos = pitch_target;
		DM_motorPitch.setSpeed = PITCH_MOVE_SPEED;
		pitch_fire_allowed =
			(DM_motorPitch.pos >= PITCH_FIRE_MIN) &&
			(DM_motorPitch.pos <= PITCH_FIRE_MAX);
		return;
	}

	if (pitch_hold_enabled)
	{
		// Pitch 保持位置时，大 Yaw 保持零速。
		DM_motorYaw.setSpeed = 0.0f;
		DM_motorYaw.SetEnableRequest(true);

		// 保持模式也必须先请求使能，不能等待反馈后再使能。
		DM_motorPitch.SetEnableRequest(true);
		if (!DM_motorPitch.FeedbackFresh(now_ms, DMProtocol::FEEDBACK_TIMEOUT_MS))
		{
			DM_motorPitch.setSpeed = 0.0f;
			pitch_target_ready = false;
			pitch_range_valid = false;
			pitch_fire_allowed = false;
			return;
		}

		if (!pitch_target_ready)
		{
			pitch_hold_target = ClampFloat(DM_motorPitch.pos, PITCH_MOVE_MIN, PITCH_MOVE_MAX);
			pitch_target = pitch_hold_target;
			pitch_test_min = PITCH_MOVE_MIN;
			pitch_test_max = PITCH_MOVE_MAX;
			pitch_target_ready = true;
			pitch_range_valid = true;
		}

		DM_motorPitch.setPos = pitch_hold_target;
		DM_motorPitch.setSpeed = PITCH_MOVE_SPEED;
		pitch_fire_allowed =
			(DM_motorPitch.pos >= PITCH_FIRE_MIN) &&
			(DM_motorPitch.pos <= PITCH_FIRE_MAX);
		return;
	}

	if (!manual_enabled)
	{
		DM_motorYaw.setSpeed = 0.0f;
		DM_motorPitch.setSpeed = 0.0f;
		// 无云台控制模式时，才主动释放
		// 两台达妙电机。
		DM_motorYaw.SetEnableRequest(false);
		DM_motorPitch.SetEnableRequest(false);
		pitch_target_ready = false;
		pitch_range_valid = false;
		pitch_fire_allowed = false;
		return;
	}

	// 手动模式需先请求使能，再检查反馈。
	DM_motorYaw.SetEnableRequest(true);
	if (DM_motorYaw.FeedbackFresh(now_ms, DMProtocol::FEEDBACK_TIMEOUT_MS))
	{
		DM_motorYaw.setSpeed = YAW_DIRECTION * yaw_manual_ratio * YAW_TEST_SPEED_MAX;
	}
	else
	{
		DM_motorYaw.setSpeed = 0.0f;
	}

	DM_motorPitch.SetEnableRequest(true);
	if (!DM_motorPitch.FeedbackFresh(now_ms, DMProtocol::FEEDBACK_TIMEOUT_MS))
	{
		DM_motorPitch.setSpeed = 0.0f;
		pitch_target_ready = false;
		pitch_range_valid = false;
		pitch_fire_allowed = false;
		return;
	}

	if (!pitch_target_ready)
	{
		pitch_target = ClampFloat(DM_motorPitch.pos, PITCH_MOVE_MIN, PITCH_MOVE_MAX);
		pitch_test_min = PITCH_MOVE_MIN;
		pitch_test_max = PITCH_MOVE_MAX;
		pitch_target_ready = true;
		pitch_range_valid = true;
	}

	pitch_target +=
		PITCH_DIRECTION *
		pitch_manual_ratio *
		PITCH_TARGET_RATE *
		CONTROL_PERIOD_S;
	pitch_target = ClampFloat(pitch_target, PITCH_MOVE_MIN, PITCH_MOVE_MAX);

	DM_motorPitch.setPos = pitch_target;
	DM_motorPitch.setSpeed = PITCH_MOVE_SPEED;
	pitch_fire_allowed =
		(DM_motorPitch.pos >= PITCH_FIRE_MIN) &&
		(DM_motorPitch.pos <= PITCH_FIRE_MAX);
}
void CONTROL::SHOOTER::StartFriction()
{
	openRub = true;
}

void CONTROL::SHOOTER::StopFriction()
{
	openRub = false;
	friction_ready = false;
	feedback_fault = false;
	ready_start_tick = 0;
	ready_drop_start_tick = 0;
	left_speed_error = 0;
	right_speed_error = 0;
}

void CONTROL::SHOOTER::Update(
	Motor* left_motor,
	Motor* right_motor)
{
	if (left_motor == nullptr || right_motor == nullptr)
	{
		StopFriction();
		return;
	}

	if (!openRub)
	{
		left_command_speed = 0;
		right_command_speed = 0;

		left_motor->setspeed = 0;
		right_motor->setspeed = 0;
		left_motor->current = 0;
		right_motor->current = 0;

		friction_ready = false;
		feedback_fault = false;
		ready_start_tick = 0;
		ready_drop_start_tick = 0;
		left_speed_error = 0;
		right_speed_error = 0;
		return;
	}

	if (!left_motor->feedback_online_ ||
		!right_motor->feedback_online_ ||
		left_motor->temperature > 70 ||
		right_motor->temperature > 70)
	{
		left_command_speed = 0;
		right_command_speed = 0;

		left_motor->setspeed = 0;
		right_motor->setspeed = 0;
		left_motor->current = 0;
		right_motor->current = 0;

		friction_ready = false;
		feedback_fault = true;
		ready_start_tick = 0;
		ready_drop_start_tick = 0;
		left_speed_error = 0;
		right_speed_error = 0;
		return;
	}

	feedback_fault = false;
	//
	const int32_t left_target =FRICTION_LEFT_DIRECTION * shoot_speed;
	const int32_t right_target =FRICTION_RIGHT_DIRECTION * shoot_speed;
	

	left_command_speed = RampInt32(
		left_command_speed,
		left_target,
		FRICTION_RAMP_STEP);

	right_command_speed = RampInt32(
		right_command_speed,
		right_target,
		FRICTION_RAMP_STEP);

	left_motor->setspeed = left_command_speed;
	right_motor->setspeed = right_command_speed;

	left_speed_error =
		left_motor->setspeed - left_motor->curspeed;

	right_speed_error =
		right_motor->setspeed - right_motor->curspeed;

	left_motor->current = ClampCurrent(
		static_cast<int32_t>(
			left_motor->pid[speed].Position(
				static_cast<float>(left_speed_error),
			static_cast<float>(left_motor->maxcurrent))),
		left_motor->maxcurrent);

	right_motor->current = ClampCurrent(
		static_cast<int32_t>(
			right_motor->pid[speed].Position(
				static_cast<float>(right_speed_error),
			static_cast<float>(right_motor->maxcurrent))),
		right_motor->maxcurrent);

	const bool command_reached =(left_command_speed == left_target) && (right_command_speed == right_target);

	const bool speed_stable =
		AbsInt32(left_motor->curspeed - left_target) <=
			FRICTION_READY_ERROR &&
		AbsInt32(right_motor->curspeed - right_target) <=
			FRICTION_READY_ERROR;

	const bool speed_within_hold_window =
		AbsInt32(left_motor->curspeed - left_target) <=
			FRICTION_READY_DROP_ERROR &&
		AbsInt32(right_motor->curspeed - right_target) <=
			FRICTION_READY_DROP_ERROR;

	const uint32_t now_ms = HAL_GetTick();

	if (!friction_ready)
	{
		ready_drop_start_tick = 0;

		if (command_reached && speed_stable)
		{
			if (ready_start_tick == 0U)
			{
				ready_start_tick = now_ms;
			}

			friction_ready =
				(now_ms - ready_start_tick) >= FRICTION_READY_TIME_MS;
		}
		else
		{
			ready_start_tick = 0;
		}
	}
	else
	{
		ready_start_tick = 0;

		// 目标转速变化后重新判定摩擦轮是否就绪。
		if (!command_reached)
		{
			friction_ready = false;
			ready_drop_start_tick = 0;
		}
		else if (speed_within_hold_window)
		{
			ready_drop_start_tick = 0;
		}
		else
		{
			if (ready_drop_start_tick == 0U)
			{
				ready_drop_start_tick = now_ms;
			}
			else if ((now_ms - ready_drop_start_tick) >=
				FRICTION_READY_DROP_TIME_MS)
			{
				friction_ready = false;
				ready_drop_start_tick = 0;
				friction_ready_drop_count++;
			}
		}
	}
}

void CONTROL::FEEDER::SetFireRequest(bool single, bool continuous)
{
	// 从拨弹测试切到发射模式时先清除测试状态。
	if (test_requested)
	{
		test_requested = false;
		test_active = false;
		test_finished = false;
		state = STOPPED;
		resume_state = STOPPED;
		fault_reason = NO_FAULT;
		command_speed = 0;
		jam_start_tick = 0;
		jam_retry_count = 0;
		single_armed = false;
		continuous_armed = false;
		single_release_timing = false;
	}

	single_request = single;
	continuous_request = continuous;
}

void CONTROL::FEEDER::ClearFireRequest()
{
	single_request = false;
	continuous_request = false;
}

void CONTROL::FEEDER::StartTest()
{
	// 拨杆持续保持时，不重复启动单次测试。
	if (test_requested)
	{
		return;
	}

	ClearFireRequest();
	single_armed = false;
	continuous_armed = false;
	single_release_timing = false;
	test_requested = true;
	test_active = false;
	test_finished = false;
	jam_detected = false;
	feedback_fault = false;
	command_speed = 0;
	test_start_tick = 0;
	jam_start_tick = 0;
	state = STOPPED;
	resume_state = STOPPED;
	fault_reason = NO_FAULT;
	jam_retry_count = 0;
}

void CONTROL::FEEDER::Stop(Motor* motor)
{
	ClearFireRequest();
	single_armed = false;
	continuous_armed = false;
	single_release_timing = false;
	single_release_tick = 0;

	test_requested = false;
	test_active = false;
	test_finished = false;
	jam_detected = false;
	feedback_fault = false;

	state = STOPPED;
	resume_state = STOPPED;
	fault_reason = NO_FAULT;
	state_start_tick = 0;
	operation_start_tick = 0;
	jam_start_tick = 0;
	jam_retry_count = 0;
	single_target_angle = 0;
	reverse_start_angle = 0;
	command_speed = 0;
	test_start_tick = 0;
	single_start_ace_completion_count = 0;

	if (motor != nullptr)
	{
		motor->setspeed = 0;
		motor->current = 0;

		motor->need_curcircle = 0.0f;
		motor->need_angle = 0;
		motor->count = 0;
		motor->brake_ticks = 0;

		motor->mode = IDLE;
	}
}

void CONTROL::FEEDER::Update(
	Motor* motor,
	bool fire_mode_enabled,
	bool fire_permitted,
	bool friction_ready,
	bool friction_feedback_ok,
	bool test_permitted)
{
	const uint32_t now_ms = HAL_GetTick();

	auto enter_state = [this, now_ms](STATE next_state)
	{
		state = next_state;
		state_start_tick = now_ms;
	};

	auto stop_motor = [this, motor]()// 停止拨弹电机并清除运行状态。
	{
		command_speed = 0;

		if (motor != nullptr)
		{
			motor->setspeed = 0;
			motor->current = 0;

			motor->need_curcircle = 0.0f;
			motor->need_angle = 0;
			motor->count = 0;
			motor->brake_ticks = 0;

			motor->mode = IDLE;
		}
	};


	auto set_fault = [this, now_ms, &stop_motor](FAULT_REASON reason)
	{
		fault_reason = reason;
		state = FAULT;
		state_start_tick = now_ms;
		test_active = false;
		stop_motor();
	};

	if (!fire_mode_enabled && !test_permitted)
	{
		Stop(motor);
		return;
	}

	if (motor == nullptr)
	{
		feedback_fault = true;
		set_fault(FEEDBACK_LOST);
		return;
	}

	if (state == FAULT)
	{
		stop_motor();
		return;
	}

	if (!motor->feedback_online_)
	{
		feedback_fault = true;
		set_fault(FEEDBACK_LOST);
		return;
	}

	if (motor->temperature > 70)
	{
		set_fault(MOTOR_OVER_TEMPERATURE);
		return;
	}

	feedback_fault = false;

	auto drive_speed = [this, motor](int32_t target_speed)
	{
		/*
		 * 从ACE或BRAKE切换到速度模式时，
		 * 清理未完成的角度目标。
		 */
		if (motor->mode != SPD)
		{
			motor->need_curcircle = 0.0f;
			motor->need_angle = 0;
			motor->count = 0;
			motor->brake_ticks = 0;

			motor->mode = SPD;
		}

		target_speed = ClampCurrent(target_speed,SUPPLY_MAX_SPEED);

		command_speed = RampInt32(
		    command_speed,
			target_speed,
			SUPPLY_RAMP_STEP);

		motor->setspeed = command_speed;
	};

	auto stalled = [this, motor, now_ms]()
	{
		return
			(now_ms - operation_start_tick) >= SUPPLY_STARTUP_GRACE_MS &&
			AbsInt32(motor->setspeed) >= 300 &&
			AbsInt32(motor->curspeed) <= SUPPLY_STALL_SPEED &&
			AbsInt32(motor->torque_current) >= SUPPLY_STALL_CURRENT;
	};

	if (test_permitted && test_requested)
	{
		if (test_finished)
		{
			stop_motor();
			return;
		}

		if (state != TEST_FEED)
		{
			test_active = true;
			test_start_tick = now_ms;
			operation_start_tick = now_ms;
			jam_start_tick = 0;
			enter_state(TEST_FEED);
		}

		if ((now_ms - test_start_tick) >= SUPPLY_TEST_TIMEOUT_MS)
		{
			test_finished = true;
			set_fault(TEST_TIMEOUT);
			return;
		}

		drive_speed(test_speed);

		if (stalled())
		{
			if (jam_start_tick == 0U)
			{
				jam_start_tick = now_ms;
			}

			if ((now_ms - jam_start_tick) >= SUPPLY_STALL_TIME_MS)
			{
				jam_detected = true;
				test_finished = true;
				set_fault(TEST_JAM);
			}
		}
		else
		{
			jam_start_tick = 0;
		}
		return;
	}

	test_active = false;

	if (!fire_mode_enabled)
	{
		Stop(motor);
		return;
	}

	if (!friction_feedback_ok)
	{
		set_fault(FEEDBACK_LOST);
		return;
	}

	// 单发触发释放达到规定时间后才重新解锁。
	if (!single_request)
	{
		if (!single_release_timing)
		{
			single_release_timing = true;
			single_release_tick = now_ms;
		}
		else if ((now_ms - single_release_tick) >= SUPPLY_SINGLE_REARM_MS)
		{
			single_armed = true;
		}
	}
	else
	{
		single_release_timing = false;
	}

	// 连发触发至少释放一个控制周期后才重新解锁。
	if (!continuous_request)
	{
		continuous_armed = true;
	}

	if (!fire_permitted)
	{
		stop_motor();
		operation_start_tick = 0;
		jam_retry_count = 0;
		resume_state = STOPPED;
		enter_state(WAIT_READY);
		return;
	}

	if (!friction_ready)
	{
		stop_motor();

		if (state != WAIT_READY)
		{
			operation_start_tick = 0;
			jam_retry_count = 0;
			resume_state = STOPPED;
			enter_state(WAIT_READY);
		}

		if ((now_ms - state_start_tick) >= SUPPLY_FRICTION_TIMEOUT_MS)
		{
			set_fault(FRICTION_TIMEOUT);
		}
		return;
	}

	if (state == STOPPED || state == WAIT_READY || state == TEST_FEED)
	{
		stop_motor();
		enter_state(READY);
	}

	STATE active_operation = state;
	if (state == JAM_CONFIRM || state == JAM_REVERSE || state == RECOVER_WAIT)
	{
		active_operation = resume_state;
	}

	if (active_operation == SINGLE_FEED &&
		(now_ms - operation_start_tick) >= SUPPLY_SINGLE_TIMEOUT_MS)
	{
		set_fault(SINGLE_TIMEOUT);
		return;
	}

	if (active_operation == CONTINUOUS_FEED &&
		(now_ms - operation_start_tick) >= SUPPLY_CONTINUOUS_TIMEOUT_MS)
	{
		set_fault(CONTINUOUS_TIMEOUT);
		return;
	}

	switch (state)
	{
	case READY:
		stop_motor();
		jam_detected = false;
		jam_retry_count = 0;

		// 两个触发量同时超过阈值时优先连发。
		if (single_request && !continuous_request && single_armed)
		{
			single_armed = false;
			/*
			 * 保存最终目标，堵转反转后仍然回到这个目标，
			 * 不会重新多走完整一格。
			 */
			single_target_angle =motor->sum_angle - SUPPLY_SINGLE_STEP_COUNTS;

			single_start_ace_completion_count = motor->ace_completion_count;

			motor->setspeed = 0;
			motor->current = 0;

			motor->need_angle = 0;
			motor->count = 0;
			motor->brake_ticks = 0;

			/*
			 * 一格为-42130编码器计数。
			 */
			motor->need_curcircle = -static_cast<float>( SUPPLY_SINGLE_STEP_COUNTS) /8192.0f;

			motor->mode = ACE;

			operation_start_tick = now_ms;

			enter_state(SINGLE_FEED);
		}
		else if (continuous_request && continuous_armed)
		{
			continuous_armed = false;
			operation_start_tick = now_ms;
			enter_state(CONTINUOUS_FEED);
		}
		break;

	case SINGLE_FEED:
		{
			/*
			 * ACE到达目标后先进入BRAKE。
			 * 必须等BRAKE结束并进入IDLE，才记录完成。
			 */
			const bool ace_completed = motor->ace_completion_count != single_start_ace_completion_count;

			if (ace_completed)
			{
				if (motor->mode == IDLE)
				{
					stop_motor();

					completed_single_count++;
					operation_start_tick = 0;
					jam_retry_count = 0;

					enter_state(READY);
				}

				/*
				 * mode仍为BRAKE时继续等待，不允许stop_motor
				 * 提前取消主动制动。
				 */
				break;
			}

			/*
			 * 堵转确认或反转恢复后，电机可能处于SPD或IDLE。
			 * 此时根据原目标位置重新计算剩余路程，
			 * 而不是重新走完整42130。
			 */
			if (motor->mode != ACE &&motor->mode != BRAKE)
			{
				int32_t remaining_counts = single_target_angle - motor->sum_angle;

				/*
				 * 正好为0时给一个最小非零值，
				 * 避免need_angle==0被ACE当成未初始化。
				 */
				if (remaining_counts == 0)
				{
					remaining_counts = -1;
				}

				motor->setspeed = 0;
				motor->current = 0;

				motor->need_angle = 0;
				motor->count = 0;
				motor->brake_ticks = 0;

				motor->need_curcircle =static_cast<float>( remaining_counts) /8192.0f;

				motor->mode = ACE;
			}

			/*
			 * ACE运行过程中仍然检测堵转。
			 */
			if (motor->mode == ACE && stalled())
			{
				/*
				 * 暂停ACE，进入原来的堵转确认流程。
				 */
				motor->setspeed = 0;
				motor->current = 0;

				motor->need_curcircle = 0.0f;
				motor->need_angle = 0;
				motor->count = 0;
				motor->brake_ticks = 0;

				motor->mode = IDLE;

				resume_state = SINGLE_FEED;
				jam_start_tick = now_ms;

				enter_state(JAM_CONFIRM);
			}

			break;
		}
		drive_speed(single_speed);
		if (stalled())
		{
			resume_state = SINGLE_FEED;
			jam_start_tick = now_ms;
			enter_state(JAM_CONFIRM);
		}
		break;

	case CONTINUOUS_FEED:
		if (!continuous_request)
		{
			stop_motor();
			operation_start_tick = 0;
			jam_retry_count = 0;
			enter_state(READY);
			break;
		}

		drive_speed(continuous_speed);
		if (stalled())
		{
			resume_state = CONTINUOUS_FEED;
			jam_start_tick = now_ms;
			enter_state(JAM_CONFIRM);
		}
		break;

	case JAM_CONFIRM:
		if (resume_state == CONTINUOUS_FEED && !continuous_request)
		{
			stop_motor();
			operation_start_tick = 0;
			jam_retry_count = 0;
			enter_state(READY);
			break;
		}

		drive_speed(
			resume_state == SINGLE_FEED ?
				single_speed : continuous_speed);

		if (!stalled())
		{
			jam_start_tick = 0;
			enter_state(resume_state);
			break;
		}

		if ((now_ms - jam_start_tick) >= SUPPLY_STALL_TIME_MS)
		{
			if (jam_retry_count >= SUPPLY_MAX_JAM_RETRIES)
			{
				set_fault(JAM_RETRY_EXCEEDED);
				break;
			}

			jam_retry_count++;
			jam_count++;
			jam_detected = true;
			stop_motor();
			reverse_start_angle = motor->sum_angle;
			enter_state(JAM_REVERSE);
		}
		break;

	case JAM_REVERSE:
	{
		drive_speed(reverse_speed);
		const uint32_t reverse_elapsed = now_ms - state_start_tick;
		const int32_t reverse_progress =
			motor->sum_angle - reverse_start_angle;

		if (reverse_elapsed >= SUPPLY_REVERSE_MIN_MS &&
			reverse_progress >= SUPPLY_REVERSE_COUNTS)
		{
			stop_motor();
			enter_state(RECOVER_WAIT);
		}
		else if (reverse_elapsed >= SUPPLY_REVERSE_TIMEOUT_MS)
		{
			set_fault(REVERSE_TIMEOUT);
		}
		break;
	}

	case RECOVER_WAIT:
		stop_motor();
		if ((now_ms - state_start_tick) >= SUPPLY_RECOVER_WAIT_MS)
		{
			recovery_count++;
			jam_start_tick = 0;

			if (resume_state == CONTINUOUS_FEED && !continuous_request)
			{
				operation_start_tick = 0;
				enter_state(READY);
			}
			else
			{
				enter_state(resume_state);
			}
		}
		break;

	case FAULT:
	case STOPPED:
	case WAIT_READY:
	case TEST_FEED:
	default:
		stop_motor();
		break;
	}
}
float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}


