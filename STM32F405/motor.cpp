#include "motor.h"
#include "gpio.h"
#include "HTmotor.h"
#include "imu.h"
#define DEG_TO_RAD 0.017453292f  // π / 180
Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _speed2)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	memcpy(&pid[speed2], &_speed2, sizeof(PID));
	this->function = function;
}


Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	this->function = function;
}

Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	this->function = function;
}

void Motor::StatusIdentifier(
	uint32_t feedback_count,
	uint32_t last_feedback_tick,
	uint32_t now_ms)
{
	feedback_count_ = feedback_count;
	last_feedback_tick_ = last_feedback_tick;

	// 启动后该电机 ID 尚未收到过反馈。
	if (feedback_count == 0U)
	{
		feedback_online_ = false;
		m_status = UNCONNECTED;
		return;
	}

	// 无符号时间差可正确处理 HAL_GetTick() 回绕。
	feedback_online_ = (now_ms - last_feedback_tick) <= FEEDBACK_TIMEOUT_MS;
	m_status = feedback_online_ ? FINE : DISCONNECTED;
}
uint8_t Motor::getStatus()const
{
	return (uint8_t)m_status;
}
void Motor::Ontimer(
	uint8_t idata[][8],
	uint8_t* odata,
	const volatile uint32_t feedback_count[],
	const volatile uint32_t last_feedback_tick[],
	uint32_t now_ms)// idata 为接收缓存，odata 为发送缓存。
{
	const uint32_t logical_id_index = this->ID - ID1;
	uint32_t receive_index = logical_id_index;
	uint32_t transmit_index = logical_id_index;

	/*----------------------------------------------------------------*/
	if (this->type == M6020)
	{
		// GM6020 ID1~ID4 的反馈为 0x205~0x208，
		// 在 can.data[StdId - 0x201] 中对应索引 4~7。
		receive_index += 4;

		// GM6020 ID1~ID4 使用 0x1FF，与普通电机 ID5~ID8 共用第二组8字节缓存。
		// 小 Yaw 为 ID1，因此写入 temp_data[8..9]，发送时对应 0x1FF Data[0..1]。
		if (logical_id_index < 4)
		{
			transmit_index += 4;
		}
	}
	/*----------------------------------------------------------------*/
	this->torque_current = getword(idata[receive_index][4], idata[receive_index][5]);//从CAN接收缓存读反馈电流
	this->StatusIdentifier(
		feedback_count[receive_index],
		last_feedback_tick[receive_index],
		now_ms);
	this->angle[now] = getword(idata[receive_index][0], idata[receive_index][1]);//读编码器机械角度
	this->temperature = idata[receive_index][6];//读电机温度
	// 读取当前转速。

	motor_status = 0;
	if (temperature > 70) {
		setspeed = 0;
	}

	if (type == EC60)
	{
		curspeed = static_cast<float>(getdeltaa(angle[now] - angle[pre])) / T / 8192.f * 60.f;
	}
	else {
		curspeed = getword(idata[receive_index][2], idata[receive_index][3]);
	}
	//20220121--hz
	/*
 * 目前只让供弹M2006使用底层模式控制。
 *
 * 其他电机的current仍由原来的control.cpp计算，
 * 避免意外激活未使用的M6020位置环。
 */
	if (type == M2006 && function == supply)
	{
		/*
		 * 反馈丢失或过温时，底层直接关闭输出。
		 */
		if (!feedback_online_ || temperature > 70)
		{
			setspeed = 0;
			current = 0;

			need_curcircle = 0.0f;
			need_angle = 0;
			count = 0;
			brake_ticks = 0;

			mode = IDLE;
		}
		else if (mode == ACE)
		{
			/*
			 * ACE开始时，把相对圈数转换成编码器计数。
			 * 7槽时： need_curcircle = -36/7，need_angle约为-42130
			 */
			if (need_angle == 0)
			{
				stopAngle = angle[now];
				count = 0;
				need_angle = static_cast<int32_t>(need_curcircle * 8192.0f);
				last_ace_target_angle = need_angle;
			}

			/*
			 * 累加本次ACE实际走过的编码器计数。
			 * getdeltaa负责处理8191→0的编码器回绕。
			 */
			count += getdeltaa(angle[now] - stopAngle);
			stopAngle = angle[now];
			const int32_t remaining_angle = need_angle - count;
			constexpr int32_t ACE_SETTLE_TOLERANCE = 300;

			/*
			 * 进入目标容差后切换到主动制动。
			 */
			if (need_angle != 0 && abs(remaining_angle) <= ACE_SETTLE_TOLERANCE)
			{
				last_ace_target_angle = need_angle;
				last_ace_final_count = count;
				last_ace_final_remaining_angle =remaining_angle;
				last_ace_final_speed =curspeed;
				ace_completion_count++;

				setspeed = 0;
				current = 0;

				need_angle = 0;
				count = 0;
				need_curcircle = 0.0f;

				/*
				 * MotorUpdateTask周期为2ms。
				 * 20个周期约为40ms主动制动。
				 */
				brake_ticks = 20;
				mode = BRAKE;
			}
			else
			{
				/*
				 * 位置环根据剩余角度计算目标速度。
				 */
				setspeed =pid[position].Position(static_cast<float>(  remaining_angle),10000.0f);

				/*
				 * 第一次测试先使用保守速度。
				 * 距离目标5000以内进一步降速。
				 */
				constexpr int32_t ACE_MAX_SPEED = 400;
				constexpr int32_t ACE_NEAR_SPEED = 150;
				constexpr int32_t ACE_NEAR_COUNTS = 5000;
				
				//根据剩余角度判断速度限制，接近目标时降低速度
				const int32_t ace_speed_limit =abs(remaining_angle) <ACE_NEAR_COUNTS? ACE_NEAR_SPEED: ACE_MAX_SPEED;

				setspeed = setrange(setspeed,ace_speed_limit);

				/*
				 * 速度环根据目标速度输出电流。
				 */
				current = pid[speed].Position(  static_cast<float>( setspeed - curspeed),	8000.0f);

				constexpr int32_t ACE_CURRENT_LIMIT = 10000;

				current = setrange(current,ACE_CURRENT_LIMIT);
			}
		}
		else if (mode == SPD)
		{
			/*
			 * 连发、拨弹测试、堵转确认和反转
			 * 都使用速度模式。
			 */
			current = pid[speed].Position( static_cast<float>( setspeed - curspeed),8000.0f);

			// 仅在SPD模式降低控制强度：
// 等效Kp从20降到10，不影响ACE单发。
			current /= 2;

			constexpr int32_t SUPPLY_SPEED_CURRENT_LIMIT = 8000;

			current = setrange(current,SUPPLY_SPEED_CURRENT_LIMIT);
		}
		else if (mode == POS)
		{
			if (!feedback_online_ || temperature > 70)
			{
				setspeed = 0;
				current = 0;
			}
			else
			{
				float angle_error = setangle - angle[now];

				// 取单圈范围内的最短旋转路径
				if (angle_error > 4096.0f)
				{
					angle_error -= 8192.0f;
				}
				else if (angle_error < -4096.0f)
				{
					angle_error += 8192.0f;
				}

				// 位置环输出目标速度
				setspeed = pid[position].Position( angle_error,static_cast<float>(maxspeed));

				// 速度环输出电流
				current = pid[speed].Position( setspeed - curspeed,static_cast<float>(maxcurrent));

				current = setrange(current, maxcurrent);
			}
		}
		else if (mode == BRAKE)
		{
			setspeed = 0;

			/*
			 * 电流方向始终与实际速度相反。
			 */
			constexpr int32_t BRAKE_CURRENT_LIMIT = 2000;

			current = setrange(-curspeed * 5,BRAKE_CURRENT_LIMIT);

			if (brake_ticks > 0)
			{
				brake_ticks--;
			}
			else
			{
				current = 0;
				mode = IDLE;
			}
		}
		else if (mode == IDLE)
		{
			setspeed = 0;
			current = 0;
		}
	}
	else if (type == M6020 && function == pantile)
	{
		// 小 Yaw GM6020：位置外环、速度内环，最终输出电流
		if (!feedback_online_ || temperature > 70)
		{
			setspeed = 0;
			current = 0;
		}
		else if (mode == POS)
		{
			float angle_error = setangle - angle[now];

			// 跨越 0/8191 边界时取最短旋转路径。
			if (angle_error > 4096.0f)
			{
				angle_error -= 8192.0f;
			}
			else if (angle_error < -4096.0f)
			{
				angle_error += 8192.0f;
			}

			const bool position_in_deadband =(angle_error >= -3.0f) && (angle_error <= 3.0f);
			const bool speed_in_deadband =(curspeed >= -5) && (curspeed <= 5);

			if (position_in_deadband && speed_in_deadband)
			{
				// 位于死区时清零输出，避免编码器抖动造成电流反向。
				setspeed = 0;
				current = 0;
			}
			else
			{
				setspeed = pid[position].Position(angle_error,static_cast<float>(maxspeed));
				setspeed = setrange(setspeed, maxspeed);

				current = pid[speed].Position(static_cast<float>(setspeed - curspeed),static_cast<float>(maxcurrent));
				current = setrange(current, maxcurrent);
			}
		}
		else
		{
			// IDLE 及未支持模式不得保留旧电流指令。
			setspeed = 0;
			current = 0;
		}
	}
	 recorded_the_Laps();
	GetDistanceFromMechanicalAngle();
	angle[pre] = angle[now];
	current = setrange(current, maxcurrent);
	odata[transmit_index * 2] = (current & 0xff00) >> 8;//高八位
	odata[transmit_index * 2 + 1] = current & 0x00ff;
}
void Motor::recorded_the_Laps() {
	int16_t delta = angle[now] - angle[pre];
	// 处理回绕：顺时针
	if (delta > 8192 / 2)
		delta -= 8192;
	// 处理回绕：逆时针
	else if (delta < -8192 / 2)
		delta += 8192;

	sum_angle+= delta;
//	round_count = total_count / encoder_resolution;
}

uint8_t initial_cnt=0;
void Motor::GetDistanceFromMechanicalAngle() {
	if (initial_cnt<5)
	initial_cnt++;
	distance=(6.2831853f/ 8192.0f)*sum_angle * (WHEEL_RADIUS_MM / GEAR_RATIO)-initial_x;  // 单位：mm

	if(initial_cnt<3)
	initial_x = distance;
}

void Motor::getmax(const type_t type)
{
	adjspeed = 3000;
	switch (type)
	{
	case M3508:
		maxcurrent = 16384;
		maxspeed = 3800;
		break;
	case M3510:
		maxcurrent = 13000;
		maxspeed = 9000;
		break;
	case M2310:
		maxcurrent = 13000;
		maxspeed = 9000;
		adjspeed = 1000;
		break;
	case EC60:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6623:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6020:
		maxcurrent = 30000;
		maxspeed = 200;
		adjspeed = 80;
		break;
	case M2006:
		maxcurrent = 10000;
		adjspeed = 1000;
		maxspeed = 3000;
		break;
	default:;
	}
}

int16_t Motor::getdeltaa(int16_t diff)
{
	if (diff <= -4096)
		diff += 8192;
	else if (diff > 4096)
		diff -= 8192;
	return diff;
}

int16_t Motor::getword(const uint8_t high, const uint8_t low)
{
	const int16_t word = high;
	return (word << 8) + low;
}

int32_t Motor::setrange(const int32_t original, const int32_t range)
{
	return std::max(std::min(range, original), -range);
}

