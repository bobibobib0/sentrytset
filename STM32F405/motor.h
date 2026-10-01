#pragma once
#include <cinttypes>
#include <cstring>
#include <cmath>
#include "PID.h"
#include "kalman.h"
#include "label.h"
#define WHEEL_RADIUS_MM 47.5f
#define GEAR_RATIO      36.0f

constexpr auto MAXSPEED = 5000;
//constexpr auto ADJUSTSPEED = 3000;

enum { ID1 = 0x205, ID2, ID3, ID4, ID5, ID6, ID7, ID8 };
enum { pre = 0, now };
enum pid_mode { speed = 0, position, speed2 };
enum motor_type { M3508, M3510, M2310, EC60, M6623, M6020, M2006 };
enum motor_mode { SPD, POS, ACE, wheel_left, wheel_right,BRAKE, IDLE};
enum function_type { chassis, pantile, shooter, supply };
typedef enum { UNINIT, UNCONNECTED, DISCONNECTED, FINE }motor_status_t;
#define SQRTF(x) ((x)>0?sqrtf(x):-sqrtf(-x))
#define T 1.e-3f

class Motor
{
	typedef motor_type type_t;
public:
	uint32_t ID;
	Kalman kalman{ 1.f,40.f };
	Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _speed2);
	Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position);// ACE 模式
	Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed);
	void recorded_the_Laps();
	void Ontimer(
		uint8_t idata[][8],
		uint8_t* odata,
		const volatile uint32_t feedback_count[],
		const volatile uint32_t last_feedback_tick[],
		uint32_t now_ms);
private:
	motor_status_t m_status = UNINIT;
	static constexpr uint32_t FEEDBACK_TIMEOUT_MS = 100U;
	void getmax(const type_t type);
	void StatusIdentifier(uint32_t feedback_count, uint32_t last_feedback_tick, uint32_t now_ms);
	void GetDistanceFromMechanicalAngle();
	static int16_t getword(const uint8_t high, const uint8_t low);
	static int32_t setrange(const int32_t original, const int32_t range);
	type_t type;
public:
	function_type function;
	float need_curcircle = 0.0f;
	int32_t stopAngle = 0;
	int32_t count = 0;
	int32_t need_angle = 0;
	uint8_t brake_ticks = 0;
	volatile int32_t last_ace_target_angle = 0;
	volatile int32_t last_ace_final_count = 0;
	volatile int32_t last_ace_final_remaining_angle = 0;
	volatile int32_t last_ace_final_speed = 0;
	volatile uint32_t ace_completion_count = 0;
	static int16_t getdeltaa(int16_t diff);
	uint8_t getStatus()const;
	// 实际 CAN 反馈流的 LiveWatch 观测量。
	volatile uint32_t feedback_count_ = 0;
	volatile uint32_t last_feedback_tick_ = 0;
	volatile bool feedback_online_ = false;
	int32_t current{}, curspeed{}, setcurrent{},setspeed{}, torque_current, motor_status, motor_angle_status, sum_angle{};//这个current用于输出电流或者电压
	int16_t adjspeed{};
	int16_t maxspeed{}, maxcurrent{};
	Kalman currentKalman{ 1.f,40.f };
	int temperature;
	int32_t mode{};
	int round_count;
	bool pd = 0, spinning = 0;//pd:单次拨弹 spinning:一秒八发
	// speed、position、speed2 三个索引都会被构造函数使用。
	PID pid[3];
	float Torque_constant_2006 = (0.18*10)/10000;
	float setangle{}, angle[2]{},distance{}, initial_x{}, rota_angle{}, reset_rota_angle{}, delta_angle{};
	float Torque_left;
	float Torque_right;
	float const_dx = 0.004974;// m / s  /rpm
};
//此处要根据实际不同can线上的电机数量进行更改
extern Motor can1_motor[CAN1_MOTOR_NUM];
extern Motor can2_motor[CAN2_MOTOR_NUM];
