#pragma once
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <cstdint>
#include"label.h"
#include "can.h"


enum class DMControlMode : uint8_t 
{
	Mit,  // 发送位置、速度、Kp、Kd、力矩。
	PositionSpeed, 
	Speed
};//达妙控制模式

namespace DMProtocol
{
	constexpr float V_MIN = -45.0f;
	constexpr float V_MAX = 45.0f;//速度反馈和 MIT 速度指令范围，单位 rad/s。
	constexpr float KP_MIN = 0.0f;
	constexpr float KP_MAX = 500.0f;//MIT 位置比例参数范围。
	constexpr float KD_MIN = 0.0f;
	constexpr float KD_MAX = 5.0f;//MIT 速度微分参数范围。
	constexpr float CURRENT_MIN = -40.0f;
	constexpr float CURRENT_MAX = 40.0f;//反馈电流范围，单位 A。
	constexpr float TORQUE_MIN = -20.0f;
	constexpr float TORQUE_MAX = 20.0f;//MIT 前馈力矩范围，单位 N·m。
	constexpr float TORQUE_CONSTANT = 1.4f;// 电流转换为力矩使用的转矩常数。

	constexpr uint32_t FEEDBACK_TIMEOUT_MS = 100U;
	constexpr uint32_t ENABLE_RETRY_MS = 500U;
}

class DMMOTOR
{
public:
	DMMOTOR(
		uint32_t feedback_id,
		uint32_t control_id,
		CAN* can,
		DMControlMode mode,
		float position_min,
		float position_max
	);

	uint32_t GetFeedbackID() const;
	uint32_t GetControlID() const;
	CAN* GetCan() const;

	uint8_t* GetRxBuffer();
	uint8_t* GetTxBuffer();

	// CAN中断只负责交付原始反馈。
	void OnCanFrame(const uint8_t* data, uint32_t now_ms);

	// 电机任务依次调用。
	bool DecodeFeedback();
	void Update(uint32_t now_ms);
	void EncodeCommand();

	// 控制层或调试配置提出使能请求。
	void SetEnableRequest(bool enable);

	// 发送任务调用，内部决定使能、失能或发送控制帧。
	void Transmit(uint32_t now_ms);

	bool HasFeedback() const;
	bool FeedbackFresh(uint32_t now_ms, uint32_t timeout_ms) const;

	uint32_t GetFeedbackCount() const;
	uint32_t GetLastFeedbackTick() const;

public:
	float pos = 0.0f;
	float curSpeed = 0.0f;
	float current = 0.0f;
	float torque = 0.0f;

	float setPos = 0.0f;
	float setSpeed = 0.0f;
	float setCurrent = 0.0f;
	float setTorque = 0.0f;

	float Kp = 0.0f;
	float Kd = 0.0f;

private:
	static float UintToFloat(
		uint16_t value,
		float minimum,
		float maximum,
		uint8_t bits
	);

	static uint16_t FloatToUint(
		float value,
		float minimum,
		float maximum,
		uint8_t bits
	);

	HAL_StatusTypeDef Enable();
	HAL_StatusTypeDef Disable();
	HAL_StatusTypeDef TransmitCommand();

private:
	uint32_t feedback_id_;
	uint32_t control_id_;

	CAN* can_;
	DMControlMode mode_;

	float position_min_;
	float position_max_;

	uint8_t rx_buffer_[8] = { 0 };
	uint8_t tx_buffer_[8] = { 0 };

	volatile uint32_t feedback_count_ = 0;
	volatile uint32_t last_feedback_tick_ = 0;

	uint32_t decoded_feedback_count_ = 0;
	uint32_t last_enable_tick_ = 0xFFFFFFFFU;

	bool enable_requested_ = false;
	bool enable_command_sent_ = false;
	bool feedback_online_ = false;
	bool target_initialized_ = false;
	bool command_encoded_ = false;
};

extern DMMOTOR DM_motorYaw;
extern DMMOTOR DM_motorPitch;