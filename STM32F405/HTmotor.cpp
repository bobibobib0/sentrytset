#include "HTmotor.h"
#include "delay.h"
#include "can.h"
#include "stdio.h"
#include <cstring>

#define now 0
#define last 1
//void buffer_append_int32(uint8_t* buffer, int32_t number, int16_t* index) {
//	buffer[(*index)++] = number >> 24;
//	buffer[(*index)++] = number >> 16;
//	buffer[(*index)++] = number >> 8;
//	buffer[(*index)++] = number;
//}
//void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t* index) {
//	buffer[(*index)++] = number >> 8;
//	buffer[(*index)++] = number;
//}

DMMOTOR::DMMOTOR(
	uint32_t feedback_id,
	uint32_t control_id,
	CAN* can,
	DMControlMode mode,
	float position_min,
	float position_max
)
	: feedback_id_(feedback_id),
	control_id_(control_id),
	can_(can),
	mode_(mode),
	position_min_(position_min),
	position_max_(position_max)
{
}

uint32_t DMMOTOR::GetFeedbackID() const
{
	return feedback_id_;
}

uint32_t DMMOTOR::GetControlID() const
{
	return control_id_;
}

CAN* DMMOTOR::GetCan() const
{
	return can_;
}

uint8_t* DMMOTOR::GetRxBuffer()
{
	return rx_buffer_;
}

uint8_t* DMMOTOR::GetTxBuffer()
{
	return tx_buffer_;
}

void DMMOTOR::OnCanFrame(const uint8_t* data,uint32_t now_ms)
{
	if (data == nullptr)
	{
		return;
	}

	/*
	 * 必须先复制完整数据，再增加反馈计数。
	 *
	 * MotorUpdateTask 可以通过前后计数是否一致，
	 * 判断复制期间有没有新中断进入。
	 */
	memcpy(rx_buffer_, data, sizeof(rx_buffer_));

	last_feedback_tick_ = now_ms;
	++feedback_count_;
}

bool DMMOTOR::DecodeFeedback()
{
	const uint32_t frame_number = feedback_count_;

	/*
	 * frame_number == 0：
	 *   从未收到过反馈。
	 *
	 * frame_number == decoded_feedback_count_：
	 *   当前反馈已经解码过。
	 */
	if (frame_number == 0U ||frame_number == decoded_feedback_count_)
	{
		return false;
	}

	uint8_t frame[8];
	memcpy(frame, rx_buffer_, sizeof(frame));

	/*
	 * 如果复制期间又收到了一帧，就放弃本次解码。
	 * 下一个 2 ms 周期会重新读取最新完整帧。
	 */
	if (frame_number != feedback_count_)
	{
		return false;
	}

	/*
	 * 达妙反馈格式：
	 *
	 * Byte1、Byte2：
	 *   16 bit 位置。
	 *
	 * Byte3、Byte4 高4位：
	 *   12 bit 速度。
	 *
	 * Byte4 低4位、Byte5：
	 *   12 bit 电流。
	 */
	const uint16_t position_raw =(static_cast<uint16_t>(frame[1]) << 8) |static_cast<uint16_t>(frame[2]);
	const uint16_t speed_raw =(static_cast<uint16_t>(frame[3]) << 4) |(static_cast<uint16_t>(frame[4]) >> 4);
	const uint16_t current_raw =(static_cast<uint16_t>(frame[4] & 0x0FU) << 8) |static_cast<uint16_t>(frame[5]);

	pos = UintToFloat(position_raw,position_min_,position_max_,16);
	curSpeed = UintToFloat(speed_raw,DMProtocol::V_MIN,DMProtocol::V_MAX,12);
	current = UintToFloat(current_raw,DMProtocol::CURRENT_MIN,DMProtocol::CURRENT_MAX,12);
	torque = current * DMProtocol::TORQUE_CONSTANT;

	decoded_feedback_count_ = frame_number;
	return true;
}

void DMMOTOR::Update(uint32_t now_ms)
{
	feedback_online_ = FeedbackFresh(now_ms, DMProtocol::FEEDBACK_TIMEOUT_MS);

	if (!feedback_online_)
	{
		command_encoded_ = false;
		target_initialized_ = false;
		return;
	}

	if (target_initialized_)
	{
		return;
	}

	// 收到首帧反馈后，为当前模式设置安全初值。
	switch (mode_)
	{
	case DMControlMode::Mit:
		setPos = pos;
		setSpeed = 0.0f;
		setTorque = 0.0f;
		Kp = 0.0f;
		Kd = 0.0f;
		break;

	case DMControlMode::PositionSpeed:
		setPos = pos;
		setSpeed = 0.0f;
		break;

	case DMControlMode::Speed:
		setSpeed = 0.0f;
		break;
	}

	target_initialized_ = true;
}
void DMMOTOR::EncodeCommand()
{
	command_encoded_ = false;

	if (!enable_requested_ ||!feedback_online_ ||!target_initialized_)
	{
		return;
	}
	memset(tx_buffer_, 0, sizeof(tx_buffer_));

	if (mode_ == DMControlMode::Mit)
	{
		const uint16_t position_uint = FloatToUint(setPos,position_min_,position_max_,16);
		const uint16_t speed_uint = FloatToUint(setSpeed,DMProtocol::V_MIN,DMProtocol::V_MAX,12);
		const uint16_t kp_uint = FloatToUint(Kp,DMProtocol::KP_MIN,DMProtocol::KP_MAX,12);
		const uint16_t kd_uint = FloatToUint(Kd,DMProtocol::KD_MIN,DMProtocol::KD_MAX,12);
		const uint16_t torque_uint = FloatToUint(setTorque,DMProtocol::TORQUE_MIN,DMProtocol::TORQUE_MAX,12);

		tx_buffer_[0] =static_cast<uint8_t>(position_uint >> 8);
		tx_buffer_[1] =static_cast<uint8_t>(position_uint & 0xFFU);
		tx_buffer_[2] =static_cast<uint8_t>(speed_uint >> 4);
		tx_buffer_[3] =static_cast<uint8_t>(((speed_uint & 0x0FU) << 4) |(kp_uint >> 8));
		tx_buffer_[4] =static_cast<uint8_t>(kp_uint & 0xFFU);
		tx_buffer_[5] =static_cast<uint8_t>(kd_uint >> 4);
		tx_buffer_[6] =static_cast<uint8_t>(((kd_uint & 0x0FU) << 4) |(torque_uint >> 8));
		tx_buffer_[7] =static_cast<uint8_t>(torque_uint & 0xFFU);
	}
	else if (mode_ == DMControlMode::PositionSpeed)
	{
		memcpy(&tx_buffer_[0],&setPos,sizeof(float));
		memcpy(&tx_buffer_[4],&setSpeed,sizeof(float));
	}
	else if (mode_ == DMControlMode::Speed)
	{
		memcpy(&tx_buffer_[0],&setSpeed,sizeof(float));
	}

	command_encoded_ = true;
}

void DMMOTOR::SetEnableRequest(bool enable)
{
	if (enable_requested_ == enable)
	{
		return;
	}

	enable_requested_ = enable;
	command_encoded_ = false;
	target_initialized_ = false;

	if (enable)
	{
		// 让下一次 Transmit() 立即发送使能帧。
		last_enable_tick_ = 0xFFFFFFFFU;
	}
}

void DMMOTOR::Transmit(uint32_t now_ms)
{
	if (!enable_requested_)
	{
		if (enable_command_sent_)
		{
			Disable();
		}

		return;
	}

	if (!enable_command_sent_ ||!feedback_online_)
	{
		const bool first_enable =last_enable_tick_ == 0xFFFFFFFFU;

		const bool retry_timeout =(now_ms - last_enable_tick_) >=DMProtocol::ENABLE_RETRY_MS;

		if (first_enable || retry_timeout)
		{
			Enable();
			last_enable_tick_ = now_ms;
		}

		return;
	}

	TransmitCommand();
}
HAL_StatusTypeDef DMMOTOR::Enable()
{
	if (can_ == nullptr)
	{
		return HAL_ERROR;
	}

	/*
	 * 达妙进入电机控制模式的特殊帧。
	 */
	const uint8_t enable_frame[8] =
	{
		0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFC
	};

	const HAL_StatusTypeDef status =can_->Transmit(control_id_,enable_frame,8);

	if (status == HAL_OK)
	{
		enable_command_sent_ = true;

		/*
		 * 使能以后必须重新编码命令，
		 * 防止发送使能前残留的旧控制数据。
		 */
		command_encoded_ = false;
	}
	return status;
}

HAL_StatusTypeDef DMMOTOR::Disable()
{
	if (can_ == nullptr)
	{
		return HAL_ERROR;
	}

	/*
	 * 达妙退出电机控制模式的特殊帧。
	 */
	const uint8_t disable_frame[8] =
	{	
		0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFD
	};

	const HAL_StatusTypeDef status =can_->Transmit(control_id_,disable_frame,8);

	if (status == HAL_OK)
	{
		enable_command_sent_ = false;
		command_encoded_ = false;
	}
	return status;
}

HAL_StatusTypeDef DMMOTOR::TransmitCommand()
{
	/*
	 * 必须同时满足：
	 *
	 * 1. CAN 对象有效；
	 * 2. 已经发送过使能命令；
	 * 3. 已经收到过电机反馈；
	 * 4. 已经完成控制帧编码。
	 */
	if (can_ == nullptr ||
		!enable_requested_ ||
		!enable_command_sent_ ||
		!feedback_online_ ||
		!command_encoded_)
	{
		return HAL_ERROR;
	}

	return can_->Transmit(	control_id_,tx_buffer_,8);
}

bool DMMOTOR::HasFeedback() const
{
	return feedback_count_ != 0U;
}

bool DMMOTOR::FeedbackFresh(uint32_t now_ms,uint32_t timeout_ms) const
{
	if (!HasFeedback())
	{
		return false;
	}

	/*
	 * 使用无符号减法，可以兼容 HAL_GetTick() 溢出。
	 */
	return
		(now_ms - last_feedback_tick_) <= timeout_ms;
}

uint32_t DMMOTOR::GetFeedbackCount() const
{
	return feedback_count_;
}

uint32_t DMMOTOR::GetLastFeedbackTick() const
{
	return last_feedback_tick_;
}

float DMMOTOR::UintToFloat(uint16_t value,float minimum,float maximum,uint8_t bits)
{
	const uint32_t maximum_integer =(1UL << bits) - 1UL;

	return
		static_cast<float>(value) *
		(maximum - minimum) /
		static_cast<float>(maximum_integer) +
		minimum;
}

uint16_t DMMOTOR::FloatToUint(float value,float minimum,float maximum,uint8_t bits)
{
	/*
	 * 在编码前限制目标范围，
	 * 避免数值越界后发生无符号整数回绕。
	 */
	if (value < minimum)
	{
		value = minimum;
	}
	else if (value > maximum)
	{
		value = maximum;
	}

	const uint32_t maximum_integer =(1UL << bits) - 1UL;

	return static_cast<uint16_t>(
		(value - minimum) *
		static_cast<float>(maximum_integer) /
		(maximum - minimum)
		);
}