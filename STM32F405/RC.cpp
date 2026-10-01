#include "label.h"
#include "RC.h"
#include "control.h"
#include "xuc.h"

void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

void RC::OnRC()
{
	RC_CheckState();
	RC_Control();

	if (Shift_mode())
	{

	}

}

void RC::OnPC()
{
	;
}

void RC::Update()
{
	if (!IsOnline())
	{
		ctrl.SetMode(CONTROL::STOP);
		ctrl.Chassis_Stop();
		ctrl.Stop_Pantile();
		ctrl.Stop_Friction();
		ctrl.Stop_Supply();
		return;
	}
	
	OnRC();
	OnPC();
}

bool RC::IsOnline() const
{
	if (!has_valid_frame_)
	{
		return false;
	}

	return
		(HAL_GetTick() - last_valid_frame_tick_) <= 100U;// 100ms内收到过有效帧则认为在线
	//否则认为离线
}

void RC::RC_CheckState() {

	switch (RC_STATE(rc.s[0], rc.s[1]))
	{
	case RC_STATE(UP, UP):
		ctrl.SetMode(CONTROL::AUTOAIM);
		break;

	case RC_STATE(UP, MID):
		ctrl.SetMode(CONTROL::ROTATION);
		break;

	case RC_STATE(UP, DOWN):
		ctrl.SetMode(CONTROL::SPINNING);
		break;

	case RC_STATE(MID, UP):
		ctrl.SetMode(CONTROL::FOLLOW);
		break;

	case RC_STATE(MID, MID):
		ctrl.SetMode(CONTROL::RESET);
		break;

	case RC_STATE(MID, DOWN):
		ctrl.SetMode(CONTROL::SEPARATE);//左中右下测试云台
		break;

	case RC_STATE(DOWN, UP):
		// 该模式下持续执行空仓拨弹测试
		ctrl.SetMode(CONTROL::FEED_TEST);
		break;

	case RC_STATE(DOWN, MID):
		// 发射模式：启动摩擦轮，并根据摇杆请求单发或连发
		ctrl.SetMode(CONTROL::FIRE);
		break;

	case RC_STATE(DOWN, DOWN):
	default:
		ctrl.SetMode(CONTROL::STOP);
		break;
	}

}

void RC::RC_Control() 
{
	// FOLLOW 模式：右摇杆控制平移，左摇杆横向控制旋转
	constexpr int16_t CHASSIS_RC_DEADBAND = 20;//遥控器死区限制
	constexpr int16_t CHASSIS_TRANSLATION_MAX_RPM = 3000;
	constexpr int16_t CHASSIS_ROTATION_MAX_RPM = 3000;

	if (ctrl.GetMode() == CONTROL::FOLLOW)
	{
		const int16_t speedx =
			(rc.ch[1] > CHASSIS_RC_DEADBAND || rc.ch[1] < -CHASSIS_RC_DEADBAND) ?
			static_cast<int16_t>(
				static_cast<int32_t>(rc.ch[1]) * CHASSIS_TRANSLATION_MAX_RPM / 660) :
			0;
		const int16_t speedy =
			(rc.ch[0] > CHASSIS_RC_DEADBAND || rc.ch[0] < -CHASSIS_RC_DEADBAND) ?
			static_cast<int16_t>(
				static_cast<int32_t>(rc.ch[0]) * CHASSIS_TRANSLATION_MAX_RPM / 660) :
			0;
		const int16_t speedz =
			(rc.ch[2] > CHASSIS_RC_DEADBAND || rc.ch[2] < -CHASSIS_RC_DEADBAND) ?
			static_cast<int16_t>(
				static_cast<int32_t>(rc.ch[2]) * CHASSIS_ROTATION_MAX_RPM / 660) :
			0;

		ctrl.chassis.speedx = speedx;
		ctrl.chassis.speedy = speedy;
		ctrl.chassis.speedz = speedz;

		ctrl.chassis.Keep_Direction();

		if (ctrl.chassis.speedx != 0 || ctrl.chassis.speedy != 0 || ctrl.chassis.speedz != 0)
		{
			ctrl.Chassis_Mecanum_Run(
			    static_cast<int16_t>(ctrl.chassis.speedx),
				static_cast<int16_t>(ctrl.chassis.speedy),
				static_cast<int16_t>(ctrl.chassis.speedz));
		}
		else
		{
			ctrl.Chassis_Stop();
		}
	}
	else
	{
		ctrl.Chassis_Stop();
	}
	//	if (ctrl.mode != CONTROL::RESET)
	//	{
	//
	//		/*ctrl.chassis.speedx = rc.ch[3] * 4000.f / 660.f;
	//		ctrl.chassis.speedy = -1 * rc.ch[2] * 4000.f / 660.f;
	//		ctrl.chassis.speedz = 0;*/
	//
	//		//ctrl.chassis.Keep_Direction();
	//
	//		switch (ctrl.mode)
	//		{
	//		case CONTROL::ROTATION:
	//
	//			break;
	//
	//		case CONTROL::FOLLOW:
	//
	//			break;
	//
	//		case CONTROL::SEPARATE:
	//
	//			break;
	//
	//		case CONTROL::AUTOAIM:
	//
	//			break;
	//
	//		case CONTROL::FIRE:
	//	
	//			break;
	//
	//		case CONTROL::STOP:
	//
	//			break;
	//
	//		case CONTROL::SPINNING:
	//
	//			break;
	//
	//		default:
	//			ctrl.chassis.speedx = 0;
	//			ctrl.chassis.speedy = 0;
	//			ctrl.chassis.speedz = 0;
	//			break;
	//		}
	//	}
	//	else {
	//		for (auto& motor : can1_motor)
	//		{
	//			motor.setspeed = 0;
	//		}
	//
	//		for (auto& motor : can2_motor)
	//		{
	//			motor.setspeed = 0;
	//		}
	//
	//		DM_motorYaw.setSpeed = 0.0f;
	//		DM_motorPitch.setSpeed = 0.0f;
	//	}
	switch (ctrl.GetMode())
	{
	case CONTROL::SEPARATE:
		// SEPARATE 模式下由遥控器摇杆控制云台。
		ctrl.Control_Pantile(rc.ch[0], rc.ch[1]);
		ctrl.Stop_Friction();
		ctrl.Stop_Supply();
		break;

	case CONTROL::AUTOAIM:
		// 台架精调时，遥控器暂时代替目标偏差输入。
		ctrl.Control_FinePantile(rc.ch[0], rc.ch[1]);
		ctrl.Stop_Friction();
		ctrl.Stop_Supply();
		break;

	case CONTROL::FIRE:
	{
		// 遥控器只把摇杆状态转换为发射请求；
		// 边沿判断、计时和电机动作由控制层完成。
		constexpr int16_t SINGLE_TRIGGER_THRESHOLD = 330;
		constexpr int16_t CONTINUOUS_TRIGGER_THRESHOLD = 80;

		const bool single_request =
			rc.ch[0] > SINGLE_TRIGGER_THRESHOLD ||
			rc.ch[0] < -SINGLE_TRIGGER_THRESHOLD;

		const bool continuous_request =
			rc.ch[1] > CONTINUOUS_TRIGGER_THRESHOLD ||
			rc.ch[1] < -CONTINUOUS_TRIGGER_THRESHOLD;

		ctrl.Hold_Pantile();
		ctrl.Start_Friction();
		ctrl.Set_Fire_Request(single_request, continuous_request);
		break;
	}

	case CONTROL::FEED_TEST:
		// 遥控器只选择测试模式，定速 PID 和保护由控制层完成。
		ctrl.Hold_Pantile();
		ctrl.Stop_Friction();
		ctrl.Start_Supply_Test();
		break;

	default:
		// 其他在线模式保持 Pitch，只有遥控器离线时才失能。
		ctrl.Hold_Pantile();
		ctrl.Stop_Friction();
		ctrl.Stop_Supply();
		break;
	}
}

void RC::Decode()
{
//	if (queueHandler == NULL || *queueHandler == NULL) {
//		return;  // 或者报错
//	}
//	else {
//		pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
//	}
//
//	if (sizeof(m_frame) < 18) return;
//	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;
//
//	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
//	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
//	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
//	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
//	if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
//	if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
//	if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
//	if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;
//
//	pre_rc.s[0] = rc.s[0];
//	pre_rc.s[1] = rc.s[1];
//
//	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
//	rc.s[1] = ((m_frame[5] >> 4) & 0x03);
//
//	pc.x = m_frame[6] | (m_frame[7] << 8);
//	pc.y = m_frame[8] | (m_frame[9] << 8);
//	pc.z = m_frame[10] | (m_frame[11] << 8);
//	pc.press_l = m_frame[12];
//	pc.press_r = m_frame[13];
//
//	pc.key_h = m_frame[15];//按键的高位部分R F G Z X C 
//	pc.key_l = m_frame[14];//按键的低8位 W S A D SHIFT CTRL Q E
	if (queueHandler == nullptr || *queueHandler == nullptr)
	{
		return;
	}

	pd_Rx = xQueueReceive(*queueHandler, m_frame, 0);

	if (pd_Rx != pdTRUE)
	{
		return;
	}

	if ((m_frame[0] |
		m_frame[1] |
		m_frame[2] |
		m_frame[3] |
		m_frame[4] |
		m_frame[5]) == 0)
	{
		return;
	}

	int16_t new_ch[4];

	new_ch[0] =
		((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;

	new_ch[1] =
		((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;

	new_ch[2] =
		((m_frame[2] >> 6 |
		  m_frame[3] << 2 |
		  m_frame[4] << 10) & 0x07FF) - 1024;

	new_ch[3] =
		((m_frame[4] >> 1 |
		  m_frame[5] << 7) & 0x07FF) - 1024;

	const uint8_t new_s0 =
		((m_frame[5] >> 4) & 0x0C) >> 2;

	const uint8_t new_s1 =
		((m_frame[5] >> 4) & 0x03);

	// 异常通道或异常拨杆数据不进入控制层。
	for (int i = 0; i < 4; ++i)
	{
		if (new_ch[i] < -700 || new_ch[i] > 700)
		{
			return;
		}

		if (new_ch[i] >= -8 && new_ch[i] <= 8)
		{
			new_ch[i] = 0;
		}
	}

	if (new_s0 < UP || new_s0 > MID ||
		new_s1 < UP || new_s1 > MID)
	{
		return;
	}

	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

	for (int i = 0; i < 4; ++i)
	{
		rc.ch[i] = new_ch[i];
	}

	rc.s[0] = new_s0;
	rc.s[1] = new_s1;

	pc.x = m_frame[6] | (m_frame[7] << 8);
	pc.y = m_frame[8] | (m_frame[9] << 8);
	pc.z = m_frame[10] | (m_frame[11] << 8);
	pc.press_l = m_frame[12];
	pc.press_r = m_frame[13];
	pc.key_l = m_frame[14];
	pc.key_h = m_frame[15];

	last_valid_frame_tick_ = HAL_GetTick();
	has_valid_frame_ = true;

}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}
