#include "imu.h"
#include "label.h"
void IMU::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type)
{
	huart->Init(Instance, BaudRate).DMARxInit();
	m_uart = huart;
	this->type = type;
	queueHandler = &huart->UartQueueHandler;
}

void IMU::Decode()
{
	//	if (queueHandler == NULL || *queueHandler == NULL) {
	//		return;  // 或者报错
	//	}
	//	else {
	//		pd_Rx = xQueueReceive(*queueHandler, rxData, NULL);
	//	}
	if (xQueueReceive(*queueHandler, rxData, 0) != pdTRUE)
	{
		return;
	}

	if (type == IMU601)
	{
		if (rxData[0] == 0x55 && rxData[1] == 0x55)
		{
			uint8_t* data = rxData + 4;
			if (rxData[2] == 0x01 && Check(rxData, rxData[3] + 4, rxData[rxData[3] + 4]))
			{
				angle.roll = (float)getword(data[1], data[0]) * 180.f / 32768.f;
				angle.pitch = (float)getword(data[3], data[2]) * 180.f / 32768.f;
				angle.yaw = (float)getword(data[5], data[4]) * 180.f / 32768.f;
			}
			else if (rxData[2] == 0x03 && Check(rxData, rxData[3] + 4, rxData[rxData[3] + 4]))
			{
				uint16_t Ax, Ay, Az, Gx, Gy, Gz;
				Ax = getword(data[1], data[0]);
				Ay = getword(data[3], data[2]);
				Az = getword(data[5], data[4]);
				Gx = getword(data[7], data[6]);
				Gy = getword(data[9], data[8]);
				Gz = getword(data[11], data[10]);
				acceleration.x = (float)Ax / 32768 * ACC_FSR;
				acceleration.y = (float)Ay / 32768 * ACC_FSR;
				acceleration.z = (float)Az / 32768 * ACC_FSR;
				angularvelocity.roll = (float)Gx / 32768 * GYRO_FSR;
				angularvelocity.pitch = (float)Gy / 32768 * GYRO_FSR;
				angularvelocity.yaw = (float)Gz / 32768 * GYRO_FSR;

			}
		}
	}
	else if (type == CH010)
	{
		if (rxData[0] == 0x5A && rxData[1] == 0xA5)
		{
			const uint16_t payload_len =(static_cast<uint16_t>(rxData[3]) << 8) | rxData[2];
			const uint16_t received_crc =(static_cast<uint16_t>(rxData[5]) << 8) | rxData[4];

			// HI91负载固定为76字节，四元数位于负载60~75字节。
			if (payload_len != 76U ||
				payload_len > UART_MAX_LEN - 6U ||
				m_uart->dataDmaNum < payload_len + 6U)
			{
				return;
			}

			if (Check(rxData, payload_len + 6U, received_crc))
			{
				if (rxData[6] == 0x91)
				{
					const int offset = 6;

					acceleration.x = R4(rxData + offset + 12);
					acceleration.y = R4(rxData + offset + 16);
					acceleration.z = R4(rxData + offset + 20);
					
					angularvelocity.pitch = R4(rxData + offset + 24);
					angularvelocity.roll = R4(rxData + offset + 28);
					angularvelocity.yaw = R4(rxData + offset + 32);
				
					angle.roll = R4(rxData + offset + 48);
					angle.pitch = R4(rxData + offset + 52);
					angle.yaw = R4(rxData + offset + 56);

					quaternion.w = R4(rxData + offset + 60);
					quaternion.x = R4(rxData + offset + 64);
					quaternion.y = R4(rxData + offset + 68);
					quaternion.z = R4(rxData + offset + 72);

					device_time_ms_ = U4(rxData + offset + 8);
					++source_counter_;
					last_sample_tick_ = HAL_GetTick();
					sample_valid_ = true;
				}
			}
		}
	}
	else if (type == HI226)
	{
		if (rxData[0] == 0x5A && rxData[1] == 0xA5)
		{
			const uint16_t payload_len =(static_cast<uint16_t>(rxData[3]) << 8) | rxData[2];

			const uint16_t received_crc =(static_cast<uint16_t>(rxData[5]) << 8) | rxData[4];

			// HI91负载固定为76字节，四元数位于负载60~75字节。
			if (payload_len != 76U ||
				payload_len > UART_MAX_LEN - 6U ||
				m_uart->dataDmaNum < payload_len + 6U)
			{
				return;
			}

			if (Check(rxData, payload_len + 6U, received_crc))
			{
				if (rxData[6] == 0x91)
				{
					const int offset = 6;

					acceleration.x = R4(rxData + offset + 12);
					acceleration.y = R4(rxData + offset + 16);
					acceleration.z = R4(rxData + offset + 20);

					angularvelocity.pitch = R4(rxData + offset + 24);
					angularvelocity.roll = R4(rxData + offset + 28);
					angularvelocity.yaw = R4(rxData + offset + 32);

					angle.roll = R4(rxData + offset + 48);
					angle.pitch = R4(rxData + offset + 52);
					angle.yaw = R4(rxData + offset + 56);

					quaternion.w = R4(rxData + offset + 60);
					quaternion.x = R4(rxData + offset + 64);
					quaternion.y = R4(rxData + offset + 68);
					quaternion.z = R4(rxData + offset + 72);

					device_time_ms_ = U4(rxData + offset + 8);
					++source_counter_;
					last_sample_tick_ = HAL_GetTick();
					sample_valid_ = true;
				}
			}
		}
	}
}

float IMU::GetAngleYaw()
{
	return angle.yaw;
}

float IMU::getangularvelocitypitch()
{
	return angularvelocity.pitch;
}

float IMU::GetAnglePitch()
{
	return angle.pitch;
}

float IMU::GetAngleRoll()
{
	return angle.roll;
}

//float* IMU::GetAcceleration()
//{
//	float temp[3]{};
//	temp[0] = acceleration.x;
//	temp[1] = acceleration.y;
//	temp[2] = acceleration.z;
//	return temp;
//}
IMU::Acceleration IMU::GetAcceleration() const
{
	return acceleration;
}

IMU::AngularVelocity IMU::GetAngularVelocity() const
{
	return angularvelocity;
}

IMU::Quaternion IMU::GetQuaternion() const
{
	return quaternion;
}

uint32_t IMU::GetSourceCounter() const
{
	return source_counter_;
}

uint32_t IMU::GetLastSampleTick() const
{
	return last_sample_tick_;
}

uint32_t IMU::GetDeviceTimeMs() const
{
	return device_time_ms_;
}

bool IMU::HasValidSample() const
{
	return sample_valid_;
}
bool IMU::Check(uint8_t* pdata, uint16_t len, uint16_t com)//com是报文里的校验码，len是报文长度，pdata是报文数据
{
	if (type == IMU601)
	{
		uint8_t t = 0;
		for (int i = 0; i < len; i++)
		{
			t += pdata[i];
		}
		return t == com;
	}
	else if (type == CH010 || type == HI226)
	{
		// pdata是完整帧，len包含6字节头。CRC覆盖SOF、长度和负载，跳过CRC字段本身。
		if (len < 6U)
		{
			return false;
		}

		crc = 0;
		for (uint16_t j = 0; j < len; ++j)
		{
			if (j == 4U || j == 5U)
			{
				continue;
			}

			uint32_t i;
			uint32_t byte = pdata[j];
			crc ^= byte << 8;
			for (i = 0; i < 8; ++i)
			{
				uint32_t temp = crc << 1;
				if (crc & 0x8000)
				{
					temp ^= 0x1021;
				}
				crc = temp;
			}
		}
		return crc == com;
	}
	return false;
}


int16_t IMU::getword(uint8_t HighBit, uint8_t LowBits)
{
	return HighBit << 8 | LowBits;
}
