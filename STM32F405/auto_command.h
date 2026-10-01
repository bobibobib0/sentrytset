#pragma once
#include <stdint.h>


struct AutoCommand //上位机原始数据和底层机器人控制之间的标准接口
{
	bool enable=false;          // 是否启用自动控制（视觉模式）
	bool target_valid=false;    // 是否识别到有效目标
	bool fire_request=false;    // 上位机是否请求发射
	bool online = false;          // 视觉通信是否在线
	
	uint8_t reference_frame = 0;  // 0：世界坐标系
	uint8_t axis_policy = 0;      // 0：小Y和P

	float vx;             // 底盘前后速度
	float vy;             // 底盘左右速度
	float wz;             // 底盘旋转速度

	float yaw=0.0f;            // 云台Yaw目标或偏差
	float pitch=0.0f;          // 云台Pitch目标或偏差
	float distance=0.0f;       // 目标距离

	uint32_t sequence=0;    // AIM帧序号
	uint32_t frame_epoch = 0;     // 坐标系连续性编号
	
	uint64_t produced_time_us = 0;
	uint64_t valid_until_us = 0;
	
	uint32_t last_rx_tick=0;// 最后一次收到有效帧的时间
	
};
