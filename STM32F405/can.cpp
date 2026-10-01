#include "can.h"
#include "label.h"
#include "string.h"
#include "HTmotor.h"

/*
* @brief		CAN通信初始化函数
* @param		CAN通道基地址
*/
void CAN::Init(CAN_TypeDef* instance)
{
	hcan.Instance = instance;				//can基地址赋值
	hcan.Init.Prescaler = 6;				//分频系数（不减一）
	hcan.Init.Mode = CAN_MODE_NORMAL;		//普通模式
	hcan.Init.SJW = CAN_SJW_1TQ;			//同步段为1个字节
	hcan.Init.BS1 = CAN_BS1_2TQ;			//相位缓冲段1为2个字节
	hcan.Init.BS2 = CAN_BS2_4TQ;			//相位缓冲段2为4个字节
	hcan.Init.TTCM = DISABLE;				//非时间触发通信模式
	hcan.Init.ABOM = ENABLE;				//允许自动离线管理
	hcan.Init.AWUM = ENABLE;				//允许自动唤醒
	hcan.Init.NART = DISABLE;				//禁止报文自动重传，即数据只传一次
	hcan.Init.RFLM = DISABLE;				//禁止报文溢出锁定
	hcan.Init.TXFP = DISABLE;				//传输优先级，EANBLE:ID优先 DISABLE:报文优先
	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;

	HAL_CAN_Init(&hcan);
	InitFilter();
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
}

/*
 * @brief      CAN过滤器初始化
*/
void CAN::InitFilter()
{
	// CAN1 和 CAN2 使用相同的过滤器配置。
	CAN_FilterConfTypeDef		CAN_FilterConfigStructure;

	//can1(0-13)和can2(14-27)分别得到一半的filter
	if (hcan.Instance == CAN1)
	{
		CAN_FilterConfigStructure.FilterNumber = 0;								//选择过滤器0
	}
	else if (hcan.Instance == CAN2)
	{
		CAN_FilterConfigStructure.FilterNumber = 14;							//选择过滤器0
	}
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;				//掩码模式
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;				//32位宽
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;							//接收所有数据
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;			//过滤器关联到FIFO0
	CAN_FilterConfigStructure.FilterActivation = ENABLE;						//激活过滤器
	CAN_FilterConfigStructure.BankNumber = 14; //0-13为CAN1，14-27为CAN2

	HAL_CAN_ConfigFilter(&hcan, &CAN_FilterConfigStructure);

}

/*
 * @brief       CAN外设配置
 * @param       *hcan    : CAN句柄指针
*/
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if (hcan->Instance == CAN1)
	{
		/* Peripheral clock enable */
		__HAL_RCC_CAN1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**CAN1 GPIO Configuration
		PD0     ------> CAN1_RX
		PD1     ------> CAN1_TX*/
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(CAN1_TX_IRQn, 7, 0);
		HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	}
	else if (hcan->Instance == CAN2)
	{
		/* Peripheral clock enable */
		__HAL_RCC_CAN2_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**CAN1 GPIO Configuration
		PB12     ------> CAN2_RX
		PB13     ------> CAN2_TX*/
		GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(CAN2_TX_IRQn, 7, 0);
		HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
		HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	}
}


/*
 * @brief       CAN通信传输函数
 * @param       ID		:	CAN外设ID
 * @param		*pData	: 	传输数据数组
 * @param		len		:	传输数据长度
*/
HAL_StatusTypeDef CAN::Transmit(const uint32_t ID, const uint8_t* const pData, const uint8_t len)
{
	hcan.pTxMsg->StdId = ID;					//设置标识符
	hcan.pTxMsg->IDE = CAN_ID_STD;				//标准帧(无拓展)
	hcan.pTxMsg->RTR = CAN_RTR_DATA;			//数据帧
	hcan.pTxMsg->DLC = len;						//设置长度
	memcpy(hcan.pTxMsg->Data, pData, len);
	HAL_StatusTypeDef status = HAL_CAN_Transmit(&hcan, 10);

	//ready = status != HAL_ERROR;// == HAL_OK;
	return status;
}

/**
 * @brief CAN 接收完成回调
 *
 * 这里只负责根据“CAN 总线 + 报文 ID”分配接收数据。
 * 不在中断中执行 PID、控制计算或发送操作。
 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	// 当前收到的 CAN 标准帧 ID
	const uint32_t std_id = hcan->pRxMsg->StdId;

	// 当前收到的 8 字节数据
	const uint8_t* rx_data = hcan->pRxMsg->Data;

	if (hcan == &can1.hcan)
	{
		if (std_id == DM_motorPitch.GetFeedbackID())//达妙Pitch
		{
			DM_motorPitch.OnCanFrame(rx_data,HAL_GetTick());
		}
		else if (std_id >= 0x201 && std_id <= 0x20C)//普通大疆电机
		{
			const uint32_t motor_index = std_id - 0x201;
			memcpy(can1.data[motor_index],rx_data,sizeof(uint8_t) * 8);
			can1.motor_last_feedback_tick[motor_index] = HAL_GetTick();
			if (can1.motor_feedback_count[motor_index] < 0xFFFFFFFFU)
			{
				++can1.motor_feedback_count[motor_index];
			}
		}
	}
	else if (hcan == &can2.hcan)
	{
	
		if (std_id ==DM_motorYaw.GetFeedbackID())//达妙大Yaw
		{
			DM_motorYaw.OnCanFrame(rx_data,HAL_GetTick());
		}
		else if (std_id >= 0x201 && std_id <= 0x20C)//普通大疆电机
		{
			const uint32_t motor_index = std_id - 0x201;
			memcpy(can2.data[motor_index],rx_data,sizeof(uint8_t) * 8);
			can2.motor_last_feedback_tick[motor_index] = HAL_GetTick();
			if (can2.motor_feedback_count[motor_index] < 0xFFFFFFFFU)
			{
				++can2.motor_feedback_count[motor_index];
			}
		}
	}

	 //重新开启FIFO0接收中断
	__HAL_CAN_ENABLE_IT(hcan,CAN_IT_FMP0
	);
}

extern "C" void CAN1_TX_IRQHandler()
{
	HAL_CAN_IRQHandler(&can1.hcan);
}
extern "C" void CAN1_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&can1.hcan);
}
extern "C" void CAN2_TX_IRQHandler()
{
	HAL_CAN_IRQHandler(&can2.hcan);
}
extern "C" void CAN2_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&can2.hcan);
}
