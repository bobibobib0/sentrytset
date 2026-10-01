#include "xuc.h"

#include "RC.h"
#include "control.h"
#include "HTmotor.h"
#include "imu.h"
#include <math.h>
#include <string.h>

namespace
{
uint16_t ReadU16LE(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t ReadU32LE(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

uint64_t ReadU64LE(const uint8_t* p)
{
    uint64_t value = 0;
    for (uint8_t i = 0; i < 8; ++i)
        value |= static_cast<uint64_t>(p[i]) << (8U * i);
    return value;
}

float ReadF32LE(const uint8_t* p)
{
    const uint32_t raw = ReadU32LE(p);
    float value = 0.0f;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

void WriteU16LE(uint8_t* p, uint16_t value)
{
    p[0] = static_cast<uint8_t>(value);
    p[1] = static_cast<uint8_t>(value >> 8);
}

void WriteU32LE(uint8_t* p, uint32_t value)
{
    p[0] = static_cast<uint8_t>(value);
    p[1] = static_cast<uint8_t>(value >> 8);
    p[2] = static_cast<uint8_t>(value >> 16);
    p[3] = static_cast<uint8_t>(value >> 24);
}

void WriteU64LE(uint8_t* p, uint64_t value)
{
    for (uint8_t i = 0; i < 8; ++i)
        p[i] = static_cast<uint8_t>(value >> (8U * i));
}

void WriteF32LE(uint8_t* p, float value)
{
    uint32_t raw = 0;
    memcpy(&raw, &value, sizeof(raw));
    WriteU32LE(p, raw);
}

int64_t TimeDifferenceUs(uint64_t left, uint64_t right)
{
    if (left >= right)
    {
        const uint64_t delta = left - right;
        return delta > 0x7FFFFFFFFFFFFFFFULL
                   ? INT64_MAX
                   : static_cast<int64_t>(delta);
    }

    const uint64_t delta = right - left;
    return delta > 0x7FFFFFFFFFFFFFFFULL ? INT64_MIN : -static_cast<int64_t>(delta);
}
}

void XUC::Init(UART* huart, USART_TypeDef* instance, uint32_t baud_rate)
{
    huart->Init(instance, baud_rate).DMARxInit(nullptr).DMATxInit();
    uart_ = huart;

    boot_id_ = GenerateBootId();
    session_id_ = 0;
    session_counter_ = 0;
    tx_seq_ = 0;
    session_active_ = false;
    remote_mode_ = MODE_SAFE;
    command = AutoCommand{};

    // 协议规定的CRC自检，不通过时拒绝建立会话。
    static const uint8_t test_data[] = "123456789";
    if (RmvCrc16(test_data, 9) != 0x29B1U)
        boot_id_ = 0;
}

void XUC::Decode()
{
    receive_result_ = xQueueReceive( uart_->UartQueueHandler,dma_frame_,  0);

    if (receive_result_ != pdTRUE)
        return;

    const uint16_t received_len = static_cast<uint16_t>(uart_->dataDmaNum);

    if (received_len == 0 || received_len > UART_MAX_LEN)
    {
        ++diagnostics_.rx_format_errors;
        return;
    }

    AppendRxData(dma_frame_, received_len);
    ParseRxStream();
}

void XUC::Encode()
{
    // 周期发送由Update统一调度，避免保留旧0x5A协议。
}

void XUC::Update()
{
    const uint32_t now_ms = HAL_GetTick();
    const uint64_t now_us = NowUs();

    if (candidate_waiting_ &&
        now_ms - candidate_start_tick_ > CANDIDATE_TIMEOUT_MS)
    {
        DropRxData(1);
        candidate_waiting_ = false;
        ++diagnostics_.rx_format_errors;
        ParseRxStream();
    }

    if (session_active_ &&
        now_ms - last_heartbeat_tick_ > HEARTBEAT_TIMEOUT_MS)
    {
        ClearRemoteControl();
        session_active_ = false;
        session_id_ = 0;
        have_last_hello_ = false;
        host_state_ = 0;
    }

    if (remote_mode_ == MODE_VISION && !LocalVisionPermit())
    {
        constraint_flags_ |= (1U << 4);  // 遥控接管/许可撤销
        ClearRemoteControl();
    }

    if (command.target_valid &&
        TimeDifferenceUs(now_us, command.valid_until_us) >= 0)
    {
        command.target_valid = false;
        constraint_flags_ |= (1U << 3);  // 目标过期
        last_aim_result_ = RESULT_STALE;
        ++diagnostics_.expired_commands;
    }

    if (!session_active_)
        return;

    // 临时联调用固定频率发送；完成PC端订阅验证后再改为SUBSCRIBE调度。
    static uint32_t last_pose_test_tick = 0;
    static uint32_t last_joint_test_tick = 0;

    if (now_ms - last_pose_test_tick >= 20U)
    {
        last_pose_test_tick = now_ms;
        SendPose();
    }

    if (now_ms - last_joint_test_tick >= 50U)
    {
        last_joint_test_tick = now_ms;
        SendJointState();
    }

    const uint32_t status_period = 1000U / status_rate_hz_;
    const uint32_t diagnostics_period = 1000U / diagnostics_rate_hz_;
    const uint32_t service_period = 1000U / service_state_rate_hz_;

    if (now_ms - last_status_tick_ >= status_period)
    {
        last_status_tick_ = now_ms;
        SendStatus();
    }

    if (now_ms - last_diagnostics_tick_ >= diagnostics_period)
    {
        last_diagnostics_tick_ = now_ms;
        SendDiagnostics();
    }

    if (now_ms - last_service_state_tick_ >= service_period)
    {
        last_service_state_tick_ = now_ms;
        SendServiceState();
    }
}

void XUC::AppendRxData(const uint8_t* data, uint16_t len)
{
    if (data == nullptr || len == 0)
        return;

    if (len > RMV_RX_STREAM_SIZE)
    {
        data += len - RMV_RX_STREAM_SIZE;
        len = RMV_RX_STREAM_SIZE;
        rx_stream_size_ = 0;
    }

    if (rx_stream_size_ + len > RMV_RX_STREAM_SIZE)
    {
        rx_stream_size_ = 0;
        candidate_waiting_ = false;
        ++diagnostics_.rx_format_errors;
    }

    memcpy(rx_stream_ + rx_stream_size_, data, len);
    rx_stream_size_ += len;
}

void XUC::DropRxData(uint16_t len)
{
    if (len >= rx_stream_size_)
    {
        rx_stream_size_ = 0;
        return;
    }

    memmove(rx_stream_, rx_stream_ + len, rx_stream_size_ - len);
    rx_stream_size_ -= len;
}

void XUC::ParseRxStream()//用于解析接收到的串口数据流，寻找有效的RMV协议帧，并处理这些帧
{
    static const uint8_t magic[4] = {0x52, 0x4D, 0x56, 0x31};

    while (rx_stream_size_ >= 4)
    {
        uint16_t magic_pos = rx_stream_size_;
        for (uint16_t i = 0; i + 4 <= rx_stream_size_; ++i)
        {
            if (memcmp(rx_stream_ + i, magic, sizeof(magic)) == 0)
            {
                magic_pos = i;
                break;
            }
        }

        if (magic_pos == rx_stream_size_)
        {
            if (rx_stream_size_ > 3)
            {
                memmove(rx_stream_, rx_stream_ + rx_stream_size_ - 3, 3);
                rx_stream_size_ = 3;
            }
            candidate_waiting_ = false;
            return;
        }

        if (magic_pos > 0)
        {
            DropRxData(magic_pos);
            candidate_waiting_ = false;
        }

        if (rx_stream_size_ < RMV_HEADER_SIZE)
        {
            if (!candidate_waiting_)
            {
                candidate_waiting_ = true;
                candidate_start_tick_ = HAL_GetTick();
            }
            return;
        }

        const uint8_t major = rx_stream_[4];
        const uint8_t minor = rx_stream_[5];
        const uint16_t payload_len = ReadU16LE(rx_stream_ + 8);
        const uint16_t flags = ReadU16LE(rx_stream_ + 10);

        if (major != RMV_MAJOR ||
            minor != RMV_MINOR ||
            payload_len > RMV_MAX_PAYLOAD ||
            (flags & 0xFFFCU) != 0)
        {
            DropRxData(1);
            candidate_waiting_ = false;
            ++diagnostics_.rx_format_errors;
            continue;
        }

        const uint16_t frame_len = RMV_HEADER_SIZE + payload_len + RMV_CRC_SIZE;

        if (rx_stream_size_ < frame_len)
        {
            if (!candidate_waiting_)
            {
                candidate_waiting_ = true;
                candidate_start_tick_ = HAL_GetTick();
            }
            return;
        }

        const uint16_t received_crc =  ReadU16LE(rx_stream_ + RMV_HEADER_SIZE + payload_len);
        const uint16_t calculated_crc = RmvCrc16(rx_stream_, RMV_HEADER_SIZE + payload_len);

        if (received_crc != calculated_crc)
        {
            DropRxData(1);
            candidate_waiting_ = false;
            ++diagnostics_.rx_crc_errors;
            continue;
        }

        candidate_waiting_ = false;
        HandleFrame(rx_stream_, frame_len, NowUs());
        DropRxData(frame_len);
    }
}

void XUC::HandleFrame(const uint8_t* frame,  uint16_t frame_len,  uint64_t receive_time_us)
{
    if (frame == nullptr || frame_len < RMV_HEADER_SIZE + RMV_CRC_SIZE)
        return;

    const uint16_t msg_type = ReadU16LE(frame + 6);
    const uint16_t payload_len = ReadU16LE(frame + 8);
    const uint16_t flags = ReadU16LE(frame + 10);
    const uint32_t seq = ReadU32LE(frame + 12);
    const uint32_t frame_session_id = ReadU32LE(frame + 16);
    const uint8_t* payload = frame + RMV_HEADER_SIZE;

    if (msg_type == MSG_HELLO)
    {
        HandleHello(payload,
                    payload_len,
                    seq,
                    frame_session_id,
                    flags);
        return;
    }

    if (!session_active_ || frame_session_id != session_id_)
    {
        ++diagnostics_.rx_bad_session;
        if ((flags & FLAG_ACK_REQUEST) != 0)
            SendAck(seq, msg_type, RESULT_BAD_SESSION, 0, 0);
        return;
    }

    if ((flags & FLAG_RESPONSE) != 0)
    {
        ++diagnostics_.rx_format_errors;
        return;
    }

    switch (msg_type)
    {
    case MSG_TIME_SYNC:
        HandleTimeSync(payload, payload_len, receive_time_us);
        break;

    case MSG_HEARTBEAT:
        HandleHeartbeat(payload, payload_len);
        break;

    case MSG_SUBSCRIBE:
        HandleSubscribe(payload, payload_len, seq, flags);
        break;

    case MSG_MODE_REQUEST:
        HandleModeRequest(payload, payload_len, seq, flags);
        break;

    case MSG_AIM_SETPOINT:
        HandleAimSetpoint(payload, payload_len, seq, flags);
        break;

    case MSG_STOP_REMOTE:
        HandleStopRemote(payload, payload_len, seq, flags);
        break;

    case MSG_SERVICE_REQUEST:
        SendAck(seq, msg_type, RESULT_UNSUPPORTED, 0, session_id_);
        break;

    case MSG_PARAM_LIST_REQ:
    case MSG_PARAM_DESC_REQ:
    case MSG_PARAM_STAGE:
    case MSG_PARAM_COMMIT:
    case MSG_PARAM_ABORT:
        SendAck(seq, msg_type, RESULT_UNSUPPORTED, 0, session_id_);
        break;

    default:
        if ((flags & FLAG_ACK_REQUEST) != 0)
            SendAck(seq, msg_type, RESULT_UNSUPPORTED, 0, session_id_);
        break;
    }
}

uint16_t XUC::RmvCrc16(const uint8_t* data, uint16_t len) const
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x8000U) != 0
                      ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                      : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

uint64_t XUC::NowUs() const
{
    // 当前工程没有1MHz自由运行计时器，先保持1ms分辨率。
    return static_cast<uint64_t>(HAL_GetTick()) * 1000ULL;
}

uint64_t XUC::GenerateBootId()
{
    RNG_HandleTypeDef rng{};
    rng.Instance = RNG;
    uint32_t high = 0;
    uint32_t low = 0;

    __HAL_RCC_RNG_CLK_ENABLE();
    const bool ok =
        HAL_RNG_Init(&rng) == HAL_OK &&
        HAL_RNG_GenerateRandomNumber(&rng, &high) == HAL_OK &&
        HAL_RNG_GenerateRandomNumber(&rng, &low) == HAL_OK;
    HAL_RNG_DeInit(&rng);
    __HAL_RCC_RNG_CLK_DISABLE();

    if (!ok)
        return 0;  // 不能保证每次上电变化时，按协议拒绝建立会话。

    const uint64_t value = (static_cast<uint64_t>(high) << 32) | low;
    return value == 0 ? 1 : value;
}

bool XUC::SendFrame(uint16_t msg_type,
                    uint16_t flags,
                    uint32_t output_session_id,
                    const uint8_t* payload,
                    uint16_t payload_len)
{
    if (uart_ == nullptr ||
        payload_len > RMV_MAX_PAYLOAD ||
        (payload_len > 0 && payload == nullptr) ||
        (flags & 0xFFFCU) != 0)
    {
        ++diagnostics_.tx_dropped_frames;
        return false;
    }

    const uint16_t frame_len =
        RMV_HEADER_SIZE + payload_len + RMV_CRC_SIZE;
    if (frame_len > sizeof(tx_frame_))
    {
        ++diagnostics_.tx_dropped_frames;
        return false;
    }

    memset(tx_frame_, 0, frame_len);
    tx_frame_[0] = 0x52;
    tx_frame_[1] = 0x4D;
    tx_frame_[2] = 0x56;
    tx_frame_[3] = 0x31;
    tx_frame_[4] = RMV_MAJOR;
    tx_frame_[5] = RMV_MINOR;

    WriteU16LE(tx_frame_ + 6, msg_type);
    WriteU16LE(tx_frame_ + 8, payload_len);
    WriteU16LE(tx_frame_ + 10, flags);

    ++tx_seq_;
    if (tx_seq_ == 0)
        ++tx_seq_;

    WriteU32LE(tx_frame_ + 12, tx_seq_);
    WriteU32LE(tx_frame_ + 16, output_session_id);
    WriteU64LE(tx_frame_ + 20, NowUs());

    if (payload_len > 0)
        memcpy(tx_frame_ + RMV_HEADER_SIZE, payload, payload_len);

    const uint16_t crc =
        RmvCrc16(tx_frame_, RMV_HEADER_SIZE + payload_len);
    WriteU16LE(tx_frame_ + RMV_HEADER_SIZE + payload_len, crc);

    // 当前UART封装只提供可靠的阻塞发送；周期带宽保持在低水平。
    uart_->UARTTransmit(tx_frame_, frame_len);
    return true;
}

void XUC::SendAck(uint32_t request_seq,
                  uint16_t request_msg_type,
                  uint16_t result,
                  uint32_t detail,
                  uint32_t output_session_id)
{
    uint8_t payload[16]{};
    WriteU32LE(payload + 0, request_seq);
    WriteU16LE(payload + 4, request_msg_type);
    WriteU16LE(payload + 6, result);
    WriteU32LE(payload + 8, detail);
    WriteU32LE(payload + 12, PARAM_REV);
    SendFrame(MSG_ACK,
              FLAG_RESPONSE,
              output_session_id,
              payload,
              sizeof(payload));
}

void XUC::HandleHello(const uint8_t* payload,
                      uint16_t payload_len,
                      uint32_t request_seq,
                      uint32_t request_session_id,
                      uint16_t request_flags)
{
    if (boot_id_ == 0)
    {
        SendAck(request_seq, MSG_HELLO, RESULT_HARDWARE_FAULT, 0, 0);
        return;
    }

    if (payload_len != 16)
    {
        SendAck(request_seq, MSG_HELLO, RESULT_BAD_LENGTH, 0, 0);
        return;
    }

    if (request_session_id != 0)
    {
        SendAck(request_seq, MSG_HELLO, RESULT_BAD_SESSION, 0, 0);
        return;
    }

    if ((request_flags & FLAG_RESPONSE) != 0)
    {
        SendAck(request_seq, MSG_HELLO, RESULT_BAD_VALUE, 0, 0);
        return;
    }

    const uint64_t nonce = ReadU64LE(payload + 0);
    const uint32_t required_caps = ReadU32LE(payload + 8);
    const uint8_t min_major = payload[12];
    const uint8_t max_major = payload[13];
    const uint16_t reserved = ReadU16LE(payload + 14);

    if (nonce == 0 || reserved != 0)
    {
        SendAck(request_seq, MSG_HELLO, RESULT_BAD_VALUE, 0, 0);
        return;
    }

    if (min_major > RMV_MAJOR ||
        max_major < RMV_MAJOR ||
        min_major > max_major)
    {
        SendAck(request_seq, MSG_HELLO, RESULT_BAD_VERSION, 0, 0);
        return;
    }

    const uint32_t unsupported = required_caps & ~SUPPORTED_CAPS;
    if (unsupported != 0)
    {
        SendAck(request_seq,
                MSG_HELLO,
                RESULT_UNSUPPORTED,
                unsupported,
                0);
        return;
    }

    if (session_active_)
    {
        if (have_last_hello_ &&
            memcmp(payload, last_hello_payload_, 16) == 0)
        {
            SendHelloReply(last_hello_nonce_);
            return;
        }

        SendAck(request_seq,
                MSG_HELLO,
                nonce == last_hello_nonce_ ? RESULT_CONFLICT : RESULT_BUSY,
                0,
                0);
        return;
    }

    ++session_counter_;
    if (session_counter_ == 0)
        ++session_counter_;

    session_id_ = session_counter_;
    session_active_ = true;
    last_heartbeat_tick_ = HAL_GetTick();
    host_state_ = 0;
    remote_mode_ = MODE_SAFE;
    last_hello_nonce_ = nonce;
    have_last_hello_ = true;
    memcpy(last_hello_payload_, payload, sizeof(last_hello_payload_));

    command = AutoCommand{};
    have_accepted_aim_seq_ = false;
    have_operation_ = false;
    constraint_flags_ = 0;
    status_rate_hz_ = 20;
    diagnostics_rate_hz_ = 1;
    service_state_rate_hz_ = 1;

    SendHelloReply(nonce);
}

void XUC::SendHelloReply(uint64_t nonce)
{
    uint8_t payload[36]{};
    WriteU64LE(payload + 0, nonce);
    WriteU64LE(payload + 8, boot_id_);
    WriteU32LE(payload + 16, FW_VERSION);
    WriteU32LE(payload + 20, SUPPORTED_CAPS);
    WriteU32LE(payload + 24, PARAM_REV);
    WriteU32LE(payload + 28, GEOMETRY_REV);
    WriteU16LE(payload + 32, RMV_MAX_PAYLOAD);
    payload[34] = 2;  // 实物IMU数量，不代表开放原始IMU消息
    payload[35] = 3;  // 小Y、P、大Y

    SendFrame(MSG_HELLO_REPLY,
              FLAG_RESPONSE,
              session_id_,
              payload,
              sizeof(payload));
}

void XUC::HandleTimeSync(const uint8_t* payload,
                         uint16_t payload_len,
                         uint64_t receive_time_us)
{
    if (payload_len != 8)
        return;

    uint8_t reply[24]{};
    WriteU64LE(reply + 0, ReadU64LE(payload));
    WriteU64LE(reply + 8, receive_time_us);
    WriteU64LE(reply + 16, NowUs());
    SendFrame(MSG_TIME_REPLY,
              FLAG_RESPONSE,
              session_id_,
              reply,
              sizeof(reply));
}

void XUC::HandleHeartbeat(const uint8_t* payload, uint16_t payload_len)
{
    if (payload_len != 4 ||
        payload[0] > 2 ||
        payload[1] != 0 ||
        payload[2] != 0 ||
        payload[3] != 0)
    {
        ++diagnostics_.rx_format_errors;
        return;
    }

    host_state_ = payload[0];
    last_heartbeat_tick_ = HAL_GetTick();
    command.online = host_state_ == 1;

    if (host_state_ != 1)
        command.target_valid = false;

    if (host_state_ == 2)
        ClearRemoteControl();
}

void XUC::HandleSubscribe(const uint8_t* payload,
                          uint16_t payload_len,
                          uint32_t seq,
                          uint16_t flags)
{
    if ((flags & FLAG_ACK_REQUEST) == 0)
    {
        SendAck(seq, MSG_SUBSCRIBE, RESULT_BAD_VALUE, 0, session_id_);
        return;
    }

    if (HandleRepeatedOperation(MSG_SUBSCRIBE, seq, payload, payload_len))
        return;

    if (payload_len < 4 || payload[0] < 1 || payload[0] > 16 ||
        payload[1] != 0 || payload[2] != 0 || payload[3] != 0 ||
        payload_len != static_cast<uint16_t>(4U + payload[0] * 8U))
    {
        RememberOperation(MSG_SUBSCRIBE, seq, payload, payload_len,
                          RESULT_BAD_LENGTH, 0);
        SendAck(seq, MSG_SUBSCRIBE, RESULT_BAD_LENGTH, 0, session_id_);
        return;
    }

    uint16_t new_status_rate = status_rate_hz_;
    uint16_t new_diagnostics_rate = diagnostics_rate_hz_;
    uint16_t new_service_rate = service_state_rate_hz_;

    for (uint8_t i = 0; i < payload[0]; ++i)
    {
        const uint8_t* item = payload + 4U + i * 8U;
        const uint16_t msg_type = ReadU16LE(item + 0);
        const uint16_t rate_hz = ReadU16LE(item + 2);
        const uint32_t source_mask = ReadU32LE(item + 4);

        if (source_mask != 0 || rate_hz > 200)
        {
            RememberOperation(MSG_SUBSCRIBE, seq, payload, payload_len,
                              RESULT_BAD_VALUE, msg_type);
            SendAck(seq, MSG_SUBSCRIBE, RESULT_BAD_VALUE,
                    msg_type, session_id_);
            return;
        }

        if (msg_type == MSG_STATUS)
        {
            if (rate_hz < 10)
            {
                RememberOperation(MSG_SUBSCRIBE, seq, payload, payload_len,
                                  RESULT_BAD_VALUE, msg_type);
                SendAck(seq, MSG_SUBSCRIBE, RESULT_BAD_VALUE,
                        msg_type, session_id_);
                return;
            }
            new_status_rate = rate_hz;
        }
        else if (msg_type == MSG_DIAGNOSTICS)
        {
            if (rate_hz < 1)
            {
                RememberOperation(MSG_SUBSCRIBE, seq, payload, payload_len,
                                  RESULT_BAD_VALUE, msg_type);
                SendAck(seq, MSG_SUBSCRIBE, RESULT_BAD_VALUE,
                        msg_type, session_id_);
                return;
            }
            new_diagnostics_rate = rate_hz;
        }
        else if (msg_type == MSG_SERVICE_STATE)
        {
            if (rate_hz < 1)
            {
                RememberOperation(MSG_SUBSCRIBE, seq, payload, payload_len,
                                  RESULT_BAD_VALUE, msg_type);
                SendAck(seq, MSG_SUBSCRIBE, RESULT_BAD_VALUE,
                        msg_type, session_id_);
                return;
            }
            new_service_rate = rate_hz;
        }
        else
        {
            RememberOperation(MSG_SUBSCRIBE, seq, payload, payload_len,
                              RESULT_UNSUPPORTED, msg_type);
            SendAck(seq, MSG_SUBSCRIBE, RESULT_UNSUPPORTED,
                    msg_type, session_id_);
            return;
        }
    }

    // 460800 8N1 每方向约46080 B/s，按协议将稳态占用限制在70%。
    const uint32_t bytes_per_second =
        static_cast<uint32_t>(new_status_rate) * (30U + 32U) +
        static_cast<uint32_t>(new_diagnostics_rate) * (30U + 40U) +
        static_cast<uint32_t>(new_service_rate) * (30U + 24U);
    if (bytes_per_second > 32256U)
    {
        const uint8_t* last_item = payload + 4U + (payload[0] - 1U) * 8U;
        const uint16_t detail = ReadU16LE(last_item);
        RememberOperation(MSG_SUBSCRIBE, seq, payload, payload_len,
                          RESULT_RESOURCE_LIMIT, detail);
        SendAck(seq, MSG_SUBSCRIBE, RESULT_RESOURCE_LIMIT,
                detail, session_id_);
        return;
    }

    status_rate_hz_ = new_status_rate;
    diagnostics_rate_hz_ = new_diagnostics_rate;
    service_state_rate_hz_ = new_service_rate;
    RememberOperation(MSG_SUBSCRIBE, seq, payload, payload_len, RESULT_OK, 0);
    SendAck(seq, MSG_SUBSCRIBE, RESULT_OK, 0, session_id_);
}

void XUC::HandleModeRequest(const uint8_t* payload,
                            uint16_t payload_len,
                            uint32_t seq,
                            uint16_t flags)
{
    if ((flags & FLAG_ACK_REQUEST) == 0)
    {
        SendAck(seq, MSG_MODE_REQUEST, RESULT_BAD_VALUE, 0, session_id_);
        return;
    }

    if (HandleRepeatedOperation(MSG_MODE_REQUEST, seq, payload, payload_len))
        return;

    if (payload_len != 8)
    {
        RememberOperation(MSG_MODE_REQUEST, seq, payload, payload_len,
                          RESULT_BAD_LENGTH, 0);
        SendAck(seq, MSG_MODE_REQUEST, RESULT_BAD_LENGTH, 0, session_id_);
        return;
    }

    const uint8_t requested_mode = payload[0];
    const uint8_t axis_mask = payload[1];
    const uint16_t reserved = ReadU16LE(payload + 2);
    const uint32_t frame_epoch = ReadU32LE(payload + 4);
    uint16_t result = RESULT_OK;

    if (reserved != 0 || (axis_mask & 0xF8U) != 0)
        result = RESULT_BAD_VALUE;
    else if (frame_epoch != FRAME_EPOCH)
        result = RESULT_CONFLICT;
    else if (requested_mode == MODE_SAFE)
        ClearRemoteControl();
    else if (requested_mode == MODE_VISION)
    {
        if ((axis_mask & 0x03U) != 0x03U)
            result = RESULT_NOT_READY;
        else if (!command.online || !LocalVisionPermit())
            result = RESULT_NOT_READY;
        else
        {
            // 当前工程没有独立回零状态接口，不能伪造“已回零”。
            result = RESULT_NOT_READY;
        }
    }
    else if (requested_mode == MODE_CALIBRATION)
        result = RESULT_UNSUPPORTED;
    else
        result = RESULT_BAD_VALUE;

    RememberOperation(MSG_MODE_REQUEST, seq, payload, payload_len, result, 0);
    SendAck(seq, MSG_MODE_REQUEST, result, 0, session_id_);
}

void XUC::HandleAimSetpoint(const uint8_t* payload,
                            uint16_t payload_len,
                            uint32_t seq,
                            uint16_t flags)
{
    last_received_aim_seq_ = seq;
    constraint_flags_ = 0;

    auto reject = [&](uint16_t result, uint16_t constraints)
    {
        last_aim_result_ = result;
        constraint_flags_ = constraints;
        command.target_valid = false;
        if ((flags & FLAG_ACK_REQUEST) != 0)
            SendAck(seq, MSG_AIM_SETPOINT, result, constraints, session_id_);
    };

    if (payload_len != 48)
    {
        reject(RESULT_BAD_LENGTH, 0);
        return;
    }

    if (!RemoteControlReady())
    {
        reject(RESULT_WRONG_MODE, 0);
        return;
    }

    if (have_accepted_aim_seq_ && !IsNewerSeq(seq, last_accepted_aim_seq_))
    {
        ++diagnostics_.duplicate_commands;
        reject(RESULT_STALE, 0);
        return;
    }

    const uint32_t frame_epoch = ReadU32LE(payload + 0);
    const uint64_t produced_us = ReadU64LE(payload + 4);
    const uint64_t valid_until_us = ReadU64LE(payload + 12);
    const uint8_t reference_frame = payload[20];
    const uint8_t axis_policy = payload[21];
    const uint16_t valid_fields = ReadU16LE(payload + 22);
    const float target_yaw = ReadF32LE(payload + 24);
    const float target_pitch = ReadF32LE(payload + 28);
    const float yaw_rate = ReadF32LE(payload + 32);
    const float pitch_rate = ReadF32LE(payload + 36);
    const float yaw_accel = ReadF32LE(payload + 40);
    const float pitch_accel = ReadF32LE(payload + 44);

    if (frame_epoch != FRAME_EPOCH)
    {
        reject(RESULT_CONFLICT, 1U << 8);
        return;
    }

    if (reference_frame != 0 || axis_policy != 0)
    {
        reject(RESULT_UNSUPPORTED, 0);
        return;
    }

    if ((valid_fields & 0xFFFEU) != 0)
    {
        reject(RESULT_UNSUPPORTED, 0);
        return;
    }

    if (!isfinite(target_yaw) || !isfinite(target_pitch) ||
        !isfinite(yaw_rate) || !isfinite(pitch_rate) ||
        !isfinite(yaw_accel) || !isfinite(pitch_accel) ||
        yaw_rate != 0.0f || pitch_rate != 0.0f ||
        yaw_accel != 0.0f || pitch_accel != 0.0f)
    {
        reject(RESULT_BAD_VALUE, 0);
        return;
    }

    if ((valid_fields & 0x0001U) == 0 &&
        (target_yaw != 0.0f || target_pitch != 0.0f))
    {
        reject(RESULT_BAD_VALUE, 0);
        return;
    }

    const uint64_t now_us = NowUs();
    const int64_t future_us = TimeDifferenceUs(produced_us, now_us);
    const int64_t age_us = TimeDifferenceUs(now_us, produced_us);
    const int64_t lifetime_us = TimeDifferenceUs(valid_until_us, produced_us);
    const int64_t remaining_us = TimeDifferenceUs(valid_until_us, now_us);

    if (future_us > 5000 || age_us > 50000 ||
        lifetime_us <= 0 || lifetime_us > 100000 || remaining_us <= 0)
    {
        ++diagnostics_.expired_commands;
        reject(RESULT_STALE, 1U << 3);
        return;
    }

    last_accepted_aim_seq_ = seq;
    have_accepted_aim_seq_ = true;

    if ((valid_fields & 0x0001U) == 0)
    {
        command.target_valid = false;
        command.sequence = seq;
        command.last_rx_tick = HAL_GetTick();
        last_aim_result_ = RESULT_OK;
        if ((flags & FLAG_ACK_REQUEST) != 0)
            SendAck(seq, MSG_AIM_SETPOINT, RESULT_OK, 0, session_id_);
        return;
    }

    command.reference_frame = reference_frame;
    command.axis_policy = axis_policy;
    command.frame_epoch = frame_epoch;
    command.yaw = target_yaw;
    command.pitch = target_pitch;
    command.produced_time_us = produced_us;
    command.valid_until_us = valid_until_us;
    command.sequence = seq;
    command.target_valid = true;
    command.last_rx_tick = HAL_GetTick();
    last_aim_result_ = RESULT_OK;

    if ((flags & FLAG_ACK_REQUEST) != 0)
        SendAck(seq, MSG_AIM_SETPOINT, RESULT_OK, 0, session_id_);
}

void XUC::HandleStopRemote(const uint8_t* payload,
                           uint16_t payload_len,
                           uint32_t seq,
                           uint16_t flags)
{
    if ((flags & FLAG_ACK_REQUEST) == 0)
    {
        SendAck(seq, MSG_STOP_REMOTE, RESULT_BAD_VALUE, 0, session_id_);
        return;
    }

    if (HandleRepeatedOperation(MSG_STOP_REMOTE, seq, payload, payload_len))
        return;

    if (payload_len != 4)
    {
        RememberOperation(MSG_STOP_REMOTE, seq, payload, payload_len,
                          RESULT_BAD_LENGTH, 0);
        SendAck(seq, MSG_STOP_REMOTE, RESULT_BAD_LENGTH, 0, session_id_);
        return;
    }

    if (ReadU16LE(payload + 2) != 0)
    {
        RememberOperation(MSG_STOP_REMOTE, seq, payload, payload_len,
                          RESULT_BAD_VALUE, 0);
        SendAck(seq, MSG_STOP_REMOTE, RESULT_BAD_VALUE, 0, session_id_);
        return;
    }

    ClearRemoteControl();
    RememberOperation(MSG_STOP_REMOTE, seq, payload, payload_len, RESULT_OK, 0);
    SendAck(seq, MSG_STOP_REMOTE, RESULT_OK, 0, session_id_);
}

bool XUC::IsNewerSeq(uint32_t value, uint32_t reference) const
{
    const uint32_t difference = value - reference;
    return difference != 0 && difference < 0x80000000U;
}

bool XUC::HandleRepeatedOperation(uint16_t msg_type,
                                  uint32_t seq,
                                  const uint8_t* payload,
                                  uint16_t payload_len)
{
    if (!have_operation_)
        return false;

    if (seq == last_operation_seq_)
    {
        ++diagnostics_.duplicate_commands;
        if (msg_type == last_operation_msg_ &&
            payload_len == last_operation_payload_len_ &&
            (payload_len == 0 ||
             memcmp(payload, last_operation_payload_, payload_len) == 0))
        {
            SendAck(seq,
                    msg_type,
                    last_operation_result_,
                    last_operation_detail_,
                    session_id_);
        }
        else
        {
            SendAck(seq, msg_type, RESULT_CONFLICT, 0, session_id_);
        }
        return true;
    }

    if (!IsNewerSeq(seq, last_operation_seq_))
    {
        ++diagnostics_.duplicate_commands;
        SendAck(seq, msg_type, RESULT_STALE, 0, session_id_);
        return true;
    }

    return false;
}

void XUC::RememberOperation(uint16_t msg_type,
                            uint32_t seq,
                            const uint8_t* payload,
                            uint16_t payload_len,
                            uint16_t result,
                            uint32_t detail)
{
    have_operation_ = true;
    last_operation_seq_ = seq;
    last_operation_msg_ = msg_type;
    last_operation_payload_len_ = payload_len;
    if (payload_len > 0)
        memcpy(last_operation_payload_, payload, payload_len);
    last_operation_result_ = result;
    last_operation_detail_ = detail;
}

bool XUC::LocalVisionPermit() const
{
    return rc.IsOnline() && ctrl.GetMode() == CONTROL::AUTOAIM;
}

bool XUC::RemoteControlReady() const
{
    return session_active_ && command.online &&  command.enable &&remote_mode_ == MODE_VISION && LocalVisionPermit();
}

void XUC::ClearRemoteControl()
{
    remote_mode_ = MODE_SAFE;
    command.enable = false;
    command.target_valid = false;
}

void XUC::SendStatus()
{
    uint8_t payload[32]{};
    const bool remote_online =  session_active_ && (HAL_GetTick() - last_heartbeat_tick_ <= HEARTBEAT_TIMEOUT_MS);
    const bool vision_permit = RemoteControlReady();

    payload[0] = vision_permit ? MODE_VISION
                               : (rc.IsOnline() ? MODE_MANUAL : MODE_SAFE);
    payload[1] = vision_permit ? 2 : (rc.IsOnline() ? 1 : 0);
    payload[2] = 0;  // 无可靠的逐轴在线状态，不能伪造
    payload[3] = 0;  // 无可靠的逐轴使能状态
    payload[4] = 0;  // 无独立回零状态接口
    payload[5] = remote_online ? 1 : 0;
    payload[6] = vision_permit ? 1 : 0;
    payload[7] = 0;  // 第一版不开放发射

    uint32_t fault_mask = 0;
    if (!rc.IsOnline())
        fault_mask |= 1U << 4;
    if (!remote_online)
        fault_mask |= 1U << 5;

    WriteU32LE(payload + 8, fault_mask);
    WriteU32LE(payload + 12, last_received_aim_seq_);
    WriteU32LE(payload + 16, last_applied_aim_seq_);
    WriteU16LE(payload + 20, last_aim_result_);
    WriteU16LE(payload + 22, constraint_flags_);
    WriteU32LE(payload + 24, PARAM_REV);
    WriteU32LE(payload + 28, GEOMETRY_REV);

    SendFrame(MSG_STATUS, 0, session_id_, payload, sizeof(payload));
}

void XUC::SendDiagnostics()
{
    uint8_t payload[40]{};
    const uint32_t values[10] = {
        diagnostics_.rx_crc_errors,
        diagnostics_.rx_format_errors,
        diagnostics_.rx_bad_session,
        diagnostics_.expired_commands,
        diagnostics_.duplicate_commands,
        diagnostics_.tx_dropped_frames,
        diagnostics_.tx_max_queue_delay_us,
        diagnostics_.control_period_us,
        diagnostics_.control_max_jitter_us,
        diagnostics_.control_overrun_count};

    for (uint8_t i = 0; i < 10; ++i)
        WriteU32LE(payload + i * 4U, values[i]);

    SendFrame(MSG_DIAGNOSTICS, 0, session_id_, payload, sizeof(payload));
}

void XUC::SendServiceState()
{
    uint8_t payload[24]{};
    WriteU64LE(payload + 8, NowUs());
    WriteU32LE(payload + 16, FRAME_EPOCH);
    SendFrame(MSG_SERVICE_STATE, 0, session_id_, payload, sizeof(payload));
}

void XUC::SendPose()
{
	constexpr float DEG_TO_RAD = 0.01745329252f;
	constexpr float G_TO_MPS2 = 9.80665f;
	constexpr uint32_t IMU_TIMEOUT_MS = 100U;

	auto send_imu = [&](uint8_t sensor_id,
	                    uint8_t frame_id,
	                    const IMU& imu)
	{
		if (!imu.HasValidSample())
			return;

		const uint32_t sample_tick = imu.GetLastSampleTick();
		if (HAL_GetTick() - sample_tick > IMU_TIMEOUT_MS)
			return;

		IMU::Quaternion q = imu.GetQuaternion();
		const IMU::AngularVelocity gyro = imu.GetAngularVelocity();
		const IMU::Acceleration accel = imu.GetAcceleration();

		// 归一化后再发送，避免异常四元数进入视觉姿态解算。
		const float norm = sqrtf(
			q.w * q.w + q.x * q.x +
			q.y * q.y + q.z * q.z);
		if (!isfinite(norm) || norm < 0.001f)
			return;

		q.w /= norm;
		q.x /= norm;
		q.y /= norm;
		q.z /= norm;

		uint8_t payload[68]{};
		payload[0] = sensor_id;
		payload[1] = frame_id;

		// 四元数、角速度、加速度有效；时间为MCU处理到该IMU样本的时间。
		WriteU16LE(payload + 2, 0x0027U);
		WriteU32LE(payload + 4, imu.GetSourceCounter());
		WriteU64LE(payload + 8,static_cast<uint64_t>(sample_tick) * 1000ULL);

		// IMU设备时钟尚未映射到MCU时钟，按协议填0并清有效位。
		WriteU64LE(payload + 16, 0);
		WriteU32LE(payload + 24, FRAME_EPOCH);

		WriteF32LE(payload + 28, q.w);
		WriteF32LE(payload + 32, q.x);
		WriteF32LE(payload + 36, q.y);
		WriteF32LE(payload + 40, q.z);

		// 当前CH010字段中pitch/roll/yaw分别对应传感器X/Y/Z轴。
		WriteF32LE(payload + 44, gyro.pitch * DEG_TO_RAD);
		WriteF32LE(payload + 48, gyro.roll * DEG_TO_RAD);
		WriteF32LE(payload + 52, gyro.yaw * DEG_TO_RAD);

		WriteF32LE(payload + 56, accel.x * G_TO_MPS2);
		WriteF32LE(payload + 60, accel.y * G_TO_MPS2);
		WriteF32LE(payload + 64, accel.z * G_TO_MPS2);

		SendFrame(MSG_POSE,0,session_id_,payload,sizeof(payload));
	};

	// 目前只发两个IMU的原生坐标；安装变换确认后再增加0/0支架姿态。
	send_imu(1, 1, imu_small_pantile);
	send_imu(2, 2, imu_big_pantile);
}

void XUC::SendJointState()
{
	constexpr float PI_F = 3.14159265359f;
	constexpr float TWO_PI = 6.28318530718f;
	constexpr float ENCODER_TO_RAD = TWO_PI / 8192.0f;
	constexpr float RPM_TO_RAD_S = TWO_PI / 60.0f;

	const uint32_t now_ms = HAL_GetTick();
	const uint64_t now_us = NowUs();
	Motor* small_yaw = ctrl.pantile_motor[CONTROL::PANTILE::YAW];

	uint8_t payload[112]{};
	payload[0] = 3;
	payload[1] = 0;
	WriteU16LE(payload + 2, 0x0001U);
	WriteU64LE(payload + 4, now_us);
	WriteU32LE(payload + 12, FRAME_EPOCH);

	auto feedback_age_us = [now_ms](uint32_t last_tick) -> uint32_t
	{
		const uint32_t age_ms = now_ms - last_tick;
		return age_ms > 4294967U ? UINT32_MAX : age_ms * 1000U;
	};

	auto write_joint = [&](uint8_t* item,
	                       uint8_t axis_id,
	                       uint8_t state_bits,
	                       uint16_t fault_code,
	                       float angle,
	                       float velocity,
	                       float target_angle,
	                       float target_velocity,
	                       float temperature,
	                       uint32_t age_us)
	{
		item[0] = axis_id;
		item[1] = state_bits;
		WriteU16LE(item + 2, fault_code);
		WriteF32LE(item + 4, angle);
		WriteF32LE(item + 8, velocity);
		WriteF32LE(item + 12, target_angle);
		WriteF32LE(item + 16, target_velocity);
		WriteF32LE(item + 20, 0.0f); // 暂无统一的驱动输出比例。
		WriteF32LE(item + 24, temperature);
		WriteU32LE(item + 28, age_us);
	};

	// axis 0：小Yaw，以251编码值为当前机械前向零点。
	uint8_t small_bits = 0;
	uint16_t small_fault = 1;
	float small_angle = 0.0f;
	float small_velocity = 0.0f;
	float small_target = 0.0f;
	float small_temperature = 0.0f;
	uint32_t small_age = UINT32_MAX;

	if (small_yaw != nullptr)
	{
		small_age = feedback_age_us(small_yaw->last_feedback_tick_);
		if (small_yaw->feedback_online_)
		{
			small_bits |= (1U << 0); // 在线
			small_bits |= (1U << 3); // 角度有效
			small_bits |= (1U << 4); // 速度有效
			small_bits |= (1U << 6); // 温度有效
			small_fault = 0;

			if (small_yaw->mode != IDLE)
				small_bits |= (1U << 1);

			float relative_count =
				small_yaw->angle[now] -
				ctrl.chassis.small_yaw_forward_count;
			while (relative_count > 4096.0f)
				relative_count -= 8192.0f;
			while (relative_count < -4096.0f)
				relative_count += 8192.0f;

			small_angle = relative_count * ENCODER_TO_RAD;
			small_velocity =
				static_cast<float>(small_yaw->curspeed) * RPM_TO_RAD_S;
			small_temperature = static_cast<float>(small_yaw->temperature);

			if (small_yaw->temperature > 70)
				small_fault = 2;

			if (small_yaw->mode == POS)
			{
				float target_count =
					small_yaw->setangle -
					ctrl.chassis.small_yaw_forward_count;
				while (target_count > 4096.0f)
					target_count -= 8192.0f;
				while (target_count < -4096.0f)
					target_count += 8192.0f;

				small_target = target_count * ENCODER_TO_RAD;
				small_bits |= (1U << 5);
			}
		}
	}

	write_joint(
		payload + 16,
		0,
		small_bits,
		small_fault,
		small_angle,
		small_velocity,
		small_target,
		0.0f,
		small_temperature,
		small_age);

	// axis 1：Pitch抬头时DM位置减小，因此协议中的抬头正方向取负。
	const bool pitch_online = DM_motorPitch.FeedbackFresh(now_ms,DMProtocol::FEEDBACK_TIMEOUT_MS);
	uint8_t pitch_bits = 0;
	uint16_t pitch_fault = 1;

	if (pitch_online)
	{
		pitch_bits |= (1U << 0);
		pitch_bits |= (1U << 3);
		pitch_bits |= (1U << 4);
		pitch_fault = 0;
		if (ctrl.pantile.pitch_target_ready)
			pitch_bits |= (1U << 5);
	}

	write_joint(
		payload + 48,
		1,
		pitch_bits,
		pitch_fault,
		pitch_online ? -DM_motorPitch.pos : 0.0f,
		pitch_online ? -DM_motorPitch.curSpeed : 0.0f,
		ctrl.pantile.pitch_target_ready ? -DM_motorPitch.setPos : 0.0f,
		0.0f,
		0.0f,
		feedback_age_us(DM_motorPitch.GetLastFeedbackTick()));

	// axis 2：大Yaw反馈为[-pi, pi]，在这里解环为连续关节角。
	const bool big_online = DM_motorYaw.FeedbackFresh(now_ms,DMProtocol::FEEDBACK_TIMEOUT_MS);
	static bool big_angle_ready = false;
	static float last_big_raw = 0.0f;
	static float continuous_big_angle = 0.0f;
	static uint32_t last_big_counter = 0;
	const uint32_t big_counter = DM_motorYaw.GetFeedbackCount();

	if (big_online &&(!big_angle_ready || big_counter != last_big_counter))
	{
		const float current_raw = DM_motorYaw.pos;
		if (!big_angle_ready)
		{
			continuous_big_angle =current_raw - ctrl.chassis.big_yaw_forward_pos;
			big_angle_ready = true;
		}
		else
		{
			float delta = current_raw - last_big_raw;
			while (delta > PI_F)
				delta -= TWO_PI;
			while (delta < -PI_F)
				delta += TWO_PI;
			continuous_big_angle += delta;
		}

		last_big_raw = current_raw;
		last_big_counter = big_counter;
	}

	uint8_t big_bits = 0;
	uint16_t big_fault = 1;
	if (big_online && big_angle_ready)
	{
		big_bits |= (1U << 0);
		big_bits |= (1U << 3);
		big_bits |= (1U << 4);
		big_fault = 0;
	}

	write_joint(
		payload + 80,
		2,
		big_bits,
		big_fault,
		big_online ? continuous_big_angle : 0.0f,
		big_online ? DM_motorYaw.curSpeed : 0.0f,
		0.0f,
		0.0f,
		0.0f,
		feedback_age_us(DM_motorYaw.GetLastFeedbackTick()));

	// 没有可靠回零状态，因此三个轴均不设置state_bits.bit2。
	SendFrame(
		MSG_JOINT_STATE,
		0,
		session_id_,
		payload,
		sizeof(payload));
}
