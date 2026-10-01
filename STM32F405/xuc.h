#pragma once

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "auto_command.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

class XUC
{
public:
    void Init(UART* huart, USART_TypeDef* instance, uint32_t baud_rate);
    void Decode();
    void Encode();
    void Update();

    const AutoCommand& GetCommand() const { return command; }
    bool IsSessionActive() const { return session_active_; }
    uint32_t GetSessionId() const { return session_id_; }

    AutoCommand command{};

private:
    static constexpr uint16_t RMV_HEADER_SIZE = 28;
    static constexpr uint16_t RMV_CRC_SIZE = 2;
    static constexpr uint16_t RMV_MAX_PAYLOAD = 256;
    static constexpr uint16_t RMV_MAX_FRAME_SIZE = 286;
    static constexpr uint16_t RMV_RX_STREAM_SIZE = 640;

    static constexpr uint8_t RMV_MAJOR = 1;
    static constexpr uint8_t RMV_MINOR = 0;

    static constexpr uint16_t FLAG_ACK_REQUEST = 0x0001;
    static constexpr uint16_t FLAG_RESPONSE = 0x0002;

    static constexpr uint16_t MSG_HELLO = 0x0001;
    static constexpr uint16_t MSG_TIME_SYNC = 0x0002;
    static constexpr uint16_t MSG_HEARTBEAT = 0x0003;
    static constexpr uint16_t MSG_SUBSCRIBE = 0x0004;
    static constexpr uint16_t MSG_ACK = 0x00FF;
    static constexpr uint16_t MSG_STATUS = 0x0101;
    static constexpr uint16_t MSG_POSE = 0x0102;
    static constexpr uint16_t MSG_JOINT_STATE = 0x0103;
    static constexpr uint16_t MSG_SERVICE_STATE = 0x0106;
    static constexpr uint16_t MSG_DIAGNOSTICS = 0x0108;
	
    static constexpr uint16_t MSG_MODE_REQUEST = 0x0200;
    static constexpr uint16_t MSG_AIM_SETPOINT = 0x0201;
    static constexpr uint16_t MSG_STOP_REMOTE = 0x0202;
    static constexpr uint16_t MSG_SERVICE_REQUEST = 0x0205;
    static constexpr uint16_t MSG_PARAM_LIST_REQ = 0x0300;
    static constexpr uint16_t MSG_PARAM_DESC_REQ = 0x0301;
    static constexpr uint16_t MSG_PARAM_STAGE = 0x0302;
    static constexpr uint16_t MSG_PARAM_COMMIT = 0x0303;
    static constexpr uint16_t MSG_PARAM_ABORT = 0x0304;
    static constexpr uint16_t MSG_HELLO_REPLY = 0x8001;
    static constexpr uint16_t MSG_TIME_REPLY = 0x8002;

    static constexpr uint32_t FW_VERSION = 20260930U;

    // 尚未完成姿态合成、关节反馈、标定和参数接口，不提前声明能力。
    static constexpr uint32_t SUPPORTED_CAPS = 0x00000000U;
    static constexpr uint32_t PARAM_REV = 0U;
    static constexpr uint32_t GEOMETRY_REV = 0U;
    static constexpr uint32_t FRAME_EPOCH = 1U;

    static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500U;
    static constexpr uint32_t CANDIDATE_TIMEOUT_MS = 50U;

    enum ResultCode : uint16_t
    {
        RESULT_OK = 0,
        RESULT_UNSUPPORTED = 1,
        RESULT_BAD_VERSION = 2,
        RESULT_BAD_LENGTH = 3,
        RESULT_BAD_VALUE = 4,
        RESULT_BAD_SESSION = 5,
        RESULT_WRONG_MODE = 6,
        RESULT_NOT_READY = 7,
        RESULT_STALE = 8,
        RESULT_OUT_OF_RANGE = 9,
        RESULT_BUSY = 10,
        RESULT_CONFLICT = 11,
        RESULT_HARDWARE_FAULT = 12,
        RESULT_RESOURCE_LIMIT = 13
    };

    enum RemoteMode : uint8_t
    {
        MODE_SAFE = 0,
        MODE_MANUAL = 1,
        MODE_VISION = 2,
        MODE_CALIBRATION = 3,
        MODE_FAULT = 4
    };

    struct Diagnostics
    {
        uint32_t rx_crc_errors = 0;
        uint32_t rx_format_errors = 0;
        uint32_t rx_bad_session = 0;
        uint32_t expired_commands = 0;
        uint32_t duplicate_commands = 0;
        uint32_t tx_dropped_frames = 0;
        uint32_t tx_max_queue_delay_us = 0;
        uint32_t control_period_us = 5000;
        uint32_t control_max_jitter_us = 0;
        uint32_t control_overrun_count = 0;
    } diagnostics_{};

    UART* uart_ = nullptr;
    BaseType_t receive_result_ = pdFALSE;

    uint8_t dma_frame_[UART_MAX_LEN]{};
    uint8_t rx_stream_[RMV_RX_STREAM_SIZE]{};
    uint16_t rx_stream_size_ = 0;
    uint8_t tx_frame_[320]{};

    bool candidate_waiting_ = false;
    uint32_t candidate_start_tick_ = 0;

    uint64_t boot_id_ = 0;
    uint32_t session_id_ = 0;
    uint32_t session_counter_ = 0;
    uint32_t tx_seq_ = 0;
    bool session_active_ = false;

    uint64_t last_hello_nonce_ = 0;
    uint8_t last_hello_payload_[16]{};
    bool have_last_hello_ = false;

    uint32_t last_heartbeat_tick_ = 0;
    uint8_t host_state_ = 0;
    RemoteMode remote_mode_ = MODE_SAFE;

    uint32_t last_received_aim_seq_ = 0;
    uint32_t last_applied_aim_seq_ = 0;
    uint32_t last_accepted_aim_seq_ = 0;
    uint16_t last_aim_result_ = RESULT_NOT_READY;
    uint16_t constraint_flags_ = 0;
    bool have_accepted_aim_seq_ = false;

    bool have_operation_ = false;
    uint32_t last_operation_seq_ = 0;
    uint16_t last_operation_msg_ = 0;
    uint16_t last_operation_payload_len_ = 0;
    uint8_t last_operation_payload_[RMV_MAX_PAYLOAD]{};
    uint16_t last_operation_result_ = RESULT_OK;
    uint32_t last_operation_detail_ = 0;

    uint16_t status_rate_hz_ = 20;
    uint16_t diagnostics_rate_hz_ = 1;
    uint16_t service_state_rate_hz_ = 1;
    uint32_t last_status_tick_ = 0;
    uint32_t last_diagnostics_tick_ = 0;
    uint32_t last_service_state_tick_ = 0;

    void AppendRxData(const uint8_t* data, uint16_t len);
    void DropRxData(uint16_t len);
    void ParseRxStream();
    void HandleFrame(const uint8_t* frame, uint16_t frame_len, uint64_t receive_time_us);

    uint16_t RmvCrc16(const uint8_t* data, uint16_t len) const;
    uint64_t NowUs() const;
    uint64_t GenerateBootId();

    bool SendFrame(uint16_t msg_type,
                   uint16_t flags,
                   uint32_t output_session_id,
                   const uint8_t* payload,
                   uint16_t payload_len);
    void SendAck(uint32_t request_seq,
                 uint16_t request_msg_type,
                 uint16_t result,
                 uint32_t detail,
                 uint32_t output_session_id);

    void HandleHello(const uint8_t* payload,
                     uint16_t payload_len,
                     uint32_t request_seq,
                     uint32_t request_session_id,
                     uint16_t request_flags);
    void SendHelloReply(uint64_t nonce);
    void HandleTimeSync(const uint8_t* payload,
                        uint16_t payload_len,
                        uint64_t receive_time_us);
    void HandleHeartbeat(const uint8_t* payload, uint16_t payload_len);
    void HandleSubscribe(const uint8_t* payload,
                         uint16_t payload_len,
                         uint32_t seq,
                         uint16_t flags);
    void HandleModeRequest(const uint8_t* payload,
                           uint16_t payload_len,
                           uint32_t seq,
                           uint16_t flags);
    void HandleAimSetpoint(const uint8_t* payload,
                           uint16_t payload_len,
                           uint32_t seq,
                           uint16_t flags);
    void HandleStopRemote(const uint8_t* payload,
                          uint16_t payload_len,
                          uint32_t seq,
                          uint16_t flags);

    bool IsNewerSeq(uint32_t value, uint32_t reference) const;
    bool HandleRepeatedOperation(uint16_t msg_type,
                                 uint32_t seq,
                                 const uint8_t* payload,
                                 uint16_t payload_len);
    void RememberOperation(uint16_t msg_type,
                           uint32_t seq,
                           const uint8_t* payload,
                           uint16_t payload_len,
                           uint16_t result,
                           uint32_t detail);

    bool LocalVisionPermit() const;
    bool RemoteControlReady() const;
    void ClearRemoteControl();

    void SendStatus();
    void SendDiagnostics();
    void SendServiceState();
	
	void SendPose();
	void SendJointState();
};

extern XUC xuc;
