/*
 *@File     : GCS_UART.h
 *@Author   : wangbo
 *@Date     : Jul 26, 2018
 *@Copyright: 2018 Beijing Institute of Technology. All rights reserved.
 *@Warning  : This content is limited to internal circulation, forbidding disclosure and for other commercial purposes.
 */
#ifndef GCS_UART_H_
#define GCS_UART_H_

#include "GCS.h"

#define INBUF_SIZE 256
#define OUTBUF_SIZE 256

enum ap_message_msp {

    MSG_MSP_API_VERSION                = 1,    //out message
    MSG_MSP_FC_VARIANT                 = 2,    //out message
    MSG_MSP_FC_VERSION                 = 3,    //out message
    MSG_MSP_BOARD_INFO                 = 4,    //out message
    MSG_MSP_BUILD_INFO                 = 5,    //out message



    MSG_MSP_NAME                       = 10,   //out message          Returns user set board name - betaflight
    MSG_MSP_SET_NAME                   = 11,   //in message           Sets board name - betaflight
    MSG_MSP_IDENT                      = 100,
    MSG_MSP_STATUS,
    MSG_MSP_RAW_IMU,

    MSP_MSP_MODE_STATUS                = 120,
    MSG_MSP_RC_NORMAL                  = 121,
    MSG_MSP_ATTITUDE_RADIANS           = 122,

    MSG_MSP_PID                        = 112,
    MSG_MSP_SET_PID                    = 202,

    MSG_MSP_SET_MOTOR_NORMAL           = 215,
    MSG_MSP_SET_ARMED                  = 216,

    MSG_MSP_RAW_GPS                    = 106,
    MSG_MSP_COMP_GPS,
    MSG_MSP_ATTITUDE                   = 108,

    MSG_MSP_UID                        = 160,    //out message         Unique device ID


    MSG_MSP_ACC_CALIBRATION = 205,
    MSG_MSP_GYRO_CALIBRATION = 206,

    MSG_MSP_RC                =105, ///
    MSG_MSP_SET_RC_TUNING_START     = 207,
    MSG_MSP_SET_RC_TUNING_END = 208,
    MSG_MSP_RC_RAW            = 110,   //

    MSG_MSP_SET_ESC_CALIBRATE      = 210,
    MSG_MSP_SET_ESC_CALIBRATE_DONE = 211,


    MSG_MSP_RETRY_DEFERRED // this must be last
};

class GCS_MSP : public GCS_Class
{
public:
    GCS_MSP();
    void    update(void);
    void    data_stream_send(uint16_t freqMin, uint16_t freqMax);
    void    data_stream_send(void);
    void    send_message(enum ap_message_msp id);
    void    mavlink_send_message(enum ap_message_msp id);

    ///  for MSP
    void    decode_char(uint8_t c);
    void    dispatchCommand(void);
public:
    //void  handleMessage(mavlink_message_t * msg);

    // NOTE! The streams enum below and the
    // set of stream rates _must_ be
    // kept in the same order
    enum streams
    {
        STREAM_MSP_START,

        STREAM_MSP_ATTITUDE,

        STREAM_MSP_END,

        NUM_STREAMS /// for stream_ticks[] and streamRates[]
    };
    // see if we should send a stream now. Called at 50Hz
    bool stream_trigger(enum streams stream_num);

    /// this is set of stream rates
    /// data stream rates 实时数据发送的频率
    int16_t streamRateMSP_Start;
    int16_t streamRateMSP_ATTITUDE;
    int16_t streamRateMSP_End;

    // number of 50Hz ticks until we next send this stream
    uint8_t stream_ticks[NUM_STREAMS];

    // saveable rate of each stream
    int16_t  streamRates[NUM_STREAMS];


public:




public :
    // 添加msp协议
    typedef enum serialState_t
    {
        IDLE,
        HEADER_START,
        HEADER_M,
        HEADER_ARROW,
        HEADER_SIZE,
        HEADER_CMD
    }serialState_t;

    uint8_t _checksum;
    int8_t  _inBuf[INBUF_SIZE];
    uint8_t _inBufIndex;
    uint8_t _outBuf[OUTBUF_SIZE];
    uint8_t _outBufIndex;
    uint8_t _outBufSize;
    uint8_t _cmdMSP;
    uint8_t _offset;
    uint8_t _dataSize;

    serialState_t   _parserState;

    /// 获取命令包标志量
    uint8_t MSP_bool_receive_CMD_PID;
    uint8_t MSP_bool_receive_CMD_ACC_CALIBRATE;
    uint8_t MSP_bool_receive_CMD_GYRO_CALIBRATE;
    uint8_t MSP_bool_receive_CMD_RC;
    uint8_t MSP_bool_receive_CMD_RC_RAW;
    uint8_t MSP_bool_receive_CMD_RC_CALIBRATE_Start;
    uint8_t MSP_bool_receive_CMD_RC_CALIBRATE_End;

public:

    void serialize8(uint8_t a);
    void serialize16(int16_t a);
    void serialize32(uint32_t a);
    void serializeFloats(float f[], uint8_t n);

    uint8_t  read8(void);
    uint16_t read16(void);
    uint32_t read32(void);
    float    readFloat(void);

    void headSerialResponse(uint8_t err, uint8_t s);

    void headSerialReply(uint8_t s);
    void headSerialError(uint8_t s);
    void tailSerialReply(void);

    uint8_t availableBytes(void);
    uint8_t readByte(void);
};




#endif /* GCS_UART_H_ */

