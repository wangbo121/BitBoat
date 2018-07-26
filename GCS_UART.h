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

#define GCS_UART_BAUD 9600
#define GCS_UART_DATABITS 8 //8 data bit
#define GCS_UART_STOPBITS 1 //1 stop bit
#define GCS_UART_PARITY 0 //no parity

#define INBUF_SIZE 256
#define OUTBUF_SIZE 256

enum BIT_message_ID {

    MSG_RealTime,

    MSG_MSP_RETRY_DEFERRED // this must be last
};

class GCS_UART : public GCS_Class
{
public:
    GCS_UART();
    void    update(void);
    void    data_stream_send(void);
    void    send_message(enum BIT_message_ID id);

    void    decode_char(uint8_t c);
    void    dispatch_command(void);

public:
    //这是数据流的列表，也就是说一共有多少个数据流，每个数据流可以只包含一种数据结构，也可以包含多个数据结构包
    enum streams
    {
        STREAM_BIT_START,

        STREAM_BIT_RealTime,

        STREAM_BIT_END,

        NUM_STREAMS /// for stream_ticks[] and stream_rates[]
    };

    // 判断是否应该发送data_stream中的某条数据结构
    bool stream_trigger(enum streams stream_num);

    /// 这是数据流发送频率的列表，需要初始化这些stream_rate_xxx 告诉发送频率
    // 这里的数据结构必须是int16_t 需要跟stream_trigger函数中的int16_t *stream_rates = &stream_rate_BIT_Start;语句一致
    int16_t stream_rate_BIT_Start;
    int16_t stream_rate_BIT_RealTime;
    int16_t stream_rate_BIT_End;

    // 记录每条数据流已经等待的tick计数
    // number of 50Hz ticks until we next send this stream
    uint8_t stream_ticks[NUM_STREAMS];

    // 每条数据流的发送频率
    int16_t  stream_rates[NUM_STREAMS];


public:

    struct T_UART_DEVICE uart_device;


public:
    // 解析地面站发送过来的数据时-采用状态机的模式-下面是状态机的各个状态
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

public:
    void gcs_init();
};




#endif /* GCS_UART_H_ */

