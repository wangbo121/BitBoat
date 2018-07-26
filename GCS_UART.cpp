/*
 *@File     : GCS_UART.cpp
 *@Author   : wangbo
 *@Date     : Jul 26, 2018
 *@Copyright: 2018 Beijing Institute of Technology. All rights reserved.
 *@Warning  : This content is limited to internal circulation, forbidding disclosure and for other commercial purposes.
 */

#include "Boat.h"
#include "GCS_UART.h"

GCS_UART::GCS_UART()
{
    stream_rate_BIT_Start               = 0;
    stream_rate_BIT_RealTime            = 2;
    stream_rate_BIT_End                 = 0;
}

void GCS_UART::gcs_init()
{
    uart_device.uart_name = (char *)UART_GCS;

    uart_device.baudrate = GCS_UART_BAUD;
    uart_device.databits = GCS_UART_DATABITS;
    uart_device.parity   = GCS_UART_PARITY;
    uart_device.stopbits = GCS_UART_STOPBITS;

    uart_device.uart_num = open_uart_dev(uart_device.uart_name);

    set_uart_opt( uart_device.uart_name, \
                  uart_device.baudrate,\
                  uart_device.databits,\
                  uart_device.parity,\
                  uart_device.stopbits);
}

// see if we should send a stream now. Called at 50Hz
bool GCS_UART::stream_trigger(enum streams stream_num)
{
    int16_t *stream_rates = &stream_rate_BIT_Start;
    uint8_t rate = (uint8_t)stream_rates[stream_num];

    if (rate == 0) {
        return false;
    }

    if (stream_ticks[stream_num] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > 50) {
            rate = 50;
        }
        stream_ticks[stream_num] = (50 / rate);
        return true;
    }

    // count down at 50Hz
    stream_ticks[stream_num]--;
    return false;
}

void
GCS_UART::data_stream_send(void)
{
    if (stream_trigger(STREAM_BIT_START))
    {
    }

    if (stream_trigger(STREAM_BIT_RealTime))
    {
        send_message(MSG_RealTime);
    }

   if (stream_trigger(STREAM_BIT_END))
    {
    }
}

void
GCS_UART::send_message(enum BIT_message_ID id)
{
    switch(id)
       {
       case MSG_RealTime:
           //copter.send_ap2gcs_MSP_ident();
           break;

       default:
           break; // just here to prevent a warning
       }
}







/***************************************************************************/
void GCS_UART::serialize8(uint8_t a)
{
    _outBuf[_outBufSize++] = a;
    _checksum ^= a;
}

void GCS_UART::serialize16(int16_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
}

uint8_t GCS_UART::read8(void)
{
    return _inBuf[_inBufIndex++] & 0xff;
}

uint16_t GCS_UART::read16(void)
{
    uint16_t t = read8();
    t += (uint16_t)read8() << 8;
    return t;
}

uint32_t GCS_UART::read32(void)
{
    uint32_t t = read16();
    t += (uint32_t)read16() << 16;
    return t;
}

float GCS_UART::readFloat(void)
{
    float f = 0;
    uint32_t t = read32();
    memcpy(&f, &t, 4);
    return f;
}

void GCS_UART::serializeFloats(float f[], uint8_t n)
{
    headSerialReply(4*n);

    for (uint8_t k=0; k<n; ++k) {
        uint32_t a;
        memcpy(&a, &f[k], 4);
        serialize32(a);
    }
}

void GCS_UART::serialize32(uint32_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
    serialize8((a >> 16) & 0xFF);
    serialize8((a >> 24) & 0xFF);
}


void GCS_UART::headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    _checksum = 0;               // start calculating a new _checksum
    serialize8(s);
    serialize8(_cmdMSP);
}

void GCS_UART::headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

void GCS_UART::headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

void GCS_UART::tailSerialReply(void)
{
    serialize8(_checksum);
}

void GCS_UART::dispatch_command(void)
{
    switch (_type)
    {
    case MSG_SET_CMD:
        /// 先处理地面站传输过来的数据

//        /// 收到数据后把数据存放到驾驶仪的某个数据结构中
//        memcpy(&msp_set_pid, _inBuf, sizeof(msp_set_pid));
//        MSP_bool_receive_CMD_PID = TRUE;
//
//        /// 接着给地面站回复响应
//        _outBufSize = 0;
//        _outBufIndex = 0;
//
//        headSerialReply(0); /// 收到任何数据都是需要返回给确认 给地面站的
        break;

    case MSG_SET_WP:
        break;
    // don't know how to handle the (valid) message, indicate error
    default:
        headSerialError(0);
        break;
    }
}


void  GCS_UART::decode_char(uint8_t c)
{
    static uint8_t  len_low;
    static uint8_t  len_high;
    static uint16_t len_to_receive;

    switch (_parserState)
    {
    case IDLE:
        _parserState = (c == 0xaa) ? HEADER_AA : IDLE;
        break;
    case HEADER_AA:
        _parserState = (c == 0x55) ? HEADER_55 : IDLE;
        break;
    case HEADER_55:
        len_low = c;
        _parserState = HEADER_LEN1;
        break;
    case HEADER_LEN1:
        len_high = c;
        len_to_receive = (uint16_t)len_high * pow(2,8) + len_low;
        if (len_to_receive > INBUF_SIZE)
        {       // now we are expecting the payload size
            _parserState = IDLE;
            return;
        }
        _dataSize = len_to_receive;
        _offset = 0;
        _checksum = 0;
        _inBufIndex = 0;
        _checksum += len_low;
        _checksum += len_high;
        _parserState = HEADER_LEN2; // the command is to follow
        break;
    case HEADER_LEN2:
        _type = c;
        _checksum += c;
        _parserState = HEADER_TYPE;
        break;
    case HEADER_TYPE:
       _vessel = c;
       _checksum += c;
       _parserState = HEADER_VESSEL;
       break;
    case HEADER_VESSEL:
        _link = c;
        _checksum += c;
        _parserState = HEADER_LINK;
        break;
    case HEADER_LINK:
        _id = c;
        _checksum += c;
        _parserState = HEADER_ID;
        break;
    case HEADER_ID:
       _cnt = c;
       _checksum += c;
       _parserState = HEADER_CNT;
       break;
    case HEADER_CNT:
       _func_flag = c;
       _checksum += c;
       _parserState = HEADER_FUNC_FLAG;
       break;
    case HEADER_FUNC_FLAG:
       _func_info1 = c;
       _checksum += c;
       _parserState = HEADER_FUNC_INFO1;
       break;
    case HEADER_FUNC_INFO1:
       _func_info2 = c;
       _checksum += c;
       _parserState = HEADER_FUNC_INFO2;
       break;
    case HEADER_FUNC_INFO2:
        if (_offset < _dataSize) {
            _checksum += c;
            _inBuf[_offset++] = c;
        } else  {
            if (_checksum == c) {        // compare calculated and transferred _checksum
                dispatch_command();
                tailSerialReply();
            }
            _parserState = IDLE;
        }
       break;

    default:
        break;
    } // switch (_parserState)
}


uint8_t GCS_UART::availableBytes(void)
{
    return _outBufSize;
}

uint8_t GCS_UART::readByte(void)
{
    _outBufSize--;
    return _outBuf[_outBufIndex++];
}







