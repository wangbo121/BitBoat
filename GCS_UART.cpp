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
    uart_device.uart_name = UART_GCS;

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

void
GCS_UART::decode_char(uint8_t c)
{



}











