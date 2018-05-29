/*
 * gps_Y901.cpp
 *
 *  Created on: Apr 25, 2018
 *      Author: wangbo
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "global.h"
#include "uart.h"
#include "utility.h"

#include "gps_Y901.h"

struct STime        stcTime;
struct SAcc         stcAcc;
struct SGyro        stcGyro;
struct SAngle   stcAngle;
struct SMag         stcMag;
struct SDStatus stcDStatus;
struct SPress   stcPress;
struct SLonLat  stcLonLat;
struct SGPSV        stcGPSV;
struct SQ       stcQ;


/*extern variable*/
nmea_msg_Y901 gps_data_Y901;

static struct T_UART_DEVICE uart_device_gps_Y901;

int gps_uart_init_Y901()
{
    uart_device_gps_Y901.uart_name=UART_GPS_Y901;

    uart_device_gps_Y901.baudrate=UART_GPS_BAUD_Y901;
    uart_device_gps_Y901.databits=UART_GPS_DATABITS_Y901;
    uart_device_gps_Y901.parity=UART_GPS_PARITY_Y901;
    uart_device_gps_Y901.stopbits=UART_GPS_STOPBITS_Y901;

    uart_device_gps_Y901.uart_num=open_uart_dev(uart_device_gps_Y901.uart_name);

    set_uart_opt( uart_device_gps_Y901.uart_name, \
                  uart_device_gps_Y901.baudrate,\
                  uart_device_gps_Y901.databits,\
                  uart_device_gps_Y901.parity,\
                  uart_device_gps_Y901.stopbits);

    return 0;
}

#define GPS_Y901_BUF_SIZE 256
#define GPS_Y901_RECV_HEAD1    0
#define GPS_Y901_RECV_HEAD2    1
#define GPS_Y901_RECV_DATA     2
#define GPS_Y901_RECV_CHECKSUM 3
static int gps_Y901_recv_state = 0;
int decode_data_gps_Y901(unsigned char *buf, int len)
{
    static unsigned char _buffer[GPS_Y901_BUF_SIZE];

    static int _pack_recv_real_len = 8;//表示收到的包中的数据包应该收到的除了帧头和校验的长度，Y901是8位
    static unsigned char _pack_recv_buf[GPS_Y901_BUF_SIZE];
    static int _pack_buf_len = 0;

    unsigned char data_type;

    int _length;
    unsigned char c;

    int i=0;
    static unsigned char checksum = 0;

    memcpy(_buffer, buf, len);

    _length=len;

    for (i = 0; i < _length; i++)
    {
        c = _buffer[i];
        switch (gps_Y901_recv_state)
        {
        case GPS_Y901_RECV_HEAD1:
            if(c == 0x55)
            {
                gps_Y901_recv_state = GPS_Y901_RECV_HEAD2;
                checksum += c;
            }
            else
            {
                gps_Y901_recv_state = GPS_Y901_RECV_HEAD1;
            }
            break;
        case GPS_Y901_RECV_HEAD2:
            if((c == 0x50) || (c == 0x51) || (c == 0x52) || (c == 0x53) || (c == 0x54) || \
              (c == 0x55) || (c == 0x56) || (c == 0x57) || (c == 0x58) || (c == 0x59) )
            {
                gps_Y901_recv_state = GPS_Y901_RECV_DATA;
                checksum += c;
                data_type = c;
                _pack_buf_len = 0;
            }
            else
            {
                gps_Y901_recv_state = GPS_Y901_RECV_HEAD1;
                checksum = 0;
            }
            break;
        case GPS_Y901_RECV_DATA:
            _pack_recv_buf[_pack_buf_len] = c;
            _pack_buf_len++;
            checksum += c;
            if(_pack_buf_len >= _pack_recv_real_len)
            {
                gps_Y901_recv_state = GPS_Y901_RECV_CHECKSUM;
            }
            break;
        case GPS_Y901_RECV_CHECKSUM:
            if(c == checksum)
            {
                switch( data_type )
                {
                case 0x50:  memcpy(&stcTime,_pack_recv_buf, 8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
                case 0x51:  memcpy(&stcAcc,_pack_recv_buf, 8);break;
                case 0x52:  memcpy(&stcGyro,_pack_recv_buf,8);break;
                case 0x53:  memcpy(&stcAngle,_pack_recv_buf,8);break;
                case 0x54:  memcpy(&stcMag,_pack_recv_buf,8);break;
                case 0x55:  memcpy(&stcDStatus,_pack_recv_buf,8);break;
                case 0x56:  memcpy(&stcPress,_pack_recv_buf,8);break;
                case 0x57:  memcpy(&stcLonLat,_pack_recv_buf,8);break;
                case 0x58:  memcpy(&stcGPSV,_pack_recv_buf,8);break;
                case 0x59:  memcpy(&stcQ,_pack_recv_buf,8);break;
                }
            }
            else
            {
                gps_Y901_recv_state = GPS_Y901_RECV_HEAD1;
                checksum = 0;
            }
            break;
        default:
            break;
        }
    }

    return 0;
}

void print_data_gps_Y901()
{
    char str[256];
    //输出时间
    sprintf(str,"Time:20%d-%d-%d %d:%d:%.3f\r\n",stcTime.ucYear,stcTime.ucMonth,stcTime.ucDay,stcTime.ucHour,stcTime.ucMinute,(float)stcTime.ucSecond+(float)stcTime.usMiliSecond/1000);
    printf("%s",str);
    delay_ms(10);
    //输出加速度
    sprintf(str,"Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
    printf("%s",str);
    delay_ms(10);
    //输出角速度
    sprintf(str,"Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
    printf("%s",str);
    delay_ms(10);
    //输出角度
    sprintf(str,"Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
    printf("%s",str);
    delay_ms(10);
    //输出磁场
    sprintf(str,"Mag:%d %d %d\r\n",stcMag.h[0],stcMag.h[1],stcMag.h[2]);
    printf("%s",str);
    delay_ms(10);
    //输出气压、高度
    sprintf(str,"Pressure:%ld Height%.2f\r\n",stcPress.lPressure,(float)stcPress.lAltitude/100);
    printf("%s",str);
    delay_ms(10);
    //输出端口状态
    sprintf(str,"DStatus:%d %d %d %d\r\n",stcDStatus.sDStatus[0],stcDStatus.sDStatus[1],stcDStatus.sDStatus[2],stcDStatus.sDStatus[3]);
    printf("%s",str);
    delay_ms(10);
    //输出经纬度
    sprintf(str,"Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",stcLonLat.lLon/10000000,(double)(stcLonLat.lLon % 10000000)/1e5,stcLonLat.lLat/10000000,(double)(stcLonLat.lLat % 10000000)/1e5);
    printf("%s",str);
    delay_ms(10);
    //输出地速
    sprintf(str,"GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n",(float)stcGPSV.sGPSHeight/10,(float)stcGPSV.sGPSYaw/10,(float)stcGPSV.lGPSVelocity/1000);
    printf("%s",str);
    delay_ms(10);
    //输出四元素
    sprintf(str,"Four elements:%.5f %.5f %.5f %.5f\r\n\r\n",(float)stcQ.q[0]/32768,(float)stcQ.q[1]/32768,(float)stcQ.q[2]/32768,(float)stcQ.q[3]/32768);
    printf("%s",str);
}

int read_gps_data_Y901()
{
    static char recv_buf[256];
    static int recv_buf_require_len = 256;// 请求接收的字节数目
    static int recv_buf_real_len = 0; //实际接收的字节数目
    static int max_wait_ms = 2;//最大等待时间ms

    recv_buf_real_len = read_uart_data(uart_device_gps_Y901.uart_name, recv_buf, max_wait_ms, recv_buf_require_len);

    if( recv_buf_real_len > 0)
    {
        //DEBUG_PRINTF("read_gps_data_Y901    :    receive %d bytes\n",recv_buf_real_len);
        decode_data_gps_Y901((unsigned char*)recv_buf, recv_buf_real_len);
    }

    return 0;
}

int gps_uart_close_Y901()
{
    uart_device_gps_Y901.uart_name=UART_GPS_Y901;
    close_uart_dev(uart_device_gps_Y901.uart_name);

    return 0;
}
