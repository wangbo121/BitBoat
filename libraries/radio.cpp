/*
 * radio.c
 *
 *  Created on: 2016年5月9日
 *      Author: wangbo
 */

#include <stdio.h>
#include <string.h>
#include <byteswap.h> //__bswap_32()，转换int或者short的字节顺序，解决大小端问题
#include <unistd.h>
#include <math.h>

#include "global.h"
#include "uart.h"

#include "radio.h"

#define RADIO_RECV_BUF_SIZE 512

static struct T_UART_DEVICE uart_device_radio;

int radio_uart_init()
{
    uart_device_radio.uart_name=UART_RADIO;

    uart_device_radio.baudrate=UART_RADIO_BAUD;
    uart_device_radio.databits=UART_RADIO_DATABITS;
    uart_device_radio.parity=UART_RADIO_PARITY;
    uart_device_radio.stopbits=UART_RADIO_STOPBITS;

    uart_device_radio.uart_num=open_uart_dev(uart_device_radio.uart_name);

    set_uart_opt( uart_device_radio.uart_name, \
							  uart_device_radio.baudrate,\
							  uart_device_radio.databits,\
							  uart_device_radio.parity,\
							  uart_device_radio.stopbits);
    /*
     * 下面是创建接收线程，但是我觉得暂时不用这种方式了，
     * 用传统的定时读取不堵塞地读取确定的几个字节的方式
     */
//    uart_device_radio.ptr_fun=read_radio_data;
//    create_uart_pthread(&uart_device_radio);

    return 0;
}

//数据包中len是用len_byte_num个字节表示的，本协议用unsigned short表示的，是2个字节
#define LEN_BYTE_NUM 2
//命令包长度 实时数据包长度, 命令包和实时数据包长度都是固定的76个字节，航点包不限制长度
#define GCS2AP_CMD_REAL 76

#define RADIO_RECV_HEAD1           0
#define RADIO_RECV_HEAD2           1
#define RADIO_RECV_LEN                2
#define RADIO_RECV_TYPE               3
#define RADIO_RECV_DATA              4
#define RADIO_RECV_CHECKSUM   5
#define RADIO_RECV_WP                 6

static int radio_recv_state = 0;
int decode_radio_data(unsigned char *buf, unsigned int len)
{
	static unsigned char _buffer[RADIO_RECV_BUF_SIZE];

	static unsigned char _pack_recv_len[4] = {0};
	static int _pack_recv_real_len = 0;//表示收到的包中的数据包长度len这个short型数据
	static unsigned char _pack_recv_buf[RADIO_RECV_BUF_SIZE];
	static int _pack_buf_len = 0;

	int _length;
	unsigned char c;

	int i=0;
	static int i_len=0;
	static unsigned int checksum = 0;

	memcpy(_buffer, buf, len);

	_length=len;

	for (i = 0; i<_length; i++)
	{
		c = _buffer[i];
		switch (radio_recv_state)
		{
		case RADIO_RECV_HEAD1:
			if (c == 0xaa)
			{
				radio_recv_state = RADIO_RECV_HEAD2;
				checksum += c;
			}
			break;
		case RADIO_RECV_HEAD2:
			if (c == 0x55)
			{
				radio_recv_state = RADIO_RECV_LEN;
				checksum += c;
			}
			else
			{
				radio_recv_state = RADIO_RECV_HEAD1;
				checksum = 0;
			}
			break;
		case RADIO_RECV_LEN:
			_pack_recv_len[i_len] = c;
			i_len++;
			checksum += c;
			if ( i_len >= LEN_BYTE_NUM)
			{
				_pack_recv_real_len = _pack_recv_len[1]*pow(2,4)+_pack_recv_len[0];
				//printf("udp收到的有效数据长度为=%d\n",_pack_recv_real_len);
				radio_recv_state = RADIO_RECV_DATA;
				i_len=0;
			}
			else
			{
				radio_recv_state = RADIO_RECV_LEN;
			}
			_pack_buf_len = 4;//0xaa 0x55 len_low len_high 共4个字节
			break;
		case RADIO_RECV_DATA:
			_pack_recv_buf[0] = 0xaa;
			_pack_recv_buf[1] = 0x55;
			_pack_recv_buf[2] = _pack_recv_len[0];
			_pack_recv_buf[3] = _pack_recv_len[1];
			_pack_recv_buf[_pack_buf_len] = c;
			_pack_buf_len++;
			checksum += c;
			if (_pack_buf_len >= _pack_recv_real_len)
			{
				if(_pack_recv_real_len == GCS2AP_CMD_REAL )
				{
					//收到的是命令包
					radio_recv_state =  RADIO_RECV_CHECKSUM;
				}
				else if( _pack_recv_real_len >= 12)
				{
					//收到的是航点包
					radio_recv_state =  RADIO_RECV_WP;
				}
			}
			break;
		case  RADIO_RECV_CHECKSUM:

			unsigned int checksum_low;
			unsigned int checksum_high;
			unsigned int checksum_temp;//暂时默认校验和发过来的先是低字节
			checksum_low = _pack_recv_buf[_pack_buf_len-2];
			checksum_high = _pack_recv_buf[_pack_buf_len-1];
			checksum_temp = checksum_high * pow(2,4)  + checksum_low;

			if(checksum == checksum_temp)
			{
				radio_recv_state = 0;
				/*
				 * 收到了命令包，准备解析命令包数据
				 */
				global_bool_boatpilot.bool_get_gcs2ap_cmd = TRUE;
				//memcpy(&gcs2ap_cmd_udp, _pack_recv_buf, _pack_recv_real_len);
			}
			else
			{
				//校验和错误，重新接收数据
				radio_recv_state = 0;
			}
			break;
		case RADIO_RECV_WP:
			/*
			 * 收到了航点数据，准备解析航点数据
			 */
			global_bool_boatpilot.bool_get_gcs2ap_waypoint = TRUE;
			memcpy(wp_data, &_pack_recv_buf[12], _pack_recv_real_len - 12);
			int wp_num;
			wp_num = (_pack_recv_real_len - 12)/sizeof(WAY_POINT);
			printf("decode_radio_data    :    wp_num = %ld \n",wp_num);
			global_bool_boatpilot.wp_total_num = wp_num;

			break;
		}
	}

	return 0;
}

int read_radio_data()
{
	int read_len;
	static char _buffer[RADIO_RECV_BUF_SIZE];
	int time_out_us = 200;//最长等待200ms

	uart_device_radio.uart_name = UART_RADIO;
	read_len = read_uart_data(uart_device_radio.uart_name, _buffer, time_out_us, sizeof(_buffer));

	if(read_len > 0)
	decode_radio_data((unsigned char*)_buffer, read_len);

	return 0;
}

int send_radio_data(unsigned char *buf, unsigned int len)
{
	uart_device_radio.uart_name = UART_RADIO;
	send_uart_data(uart_device_radio.uart_name, (char *)buf, len);

	return 0;
}

int radio_uart_close()
{
    uart_device_radio.uart_name = UART_RADIO;
    close_uart_dev(uart_device_radio.uart_name);

	return 0;
}
