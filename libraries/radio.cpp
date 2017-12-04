/*
 * radio.c
 *
 *  Created on: 2016年5月9日
 *      Author: wangbo
 */

#include <stdio.h>
#include <string.h>
/*转换int或者short的字节顺序，该程序arm平台为大端模式，地面站x86架构为小端模式*/
//__bswap_32()
#include <byteswap.h>
/*open() close()所需头文件*/
#include <unistd.h>

#include "global.h"
#include "uart.h"
#include "boatlink.h"
#include "utility.h"

#include "radio.h"

#define RADIO_RECV_BUF_LEN 512

static char send_buf_test[] = { "test" };
static struct T_UART_DEVICE uart_device_radio;

int radio_uart_init()
{
    uart_device_radio.uart_name=UART_RADIO;

    uart_device_radio.baudrate=UART_RADIO_BAUD;
    uart_device_radio.databits=UART_RADIO_DATABITS;
    uart_device_radio.parity=UART_RADIO_PARITY;
    uart_device_radio.stopbits=UART_RADIO_STOPBITS;

    uart_device_radio.uart_num=open_uart_dev(uart_device_radio.uart_name);

    uart_device_radio.ptr_fun=read_radio_data;

    set_uart_opt( uart_device_radio.uart_name, \
                  uart_device_radio.baudrate,\
                  uart_device_radio.databits,\
                  uart_device_radio.parity,\
                  uart_device_radio.stopbits);

    create_uart_pthread(&uart_device_radio);

    return 0;
}

#define RADIO_RECV_HEAD1  0
#define RADIO_RECV_HEAD2  1
#define RADIO_RECV_LEN  2
#define RADIO_RECV_CNT  3
#define RADIO_RECV_SYSID 4
#define RADIO_RECV_TYPE 5
#define RADIO_RECV_COM_LINK 6
#define RADIO_RECV_ACK_REQ 7
#define RADIO_RECV_DATA 8

#define RADIO_RECV_CHECKSUM0 9
#define RADIO_RECV_CHECKSUM1 10

static int radio_recv_state = 0;
int read_radio_data(unsigned char *buf, unsigned int len)
{
    /* 直接把读取到的数据拷贝到_buffer数组中 */
	static unsigned char _buffer[RADIO_RECV_BUF_LEN];

	/* 这个帧头的8个字节，都保存下来，放在_frame_received里面 */
	//head1=0xaa;
	//head2=0x55;
	static int _pack_recv_len = 0;
	static int _pack_recv_cnt = 0;
	static unsigned char _sysid;
	static unsigned char _pack_recv_type;
	static unsigned char _pack_recv_com_link;
    static unsigned char _pack_recv_ack_req;

    /* 这个是帧尾的校验和 其实是2个字节 但是我们现在只用1个字节 */
	static unsigned char _checksum = 0;

	int i = 0;
	static unsigned char c;
	static unsigned char valid_len=5;

	static unsigned char _frame_received_cnt;
	static unsigned char _frame_received[RADIO_RECV_BUF_LEN];//存放一帧的数据，从帧头到帧尾校验和都包括，完整的一帧
	static unsigned char _frame_checksum_len=2;

	/*
	 * 判断是否持续在接收
	 */
	if(len > valid_len)
	{
	    global_bool_boatpilot.radio_wait_time=0;
	}

	/*
	 * len 表示通过电台读到的数据字节个数
	 * 判断 如果电台没有数据则转换到北斗接收
	 * 因为帧头是8个字节，只有每次接收数据大于等于8个字节，我们才认为接收到有效数据，否则认为数据丢失
	 */
	if(len>=8)
	{
	    global_bool_boatpilot.radio_get_data_previous_time_s=clock_gettime_s();
	}
	else
	{
	    global_bool_boatpilot.radio_lose_data_time_s=clock_gettime_s();
	}

	/* 当电台丢失数据超过10秒，我们就切换到北斗 */
	if((global_bool_boatpilot.radio_lose_data_time_s-global_bool_boatpilot.radio_get_data_previous_time_s)>10)
	{
	    //global_bool_boatpilot.bool_gcs2ap_beidou=1;
	    //return 0;
	}
	else
	{
	    //global_bool_boatpilot.bool_gcs2ap_beidou=0;
	}

	memcpy(_buffer, buf, len);
#if 1
	printf("radio data buf=\n");
	for(i=0;i<len;i++)
	{
		printf("%0x ",buf[i]);
	}
	printf("\n");
#endif

	for (i = 0; i<len; i++)
	{
		c = _buffer[i];
		switch (radio_recv_state)
		{
		case RADIO_RECV_HEAD1:
			if (c == 0xaa)
			{
				radio_recv_state = RADIO_RECV_HEAD2;
				_checksum = c;
			}
			break;
		case RADIO_RECV_HEAD2:
			if (c == 0x55)
			{
				radio_recv_state = RADIO_RECV_LEN;
				_checksum += c;
			}
			else
			{
			    _checksum = 0;
				radio_recv_state = RADIO_RECV_HEAD1;
				_frame_received_cnt=0;
			}
			break;
		case RADIO_RECV_LEN:
			_checksum += c;
			_pack_recv_len=c;

			_frame_received[0]=0xaa;
			_frame_received[1]=0x55;
			_frame_received[2]=c;//20170728这个是统一为76个字节了，可以打印出来看
			_frame_received_cnt=3;

			radio_recv_state = RADIO_RECV_CNT;
			break;
		case RADIO_RECV_CNT:
		    _pack_recv_cnt = c;
            _checksum += c;

            _frame_received[_frame_received_cnt]=c;
            _frame_received_cnt++;

			radio_recv_state = RADIO_RECV_SYSID;
			break;
		case RADIO_RECV_SYSID:
			_sysid = c;
			if(0!=_sysid)
			{
				_checksum += c;

                _frame_received[_frame_received_cnt]=c;
                _frame_received_cnt++;

                radio_recv_state = RADIO_RECV_TYPE;
			}
			break;
		case RADIO_RECV_TYPE:
			_pack_recv_type = c;
			_checksum += c;

			_frame_received[_frame_received_cnt]=c;
            _frame_received_cnt++;

            radio_recv_state = RADIO_RECV_COM_LINK;
			break;
		case RADIO_RECV_COM_LINK:
		    _pack_recv_com_link = c;
            _checksum += c;

            _frame_received[_frame_received_cnt]=c;
            _frame_received_cnt++;

            radio_recv_state = RADIO_RECV_ACK_REQ;
		    break;
		case RADIO_RECV_ACK_REQ:
		    _pack_recv_ack_req = c;
            _checksum += c;

            //_frame_received[7]=c;
            _frame_received[_frame_received_cnt]=c;
            _frame_received_cnt++;

            radio_recv_state = RADIO_RECV_DATA;
		    break;
		case RADIO_RECV_DATA:
			_checksum += c;

			_frame_received[_frame_received_cnt]=c;
            _frame_received_cnt++;

			if (_frame_received_cnt >= _pack_recv_len-_frame_checksum_len)
			{
				//printf("checksum=%0x \n",_checksum);
                radio_recv_state = RADIO_RECV_CHECKSUM0;
			}
			break;
		case RADIO_RECV_CHECKSUM0:
		    _checksum += c;

		    _frame_received[_frame_received_cnt]=c;
            _frame_received_cnt++;

            radio_recv_state = RADIO_RECV_CHECKSUM1;
		    break;
		case RADIO_RECV_CHECKSUM1:
			if (_checksum == c)
			{
			    _frame_received[_frame_received_cnt]=c;//如果正常的话，这里的_frame_received_cnt应该等于76，包含帧头和帧尾
			    _frame_received_cnt++;
			    //printf("_frame_received_cnt=%d\n",_frame_received_cnt);

			    global_bool_boatpilot.radio_recv_packet_cnt = _pack_recv_cnt;
			    if(global_bool_boatpilot.radio_recv_packet_cnt_previous!=global_bool_boatpilot.radio_recv_packet_cnt)
			    {
			        switch (_pack_recv_type)
                    {
                    case COMMAND_GCS2AP_WAYPOINT:
                        if (_frame_received_cnt == sizeof(gcs_ap_wp))
                        {
                            printf("正确接收到GCS_AP_WP数据包，且数据包数据长度与航点结构长度相同\n");
                            memcpy(&gcs_ap_wp, _frame_received, _frame_received_cnt);
                            global_bool_boatpilot.bool_get_gcs2ap_waypoint = TRUE;
                            //global_bool_boatpilot.bool_gcs2ap_beidou=_pack_recv_com_link;
                            //global_bool_boatpilot.send_ap2gcs_wp_req=_pack_recv_ack_req;
                            //global_bool_boatpilot.gcs2ap_wp_cnt=_pack_recv_cnt;
                            //global_bool_boatpilot.ap2gcs_wp_cnt=_pack_recv_cnt;
                        }
                        _checksum = 0;
                        radio_recv_state = 0;
                        _frame_received_cnt=0;
                        break;
                    case COMMAND_GCS2AP_CMD:
                        if (_frame_received_cnt == sizeof(gcs2ap_cmd))
                        {
                            //printf("正确接收到GCS2AP_CMD数据包，且数据包数据长度与命令结构长度相同\n");//20170410已测试
                            memcpy(&gcs2ap_cmd, _frame_received, _frame_received_cnt);
                            global_bool_boatpilot.bool_get_gcs2ap_cmd = TRUE;
                            //global_bool_boatpilot.bool_gcs2ap_beidou=_pack_recv_com_link;//这次先不判断北斗，北斗和电台同时发送实时数据，同时接收并解析命令包
                            //global_bool_boatpilot.send_ap2gcs_cmd_req=_pack_recv_ack_req;
                            //global_bool_boatpilot.gcs2ap_cmd_cnt=_pack_recv_cnt;
                            global_bool_boatpilot.ap2gcs_cmd_cnt=_pack_recv_cnt;
                        }
                        _checksum = 0;
                        radio_recv_state = 0;
                        _frame_received_cnt=0;
                        break;

                    case COMMAND_AP2GCS_TEST:
                        send_uart_data(UART_RADIO, send_buf_test, 4);
                        _checksum = 0;
                        radio_recv_state = 0;
                        _frame_received_cnt=0;
                        break;
                    default:
                        break;
                    }
			        global_bool_boatpilot.radio_recv_packet_cnt_previous=global_bool_boatpilot.radio_recv_packet_cnt;
			    }
			}
			else
			{
			    printf("电台--数据校验和错误，校验和是加和\n");
			    _checksum = 0;
				radio_recv_state = 0;
				_frame_received_cnt=0;
			}
		}
	}

	return 0;
}

int send_radio_data(unsigned char *buf, unsigned int len)
{
	global_bool_boatpilot.radio_need_to_send_time=clock_gettime_s();
	global_bool_boatpilot.radio_send_delta_time=global_bool_boatpilot.radio_need_to_send_time-global_bool_boatpilot.radio_send_time;
	//printf("电台发送之间的间隔=%f\n",global_bool_boatpilot.radio_send_delta_time);//已测试20170413

	send_uart_data(UART_RADIO, (char *)buf, len);

	global_bool_boatpilot.radio_send_time=clock_gettime_s();

	return 0;
}

int radio_uart_close()
{
    uart_device_radio.uart_name=UART_RADIO;
    close_uart_dev(uart_device_radio.uart_name);

	return 0;
}
