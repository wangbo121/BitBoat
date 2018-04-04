/*
 * udp.c
 *
 *  Created on: 2017-8-9
 *      Author: wangbo
 */

#include<stdio.h>
#include <stdint.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<sys/select.h>
#include<sys/ioctl.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<fcntl.h>
#include<unistd.h>
#include<signal.h>
#include<pthread.h>
#include<unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<errno.h>
#include<netdb.h>
#include<stdarg.h>
#include<string.h>
#include<math.h>
/*转换int或者short的字节顺序，该程序arm平台为大端模式，地面站x86架构为小端模式*/
#include <byteswap.h>

#include "global.h"
#include "boatlink_udp.h"
#include "udp.h"

uint64_t htonll(uint64_t n) {
return (((uint64_t)htonl(n)) << 32) | htonl(n >> 32);
}
uint64_t ntohll(uint64_t n) {
return (((uint64_t)ntohl(n)) << 32) | ntohl(n >> 32);
}

double ntoh_double(double net_double) {
uint64_t host_int64;
host_int64 = ntohll(*((uint64_t *) &net_double));
return *((double *) &host_int64);
}

double hton_double(double host_double) {
uint64_t net_int64;
net_int64 = htonll(*((uint64_t *) &host_double));

return *((double *) &net_int64);
}

float ntoh_float(float net_float) {
	uint32_t host_int32;
	host_int32 = ntohl(*((uint32_t *) &net_float));

	return *((double *) &host_int32);
}
float hton_float(float host_float) {
	uint32_t net_int32;
	net_int32 = htonl(*((uint32_t *) &host_float));

	return *((double *) &net_int32);
}

double htond (double x)
{
    int * p = (int*)&x;
    int tmp = p[0];
    p[0] = htonl(p[1]);
    p[1] = htonl(tmp);

    return x;
}

float htonf (float x)
{
    int * p = (int *)&x;
    *p = htonl(*p);
    return x;
}

/*
 * 按照udp通信的要求
 * 每个socket按道理都应该绑定ip和端口的，
 * 但是当用某个socket发送的时候，这个socket不需要绑定端口，系统会自动分配一个端口，
 * 但是我们为了确定这个发送的socket到底用的哪个ip和端口，给他绑定一下
 * 作为接收端，当你调用bind()函数绑定IP时使用INADDR_ANY，表明接收来自任意IP、任意网卡的发给指定端口的数据。一个电脑可能有多个网卡，从而有多个ip
 * 作为发送端，当用调用bind()函数绑定IP时使用INADDR_ANY，表明使用网卡号最低的网卡进行发送数据，也就是UDP数据广播。
 */
int open_socket_udp_dev(int *ptr_fd_socket, char* ip, unsigned int port)
{
	int fd_socket;
	struct sockaddr_in socket_udp_addr;//fd_socket是socket的描述符，socket_udp_addr表示socket的地址属性，需要绑定fd_socket文件描述符
    int sockaddr_size;

    sockaddr_size = sizeof(struct sockaddr_in);
    bzero((char*)&socket_udp_addr, sockaddr_size);
    socket_udp_addr.sin_family = AF_INET;//地址族，等价于协议族，AF_INET表示ipv4，AF_INET6表示ipv6

    /*
     * 某一个应用创建socket生成fd_socket，这个socket需要绑定ip把数据通过ip对应的网卡发出去
     * 比如我的电脑就有2个网卡，2个ip地址，再加上127.0.0.1就是3个地址
     * INADDR_ANY是任意地址，作为接收端，当你调用bind()函数绑定IP时使用INADDR_ANY，表明接收来自任意IP、任意网卡的发给指定端口的数据。一个电脑可能有多个网卡，从而有多个ip
     *                                                 作为发送端，当用调用bind()函数绑定IP时使用INADDR_ANY，表明使用网卡号最低的网卡进行发送数据，也就是UDP数据广播。
     */
    //udp_sendto_addr.sin_addr.s_addr = INADDR_ANY;//
    inet_pton(AF_INET, ip, &socket_udp_addr.sin_addr);

    socket_udp_addr.sin_port = htons(port);

    /*
     * 利用socket函数创建socket，生成文件描述符fd_socket
     */
    if( (fd_socket = socket(AF_INET, SOCK_DGRAM, 0) ) < 0)
    {
    	printf("open_socket_udp_dev    :    udp %s create socket failed!!!\n",ip);
    }
    else
    {
        printf("open_socket_udp_dev    :    udp %s ini ok!!!\n",ip);
    }

    /*
     * 绑定端口
     * 作为接收端，当你调用bind()函数绑定IP时使用INADDR_ANY，表明接收来自任意IP、任意网卡的发给指定端口的数据。一个电脑可能有多个网卡，从而有多个ip
     * 作为发送端，当用调用bind()函数绑定IP时使用INADDR_ANY，表明使用网卡号最低的网卡进行发送数据，也就是UDP数据广播。
     */
	if(-1 == (bind(fd_socket,(struct sockaddr*)&socket_udp_addr,sizeof(struct sockaddr_in))))
	{
		perror("open_socket_udp_dev    :    bind failed!!!\n");
	}
	else
	{
		printf("open_socket_udp_dev    :    bind success!!!\n");
		*ptr_fd_socket = fd_socket;
	}

	return 0;
}

int send_socket_udp_data(int fd_socket, unsigned char *buf, unsigned int len, char *target_ip, unsigned int target_port)
{
	if( fd_socket < 0)
	{
		printf("send_socket_udp_data    :    fd_socket is invalid!!!\n");
		return 0;
	}

	struct sockaddr_in udp_sendto_addr;//想要把数据发送给对方，我们需要知道对方的socket对应的ip地址和端口
    int sockaddr_size;

    sockaddr_size = sizeof(struct sockaddr_in);
    bzero((char*)&udp_sendto_addr, sockaddr_size);

    /*
     * 3个要点，3句话，1协议族，2ip地址，3端口号
     */
    udp_sendto_addr.sin_family = AF_INET;
    //udp_sendto_addr.sin_addr.s_addr = INADDR_ANY;//如果对方的ip地址写成了INADDR_ANY，那就意味着我们是广播出去的，不是给特定ip发送数据
    inet_pton(AF_INET, target_ip, &udp_sendto_addr.sin_addr);
    udp_sendto_addr.sin_port = htons(target_port);

	int send_len;
	unsigned char send_buf[2000];

	memcpy(send_buf, buf, len);
	send_len=len;
	//printf("send len=%d\n",send_len);

	sendto(fd_socket, send_buf, send_len, 0, (struct sockaddr *)&udp_sendto_addr, sizeof(struct sockaddr_in));

	return 0;
}

//数据包中len是用len_byte_num个字节表示的，本协议用unsigned short表示的，是2个字节
#define LEN_BYTE_NUM 2
//命令包长度 实时数据包长度
#define GCS2AP_CMD_REAL 76

#define UDP_RECV_HEAD1           0
#define UDP_RECV_HEAD2           1
#define UDP_RECV_LEN                2
#define UDP_RECV_TYPE               3
#define UDP_RECV_DATA              4
#define UDP_RECV_CHECKSUM   5
#define UDP_RECV_WP                 6

static int udp_recv_state=0;
int decode_udp_data(char *buf, int len)
{
	static unsigned char _buffer[UDP_BUF_SIZE];

	static unsigned char _pack_recv_len[4] = {0};
	static int _pack_recv_real_len = 0;//表示收到的包中的数据包长度len这个short型数据
	static unsigned char _pack_recv_buf[UDP_BUF_SIZE];
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
		switch (udp_recv_state)
		{
		case UDP_RECV_HEAD1:
			if (c == 0xaa)
			{
				udp_recv_state = UDP_RECV_HEAD2;
				checksum += c;
			}
			break;
		case UDP_RECV_HEAD2:
			if (c == 0x55)
			{
				udp_recv_state = UDP_RECV_LEN;
				checksum += c;
			}
			else
			{
				udp_recv_state = UDP_RECV_HEAD1;
				checksum = 0;
			}
			break;
		case UDP_RECV_LEN:
			_pack_recv_len[i_len] = c;
			i_len++;
			checksum += c;
			if ( i_len >= LEN_BYTE_NUM)
			{
				_pack_recv_real_len = _pack_recv_len[1]*pow(2,4)+_pack_recv_len[0];
				//printf("udp收到的有效数据长度为=%d\n",_pack_recv_real_len);
				udp_recv_state = UDP_RECV_DATA;
				i_len=0;
			}
			else
			{
				udp_recv_state = UDP_RECV_LEN;
			}
			_pack_buf_len = 4;//0xaa 0x55 len_low len_high 共4个字节
			break;
		case UDP_RECV_DATA:
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
					udp_recv_state = UDP_RECV_CHECKSUM;
				}
				else if( _pack_recv_real_len >= 12)
				{
					//收到的是航点包
					udp_recv_state = UDP_RECV_WP;
				}
			}
			break;
		case UDP_RECV_CHECKSUM:

			unsigned int checksum_low;
			unsigned int checksum_high;
			unsigned int checksum_temp;//暂时默认校验和发过来的先是低字节
			checksum_low = _pack_recv_buf[_pack_buf_len-2];
			checksum_high = _pack_recv_buf[_pack_buf_len-1];
			checksum_temp = checksum_high * pow(2,4)  + checksum_low;

			if(checksum == checksum_temp)
			{
				udp_recv_state = 0;
				/*
				 * 收到了命令包，准备解析命令包数据
				 */
				global_bool_boatpilot.bool_get_gcs2ap_cmd = TRUE;
				memcpy(&gcs2ap_cmd_udp, _pack_recv_buf, _pack_recv_real_len);
			}
			else
			{
				//校验和错误，重新接收数据
				udp_recv_state = 0;
			}
			break;
		case UDP_RECV_WP:
			/*
			 * 收到了航点数据，准备解析航点数据
			 */
			global_bool_boatpilot.bool_get_gcs2ap_waypoint = TRUE;
			memcpy(wp_data, &_pack_recv_buf[12], _pack_recv_real_len - 12);
			int wp_num;
			wp_num = (_pack_recv_real_len - 12)/sizeof(WAY_POINT);
			printf("decode_udp_data    :    wp_num = %ld \n",wp_num);
			global_bool_boatpilot.wp_total_num = wp_num;

			break;
		}
	}

	return 0;
}

int read_socket_udp_data(int fd_socket)
{
	if( fd_socket < 0)
	{
		printf("read_socket_udp_data    :    fd_socket is invalid!!!\n");
		return 0;
	}

	struct timeval timeout;
	timeout.tv_sec = 0;//秒
	timeout.tv_usec = UDP_RECVFROM_BLOCK_TIME;//微秒  等待100ms吧
	if (setsockopt(fd_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
	{
		printf("read_socket_udp_data    :    setsockopt failed \n");
	}

	char recv_buf[256];
	int recv_len;

	struct sockaddr_in addr;
	unsigned int addr_len = sizeof(struct sockaddr_in);

	/*
	 * 参数len为可接收数据的最大长度
	 */
	//printf("read_socket_udp_data    :    ready to receive \n");
	recv_len = recvfrom(fd_socket, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr, &addr_len);

	if( recv_len > 0)
	{
		printf("read_socket_udp_data    :    recv_len = %d \n",recv_len);
		decode_udp_data(recv_buf, recv_len);
	}

	return 0;
}
