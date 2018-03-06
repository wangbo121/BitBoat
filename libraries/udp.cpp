/*
 * udp.c
 *
 *  Created on: 2017-8-9
 *      Author: wangbo
 */

#include "global.h"
#if 1

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


#include "udp.h"

#include "global.h"


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

int open_socket_udp_dev(int *ptr_fd_socket, char* ip, unsigned int port)
{

    /*
     * 指定了发送目标的端口为port_sendto，也就是如果要发送给服务器，必须知道服务器哪个端口是会处理我的数据
     * 指定了我这个程序的接收端口，既然服务器的端口是比如6789，那么我就也把我的接收端口设置为6789呗，从而统一个数据接收和发送
     * 整体来说，发送时指定对方ip地址，接收来自任意ip的数据
     * 发送时不需要绑定端口，系统会自动分配，当然了其实客户端也可以固定端口发送的
     * 接收时绑定端口，作为服务器肯定是固定端口接收
     */

	int fd_socket;
	struct sockaddr_in socket_udp_addr;//服务器用于发送的socket
    int sockaddr_size;

    /*设置发送目标的ip地址和发送目标的端口*/
    sockaddr_size = sizeof(struct sockaddr_in);
    bzero((char*)&socket_udp_addr, sockaddr_size);
    socket_udp_addr.sin_family = AF_INET;

    //socket_udp_addr.sin_addr.s_addr = inet_addr(ip);//这个ip地址肯定是创建socket本机的某个ip地址，比如我的电脑就有2个网卡，2个ip地址，再加上127.0.0.1就是3个地址
    inet_pton(AF_INET, ip, &socket_udp_addr.sin_addr);
    //udp_sendto_addr.sin_addr.s_addr = INADDR_ANY;//INADDR_ANY是任意地址

    socket_udp_addr.sin_port = htons(port);
//    printf("port_sendto=%d\n",port);

    /*建立“发送”套接字*/
    fd_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_sock_send == -1)
    {
        printf("udp %s create socket failed",ip);
        close(fd_socket );
        return 0;
    }
    else
    {
        printf("udp %s ini ok!\n",ip);
    }

    /*
     * 绑定端口
     */
	if(-1 == (bind(fd_socket,(struct sockaddr*)&socket_udp_addr,sizeof(struct sockaddr_in))))
	{
		perror("Server Bind Failed:");
		//exit(1);
	}
	else
	{
		printf("bind success\n");
		*ptr_fd_socket = fd_socket;
	}

	return 0;
}

int send_socket_udp_data(int fd_socket, unsigned char *buf, unsigned int len, char *target_ip, unsigned int target_port)
{
	struct sockaddr_in udp_sendto_addr;//服务器用于发送的socket

    int sockaddr_size;

    sockaddr_size = sizeof(struct sockaddr_in);
    bzero((char*)&udp_sendto_addr, sockaddr_size);

    udp_sendto_addr.sin_family = AF_INET;//这个socket的协议类型
    //udp_sendto_addr.sin_addr.s_addr = inet_addr(ip_sendto);
    //inet_pton(AF_INET, "10.108.16.163", &udp_sendto_addr.sin_addr);
    inet_pton(AF_INET, target_ip, &udp_sendto_addr.sin_addr);

    udp_sendto_addr.sin_port = htons(target_port);
//    printf("port_sendto=%d\n",target_port);

	int send_len;
	unsigned char send_buf[2000];

	memcpy(send_buf, buf, len);
	send_len=len;

	//printf("send len=%d\n",send_len);

	//sendto(fd_sock_send, send_buf, send_len, 0, (struct sockaddr *)&udp_sendto_addr, sizeof(struct sockaddr_in));
	sendto(fd_socket, send_buf, send_len, 0, (struct sockaddr *)&udp_sendto_addr, sizeof(struct sockaddr_in));

	return 0;
}

int read_socket_udp_data(int fd_socket, unsigned char *buf, unsigned int len)
{

	return 0;
}



int fd_sock_send;
int fd_sock_recv;

struct T_UDP_DEVICE udp_device;

static struct sockaddr_in udp_mysendto_addr;//服务器用于接收的socket

static struct sockaddr_in udp_myrecv_addr;//服务器用于接收的socket
//static struct sockaddr_in udp_sendto_addr;//服务器用于发送的socket
struct sockaddr_in udp_sendto_addr;//服务器用于发送的socket
static struct sockaddr_in client_addr;//服务器用来保存客户端的发送socket，把客户端的socket属性保存在client_addr

int open_udp_dev(char* ip_sendto, unsigned int port_sendto, unsigned int port_myrecv)
{
    /*
     * 指定了发送目标的端口为port_sendto，也就是如果要发送给服务器，必须知道服务器哪个端口是会处理我的数据
     * 指定了我这个程序的接收端口，既然服务器的端口是比如6789，那么我就也把我的接收端口设置为6789呗，从而统一个数据接收和发送
     * 整体来说，发送时指定对方ip地址，接收来自任意ip的数据
     * 发送时不需要绑定端口，系统会自动分配，当然了其实客户端也可以固定端口发送的
     * 接收时绑定端口，作为服务器肯定是固定端口接收
     */

    int sockaddr_size;
    /*设置发送目标的ip地址和发送目标的端口*/
    sockaddr_size = sizeof(struct sockaddr_in);
    bzero((char*)&udp_sendto_addr, sockaddr_size);
    udp_sendto_addr.sin_family = AF_INET;
    //udp_sendto_addr.sin_addr.s_addr = inet_addr(ip_sendto);
    //inet_pton(AF_INET, "10.108.16.163", &udp_sendto_addr.sin_addr);
    inet_pton(AF_INET, ip_sendto, &udp_sendto_addr.sin_addr);



    //udp_sendto_addr.sin_addr.s_addr = inet_addr("10.108.16.163");
    //udp_sendto_addr.sin_addr.s_addr = INADDR_ANY;
    udp_sendto_addr.sin_port = htons(port_sendto);
    printf("port_sendto=%d\n",port_sendto);
    printf("udp_sendto_addr.sin_port=%d\n",udp_sendto_addr.sin_port);

    /*建立“发送”套接字*/
    fd_sock_send = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_sock_send == -1)
    {
        perror("udp send create socket failed");
        close(fd_sock_send);
        return 0;
    }
    else
    {
        printf("udp send ini ok!\n");
    }

    /*
     * 绑定发送的端口
     */
#if 1
	sockaddr_size = sizeof(struct sockaddr_in);
	bzero((char*)&udp_mysendto_addr, sockaddr_size);
	udp_mysendto_addr.sin_family = AF_INET;
	udp_mysendto_addr.sin_addr.s_addr = inet_addr("10.108.16.163");
	//udp_mysendto_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	//udp_mysendto_addr.sin_port = htons(49000);
	udp_mysendto_addr.sin_port = htons(49000);//发送的端口跟目标ip地址的端口一致，我从49000发送，另一边也从49000接收

#else
    /* 绑定套接口 */
     //if(-1 == (bind(fd_sock_send,(struct sockaddr*)&udp_sendto_addr,sizeof(struct sockaddr_in))))
    if(-1 == (bind(fd_sock_send,(struct sockaddr*)&udp_mysendto_addr,sizeof(struct sockaddr_in))))
     {
      perror("Server Bind Failed:");
      //exit(1);
     }
     else
     {
    	 printf("bind success\n");

     }
#endif




    /*设置本机的接收ip地址和端口，但是接收的ip不指定，因为任何的ip都有可能给我发数据呀*/
    sockaddr_size = sizeof(struct sockaddr_in);
    bzero((char*)&udp_myrecv_addr, sockaddr_size);
    udp_myrecv_addr.sin_family = AF_INET;
    udp_myrecv_addr.sin_addr.s_addr = INADDR_ANY;//任意地址
    //udp_myrecv_addr.sin_addr.s_addr = inet_addr("10.108.16.163");;//任意地址

    //udp_myrecv_addr.sin_port = htons(port_myrecv);
    udp_myrecv_addr.sin_port = htons(49005);
    /*建立“接收”套接字*/
    fd_sock_recv = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_sock_recv == -1)
    {
        perror("udp recv create socket failed");
        close(fd_sock_recv);
        return 0;
    }
    else
    {

    }

    /*
     * 建立socket获取fd_sock文件描述符后，需要把这个文件描述符与某一个soucket绑定
     * 因为socket函数只是建立了一个通用的针对某一地址族的文件描述符，但是该socket的属性还需要设置并且绑定
     */
    if (bind(fd_sock_recv, (struct sockaddr *)&udp_myrecv_addr, sizeof(struct sockaddr_in)) == -1)
    {
        perror("udp recv bind err");
        close(fd_sock_recv);
        return 0;
    }
    else
    {
        printf("udp recv ini ok!\n");
    }

    int ret=0;
    udp_device.ptr_fun=read_udp_data;
    /*udp_recvbuf_and_process函数中调用了fd_sock_recv来确定从哪个socket获取数据*/
    ret = pthread_create (&udp_device.fd,            //线程标识符指针
                          NULL,                            //默认属性
                          udp_recvbuf_and_process,//运行函数
                          &udp_device);                 //运行函数的参数
    if (0 != ret)
    {
       perror ("pthread create error\n");
    }

    return 0;
}

#define UDP_RCV_BUF_SIZE 2000
#define UDP_PACKET_HEAD 0xa55a5aa5
#define UDP_PACKET_END 0x12345678

int send_udp_data(unsigned char *buf, unsigned int len)
{
    int m_tail;
    int m_len;
    unsigned char m_sendbuf[UDP_RCV_BUF_SIZE];

    static unsigned int m_packcnt = 0;

    /*
     * 帧尾0x12345678 4个字节
     * 没有帧尾
     */
    memcpy(m_sendbuf, buf, len);
    m_len=len;

#if 0
    m_tail = htonl(UDP_PACKET_END);
    memcpy(&m_sendbuf[len], &m_tail, 4);
    m_len = 4 + len;
#endif

#if 0
    int i=0;
    for(i=0;i<m_len;i++)
    {
       printf("%02x ",m_sendbuf[i]);
    }
    printf("\n");
#endif
    printf("send len=%d\n",m_len);

    sendto(fd_sock_send, m_sendbuf, m_len, 0, (struct sockaddr *)&udp_sendto_addr, sizeof(struct sockaddr_in));

    return 0;
}


//#define UDP_PACKET_HEAD 0xa55a5aa5
#define UDP_RECV_HEAD1  0
#define UDP_RECV_HEAD2  1
#define UDP_RECV_HEAD3  2
#define UDP_RECV_HEAD4  3
#define UDP_RECV_CNT  4
#define UDP_RECV_LEN  5
#define UDP_RECV_DATA 6
#define UDP_RECV_CHECKSUM 7

static int udp_recv_state=0;

int read_udp_data(unsigned char *buf, unsigned int len)
{
    static unsigned char _buffer[UDP_RCV_BUF_SIZE];

    static unsigned char _pack_recv_cnt[4] = {0};
    static int _pack_recv_real_cnt=0;
    static unsigned char _pack_recv_len[4] = {0};
    static int _pack_recv_real_len = 0;
    static unsigned char _pack_recv_buf[UDP_RCV_BUF_SIZE];
    static int _pack_buf_len = 0;

    int _length;
    unsigned char c;

    int i=0;
    static int i_cnt=0;
    static int i_len=0;

#if 0
    printf("wangbo 20170809 udp len=%d,udp data buf=\n",len);
    for(i=0;i<len;i++)
    {
        printf("%x",buf[i]);
    }
    printf("\n");

    printf("sizeof(fg2ap)=%d\n",sizeof(fg2ap));
    printf("sizeof(uint64_t)=%d\n",sizeof(uint64_t));
#endif

#if 1

    /*
    memcpy(&fg2ap,buf,sizeof(fg2ap));
    fg2ap.pitch_deg= ntoh_double(fg2ap.pitch_deg);
    fg2ap.heading_deg= ntoh_double(fg2ap.heading_deg);
    fg2ap.longitude_deg= ntoh_double(fg2ap.longitude_deg);
    fg2ap.latitude_deg= ntoh_double(fg2ap.latitude_deg);
*/


	//fg2ap.pitch_deg=(double)(*((uint64_t*)(&fg2ap.pitch_deg)));
	//fg2ap.heading_deg=(double)(*((uint64_t*)(&fg2ap.heading_deg)));
	//fg2ap.longitude_deg=(double)(*(uint64_t*)&fg2ap.longitude_deg);
	//fg2ap.latitude_deg=(double)(*(uint64_t*)&fg2ap.latitude_deg);
	//fg2ap.pitch_deg=(double)__bswap_64(*(uint64_t*)&fg2ap.pitch_deg);
	//fg2ap.heading_deg=(double)__bswap_64(*(uint64_t*)&fg2ap.heading_deg);
	//fg2ap.longitude_deg=(double)__bswap_64(*(uint64_t*)&fg2ap.longitude_deg);
	//fg2ap.latitude_deg=(double)__bswap_64(*(uint64_t*)&fg2ap.latitude_deg);



    //printf("俯仰=%f，航向=%f\n",fg2ap.pitch_deg,fg2ap.heading_deg);
   // printf("纬度=%f，经度=%f\n",fg2ap.latitude_deg,fg2ap.longitude_deg);
#else
    memcpy(&ap2fg_recv,buf,sizeof(ap2fg_recv));
    ap2fg_recv.throttle0= ntoh_double(ap2fg_recv.throttle0);

    printf("ap2fg_recv.throttle0=%f\n",ap2fg_recv.throttle0);

#endif
    return 0;

    memcpy(_buffer, buf, len);

    _length=len;

    for (i = 0; i<_length; i++)
    {
        c = _buffer[i];
        switch (udp_recv_state)
        {
        case UDP_RECV_HEAD1:
            if (c == 0xa5)
            {
                udp_recv_state = UDP_RECV_HEAD2;
            }

            break;
        case UDP_RECV_HEAD2:
            if (c == 0x5a)
            {
                udp_recv_state = UDP_RECV_HEAD3;
            }
            else
            {
                udp_recv_state = UDP_RECV_HEAD1;
            }

            break;
        case UDP_RECV_HEAD3:
           if (c == 0x5a)
           {
               udp_recv_state = UDP_RECV_HEAD4;
           }
           else
           {
               udp_recv_state = UDP_RECV_HEAD1;
           }

           break;
        case UDP_RECV_HEAD4:
           if (c == 0xa5)
           {
               udp_recv_state = UDP_RECV_CNT;
           }
           else
           {
               udp_recv_state = UDP_RECV_HEAD1;
           }

           break;
        case UDP_RECV_CNT:
            _pack_recv_cnt[i_cnt] = c;
            i_cnt++;

            if ( i_cnt>= 4)
            {
                //低位在前
                _pack_recv_real_cnt=_pack_recv_cnt[3]*pow(2,16)+_pack_recv_cnt[2]*pow(2,8)+_pack_recv_cnt[1]*pow(2,4)+_pack_recv_cnt[0];
                udp_recv_state = UDP_RECV_LEN;
                i_cnt=0;
            }
            else
            {
                udp_recv_state = UDP_RECV_CNT;
            }
            _pack_buf_len = 0;

            break;
        case UDP_RECV_LEN:
            _pack_recv_len[i_len] = c;
            i_len++;
            if ( i_len>= 4)
            {
                _pack_recv_real_len=_pack_recv_len[3]*pow(2,16)+_pack_recv_len[2]*pow(2,8)+_pack_recv_len[1]*pow(2,4)+_pack_recv_len[0];
                //printf("udp收到的有效数据长度为=%d\n",_pack_recv_real_len);
                udp_recv_state = UDP_RECV_DATA;
                i_len=0;
            }
            else
            {
                udp_recv_state = UDP_RECV_LEN;
            }
            _pack_buf_len = 0;

            break;
        case UDP_RECV_DATA:
            _pack_recv_buf[_pack_buf_len] = c;
            _pack_buf_len++;
            if (_pack_buf_len >= _pack_recv_real_len)
            {
                udp_recv_state = UDP_RECV_CHECKSUM;
            }

            break;
        case UDP_RECV_CHECKSUM:
            //if (_checksum == c)
            if (1)
            {

                udp_recv_state = 0;

                /*
                 * 收到udp传来的副控数据后，根据ack来决定，m2s的数据
                 * 先放在这里测试，最后放在boatpilot.c文件中，接收线程中不要太多的判断
                 */

            }
            else
            {
                udp_recv_state = 0;
            }
            break;
        }
    }

    return 0;
}

void *udp_recvbuf_and_process(void * ptr_udp_device)
{
    char buf[UDP_RCV_BUF_SIZE] = { 0 };
    unsigned int read_len;

    struct T_UDP_DEVICE *ptr_udp;
    ptr_udp=(struct T_UDP_DEVICE *)ptr_udp_device;

    unsigned int client_addr_len=0;
    client_addr_len=sizeof(struct sockaddr);

    while(1)
    {
        if(-1!=(read_len=recvfrom(fd_sock_recv, buf, 2000, 0, (struct sockaddr *)&client_addr, &client_addr_len)))
        {
#if 0
            printf("read_len=%d\n",read_len);
            buf[read_len]='\0';
            printf("%s\n",buf);
#endif
            if(read_len>0)
            {
                ptr_udp->ptr_fun((unsigned char*)buf,read_len);
            }
        }
    }

    pthread_exit(NULL);


}
#endif
