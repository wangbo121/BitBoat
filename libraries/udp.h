/*
 * udp.h
 *
 *  Created on: 2017-8-9
 *      Author: wangbo
 */

#ifndef UDP_H_
#define UDP_H_

#include <stdint.h>

/*
 * 为了进行转换 bsd socket提供了转换的函数 有下面四个
 * htons 把unsigned short类型从主机序转换到网络序
 *	htonl 把unsigned long类型从主机序转换到网络序
 * ntohs 把unsigned short类型从网络序转换到主机序
 *	ntohl 把unsigned long类型从网络序转换到主机序
 * 网络与主机字节转换函数:htons ntohs htonl ntohl (s 就是short l是long h是host n是network)
 * 其实htonl这些函数最后调用的还是__bswap
 */

#define UDP_BUF_SIZE 2048

#define IP_SEND_TO "127.0.0.1"
//#define IP_SEND_TO "10.108.17.250"
//#define IP_SEND_TO "10.108.17.234"
//#define IP_SEND_TO "10.108.17.235"

#define PORT_SENT_TO 49000
#define PORT_RECEIVE 49005

struct T_UDP_DEVICE
{
    unsigned long int fd;
    int (*ptr_fun)(unsigned char *buf,unsigned int len);
};

extern unsigned char udp_recvbuf[UDP_BUF_SIZE];

int open_udp_dev(char* ip_sendto, unsigned int port_sendto,unsigned int port_myrecv);
int read_udp_data(unsigned char *buf, unsigned int len);
int send_udp_data(unsigned char *buf, unsigned int len);
int close_udp_dev();

/*udp_recvbuf_and_process需要调用read函数来解析数据包获取实际数据*/
//void *udp_recvbuf_and_process(void * ptr_udp_device);

uint64_t htonll(uint64_t n) ;
uint64_t ntohll(uint64_t n) ;

double ntoh_double(double net_double) ;
double hton_double(double host_double) ;

float ntoh_float(float net_float) ;
float hton_float(float host_float) ;

double htond (double x);
float htonf (float x);

int open_socket_udp_dev(int *ptr_fd_socket, char* ip, unsigned int port);
int read_socket_udp_data(int fd_socket);
int send_socket_udp_data(int fd_socket, unsigned char *buf, unsigned int len, char *target_ip, unsigned int target_port);

extern struct sockaddr_in udp_sendto_addr;//服务器用于发送的socket
extern int fd_sock_send;
extern int fd_sock_recv;



#endif /* UDP_H_ */
