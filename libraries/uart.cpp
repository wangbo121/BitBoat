/*
 * uart.cpp
 *
 *  Created on: Nov 4, 2017
 *      Author: wangbo
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
 /*set baud rate*/
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>
#include <sys/stat.h>
/*create uart device pthread*/
#include <pthread.h>

/*
 * 设置特殊的波特率
 */
#include <sys/ioctl.h>
#include <linux/serial.h>

#include "uart.h"

#define UART_DEV_TOTAL 12
#define UART_BUF_SIZE  512

static const char *uart_dev[UART_DEV_TOTAL]={"/dev/ttyO0","/dev/ttyO1","/dev/ttyO2","/dev/ttyO3","/dev/ttyO4","/dev/ttyO5",\
                                                                             "/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3","/dev/ttyUSB4","/dev/ttyUSB5",\
                                                                            };
static int uart_fd[UART_DEV_TOTAL]={0};

static int get_uart_num(char *uart_name)
{
    int i=0;
    int uart_no=0;

    //printf("串口名字=%s\n",uart_name);
    for(i=0;i<UART_DEV_TOTAL;i++)
    {
        if(strcmp(uart_name,uart_dev[i])==0)
        {
            //printf("这是第%d个串口\n",i);
            uart_no=i;
            break;
        }
    }

    if(i==UART_DEV_TOTAL)
    {
        printf("串口名称错误，没有相应的串口\n");
        return -1;
    }
    else
    {
        return uart_no;
    }
}

static int get_uart_fd(char *uart_name)
{
    int uart_num;

    uart_num=get_uart_num(uart_name);

    if(-1!=uart_num)
    {
        return uart_fd[uart_num];
    }
    else
    {
        return -1;
    }
}

int open_uart_dev(char *uart_name)
{
    int uart_no=0;
    int fd=0;

    uart_no=get_uart_num(uart_name);

    if(-1 != uart_no)
    {
        fd = open(uart_dev[uart_no], O_RDWR | O_NOCTTY | O_NONBLOCK);
    }
    else
    {
        printf("Can't Open Serial Port %s \n",uart_dev[uart_no]);
    }

    if (-1 == fd)
    {
        printf("Can't Open Serial Port %s \n",uart_dev[uart_no]);
        return(-1);
    }
    else
    {
        //printf("Open %s  .....\n",uart_dev[uart_no]);
    }

    if (fcntl(fd, F_SETFL, 0)<0)
    {
        printf("fcntl failed!\n");
    }
    else
    {
        //printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
    }

    if (isatty(STDIN_FILENO) == 0)
    {
        printf("standard input is not a terminal device\n");
    }
    else
    {
        //printf("is a tty success!\n");
    }

    printf("uart_dev[%s]  :  uart_dev[%d]  :  fd=%d\n",uart_dev[uart_no],uart_no,fd);

    uart_fd[uart_no]=fd;

    return uart_no;
}

struct serial_t
{
    int     fd;
    char    *device;
    int     baud;
    int     databit;
    char    parity;
    int    stopbit;
    int    startbit;
    struct termios    options;
};

int serial_set_speci_baud(struct serial_t *tty,int baud)
{
   struct serial_struct ss,ss_set;
   cfsetispeed(&tty->options,B38400);
   cfsetospeed(&tty->options,B38400);
   tcflush(tty->fd,TCIFLUSH);
   tcsetattr(tty->fd,TCSANOW,&tty->options);
   if((ioctl(tty->fd,TIOCGSERIAL,&ss))<0){
       printf("BAUD: error to get the serial_struct info:%s\n",strerror(errno));
       return -1;
   }
   ss.flags = ASYNC_SPD_CUST;
   ss.custom_divisor = ss.baud_base / baud;
   //设置波特率
   if((ioctl(tty->fd,TIOCSSERIAL,&ss))<0){
       printf("BAUD: error to set serial_struct:%s\n",strerror(errno));
       return -2;
   }

   //获取波特率
   ioctl(tty->fd,TIOCGSERIAL,&ss_set);
   printf("BAUD: success set baud to %d,custom_divisor=%d,baud_base=%d\n",
           baud,ss_set.custom_divisor,ss_set.baud_base);
   return 0;
}

int set_uart_opt(char *uart_name, int speed, int bits, char event, int stop)
{
    int fd;
    struct termios newtio, oldtio;

    fd=get_uart_fd(uart_name);

    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (bits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch (event)
    {
    /*odd check*/
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    /*even check*/
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    case 0:
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch (speed)
    {
    case 300:
        cfsetispeed(&newtio, B300);
        cfsetospeed(&newtio, B300);
        break;
    case 600:
        cfsetispeed(&newtio, B600);
        cfsetospeed(&newtio, B600);
        break;
    case 1200:
        cfsetispeed(&newtio, B1200);
        cfsetospeed(&newtio, B1200);
        break;
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
		cfsetispeed(&newtio, B19200);
		cfsetospeed(&newtio, B19200);
		break;
    case 38400:
        cfsetispeed(&newtio, B38400);
        cfsetospeed(&newtio, B38400);
        break;
    case 57600:
		cfsetispeed(&newtio, B57600);
		cfsetospeed(&newtio, B57600);
		break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;

    case 100000:
    	struct serial_t tty;
    	tty.fd = fd;
    	tty.baud = 100000; //读取sbus串口数据
    	serial_set_speci_baud(&tty, 100000);
    	return 0;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }

    if (stop == 1)
    {
        newtio.c_cflag &= ~CSTOPB;
    }
    else if (stop == 2)
    {
        newtio.c_cflag |= CSTOPB;
    }

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);

    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }

    return 0;
}

int send_uart_data(char *uart_name, char *send_buf, int buf_len)
{
    int ret=0;
    int fd=0;

    fd=get_uart_fd(uart_name);

    ret = write(fd, send_buf, buf_len);
    if (ret == -1)
    {
        printf("write device error fd = %d\n",fd);
        return -1;
    }

    return 1;
}

/*
 * 这个函数一定会读取到buf_len个数据到rcv_buf后才会结束，
 * 除非到达time_out_ms时间后，还没有收到buf_len个数的数据才会结束
 */
int read_uart_data(char *uart_name, char *rcv_buf, int time_out_ms, int buf_len)
{
    int fd=0;
    int retval;
    static fd_set rfds;
    struct timeval tv;
    int ret, pos;
    tv.tv_sec = time_out_ms / 1000;  //set the rcv wait time
    tv.tv_usec = (time_out_ms % 1000) * 1000;  //100000us = 0.1s
    //tv.tv_usec = 100;  //100000us = 0.1s

    struct stat temp_stat;

    fd=get_uart_fd(uart_name);
    pos = 0;

    while (1)
    {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        if(-1==fstat(fd,&temp_stat))
        {
            printf("fstat %d error:%s",fd,strerror(errno));
        }

        //retval = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
        retval = select(fd + 1, &rfds, NULL, NULL, &tv);//非堵塞模式读取串口数据

        if (retval == -1)
        {
            perror("select()");
            break;
        }
        else if (retval)
        {
            ret = read(fd, rcv_buf + pos, 1);
            if (-1 == ret)
            {
                break;
            }

            pos++;
            if (buf_len <= pos)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    return pos;
}

int read_uart_data_one_byte(char *uart_name)
{
    int fd=0;
    int retval;
    static fd_set rfds;
    struct timeval tv;
    int ret;

    tv.tv_sec = 0;  //set the rcv wait time
    tv.tv_usec = 20 * 1000;  //100000us = 0.1s  2毫秒

    char rcv_char;
    struct stat temp_stat;

    fd=get_uart_fd(uart_name);

    while (1)
    {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        if(-1==fstat(fd,&temp_stat))
        {
            printf("fstat %d error:%s",fd,strerror(errno));
        }

        retval = select(fd + 1, &rfds, NULL, NULL, &tv);

        if (retval == -1)
        {
            perror("select()");
            break;
        }
        else if (retval)
        {
            ret = read(fd,  &rcv_char, 1);
            if (-1 == ret)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    if(ret==-1)
    {
        return -1;
    }
    else
    {
        return rcv_char;
    }
}

int close_uart_dev(char *uart_name)
{
    int fd=0;

    fd=get_uart_fd(uart_name);

    if(fd != -1)
    {
        close(fd);
    }

    return 0;
}
