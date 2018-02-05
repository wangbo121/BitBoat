/*
 * uart.h
 *
 *  Created on: Nov 4, 2017
 *      Author: wangbo
 */

#ifndef UART_H_
#define UART_H_

/*
 * 适用范围
 * 波特率:   2400 4800 9600 57600 115200
 * 设备:       "/dev/ttyO0","/dev/ttyO1","/dev/ttyO2","/dev/ttyO3","/dev/ttyO4","/dev/ttyO5"
 *                 "/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3","/dev/ttyUSB4","/dev/ttyUSB5"
 */

struct T_UART_DEVICE
{
    char *uart_name;
    int (*ptr_fun)(unsigned char *buf,unsigned int len);

    unsigned char uart_num;

    unsigned char databits;
    unsigned char parity;
    unsigned char stopbits;
    unsigned int  baudrate;
};

/*
 * 根据串口的名字打开串口设备，比如/dev/ttyO0
 */
int open_uart_dev(char *uart_name);

/*
 * 根据串口名字和参数，设置串口波特率等
 */
int set_uart_opt(char *uart_name, int speed, int bits, char event, int stop);

/*
 * 读取串口设备的数据，buf_len是设置的想要读取的字节数，time_out_ms是设置的最大等待时间
 */
int read_uart_data(char *uart_name, char *rcv_buf, int time_out_ms, int buf_len);

/*
 * 根据串口名字，向外发送数据
 */
int send_uart_data(char *uart_name, char *send_buf, int buf_len);

/*
 * 根据串口名字，关闭串口设备
 */
int close_uart_dev(char *uart_name);

/*
 * 创建串口线程，这个是与利用串口发送数据的外部设备最有关的函数
 * uart_recvbuf_and_process是串口线程中的处理函数，参数为ptr_uart_device
 * 处理函数必须是int ptr_fun (unsigned char *buf,unsigned int len)类型的函数
 * 而这个设备是什么样的协议就需要修改这个ptr_fun函数的具体实现，
 * 在使用时只需要把接收该设备的数据的函数的地址赋给ptr_uart_device就可以了
 */
int create_uart_pthread(struct T_UART_DEVICE *ptr_uart_device);

#endif /* UART_H_ */
