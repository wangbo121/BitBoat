/*
 * radio.h
 *
 *  Created on: 2016年5月9日
 *      Author: wangbo
 */

#ifndef HEADERS_RADIO_H_
#define HEADERS_RADIO_H_

#define UART_RADIO_BAUD 9600
#define UART_RADIO_DATABITS 8 //8 data bit
#define UART_RADIO_STOPBITS 1 //1 stop bit
#define UART_RADIO_PARITY 0 //no parity

#define RADIO_MAX_WAIT_TIME 6//[s]

/*
 * Function:       radio_uart_init
 * Description:  设置电台串口号及波特率，打开电台的串口，
 *                        与此同时创建串口接收线程，为接收地面站来的数据做准备
 */
int radio_uart_init();

/*
 * Function:       read_radio_data
 * Description:  通过电台接收cmd，waypoint，link，cte，parameter等由地面站发送过来的数据
 *                        如果要增加地面站与驾驶仪的通信包的类型，则需要修改这个源文件
 *                        传输过来的数据尽量每个都是unsigned char型，否则就要考虑大小端，然后在实现中交换位置
 */
int read_radio_data();

/*
 * Function:       send_radio_data
 * Description:  通过电台发送数据，目前是发送了实时数据，发送了回传命令包，发送了回传航点
 */
int send_radio_data(unsigned char *buf, unsigned int len);

/*
 * Function:       radio_uart_close
 * Description:  关闭电台的串口
 */
int radio_uart_close();

#endif /* HEADERS_RADIO_H_ */
