/*
 * gps.h
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#ifndef HEADERS_GPS_H_
#define HEADERS_GPS_H_

#define UART_GPS_BAUD 115200
#define UART_GPS_DATABITS 8 //8 data bit
#define UART_GPS_STOPBITS 1 //1 stop bit
#define UART_GPS_PARITY 0 //no parity

typedef struct
{
	unsigned short year;
	unsigned char month;
	unsigned char date;
	unsigned char hour;
	unsigned char min;
	unsigned char sec;
}nmea_utc_time;

typedef struct
{
    int longitude;//单位[0.00001度]，扩大了100000倍，范围是[-180-+180度]
    int latitude;//单位[0.00001度]，扩大了100000倍，范围是[-180-+180度]
	int altitude;//单位[米]，扩大了100倍

	int roll;//单位[0.01度]，扩大了100倍，范围是[-180-+180度]
	int pitch;//单位[0.01度]，扩大了100倍，范围是[-90-+90度]
	/*
	 * 一定要分清楚yaw=heading 与 course angle不同，course angle指的是前进的速度方向与正北的夹角，也就是航迹与正北的夹角
	 * 20170410修改 单位[0.01度]，扩大了100倍，范围是[-180-+180度]船尾与船头的连线跟正北的夹角 对应GPFPD包中的heading
	 */
	int yaw;

	int velocity_north;//单位[0.001米/秒]，扩大了1000倍
    int velocity_east;//单位[0.001米/秒]，扩大了1000倍
    int velocity_u;//单位[0.001米/秒]，扩大了1000倍
    unsigned int speed;//单位[0.01米/秒]，扩大了100倍

    float course;/*单位[弧度]，没有放大，范围是[-pi-+pi]，course angle值得是前进方向也就是航迹与正北的夹角*/

	unsigned char posslnum1;
	unsigned char posslnum2;

	nmea_utc_time utc;

	unsigned char ewhemi;
	unsigned char nshemi;
	unsigned char svnum;
    unsigned char posslnum;
    unsigned char gpssta;
}nmea_msg;

/*
 * gps_data这个变量就是gps.h文件对外的接口，
 * gps能够确定的物理量比如经度纬度速度等都从这里获取
 */
extern nmea_msg gps_data;

/*
 * Function:       read_gps_data
 * Description:  gps模块是串口设备，所以gps的初始化是设置串口的号和波特率
 *                        与此同时创建驾驶仪接收到gps数据后的接收线程
 */
int gps_uart_init();

/*
 * Function:       read_gps_data
 * Description:  通过nmea的gps或者导航模块读取gps数据，经过解析，存储到gps_data这个全局变量中
 */
int read_gps_data(unsigned char *buf, unsigned int len);

/*
 * Function:       gps_uart_close
 * Description:  关闭串口
 */
int gps_uart_close();

#endif /* HEADERS_GPS_H_ */
