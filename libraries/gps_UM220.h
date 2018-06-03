/*
 * gps_UM220.h
 *
 *  Created on: Jun 1, 2018
 *      Author: wangbo
 */

#ifndef LIBRARIES_GPS_UM220_H_
#define LIBRARIES_GPS_UM220_H_

#include <stdint.h>

#define UART_GPS_BAUD_UM220              9600
#define UART_GPS_DATABITS_UM220          8 //8 data bit
#define UART_GPS_STOPBITS_UM220          1 //1 stop bit
#define UART_GPS_PARITY_UM220            0 //no parity

struct T_GPS_UM220
{
//    int longitude;//单位[0.00001度]，扩大了100000倍，范围是[-180-+180度]
//    int latitude;//单位[0.00001度]，扩大了100000倍，范围是[-180-+180度]
//
    int64_t longitude; // 放大了 1e7 倍
    int64_t latitude; // 放大了 1e7 倍

    int altitude; // 单位[米]



    int velocity_north;//单位[0.001米/秒]，扩大了1000倍
    int velocity_east;//单位[0.001米/秒]，扩大了1000倍
    int velocity_u;//单位[0.001米/秒]，扩大了1000倍
    unsigned int velocity;//单位[0.1米/秒]，扩大了10倍

    //int course_radian; // 单位[0.01弧度]，范围是[-pi ~ +pi]，course angle值得是前进方向也就是航迹与正北的夹角*/
    float course_radian;/*单位[弧度]，没有放大，范围是[-pi-+pi]，course angle值得是前进方向也就是航迹与正北的夹角*/

    unsigned char posslnum1;
    unsigned char posslnum2;


    unsigned char ewhemi;
    unsigned char nshemi;
    unsigned char svnum; // 本系统可见卫星的总数 这个值和GPGGA中的 参与定位的星数不一样
    unsigned char posslnum; // // 参与定位的卫星数量
    unsigned char gpssta; // A = 定位  V = 未定位

    /*
     * 按道理gps模块是不能输出姿态角的，但是有些模块把gps和姿态集成了，所以我们的gps结构就把姿态包括了进来
     * 以备将来融合使用
     * 一定要分清楚yaw = heading 与 course angle不同，course angle指的是前进的速度方向与正北的夹角，也就是航迹与正北的夹角
     */
    int roll;//单位[0.01度]，扩大了100倍，范围是[-180 ~ +180度]
    int pitch;//单位[0.01度]，扩大了100倍，范围是[-90 ~ +90度]
    int yaw; // 单位[0.01度]，扩大了100倍，范围是[-180 ~ +180]


    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;


    int Magnetic_declination; // 磁偏角
    unsigned char mag_east; //磁偏角是东还是西
    unsigned int HDOP;

};


int gps_uart_init_UM220();

int read_gps_data_UM220();
int write_gps_data_UM220();

int gps_uart_close_UM220();



extern struct T_GPS_UM220 gps_data_UM220;














#endif /* LIBRARIES_GPS_UM220_H_ */













