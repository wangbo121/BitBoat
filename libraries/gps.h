/*
 *@File     : gps.h
 *@Author   : wangbo
 *@Date     : May 10, 2016
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */

#ifndef HEADERS_GPS_H_
#define HEADERS_GPS_H_

#include <stdint.h>

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
    /*
     * 这个是抽象数据结构，也就是关于gps的数据都放在这个结构里面
     * 这个结构中的数据范围是由控制精度需求决定的
     * 比如导航控制精度是厘米级别的，那么经纬度至少需要放大1e7倍以上，所以采用了int64_t结构
     * int类型能表示的数据范围有限
     */
//    int longitude; // 单位[1e-7度]，扩大了1e7倍，范围是[-180 ~ +180度]
//    int latitude;  // 单位[1e-7度]，扩大了1e7倍，范围是[-90 ~ +90度]
    int64_t longitude; // 单位[1e-7度]，扩大了1e7倍，范围是[-180 ~ +180度]
    int64_t latitude;  // 单位[1e-7度]，扩大了1e7倍，范围是[-90 ~ +90度]
	int altitude;  // 单位[米]，扩大了100倍

    unsigned int velocity;//单位[0.001米/秒]，扩大了1000倍

    //int course_radian; // 单位[0.01弧度]，范围是[-pi ~ +pi]，course angle值得是前进方向也就是航迹与正北的夹角*/
    float course_radian;/*单位[弧度]，没有放大，范围是[-pi-+pi]，course angle值得是前进方向也就是航迹与正北的夹角*/

	nmea_utc_time utc;
    int velocity_north;//单位[0.001米/秒]，扩大了1000倍
    int velocity_east;//单位[0.001米/秒]，扩大了1000倍
    int velocity_u;//单位[0.001米/秒]，扩大了1000倍

    unsigned char posslnum1;
    unsigned char posslnum2;

	unsigned char ewhemi;
	unsigned char nshemi;
	unsigned char svnum;
    unsigned char posslnum;
    unsigned char gpssta;

    /*
     * 按道理gps模块是不能输出姿态角的，但是有些模块把gps和姿态集成了，所以我们的gps结构就把姿态包括了进来
     * 以备将来融合使用
     * 一定要分清楚yaw = heading 与 course angle不同，course angle指的是前进的速度方向与正北的夹角，也就是航迹与正北的夹角
     * 姿态角的经度我们限制在0.01度，没必要再小了
     */
    int roll;//单位[0.01度]，扩大了100倍，范围是[-180 ~ +180度]
    int pitch;//单位[0.01度]，扩大了100倍，范围是[-90 ~ +90度]
    int yaw; // 单位[0.01度]，扩大了100倍，范围是[-180 ~ +180]
}nmea_msg;

/*
 * gps_data这个变量就是gps.h文件对外的接口，
 * gps能够确定的物理量比如经度纬度速度等都从这里获取
 */
extern nmea_msg gps_data;

int gps_uart_init();

int write_gps_data();

int read_gps_data();

int gps_uart_close();

#endif /* HEADERS_GPS_H_ */
