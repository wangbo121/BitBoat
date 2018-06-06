/*
 * gps_Y901.h
 *
 *  Created on: Apr 25, 2018
 *      Author: wangbo
 */

#ifndef LIBRARIES_GPS_Y901_H_
#define LIBRARIES_GPS_Y901_H_

#define UART_GPS_BAUD_Y901 115200
#define UART_GPS_DATABITS_Y901 8 //8 data bit
#define UART_GPS_STOPBITS_Y901 1 //1 stop bit
#define UART_GPS_PARITY_Y901 0 //no parity

#define DATA_RECV_BUF_SIZE_Y901     512         // UM220 每次期望读取的字节数

#define DATA_TO_RECV_LEN_Y901       256         // UM220 每次期望读取的字节数
//#define DATA_TO_RECV_LEN_Y901       330         // UM220 每次期望读取的字节数
//#define DATA_TO_RECV_LEN_Y901       2         // UM220 每次期望读取的字节数 2个字节需要72微秒

//#define MAX_WAIT_TIME_US_Y901       200    // 读取UM220时允许等待的最大时间[us]  波特率是115200的 1ms大概1万个字节
#define MAX_WAIT_TIME_US_Y901       450    // 读取UM220时允许等待的最大时间[us]  波特率是115200的 1ms大概1万个字节

typedef struct
{
    unsigned short year;
    unsigned char month;
    unsigned char date;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
}nmea_utc_time_Y901;

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

    nmea_utc_time_Y901 utc;

    unsigned char ewhemi;
    unsigned char nshemi;
    unsigned char svnum;
    unsigned char posslnum;
    unsigned char gpssta;
}nmea_msg_Y901;

/*
 * gps_data这个变量就是gps.h文件对外的接口，
 * gps能够确定的物理量比如经度纬度速度等都从这里获取
 */
extern nmea_msg_Y901 gps_data_Y901;

/*
 * Function:       read_gps_data
 * Description:  gps模块是串口设备，所以gps的初始化是设置串口的号和波特率
 *                        与此同时创建驾驶仪接收到gps数据后的接收线程
 */
int gps_uart_init_Y901();

/*
 * Function:       read_gps_data
 * Description:  通过nmea的gps或者导航模块读取gps数据，经过解析，存储到gps_data这个全局变量中
 */
int read_gps_data_Y901();

/*
 * Function:       gps_uart_close
 * Description:  关闭串口
 */
int gps_uart_close_Y901();

void print_data_gps_Y901();


/****************/
struct STime
{
    unsigned char ucYear;
    unsigned char ucMonth;
    unsigned char ucDay;
    unsigned char ucHour;
    unsigned char ucMinute;
    unsigned char ucSecond;
    unsigned short usMiliSecond;
};
struct SAcc
{
    short a[3];
    short T;
};
struct SGyro
{
    short w[3];
    short T;
};
struct SAngle
{
    short Angle[3];
    short T;
};
struct SMag
{
    short h[3];
    short T;
};

struct SDStatus
{
    short sDStatus[4];
};

struct SPress
{
    long lPressure;
    long lAltitude;
};

struct SLonLat
{
    long lLon;
    long lLat;
};

struct SGPSV
{
    short sGPSHeight;
    short sGPSYaw;
    long lGPSVelocity;
};
struct SQ
{
    short q[4];
};

extern struct STime        stcTime;
extern struct SAcc         stcAcc;
extern struct SGyro        stcGyro;
extern struct SAngle   stcAngle;
extern struct SMag         stcMag;
extern struct SDStatus stcDStatus;
extern struct SPress   stcPress;
extern struct SLonLat  stcLonLat;
extern struct SGPSV        stcGPSV;
extern struct SQ       stcQ;


#endif /* LIBRARIES_GPS_Y901_H_ */
