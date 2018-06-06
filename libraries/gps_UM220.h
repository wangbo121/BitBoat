/*
 *@File     : gps_UM220.h
 *@Author   : wangbo
 *@Date     : Jun 1, 2018
 *@Copyright: 2018 Beijing Institute of Technology. All rights reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */

#ifndef LIBRARIES_GPS_UM220_H_
#define LIBRARIES_GPS_UM220_H_

#include <stdint.h>

#define UART_GPS_BAUD_UM220              115200
#define UART_GPS_DATABITS_UM220          8 //8 data bit
#define UART_GPS_STOPBITS_UM220          1 //1 stop bit
#define UART_GPS_PARITY_UM220            0 //no parity

#define GPS_MINUTE_TO_DEGREE   0.016666666f

/*
 * 我们要求读取1024个字节 但是不知道UM220什么时候能够发送过来1024个字节来，不能一直傻等着，
 * 所以需要设置最大等待时间，如果在该时间内没有收够1024个字节，就不再继续等待
 */
//#define GPS_DATA_LEN_UM220 1024   // UM220 每次发送的数据最大长度 其实有可能比1024少 这里表示串口每次读取时想要获取的数据字节个数
#define DATA_RECV_BUF_SIZE_UM220     512         // UM220 每次期望读取的字节数
#define DATA_TO_RECV_LEN_UM220       256         // UM220 每次期望读取的字节数
#define MAX_WAIT_TIME_US_UM220       200    // 读取UM220时允许等待的最大时间[us]  波特率是115200的 1ms大概1万个字节

/*
 * 该设备目前是115200波特率 1hz的
 * 可以改为其他波特率和频率
 */

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
    unsigned char spare;

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
    unsigned char mag_east; //磁偏角是东还是西


    int Magnetic_declination; // 磁偏角
    unsigned int HDOP;

};


int gps_uart_init_UM220();

int read_gps_data_UM220();
int write_gps_data_UM220();

int gps_uart_close_UM220();



extern struct T_GPS_UM220 gps_data_UM220;














#endif /* LIBRARIES_GPS_UM220_H_ */













