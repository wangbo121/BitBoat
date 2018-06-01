/*
 * global.h
 *
 *  Created on: Nov 6, 2017
 *      Author: wangbo
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

/*
 * 简单打印调试信息
 */
#define DEBUG_SWITCH   1//如果不想打印信息，就将这句代码注释掉

#ifdef    DEBUG_SWITCH
#define DEBUG_PRINTF(fmt,args...) printf(fmt, ##args)
#else
#define DEBUG_PRINTF(fmt,args...) /*do nothing */
#endif

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

#ifndef M_E
#define M_E        2.71828182845904523536  /* 自然数 */
#endif
#ifndef M_PI
#define M_PI       3.14159265358979323846  /* PI */
#endif
#ifndef M_1_PI
#define M_1_PI		0.31830988618379067154  /* 1/pi */
#endif
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/*
 * 下面是与控制相关的
 */
#define TEST 0



#define __RADIO_

#define __GPS_
//#define XW_GPFPD_GPS //GPS的型号
#define NMEA_GPS

//#define UART_RADIO          "/dev/ttyUSB0"
#define UART_RADIO          "/dev/ttyO3"

//#define UART_GPS               "/dev/ttyO2"
//#define UART_GPS_Y901               "/dev/ttyO1"


//test
#define UART_GPS               "/dev/ttyO2"
#define UART_GPS_Y901               "/dev/ttyO1"




/*
 * 因为从导航姿态模块读回来的gps数据和航向数据都是经过 扩大倍数 转为int整型的，
 * 但是实际使用的是真实的浮点型，所以有下面的转换
 * 建议写成1e-5这种形式
 */
#define GPS_LOCATION_SCALE 1e-5 //gps中经纬度因为是放大了10^5的，所以需要缩小

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

/*
 * 利用偏航距离进行修正的最大度数
 * 30.0/180.0*3.1415926=0.5236
 */
#define MAX_CTE_CORRECT_RADIAN 0.5236

/*
 * SAIL_MODE是地面站发送的控制模式可以有8种，
 * 但是sail_mode这8个模式对应的是RC_MODE还是AUTO_MODE是可以重新映射的
 */
#define SAIL_MODE_0    0//遥控
#define SAIL_MODE_1    1//自驾
#define SAIL_MODE_2    2//停止
#define SAIL_MODE_3    3//返航
#define SAIL_MODE_4    4//guide
#define SAIL_MODE_5    5//逗留

#define RC_MODE             0
#define AUTO_MODE       1
#define STOP_MODE        2
#define RTL_MODE           3

/*
 * 在AUTO_MODE模式下又有不同的自驾模式
 */
#define AUTO_MISSION_MODE     0
#define AUTO_GUIDE_MODE         1
#define AUTO_LOITER_MODE        2

//FORMATION_是无人船编队的方式，0:单独 1:领导跟随 2:分布式
#define FORMATION_SOLO                             0
#define FORMATION_LEADER_FOLLOWER    1
#define FORMATION_DISTRIBUTED                2

//传感器校准
#define SENSRO_CHECK_ACC         0
#define SENSRO_CHECK_GYRO      1
#define SENSRO_CHECK_MAG       2
#define SENSRO_CHECK_BARO     3
#define SENSRO_CHECK_RESET     4

//本机测试时使用"127.0.0.1"
//#define AP_LISTEN_UDP_IP           "127.0.0.1"           //自驾仪监听地面站发送来的数据时用的网卡ip地址
//#define AP_LISTEN_UDP_PORT               49000           //自驾仪监听地面站发送来的数据时用的端口号
//#define AP_SENDTO_UDP_IP           "127.0.0.1"           //自驾仪向对方发送数据时，对方的socket或者网卡对应的ip地址
//#define AP_SENDTO_UDP_PORT               49000           //自驾仪向对方发送数据时，对方的socket或者网卡对应的端口号

#define AP_LISTEN_UDP_IP          "192.168.0.15"           //自驾仪监听地面站发送来的数据时用的网卡ip地址
#define AP_LISTEN_UDP_PORT                  1100           //自驾仪监听地面站发送来的数据时用的端口号
#define AP_SENDTO_UDP_IP          "192.168.0.10"           //自驾仪向对方发送数据时，对方的socket或者网卡对应的ip地址
#define AP_SENDTO_UDP_PORT                  1000           //自驾仪向对方发送数据时，对方的socket或者网卡对应的端口号

#define CONTROLLER_TYPE_PID      0
#define CONTROLLER_TYPE_ADRC  1
#define CONTROLLER_TYPE_SMC    2

#define UDP_RECVFROM_BLOCK_TIME 200 //udp监听数据最大阻塞时间 200us
//#define UDP_RECVFROM_BLOCK_TIME 900

#define TURN_MODE_DIFFSPD 0
#define TURN_MODE_RUDDER  1

struct WAY_POINT
{
    unsigned char no;
    unsigned char type;
    unsigned char spd;
    unsigned char alt;

    unsigned int lng;
    unsigned int lat;
};

struct T_GLOBAL_BOOL_BOATPILOT
{
	unsigned char bool_get_gcs2ap_cmd;//电台获取到地面站的命令包
	unsigned char bool_get_gcs2ap_waypoint;//电台获取到地面站的航点包

	unsigned char turn_mode;//转弯方式，0:方向舵，1:差速 2:方向舵和差速同时混合转弯

	unsigned char send_ap2gcs_real_req;//驾驶仪 发送实时数据请求
	unsigned char ap2gcs_real_cnt_previous;//驾驶仪 发送实时数据计数
	unsigned char ap2gcs_real_cnt;
	unsigned char send_ap2gcs_cmd_req;//驾驶仪 发送(回传)命令请求
	unsigned char ap2gcs_cmd_cnt_previous;//驾驶仪 发送(回传)命令计数
	unsigned char ap2gcs_cmd_cnt;

	unsigned char wp_total_num;//wp_data[]数组记录了航点，wp_total_num则用来表示每次自动驾驶时地面站发给驾驶仪的航点总个数，最小是1
	unsigned char wp_next;//自动驾驶时，由地面站设置的或者改变的当前目标航点，最小值0

	unsigned char save_boatpilot_log_req;//保存无人船日志请求，这个是定时保存，每秒存储一个日志结构
	unsigned char save_config_req;//驾驶仪中的config结构发生变化时，就提出保存config请求
	unsigned char save_wp_req;// 保存航点请求

	short left_motor_voltage;// 单位[0.1伏特]
	short right_motor_voltage;// 单位[0.1伏特]

	short current_to_target_radian;// [-180*100-+180*100][0.01弧度]
	short current_to_target_degree;// [0.01度]当前位置与目标航点位置的方位角bearing angle
	short command_course_radian;// [-180*100-+180*100][0.01弧度]
	short command_course_degree;// [0.01度]制导算法得到的导航目标航迹角course angle或者航向角heading angle

	short cte_error_check_radian;// psi_r根据偏航距得到的修正方向舵角[-3.14*1000-+3.14*1000]
	int cte_distance_error; // [0.01米]

	unsigned int cnt_test;


	 float motor_left ;
	       float motor_right;
};

/*
 * 一些通过地面站发送的参数需要记录在驾驶仪的flash中，
 * 地面站只需要同步，不需要重新设置
 */
struct T_CONFIG
{
    unsigned char work_mode;
    unsigned char rud_p;
    unsigned char rud_i;
    unsigned char rud_d;
    unsigned char cte_p;
    unsigned char cte_i;
    unsigned char cte_d;
    unsigned char rudder_setup_reverse;
    unsigned char thruster_setup_reverse;

    unsigned char cruise_throttle_percent;//[0-100%]巡航油门百分比数
    unsigned char throttle_change_time;//[秒],油门改变百分之10所用的时间
    unsigned char arrive_radius;//[10米],到达半径
    unsigned char cte_max_degree;//[度],偏航距最大补偿角
    unsigned char total_wp_num;
};

typedef struct T_DateTime
{
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
    unsigned char stuffing;//填充字节，保证数据包字节数为4的整数倍
}T_DATETIME;

struct T_BIT_LOG
{
	T_DATETIME data_time;
	struct T_GLOBAL_BOOL_BOATPILOT global_bool_boatpilot;
};

/*
 * 存放电台传过来的航点数据包，最大航点数为255
 */
#define MAX_WAYPOINT_NUM 255
extern struct WAY_POINT wp_data[MAX_WAYPOINT_NUM];

extern T_DATETIME datetime_now;//当前的日期和时间，精确到秒。在主线程中每秒更新一次，其它程序直接使用即可

#define GPS_LOG_FILE "gps.log"
#define BOATPILOT_LOG_FILE "boatpilot.log"
#define WAY_POINT_FILE "waypoint.log"
#define CONFIG_FILE "config.log"
extern int fd_gps_log;
extern int fd_boatpilot_log;
extern int fd_waypoint;
extern int fd_config;

extern int fd_socket_generic;

extern struct T_GLOBAL_BOOL_BOATPILOT  global_bool_boatpilot;
extern struct T_BIT_LOG boatpilot_log;

#endif /* GLOBAL_H_ */
