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

#define __RADIO_
#define __GPS_

#define REAL_START
//#define TEST

#define XW_GPFPD_GPS
//#define NMEA_GPS

//#define UART_RADIO         "/dev/ttyO3"
#define UART_RADIO         "/dev/ttyUSB0"
#define UART_GPS           "/dev/ttyO2"

/*建议写成1e-5这种形式*/
#define GPS_LOCATION_SCALE 1e-5 //gps中经纬度因为是放大了10^5的，所以需要缩小

/*
 * 因为从导航姿态模块读回来的gps数据和航向数据都是经过 扩大倍数 转为int整型的，
 * 但是实际使用用真实的浮点型，所以有下面的转换
 */
#define GPS_INT_TO_REAL 1e-5
#define GPS_DIRECTION_INT_TO_REAL 1e-2

#ifndef M_E
#define M_E        2.71828182845904523536
#endif

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

/*PI-3.1415926的倒数*/
#ifndef M_PI_RECIPROCAL
#define M_PI_RECIPROCAL 0.318309891
#endif

/*利用偏航距离进行修正的最大度数*/
/*30.0/180.0*3.1415926=0.5236*/
#define MAX_CTE_CORRECT_RADIAN 0.5236

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define STOP_MODE        0
#define RC_MODE          1
#define AUTO_MODE        2
#define MIX_MODE         3
#define RTL_MODE         4

#define AUTO_MISSION_MODE     0
#define AUTO_GUIDE_MODE       1
#define AUTO_LOITER_MODE      2

struct T_GLOBAL_BOOL_BOATPILOT
{
    unsigned char bool_get_gcs2ap_cmd;//电台获取到地面站的命令包
    unsigned char bool_get_gcs2ap_waypoint;//电台获取到地面站的航点包
    unsigned char bool_gcs2ap_beidou;//0:不解析北斗数据，1:解析北斗接收的地面站数据包，并采用
    unsigned char bool_generator_on;//发电机 0:不工作，1:工作
    unsigned char bool_is_sending_wp_ap2gcs;//电台 在回传全部航点时，作为正在回传的标志
    unsigned char bool_beidou_get_gcs2ap_cmd;
    unsigned char bool_beidou_get_gcs2ap_waypoint;
    unsigned char bool_loiter_mode;//停留模式=true
    unsigned char bool_shutdown_master;//主控重启
    unsigned char bool_shutdown_slave;//副控重启
    unsigned char bool_rudder_calib_success;//方向舵极限值标定成功
    unsigned char bool_is_calib_rudder;//正在标定状态，方向舵可以随意摆动
    unsigned char turn_mode;//转弯方式，0:方向舵，1:差速 2:方向舵和差速同时混合转弯
    unsigned char s2m_generator_onoff_req_previous;//上一次副控向主控请求发电机的工作状态
    unsigned char radio_recv_packet_cnt;//电台 地面站发送给驾驶仪的数据包的计数，计数不同时，驾驶仪才接收地面站的数据，否则则舍弃
    unsigned char radio_recv_packet_cnt_previous;
    unsigned char udp_recv_packet_cnt;//只是用来记录主控接收到副控的udp的数据包计数，没有用作别的用途
    unsigned char wp_total_num;//wp_data[]数组记录了航点，wp_total_num则用来表示每次自动驾驶时地面站发给驾驶仪的航点总个数，最小是1
    unsigned char send_ap2gcs_wp_req;//驾驶仪 发送航点请求
    unsigned char ap2gcs_wp_cnt_previous;//驾驶仪 发送航点计数
    unsigned char ap2gcs_wp_cnt;
    unsigned char send_ap2gcs_real_req;//驾驶仪 发送实时数据请求
    unsigned char ap2gcs_real_cnt_previous;//驾驶仪 发送实时数据计数
    unsigned char ap2gcs_real_cnt;
    unsigned char send_ap2gcs_cmd_req;//驾驶仪 发送(回传)命令请求
    unsigned char ap2gcs_cmd_cnt_previous;//驾驶仪 发送(回传)命令计数
    unsigned char ap2gcs_cmd_cnt;
    unsigned char send_m2s_udp_req;//主控给副控发送udp数据包请求
    unsigned char m2s_udp_cnt_previous;//主控给副控发送udp数据包计数
    unsigned char m2s_udp_cnt;
    unsigned char bd_send_ap2gcs_wp_req;
    unsigned char bd_ap2gcs_wp_cnt_previous;
    unsigned char bd_ap2gcs_wp_cnt;
    unsigned char bd_send_ap2gcs_real_req;
    unsigned char bd_ap2gcs_real_cnt_previous;
    unsigned char bd_ap2gcs_real_cnt;
    unsigned char bd_send_ap2gcs_cmd_req;
    unsigned char bd_ap2gcs_cmd_cnt_previous;
    unsigned char bd_ap2gcs_cmd_cnt;
    unsigned char rudder_calib_cnt_previous;//标定方向舵的左极限右极限和中间值
    unsigned char launch_req_ack_cnt_previous;//副控确认火箭可发射后，向主控提出发射火箭请求
    unsigned char save_boatpilot_log_req;//保存无人船日志请求，这个是定时保存，每秒存储一个日志结构
    unsigned char wp_packet_cnt;//地面站发送给驾驶仪的第wp_packet_cnt个航点数据包
    unsigned char assign_config_req;//当驾驶仪接收到地面站的命令包时，驾驶仪认为需要更新配置(也就是一些参数设置)
    unsigned char assign_config_cnt_previous;
    unsigned char assign_config_cnt;
    unsigned char save_config_req;//驾驶仪中的config结构发生变化时，就提出保存config请求
    unsigned char set_switch_channel_previous;//切换器 上一次的通道
    unsigned char voltage_llim_previous;//切换器 放电电压最低值
    unsigned char voltage_hlim_previous;//切换器 放电电压最高值
    unsigned char bat0_is_discharing;//切换器 电池0通道正在放电
    unsigned char bat1_is_discharing;//切换器 电池1通道正在放电
    unsigned char charger_set_channel_previous;//充电机 上一次的通道
    unsigned char charger_set_voltage_previous;//充电机 上一次的电压
    unsigned char charger_set_current_previous;//充电机 上一次的电流
    unsigned char charge_start_previous;//充电机 上次的开机关机状态，只有检测到与当前状态不同时才改变
    unsigned char wp_next;//自动驾驶时，由地面站设置的或者改变的当前目标航点，最小值0
    unsigned char send_ap2gcs_wp_start_num;//发送航点数据包时，起始航点数，最小值0
    unsigned char send_ap2gcs_wp_end_num;//发送航点数据包时，航点数木，最小值1
    unsigned char send_ap2gcs_specific_wp_req;//地面站请求驾驶仪回传特定的某几个航点，不是回传全部航点
    unsigned char master_state;//主控的状态 D7:电台等待超时 D6:gps等待超时 D5:modbus继电器模拟量等待超时 D4:modbus码盘等待超时 D3:udp等待超时 D2:北斗等待超时 D1: D0:
    unsigned char slave_state;//副控的状态
    unsigned char slave_config_previous;//地面站配置副控，现在用来控制是否读取485的电流通道
    unsigned char spare0;//64字节

    short left_motor_voltage;//单位[0.1伏特]
    short right_motor_voltage;//单位[0.1伏特]
    short rudder_angle_degree;//单位[度] 范围[-45-+45度]这是通过码盘读回来的真实的方向舵的角度值，通过实时数据返回给地面站
    short cte_error_check_radian;//psi_r根据偏航距得到的修正方向舵角[-3.14*1000-+3.14*1000]
    short current_to_target_radian;//[-180*1000-+180*1000][0.001弧度]
    short command_radian;//[-180*1000-+180*1000][0.001弧度]
    short dir_target_degree;//[0.01度]当前位置与目标航点位置的方位角bearing angle
    short dir_nav_degree;//80字节//[0.01度]制导算法得到的导航目标航迹角course angle或者航向角heading angle

    int cte_distance_error;//84字节//[0.01米]

    short rudder_middle_position;//方向舵处于中间位置时对应的码盘的读数
    short rudder_left_limit_position;//[0-720]方向舵标定时的左极限位置，对应的码盘读数
    short rudder_right_limit_position;//[0-720]方向舵标定时的右极限位置，对应的码盘读数
    short rudder_delta_fabs;//92字节//[0-90]方向舵标定时，为保证向左和向右转动时左右极限值对称，故有此变量

    /*
     * 以后改变global变量结构
     * 必须从这个以后添加，否则需要修改日志记录那里
     * 20170518
     */
    unsigned char gcs2ap_wp_cnt;//电台--接收到的地面站发送给驾驶仪的航点包计数
    unsigned char gcs2ap_cmd_cnt;//电台--接收到的地面站发送给驾驶仪的航点包计数
    unsigned char bd_gcs2ap_wp_cnt;//北斗--接收到的地面站发送给驾驶仪的航点包计数
    unsigned char bd_gcs2ap_cmd_cnt;//北斗--接收到的地面站发送给驾驶仪的命令包计数

    float radio_send_time;
    float radio_need_to_send_time;
    float radio_send_delta_time;
    float radio_get_data_previous_time_s;//电台获取到数据的前一时间单位秒[s]
    float radio_lose_data_time_s;//电台获取到数据的前一时间单位秒[s]

    float radio_wait_time;
    float gps_wait_time;
    float modbus_wait_time;
    float modbus_rotary_wait_time;
    float udp_wait_time;
    float bd_wait_time;
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

#define GPS_LOG_FILE "gps.log"
#define BOATPILOT_LOG_FILE "boatpilot.log"
#define WAY_POINT_FILE "waypoint.log"
#define CONFIG_FILE "config.log"

extern int fd_gps_log;
extern int fd_boatpilot_log;
extern int fd_waypoint;
extern int fd_config;

extern struct T_GLOBAL_BOOL_BOATPILOT  global_bool_boatpilot;

extern T_DATETIME datetime_now;//当前的日期和时间，精确到秒。在主线程中每秒更新一次，其它程序直接使用即可。
extern struct tm *gbl_time_val;//全局时间变量，其它的时间都从这里取

#endif /* GLOBAL_H_ */
