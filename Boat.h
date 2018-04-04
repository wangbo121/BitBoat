/*
 * Boat.h
 *
 *  Created on: Nov 6, 2017
 *      Author: wangbo
 */

#ifndef BOAT_H_
#define BOAT_H_

//C标准头文件
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>//创建文件
#include <pthread.h>
#include <semaphore.h>
#include <sys/stat.h>

//C++标准头文件
#include <iostream>
#include <cmath>


#include "scheduler.h"
#include "utility.h"
#include "radio.h"
#include "save_data.h"
#include "pid.h"
#include "all_external_device.h"
#include "SIM_Vehicle.h"
#include "global.h"
//#include "boatlink.h"
#include "control.h"
#include "navigation.h"
#include "boatlink_udp.h"

#include "udp.h"

/*
 * 控制模式的宏定义
 */
#define STABILIZE 0                     // hold level position
#define ACRO 1                          // rate control  比例控制，其实也就是纯手动控制
#define ALT_HOLD 2                      // AUTO control
#define AUTO 3                          // AUTO control
#define GUIDED 4                        // AUTO control
#define LOITER 5                        // Hold a single location
#define RTL 6                           // AUTO control
#define CIRCLE 7                        // AUTO control
#define POSITION 8                      // AUTO control
#define LAND 9                          // AUTO control
#define OF_LOITER 10            // Hold a single location using optical flow



class Boat
{
public:
    friend class GCS_MAVLINK;

    BIT_PID pid_yaw;
    BIT_PID pid_CTE;

    Boat(void)
    {
		/*
		* 在构造函数的开始就初始化一些内部变量
		*/
		control_mode            = STABILIZE;
    }

    // main loop scheduler
    BIT_Scheduler scheduler;
    static const BIT_Scheduler::Task scheduler_tasks[];

    void setup();
    void loop();

//    static int 				   fd_boatpilot_log;
//    static  int				   fd_waypoint;


    private:
    uint8_t          control_mode;
    uint32_t        loop_cnt;

//    int 				   fd_boatpilot_log;
//    int				   fd_waypoint;
    //int				   fd_config;

    // Global parameters are all contained within the 'g' class. 勿删保留
    //Parameters g;

    private:
    void loop_fast();
    void loop_slow();
    void end_of_task();

    void send_ap2gcs_cmd_boatlink();
    void send_ap2gcs_wp_boatlink();
    void send_ap2gcs_realtime_data_boatlink();

    void send_ap2gcs_realtime_data_boatlink_by_udp();

    void record_config();//记录配置文件
    void record_wp();//记录航点文件
    void record_log();//记录日志

    void set_rc_out();//这给用来设置舵机和电机所使用的pwm波，频率是50hz
    void set_gpio();//设置gpio，
    void set_analogs();//设置模拟量
    void set_relays();//设置继电器开关量

    void get_timedata_now();//获取当前的时间，包含年月日时分秒的
    void update_all_external_device_input( void );
    void update_GPS();

    Watercraft::sitl_input input;//这个是4个电机的输入，然后用于multi_copter.update(input)更新出飞机的飞行状态
    Watercraft::sitl_fdm fdm;
    uint16_t servos_set_out[4];//这是驾驶仪计算的到的motor_out中的四个电机的转速，给电调的信号，1000～2000

};

extern Boat boat;

#endif /* BOAT_H_ */
