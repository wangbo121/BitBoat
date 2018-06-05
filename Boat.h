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
#include "control.h"
#include "navigation.h"
#include "boatlink_udp.h"

#include "gps.h"
#include "gps_Y901.h"
#include "gps_UM220.h"
#include "gps_XW5651.h"

#include "IMU.h"

#include "servo.h"
#include "udp.h"

#include "II2C.h"

#include "BIT_Math.h"

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
    BIT_PID pid_yaw;
    BIT_PID pid_CTE;

    Boat(void):sim_water_craft("39.95635,116.31574,10,0","+")
    {
		/*
		* 在构造函数的开始，必须初始化私有变量，因为私有变量外部函数无法更改
		*/
		control_mode        =       STABILIZE;
		fastloop_cnt        =       0;
    }

    // main loop scheduler
    BIT_Scheduler scheduler;
    static const BIT_Scheduler::Task scheduler_tasks[];

    void setup();
    void loop();

private:
    uint8_t     control_mode;
    uint32_t    fastloop_cnt;

private:
    static void get_timedata_now();//获取当前的时间，包含年月日时分秒的

    void loop_fast();
    static void loop_one_second();
    static void end_of_task();

    static void get_gcs_udp(); // 通过udp获取地面站发送给自驾仪的命令
    static void get_gcs_radio(); // 通过无线电radio获取地面站发送给自驾仪的命令
    static void send_ap2gcs_realtime_data_boatlink_by_udp();

    void update_all_external_device_input( void );
    /*
     * update_GPS update_mpu6050都是假的获取传感器数据，是从all_external_device_input这个结构中获取想要的数据
     * 真正的读取传感器的函数是read_device_...，这些函数把从传感器读取的数据存入到all_external_device_input中
     */
    static void update_GPS();//
    static void update_IMU();
    static void update_mpu6050();// 从all_external_device_input 获取 acc/gyo/姿态

    //  //自驾仪虚拟地输出数据，把控制量啥的输出到all_external_device_output
    void out_execute_ctrloutput();

    void motros_arm_check();
    void motors_set();
    static void write_device_motors_output();

    void write_device_motors_on();

    void write_device_motors_off();

    void disarm_motors();

    void relay_switch_init();


    static void record_config();//记录配置文件
    static void record_wp();//记录航点文件
    static void record_log();//记录日志

    /*
     * 读取和设置外围设备函数
     * 这些是真正的读取硬件设备的函数
     */
    void read_device_gps();
    static void read_device_IMU_mpu6050();
    static void read_device_mpu6050();
    void set_device_rc_out();//这给用来设置舵机和电机所使用的pwm波，频率是50hz
    void set_device_gpio();//设置gpio
    void set_device_analog();//设置模拟量
    void set_device_relays();//设置继电器开关量


    static void read_device_gps_JY901();
    static void read_device_gps_NMEA();
    static void read_device_gps_UM220();

    static void write_device_II2C_test();



    static void update_navigation_loop();

    void start_central_control_computer();

    void write_motors_device_init();

public:
    /*
     * input_servos表示输入控制量
     * 对于船而言input_servos[2] 表示 油门throttle
     *         input_servos[3] 表示 方向rudder
     *         input_servos[0] 表示 滚转aileron
     *         input_servos[1] 表示 俯仰elevator
     */
    float servos_input[4];
    float motors_speed[4];

public:
    /*
     * 仿真测试时使用
     */
    //void loop_fast_simulate();
    void update_sim_water_craft();
    void simulate_init();

    /*
     * 航行器的物理数学模型
     */
    Watercraft sim_water_craft;//+型机架，起始高度为10，yaw是0
    Watercraft::sitl_input input;// 是rudder 和 thruster输入，然后用于更新出航行器的航行状态
    Watercraft::sitl_fdm fdm;
    uint16_t servos_set_out[4];//这是驾驶仪计算的到的motor_out中的四个电机的转速，给电调的信号，1000～2000
};

extern Boat boat;

#endif /* BOAT_H_ */
