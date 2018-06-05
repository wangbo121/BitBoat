/*
 * Boat.cpp
 *
 *  Created on: Nov 6, 2017
 *      Author: wangbo
 */

#include "Boat.h"

Boat boat;

void Boat::setup( void )
{

    /*
     * 创建要保存的二进制日志文件boatpilot_log
     */
    fd_boatpilot_log = create_log_file( (char *)BOATPILOT_LOG_FILE );

    /*
     * 载入航点文件waypoint
     */
    fd_waypoint     = load_data_struct_from_binary( (char *)WAY_POINT_FILE, &wp_data, sizeof(wp_data));
    if(fd_waypoint  == -1)
    {
        printf("无法创建航点文件\n");
    }
    else
    {
        printf("可以读取航点或者创建航点文件 fd = %d\n",fd_waypoint);
    }

    /*
     * 载入配置文件config
     */
    fd_config      = load_data_struct_from_binary( (char *)CONFIG_FILE, &boatpilot_config_udp, sizeof(boatpilot_config_udp) );
    if(fd_config   == -1)
    {
        printf("无法创建配置文件\n");
    }
    else
    {
        printf("可以读取配置或者创建配置文件 fd = %d\n",fd_config);

        gcs2ap_all_udp.rud_p                          = boatpilot_config_udp.rud_p;
        gcs2ap_all_udp.rud_i                          = boatpilot_config_udp.rud_i;
        gcs2ap_all_udp.rud_d                          = boatpilot_config_udp.rud_d;
        gcs2ap_all_udp.cte_p                          = boatpilot_config_udp.cte_p;
        gcs2ap_all_udp.cte_i                          = boatpilot_config_udp.cte_i;
        gcs2ap_all_udp.cte_d                          = boatpilot_config_udp.cte_d;
        gcs2ap_all_udp.cruise_throttle_percent        = boatpilot_config_udp.cruise_throttle_percent;
        gcs2ap_all_udp.arrive_radius                  = boatpilot_config_udp.arrive_radius;
        global_bool_boatpilot.wp_next                 = boatpilot_config_udp.current_target_wp_num;
        global_bool_boatpilot.wp_total_num            = boatpilot_config_udp.total_wp_num;
    }

#ifdef __RADIO_
    radio_uart_init();
#endif

#ifdef __GPS_
    gps_uart_init(); // 以后要把gps作为某一个抽象类来处理，否则每换1个gps都需要重新写
    gps_uart_init_UM220();
    gps_uart_init_Y901();
#endif

    /*
     * 有1个网卡是用来监听地面站向自驾仪发送数据的
     * 与此同时，发送也是通过这个网卡向地面站回传实时数据
     * 现在这个udp通信代替电台通信 ，作为地面站和自驾仪的通信连接媒介
     */
    DEBUG_PRINTF("Boat::setup    :    fd_socket_generic = %d \n",fd_socket_generic);
    open_socket_udp_dev(&fd_socket_generic, (char *)"AP_LISTEN_UDP_IP", AP_LISTEN_UDP_PORT);

    II2C_init();
    write_motors_device_init(); // motor的通信用iic

    /* **********************************************************************************************************
     * 作为分界线
     * 以上都是硬件初始化
     * 下面的是驾驶仪内部软件初始化
     */


    /*
     * 初始化自驾仪控制量输入以及参数的初始值，
     * 下面所有的设置都是从地面站获取后使用
     * 尤其注意初始的方向舵和油门量值
     * 控制量的限幅参数
     */
    gcs2ap_all_udp.cmd.pilot_manual                 =   1; // 默认通过驾驶仪
    gcs2ap_all_udp.cmd.throttle                     =   0;
    gcs2ap_all_udp.cmd.rudder                       =   127;

    gcs2ap_all_udp.rud_p                            =   20;//rud_p单位是[0.1]所以一开始要赋值大一些
    gcs2ap_all_udp.rud_i                            =   0;
    gcs2ap_all_udp.rud_d                            =   0;
    gcs2ap_all_udp.cte_p                            =   20;
    gcs2ap_all_udp.cte_i                            =   0;
    gcs2ap_all_udp.cte_d                            =   0;

	gcs2ap_all_udp.arrive_radius                    =   10;//单位是[10米]，初始到达半径设置为100米
    gcs2ap_all_udp.cruise_throttle_percent          =   50;//初始巡航油门设置为百分之50

    gcs2ap_all_udp.workmode                         = RC_MODE;

	gcs2ap_all_udp.mmotor_off_pos                   =   0;
	gcs2ap_all_udp.mmotor_on_pos                    =   255;
	gcs2ap_all_udp.rudder_left_pos                  =   0;
	gcs2ap_all_udp.rudder_right_pos                 =   255;
	gcs2ap_all_udp.rudder_mid_pos                   =   127;


    /*
     * 下面是全局变量global_bool_boatpilot的初始化
     * global_bool_boatpilot和gcs2ap_all_udp的区别在于后者是从地面站发送过来的参数，
     * 而global_bool_boatpilot是驾驶仪内部需要的全局计算量
     * 同时，一些标志量也用global_bool_boatpilot中的bool表示
     */
	global_bool_boatpilot.turn_mode                =    TURN_MODE_DIFFSPD;
    global_bool_boatpilot.save_boatpilot_log_req   =    TRUE;

    /*
     * 设置PID控制器的参数 下面的这个pid_yaw还没有开始使用
     */
    pid_yaw.set_kP( 2 );
    pid_yaw.set_kI( 0 );
    pid_yaw.set_kD( 0 );
    pid_yaw.set_imax( 0.174 * 3 );//30度

    pid_CTE.set_kP( 2 );
    pid_CTE.set_kI( 0 );
    pid_CTE.set_kD( 0 );
    pid_CTE.set_imax( 0.174 * 3 );//30度

    /*
     * 初始化导航环节
     */
    navigation_init();


    /* **************************************************************************************
     * 仿真时使用
     */
#if SIMULATE_BOAT
    simulate_init();
#endif
}

void Boat::simulate_init()
{
#if SIMULATE_BOAT
    unsigned int lng_start = 116.31574 * 1e5;
    unsigned int lat_start = 39.95635 * 1e5;
    unsigned char wp_total_num_test = 5;
    for(int i=0; i< wp_total_num_test; i++)
    {
        wp_data[i].no = i;
        wp_data[i].type = 0;
        wp_data[i].spd = 50;
        wp_data[i].alt = 12;

        wp_data[i].lng = lng_start + 3e3 * i;
        wp_data[i].lat = lat_start;

        DEBUG_PRINTF("wp_data[%d].lng = %d \n",i,wp_data[i].lng);
        DEBUG_PRINTF("wp_data[%d].lat = %d \n",i,wp_data[i].lat);
    }
    global_bool_boatpilot.wp_total_num = wp_total_num_test;
#endif
}

T_DATETIME datetime_now;//当前的日期和时间，精确到秒。在主线程中每秒更新一次，其它程序直接使用即可。
void Boat::get_timedata_now()
{
    struct tm *gbl_time_val;//全局时间变量，其它的时间都从这里取
    time_t timep;

    //获取系统时间
    time(&timep);
    gbl_time_val = localtime(&timep);
    datetime_now.year = (unsigned short)(gbl_time_val->tm_year + 1900);
    datetime_now.month = (unsigned char)(gbl_time_val->tm_mon + 1);
    datetime_now.day = (unsigned char)gbl_time_val->tm_mday;
    datetime_now.hour = (unsigned char)gbl_time_val->tm_hour;
    datetime_now.minute = (unsigned char)gbl_time_val->tm_min;
    datetime_now.second = (unsigned char)gbl_time_val->tm_sec;

    DEBUG_PRINTF("当前系统时间是:%d年%d月%d日%d时%d分%d秒\n", datetime_now.year, datetime_now.month, datetime_now.day, datetime_now.hour, datetime_now.minute, datetime_now.second);
}

void Boat::update_all_external_device_input( void )
{
	/*
	 * 驾驶仪本身假设所有的外部设备的信息都已经获取 并且存在了all_external_device_input中
	 * 但是我们还是要假设去更新外部设备数据，比如用update_GPS来从all_external_device_input中获取数据
	 * 之所以增加all_external_device_input这一层，不用考虑外部的数据，
	 * 从而在没有外部传感器数据的情况下，也能靠着fdm的仿真数据进行模拟测试
	 */

	/*
	* all_external_device_input其实应该是由外部设备有数据更新后，把数据赋值给all_external_device_input，
	* 驾驶仪只是从这里获取数据，不用管真实的传感器的数据是否更新
	* 而且我只是从这里读数据，不写，应该不会出现同时写某一个变量的情况吧
	* 但是这种方式有可能出现这边在写内存，而另一边在读内存，这个概率有多大又会造成什么影响呢
	*/

#if SIMULATE_BOAT
	/*
	 * 获取遥控器的信号
	 */
//	all_external_device_input.rc_raw_in_0 = 1500;
//	all_external_device_input.rc_raw_in_1 = 1500;
//	all_external_device_input.rc_raw_in_2 = 1500;
//	all_external_device_input.rc_raw_in_3 = 1500;
//	all_external_device_input.rc_raw_in_4 = 1990;//绕航点飞行模式
//	all_external_device_input.rc_raw_in_5 = 1500;
//	all_external_device_input.rc_raw_in_6 = 1500;
//	all_external_device_input.rc_raw_in_7 = 1500;
//	all_external_device_input.rc_raw_in_8 = 1500;

	/*
	 * 这里使用仿真sim_water_craft的数据
	 * 如果有相应的硬件传感器则，应该是真实的传感器反馈数据
	 */
	all_external_device_input.latitude    = fdm.latitude;
	all_external_device_input.longitude   = fdm.longitude;
	all_external_device_input.altitude    = 10 ;
	all_external_device_input.v_north     = fdm.speedN;
	all_external_device_input.v_east      = fdm.speedE;
	all_external_device_input.v_down      = fdm.speedD;
	all_external_device_input.heading     = fdm.heading;
    all_external_device_input.course      = fdm.heading * 3.14f / 180.0f;



#else
	/*
	 * 20180601 下面是更新使用真实的外部设备
	 */
	all_external_device_input.latitude              = ((float)gps_data_UM220.latitude)  * GPS_SCALE;
    all_external_device_input.longitude             = ((float)gps_data_UM220.longitude) * GPS_SCALE;
    all_external_device_input.altitude              = gps_data_UM220.altitude;
    all_external_device_input.speed                 = ((float)gps_data_UM220.velocity) * 0.1;
    all_external_device_input.course                = gps_data_UM220.course_radian;

    all_external_device_input.phi                   = (float)stcAngle.Angle[1]/32768*180;
    all_external_device_input.theta                 = (float)stcAngle.Angle[0]/32768*180;
    all_external_device_input.psi                   = (float)stcAngle.Angle[2]/32768*180;


    all_external_device_input._accel_x              = 0;
    all_external_device_input._accel_y              = 0;
    all_external_device_input._accel_z              = 0;

    all_external_device_input._gyro_x               = 0;
    all_external_device_input._gyro_y               = 0;
    all_external_device_input._gyro_z               = 0;



#endif
}

void Boat::get_gcs_udp()
{
	read_socket_udp_data( fd_socket_generic);
}

void Boat::get_gcs_radio()
{
	read_radio_data();
}

void Boat::send_ap2gcs_realtime_data_boatlink_by_udp()
{
	send_ap2gcs_real_udp();
}

void Boat::update_GPS()
{
	gps_data.latitude        = (int64_t)( all_external_device_input.latitude * GPS_SCALE_LARGE);
	gps_data.longitude       = (int64_t)(all_external_device_input.longitude * GPS_SCALE_LARGE);

	gps_data.course_radian   = all_external_device_input.course;

	// JY901
	gps_data.roll = (int)(all_external_device_input.phi * 1e2);
	gps_data.pitch = (int)(all_external_device_input.theta * 1e2);
	gps_data.yaw = (int)(all_external_device_input.psi * 1e2);

	gps_data.velocity_north = (int)(all_external_device_input.v_north * 1e3);
	gps_data.velocity_east  = (int)(all_external_device_input.v_east  * 1e3);
}

void Boat::update_IMU()
{
    IMU_data.acc_x    = all_external_device_input._accel_x; // 暂时不需要 所以是0
    IMU_data.acc_y    = all_external_device_input._accel_y;
    IMU_data.acc_z    = all_external_device_input._accel_z;

    IMU_data.gyro_x   = all_external_device_input._gyro_x;  // 暂时不许要 所以是0
    IMU_data.gyro_x   = all_external_device_input._gyro_y;
    IMU_data.gyro_x   = all_external_device_input._gyro_z;

    IMU_data.roll     = (int)(all_external_device_input.phi   * 1e2);
    IMU_data.pitch    = (int)(all_external_device_input.theta * 1e2);
    IMU_data.yaw      = (int)(all_external_device_input.psi   * 1e2);



}

void Boat::update_mpu6050()
{

}

void Boat::update_sim_water_craft()
{
#if SIMULATE_BOAT

    /*
     * 下面是把驾驶仪计算得到的电机或者舵机的输出给到simulator模拟器中
     */
    servos_set_out[0] = (uint16_t)(ctrloutput.rudder_pwm);
    servos_set_out[1] = (uint16_t)(ctrloutput.mmotor_onoff_pwm);

    memcpy(input.servos, servos_set_out, sizeof(servos_set_out));
    sim_water_craft.update(input);
    sim_water_craft.fill_fdm(fdm);
#endif
}

void Boat::read_device_gps_JY901()
{
    read_gps_data_Y901();
}

void Boat::read_device_gps_UM220()
{
    read_gps_data_UM220();
}

void Boat::read_device_gps_NMEA()
{

}

void Boat::out_execute_ctrloutput()
{
	execute_ctrloutput(&ctrloutput);
}

void Boat::motros_arm_check()
{



}

#define MOTORS_SET_NUM 4
void Boat::motors_set()
{
    /*
     * 这个函数的作用是把throttle 和 rudder
     * 映射成为左电机 和 右电机的输出
     */
#if 0
    /*
     * servos_input表示输入控制量
     * 对于船而言servos_input[2] 表示 油门throttle
     *         servos_input[3] 表示 方向rudder
     *         servos_input[0] 表示 滚转aileron
     *         servos_input[1] 表示 俯仰elevator
     */
    float                   _throttle_factor[MOTORS_SET_NUM];
    float                   _yaw_factor[MOTORS_SET_NUM];  // each motors contribution to yaw (normally 1 or -1)

    servos_input[2]         = constrain_value(ctrloutput.mmotor_onoff_pwm, 1000.0, 2000.0);
    servos_input[3]         = constrain_value(ctrloutput.rudder_pwm, 1000.0, 2000.0);

    // 油门量是百分之百 方向则需要有正负
    servos_input[3] = servos_input[3] - 1500.0f; // 转为-500 ~ +500

    //这个是针对电机的系数，如果左右各2个电机，则需要用四个了
    _throttle_factor[0]= +1;  _yaw_factor[0]  = +1; // 0 : left motor
    _throttle_factor[1]= +1;  _yaw_factor[1]  = -1; // 1 : left motor
//    _throttle_factor[2]= +1;  _yaw_factor[2]  = +1; // 2 : left motor
//    _throttle_factor[3]= +1;  _yaw_factor[3]  = -1; // 3 : right motor

    for(int i=0;i<4;i++)
    {
        motors_speed[i] = servos_input[2] * _throttle_factor[i]+ \
                          servos_input[3] * _yaw_factor[i];
    }
#else
    float motor_left_pwm_out  = 0.0;
    float motor_right_pwm_out = 0.0;
    float rudder2motor        = 0.0;
    int   delta_rudder        = 0;
    float throttle_scale      = 0.95;

    servos_input[2]         = constrain_value(ctrloutput.mmotor_onoff_pwm, 1000.0, 2000.0);
    servos_input[3]         = constrain_value(ctrloutput.rudder_pwm, 1000.0, 2000.0);

    /*
     * 增加差速，在油门的基础上，添加舵效，分为5个档位
     */
    motor_left_pwm_out  = servos_input[2]; //先把油门加上
    motor_right_pwm_out = servos_input[2];
    if(servos_input[2] > 1900)
    {
        motor_left_pwm_out  = servos_input[2] * throttle_scale; //油门最大时也要保证能转向，所以缩小了油门量，当油门量大于1900时，乘以系数
        motor_right_pwm_out = servos_input[2] * throttle_scale;
    }

    /*
     * 下面程序是为了阶梯性变化油门，是为了避免油门变化太快，
     * 但是如果电调电机性能都很好，那就不用阶梯性变化油门
     */
    rudder2motor = servos_input[3] - 1500;
    if(rudder2motor > 50)
    {
        //右舵
        delta_rudder        =   (int)rudder2motor;
        delta_rudder        =   fabs(delta_rudder);
        motor_left_pwm_out  =   motor_left_pwm_out  + (delta_rudder / 100) * 100;
        motor_right_pwm_out =   motor_right_pwm_out - (delta_rudder / 100) * 100;
    }
    else if(rudder2motor < -50)
    {
        //左舵
        delta_rudder        =   (int)rudder2motor;
        delta_rudder        =   fabs(delta_rudder);
        motor_left_pwm_out  =   motor_left_pwm_out  - (delta_rudder / 100) * 100;
        motor_right_pwm_out =   motor_right_pwm_out + (delta_rudder / 100) * 100;
    }

    motor_left_pwm_out  = constrain_value(motor_left_pwm_out,  1000.0, 2000.0);
    motor_right_pwm_out = constrain_value(motor_right_pwm_out, 1000.0, 2000.0);

    motors_speed[0]  = motor_left_pwm_out;
    motors_speed[1]  = motor_right_pwm_out;

    /*
     * 把motor_left_pwm_out和motor_right_pwm_out值存储到global中
     * 然后在one_second_loop中打印
     */
    global_bool_boatpilot.motor_left  = motors_speed[0];
    global_bool_boatpilot.motor_right = motors_speed[1];
#endif
}

void Boat::write_device_motors_output()
{
    set_motors_speed(boat.motors_speed);
}

void Boat::write_device_motors_on()
{
    set_motor_on();
}

void Boat::write_device_motors_off()
{
    set_motor_on();
}


void Boat::record_wp()
{
	int write_len;

	if(global_bool_boatpilot.save_wp_req)
	{
		write_len = save_data_to_binary_log(fd_waypoint, &wp_data, sizeof(wp_data));
		printf("Boat::record_wp()    :    write_len = %d \n",write_len);
		global_bool_boatpilot.save_wp_req = FALSE;
	}
}

void Boat::record_log()
{
    /*
     * 保存日志文件到二进制文件boatpilot_log
     * 尽量把sizeof(boatpilot_log)作为4字节对齐，勿删
     */

    unsigned char boatpilot_log_save[1024] = {'*'};
    int save_bytes_cnt;

	//DEBUG_PRINTF("记录日志\n");
	global_bool_boatpilot.save_boatpilot_log_req = TRUE;
    if(global_bool_boatpilot.save_boatpilot_log_req)
    {
        /*时间戳*/
        boatpilot_log.data_time.year=datetime_now.year;
        boatpilot_log.data_time.month=datetime_now.month;
        boatpilot_log.data_time.day=datetime_now.day;
        boatpilot_log.data_time.hour=datetime_now.hour;
        boatpilot_log.data_time.minute=datetime_now.minute;
        boatpilot_log.data_time.second=datetime_now.second;
        boatpilot_log.data_time.stuffing=datetime_now.stuffing;

        memcpy(&boatpilot_log.global_bool_boatpilot, &global_bool_boatpilot, sizeof(global_bool_boatpilot));

        memcpy(boatpilot_log_save, &boatpilot_log, sizeof(boatpilot_log));
        memcpy(&boatpilot_log_save[(int)sizeof(boatpilot_log) + 1], &gcs2ap_all_udp, sizeof(gcs2ap_all_udp));
        save_bytes_cnt = save_data_to_binary_log(fd_boatpilot_log, boatpilot_log_save, (int)sizeof(boatpilot_log) + 1 + (int)sizeof(gcs2ap_all_udp));

        //DEBUG_PRINTF("boatpilot_log_save    : save %u bytes \n", save_bytes_cnt);

        global_bool_boatpilot.save_boatpilot_log_req = FALSE;
    }
}

void Boat::record_config()
{

    boatpilot_config_udp.workmode                   = gcs2ap_all_udp.workmode;
    boatpilot_config_udp.rud_p                      = gcs2ap_all_udp.rud_p;
    boatpilot_config_udp.rud_i                      = gcs2ap_all_udp.rud_i;
    boatpilot_config_udp.rud_d                      = gcs2ap_all_udp.rud_d;
    boatpilot_config_udp.cte_p                      = gcs2ap_all_udp.cte_p;
    boatpilot_config_udp.cte_i                      = gcs2ap_all_udp.cte_i;
    boatpilot_config_udp.cte_d                      = gcs2ap_all_udp.cte_d;
    boatpilot_config_udp.cruise_throttle_percent    = gcs2ap_all_udp.cruise_throttle_percent;
    boatpilot_config_udp.arrive_radius              = gcs2ap_all_udp.arrive_radius;

    boatpilot_config_udp.total_wp_num               = gcs2ap_all_udp.wp_total_num;


    if(memcmp(&boatpilot_config_udp_previous, &boatpilot_config_udp,sizeof(boatpilot_config_udp))==0)
    {
        //printf("config没有变化，不保存\n");
    }
    else
    {
        printf("config发生了变化，需要重新保存\n"); // 20170413已测试，可以在参数更改后保存boatpilot_config
        global_bool_boatpilot.save_config_req=TRUE;
    }
    if(global_bool_boatpilot.save_config_req)
    {
        int write_len;

        write_len=write(fd_config, &boatpilot_config_udp, sizeof(struct T_CONFIG));
        printf("config写入了%d个字节的数据\n",write_len);

        boatpilot_config_udp_previous = boatpilot_config_udp;
        global_bool_boatpilot.save_config_req=FALSE;
    }
}

void Boat::read_device_gps()
{

}

void Boat::read_device_IMU_mpu6050()
{

}

void Boat::read_device_mpu6050()
{

}

void Boat::set_device_rc_out()
{
//    float rc_raw_out_0;
//    float rc_raw_out_1;
//    float rc_raw_out_2;
//    float rc_raw_out_3;
//    float rc_raw_out_4;
//    float rc_raw_out_5;
//    float rc_raw_out_6;
//    float rc_raw_out_7;
//    float rc_raw_out_8;
//
//    rc_raw_out_0 = all_external_device_output.rc_raw_out_0;
//    rc_raw_out_1 = all_external_device_output.rc_raw_out_1;
//    rc_raw_out_2 = all_external_device_output.rc_raw_out_2;
//    rc_raw_out_3 = all_external_device_output.rc_raw_out_3;
//    rc_raw_out_4 = all_external_device_output.rc_raw_out_4;
//    rc_raw_out_5 = all_external_device_output.rc_raw_out_5;
//    rc_raw_out_6 = all_external_device_output.rc_raw_out_6;
//    rc_raw_out_7 = all_external_device_output.rc_raw_out_7;

    //然后把rc_raw_out_0输出给舵机或者电机，频率是50hz，勿删
}

void Boat::set_device_gpio()
{

}

void Boat::update_navigation_loop()
{
    navigation_loop();

    global_bool_boatpilot.current_to_target_radian    = (short)(navi_output.current_to_target_radian * 100.0);
    global_bool_boatpilot.current_to_target_degree    = (short)(navi_output.current_to_target_degree * 100);
    global_bool_boatpilot.command_course_radian       = (short)(navi_output.command_course_angle_radian * 100.0);
    global_bool_boatpilot.command_course_degree       = (short)(navi_output.command_course_angle_degree * 100.0);
    global_bool_boatpilot.wp_next                     = navi_output.current_target_wp_cnt;
}

void Boat::loop_one_second()
{
    //DEBUG_PRINTF("Hello loop_slow\n");

    //print_data_gps_Y901(); // do not delete, test for GPS_JY901

    //DEBUG_PRINTF("GPS_DATA_UM220: longitude:=%lld, latitude:%lld \n  gps.nshemi = %c, gps.ewhemi = %c, gps.gpssta = %c gps.posslnum = %d\n", \
                 gps_data_UM220.longitude, gps_data_UM220.latitude, \
                 gps_data_UM220.nshemi, gps_data_UM220.ewhemi, gps_data_UM220.gpssta, \
                 gps_data_UM220.posslnum); // do not delete, test for GPS signal


    //DEBUG_PRINTF("RC rudder    := %d, RC throttle    := %d \n", gcs2ap_all_udp.cmd.rudder, gcs2ap_all_udp.cmd.throttle);
    //DEBUG_PRINTF("ctrlout rudder := %d, ctrlout throttle := %d \n", (int)ctrloutput.rudder_pwm, (int)ctrloutput.mmotor_onoff_pwm);
    //DEBUG_PRINTF("left motor = %4.2f, right motor = %4.2f \n", global_bool_boatpilot.motor_left, global_bool_boatpilot.motor_right);

    //DEBUG_PRINTF("left volatage = %4.2f, right voltage = %4.2f \n", global_bool_boatpilot.voltage0, global_bool_boatpilot.voltage0);
}

void Boat::write_motors_device_init()
{
    set_motors_init();
}
void Boat::relay_switch_init()
{
    unsigned char send;

    send = 0xfa;
    PFC8574_DO(PCF8574_DO1_ADDR, send);

    send = 0x0f;
    PFC8574_DO(PCF8574_DO2_ADDR, send);
}


void Boat::disarm_motors()
{
    set_motor_off();
}



void Boat::write_device_II2C_test()
{
    unsigned char send_buf[256];
    float voltage;

    send_buf[0] = 0x0a;
    PFC8574_DO(PCF8574_DO1_ADDR, send_buf[0]);

    send_buf[0] = 0x0f;
    PFC8574_DO(PCF8574_DO2_ADDR, send_buf[0]);

    voltage = 3.0f;
    DAC7574_DA(DAC7574_DA1_ADDR, DA_CHANNEL_0, voltage);

    voltage = 4.0f;
    DAC7574_DA(DAC7574_DA1_ADDR, DA_CHANNEL_1, voltage);

    voltage = 3.0f;
    DAC7574_DA(DAC7574_DA1_ADDR, DA_CHANNEL_2, voltage);

    voltage = 3.0f;
    DAC7574_DA(DAC7574_DA1_ADDR, DA_CHANNEL_3, voltage);


    voltage = 4.5f;
    DAC7574_DA(DAC7574_DA2_ADDR, DA_CHANNEL_0, voltage);

    voltage = 4.5f;
    DAC7574_DA(DAC7574_DA2_ADDR, DA_CHANNEL_1, voltage);


    voltage = 4.5f;
    DAC7574_DA(DAC7574_DA2_ADDR, DA_CHANNEL_2, voltage);

    voltage = 4.5f;
    DAC7574_DA(DAC7574_DA2_ADDR, DA_CHANNEL_3, voltage);

}

void Boat::start_central_control_computer()
{


}

void Boat::end_of_task()
{
	//DEBUG_PRINTF("Hello end_of_task\n");
}
