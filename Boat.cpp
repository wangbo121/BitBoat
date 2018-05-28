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
    fd_boatpilot_log=create_log_file(BOATPILOT_LOG_FILE);

    /*
     * 载入航点文件waypoint
     */
    fd_waypoint=load_data_struct_from_binary(WAY_POINT_FILE,&wp_data,sizeof(wp_data));
    if(fd_waypoint==-1)
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
    fd_config=load_data_struct_from_binary(CONFIG_FILE,&boatpilot_config_udp,sizeof(boatpilot_config_udp));
    if(fd_config==-1)
    {
        printf("无法创建配置文件\n");
    }
    else
    {
        printf("可以读取配置或者创建配置文件 fd = %d\n",fd_config);

        gcs2ap_all_udp.workmode=boatpilot_config_udp.work_mode;
        gcs2ap_all_udp.rud_p=boatpilot_config_udp.rud_p;
        gcs2ap_all_udp.rud_i=boatpilot_config_udp.rud_i;
        gcs2ap_all_udp.rud_d=boatpilot_config_udp.rud_d;
        gcs2ap_all_udp.cte_p=boatpilot_config_udp.cte_p;
        gcs2ap_all_udp.cte_i=boatpilot_config_udp.cte_i;
        gcs2ap_all_udp.cte_d=boatpilot_config_udp.cte_d;
        gcs2ap_all_udp.cruise_throttle_percent=boatpilot_config_udp.cruise_throttle_percent;
        gcs2ap_all_udp.throttle_change_time=boatpilot_config_udp.throttle_change_time;
        gcs2ap_all_udp.arrive_radius=boatpilot_config_udp.arrive_radius;
        gcs2ap_all_udp.cte_max_degree=boatpilot_config_udp.cte_max_degree;
        gcs2ap_all_udp.rudder_left_pos=boatpilot_config_udp.rudder_left_pos;
        gcs2ap_all_udp.rudder_right_pos=boatpilot_config_udp.rudder_right_pos;
        gcs2ap_all_udp.rudder_mid_pos=boatpilot_config_udp.rudder_mid_pos;
        global_bool_boatpilot.wp_next=boatpilot_config_udp.current_target_wp_num;
        global_bool_boatpilot.wp_total_num=boatpilot_config_udp.total_wp_num;
    }

#ifdef __RADIO_
    radio_uart_init();
#endif

#ifdef __GPS_
    //gps_uart_init();
    gps_uart_init_Y901();
#endif

    /*
     * 初始化导航环节
     */
    navigation_init();

    /*
     * 初始化自驾仪控制量输入以及参数的初始值，
     * 下面所有的设置都是从地面站获取后使用
     * 尤其注意初始的方向舵和油门量值
     * 控制量的限幅参数
     */
    gcs2ap_all_udp.rud_p=20;//rud_p单位是[0.1]所以一开始要赋值大一些
	gcs2ap_all_udp.arrive_radius=10;//单位是[10米]，初始到达半径设置为100米
	gcs2ap_all_udp.mmotor_off_pos=0;
	gcs2ap_all_udp.mmotor_on_pos=255;
	gcs2ap_all_udp.rudder_left_pos=0;
	gcs2ap_all_udp.rudder_right_pos=255;
	gcs2ap_all_udp.rudder_mid_pos=127;
	gcs2ap_all_udp.rc_thruster=0;
	gcs2ap_all_udp.rc_rudder=127;
	gcs2ap_all_udp.cruise_throttle_percent=50;//初始巡航油门设置为百分之50
	gcs2ap_all_udp.cte_max_degree=5;/*初始化偏航距的最大修正角度*/
	gcs2ap_all_udp.throttle_change_time=5;/*油门改变10%所消耗的时间[秒]*/
	gcs2ap_all_udp.navigation_mode=NAVIGATION_COURSE_ANGLE;
	gcs2ap_all_udp.rudder_dead_zone_angle_degree=3;
	gcs2ap_all_udp.diffspd_coef=100;
	gcs2ap_all_udp.diffspd_lim=10;

    /*
     * 下面是全局变量global_bool_boatpilot的初始化
     * global_bool_boatpilot和gcs2ap_all_udp的区别在于后者是从地面站发送过来的参数，
     * 而global_bool_boatpilot是驾驶仪内部需要的全局计算量
     * 一些标志量用global_bool_boatpilot中的bool表示
     */
    global_bool_boatpilot.turn_mode=1;////转弯方式，0:方向舵，1:差速 2:方向舵和差速同时混合转弯
    global_bool_boatpilot.save_boatpilot_log_req = TRUE;

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
     * 有1个网卡是用来监听地面站向自驾仪发送数据的
     * 与此同时，发送也是通过这个网卡向地面站回传实时数据
     */
    printf("Boat::setup    :    fd_socket_generic = %d \n",fd_socket_generic);
    open_socket_udp_dev(&fd_socket_generic, "AP_LISTEN_UDP_IP", AP_LISTEN_UDP_PORT);

#ifdef TEST
    unsigned int lng_start = 116.31574 * 1e5;
    unsigned int lat_start = 39.95635 * 1e5;
    unsigned char wp_total_num_test =5;
    for(int i=0; i< wp_total_num_test; i++)
    {
        wp_data[i].no = i;
        wp_data[i].type = 0;
        wp_data[i].spd = 50;
        wp_data[i].alt = 12;

        wp_data[i].lng = lng_start + 3e3 * i;
        //wp_data[i].lng = lng_start + 1e4 * i;
        //wp_data[i].lat = lat_start + 1e5;
        wp_data[i].lat = lat_start;

        printf("wp_data[%d].lng = %d \n",i,wp_data[i].lng);
        printf("wp_data[%d].lat = %d \n",i,wp_data[i].lat);
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

    DEBUG_PRINTF("当前系统时间是:%d年%d月%d日%d时%d分%d秒\n",datetime_now.year,datetime_now.month,datetime_now.day,datetime_now.hour,datetime_now.minute,datetime_now.second);
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

	/*
	 * 获取遥控器的信号
	 */
	all_external_device_input.rc_raw_in_0 = 1500;
	all_external_device_input.rc_raw_in_1 = 1500;
	all_external_device_input.rc_raw_in_2 = 1500;
	all_external_device_input.rc_raw_in_3 = 1500;
	all_external_device_input.rc_raw_in_4 = 1990;//绕航点飞行模式
	all_external_device_input.rc_raw_in_5 = 1500;
	all_external_device_input.rc_raw_in_6 = 1500;
	all_external_device_input.rc_raw_in_7 = 1500;
	all_external_device_input.rc_raw_in_8 = 1500;

	/*
	 * 这里使用仿真sim_water_craft的数据
	 * 如果有相应的硬件传感器则，应该是真实的传感器反馈数据
	 */
	all_external_device_input.latitude = fdm.latitude;
	all_external_device_input.longitude = fdm.longitude;
	all_external_device_input.altitude = 10 ;
	all_external_device_input.v_north = fdm.speedN;
	all_external_device_input.v_east = fdm.speedE;
	all_external_device_input.v_down = fdm.speedD;
	all_external_device_input.heading = fdm.heading;
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
	gps_data.latitude =(int)( all_external_device_input.latitude * 1e5);
	gps_data.longitude = (int)(all_external_device_input.longitude * 1e5);

	gps_data.course = all_external_device_input.heading * DEG_TO_RAD;
	gps_data.yaw = (int)(all_external_device_input.heading * 1e2);

	gps_data.velocity_north = (int)(all_external_device_input.v_north *1e3);
	gps_data.velocity_east = (int)(all_external_device_input.v_east * 1e3);
}

void Boat::update_mpu6050()
{

}

void Boat::out_execute_ctrloutput()
{
	execute_ctrloutput(&ctrloutput);
}

void Boat::record_wp()
{
	int write_len;

	if(global_bool_boatpilot.save_wp_req)
	{
		//write_len=write(fd_waypoint,(char *)wp_data,sizeof(struct WAY_POINT)*MAX_WAYPOINT_NUM);
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

        //printf("sizeof(boatpilot_log)=%d,sizeof(boatpilot_log.global)=%d\n",sizeof(boatpilot_log),sizeof(boatpilot_log.global));
        //printf("fd_boatpilot_log = %d\n",fd_boatpilot_log);
        save_data_to_binary_log(fd_boatpilot_log, &boatpilot_log, sizeof(boatpilot_log));

        global_bool_boatpilot.save_boatpilot_log_req=FALSE;
    }
}

void Boat::record_config()
{

}

void Boat::read_device_gps()
{
    //read_gps_data_Y901();

}

void Boat::read_device_mpu6050()
{

}

void Boat::set_device_rc_out()
{
    float rc_raw_out_0;
    float rc_raw_out_1;
    float rc_raw_out_2;
    float rc_raw_out_3;
    float rc_raw_out_4;
    float rc_raw_out_5;
    float rc_raw_out_6;
    float rc_raw_out_7;
    float rc_raw_out_8;

    rc_raw_out_0 = all_external_device_output.rc_raw_out_0;
    rc_raw_out_1 = all_external_device_output.rc_raw_out_1;
    rc_raw_out_2 = all_external_device_output.rc_raw_out_2;
    rc_raw_out_3 = all_external_device_output.rc_raw_out_3;
    rc_raw_out_4 = all_external_device_output.rc_raw_out_4;
    rc_raw_out_5 = all_external_device_output.rc_raw_out_5;
    rc_raw_out_6 = all_external_device_output.rc_raw_out_6;
    rc_raw_out_7 = all_external_device_output.rc_raw_out_7;

    //然后把rc_raw_out_0输出给舵机或者电机，频率是50hz，勿删
}

void Boat::set_device_gpio()
{

}

void Boat::loop_one_second()
{
    //DEBUG_PRINTF("Hello loop_slow\n");
    //printf("gcs2ap_all_udp.workmode    :    %d \n", gcs2ap_all_udp.workmode);

    //print_data_gps_Y901();
    //DEBUG_PRINTF("GPS_DATA: \n");
    //DEBUG_PRINTF("gps_data.gps_data.longitude: %d\n",gps_data.longitude);
    //DEBUG_PRINTF("gps_data.gps_data.latitude: %d\n",gps_data.latitude);


    //DEBUG_PRINTF("global_bool_boatpilot.wp_next: %d\n",global_bool_boatpilot.wp_next);
    //DEBUG_PRINTF("global_bool_boatpilot.cnt_test : %d\n",global_bool_boatpilot.cnt_test );

    DEBUG_PRINTF("gcs2ap_cmd_udp.func_info2    :    =%d \n", gcs2ap_cmd_udp.func_info2);


}

void Boat::end_of_task()
{
	//DEBUG_PRINTF("Hello end_of_task\n");
}
