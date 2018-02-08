/*
 * Boat.cpp
 *
 *  Created on: Nov 6, 2017
 *      Author: wangbo
 */

#include "Boat.h"

Boat boat;

int fd_boatpilot_log;
int fd_waypoint;
int fd_config;

void Boat::setup( void )
{

    /*
     * 创建要保存的二进制日志文件boatpilot_log
     */
    fd_boatpilot_log=create_log_file(BOATPILOT_LOG_FILE);
    //printf("setup fd =%d\n",fd_boatpilot_log);// 20180208已测试

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
    fd_config=load_data_struct_from_binary(CONFIG_FILE,&boatpilot_config,sizeof(boatpilot_config));
    if(fd_config==-1)
    {
        printf("无法创建配置文件\n");
    }
    else
    {
        printf("可以读取配置或者创建配置文件 fd = %d\n",fd_config);

        gcs2ap_radio_all.workmode=boatpilot_config.work_mode;
        gcs2ap_radio_all.rud_p=boatpilot_config.rud_p;
        gcs2ap_radio_all.rud_i=boatpilot_config.rud_i;
        gcs2ap_radio_all.rud_d=boatpilot_config.rud_d;
        gcs2ap_radio_all.cte_p=boatpilot_config.cte_p;
        gcs2ap_radio_all.cte_i=boatpilot_config.cte_i;
        gcs2ap_radio_all.cte_d=boatpilot_config.cte_d;
        gcs2ap_radio_all.rudder_setup_reverse=boatpilot_config.rudder_setup_reverse;
        gcs2ap_radio_all.thruster_setup_reverse=boatpilot_config.thruster_setup_reverse;
        gcs2ap_radio_all.cruise_throttle_percent=boatpilot_config.cruise_throttle_percent;
        gcs2ap_radio_all.throttle_change_time=boatpilot_config.throttle_change_time;
        gcs2ap_radio_all.arrive_radius=boatpilot_config.arrive_radius;
        gcs2ap_radio_all.cte_max_degree=boatpilot_config.cte_max_degree;
        gcs2ap_radio_all.rudder_left_pos=boatpilot_config.rudder_left_pos;
        gcs2ap_radio_all.rudder_right_pos=boatpilot_config.rudder_right_pos;
        gcs2ap_radio_all.rudder_mid_pos=boatpilot_config.rudder_mid_pos;

        gcs2ap_radio_all.set_switch_channel=boatpilot_config.set_switch_channel;
        gcs2ap_radio_all.set_switch_low_limit=boatpilot_config.set_switch_low_limit;
        gcs2ap_radio_all.set_switch_high_limit=boatpilot_config.set_switch_high_limit;
        gcs2ap_radio_all.set_charge_channel=boatpilot_config.set_charge_channel;
        gcs2ap_radio_all.set_charge_voltage=boatpilot_config.set_charge_voltage;
        gcs2ap_radio_all.set_charge_current=boatpilot_config.set_charge_current;
        gcs2ap_radio_all.rudder_dead_zone_angle_degree=boatpilot_config.rudder_dead_zone_angle_degree;

        global_bool_boatpilot.wp_next=boatpilot_config.current_target_wp_num;
        global_bool_boatpilot.wp_total_num=boatpilot_config.total_wp_num;
    }

#ifdef __RADIO_
    radio_uart_init();
#endif

#ifdef __GPS_
    gps_uart_init();
#endif

    /*
     * 初始化导航环节
     */
    navigation_init();

    /*
     * 初始化自驾仪控制量输入以及参数的初始值，
     * 下面所有的设置都是从地面站获取后使用
     * 尤其注意初始的方向舵和油门量值
     * 控制量的限幅参数，
     * 这些参数都是unsigned char 8位无符号
     */
    gcs2ap_radio_all.rud_p=20;//rud_p单位是[0.1]所以一开始要赋值大一些
    gcs2ap_radio_all.arrive_radius=10;//单位是[10米]，初始到达半径设置为100米
    gcs2ap_radio_all.mmotor_off_pos=0;
    gcs2ap_radio_all.mmotor_on_pos=255;
    gcs2ap_radio_all.rudder_left_pos=0;
    gcs2ap_radio_all.rudder_right_pos=255;
    gcs2ap_radio_all.rudder_mid_pos=127;
    gcs2ap_radio_all.rc_thruster=0;
    gcs2ap_radio_all.rc_rudder=127;
    gcs2ap_radio_all.cruise_throttle_percent=50;//初始巡航油门设置为百分之50
    gcs2ap_radio_all.cte_max_degree=5;/*初始化偏航距的最大修正角度*/
    gcs2ap_radio_all.throttle_change_time=5;/*油门改变10%所消耗的时间[秒]*/
    gcs2ap_radio_all.navigation_mode=NAVIGATION_COURSE_ANGLE;
    gcs2ap_radio_all.rudder_dead_zone_angle_degree=3;
    gcs2ap_radio_all.diffspd_coef=100;
    gcs2ap_radio_all.diffspd_lim=10;

    /*
     * 下面是全局变量global_bool_boatpilot的初始化
     * global_bool_boatpilot和gcs2ap_radio_all的区别在于后者是从地面站发送过来的参数
     * 但是一些标志量用global_bool_boatpilot中的bool表示
     */
    global_bool_boatpilot.radio_send_time=clock_gettime_s();/*从系统开启的到当前的时间，作为程序运行时间的开始计时*/
    global_bool_boatpilot.rudder_middle_position=368.0;
    global_bool_boatpilot.rudder_left_limit_position=368.0*0.5;
    global_bool_boatpilot.rudder_right_limit_position=368.0*0.5;
    global_bool_boatpilot.turn_mode=1;////转弯方式，0:方向舵，1:差速 2:方向舵和差速同时混合转弯

    global_bool_boatpilot.save_boatpilot_log_req = TRUE;

    /*
     * 设置PID控制器的参数
     */
    pid_yaw.set_kP(2);
    pid_yaw.set_kI(0);
    pid_yaw.set_kD(0);
    pid_yaw.set_imax(0.174*3);//30度

    pid_CTE.set_kP(2);
    pid_CTE.set_kI(0);
    pid_CTE.set_kD(0);
    pid_CTE.set_imax(0.174*3);//30度
}

void Boat::send_ap2gcs_cmd_boatlink()
{
    if(global_bool_boatpilot.send_ap2gcs_cmd_req)
    {
        //printf("电台--请求发送命令数据给地面站\n");//20170410已测试，地面站能够接收所有回传命令
        global_bool_boatpilot.ap2gcs_cmd_cnt++;
        if(global_bool_boatpilot.ap2gcs_cmd_cnt_previous!=global_bool_boatpilot.ap2gcs_cmd_cnt)
        {
            //发送/回传命令给地面站
            send_ap2gcs_cmd();

            global_bool_boatpilot.ap2gcs_cmd_cnt_previous=global_bool_boatpilot.ap2gcs_cmd_cnt;
            global_bool_boatpilot.send_ap2gcs_cmd_req=FALSE;
        }
    }
}

void Boat::send_ap2gcs_wp_boatlink()
{
    if(global_bool_boatpilot.send_ap2gcs_wp_req)
    {
        printf("电台--请求发送航点数据给地面站\n");
        global_bool_boatpilot.ap2gcs_wp_cnt++;

        if(global_bool_boatpilot.ap2gcs_wp_cnt_previous!=global_bool_boatpilot.ap2gcs_wp_cnt)
        {
            ap2gcs_wp.pack_func_info3=global_bool_boatpilot.ap2gcs_wp_cnt;

            if(global_bool_boatpilot.send_ap2gcs_wp_end_num>=global_bool_boatpilot.wp_total_num-1)
            {
                global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.wp_total_num-1;
            }
            send_ap2gcs_waypoint_num(global_bool_boatpilot.send_ap2gcs_wp_start_num,global_bool_boatpilot.send_ap2gcs_wp_end_num-global_bool_boatpilot.send_ap2gcs_wp_start_num+1);

            global_bool_boatpilot.ap2gcs_wp_cnt_previous=global_bool_boatpilot.ap2gcs_wp_cnt;
            global_bool_boatpilot.send_ap2gcs_wp_req=FALSE;
        }
    }
}

void Boat::send_ap2gcs_realtime_data_boatlink()
{
    if( (!global_bool_boatpilot.send_ap2gcs_wp_req) && (!global_bool_boatpilot.send_ap2gcs_cmd_req) )
    {
    	//printf("电台--请求发送实时数据给地面站\n");//20170410已测试，地面站能够接收所有实时数据
		global_bool_boatpilot.ap2gcs_real_cnt++;
		if(global_bool_boatpilot.ap2gcs_real_cnt_previous!=global_bool_boatpilot.ap2gcs_real_cnt)
		{
			//发送实时数据
			//DEBUG_PRINTF("发送实时数据\n");
			send_ap2gcs_real();

			global_bool_boatpilot.ap2gcs_real_cnt_previous=global_bool_boatpilot.ap2gcs_real_cnt;
			global_bool_boatpilot.send_ap2gcs_real_req=FALSE;
		}
    }
}

void Boat::record_config()
{
    if(global_bool_boatpilot.assign_config_req)
    {
    	//printf("更新config fd_config = %d \n",fd_config);
        global_bool_boatpilot.assign_config_cnt++;
        if(global_bool_boatpilot.assign_config_cnt_previous!=global_bool_boatpilot.assign_config_cnt)
        {
            boatpilot_config.work_mode=gcs2ap_radio_all.workmode;
            boatpilot_config.rud_p=gcs2ap_radio_all.rud_p;
            boatpilot_config.rud_i=gcs2ap_radio_all.rud_i;
            boatpilot_config.rud_d=gcs2ap_radio_all.rud_d;
            boatpilot_config.cte_p=gcs2ap_radio_all.cte_p;
            boatpilot_config.cte_i=gcs2ap_radio_all.cte_i;
            boatpilot_config.cte_d=gcs2ap_radio_all.cte_d;
            boatpilot_config.rudder_setup_reverse=gcs2ap_radio_all.rudder_setup_reverse;
            boatpilot_config.thruster_setup_reverse=gcs2ap_radio_all.thruster_setup_reverse;
            boatpilot_config.cruise_throttle_percent=gcs2ap_radio_all.cruise_throttle_percent;
            boatpilot_config.throttle_change_time=gcs2ap_radio_all.throttle_change_time;
            boatpilot_config.arrive_radius=gcs2ap_radio_all.arrive_radius;
            boatpilot_config.cte_max_degree=gcs2ap_radio_all.cte_max_degree;
            boatpilot_config.rudder_left_pos=gcs2ap_radio_all.rudder_left_pos;
            boatpilot_config.rudder_right_pos=gcs2ap_radio_all.rudder_right_pos;
            boatpilot_config.rudder_mid_pos=gcs2ap_radio_all.rudder_mid_pos;

            boatpilot_config.set_switch_channel=gcs2ap_radio_all.set_switch_channel;
            boatpilot_config.set_switch_low_limit=gcs2ap_radio_all.set_switch_low_limit;
            boatpilot_config.set_switch_high_limit=gcs2ap_radio_all.set_switch_high_limit;
            boatpilot_config.set_charge_channel=gcs2ap_radio_all.set_charge_channel;
            boatpilot_config.set_charge_voltage=gcs2ap_radio_all.set_charge_voltage;
            boatpilot_config.set_charge_current=gcs2ap_radio_all.set_charge_current;
            boatpilot_config.rudder_dead_zone_angle_degree=gcs2ap_radio_all.rudder_dead_zone_angle_degree;

            boatpilot_config.current_target_wp_num=global_bool_boatpilot.wp_next;
            boatpilot_config.total_wp_num=global_bool_boatpilot.wp_total_num;

            global_bool_boatpilot.assign_config_cnt_previous=global_bool_boatpilot.assign_config_cnt;
            global_bool_boatpilot.assign_config_req=FALSE;
        }
    }

    if(memcmp(&boatpilot_config_previous,&boatpilot_config,sizeof(boatpilot_config))==0)
    {
        //printf("config没有变化，不保存\n");
    }
    else
    {
        printf("config发生了变化，需要重新保存\n");//20170413已测试，可以在参数更改后保存boatpilot_config
        global_bool_boatpilot.save_config_req=TRUE;
    }
    if(global_bool_boatpilot.save_config_req)
    {
        int write_len;

        write_len=write(fd_config,&boatpilot_config,sizeof(struct T_CONFIG));
        printf("config写入了%d个字节的数据\n",write_len);

        boatpilot_config_previous=boatpilot_config;
        global_bool_boatpilot.save_config_req=FALSE;
    }
}

void Boat::record_wp()
{
	int write_len;

	if(global_bool_boatpilot.save_wp_req)
	{
		write_len=write(fd_waypoint,(char *)wp_data,sizeof(struct WAY_POINT)*MAX_WAYPOINT_NUM);
		global_bool_boatpilot.save_wp_req = FALSE;
	}
}

void Boat::record_log()
{
	//DEBUG_PRINTF("记录日志\n");
	global_bool_boatpilot.save_boatpilot_log_req = TRUE;
    if(global_bool_boatpilot.save_boatpilot_log_req)
    {
        /*时间戳*/
        boatpilot_log.year=datetime_now.year;
        boatpilot_log.month=datetime_now.month;
        boatpilot_log.day=datetime_now.day;
        boatpilot_log.hour=datetime_now.hour;
        boatpilot_log.minute=datetime_now.minute;
        boatpilot_log.second=datetime_now.second;
        boatpilot_log.stuffing=datetime_now.stuffing;

        memcpy(&boatpilot_log.ap2gcs_real,&ap2gcs_real,sizeof(ap2gcs_real));
        memcpy(&boatpilot_log.gcs2ap_radio,&gcs2ap_radio_all,sizeof(gcs2ap_radio_all));
        memcpy(&boatpilot_log.global,&global_bool_boatpilot.bool_get_gcs2ap_cmd,sizeof(boatpilot_log.global));

        /*
         * 保存日志文件到二进制文件boatpilot_log
         * 尽量把sizeof(boatpilot_log)作为4字节对齐
         * 别删除！！20170508 sizeof(boatpilot_log)=244 sizeof(boatpilot_log.global)=92
         *
         */
        //printf("sizeof(boatpilot_log)=%d,sizeof(boatpilot_log.global)=%d\n",sizeof(boatpilot_log),sizeof(boatpilot_log.global));
        //printf("fd_boatpilot_log = %d\n",fd_boatpilot_log);
        save_data_to_binary_log(fd_boatpilot_log,&boatpilot_log,sizeof(boatpilot_log));

        global_bool_boatpilot.save_boatpilot_log_req=FALSE;
    }
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
	 * 之所以增加all_external_device_input这一层，是为了增加模块化，不用考虑外部的数据
	 */

	/*
	 * 这里是获取遥控器的信号
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
	* 上面的all_external_device_input其实应该是由外部设备有数据更新后把数据
	* 赋值给all_external_device_input，而我的飞控只是从这里获取数据，不用管数据是否更新
	* 而且我只是从这里读取数据，应该不会出现同时写某一个变量的情况吧
	* 但是这种方式有可能出现这边在写内存，而另一边在读内存，这个概率有多大又会造成什么影响呢
	* 上面的这些赋值应该是由王正阳从设备驱动那里获取数据值
	* 实际使用时，上面的需要删除掉我这里并不需要
	* 我需要的是下面的从all_external_device_input获取数据
	*/


	/*
	 * 这里使用仿真sim_water_craft的数据
	 */
	all_external_device_input.latitude = fdm.latitude;
	all_external_device_input.longitude = fdm.longitude;
	all_external_device_input.altitude = 10 ;
	all_external_device_input.v_north = fdm.speedN;
	all_external_device_input.v_east = fdm.speedE;
	all_external_device_input.v_down = fdm.speedD;
	all_external_device_input.heading = fdm.heading;
}

void Boat::set_rc_out()
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

void Boat::update_GPS()
{
	gps_data.latitude =(int)( all_external_device_input.latitude * 1e5);
	gps_data.longitude = (int)(all_external_device_input.longitude * 1e5);

	gps_data.course = all_external_device_input.heading * DEG_TO_RAD;
	gps_data.yaw = (int)(all_external_device_input.heading * 1e2);

	gps_data.velocity_north = (int)(all_external_device_input.v_north *1e3);
	gps_data.velocity_east = (int)(all_external_device_input.v_east * 1e3);
}
