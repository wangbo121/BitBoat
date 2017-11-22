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
        printf("可以读取航点或者创建航点文件\n");
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
        printf("可以读取配置或者创建配置文件\n");

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
        //printf("global_bool_boatpilot.wp_total_num=%d\n",global_bool_boatpilot.wp_total_num);//20170508已测试

    }

#ifdef __RADIO_
    radio_uart_init();
#endif

#ifdef __GPS_
    gps_uart_init();
#endif

#ifdef __MODBUS_
//    modbus_uart_init();//继电器，模拟量设备485串口初始化
//    modbus_rotary_uart_init();//码盘设备485串口初始化
#endif

#ifdef __UDP_
    //open_udp_dev(IP_MASTER, UDP_SERVER_PORT, UDP_M_RECV_PORT);
//    open_udp_dev(IP_SLAVER, UDP_SERVER_PORT, UDP_M_RECV_PORT);
#endif

#ifdef __BD_
//    bd_uart_init();
#endif

    /*
     * 设置继电器初始状态都置为0，也就是继电器的线圈不通电的状态
     */
//    set_switch_init();

    /*
     * 设置推进器所采用的模拟量通道
     */
//    set_switch.switch10_state=0xff00;
//    set_switch.switch14_state=0xff00;
//    global_bool_modbus.send_request_switch_cnt++;

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
    gcs2ap_radio_all.arrive_radius=10;//初始到达半径设置为100米
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
    //gcs2ap_radio_all.turn_mode=1;//1:rudder 2:diffspd差速 3:方向舵和差速混合
    //gcs2ap_radio_all.turn_mode=2;//1:rudder 2:diffspd差速 3:方向舵和差速混合
    gcs2ap_radio_all.rudder_dead_zone_angle_degree=3;
    gcs2ap_radio_all.diffspd_coef=100;
    gcs2ap_radio_all.diffspd_lim=10;

    /*
     * 下面是全局变量global_bool_boatpilot的初始化
     * global_bool_boatpilot和gcs2ap_radio_all的区别在于后者是从地面站发送过来的参数
     * 但是一些标志量用global_bool_boatpilot中的bool表示
     */
    global_bool_boatpilot.radio_send_time=clock_gettime_s();/*从系统开启的到当前的时间，作为程序运行时间的开始计时*/
    global_bool_boatpilot.bool_gcs2ap_beidou=1;//20170419电台和北斗，同时接收并解析命令包，同时发送实时数据包
    global_bool_boatpilot.rudder_middle_position=368.0;
    global_bool_boatpilot.rudder_left_limit_position=368.0*0.5;
    global_bool_boatpilot.rudder_right_limit_position=368.0*0.5;
    global_bool_boatpilot.turn_mode=1;////转弯方式，0:方向舵，1:差速 2:方向舵和差速同时混合转弯

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
    //printf("电台--请求发送实时数据给地面站\n");//20170410已测试，地面站能够接收所有实时数据
    global_bool_boatpilot.ap2gcs_real_cnt++;
    if(global_bool_boatpilot.ap2gcs_real_cnt_previous!=global_bool_boatpilot.ap2gcs_real_cnt)
    {
        //发送实时数据
        send_ap2gcs_real();

        global_bool_boatpilot.ap2gcs_real_cnt_previous=global_bool_boatpilot.ap2gcs_real_cnt;
        global_bool_boatpilot.send_ap2gcs_real_req=FALSE;
    }

}

void Boat::record_config()
{
    if(global_bool_boatpilot.assign_config_req)
    {
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

}

void Boat::record_log()
{
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

}
