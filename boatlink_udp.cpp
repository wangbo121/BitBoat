/*
 * boatlink_udp.cpp
 *
 *  Created on: 2018-3-6
 *      Author: wangbo
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*转换int或者short的字节顺序，该程序arm平台为大端模式，地面站x86架构为小端模式*/
#include <byteswap.h>
#include <unistd.h>

#include <fcntl.h>//创建文件
#include <radio.h>
#include <sys/stat.h>

#include "global.h"
#include "uart.h"
#include "control.h"
#include "navigation.h"
#include "gps.h"
#include "utility.h"
#include "save_data.h"
#include "servo.h"
#include "location.h"
#include "Boat.h"
#include "udp.h"

#include "boatlink_udp.h"

struct AP2GCS_REAL_UDP ap2gcs_real_udp;
struct GCS2AP_CMD_UDP gcs2ap_cmd_udp;
struct GCS2AP_ALL_UDP gcs2ap_all_udp;
struct T_CONFIG_UDP boatpilot_config_udp;

#define REAL_DATA_BUF_SIZE 256
int send_ap2gcs_real_udp()
{
    unsigned char real[REAL_DATA_BUF_SIZE];
    int ret;
    static unsigned int real_udp_cnt;

    real_udp_cnt++;
    ap2gcs_real_udp.head1 = 0xaa;
    ap2gcs_real_udp.head2 = 0x55;
    ap2gcs_real_udp.len = 76;
    ap2gcs_real_udp.type = COMMAND_AP2GCS_REAL_UDP;
    ap2gcs_real_udp.vessel = 1;  //发送数据的船的ID编号，从1开始，最多到100,0为无效船
    ap2gcs_real_udp.master_ap_link_ack = 0x10;//D7:主/备自驾仪(1主,0备);D6~4:自驾仪编号(0..7);D3~1:链路编号(0:局域网;1:北斗;2:串口数传电台);D0:是否需要返回确认包(1:需要;0:不需)
    ap2gcs_real_udp.plan_id = 0;//采用的航线编号，从1开始，最大为10，0表示没有航线

    ap2gcs_real_udp.cnt = real_udp_cnt;
    ap2gcs_real_udp.pack_func_flag=0;
    ap2gcs_real_udp.pack_func_info1 = 0;
    ap2gcs_real_udp.pack_func_info2 = 0;
    ap2gcs_real_udp.lng = gps_data.longitude;
    ap2gcs_real_udp.lat = gps_data.latitude;
    ap2gcs_real_udp.spd = gps_data.speed;
    ap2gcs_real_udp.dir_gps=(short)(convert_radian_to_degree(gps_data.course))*100;
    ap2gcs_real_udp.dir_heading=(short)gps_data.yaw;
    ap2gcs_real_udp.dir_target=global_bool_boatpilot.current_to_target_degree;
    ap2gcs_real_udp.dir_nav=global_bool_boatpilot.command_course_degree;

    ap2gcs_real_udp.roll=(short)gps_data.roll;
    ap2gcs_real_udp.pitch=(short)gps_data.pitch;
    ap2gcs_real_udp.yaw=(short)gps_data.yaw;

    ap2gcs_real_udp.wp_next = global_bool_boatpilot.wp_next;
    //ap2gcs_real_udp.wp_next = 99;
    ap2gcs_real_udp.sail_mode = gcs2ap_all_udp.sail_mode;
    ap2gcs_real_udp.form_type = gcs2ap_all_udp.formation_type;
    ap2gcs_real_udp.pilot_vessel = 0;

    printf("send_ap2gcs_real_udp    :    sizeof (struct AP2GCS_REAL_UDP) = %ld \n",sizeof (struct AP2GCS_REAL_UDP));
    memcpy(real, &ap2gcs_real_udp, sizeof (struct AP2GCS_REAL_UDP));
    ret = sizeof (struct AP2GCS_REAL_UDP);
    send_socket_udp_data(fd_socket_generic, real, ret, AP_SENDTO_UDP_IP, AP_SENDTO_UDP_PORT );

    return 0;
}

static int decode_gcs2ap_cmd_udp(struct GCS2AP_ALL_UDP *ptr_gcs2ap_all_udp, struct GCS2AP_CMD_UDP *ptr_gcs2ap_cmd_udp);

int decode_gcs2ap_udp()
{
	/*1 decode gcs2ap_cmd*/
	if (global_bool_boatpilot.bool_get_gcs2ap_cmd)
	{
		decode_gcs2ap_cmd_udp(&gcs2ap_all_udp, &gcs2ap_cmd_udp);
		global_bool_boatpilot.bool_get_gcs2ap_cmd = FALSE;
	}

	return 0;
}

static int decode_gcs2ap_cmd_udp(struct GCS2AP_ALL_UDP *ptr_gcs2ap_all_udp, struct GCS2AP_CMD_UDP *ptr_gcs2ap_cmd_udp)
{
	unsigned char temp;
	ptr_gcs2ap_all_udp->head1 = ptr_gcs2ap_cmd_udp->head1;
	ptr_gcs2ap_all_udp->head2 = ptr_gcs2ap_cmd_udp->head2;
	ptr_gcs2ap_all_udp->len = ptr_gcs2ap_cmd_udp->len;
	ptr_gcs2ap_all_udp->type = ptr_gcs2ap_cmd_udp->type;
	ptr_gcs2ap_all_udp->vessel_ID = ptr_gcs2ap_cmd_udp->vessel_ID;
	ptr_gcs2ap_all_udp->master_ap_link_ack = ptr_gcs2ap_cmd_udp->master_ap_link_ack;
	ptr_gcs2ap_all_udp->gcs_ID = ptr_gcs2ap_cmd_udp->gcs_ID;
	ptr_gcs2ap_all_udp->cnt = ptr_gcs2ap_cmd_udp->cnt;
	ptr_gcs2ap_all_udp->func_flag = ptr_gcs2ap_cmd_udp->func_flag;
	ptr_gcs2ap_all_udp->func_info1 = ptr_gcs2ap_cmd_udp->func_info1;
	ptr_gcs2ap_all_udp->func_info2 = ptr_gcs2ap_cmd_udp->func_info2;
	ptr_gcs2ap_all_udp->auto_manu = ptr_gcs2ap_cmd_udp->auto_manu;
	ptr_gcs2ap_all_udp->throttle = ptr_gcs2ap_cmd_udp->throttle;
	ptr_gcs2ap_all_udp->rudder = ptr_gcs2ap_cmd_udp->rudder;
	ptr_gcs2ap_all_udp->fwdbwd = ptr_gcs2ap_cmd_udp->fwdbwd;
	ptr_gcs2ap_all_udp->controller_type = ptr_gcs2ap_cmd_udp->controller_type;
	ptr_gcs2ap_all_udp->ctrl_para_1 = ptr_gcs2ap_cmd_udp->ctrl_para_1;
	ptr_gcs2ap_all_udp->ctrl_para_2 = ptr_gcs2ap_cmd_udp->ctrl_para_2;
	ptr_gcs2ap_all_udp->ctrl_para_3 = ptr_gcs2ap_cmd_udp->ctrl_para_3;
	ptr_gcs2ap_all_udp->ctrl_para_4 = ptr_gcs2ap_cmd_udp->ctrl_para_4;
	ptr_gcs2ap_all_udp->ctrl_para_5 = ptr_gcs2ap_cmd_udp->ctrl_para_5;
	ptr_gcs2ap_all_udp->ctrl_para_6 = ptr_gcs2ap_cmd_udp->ctrl_para_6;
	ptr_gcs2ap_all_udp->ctrl_para_7 = ptr_gcs2ap_cmd_udp->ctrl_para_7;
	ptr_gcs2ap_all_udp->ctrl_para_8 = ptr_gcs2ap_cmd_udp->ctrl_para_8;
	ptr_gcs2ap_all_udp->ctrl_para_9 = ptr_gcs2ap_cmd_udp->ctrl_para_9;
	ptr_gcs2ap_all_udp->ctrl_para_10 = ptr_gcs2ap_cmd_udp->ctrl_para_10;
	ptr_gcs2ap_all_udp->ctrl_para_11 = ptr_gcs2ap_cmd_udp->ctrl_para_11;
	ptr_gcs2ap_all_udp->ctrl_para_12 = ptr_gcs2ap_cmd_udp->ctrl_para_12;
	ptr_gcs2ap_all_udp->sail_mode = ptr_gcs2ap_cmd_udp->sail_mode;
	ptr_gcs2ap_all_udp->wp_next = ptr_gcs2ap_cmd_udp->wp_next;
	ptr_gcs2ap_all_udp->sensor_correct = ptr_gcs2ap_cmd_udp->sensor_correct;
	ptr_gcs2ap_all_udp->pilot_vessel = ptr_gcs2ap_cmd_udp->pilot_vessel;

	if(ptr_gcs2ap_all_udp->master_ap_link_ack & 0x80)
	{
		ptr_gcs2ap_all_udp->pilot_type = 1;//1表示是主驾驶仪
	}
	ptr_gcs2ap_all_udp->pilot_cnt = ptr_gcs2ap_all_udp->master_ap_link_ack & 0x70;
	ptr_gcs2ap_all_udp->link_ID = ptr_gcs2ap_all_udp->master_ap_link_ack & 0x0e;
	if(ptr_gcs2ap_all_udp->master_ap_link_ack & 0x01)
	{
		//地面站需要返回确认包
	}

	if(ptr_gcs2ap_all_udp->auto_manu == 1)
	{
		//自动生效
	}
	else
	{
		//遥控生效
	}

	ptr_gcs2ap_all_udp->rc_thruster = ptr_gcs2ap_all_udp->throttle;
	ptr_gcs2ap_all_udp->rc_rudder = ptr_gcs2ap_all_udp->rudder;
	ptr_gcs2ap_all_udp->thruster_backward = ptr_gcs2ap_all_udp->fwdbwd;

	switch(ptr_gcs2ap_all_udp->controller_type)
	{
	case CONTROLLER_TYPE_PID:
		//ptr_gcs2ap_all_udp->ctrl_para_1;//throttle_p
		//ptr_gcs2ap_all_udp->ctrl_para_2;//throttle_i
		//ptr_gcs2ap_all_udp->ctrl_para_3;//throttle_d
		ptr_gcs2ap_all_udp->rud_p= ptr_gcs2ap_all_udp->ctrl_para_4;
		ptr_gcs2ap_all_udp->rud_i= ptr_gcs2ap_all_udp->ctrl_para_5;
		ptr_gcs2ap_all_udp->rud_d= ptr_gcs2ap_all_udp->ctrl_para_6;
		ptr_gcs2ap_all_udp->cte_p= ptr_gcs2ap_all_udp->ctrl_para_7;
		ptr_gcs2ap_all_udp->cte_i= ptr_gcs2ap_all_udp->ctrl_para_8;
		ptr_gcs2ap_all_udp->cte_d= ptr_gcs2ap_all_udp->ctrl_para_9;
		//ptr_gcs2ap_all_udp->ctrl_para_10;//yaw_rate_p
		//ptr_gcs2ap_all_udp->ctrl_para_11;//yaw_rate_i
		//ptr_gcs2ap_all_udp->ctrl_para_12;//yaw_rate_d

		break;
	case CONTROLLER_TYPE_ADRC:
		break;
	case CONTROLLER_TYPE_SMC:
		break;
	default:
		break;
	}

	temp = ptr_gcs2ap_all_udp->sail_mode & 0x0f;
	switch(temp)
	{
	case SAIL_MODE_0:
		ptr_gcs2ap_all_udp->workmode = RC_MODE;
		break;
	case SAIL_MODE_1:
		ptr_gcs2ap_all_udp->workmode = AUTO_MODE;
		ptr_gcs2ap_all_udp->auto_work_mode = AUTO_MISSION_MODE;
		break;
	case SAIL_MODE_2:
		ptr_gcs2ap_all_udp->workmode = STOP_MODE;
		break;
	case SAIL_MODE_3:
		ptr_gcs2ap_all_udp->workmode = RTL_MODE;
		break;
	case SAIL_MODE_4:
		ptr_gcs2ap_all_udp->workmode = AUTO_MODE;
		ptr_gcs2ap_all_udp->auto_work_mode = AUTO_GUIDE_MODE;
		break;
	case SAIL_MODE_5:
		ptr_gcs2ap_all_udp->workmode = AUTO_MODE;
		ptr_gcs2ap_all_udp->auto_work_mode = AUTO_LOITER_MODE;
		break;
	default:
		ptr_gcs2ap_all_udp->workmode = RC_MODE;
		break;
	}

	temp = ptr_gcs2ap_all_udp->sail_mode & 0x70;
	switch(temp)
	{
	case FORMATION_SOLO:
		ptr_gcs2ap_all_udp->formation_type = FORMATION_SOLO;
		//单独航行
		break;
	case FORMATION_LEADER_FOLLOWER:
		//领导跟随编队
		ptr_gcs2ap_all_udp->formation_type = FORMATION_LEADER_FOLLOWER;
		break;
	case FORMATION_DISTRIBUTED:
		//分布式编队
		ptr_gcs2ap_all_udp->formation_type = FORMATION_DISTRIBUTED;
		break;
	default:
		break;
	}

	temp = ptr_gcs2ap_all_udp->sail_mode & 0x80;
	if(temp == 1)
	{
		  ptr_gcs2ap_all_udp->wp_guide_no=ptr_gcs2ap_all_udp->wp_next;
	}

	switch(ptr_gcs2ap_all_udp->sensor_correct)
	{
	case SENSRO_CHECK_ACC:
		//校准加速度计
		break;
	case SENSRO_CHECK_GYRO:
		//校准陀螺仪
		break;
	case SENSRO_CHECK_MAG:
		//校准磁力计
		break;
	case SENSRO_CHECK_BARO:
		//校准气压计
		break;
	case SENSRO_CHECK_RESET:
		//重启主控
		break;
	default:
		break;
	}


	return 0;
}



