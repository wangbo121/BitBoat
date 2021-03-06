/*
 *@File     : boatlink_udp.cpp
 *@Author   : wangbo
 *@Date     : Mar 6, 2017
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
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
struct T_CONFIG_UDP boatpilot_config_udp_previous;
struct T_CONFIG_UDP boatpilot_config_udp;


#define REAL_DATA_BUF_SIZE 256
int send_ap2gcs_real_udp()
{
    unsigned char real[REAL_DATA_BUF_SIZE];
    int ret;
    static unsigned int real_udp_cnt;

    real_udp_cnt++;
    ap2gcs_real_udp.head1                 = 0xaa;
    ap2gcs_real_udp.head2                 = 0x55;
    ap2gcs_real_udp.len                   = 76;
    ap2gcs_real_udp.type                  = COMMAND_AP2GCS_REAL_UDP;
    ap2gcs_real_udp.vessel                = 1;  //发送数据的船的ID编号，从1开始，最多到100,0为无效船
    ap2gcs_real_udp.master_ap_link_ack    = 0x90;//D7:主/备自驾仪(1主,0备);D6:主/备自驾仪(1主,0备);D5~4:自驾仪编号(0..3);D3~1:链路编号(0:局域网;1:北斗;2:串口数传电台);D0:是否需要返回确认包(1:需要;0:不需)
    ap2gcs_real_udp.plan_id               = 0;//采用的航线编号，从1开始，最大为10，0表示没有航线

    ap2gcs_real_udp.cnt                   = real_udp_cnt;
    ap2gcs_real_udp.pack_func_flag        = 0;
    ap2gcs_real_udp.pack_func_info1       = 0;
    ap2gcs_real_udp.pack_func_info2       = 0;
    ap2gcs_real_udp.lng                   = all_external_device_input.longitude * 1e7;
    ap2gcs_real_udp.lat                   = all_external_device_input.latitude  * 1e7;
    ap2gcs_real_udp.spd                   = all_external_device_input.speed;
    ap2gcs_real_udp.dir_gps               = (short)(convert_radian_to_degree(all_external_device_input.course)) * 1e2;
    ap2gcs_real_udp.dir_heading           = (short)(all_external_device_input.psi * 1e2);
    ap2gcs_real_udp.dir_target            = global_bool_boatpilot.current_to_target_degree;
    ap2gcs_real_udp.dir_nav               = global_bool_boatpilot.command_course_degree;

    ap2gcs_real_udp.roll                  = (short)(all_external_device_input.phi * 1e2);
    ap2gcs_real_udp.pitch                 = (short)(all_external_device_input.theta * 1e2);
    ap2gcs_real_udp.yaw                   = (short)(all_external_device_input.psi * 1e2);

    ap2gcs_real_udp.wp_next               = global_bool_boatpilot.wp_next;
    ap2gcs_real_udp.sail_mode             = gcs2ap_all_udp.workmode;
    ap2gcs_real_udp.form_type             = gcs2ap_all_udp.formation_type;
    ap2gcs_real_udp.pilot_vessel          = 0;

    ap2gcs_real_udp.wp_success_cnt        = global_bool_boatpilot.wp_success_cnt;

    memcpy(real, &ap2gcs_real_udp, sizeof (struct AP2GCS_REAL_UDP));
    ret = sizeof (struct AP2GCS_REAL_UDP);
    send_socket_udp_data(fd_socket_generic, real, ret, (char *)AP_SENDTO_UDP_IP, AP_SENDTO_UDP_PORT);

    return 0;
}

int send_ap2gcs_real_udp_old_old()
{
    unsigned char real[REAL_DATA_BUF_SIZE];
    int ret;
    static unsigned int real_udp_cnt;

    real_udp_cnt++;
    ap2gcs_real_udp.head1                 = 0xaa;
    ap2gcs_real_udp.head2                 = 0x55;
    ap2gcs_real_udp.len                   = 76;
    ap2gcs_real_udp.type                  = COMMAND_AP2GCS_REAL_UDP;
    ap2gcs_real_udp.vessel                = 1;  //发送数据的船的ID编号，从1开始，最多到100,0为无效船
    ap2gcs_real_udp.master_ap_link_ack    = 0x90;//D7:主/备自驾仪(1主,0备);D6:主/备自驾仪(1主,0备);D5~4:自驾仪编号(0..3);D3~1:链路编号(0:局域网;1:北斗;2:串口数传电台);D0:是否需要返回确认包(1:需要;0:不需)
    ap2gcs_real_udp.plan_id               = 0;//采用的航线编号，从1开始，最大为10，0表示没有航线

    ap2gcs_real_udp.cnt                   = real_udp_cnt;
    ap2gcs_real_udp.pack_func_flag        = 0;
    ap2gcs_real_udp.pack_func_info1       = 0;
    ap2gcs_real_udp.pack_func_info2       = 0;
    ap2gcs_real_udp.lng                   = gps_data.longitude;
    ap2gcs_real_udp.lat                   = gps_data.latitude;
    ap2gcs_real_udp.spd                   = gps_data.velocity;
    ap2gcs_real_udp.dir_gps               = (short)(convert_radian_to_degree(gps_data.course_radian))*100;
    ap2gcs_real_udp.dir_heading           = (short)IMU_data.yaw;
    ap2gcs_real_udp.dir_heading           = (short)(convert_radian_to_degree(gps_data.course_radian))*100;
    ap2gcs_real_udp.dir_target            = global_bool_boatpilot.current_to_target_degree;
    ap2gcs_real_udp.dir_nav               = global_bool_boatpilot.command_course_degree;

    ap2gcs_real_udp.roll                  = (short)IMU_data.roll;
    ap2gcs_real_udp.pitch                 = (short)IMU_data.pitch;
    ap2gcs_real_udp.yaw                   = (short)IMU_data.yaw;

    ap2gcs_real_udp.wp_next               = global_bool_boatpilot.wp_next;
    ap2gcs_real_udp.sail_mode             = gcs2ap_all_udp.workmode;
    ap2gcs_real_udp.form_type             = gcs2ap_all_udp.formation_type;
    ap2gcs_real_udp.pilot_vessel          = 0;

    ap2gcs_real_udp.wp_success_cnt        = global_bool_boatpilot.wp_success_cnt;

    memcpy(real, &ap2gcs_real_udp, sizeof (struct AP2GCS_REAL_UDP));
    ret = sizeof (struct AP2GCS_REAL_UDP);
    send_socket_udp_data(fd_socket_generic, real, ret, (char *)AP_SENDTO_UDP_IP, AP_SENDTO_UDP_PORT);

    return 0;
}

static int decode_gcs2ap_cmd_udp(struct GCS2AP_ALL_UDP *ptr_gcs2ap_all_udp, struct GCS2AP_CMD_UDP *ptr_gcs2ap_cmd_udp);

int decode_gcs2ap_udp()
{
	/*1 decode gcs2ap_cmd*/
	if (global_bool_boatpilot.bool_get_gcs2ap_cmd)
	{
	    global_bool_boatpilot.bool_get_gcs2ap_cmd = FALSE;
		decode_gcs2ap_cmd_udp(&gcs2ap_all_udp, &gcs2ap_cmd_udp);
	}

	if(global_bool_boatpilot.bool_get_gcs2ap_waypoint)
	{
	    global_bool_boatpilot.bool_get_gcs2ap_waypoint = FALSE;
	    global_bool_boatpilot.wp_success_cnt ++;
	}

	return 0;
}

static int decode_gcs2ap_cmd_udp(struct GCS2AP_ALL_UDP *ptr_gcs2ap_all_udp, struct GCS2AP_CMD_UDP *ptr_gcs2ap_cmd_udp)
{
    unsigned char temp;

    /*
     * 把地面站传过来的命令包数据全盘接收
     * 有一些数据不需要进行判断的，就从gcs2ap_all_udp->cmd中获取，比如throttle rudder等
     * 有一些数据需要进行判断后的，比如workmode就是判断 位 的值然后决定workmode的值
     */
    memcpy(&ptr_gcs2ap_all_udp->cmd, ptr_gcs2ap_cmd_udp, sizeof(struct GCS2AP_CMD_UDP));

    if(ptr_gcs2ap_all_udp->cmd.master_ap_link_ack & 0xc0)
    {
        // master_ap_link_ack的D7和D6位是驾驶仪类型，船上有2套驾驶仪，一套是备份，当第1套驾驶仪宕机时，切换到第2套
        // 而每套驾驶仪都有2个AM3358的处理器，1个管理自动驾驶，第2个AM3358管理其他数据传感器等比如气象站等
        ptr_gcs2ap_all_udp->pilot_type = (ptr_gcs2ap_all_udp->cmd.master_ap_link_ack & 0xc0) >> 6;
    }
    ptr_gcs2ap_all_udp->pilot_cnt = ptr_gcs2ap_all_udp->cmd.master_ap_link_ack & 0x30;
    ptr_gcs2ap_all_udp->link_ID = ptr_gcs2ap_all_udp->cmd.master_ap_link_ack & 0x0e; // 然后判断通信链路
    if(ptr_gcs2ap_all_udp->cmd.master_ap_link_ack & 0x01)
    {
        //地面站需要返回确认包
    }

    switch(ptr_gcs2ap_all_udp->cmd.controller_type)
    {
    case CONTROLLER_TYPE_PID:
        ptr_gcs2ap_all_udp->rud_p= ptr_gcs2ap_all_udp->cmd.ctrl_para_1;
        ptr_gcs2ap_all_udp->rud_i= ptr_gcs2ap_all_udp->cmd.ctrl_para_2;
        ptr_gcs2ap_all_udp->rud_d= ptr_gcs2ap_all_udp->cmd.ctrl_para_3;
        ptr_gcs2ap_all_udp->cte_p= ptr_gcs2ap_all_udp->cmd.ctrl_para_4;
        ptr_gcs2ap_all_udp->cte_i= ptr_gcs2ap_all_udp->cmd.ctrl_para_5;
        ptr_gcs2ap_all_udp->cte_d= ptr_gcs2ap_all_udp->cmd.ctrl_para_6;
        break;
    case CONTROLLER_TYPE_ADRC:
        break;
    case CONTROLLER_TYPE_SMC:
        break;
    default:
        break;
    }

    temp = ptr_gcs2ap_all_udp->cmd.sail_mode & 0x0f;
    switch(temp)
    {
    case SAIL_MODE_0:
        ptr_gcs2ap_all_udp->workmode = RC_MODE;
        break;
    case SAIL_MODE_1:
        ptr_gcs2ap_all_udp->workmode = AUTO_MODE;
        ptr_gcs2ap_all_udp->auto_workmode = AUTO_MISSION_MODE;
        break;
    case SAIL_MODE_2:
        ptr_gcs2ap_all_udp->workmode = AUTO_MODE;
        ptr_gcs2ap_all_udp->auto_workmode = AUTO_STOP_MODE;
        break;
    case SAIL_MODE_3:
        ptr_gcs2ap_all_udp->workmode = AUTO_MODE;
        ptr_gcs2ap_all_udp->auto_workmode = AUTO_RTL_MODE;
        break;
    case SAIL_MODE_4:
        ptr_gcs2ap_all_udp->workmode = AUTO_MODE;
        ptr_gcs2ap_all_udp->auto_workmode = AUTO_GUIDE_MODE;
        break;
    case SAIL_MODE_5:
        ptr_gcs2ap_all_udp->workmode = AUTO_MODE;
        ptr_gcs2ap_all_udp->auto_workmode = AUTO_LOITER_MODE;
        break;
    default:
        ptr_gcs2ap_all_udp->workmode = RC_MODE;
        break;
    }

    temp = ptr_gcs2ap_all_udp->cmd.multi_mode & 0x70;
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

    temp = ptr_gcs2ap_all_udp->cmd.sail_mode & 0x80;
    if(temp == 1)
    {
          ptr_gcs2ap_all_udp->wp_guide_no=ptr_gcs2ap_all_udp->cmd.wp_next;
    }

    switch(ptr_gcs2ap_all_udp->cmd.system_calib)
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



