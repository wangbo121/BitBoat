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

int send_ap2gcs_real_udp()
{
    unsigned char real[256];
    unsigned char buf_packet[256];
    int ret;
    static unsigned int real_udp_cnt;

    real_udp_cnt++;
    ap2gcs_real_udp.cnt = real_udp_cnt;
    ap2gcs_real_udp.pack_func_flag=0;
    ap2gcs_real_udp.pack_func_info1=global_bool_boatpilot.gcs2ap_cmd_cnt;
    ap2gcs_real_udp.pack_func_info2=global_bool_boatpilot.gcs2ap_wp_cnt;;
    ap2gcs_real_udp.lng = gps_data.longitude;
    ap2gcs_real_udp.lat = gps_data.latitude;
    ap2gcs_real_udp.spd = gps_data.speed;
    ap2gcs_real_udp.dir_gps=(short)(convert_radian_to_degree(gps_data.course))*100;
    ap2gcs_real_udp.dir_heading=(short)gps_data.yaw;
    ap2gcs_real_udp.dir_target=global_bool_boatpilot.dir_target_degree;
    ap2gcs_real_udp.dir_nav=global_bool_boatpilot.dir_nav_degree;

    ap2gcs_real_udp.roll=(short)gps_data.roll;
    ap2gcs_real_udp.pitch=(short)gps_data.pitch;
    ap2gcs_real_udp.yaw=(short)gps_data.yaw;


    //ap2gcs_real_udp.wp_next=global_bool_boatpilot.wp_next;
    ap2gcs_real_udp.wp_next = 8;
    ap2gcs_real_udp.sail_mode = 1;
    ap2gcs_real_udp.form_type = 0;
    ap2gcs_real_udp.pilot_vessel = 0;

    printf("sizeof (struct AP2GCS_REAL_UDP) = %d \n",sizeof (struct AP2GCS_REAL_UDP));

    unsigned char vessel_id = 6;
    unsigned char master_ap_link_ack = 0xa0;
    unsigned char plan_id = 3;

    //printf("ap2gcs_real.toggle_state=%x\n",ap2gcs_real.toggle_state);//已测试20170413
    memcpy(real, &ap2gcs_real_udp, sizeof (struct AP2GCS_REAL_UDP));
    ret=generate_packet_udp(buf_packet, real, sizeof (struct AP2GCS_REAL_UDP),\
    		real_udp_cnt, COMMAND_AP2GCS_REAL_UDP,\
    		vessel_id,  master_ap_link_ack, plan_id);

    printf("send_ap2gcs_real_udp    :    ret = %d   fd_socket_generic = %d\n",ret,fd_socket_generic);
    send_socket_udp_data(fd_socket_generic, buf_packet, ret,"10.108.16.151",4000 );
    //send_socket_udp_data(fd_socket_generic, buf_packet, ret,"127.0.0.1",4006 );

    return 0;
}

int generate_packet_udp(unsigned char*dst_buf,unsigned char *src_buf,unsigned char len,\
                    unsigned int packet_cnt,unsigned char message_type,\
                    unsigned char vessel_id,unsigned char master_ap_link_ack, unsigned char plan_id)
{

	static unsigned char frame_head_len=8;
	static unsigned char frame_end_len=4;
	unsigned char packet[128];
	unsigned char checksum = 0;

	int i, j;
	int packet_data_len;

	packet[0] = 0xaa;
	packet[1] = 0x55;
	//packet[2] = len;//按道理应该是根据包所包含的字节数设置这个len
	packet[2] = 76;//统一定义为76字节，包含帧头帧尾
	packet_data_len = len;
	//packet_data_len = 76;

	packet[3] = 0;//数据包长度高8位

	packet[4] = message_type;
	packet[5] = vessel_id;

	packet[6]= master_ap_link_ack;
	packet[7]= plan_id;

	for (i = frame_head_len, j = 0; i < packet_data_len + frame_head_len; i++, j++)
	{
		packet[i] = src_buf[j];
	}

	for (i = 0; i < len + frame_head_len; i++)
	{
		checksum += packet[i];
	}

	i = len + frame_head_len;

	//20170503根据实时数据包的需求，利用了本来是帧尾的2个字节
	packet[i] = 0 ;
	packet[i+1] = 0 ;
	packet[i+2] = 0;
	checksum=checksum+packet[i]+packet[i+1]+packet[i+2];
	packet[i+3] = (checksum & 0xFF);

	i += 1;

	memcpy(dst_buf, packet, packet_data_len + frame_head_len + frame_end_len);

	/*返回总的发送字节数*/
	return packet_data_len + frame_head_len + frame_end_len;
}


