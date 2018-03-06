/*
 * boatlink_udp.h
 *
 *  Created on: 2018-3-6
 *      Author: wangbo
 */

#ifndef BOATLINK_UDP_H_
#define BOATLINK_UDP_H_



#define COMMAND_AP2GCS_REAL_UDP 0x01




struct AP2GCS_REAL_UDP
{
	unsigned char cnt;
    unsigned char pack_func_flag;//包功能标志，暂时固定为0
    unsigned char pack_func_info1;//接收到的命令包计数
    unsigned char pack_func_info2;//接收到的航点包计数

    unsigned int lng;//[度*0.00001]，GPS经度坐标，整型，精确到米
    unsigned int lat;//[度*0.00001]，GPS纬度坐标，整型，精确到米
    unsigned short spd;//[Knot*0.01]，实时航速
    short dir_gps;//16个字节//[度*0.01]，地速航向，GPS航向
    short dir_heading;//[度*0.01]，机头朝向
    short dir_target;//[度*0.01]，目标点朝向
    short dir_nav;//[度*0.01]，导航航向
    short roll;//24个字节//[度*0.01]，滚转
    short pitch;//[度*0.01]，俯仰
    short yaw;//28个字节//[度*0.01]，偏航

    unsigned char wp_next;
    unsigned char sail_mode;//遥控 自驾 驻航 返航
    unsigned char form_type;
    unsigned char pilot_vessel;//32个字节

    unsigned int spare1;
    unsigned int spare2;
    unsigned int spare3;
    unsigned int spare4;
    unsigned int spare5;
    unsigned int spare6;
    unsigned int spare7;
    unsigned int spare8;//64个字节
};























/*
 * Function:       send_ap2gcs_real_udp
 * Description:  驾驶仪向地面站发送ap2gcs_real_udp实时数据包
 */
int send_ap2gcs_real_udp();

int generate_packet_udp(unsigned char*dst_buf,unsigned char *src_buf,unsigned char len,\
                    unsigned int packet_cnt,unsigned char message_type,\
                    unsigned char vessel_id,unsigned char master_ap_link_ack, unsigned char plan_id);





extern struct AP2GCS_REAL_UDP ap2gcs_real_udp;




#endif /* BOATLINK_UDP_H_ */
