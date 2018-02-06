/*
 * boatlink.c
 *
 *  Created on: 2016年5月16日
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

#include "boatlink.h"

#define MOTOR_FORWARD 0

#define GENERATOR_STOP 0
#define GENERATOR_START 1
#define GENERATOR_AUTO 2

#define TURN_MODE_RUDDER 0
#define TURN_MODE_DIFFSPD 1
#define TURN_MODE_MIX 2

#define SWITCH_AUTO      0
#define SWITCH_CHANNEL_0 1
#define SWITCH_CHANNEL_1 2
#define SWITCH_STOP      3

struct GCS_AP_WP gcs_ap_wp;
struct GCS_AP_WP ap2gcs_wp;
struct GCS2AP_CMD gcs2ap_cmd;
struct GCS2AP_CMD gcs2ap_cmd_return;
struct AP2GCS_REAL ap2gcs_real;

struct WAY_POINT wp_data[MAX_WAYPOINT_NUM];

struct GCS2AP_RADIO gcs2ap_radio_all;

struct T_CONFIG boatpilot_config_previous;
struct T_CONFIG boatpilot_config;

struct T_BOATPILOT_LOG boatpilot_log;

static int decode_gcs2ap_cmd(struct GCS2AP_RADIO *ptr_gcs2ap_radio, struct GCS2AP_CMD *ptr_gcs2ap_cmd);
static int decode_gcs2ap_waypoint(struct WAY_POINT *ptr_wp_data, struct GCS_AP_WP *ptr_gcs2ap_wp);

int decode_gcs2ap_radio()
{
    /*1 decode gcs2ap_cmd*/
    if (global_bool_boatpilot.bool_get_gcs2ap_cmd)
    {
        decode_gcs2ap_cmd(&gcs2ap_radio_all, &gcs2ap_cmd);
        global_bool_boatpilot.assign_config_req=TRUE;

        global_bool_boatpilot.bool_get_gcs2ap_cmd = FALSE;
    }
    /*2 decode gcs2ap_waypoint*/
    if (global_bool_boatpilot.bool_get_gcs2ap_waypoint)
    {
        decode_gcs2ap_waypoint(wp_data,&gcs_ap_wp);
        global_bool_boatpilot.bool_get_gcs2ap_waypoint = FALSE;
    }

    return 0;
}

/*
 * 发送指定从wp_start起始的wp_num个航点
 */
int send_ap2gcs_waypoint_num(unsigned char wp_start,unsigned char wp_num)
{
    unsigned char buf_temp[256];
    unsigned char buf_packet[256];

    unsigned int ret;

    printf("最终发送给地面站的航点个数=%d\n",wp_num);
    memset(&ap2gcs_wp.way_point0,0,5*sizeof(struct WAY_POINT));
    memcpy(&ap2gcs_wp.way_point0,&wp_data[wp_start],wp_num*sizeof(struct WAY_POINT));
    memcpy(buf_temp,&ap2gcs_wp,sizeof(struct GCS_AP_WP));

    ret=generate_packet(buf_packet, buf_temp, sizeof(struct GCS_AP_WP),\
                        global_bool_boatpilot.ap2gcs_wp_cnt, COMMAND_AP2GCS_WP,\
                        0,1);
    send_radio_data(buf_packet, ret);

    int i=0;
    for(i=0;i<wp_num;i++)
    {
        printf("驾驶仪-->地面站航点包的第%d个航点的编号=%d\n",wp_start+i,wp_data[wp_start+i].no);
        printf("驾驶仪-->地面站航点包的第%d个航点的经度=%d\n",wp_start+i,wp_data[wp_start+i].lng);
        printf("驾驶仪-->地面站航点包的第%d个航点的纬度=%d\n",wp_start+i,wp_data[wp_start+i].lat);
        printf("驾驶仪-->地面站航点包的第%d个航点的高度=%d\n",wp_start+i,wp_data[wp_start+i].alt);
        printf("驾驶仪-->地面站航点包的第%d个航点的速度=%d\n",wp_start+i,wp_data[wp_start+i].spd);
    }

    return 0;
}

int send_ap2gcs_waypoint()
{
    unsigned char wp[256];
    unsigned char buf_packet[256];

    /*
     * 这个ret 之前粗心写成了unsigned char ret 导致后面发送数据出现了严重错误，
     * 所以，切记函数的类型一定要一致
     */
    unsigned int ret;

    memcpy(wp,&gcs_ap_wp,sizeof(struct GCS_AP_WP));
    printf("gcs_ap_wp=%d\n",gcs_ap_wp.pack_func_info3);

    ret=generate_packet(buf_packet, wp, sizeof(struct GCS_AP_WP),\
                        global_bool_boatpilot.ap2gcs_wp_cnt, COMMAND_AP2GCS_WP,\
                        0,1);
    send_radio_data(buf_packet, ret);

    return 0;
}

int send_ap2gcs_cmd()
{
    unsigned char cmd[256];
    unsigned char buf_packet[256];
    int ret;

    memcpy(&gcs2ap_cmd_return,&gcs2ap_radio_all,sizeof(struct GCS2AP_CMD));
    memcpy(cmd,&gcs2ap_cmd_return,sizeof(struct GCS2AP_CMD));
    //printf("sizeof(struct GCS2AP_CMD)=%d\n",sizeof(struct GCS2AP_CMD));//已测试20170413

    ret=generate_packet(buf_packet, cmd, sizeof(struct GCS2AP_CMD),\
                        global_bool_boatpilot.ap2gcs_cmd_cnt, COMMAND_AP2GCS_CMD,\
                        0,1);
    send_radio_data(buf_packet, ret);

    return 0;
}

int send_ap2gcs_real()
{
    unsigned char real[256];
    unsigned char buf_packet[256];
    int ret;

    ap2gcs_real.pack_func_flag=0;
    ap2gcs_real.pack_func_info1=global_bool_boatpilot.gcs2ap_cmd_cnt;
    ap2gcs_real.pack_func_info2=global_bool_boatpilot.gcs2ap_wp_cnt;;
    ap2gcs_real.pack_func_info3=global_bool_boatpilot.wp_packet_cnt;
    ap2gcs_real.lng = gps_data.longitude;
    ap2gcs_real.lat = gps_data.latitude;
    ap2gcs_real.spd = gps_data.speed;
    ap2gcs_real.dir_gps=(short)(convert_radian_to_degree(gps_data.course))*100;
    ap2gcs_real.dir_heading=(short)gps_data.yaw;
    ap2gcs_real.dir_target=global_bool_boatpilot.dir_target_degree;
    ap2gcs_real.dir_nav=global_bool_boatpilot.dir_nav_degree;

    ap2gcs_real.roll=(short)gps_data.roll;
    ap2gcs_real.pitch=(short)gps_data.pitch;
    ap2gcs_real.yaw=(short)gps_data.yaw;

    ap2gcs_real.codedisc=0;//0-360
    ap2gcs_real.da_out1=global_bool_boatpilot.left_motor_voltage;
    ap2gcs_real.da_out2=global_bool_boatpilot.right_motor_voltage;
    ap2gcs_real.rudder_pos=(unsigned char)(global_bool_boatpilot.rudder_angle_degree+45);

    ap2gcs_real.rc_thruster=gcs2ap_radio_all.rc_thruster;
    ap2gcs_real.rc_rudder=gcs2ap_radio_all.rc_rudder;
    ap2gcs_real.rud_p=gcs2ap_radio_all.rud_p;
    ap2gcs_real.cte_p=gcs2ap_radio_all.cte_p;
    ap2gcs_real.boat_temp1=0;//待修改
    ap2gcs_real.boat_temp2=0;//待修改
    ap2gcs_real.boat_humi=0;//待修改
//    ap2gcs_real.voltage_bat1=data_s2m.switcher.voltage[0];
//    ap2gcs_real.voltage_bat2=data_s2m.switcher.voltage[1];
//    ap2gcs_real.current_bat1=data_s2m.current[0]/50;
//    ap2gcs_real.current_bat2=data_s2m.current[1]/50;
//
//    if(data_s2m.switcher.workstate[0]==0x55)
//    {
//        //切换器通道是1
//        ap2gcs_real.toggle_state=(ap2gcs_real.toggle_state & 0xfc) | 0x01;
//
//        if(data_s2m.switcher.workstate[0]==0x55)
//        {
//            //正在放电
//            ap2gcs_real.toggle_state=(ap2gcs_real.toggle_state & 0xf3) | 0x04;
//        }
//        if(data_s2m.switcher.workstate[0]==0x5a)
//        {
//            //请求放电
//            ap2gcs_real.toggle_state=(ap2gcs_real.toggle_state & 0xf3) | 0x08;
//        }
//        if(data_s2m.switcher.workstate[0]==0xaa)
//        {
//            //停止放电
//            ap2gcs_real.toggle_state=(ap2gcs_real.toggle_state & 0xf3) | 0x0c;
//        }
//    }
//    else if(data_s2m.switcher.workstate[1]==0x55)
//    {
//        //切换器通道是2
//        ap2gcs_real.toggle_state=(ap2gcs_real.toggle_state & 0xfc) | 0x02;
//
//        if(data_s2m.switcher.workstate[1]==0x55)
//        {
//            //正在放电
//            ap2gcs_real.toggle_state=(ap2gcs_real.toggle_state & 0xcf) | 0x10;
//        }
//        if(data_s2m.switcher.workstate[1]==0x5a)
//        {
//            //请求放电
//            ap2gcs_real.toggle_state=(ap2gcs_real.toggle_state & 0xcf) | 0x20;
//        }
//        if(data_s2m.switcher.workstate[1]==0xaa)
//        {
//            //停止放电
//            ap2gcs_real.toggle_state=(ap2gcs_real.toggle_state & 0xcf) | 0x30;
//        }
//    }
//    else
//    {
//        ap2gcs_real.toggle_state=(ap2gcs_real.toggle_state & 0xfc);
//    }
//
//
//    //充电机通道
//    if(data_s2m.charger.channel==1)
//    {
//        ap2gcs_real.charge_state=(ap2gcs_real.charge_state & 0xfc) | 0x01;
//
//        ap2gcs_real.charger_voltage=data_s2m.charger.voltage[0];
//        ap2gcs_real.charger_current=data_s2m.charger.current[0];
//    }
//    else if(data_s2m.charger.channel==2)
//    {
//        ap2gcs_real.charge_state=(ap2gcs_real.charge_state & 0xfc) | 0x02;
//
//        ap2gcs_real.charger_voltage=data_s2m.charger.voltage[1];
//        ap2gcs_real.charger_current=data_s2m.charger.current[1];
//    }
//    else if(data_s2m.charger.channel==3)
//    {
//        ap2gcs_real.charge_state=(ap2gcs_real.charge_state & 0xfc) | 0x03;
//    }
//    else if(data_s2m.charger.channel==0)
//    {
//
//    }
//
//    //充电机状态
//    if(data_s2m.charger.work_state & 0x01)
//    {
//        //关机状态
//        ap2gcs_real.charge_state=(ap2gcs_real.charge_state & 0xfb) | 0x04;
//    }
//    else
//    {
//        //开机状态
//        ap2gcs_real.charge_state=(ap2gcs_real.charge_state & 0xfb);
//    }

//    //发电机
//    if(global_bool_boatpilot.bool_generator_on)
//    {
//        ap2gcs_real.charge_state=(ap2gcs_real.charge_state & 0x7f) | 0x80;
//    }
//    else
//    {
//        ap2gcs_real.charge_state=(ap2gcs_real.charge_state & 0x7f);
//    }
//
//    //气象站数据
//    ap2gcs_real.temp=data_s2m.aws.temp;
//    ap2gcs_real.humi=data_s2m.aws.humi;
//    ap2gcs_real.windspeed=data_s2m.aws.windspeed;
//    ap2gcs_real.winddir=data_s2m.aws.winddir;
//    ap2gcs_real.airpress=data_s2m.aws.airpress;
//    ap2gcs_real.seasault=data_s2m.aws.seasault;
//    ap2gcs_real.elec_cond=data_s2m.aws.elec_cond;
//    ap2gcs_real.seatemp1=data_s2m.aws.seatemp1;
//    //ap2gcs_real.seatemp2=data_s2m.aws.seatemp2;//待修改
//    //ap2gcs_real.seatemp3=data_s2m.aws.seatemp3;//待修改
//    //ap2gcs_real.alt=35;//待修改
//    //ap2gcs_real.radiation=36;//待修改
//
//    //火箭数据
//    ap2gcs_real.launch_req_ack=data_s2m.rkt.launch_req_ack;
//    ap2gcs_real.rocket_state=data_s2m.rkt.state;
//    ap2gcs_real.rktnumber=data_s2m.rkt.rktnumber;
//    ap2gcs_real.rkt_alt=data_s2m.rkt.alt;
    ap2gcs_real.work_mode=gcs2ap_radio_all.workmode;
    ap2gcs_real.wp_next=global_bool_boatpilot.wp_next;

    //printf("ap2gcs_real.toggle_state=%x\n",ap2gcs_real.toggle_state);//已测试20170413
    memcpy(real, &ap2gcs_real, sizeof (struct AP2GCS_REAL));
    ret=generate_packet(buf_packet, real, sizeof (struct AP2GCS_REAL),\
                        global_bool_boatpilot.ap2gcs_real_cnt, COMMAND_AP2GCS_REAL,\
                        0,1);
    send_radio_data(buf_packet, ret);

    return 0;
}


static int decode_gcs2ap_waypoint(struct WAY_POINT *ptr_wp_data, struct GCS_AP_WP *ptr_gcs2ap_wp)
{
    int write_len=0;

    unsigned char wp_total;//航点总数
    unsigned char wp_num_need_to_send;//要发送的航点数
    unsigned char wp_num_in_pack;//本航点包所含有的航点数
    unsigned char wp_packet_cnt;//本航点包的编号，记录已经保存了多少个航点,需要返回给实时数据用于确认航点
    static unsigned char wp_cnt;

    unsigned char wp_start_no;//本航点包的第一个航点的编号

    wp_total=ptr_gcs2ap_wp->pack_func_flag;
    wp_num_need_to_send=ptr_gcs2ap_wp->pack_func_info1;
    wp_num_in_pack=ptr_gcs2ap_wp->pack_func_info2;
    wp_packet_cnt=ptr_gcs2ap_wp->pack_func_info3;

    wp_start_no=ptr_gcs2ap_wp->way_point0.no;

    printf("gcs2ap_radio_all.wp_flag=%d\n",gcs2ap_radio_all.wp_flag);

    if(wp_num_need_to_send>wp_total)
    {
        printf("要发送的航点数大于总航点数，请重新发送\n");
        return -1;
    }
    else
    {
        memcpy(&(ptr_wp_data[wp_start_no]),&(ptr_gcs2ap_wp->way_point0),sizeof(struct WAY_POINT)*wp_num_in_pack);

        wp_cnt+=wp_num_in_pack;

        if(wp_cnt>=wp_num_need_to_send)
        {
            printf("航点接收完全\n");

            wp_cnt=0;

            /*保存航点到航点文件*/
            //write_len=write(fd_waypoint,(char *)ptr_wp_data,sizeof(struct WAY_POINT)*MAX_WAYPOINT_NUM);
            printf("write_len 写入了%d个字节的航点\n",write_len);
        }

        global_bool_boatpilot.wp_total_num = wp_total;//这个global_bool_boatpilot.wp_total_num必须留着，因为gcs2ap_radio_all中的wp_total可能为0，来说明航点无效
        global_bool_boatpilot.wp_packet_cnt=wp_packet_cnt;//包的计数通过实时数据返回给地面站，地面站确认后，再继续发航点包

        printf("wp_num_in_pack=%d\n",wp_num_in_pack);
        int i=0;
        for(i=0;i<wp_num_in_pack;i++)
        {
            printf("GCS-->boatpilot传输航点包的第%d个航点的编号=%d\n",wp_start_no+i,ptr_wp_data[wp_start_no+i].no);
            printf("GCS-->boatpilot传输航点包的第%d个航点的经度=%d\n",wp_start_no+i,ptr_wp_data[wp_start_no+i].lng);
            printf("GCS-->boatpilot传输航点包的第%d个航点的纬度=%d\n",wp_start_no+i,ptr_wp_data[wp_start_no+i].lat);
            printf("GCS-->boatpilot传输航点包的第%d个航点的高度=%d\n",wp_start_no+i,ptr_wp_data[wp_start_no+i].alt);
            printf("GCS-->boatpilot传输航点包的第%d个航点的速度=%d\n",wp_start_no+i,ptr_wp_data[wp_start_no+i].spd);
        }
    }

    return 0;
}

static int decode_gcs2ap_cmd(struct GCS2AP_RADIO *ptr_gcs2ap_radio, struct GCS2AP_CMD *ptr_gcs2ap_cmd)
{
    ptr_gcs2ap_radio->pack_func_flag=ptr_gcs2ap_cmd->pack_func_flag;
    ptr_gcs2ap_radio->pack_func_info1=ptr_gcs2ap_cmd->pack_func_info1;
    ptr_gcs2ap_radio->pack_func_info2=ptr_gcs2ap_cmd->pack_func_info2;
    ptr_gcs2ap_radio->pack_func_info3=ptr_gcs2ap_cmd->pack_func_info3;

    if((ptr_gcs2ap_radio->pack_func_flag & 0x01)==0)
    {
        /*
         * 命令包优先，如果pack_func_flag的从右往左第二位也等于1--请求航点回传，
         * 则忽视，优先回传命令包
         * 不是回传命令包时，我们会判断是否回传航点包，并同时更新地面站传到驾驶仪的命令
         */
        global_bool_boatpilot.send_ap2gcs_cmd_req=TRUE;
        printf("地面站请求同步，回传gcs2ap_radio中保存的关于cmd命令包的数据\n");
    }
    else
    {
        /*
         * 不是回传命令时，则需要获取cmd的值到gcs2ap_radio中
         * 也就是说进入这个判断则意味着地面站在发送正常命令包
         * 需要把一些标志为如回传命令，发送特定航点标志清零
         */
        global_bool_boatpilot.send_ap2gcs_cmd_req=FALSE;
        global_bool_boatpilot.send_ap2gcs_specific_wp_req=FALSE;

        if(((ptr_gcs2ap_radio->pack_func_flag >> 1) & 0x01))
        {
            printf("地面站请求回传航点\n");

            if(global_bool_boatpilot.wp_total_num>=1)
            {
                if(ptr_gcs2ap_radio->pack_func_info1==255)
                {
                    printf("地面站要求回传全部航点\n");
                    global_bool_boatpilot.bool_is_sending_wp_ap2gcs=TRUE;//正在发送航点
                    global_bool_boatpilot.send_ap2gcs_specific_wp_req=FALSE;//不是发送指定的某些航点，把这个标志量归零

                    if(global_bool_boatpilot.wp_total_num/5)
                    {
                        //航点总数目大于5个了
                        global_bool_boatpilot.send_ap2gcs_wp_start_num=0;
                        global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+5-1;
                    }
                    else if(global_bool_boatpilot.wp_total_num%5)
                    {
                        global_bool_boatpilot.send_ap2gcs_wp_start_num=0;
                        global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+global_bool_boatpilot.wp_total_num-1;
                    }
                    ap2gcs_wp.pack_func_info1=global_bool_boatpilot.wp_total_num;//当地面站要求回传全部航点时，要发送的航点总数就是总的航点数目
                }
                else
                {
                    //地面站请求回传航点一次后，就会把ptr_gcs2ap_radio->pack_func_info1=255置零
                    printf("地面站请求回传特定航点，起始航点是%d，航点数是%d\n",ptr_gcs2ap_radio->pack_func_info1,ptr_gcs2ap_radio->pack_func_info2);
                    if(ptr_gcs2ap_radio->pack_func_info2>5)
                    {
                        global_bool_boatpilot.send_ap2gcs_specific_wp_req=TRUE;
                        global_bool_boatpilot.send_ap2gcs_wp_start_num=0;
                        global_bool_boatpilot.send_ap2gcs_wp_end_num=0;

                        /*
                         * 地面站请求回传特定航点时，所请求回传的航点数大于5了，
                         * 但是驾驶仪目前在回传特定航点时最多只能回传5个，拒绝回传
                         * 只回传0航点
                         */
                        printf("地面站请求回传特定航点时，所请求回传的航点数大于5了，只回传0航点\n");
                    }
                    else
                    {
                        global_bool_boatpilot.send_ap2gcs_specific_wp_req=TRUE;

                        if(global_bool_boatpilot.wp_total_num/(ptr_gcs2ap_radio->pack_func_info1+ptr_gcs2ap_radio->pack_func_info2))
                        {
                            //wp_data数组中有足够多的航点，能够回传所请求的航点数
                            global_bool_boatpilot.send_ap2gcs_wp_start_num=ptr_gcs2ap_radio->pack_func_info1;
                            global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+ptr_gcs2ap_radio->pack_func_info2-1;
                        }
                        else if(global_bool_boatpilot.wp_total_num%(ptr_gcs2ap_radio->pack_func_info1+ptr_gcs2ap_radio->pack_func_info2))
                        {
                            global_bool_boatpilot.send_ap2gcs_wp_start_num=ptr_gcs2ap_radio->pack_func_info1;
                            global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+((global_bool_boatpilot.wp_total_num-1)-global_bool_boatpilot.send_ap2gcs_wp_start_num);
                        }
                        ap2gcs_wp.pack_func_info1=global_bool_boatpilot.send_ap2gcs_wp_end_num-global_bool_boatpilot.send_ap2gcs_wp_start_num+1;
                        //printf("wp_total_num=%d,info1=%d,info2=%d\n",global_bool_boatpilot.wp_total_num,ap2gcs_wp.pack_func_info1,ptr_gcs2ap_radio->pack_func_info2);//20170508已测试
                    }
                }

                /*
                 * 到了这一级判断，则
                 * 1.地面站请求回传航点了
                 * 2.并且航点数>=1了
                 * 所以总是要回传一个航点包的
                 */
                ap2gcs_wp.pack_func_flag=global_bool_boatpilot.wp_total_num;
                ap2gcs_wp.pack_func_info2=global_bool_boatpilot.send_ap2gcs_wp_end_num-global_bool_boatpilot.send_ap2gcs_wp_start_num+1;
                global_bool_boatpilot.send_ap2gcs_wp_req=TRUE;
            }
            else
            {
                //总的航点数小于1，所以什么也不做
                //printf("虽然地面站请求回传航点，但是驾驶仪中保存的航点数小于1，无法回传\n");
            }
        }
        else if((global_bool_boatpilot.ap2gcs_wp_cnt==ptr_gcs2ap_radio->pack_func_info3))
        {
            if(global_bool_boatpilot.wp_total_num>=1)
            {
                if(global_bool_boatpilot.send_ap2gcs_specific_wp_req)
                {
                    /*
                     * 如果是请求发送指定航点，怎么判断结束呢
                     * 因为请求发送指定航点时，数目肯定是小于等于5个的，所以不需要判断是否回传结束
                     * 直接回传发送一次就好
                     */
                    global_bool_boatpilot.send_ap2gcs_specific_wp_req=FALSE;
                }
                else
                {
                    //在不是回传特点航点的情况下，判断回传全部航点是否结束，如果没有结束，继续回传
                    if(global_bool_boatpilot.send_ap2gcs_wp_end_num==(global_bool_boatpilot.wp_total_num-1))
                    {
                        //printf("已经回传结束全部航点\n");//20170426已测试
                        global_bool_boatpilot.bool_is_sending_wp_ap2gcs=FALSE;
                    }
                    else if(global_bool_boatpilot.bool_is_sending_wp_ap2gcs)
                    {
                        printf("已经开始回传航点，但是还没有回传结束\n");
                        int remain;
                        remain=global_bool_boatpilot.wp_total_num-(global_bool_boatpilot.send_ap2gcs_wp_end_num+1);

                        if(remain/5)
                        {
                            global_bool_boatpilot.send_ap2gcs_wp_start_num=global_bool_boatpilot.send_ap2gcs_wp_end_num+1;
                            global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+4;
                        }
                        else if(remain%5)
                        {
                            global_bool_boatpilot.send_ap2gcs_wp_start_num=global_bool_boatpilot.send_ap2gcs_wp_end_num+1;
                            global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+remain-1;
                        }

                        ap2gcs_wp.pack_func_info1=global_bool_boatpilot.wp_total_num;

                        ap2gcs_wp.pack_func_flag=global_bool_boatpilot.wp_total_num;
                        ap2gcs_wp.pack_func_info2=global_bool_boatpilot.send_ap2gcs_wp_end_num-global_bool_boatpilot.send_ap2gcs_wp_start_num+1;
                        global_bool_boatpilot.send_ap2gcs_wp_req=TRUE;
                    }
                }
            }
            else
            {
                //总的航点数小于1，所以什么也不做
                //printf("虽然地面站请求回传航点，但是驾驶仪中保存的航点数小于1，无法回传\n");
            }
        }

        ptr_gcs2ap_radio->workmode=ptr_gcs2ap_cmd->workmode;
        ptr_gcs2ap_radio->rc_thruster=ptr_gcs2ap_cmd->rc_thruster;
        ptr_gcs2ap_radio->rc_rudder=ptr_gcs2ap_cmd->rc_rudder;
        ptr_gcs2ap_radio->rud_p=ptr_gcs2ap_cmd->rud_p;
        ptr_gcs2ap_radio->rud_i=ptr_gcs2ap_cmd->rud_i;
        ptr_gcs2ap_radio->rud_d=ptr_gcs2ap_cmd->rud_d;
        ptr_gcs2ap_radio->cte_p=ptr_gcs2ap_cmd->cte_p;
        ptr_gcs2ap_radio->cte_i=ptr_gcs2ap_cmd->cte_i;
        ptr_gcs2ap_radio->cte_d=ptr_gcs2ap_cmd->cte_d;
        ptr_gcs2ap_radio->rudder_setup_reverse=ptr_gcs2ap_cmd->rudder_setup_reverse;
        ptr_gcs2ap_radio->thruster_setup_reverse=ptr_gcs2ap_cmd->thruster_setup_reverse;
        ptr_gcs2ap_radio->generator_on=ptr_gcs2ap_cmd->generator_on;
        ptr_gcs2ap_radio->thruster_backward=ptr_gcs2ap_cmd->thruster_backward;
        ptr_gcs2ap_radio->motor_lock=ptr_gcs2ap_cmd->motor_lock;
        ptr_gcs2ap_radio->middle_motor_on=ptr_gcs2ap_cmd->middle_motor_on;
        ptr_gcs2ap_radio->navigation_mode=ptr_gcs2ap_cmd->navigation_mode;
        ptr_gcs2ap_radio->charge_start=ptr_gcs2ap_cmd->charge_start;
        ptr_gcs2ap_radio->rocket_hat=ptr_gcs2ap_cmd->rocket_hat;
        ptr_gcs2ap_radio->rocket_launch=ptr_gcs2ap_cmd->rocket_launch;
        ptr_gcs2ap_radio->turn_mode=ptr_gcs2ap_cmd->turn_mode;
        ptr_gcs2ap_radio->diffspd_coef=ptr_gcs2ap_cmd->diffspd_coef;
        ptr_gcs2ap_radio->diffspd_lim=ptr_gcs2ap_cmd->diffspd_lim;
        ptr_gcs2ap_radio->cruise_throttle_percent=ptr_gcs2ap_cmd->cruise_throttle_percent;
        ptr_gcs2ap_radio->throttle_change_time=ptr_gcs2ap_cmd->throttle_change_time;
        ptr_gcs2ap_radio->arrive_radius=ptr_gcs2ap_cmd->arrive_radius;
        ptr_gcs2ap_radio->cte_max_degree=ptr_gcs2ap_cmd->cte_max_degree;
        ptr_gcs2ap_radio->rudder_calib=ptr_gcs2ap_cmd->rudder_calib;
        ptr_gcs2ap_radio->rudder_calib_cnt=ptr_gcs2ap_cmd->rudder_calib_cnt;
        ptr_gcs2ap_radio->set_switch_channel=ptr_gcs2ap_cmd->set_switch_channel;
        ptr_gcs2ap_radio->set_switch_low_limit=ptr_gcs2ap_cmd->set_switch_low_limit;
        ptr_gcs2ap_radio->set_switch_high_limit=ptr_gcs2ap_cmd->set_switch_high_limit;
        ptr_gcs2ap_radio->set_charge_channel=ptr_gcs2ap_cmd->set_charge_channel;
        ptr_gcs2ap_radio->set_charge_voltage=ptr_gcs2ap_cmd->set_charge_voltage;
        ptr_gcs2ap_radio->set_charge_current=ptr_gcs2ap_cmd->set_charge_current;
        ptr_gcs2ap_radio->rudder_dead_zone_angle_degree=ptr_gcs2ap_cmd->rudder_dead_zone_angle_degree;
        ptr_gcs2ap_radio->slave_config=ptr_gcs2ap_cmd->slave_config;

        ptr_gcs2ap_radio->wp_flag=ptr_gcs2ap_cmd->wp_flag;
        ptr_gcs2ap_radio->wp_next=ptr_gcs2ap_cmd->wp_next;
        ptr_gcs2ap_radio->spd=ptr_gcs2ap_cmd->spd;
        ptr_gcs2ap_radio->alt=ptr_gcs2ap_cmd->alt;
        ptr_gcs2ap_radio->lng=ptr_gcs2ap_cmd->lat;
        ptr_gcs2ap_radio->lat=ptr_gcs2ap_cmd->lat;
    }

#if 0
    printf("ptr_gcs2ap_radio->workmode=%d\n",ptr_gcs2ap_radio->workmode);
    printf("ptr_gcs2ap_radio->rc_thruster=%d\n",ptr_gcs2ap_radio->rc_thruster);
    printf("ptr_gcs2ap_radio->rc_rudder=%d\n",ptr_gcs2ap_radio->rc_rudder);
    printf("ptr_gcs2ap_radio->rud_p=%d\n",ptr_gcs2ap_radio->rud_p);
    printf("ptr_gcs2ap_radio->rud_i=%d\n",ptr_gcs2ap_radio->rud_i);
    printf("ptr_gcs2ap_radio->rud_d=%d\n",ptr_gcs2ap_radio->rud_d);
    printf("ptr_gcs2ap_radio->cte_p=%d\n",ptr_gcs2ap_radio->cte_p);
    printf("ptr_gcs2ap_radio->cte_i=%d\n",ptr_gcs2ap_radio->cte_i);
    printf("ptr_gcs2ap_radio->cte_d=%d\n",ptr_gcs2ap_radio->cte_d);
    printf("ptr_gcs2ap_radio->rudder_setup_reverse=%d\n",ptr_gcs2ap_radio->rudder_setup_reverse);
    printf("ptr_gcs2ap_radio->thruster_setup_reverse=%d\n",ptr_gcs2ap_radio->thruster_setup_reverse);
    printf("ptr_gcs2ap_radio->generator_on=%d\n",ptr_gcs2ap_radio->generator_on);
    printf("ptr_gcs2ap_radio->thruster_backward=%d\n",ptr_gcs2ap_radio->thruster_backward);
    printf("ptr_gcs2ap_radio->motor_lock=%d\n",ptr_gcs2ap_radio->motor_lock);
    printf("ptr_gcs2ap_radio->middle_motor_on=%d\n",ptr_gcs2ap_radio->middle_motor_on);
    printf("ptr_gcs2ap_radio->navigation_mode=%d\n",ptr_gcs2ap_radio->navigation_mode);
    printf("ptr_gcs2ap_radio->charge_start=%d\n",ptr_gcs2ap_radio->charge_start);
    printf("ptr_gcs2ap_radio->rocket_hat=%d\n",ptr_gcs2ap_radio->rocket_hat);
    printf("ptr_gcs2ap_radio->rocket_launch=%d\n",ptr_gcs2ap_radio->rocket_launch);
    printf("ptr_gcs2ap_radio->turn_mode=%d\n",ptr_gcs2ap_radio->turn_mode);
    printf("ptr_gcs2ap_radio->diffspd_coef=%d\n",ptr_gcs2ap_radio->diffspd_coef);
    printf("ptr_gcs2ap_radio->diffspd_lim=%d\n",ptr_gcs2ap_radio->diffspd_lim);
    printf("ptr_gcs2ap_radio->cruise_throttle_percent=%d\n",ptr_gcs2ap_radio->cruise_throttle_percent);
    printf("ptr_gcs2ap_radio->throttle_change_time=%d\n",ptr_gcs2ap_radio->throttle_change_time);
    printf("ptr_gcs2ap_radio->arrive_radius=%d\n",ptr_gcs2ap_radio->arrive_radius);
    printf("ptr_gcs2ap_radio->cte_max_degree=%d\n",ptr_gcs2ap_radio->cte_max_degree);
    printf("ptr_gcs2ap_radio->rudder_calib=%d\n",ptr_gcs2ap_radio->rudder_calib);
    printf("ptr_gcs2ap_radio->rudder_calib_cnt=%d\n",ptr_gcs2ap_radio->rudder_calib_cnt);
    printf("ptr_gcs2ap_radio->set_switch_channel=%d\n",ptr_gcs2ap_radio->set_switch_channel);
    printf("ptr_gcs2ap_radio->set_switch_low_limit=%d\n",ptr_gcs2ap_radio->set_switch_low_limit);
    printf("ptr_gcs2ap_radio->set_switch_high_limit=%d\n",ptr_gcs2ap_radio->set_switch_high_limit);
    printf("ptr_gcs2ap_radio->set_charge_channel=%d\n",ptr_gcs2ap_radio->set_charge_channel);
    printf("ptr_gcs2ap_radio->set_charge_voltage=%d\n",ptr_gcs2ap_radio->set_charge_voltage);
    printf("ptr_gcs2ap_radio->set_charge_current=%d\n",ptr_gcs2ap_radio->set_charge_current);
    printf("ptr_gcs2ap_radio->wp_flag=%d\n",ptr_gcs2ap_radio->wp_flag);
    printf("ptr_gcs2ap_radio->wp_next=%d\n",ptr_gcs2ap_radio->wp_next);
    printf("ptr_gcs2ap_radio->spd=%d\n",ptr_gcs2ap_radio->spd);
    printf("ptr_gcs2ap_radio->alt=%d\n",ptr_gcs2ap_radio->alt);
    printf("ptr_gcs2ap_radio->lng=%d\n",ptr_gcs2ap_radio->lng);
    printf("ptr_gcs2ap_radio->lat=%d\n",ptr_gcs2ap_radio->lat);
#endif

    if(ptr_gcs2ap_radio->pack_func_flag==0xac)
    {
        switch(ptr_gcs2ap_radio->pack_func_info1)
        {
        case 0x55:
            //主控副控都重启
            global_bool_boatpilot.bool_shutdown_master=TRUE;
            global_bool_boatpilot.bool_shutdown_slave=TRUE;
            break;
        case 0x05:
            //主控重启
            global_bool_boatpilot.bool_shutdown_master=TRUE;
            global_bool_boatpilot.bool_shutdown_slave=FALSE;
            break;
        case 0x50:
            //副控重启
            global_bool_boatpilot.bool_shutdown_master=FALSE;
            global_bool_boatpilot.bool_shutdown_slave=TRUE;
            break;
        default:
            global_bool_boatpilot.bool_shutdown_master=FALSE;
            global_bool_boatpilot.bool_shutdown_slave=FALSE;
            break;
        }
    }

    switch(ptr_gcs2ap_radio->generator_on)
    {
    case GENERATOR_STOP:
        //停止
//        close_generator();
        global_bool_boatpilot.bool_generator_on=FALSE;
        break;
    case GENERATOR_START:
        //工作
//        open_generator();
        global_bool_boatpilot.bool_generator_on=TRUE;
        break;
    case GENERATOR_AUTO:
        //自动
        /*
         * 副控请求主控开启发电机指令
         * 有变化时才进行继电器操作
         * 就是因为这一段，导致20170119 浪费一晚上查找推进器无缘无故失效了的原因
         */
//        if(global_bool_boatpilot.s2m_generator_onoff_req_previous!=data_s2m.generator_onoff_req)
//        {
//            if(data_s2m.generator_onoff_req==TRUE)
//            {
//                open_generator();
//                global_bool_boatpilot.bool_generator_on=TRUE;
//            }
//            else
//            {
//                close_generator();
//                global_bool_boatpilot.bool_generator_on=FALSE;
//            }
//            global_bool_boatpilot.s2m_generator_onoff_req_previous=data_s2m.generator_onoff_req;
//        }
//        data_m2s.generator_state=global_bool_boatpilot.bool_generator_on;
        break;
    default:
        break;
    }

    if(ptr_gcs2ap_radio->thruster_backward==MOTOR_FORWARD)
    {
        //电机正转，前进
        set_left_motor_forward();
        set_right_motor_forward();
    }
    else
    {
        //电机反转，后退
        set_left_motor_backward();
        set_right_motor_backward();
    }

    if(ptr_gcs2ap_radio->motor_lock & 0x01)
    {
        //右电机解锁
        set_right_motor_on();
    }
    else
    {
        set_right_motor_off();
    }

    if((ptr_gcs2ap_radio->motor_lock >> 1) & 0x01)
    {
        //左电机解锁
        //printf("左电机解锁\n");
        set_left_motor_on();
    }
    else
    {
        set_left_motor_off();
    }

    if(ptr_gcs2ap_radio->middle_motor_on)
    {
        start_third_small_motor();
    }
    else
    {
        stop_third_small_motor();
    }
//
//    //火箭舱盖
//    if(ptr_gcs2ap_radio->rocket_hat)
//    {
//        //打开舱盖
//        data_m2s.rkt.open_rocket_hatch=TRUE;
//    }
//    else
//    {
//        data_m2s.rkt.open_rocket_hatch=FALSE;
//    }
//
//    //火箭发射
//    if(global_bool_boatpilot.launch_req_ack_cnt_previous!=data_s2m.rkt.launch_req_ack)
//    {
//        printf("发射火箭\n");
//        data_m2s.rkt.launch_req=TRUE;
//        global_bool_boatpilot.launch_req_ack_cnt_previous=data_s2m.rkt.launch_req_ack;
//    }

    //转弯模式
    if((ptr_gcs2ap_radio->turn_mode & 0x01) && ((ptr_gcs2ap_radio->turn_mode >> 1) & 0x01))
    {
        global_bool_boatpilot.turn_mode=TURN_MODE_MIX;
    }
    else if(ptr_gcs2ap_radio->turn_mode & 0x01)
    {
        global_bool_boatpilot.turn_mode=TURN_MODE_RUDDER;
    }
    else if((ptr_gcs2ap_radio->turn_mode >> 1) & 0x01)
    {
        global_bool_boatpilot.turn_mode=TURN_MODE_DIFFSPD;
    }
    else
    {
        //global_bool_boatpilot.turn_mode=TURN_MODE_RUDDER;
        global_bool_boatpilot.turn_mode=TURN_MODE_DIFFSPD;//默认差速控制
    }

//    if((ptr_gcs2ap_radio->rudder_calib>>7 & 0x01)==1)
//    {
//        //进入标定状态
//        global_bool_boatpilot.bool_is_calib_rudder=TRUE;
//        printf("进入方向舵标定状态\n");
//
//        //static float left_detla_fabs;
//        //static float right_detla_fabs;
//        static short left_detla_fabs;
//        static short right_detla_fabs;
//        static short min_delta_fabs=10;//10对应5度(左右各5度)
//        static short max_delta_fabs=90;//90对应45度(左右各45度)
//        if(global_bool_boatpilot.rudder_calib_cnt_previous!=ptr_gcs2ap_radio->rudder_calib_cnt)
//        {
//            global_bool_boatpilot.bool_rudder_calib_success=FALSE;
//            printf("地面站-->标定方向舵\n");
//            if((ptr_gcs2ap_radio->rudder_calib & 0x01)==1)
//            {
//                //舵标定，到达左舵限位
//                ptr_gcs2ap_radio->rudder_left_pos=ptr_gcs2ap_radio->rc_rudder;
//                global_bool_boatpilot.rudder_left_limit_position=read_encoder.postion;
//                printf("global_bool_boatpilot.rudder_left_limit_position=%d\n",global_bool_boatpilot.rudder_left_limit_position);
//                left_detla_fabs=fabs(global_bool_boatpilot.rudder_middle_position-global_bool_boatpilot.rudder_left_limit_position);
//            }
//            if(((ptr_gcs2ap_radio->rudder_calib >> 1) & 0x01)==1)
//            {
//                //舵标定，到达右舵限位
//                ptr_gcs2ap_radio->rudder_right_pos=ptr_gcs2ap_radio->rc_rudder;
//                global_bool_boatpilot.rudder_right_limit_position=read_encoder.postion;
//                printf("global_bool_boatpilot.rudder_right_limit_position=%d\n",global_bool_boatpilot.rudder_right_limit_position);
//                right_detla_fabs=fabs(global_bool_boatpilot.rudder_right_limit_position-global_bool_boatpilot.rudder_middle_position);
//            }
//            if(((ptr_gcs2ap_radio->rudder_calib >> 2) & 0x01)==1)
//            {
//                //舵标定，到达中间位置
//                ptr_gcs2ap_radio->rudder_mid_pos=ptr_gcs2ap_radio->rc_rudder;
//                global_bool_boatpilot.rudder_middle_position=read_encoder.postion;
//                printf("global_bool_boatpilot.rudder_middle_position=%d\n",global_bool_boatpilot.rudder_middle_position);
//
//                left_detla_fabs=fabs(global_bool_boatpilot.rudder_middle_position-global_bool_boatpilot.rudder_left_limit_position);
//                right_detla_fabs=fabs(global_bool_boatpilot.rudder_right_limit_position-global_bool_boatpilot.rudder_middle_position);
//            }
//
//            //10对应的是5度
//            if(right_detla_fabs>min_delta_fabs && left_detla_fabs>min_delta_fabs)
//            {
//                if(right_detla_fabs>left_detla_fabs)
//                {
//                    global_bool_boatpilot.rudder_delta_fabs=left_detla_fabs;
//                }
//                else
//                {
//                    global_bool_boatpilot.rudder_delta_fabs=right_detla_fabs;
//                }
//
//                if(global_bool_boatpilot.rudder_delta_fabs>min_delta_fabs && global_bool_boatpilot.rudder_delta_fabs<max_delta_fabs)
//                {
//                    global_bool_boatpilot.bool_rudder_calib_success=TRUE;
//                }
//                else
//                {
//                    global_bool_boatpilot.bool_rudder_calib_success=FALSE;
//                }
//            }
//            else
//            {
//                global_bool_boatpilot.bool_rudder_calib_success=FALSE;
//            }
//
//            global_bool_boatpilot.rudder_calib_cnt_previous=ptr_gcs2ap_radio->rudder_calib_cnt;
//        }
//
//    }
//    else
//    {
//        global_bool_boatpilot.bool_is_calib_rudder=FALSE;
//    }

//    if(global_bool_boatpilot.set_switch_channel_previous!=ptr_gcs2ap_radio->set_switch_channel)
//    {
//        if(ptr_gcs2ap_radio->set_switch_channel==SWITCH_AUTO)
//        {
//            data_m2s.switcher.auto_req=TRUE;
//            data_m2s.switcher.disable_req=FALSE;
//            data_m2s.switcher.ch0_on_req=FALSE;
//            data_m2s.switcher.ch1_on_req=FALSE;
//        }
//        if(ptr_gcs2ap_radio->set_switch_channel==SWITCH_STOP)
//        {
//            data_m2s.switcher.auto_req=FALSE;
//            data_m2s.switcher.disable_req=TRUE;
//            data_m2s.switcher.ch0_on_req=FALSE;
//            data_m2s.switcher.ch1_on_req=FALSE;
//        }
//        if(ptr_gcs2ap_radio->set_switch_channel==SWITCH_CHANNEL_0)
//        {
//            data_m2s.switcher.auto_req=FALSE;
//            data_m2s.switcher.disable_req=FALSE;
//            data_m2s.switcher.ch0_on_req=TRUE;
//            data_m2s.switcher.ch1_on_req=FALSE;
//        }
//        if(ptr_gcs2ap_radio->set_switch_channel==SWITCH_CHANNEL_1)
//        {
//            data_m2s.switcher.auto_req=FALSE;
//            data_m2s.switcher.disable_req=FALSE;
//            data_m2s.switcher.ch0_on_req=FALSE;
//            data_m2s.switcher.ch1_on_req=TRUE;
//        }
//        global_bool_boatpilot.set_switch_channel_previous=ptr_gcs2ap_radio->set_switch_channel;
//    }
//
//    if(global_bool_boatpilot.voltage_llim_previous!=ptr_gcs2ap_radio->set_switch_low_limit)
//    {
//        data_m2s.switcher.llim_req=TRUE;
//        data_m2s.switcher.voltage_llim=ptr_gcs2ap_radio->set_switch_low_limit ;
//
//        global_bool_boatpilot.voltage_llim_previous=ptr_gcs2ap_radio->set_switch_low_limit;
//    }
//
//    if(global_bool_boatpilot.voltage_hlim_previous!=ptr_gcs2ap_radio->set_switch_high_limit)
//    {
//        data_m2s.switcher.hlim_req=TRUE;
//        data_m2s.switcher.voltage_hlim=ptr_gcs2ap_radio->set_switch_high_limit ;
//
//        global_bool_boatpilot.voltage_hlim_previous=ptr_gcs2ap_radio->set_switch_high_limit ;
//    }
//
//    if(global_bool_boatpilot.charge_start_previous!=ptr_gcs2ap_radio->charge_start)
//    {
//        data_m2s.charger.turn_on_req=ptr_gcs2ap_radio->charge_start;
//        if(data_m2s.charger.turn_on_req)
//        {
//            data_m2s.charger.turn_off_req=FALSE;
//        }
//        else
//        {
//            data_m2s.charger.turn_off_req=TRUE;
//        }
//
//        global_bool_boatpilot.charge_start_previous=ptr_gcs2ap_radio->charge_start;
//    }
//
//    if(global_bool_boatpilot.charger_set_channel_previous!=ptr_gcs2ap_radio->set_charge_channel)
//    {
//        data_m2s.charger.set_channel_req=TRUE;
//        data_m2s.charger.set_channel=ptr_gcs2ap_radio->set_charge_channel;
//
//        global_bool_boatpilot.charger_set_channel_previous=ptr_gcs2ap_radio->set_charge_channel;
//    }
//
//    if(global_bool_boatpilot.charger_set_voltage_previous!=ptr_gcs2ap_radio->set_charge_voltage )
//    {
//        data_m2s.charger.set_voltage_req=TRUE;
//        data_m2s.charger.set_voltage=ptr_gcs2ap_radio->set_charge_voltage ;
//
//        global_bool_boatpilot.charger_set_voltage_previous=ptr_gcs2ap_radio->set_charge_voltage;
//    }
//
//    if(global_bool_boatpilot.charger_set_current_previous!=ptr_gcs2ap_radio->set_charge_current)
//    {
//        data_m2s.charger.set_current_req=TRUE;
//        data_m2s.charger.set_current=ptr_gcs2ap_radio->set_charge_current;
//
//        global_bool_boatpilot.charger_set_current_previous=ptr_gcs2ap_radio->set_charge_current;
//    }

    if(ptr_gcs2ap_radio->rudder_dead_zone_angle_degree<=1)
    {
        ptr_gcs2ap_radio->rudder_dead_zone_angle_degree=1;
    }
    else if(ptr_gcs2ap_radio->rudder_dead_zone_angle_degree>=10)
    {
        ptr_gcs2ap_radio->rudder_dead_zone_angle_degree=10;
    }

//    if(global_bool_boatpilot.slave_config_previous!=ptr_gcs2ap_radio->slave_config)
//    {
//        printf("电流读取使能发生了变化\n");
//        if(ptr_gcs2ap_radio->slave_config & (0x01<<2))
//        {
//            data_m2s.rs485_read_current_ena[0]=TRUE;
//        }
//        else
//        {
//            data_m2s.rs485_read_current_ena[0]=FALSE;
//        }
//
//        if(ptr_gcs2ap_radio->slave_config & (0x01<<3))
//        {
//            data_m2s.rs485_read_current_ena[1]=TRUE;
//        }
//        else
//        {
//            data_m2s.rs485_read_current_ena[1]=FALSE;
//        }
//
//        global_bool_boatpilot.slave_config_previous=ptr_gcs2ap_radio->slave_config;
//    }

    if(ptr_gcs2ap_radio->wp_flag==0)
    {
        //命令包中的航点无效，不给航点数组赋值

        //printf("AUTO_MISSION_MODE\n");
        ptr_gcs2ap_radio->auto_work_mode=AUTO_MISSION_MODE;//auto自动模式下，始终是mission模式，除非wp_flag始终有特殊值
    }
    else if(ptr_gcs2ap_radio->wp_flag==255)
    {
        //自动模式下的引导工作模式，设置当前目标航点的计数
        printf("设置引导航点，航点标号=%d\n",ptr_gcs2ap_radio->wp_next);
        ptr_gcs2ap_radio->auto_work_mode=AUTO_GUIDE_MODE;
        ptr_gcs2ap_radio->wp_guide_no=ptr_gcs2ap_radio->wp_next;
    }
    else if(ptr_gcs2ap_radio->wp_flag==254)
    {
        //自动模式下的逗留工作模式
    }
    else if(ptr_gcs2ap_radio->wp_flag==253)
    {
        global_bool_boatpilot.bool_loiter_mode=FALSE;
    }
    else
    {
        int wp_cnt;
        wp_cnt=ptr_gcs2ap_radio->wp_next;
        wp_data[wp_cnt].spd=ptr_gcs2ap_radio->spd;
        wp_data[wp_cnt].alt=ptr_gcs2ap_radio->alt;
        wp_data[wp_cnt].lng=ptr_gcs2ap_radio->lng;
        wp_data[wp_cnt].lat=ptr_gcs2ap_radio->lat;
        printf("修改第%d航点的经纬度\n",wp_cnt);
        printf("wp_data[%d].spd=%d\n",wp_cnt,wp_data[wp_cnt].spd);
        printf("wp_data[%d].spd=%d\n",wp_cnt,wp_data[wp_cnt].spd);
        printf("wp_data[%d].spd=%d\n",wp_cnt,wp_data[wp_cnt].spd);
        printf("wp_data[%d].spd=%d\n",wp_cnt,wp_data[wp_cnt].spd);
    }

    return 0;
}

int generate_packet(unsigned char*dst_buf, unsigned char *src_buf,unsigned char len,\
                    unsigned int packet_cnt, unsigned char message_type,\
                    unsigned char commu_method, unsigned char ack_req)
{
    static unsigned char frame_head_len=8;
    static unsigned char frame_end_len=4;
    unsigned char packet[128];
    unsigned char checksum = 0;

    int i, j;
    int packet_data_len;

    packet[0] = 0xaa;
    packet[1] = 0x55;
    //packet[2] = len;
    packet[2] = 80;//统一定义为80字节，包含帧头帧尾
    packet_data_len = len;

    packet[3] = packet_cnt;

    packet[4] = ADDRESS_BOATPILOT;
    packet[5] = message_type;

    packet[6]=commu_method;
    packet[7]=ack_req;

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
    packet[i]=global_bool_boatpilot.master_state;
    packet[i+1]=global_bool_boatpilot.slave_state;
    packet[i+2]=0;
    checksum=checksum+packet[i]+packet[i+1]+packet[i+2];
    packet[i+3] = (checksum & 0xFF);

    i += 1;

    memcpy(dst_buf, packet, packet_data_len + frame_head_len + frame_end_len);

    /*返回总的发送字节数*/
    return packet_data_len + frame_head_len + frame_end_len;
}

/*
 * 测试保存的二进制文件是否成功，把二进制文件再读回来然后保存为字符串
 */
#define BOATPILOT_LOG_TXT "boatpilot_log.txt"
#define BOATPILOT_BINARY_FILE "boatpilot.log"

int decode_binary_data()
{
//如果不需要解析数据decode_binary_data，那就把下面的#if 1改为 #if 0
#if 1
    struct stat f_stat;

    int fd_boatpilot_log_txt=0;
    fd_boatpilot_log_txt=create_log_file(BOATPILOT_LOG_TXT);//把二进制的log文件转为文本文件保存在BOATPILOT_LOG_TXT中

    /*
     * 经过验证，下面的程序可以把二进制文件解析出来再保存到txt文件中便于分析
     */
    int fd_boatpilot_log_binary=0;
    int boatpilot_len=0;
    static char boatpilot_save_data[4096]={"hello"};//截至20170512日志文件是244个字节，所以数组定义为256是够用的
    int i=0;

    fd_boatpilot_log_binary=open(BOATPILOT_BINARY_FILE,O_RDWR |O_CREAT);
    stat( BOATPILOT_BINARY_FILE, &f_stat );
    printf("f_stat.st_size=%u\n",(unsigned int)f_stat.st_size);
    printf("sizeof(boatpilot_log)=%d\n",sizeof(boatpilot_log));

    for(i=0;i<f_stat.st_size/sizeof(boatpilot_log);i++)
    {
        lseek(fd_boatpilot_log_binary,i*sizeof(boatpilot_log),SEEK_SET);
        boatpilot_len=read(fd_boatpilot_log_binary, &boatpilot_log, sizeof(boatpilot_log));
        printf("boatpilot_len=%d\n",boatpilot_len);
        printf("boatpilot_log.year=%d\n",boatpilot_log.year);
        printf("boatpilot_log.gcs2ap_radio.turn_mode=%d\n",boatpilot_log.gcs2ap_radio.turn_mode);
        printf("boatpilot_log.ap2gcs_real.lng=%d\n",boatpilot_log.ap2gcs_real.lng);
        printf("boatpilot_log.ap2gcs_real.lat=%d\n",boatpilot_log.ap2gcs_real.lat);
        printf("boatpilot_log.global.wp_total_num=%d\n",boatpilot_log.global.wp_total_num);

        /*截至2017年05月12日-总共 个数据*/
        boatpilot_len = sprintf(boatpilot_save_data,\
                        "%hu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,\
                        %u,%u,%u,\
                        %hhu,%hhu,%hhu,%hhu,\
                        %u,%u,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,\
                        %u,%u,\
                        %hu,%hd,%hd,%hd,%hd,%hd,%hd,%hd,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,\
                        %hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,\
                        %d,\
                        %hd,%hd,%hd,%hd\n",\
                        boatpilot_log.year,boatpilot_log.month,boatpilot_log.day,boatpilot_log.hour,boatpilot_log.minute,boatpilot_log.second,boatpilot_log.stuffing,\
                        boatpilot_log.gcs2ap_radio.pack_func_flag,boatpilot_log.gcs2ap_radio.pack_func_info1,boatpilot_log.gcs2ap_radio.pack_func_info2,boatpilot_log.gcs2ap_radio.pack_func_info3,boatpilot_log.gcs2ap_radio.workmode,boatpilot_log.gcs2ap_radio.rc_thruster,boatpilot_log.gcs2ap_radio.rc_rudder,boatpilot_log.gcs2ap_radio.rud_p,\
                        boatpilot_log.gcs2ap_radio.rud_i,boatpilot_log.gcs2ap_radio.rud_d,boatpilot_log.gcs2ap_radio.cte_p,boatpilot_log.gcs2ap_radio.cte_i,boatpilot_log.gcs2ap_radio.cte_d,boatpilot_log.gcs2ap_radio.rudder_setup_reverse,boatpilot_log.gcs2ap_radio.thruster_setup_reverse,boatpilot_log.gcs2ap_radio.generator_on,\
                        boatpilot_log.gcs2ap_radio.thruster_backward,boatpilot_log.gcs2ap_radio.motor_lock,boatpilot_log.gcs2ap_radio.middle_motor_on,boatpilot_log.gcs2ap_radio.navigation_mode,boatpilot_log.gcs2ap_radio.charge_start,boatpilot_log.gcs2ap_radio.rocket_hat,boatpilot_log.gcs2ap_radio.rocket_launch,boatpilot_log.gcs2ap_radio.turn_mode,\
                        boatpilot_log.gcs2ap_radio.diffspd_coef,boatpilot_log.gcs2ap_radio.diffspd_lim,boatpilot_log.gcs2ap_radio.cruise_throttle_percent,boatpilot_log.gcs2ap_radio.throttle_change_time,boatpilot_log.gcs2ap_radio.arrive_radius,boatpilot_log.gcs2ap_radio.cte_max_degree,boatpilot_log.gcs2ap_radio.rudder_calib,boatpilot_log.gcs2ap_radio.rudder_calib_cnt,\
                        boatpilot_log.gcs2ap_radio.set_switch_channel,boatpilot_log.gcs2ap_radio.set_switch_low_limit,boatpilot_log.gcs2ap_radio.set_switch_high_limit,boatpilot_log.gcs2ap_radio.set_charge_channel,boatpilot_log.gcs2ap_radio.set_charge_voltage,boatpilot_log.gcs2ap_radio.set_charge_current,boatpilot_log.gcs2ap_radio.rudder_dead_zone_angle_degree,boatpilot_log.gcs2ap_radio.master_config,\
                        boatpilot_log.gcs2ap_radio.slave_config,boatpilot_log.gcs2ap_radio.spare1_0,boatpilot_log.gcs2ap_radio.spare1_1,boatpilot_log.gcs2ap_radio.spare1_2,\
                        boatpilot_log.gcs2ap_radio.spare2_int,boatpilot_log.gcs2ap_radio.spare3_int,boatpilot_log.gcs2ap_radio.spare4_int,\
                        boatpilot_log.gcs2ap_radio.wp_flag,boatpilot_log.gcs2ap_radio.wp_next,boatpilot_log.gcs2ap_radio.spd,boatpilot_log.gcs2ap_radio.alt,\
                        boatpilot_log.gcs2ap_radio.lng,boatpilot_log.gcs2ap_radio.lat,\
                        boatpilot_log.gcs2ap_radio.mmotor_on_pos,boatpilot_log.gcs2ap_radio.mmotor_off_pos,boatpilot_log.gcs2ap_radio.rudder_left_pos,boatpilot_log.gcs2ap_radio.rudder_right_pos,boatpilot_log.gcs2ap_radio.rudder_mid_pos,boatpilot_log.gcs2ap_radio.auto_work_mode,boatpilot_log.gcs2ap_radio.wp_guide_no,boatpilot_log.gcs2ap_radio.spare+121,\
                        boatpilot_log.ap2gcs_real.pack_func_flag,boatpilot_log.ap2gcs_real.pack_func_info1,boatpilot_log.ap2gcs_real.pack_func_info2,boatpilot_log.ap2gcs_real.pack_func_info3,\
                        boatpilot_log.ap2gcs_real.lng,boatpilot_log.ap2gcs_real.lat,\
                        boatpilot_log.ap2gcs_real.spd,boatpilot_log.ap2gcs_real.dir_gps,boatpilot_log.ap2gcs_real.dir_heading,boatpilot_log.ap2gcs_real.dir_target,boatpilot_log.ap2gcs_real.dir_nav,boatpilot_log.ap2gcs_real.roll,boatpilot_log.ap2gcs_real.pitch,boatpilot_log.ap2gcs_real.yaw,\
                        boatpilot_log.ap2gcs_real.codedisc,boatpilot_log.ap2gcs_real.da_out1,boatpilot_log.ap2gcs_real.da_out2,boatpilot_log.ap2gcs_real.rudder_pos,boatpilot_log.ap2gcs_real.rc_thruster,boatpilot_log.ap2gcs_real.rc_rudder,boatpilot_log.ap2gcs_real.rud_p,boatpilot_log.ap2gcs_real.cte_p,\
                        boatpilot_log.ap2gcs_real.boat_temp1,boatpilot_log.ap2gcs_real.boat_temp1,boatpilot_log.ap2gcs_real.humi,boatpilot_log.ap2gcs_real.voltage_bat1,boatpilot_log.ap2gcs_real.voltage_bat2,boatpilot_log.ap2gcs_real.current_bat1,boatpilot_log.ap2gcs_real.current_bat2,boatpilot_log.ap2gcs_real.toggle_state,\
                        boatpilot_log.ap2gcs_real.charge_state,boatpilot_log.ap2gcs_real.temp,boatpilot_log.ap2gcs_real.humi,boatpilot_log.ap2gcs_real.windspeed,boatpilot_log.ap2gcs_real.winddir,boatpilot_log.ap2gcs_real.airpress,boatpilot_log.ap2gcs_real.seasault,boatpilot_log.ap2gcs_real.elec_cond,\
                        boatpilot_log.ap2gcs_real.seatemp1,boatpilot_log.ap2gcs_real.seatemp2,boatpilot_log.ap2gcs_real.seatemp3,boatpilot_log.ap2gcs_real.alt,boatpilot_log.ap2gcs_real.radiation,boatpilot_log.ap2gcs_real.launch_req_ack,boatpilot_log.ap2gcs_real.rocket_state,boatpilot_log.ap2gcs_real.rktnumber,\
                        boatpilot_log.ap2gcs_real.rkt_alt,boatpilot_log.ap2gcs_real.work_mode,boatpilot_log.ap2gcs_real.charger_voltage,boatpilot_log.ap2gcs_real.charger_current,boatpilot_log.ap2gcs_real.spare3,boatpilot_log.ap2gcs_real.spare4,boatpilot_log.ap2gcs_real.spare5,boatpilot_log.ap2gcs_real.wp_next,\
                        boatpilot_log.global.bool_get_gcs2ap_cmd,boatpilot_log.global.bool_get_gcs2ap_waypoint,boatpilot_log.global.bool_gcs2ap_beidou,boatpilot_log.global.bool_generator_on,boatpilot_log.global.bool_is_sending_wp_ap2gcs,boatpilot_log.global.bool_beidou_get_gcs2ap_cmd,boatpilot_log.global.bool_beidou_get_gcs2ap_waypoint,boatpilot_log.global.bool_loiter_mode,\
                        boatpilot_log.global.bool_shutdown_master,boatpilot_log.global.bool_shutdown_slave,boatpilot_log.global.bool_rudder_calib_success,boatpilot_log.global.bool_is_calib_rudder,boatpilot_log.global.turn_mode,boatpilot_log.global.s2m_generator_onoff_req_previous,boatpilot_log.global.radio_recv_packet_cnt,boatpilot_log.global.radio_recv_packet_cnt_previous,\
                        boatpilot_log.global.udp_recv_packet_cnt,boatpilot_log.global.wp_total_num,boatpilot_log.global.send_ap2gcs_wp_req,boatpilot_log.global.ap2gcs_wp_cnt_previous,boatpilot_log.global.ap2gcs_wp_cnt,boatpilot_log.global.send_ap2gcs_real_req,boatpilot_log.global.ap2gcs_real_cnt_previous,boatpilot_log.global.ap2gcs_real_cnt,\
                        boatpilot_log.global.send_ap2gcs_cmd_req,boatpilot_log.global.ap2gcs_cmd_cnt_previous,boatpilot_log.global.ap2gcs_cmd_cnt,boatpilot_log.global.send_m2s_udp_req,boatpilot_log.global.m2s_udp_cnt_previous,boatpilot_log.global.m2s_udp_cnt,boatpilot_log.global.bd_send_ap2gcs_wp_req,boatpilot_log.global.bd_ap2gcs_wp_cnt_previous,\
                        boatpilot_log.global.bd_ap2gcs_wp_cnt,boatpilot_log.global.bd_send_ap2gcs_real_req,boatpilot_log.global.bd_ap2gcs_real_cnt_previous,boatpilot_log.global.bd_ap2gcs_real_cnt,boatpilot_log.global.bd_send_ap2gcs_cmd_req,boatpilot_log.global.bd_ap2gcs_cmd_cnt_previous,boatpilot_log.global.bd_ap2gcs_cmd_cnt,boatpilot_log.global.rudder_calib_cnt_previous,\
                        boatpilot_log.global.launch_req_ack_cnt_previous,boatpilot_log.global.save_boatpilot_log_req,boatpilot_log.global.wp_packet_cnt,boatpilot_log.global.assign_config_req,boatpilot_log.global.assign_config_cnt_previous,boatpilot_log.global.assign_config_cnt,boatpilot_log.global.save_config_req,boatpilot_log.global.set_switch_channel_previous,\
                        boatpilot_log.global.voltage_llim_previous,boatpilot_log.global.voltage_hlim_previous,boatpilot_log.global.bat0_is_discharing,boatpilot_log.global.bat1_is_discharing,boatpilot_log.global.charger_set_channel_previous,boatpilot_log.global.charger_set_voltage_previous,boatpilot_log.global.charger_set_current_previous,boatpilot_log.global.charge_start_previous,\
                        boatpilot_log.global.wp_next,boatpilot_log.global.send_ap2gcs_wp_start_num,boatpilot_log.global.send_ap2gcs_wp_end_num,boatpilot_log.global.send_ap2gcs_specific_wp_req,boatpilot_log.global.master_state,boatpilot_log.global.slave_state,boatpilot_log.global.slave_config_previous,boatpilot_log.global.spare0,\
                        boatpilot_log.global.left_motor_voltage,boatpilot_log.global.right_motor_voltage,boatpilot_log.global.rudder_angle_degree,boatpilot_log.global.cte_error_check_radian,boatpilot_log.global.current_to_target_radian,boatpilot_log.global.command_radian,boatpilot_log.global.dir_target_degree,boatpilot_log.global.dir_nav_degree,\
                        boatpilot_log.global.cte_distance_error,\
                        boatpilot_log.global.rudder_middle_position,boatpilot_log.global.rudder_left_limit_position,boatpilot_log.global.rudder_right_limit_position,boatpilot_log.global.rudder_delta_fabs);
        printf("boatpilot_len转换为字符串的字节数=%d\n",boatpilot_len);

        save_data_to_string_log(fd_boatpilot_log_txt,boatpilot_save_data,boatpilot_len);
    }

    exit(0);
#endif

    return 0;
}
