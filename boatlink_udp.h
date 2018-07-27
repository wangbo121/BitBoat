/*
 *@File     : boatlink_udp.h
 *@Author   : wangbo
 *@Date     : Mar 6, 2017
 *@Copyright: 2018 Beijing Institute of Technology. All rights reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */


#ifndef BOATLINK_UDP_H_
#define BOATLINK_UDP_H_

#define COMMAND_AP2GCS_REAL_UDP      0x01
#define COMMAND_GCS2AP_CMD_UDP       0x02
#define COMMAND_GCS2AP_WP_UDP        0x03

struct AP2GCS_REAL_UDP
{
    unsigned char head1;
    unsigned char head2;
    unsigned short len;      //到此4个字节

    unsigned char type;
    unsigned char vessel;
    unsigned char master_ap_link_ack;
    unsigned char plan_id;
    unsigned char cnt;
    unsigned char pack_func_flag;//包功能标志，暂时固定为0
    unsigned char pack_func_info1;//接收到的命令包计数
    unsigned char pack_func_info2;//接收到的航点包计数                                       //到此12个字节

//    unsigned int lng;//[度*0.00001]，GPS经度坐标，整型，精确到米
//    unsigned int lat;//[度*0.00001]，GPS纬度坐标，整型，精确到米                      //到此20个字节
    int32_t lng;//[度*0.0000001]，GPS经度坐标，整型，10^-7 精确到厘米
    int32_t lat;//[度*0.0000001]，GPS纬度坐标，整型，10^-7 精确到厘米                      //到此20个字节
    unsigned short spd;//[Knot*0.01]，实时航速
    short dir_gps;//[度*0.01]，地速航向，GPS航向
    short dir_heading;//[度*0.01]，机头朝向
    short dir_target;//[度*0.01]，目标点朝向
    short dir_nav;//[度*0.01]，导航航向
    short roll;//[度*0.01]，滚转
    short pitch;//[度*0.01]，俯仰
    short yaw;//[度*0.01]，偏航                                                                                       //到此36个字节

    unsigned char wp_next;
    unsigned char sail_mode;//遥控 自驾 驻航 返航
    unsigned char form_type;
    unsigned char pilot_vessel;                                                                                //到此40给字节

    unsigned char wp_success_cnt;
    unsigned char spare_char_0;
    unsigned char spare_char_1;
    unsigned char spare_char_2;


    unsigned int spare2;
    unsigned int spare3;
    unsigned int spare4;
    unsigned int spare5;
    unsigned int spare6;
    unsigned int spare7;
    unsigned int spare8;                                                                                         //到此72个字节

    unsigned char check_spare0;
    unsigned char check_spare1;
    unsigned char check1;
    unsigned char check2;
};

struct GCS2AP_CMD_UDP
{
	unsigned char head1;
	unsigned char head2;
	unsigned short len;// !!!! 注意这个是short型，到此4个字节
	unsigned char type;
	unsigned char vessel_ID;
	unsigned char master_ap_link_ack;
	unsigned char gcs_ID;
	unsigned char cnt;
	unsigned char func_flag;
	unsigned char func_info1;
	unsigned char func_info2;//到此12个字节

	unsigned char system_calib;   //12.  0x01:acc和gyro校准, 0x02:mag校准, 0x03:baro校准，0x04:重启
	unsigned char actutor_calib;//13.推进器正反向设置, D1:左侧,D0:右侧. 1:反向,0:正向
	unsigned char actutor_balance_coef;//14. 左右推进器推力比调整系数,左:右[50~200]%,超出为异常，可默认为100
	unsigned char navigation_mode;//15.导航模式,0:船头导航;1:航迹导航. 在船没有建立速度和航速方向时，使用姿态或磁航向传感器建立船头方向

	unsigned char sail_mode;//16:航行模式.D7~D4备用; D3~D1: 自驾类型: 001:巡航;010:返航;011:驻航;100:停车; D0:遥控/自驾切换,D0:0:遥控,1:自驾
	unsigned char throttle;
	unsigned char rudder;
	unsigned char fwdbwd;

	unsigned char cruise_throttle_percent;//20:[0,100][%]巡航油门百分比数
    unsigned char throttle_change_time;//21:[秒],油门改变百分之10所用的时间
    unsigned char arrive_radius;//22:[米],到达半径
    unsigned char cte_max_degree;//23:[度],偏航距最大补偿角
    unsigned char diffspd_lim;//24:[%]差速调整限幅,0-100,即:50表示单侧最大增50%,最小50%
    unsigned char spare_para1;//25
    unsigned char spare_para2;//26
    unsigned char spare_para3;//27
    unsigned char spare_para4;//28
    unsigned char spare_para5;//29
    unsigned char spare_para6;//30
    unsigned char spare_para7;//31

	unsigned char controller_type;
	unsigned char ctrl_para_1;//33.当控制器类型为0(PID)时，此参数为yaw_p
	unsigned char ctrl_para_2;
	unsigned char ctrl_para_3;
	unsigned char ctrl_para_4;//36.当控制器类型为0(PID)时，此参数为cte_p
	unsigned char ctrl_para_5;
	unsigned char ctrl_para_6;
	unsigned char ctrl_para_7;
	unsigned char ctrl_para_8;
	unsigned char ctrl_para_9;
	unsigned char ctrl_para_10;
	unsigned char ctrl_para_11;
	unsigned char ctrl_para_12;
	unsigned char ctl_para_13;//45.
	unsigned char ctl_para_14;//46.
	unsigned char ctl_para_15;//47.

    unsigned char change_next_wp;//48 0表示wp_next无效，1表示请求修改下一航点
    unsigned char wp_next;//49.   下一个航点编号
    unsigned char multi_mode;//50. D6-4是form_type,0:独立航行，1：领导跟随，2分布式一致性编队。
    unsigned char pilot_vessel;//51.  领航船编号

    int32_t spare_int_0;//52~55
    int32_t spare_int_1;//56~59
    int32_t spare_int_2;//60~63
    int32_t spare_int_3;//64~67
    int32_t spare_int_4;//68~71

    unsigned char spare5;//72
    unsigned char spare6;//73
    unsigned char check1;//74
    unsigned char check2;//75
};

struct GCS2AP_ALL_UDP
{
    /*
     * GCS2AP_ALL_UDP接收从地面站传输过来的所有有价值的信息，都保存在这个变量结构中，但是除了航点
     * 航点数据直接保存在wp_data数组中，其实也可以把所有的航点数组放在这里面
     * 这里的变量名称跟GCS2AP_CMD_UDP是基本一致的
     * 但是因为GCS2AP_CMD_UDP中有一些变量是按位表示的
     * 所以需要根据对 位 的判断，翻译为unsigned char型的其他变量名称
     */

    struct GCS2AP_CMD_UDP cmd; // 这个结构是76个字节

    /*
     * 对master_ap_link_ack字节的翻译解释
     */
    unsigned char pilot_type;//是主驾驶仪还是备用驾驶仪类型，1是主驾驶仪，0是备用驾驶仪
    unsigned char pilot_cnt;//驾驶仪编号
    unsigned char link_ID;//链路编号 (0:局域网;1:北斗;2:串口数传电台)

    /*
     * 对controller_type字节的解释 可以是PID控制器，也可以是ADRC自抗扰控制器 也可以是SMC滑模控制器
     */
    unsigned char rud_p;//[0.1],转弯参数P  //8字节
    unsigned char rud_i;//[0.01],转弯参数I
    unsigned char rud_d;//[0.1],转弯参数D
    unsigned char cte_p;//[0.1],CTE参数P
    unsigned char cte_i;//[0.01],CTE参数I
    unsigned char cte_d;//[0.1],CTE参数D
    unsigned char cruise_throttle_percent;//[0-100%]巡航油门百分比数
    unsigned char arrive_radius; //[10米]

    /*
     * 对sail_mode字节的翻译解释
     */
    unsigned char workmode; //分为 遥控 和 自动驾驶 而自动驾驶下又有具体的自动方式 比如航点航行 guide航行 Return To Launch航行
    unsigned char auto_workmode;//自动模式下的工作模式，mission guide loiter

    unsigned char formation_type;//编队方式，0:独立航行 1领导跟随 2分布式
    unsigned char change_next_wp; // 是否更改下一航点 1:更改 0:不更改
    unsigned char wp_guide_no;//设定的航点编号

    //以下是通过命令包的位操作字节翻译后，需要得到的值
    unsigned char mmotor_on_pos;//电机启动时，遥控器的数值，也就是遥控器的最大值，
    unsigned char mmotor_off_pos;//电机关闭时，遥控器的数值，也就是遥控器的最小值，
    unsigned char rudder_left_pos;//方向舵处于最左边时，遥控器的数值，
    unsigned char rudder_right_pos;//方向舵处于最右边时，遥控器的数值，
    unsigned char rudder_mid_pos;//方向舵处于中间时，遥控器的数值，

    unsigned char wp_total_num;

    //struct WAY_POINT      wp_data[MAX_WAYPOINT_NUM]; //  255个航点 255*12 = 3060个字节


    //    unsigned char navigation_mode;//导航模式,1用航迹导航，0用偏航即船头导航
    //    unsigned char turn_mode;//转弯模式,D0=1:方向舵有效;D1=1:差速有效  //24字节
    //    unsigned char throttle_change_time;//[秒],油门改变百分之10所用的时间
    //
    //    unsigned char cte_max_degree;//[度],偏航距最大补偿角
    //    unsigned char diffspd_coef;//[%]差速调整系数,(0.1-1.9)*100，即：两侧不调为100，若=90表示左侧出力为90%，右侧为110%，若110表示：左侧110%,右侧90%
    //    unsigned char diffspd_lim;//[%]差速调整限幅,0-100,即:50表示单侧最大增50%,最小50%
    //    unsigned char rudder_dead_zone_angle_degree;//[度]方向舵控制闭环时所用的死区角度数
    //
};




/*
 * 一些通过地面站发送的参数需要记录在驾驶仪的flash中，
 * 地面站只需要同步，不需要重新设置
 */
struct T_CONFIG_UDP
{
    unsigned char workmode;
    unsigned char rud_p;
    unsigned char rud_i;
    unsigned char rud_d;
    unsigned char cte_p;
    unsigned char cte_i;
    unsigned char cte_d;
    unsigned char rudder_setup_reverse;
    unsigned char thruster_setup_reverse;
    unsigned char cruise_throttle_percent;//[0-100%]巡航油门百分比数
    unsigned char throttle_change_time;//[秒],油门改变百分之10所用的时间
    unsigned char arrive_radius;//[10米],到达半径
    unsigned char cte_max_degree;//[度],偏航距最大补偿角
    unsigned char rudder_left_pos;
    unsigned char rudder_right_pos;
    unsigned char rudder_mid_pos;
    unsigned char rudder_dead_zone_angle_degree;//[度]方向舵控制闭环时所用的死区角度数
    unsigned char total_wp_num;
    unsigned char current_target_wp_num;
    unsigned char spare0; // 20个字节


};

/*
 * Function:       send_ap2gcs_real_udp
 * Description:  驾驶仪向地面站发送ap2gcs_real_udp实时数据包
 */
int send_ap2gcs_real_udp();

/*
 * Function:       send_ap2gcs_cmd_udp
 * Description:  地面站请求回传命令时，发送/回传命令
 */
int send_ap2gcs_cmd_udp();

/*
 * Function:       decode_gcs2ap
 * Description:  解析由地面站发给自驾仪的命令包，解析后，
 *                        都放在gcs2ap结构的变量中
 */
int decode_gcs2ap_udp();

extern struct AP2GCS_REAL_UDP ap2gcs_real_udp;
extern struct GCS2AP_CMD_UDP gcs2ap_cmd_udp;

/*
 * 存放地面站传过来的除了航点数据包的所有数据
 * 因为地面站传过来的数据有些是用位来表示的，需要翻译成bool型或者标志量
 */
extern struct GCS2AP_ALL_UDP gcs2ap_all_udp;

extern struct T_CONFIG_UDP boatpilot_config_udp_previous;
extern struct T_CONFIG_UDP boatpilot_config_udp;

#endif /* BOATLINK_UDP_H_ */
