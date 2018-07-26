/*
 *@File     : GCS.h
 *@Author   : wangbo
 *@Date     : Jul 26, 2018
 *@Copyright: 2018 Beijing Institute of Technology. All rights reserved.
 *@Warning  : This content is limited to internal circulation, forbidding disclosure and for other commercial purposes.
 */
#ifndef GCS_H_
#define GCS_H_


#include <stdint.h>

typedef struct
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

    unsigned int lng;//[度*0.00001]，GPS经度坐标，整型，精确到米
    unsigned int lat;//[度*0.00001]，GPS纬度坐标，整型，精确到米                      //到此20个字节
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
}T_AP2GCS_REAL;

typedef struct
{
    unsigned char head1;
    unsigned char head2;
    unsigned short len;//注意这个是short型，到此4个字节
    unsigned char type;
    unsigned char vessel_ID;
    unsigned char master_ap_link_ack;
    unsigned char gcs_ID;
    unsigned char cnt;
    unsigned char func_flag;
    unsigned char func_info1;
    unsigned char func_info2;//到此12个字节

    unsigned char pilot_manual;// 控制信号是通过驾驶仪还是直接输出到执行器，这里控制的是个继电器，会有一个开关切换，保证信号直接输出到执行器，不通过驾驶仪，按道理这个应该总是1，也就是通过驾驶仪
    unsigned char throttle;
    unsigned char rudder;
    unsigned char fwdbwd;

    unsigned char controller_type;
    unsigned char ctrl_para_1;
    unsigned char ctrl_para_2;
    unsigned char ctrl_para_3;
    unsigned char ctrl_para_4;
    unsigned char ctrl_para_5;
    unsigned char ctrl_para_6;
    unsigned char ctrl_para_7;
    unsigned char ctrl_para_8;
    unsigned char ctrl_para_9;
    unsigned char ctrl_para_10;
    unsigned char ctrl_para_11;
    unsigned char ctrl_para_12;
    unsigned char sail_mode;
    unsigned char wp_next;
    unsigned char sensor_correct;//到此32个字节

    unsigned char pilot_vessel;
    unsigned char spare0;
    unsigned char spare1;
    unsigned char spare2;//到此36

    unsigned int spare3;
    unsigned int spare4;
    unsigned int spare5;
    unsigned int spare6;
    unsigned int spare7;
    unsigned int spare8;
    unsigned int spare9;
    unsigned int spare10;
    unsigned int spare11;// 到此72个字节

    unsigned char check_spare0;
    unsigned char check_spare1;
    unsigned char check1;
    unsigned char check2;// 到此76个字节
}T_GCS2AP_CMD;

typedef struct
{
    /*
     * GCS2AP_ALL_UDP接收从地面站传输过来的所有有价值的信息，都保存在这个变量结构中，但是除了航点
     * 航点数据直接保存在wp_data数组中，其实也可以把所有的航点数组放在这里面
     * 这里的变量名称跟GCS2AP_CMD_UDP是基本一致的
     * 但是因为GCS2AP_CMD_UDP中有一些变量是按位表示的
     * 所以需要根据对 位 的判断，翻译为unsigned char型的其他变量名称
     */

    T_GCS2AP_CMD cmd; // 这个结构是76个字节

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
}T_GCS2AP_ALL;

class GCS_Class
{
public:
    /*
     * 控制上的通信机制应该就2种
     * 1 是一问一答模式的，比如485 MSP(Multiwii Serial Protocol) 都是这种
     * 2 是主动式的，例如GPS这种就是，主动往外定时发送数据
     * update           函数解决第1种 一问一答模式的 解析地面站数据并考虑是否回复
     * data_stream_send 函数解决主动式的定时向地面站发送数据
     */

    /*
     * upddate函数用来解析地面站发送过来的数据
     * 或者那种一问一答的通信机制
     */
    //void      update(void) {}

    // 用来发送需要定时发送给地面站的数据
    //void data_stream_send();
};




#endif /* GCS_H_ */

