/*
 *@File     : location.h
 *@Author   : wangbo
 *@Date     : May 10, 2017
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */
#ifndef HEADERS_LOCATION_H_
#define HEADERS_LOCATION_H_

/*
 * 莫卡托坐标系的参考经度，在左边比较好，所以选择了东经110度，也就是说坐标系从东经110纬度0，为莫卡托坐标原点
 * 如果不在中国境内，则把这个参考经度置为0度
 */
#define REFERENCE_LONGITUDE_MERCATOR 110.0*M_PI/180.0

/*地球半径 单位：米[m]*/
#define EARTH_RADIUS 6378137.0

/*地球周长，莫卡托坐标系用的地球赤道周长*/
#define EARTH_CIRCUMFERENCE 20037508.34

#ifndef M_PI
#define M_PI       3.14159265358979323846
#define M_1_PI 0.31830988618379067154
#endif

struct T_LOCATION
{
	double lng;
	double lat;
	double alt;
};

struct T_NED
{
    double x;
    double y;
    double z;
};

/*
 * 把度转为弧度
 */
double convert_degree_to_radian(double degree);

/*
 * 把弧度转为度
 */
double convert_radian_to_degree(double radian);

/*
 * 把弧度值转为 -PI～+PI 之间
 * wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
*/
double wrap_PI(double angle_in_radians);

/*
 * 把弧度值转为 0～2PI之间
 * wrap an angle in radians to 0..2PI
*/
double wrap_2PI(double angle);

/*
 * 利用墨卡托投影算法计算得到地球表面两个位置的距离
 */
double get_distance_loc_to_loc(struct T_LOCATION *loc_start, struct T_LOCATION *loc_end);

/*
 * 得到墨卡托投影的x坐标，也就是经度值
 */
double get_mercator_x(struct T_LOCATION *loc);

/*
 * 得到墨卡托投影的y坐标，也就是纬度值
 */
double get_mercator_y(struct T_LOCATION *loc);

/*
 * 用来判断是否到达指定的位置，其实可以有3种方式，
 * 最简单的是1判断当前位置current_loc是否在指定位置specific_loc的某个半径范围内
 * 2利用投影判断当前位置是否超过了当前目标航点和下一目标航点连线
 */
unsigned char arrive_specific_location_radius(struct T_LOCATION *current_loc, struct T_LOCATION *specific_loc,unsigned int arrive_radius);
unsigned char arrive_specific_location_over_line_project_NED(struct T_LOCATION *last_target_loc,struct T_LOCATION *current_loc,struct T_LOCATION *target_loc);

/*
 * 在北东地坐标系下，计算得到last_target_loc和target_loc这2个点连线所组成的向量与正北的夹角
 */
double get_bearing_point_2_point_NED(struct T_LOCATION *last_target_loc, struct T_LOCATION *target_loc);

/*
 * 输入三个点的经纬度，输出是中间的current的点到另外2点连线的垂直距离
 * 也就是输出偏航距，也就是船本身位置距离航线的垂直距离，单位是米
 */
double get_cross_track_error_NED(struct T_LOCATION *last_target_loc,struct T_LOCATION *current_loc,struct T_LOCATION *target_loc);

/*
 * 我们得到偏航距离这是知道了偏航程度
 * 如果要减小这个偏航距离，则需要把偏离的距离考虑进来从而调整航向，使得尽快靠近航线
 */
double get_cross_track_error_correct_radian_NED(struct T_LOCATION *last_target_loc,struct T_LOCATION *current_loc, struct T_LOCATION *target_loc);

/*
 * 比上面的加入了偏航距控制器的结构
 * 使用的是偏航距PID控制器
 */
double get_cross_track_error_correct_radian_NED_PID(struct T_LOCATION *last_target_loc,
		                                                                                                struct T_LOCATION *current_loc, struct T_LOCATION *target_loc,
		                                                                                                 void *pid_class_ptr);

#endif /* HEADERS_LOCATION_H_ */
