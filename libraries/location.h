/*
 * location.h
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#ifndef HEADERS_LOCATION_H_
#define HEADERS_LOCATION_H_

/*莫卡托坐标系的参考经度，在左边比较好，所以选择了东经110度，也就是说坐标系从东经110纬度0，为莫卡托坐标原点*/
#define REFERENCE_LONGITUDE_MERCATOR 110.0*M_PI/180.0

/*地球半径 单位：米[m]*/
#define EARTH_RADIUS 6378137

/*地球周长，莫卡托坐标系用的地球赤道周长*/
#define EARTH_CIRCUMFERENCE 20037508.34

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

struct T_LOCATION
{
	float lng;
	float lat;
	float alt;
};

struct T_NED
{
    float x;
    float y;
};

float convert_degree_to_radian(float degree);
float convert_radian_to_degree(float radian);

/*
wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
*/
float wrap_PI(float angle_in_radians);

/*
* wrap an angle in radians to 0..2PI
*/
float wrap_2PI(float angle);

float get_distance_loc_to_loc(struct T_LOCATION *loc_start, struct T_LOCATION *loc_end);
float get_mercator_x(struct T_LOCATION *loc);
float get_mercator_y(struct T_LOCATION *loc);

unsigned char arrive_specific_location_radius(struct T_LOCATION *current_loc, struct T_LOCATION *specific_loc,unsigned int arrive_radius);
unsigned char arrive_specific_location_over_line_project_NED(struct T_LOCATION *last_target_loc,struct T_LOCATION *current_loc,struct T_LOCATION *target_loc);

float get_bearing_point_2_point_NED(struct T_LOCATION *last_target_loc, struct T_LOCATION *target_loc);
float get_cross_track_error_NED(struct T_LOCATION *last_target_loc,struct T_LOCATION *current_loc,struct T_LOCATION *target_loc);
float get_cross_track_error_correct_radian_NED(struct T_LOCATION *last_target_loc,struct T_LOCATION *current_loc, struct T_LOCATION *target_loc);

#if 0
float get_bearing_loc_to_loc(struct T_LOCATION *loc_start, struct T_LOCATION *loc_end);
unsigned char arrive_specific_location_over_line(struct T_LOCATION *last_target,struct T_LOCATION *current_loc,struct T_LOCATION *target_loc);
unsigned char arrive_specific_location_over_line_project(struct T_LOCATION *last_target,struct T_LOCATION *current_loc,struct T_LOCATION *target_loc);

float get_cross_track_error_correct_radian(struct T_LOCATION *target, \
                                                  struct T_LOCATION *current, \
                                                  float command_radian);

float get_cross_track_error_correct_radian2(struct T_LOCATION *previous_target,struct T_LOCATION *target, struct T_LOCATION *current);
#endif

#endif /* HEADERS_LOCATION_H_ */
