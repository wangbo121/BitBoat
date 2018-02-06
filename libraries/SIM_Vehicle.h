/*
 * SIM_Vehicle.h
 *
 *  Created on: 2017-11-25
 *      Author: wangbo
 */

#ifndef SIM_VEHICLE_H_
#define SIM_VEHICLE_H_

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#ifndef RADIUS_EARTH
// 这个表示在赤道上 1*10^(-7)的角度对应的距离（单位：米）
// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

// 重力加速度 m/s/s
#define GRAVITY_MSS     9.80665f

// 地球半径
#define RADIUS_OF_EARTH 6378100

// 把经度或者纬度转化米或者厘米
#define LATLON_TO_M     0.01113195f
#define LATLON_TO_CM    1.113195f
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_2PI
#define M_2PI         (M_PI * 2)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)
#endif

struct Location
{
    uint8_t id;                                                 ///< command id
    uint8_t options;                                    ///< options bitmask (1<<0 = relative altitude)
    uint8_t p1;                                                 ///< param 1
    int32_t alt;                                        ///< param 2 - Altitude in centimeters (meters * 100)
    int32_t lat;                                        ///< param 3 - Lattitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
};

float constrain_value(const float amt, const float low, const float high);

float constrain_float(const float amt, const float low, const float high);

float longitude_scale(const struct Location &loc);

unsigned char is_zero(float x);

/*
 *  由距离loc的大小推算距离当前位置loc的经度和纬度
 */
void location_offset(struct Location &loc, float ofs_north, float ofs_east);
void location_update(struct Location &loc, float bearing, float distance);

/*
  parent class for all simulator types
 */
class Watercraft
{
public:
	Watercraft(const char *home_str, const char *frame_str);

	/*
	 * 这个结构是被这个simulator模拟器发送出去的数据结构
	 * 类似于真实情况下的反馈系统，包括所有航行器本身的状态
	 * 比如经度纬度高度速度姿态等
	 */
	struct  sitl_fdm
	{
	    uint64_t timestamp_us;
	    double latitude, longitude; // degrees
	    double altitude;  // MSL
	    double heading;   // degrees
	    double speedN, speedE, speedD; // m/s
	    double xAccel, yAccel, zAccel;       // m/s/s in body frame
	    double rollRate, pitchRate, yawRate; // degrees/s/s in body frame
	    double rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
	    double airspeed; // m/s
	    uint32_t magic; // 0x4c56414f
	};

    /*
     * 这个是模拟器simulator的输入部分，如果把模拟器当作一个黑匣子
     * 这几个电机就是输入，航行器的姿态就是输出
     */
    struct sitl_input
    {
        uint16_t servos[16];
    };

    /*
     * 差分方程，一步一步更新动力模型
     */
    void update(const struct sitl_input &input);

    /*
     * 把模拟器simulator计算得到的航行器的状态填入到fdm这个结构中去
     */
    void fill_fdm(struct sitl_fdm &fdm) const;

    /*
     * 从位移计算得到位置，位移是距离概念也就是向量，位置指的是经度和纬度
     */
    void update_position(void);

    /*
     * 增加噪声
     */
    //void add_noise(float throttle);

    Location home;
    Location location;

protected:
    float math_heading;//弧度radian
    float math_omega_z;//偏航角的角速度 rad/s

    float position_x;//由速度积分导致的朝北走的距离 因为我用的是北东地坐标系
    float position_y;//由速度积分导致的朝东走的距离
    float velocity_x;
    float velocity_y;

private:

};

#endif /* SIM_VEHICLE_H_ */
