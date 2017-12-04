/*
 * SIM_Watercraft.h
 *
 *  Created on: 2017-11-25
 *      Author: wangbo
 */

#ifndef SIM_WATERCRAFT_H_
#define SIM_WATERCRAFT_H_

#ifndef RADIUS_EARTH
// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH //这个表示1*10^(-7)角度对应的距离（单位：米）
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

// acceleration due to gravity in m/s/s
#define GRAVITY_MSS     9.80665f

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100

// convert a longitude or latitude point to meters or centimeteres.
// Note: this does not include the longitude scaling which is dependent upon location
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

/*
  class to describe a motor position
 */
class Motor {
public:
    float angle;
    bool clockwise;
    uint8_t servo;

    Motor(float _angle, bool _clockwise, uint8_t _servo) :
        angle(_angle), // angle in degrees from front
        clockwise(_clockwise), // clockwise == true, anti-clockwise == false
        servo(_servo) // what servo output drives this motor
    {}
};

/*
  class to describe a multicopter frame type
 */
class Frame {
public:
    const char *name;
    uint8_t num_motors;
    const Motor *motors;

    Frame(const char *_name,
          uint8_t _num_motors,
          const Motor *_motors) :
        name(_name),
        num_motors(_num_motors),
        motors(_motors) {
    	//printf("num_motors=%d  ##\n",num_motors);//20170818已测试
    	//std::cout<<"num_motors="<<num_motors<<"name="<<name<<std::endl;//uint_8不要用std打印，因为是unsigned char的所以会打印出符号来，而不是数字
    }
};

struct Location {
    uint8_t id;                                                 ///< command id
    uint8_t options;                                    ///< options bitmask (1<<0 = relative altitude)
    uint8_t p1;                                                 ///< param 1
    int32_t alt;                                        ///< param 2 - Altitude in centimeters (meters * 100)
    int32_t lat;                                        ///< param 3 - Lattitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
};

float constrain_value(const float amt, const float low, const float high)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_value(). The normal
    // float semantics already handle -Inf and +Inf
    if (isnan(amt)) {
        return (low + high) / 2;
    }

    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

float constrain_float(const float amt, const float low, const float high)
{
    return constrain_value(amt, low, high);
}

float longitude_scale(const struct Location &loc)
{
    float scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
    return constrain_float(scale, 0.01f, 1.0f);
}

unsigned char is_zero(float x)
{
	if(x==0.0)
	{
		return 1;
	}
	else
	{
		return 0;
	}

}

/*
 *  extrapolate latitude/longitude given distances north and east
 *   extrapolate : （由已知资料对未知事实或价值）推算，推断
 */
void location_offset(struct Location &loc, float ofs_north, float ofs_east)
{
    if (!is_zero(ofs_north) || !is_zero(ofs_east))
    {
        int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
        int32_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(loc);
        loc.lat += dlat;
        loc.lng += dlng;
    }
}

void location_update(struct Location &loc, float bearing, float distance)
{
    float ofs_north = cosf(DEG_TO_RAD * (bearing)) * distance;
    float ofs_east  = sinf(DEG_TO_RAD * (bearing)) * distance;
    location_offset(loc, ofs_north, ofs_east);
}




/*
  parent class for all simulator types
 */
class Watercraft
{
public:
	Watercraft(const char *home_str, const char *frame_str);

	struct  sitl_fdm {
	    // this is the packet sent by the simulator
	    // to the APM executable to update the simulator state
	    // All values are little-endian
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

//	struct Location {
//	    uint8_t id;                                                 ///< command id
//	    uint8_t options;                                    ///< options bitmask (1<<0 = relative altitude)
//	    uint8_t p1;                                                 ///< param 1
//	    int32_t alt;                                        ///< param 2 - Altitude in centimeters (meters * 100)
//	    int32_t lat;                                        ///< param 3 - Lattitude * 10**7
//	    int32_t lng;                                        ///< param 4 - Longitude * 10**7
//	};


    /*
      structure passed in giving servo positions as PWM values in
      microseconds
     */
    struct sitl_input {
        uint16_t servos[16];
        struct {
            float speed;      // m/s
            float direction;  // degrees 0..360
            float turbulence;
        } wind;
    };

    /*
      set simulation speedup
     */
    void set_speedup(float speedup);

    /*
      set instance number
     */
    void set_instance(uint8_t _instance) {
        instance = _instance;
    }

    /*
      set directory for additional files such as aircraft models
     */
    void set_autotest_dir(const char *_autotest_dir) {
        autotest_dir = _autotest_dir;
    }

    /*
      step the FDM by one time step
     */
    void update(const struct sitl_input &input);

    /* fill a sitl_fdm structure from the simulator state */
    void fill_fdm(struct sitl_fdm &fdm) const;
    //void fill_fdm_flightgear( T_FDM &fdm) const;

    Location home;
    Location location;
protected:
    //Location home;
    //Location location;

    float ground_level;
    float frame_height;
    float math_heading;
    float math_omega_z;//偏航角的角速度
//    Matrix3f dcm;  // rotation matrix, APM conventions, from body to earth//20170818 这个dcm是cbn也就是从机体坐标系转到参考坐标系的矩阵
//    Vector3f gyro; // rad/s
//    Vector3f velocity_ef; // m/s, earth frame
//    Vector3f position; // meters, NED from origin


    float position_x;//由速度积分导致的朝北走的距离 因为我用的是北东地坐标系
    float position_y;//由速度积分导致的朝东走的距离
    float velocity_x;
    float velocity_y;


    float mass; // kg
//    Vector3f accel_body; // m/s/s NED, body frame
    float airspeed; // m/s, apparent airspeed

    uint64_t time_now_us;

    const float gyro_noise;
    const float accel_noise;
    float rate_hz;
    float achieved_rate_hz;
    float target_speedup;
    uint64_t frame_time_us;
    float scaled_frame_time_us;
    uint64_t last_wall_time_us;
    uint8_t instance;
    const char *autotest_dir;
    //const char *frame;

    //20170817增加 机体坐标系下的加速度
//    Vector3f velocity_air_bf;

//    bool on_ground(const Vector3f &pos) const;

    /* update location from position */
    void update_position(void);

    /* rotate to the given yaw */
    void set_yaw_degrees(float yaw_degrees);

    /* advance time by deltat in seconds */
    void time_advance(float deltat);

    /* setup the frame step time */
    void setup_frame_time(float rate, float speedup);

    /* adjust frame_time calculation */
    void adjust_frame_time(float rate);

    /* try to synchronise simulation time with wall clock time, taking
       into account desired speedup */
    void sync_frame_time(void);

    /* add noise based on throttle level (from 0..1) */
    void add_noise(float throttle);

    /* return wall clock time in microseconds since 1970 */
    uint64_t get_wall_time_us(void) const;

    /* return normal distribution random numbers */
    double rand_normal(double mean, double stddev);

    void fill_fdm(struct sitl_fdm &fdm) const;

private:
    uint64_t last_time_us = 0;
    uint32_t frame_counter = 0;
    const uint32_t min_sleep_time;

    const Frame *frame;
};



#endif /* SIM_WATERCRAFT_H_ */
