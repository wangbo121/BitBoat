/*
 * SIM_Watercraft.h
 *
 *  Created on: 2017-11-25
 *      Author: wangbo
 */

#ifndef SIM_WATERCRAFT_H_
#define SIM_WATERCRAFT_H_

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

	struct Location {
	    uint8_t id;                                                 ///< command id
	    uint8_t options;                                    ///< options bitmask (1<<0 = relative altitude)
	    uint8_t p1;                                                 ///< param 1
	    int32_t alt;                                        ///< param 2 - Altitude in centimeters (meters * 100)
	    int32_t lat;                                        ///< param 3 - Lattitude * 10**7
	    int32_t lng;                                        ///< param 4 - Longitude * 10**7
	};
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
protected:
    //Location home;
    Location location;

    float ground_level;
    float frame_height;
//    Matrix3f dcm;  // rotation matrix, APM conventions, from body to earth//20170818 这个dcm是cbn也就是从机体坐标系转到参考坐标系的矩阵
//    Vector3f gyro; // rad/s
//    Vector3f velocity_ef; // m/s, earth frame
//    Vector3f position; // meters, NED from origin
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
    const char *frame;

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

private:
    uint64_t last_time_us = 0;
    uint32_t frame_counter = 0;
    const uint32_t min_sleep_time;
};



#endif /* SIM_WATERCRAFT_H_ */
