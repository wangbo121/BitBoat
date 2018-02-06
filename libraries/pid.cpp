/*
 * pid.c
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#include <math.h>
#include <stdio.h>

#include "pid.h"
#include "utility.h"

/*
 * 20180205目前先这样用，随后把PID单独改为一个类
 */

float _last_derivative = 0.0;
float _last_error = 0.0;
float _integrator = 0.0;

float _imax = 0.174*3;//这个是积分幅度的最大值，限制在10度，10/180*3.14=0.174
float last_time=0.0;
int _fCut=20;

#define SAMPLE_TIME 20
float get_pid(float error, float scaler,float _kp,float _ki,float _kd )
{
	float output = 0;
	float delta_time;
	float dt=0.0;
	float now_time=0.0;
	now_time=clock_gettime_ms();

	//delta_time = SAMPLE_TIME * 0.001;
	//delta_time = 0.200;

	dt=now_time-last_time;// 采样时间，或者是步长的时间间隔
	/*
	 * 如果前后2次计算pid的时间大于1000毫秒，那么就把积分清零
	 */
	if (last_time == 0 || dt > 1000)
	{
        dt = 0;
        reset_I();
    }
	last_time = now_time;

	delta_time=dt/1000.0;

	// 计算误差的比例部分
	output += error * _kp;

	// 计算误差的微分部分Compute derivative component if time has elapsed
	if ((fabsf(_kd) > 0))
	{
		float derivative;

		/*
		 * 如果微分信号过大，其实也就是如果有了高频噪声或者非常大的干扰
		 * 就把微分和积分都清零，只保留比例环节
		 * 这样可以避免方向舵的高频振荡
		 */
		if (isnan(_last_derivative))
		{
			derivative = 0;
			_last_derivative = 0;
		}
		else
		{
			derivative = (error - _last_error) / delta_time;
		}

		/*
		 * 低通滤波器的离散化形式
		 * discrete low pass filter, cuts out the
		 * high frequency noise that can drive the controller crazy
		 */
		float RC = 1 / (2 * M_PI*_fCut);
		derivative = _last_derivative + ((delta_time / (RC + delta_time)) * (derivative - _last_derivative));
		//printf("derivative=%f\n",derivative);

		// update state
		_last_error = error;
		_last_derivative = derivative;

		// add in derivative component
		output += _kd * derivative;
	}

	// scale the P and D components
	output *= scaler;

	// Compute integral component if time has elapsed
	if ((fabsf(_ki) > 0))
	{
		_integrator += (error * _ki) * scaler * delta_time;
		if (_integrator < -_imax)
		{
			_integrator = -_imax;
		}
		else if (_integrator > _imax)
		{
			_integrator = _imax;
		}
		output += _integrator;
	}
	//printf("integrator=%f\n",_integrator);

	return output;
}

void reset_I()
{
	_integrator = 0;
	_last_derivative = 0;
}

/***********************cross track error 的pid控制***************************/
float _last_derivative_cte = 0.0;
float _last_error_cte = 0.0;
float _integrator_cte = 0.0;

float _imax_cte = 0.174*3;//这个是积分幅度的最大值，限制在10度，10/180*3.14=0.174
float last_time_cte=0.0;
int _fCut_cte=20;

float get_pid_cte(float error, float scaler,float _kp,float _ki,float _kd )
{
    float output = 0;
    float delta_time;
    float dt=0.0;
    float now_time=0.0;
    now_time=clock_gettime_ms();

    dt=now_time-last_time_cte;
    if (last_time_cte == 0 || dt > 1000)
    {
        dt = 0;

        // if this PID hasn't been used for a full second then zero
        // the intergator term. This prevents I buildup from a
        // previous fight mode from causing a massive return before
        // the integrator gets a chance to correct itself
        reset_I_cte();
    }
    last_time_cte = now_time;

    delta_time=dt/1000.0;

    // Compute proportional component
    output += error * _kp;

    // Compute derivative component if time has elapsed
    if ((fabsf(_kd) > 0))
    {
        float derivative_cte;

        if (isnan(_last_derivative_cte))
        //if (1)
        {
            // we've just done a reset, suppress the first derivative
            // term as we don't want a sudden change in input to cause
            // a large D output change
            derivative_cte = 0;
            _last_derivative_cte = 0;
        }
        else
        {
            derivative_cte = (error - _last_error_cte) / delta_time;
        }

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        float RC = 1 / (2 * M_PI*_fCut);
        derivative_cte = _last_derivative_cte +
            ((delta_time / (RC + delta_time)) *
            (derivative_cte - _last_derivative_cte));
        //printf("derivative=%f\n",derivative);

        // update state
        _last_error_cte = error;
        _last_derivative_cte = derivative_cte;

        // add in derivative component
        output += _kd * derivative_cte;
    }

    // scale the P and D components
    output *= scaler;

    // Compute integral component if time has elapsed
    if ((fabsf(_ki) > 0))
    {
        _integrator_cte += (error * _ki) * scaler * delta_time;
        if (_integrator_cte < -_imax_cte)
        {
            _integrator_cte = -_imax_cte;
        }
        else if (_integrator_cte > _imax_cte)
        {
            _integrator_cte = _imax_cte;
        }
        output += _integrator_cte;
    }
    //printf("integrator_cte=%f\n",_integrator_cte);

    return output;
}

void reset_I_cte()
{
    _integrator_cte = 0;
    // we use NAN (Not A Number) to indicate that the last
    // derivative value is not valid
    //_last_derivative = NAN;
    _last_derivative_cte = 0;
}


/**********************************/
#ifndef  M_PI
# define M_PI		3.14159265358979323846	/* pi */
# define M_PI_2		1.57079632679489661923	/* pi/2 */
#endif

AP_PID::AP_PID()
{

}
const float        AP_PID::_filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";

float AP_PID::get_pid(float error, float dt, float scaler)
{
	float output		= 0;
 	float delta_time	= (float)dt / 1000.0;

	// Compute proportional component
	output += error * _kp;

	// Compute derivative component if time has elapsed
	if ((fabs(_kd) > 0) && (dt > 0)) {
		float derivative = (error - _last_error) / delta_time;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		float RC = 1/(2*M_PI*_fCut);
		derivative = _last_derivative +
		        (delta_time / (RC + delta_time)) * (derivative - _last_derivative);

		// update state
		_last_error 		= error;
		_last_derivative    = derivative;

		// add in derivative component
		output 				+= _kd * derivative;
	}

	// scale the P and D components
	output *= scaler;

	// Compute integral component if time has elapsed
	if ((fabs(_ki) > 0) && (dt > 0)) {
		_integrator 		+= (error * _ki) * scaler * delta_time;
		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
		output 				+= _integrator;
	}

	return output;
}

void
AP_PID::reset_I()
{
	_integrator = 0;
	_last_error = 0;
	_last_derivative = 0;
}

float AP_PID::get_p(float error)
{
    return (float)error * _kp;
}

float AP_PID::get_i(float error, float dt)
{
    if(dt != 0) {
        _integrator += ((float)error * _ki) * dt;

        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
    }
    return _integrator;
}

float        AP_PID:: get_d(float error, float dt)
{
	if ((_kd != 0) && (dt != 0)) {
		_derivative = (error - _last_error) / dt;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		_derivative = _last_derivative +
					  (dt / ( _filter + dt)) * (_derivative - _last_derivative);

		// update state
		_last_error            = error;
		_last_derivative    = _derivative;

		// add in derivative component
		return _kd * _derivative;
	}
	return 0;

}

float AP_PID::get_pi(float error, float dt)
{
    return get_p(error) + get_i(error, dt);
}



