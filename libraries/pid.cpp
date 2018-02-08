/*
 * pid.c
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#include <stdio.h>
#include <math.h>

#include "pid.h"
#include "utility.h"

#ifndef  M_PI
# define M_PI		3.14159265358979323846	/* pi */
# define M_PI_2		1.57079632679489661923	/* pi/2 */
#endif

BIT_PID::BIT_PID()
{

}
const float        BIT_PID::_filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";

/*
 * 这个函数还是有点问题，这个dt我现在是固定的为20ms
 */
float BIT_PID::get_pid(float error, float dt, float scaler)
{
	float output		= 0;
 	float delta_time	= (float)dt / 1000.0;

	// Compute proportional component
	output += error * _kp;

	// Compute derivative component if time has elapsed
	if ((fabs(_kd) > 0) && (dt > 0))
	{
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
	if ((fabs(_ki) > 0) && (dt > 0))
	{
		_integrator 		+= (error * _ki) * scaler * delta_time;
		if (_integrator < -_imax)
		{
			_integrator = -_imax;
		}
		else if (_integrator > _imax)
		{
			_integrator = _imax;
		}
		output 				+= _integrator;
	}

	return output;
}

void
BIT_PID::reset_I()
{
	_integrator = 0;
	_last_error = 0;
	_last_derivative = 0;
}

float BIT_PID::get_p(float error)
{
    return (float)error * _kp;
}

float BIT_PID::get_i(float error, float dt)
{
    if(dt != 0)
    {
        _integrator += ((float)error * _ki) * dt;

        if (_integrator < -_imax)
        {
            _integrator = -_imax;
        }
        else if (_integrator > _imax)
        {
            _integrator = _imax;
        }
    }

    return _integrator;
}

float BIT_PID:: get_d(float error, float dt)
{
	if ((_kd != 0) && (dt != 0))
	{
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

float BIT_PID::get_pi(float error, float dt)
{
    return get_p(error) + get_i(error, dt);
}



/***************************/
/*
 * 下面是为了论文中添加有限时间控制器
 */
/*
 * 这个函数还是有点问题，这个dt我现在是固定的为20ms
 */
float sig(float x, float alpha)
{
	if(x > 0)
		return pow(x, alpha);
	else
		return - pow(- x, alpha);
}


float BIT_PID::get_pid_finite(float error, float dt, float scaler)
{
	float output		= 0;
 	float delta_time	= (float)dt / 1000.0;

	// Compute proportional component
 	float alpha1 = 0.6;
 	float alpha2 = 0.75;

 	float error_finite_time = sig(error, alpha1);
	//output += error * _kp;
 	output += error_finite_time * _kp;

	// Compute derivative component if time has elapsed
	if ((fabs(_kd) > 0) && (dt > 0))
	{
		float derivative = (error - _last_error) / delta_time;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		float RC = 1/(2*M_PI*_fCut);
		derivative = _last_derivative +
							 (delta_time / (RC + delta_time)) * (derivative - _last_derivative);

		// update state
		_last_error 		= error;
		_last_derivative    = derivative;

		float derivative_finite_time = sig(derivative, alpha2);
		output 				+= _kd * derivative_finite_time;
		// add in derivative component
		//output 				+= _kd * derivative;
	}

	// scale the P and D components
	output *= scaler;

//	// Compute integral component if time has elapsed
//	if ((fabs(_ki) > 0) && (dt > 0))
//	{
//		_integrator 		+= (error * _ki) * scaler * delta_time;
//		if (_integrator < -_imax)
//		{
//			_integrator = -_imax;
//		}
//		else if (_integrator > _imax)
//		{
//			_integrator = _imax;
//		}
//		output 				+= _integrator;
//	}

	return output;
}
