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

	dt=now_time-last_time;
	if (last_time == 0 || dt > 1000)
	{
        dt = 0;

        // if this PID hasn't been used for a full second then zero
        // the intergator term. This prevents I buildup from a
        // previous fight mode from causing a massive return before
        // the integrator gets a chance to correct itself
        reset_I();
    }
	last_time = now_time;

	delta_time=dt/1000.0;

	// Compute proportional component
	output += error * _kp;

	// Compute derivative component if time has elapsed
	if ((fabsf(_kd) > 0))
	{
		float derivative;

		if (isnan(_last_derivative))
		//if (1)
		{
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change
			derivative = 0;
			_last_derivative = 0;
		}
		else
		{
			derivative = (error - _last_error) / delta_time;
		}

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		float RC = 1 / (2 * M_PI*_fCut);
		derivative = _last_derivative +
			((delta_time / (RC + delta_time)) *
			(derivative - _last_derivative));
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
	// we use NAN (Not A Number) to indicate that the last
	// derivative value is not valid
	//_last_derivative = NAN;
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
