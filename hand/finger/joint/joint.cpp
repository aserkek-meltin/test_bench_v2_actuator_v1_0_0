/*
 * joint.cpp
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 *
 *      This class is written for a Test Bench.
 *      Each test_bench is basically responsible from a one joint of the hand.
 *      So, it is the basic element of the system
 *      It takes Required Joint Torque and Required Joint Angle. It tries its best to achive for those.
 *      Any compensation, control should be given as a input to Test Bench object.
 *      So, there should be outer, wider manager to decide joint torques etc.
 */

#include "joint.h"
#include "../../../system/utilities/loadcell_reader.h"
#include "../../../system/system_settings.h"
#include "../../../system/utilities/Global.h"

Joint::Joint(Joint_Settings_t joint_settings_t)
	:it1_pid(&it1_pid_settings_t.input,  &it1_pid_settings_t.output, &it1_pid_settings_t.pid_setpoint, it1_pid_settings_t.Kp, it1_pid_settings_t.Ki, it1_pid_settings_t.Kd, REVERSE)
	,it2_pid(&it2_pid_settings_t.input,  &it2_pid_settings_t.output, &it2_pid_settings_t.pid_setpoint, it2_pid_settings_t.Kp, it2_pid_settings_t.Ki, it2_pid_settings_t.Kd, REVERSE)
	,its_ma(ITS_MA_FILTER_WINDOW)
	,gfs_ma(GFS_MA_FILTER_WINDOW)
{
	joint_id		= joint_settings_t.joint_id;
	ita_id 			= joint_settings_t.ita_id;
	jaa_id 			= joint_settings_t.jaa_id;

	//Initialization of members
	its 					= {};
	gfs 					= {};
	ita 					= {};
	jaa_ecs 				= {};
	jaa_mcs 				= {};
	joint_torque_setpoint 	= 0;
	it1_estimated 			= 0;
	it2_estimated 			= 0;
	it1_setpoint 			= 0;
	it2_setpoint 			= 0;

	is_ita_sign_positive 	= true;
	is_jaa_sign_positive 	= true;

	jaa_zero 				= 0;
	ita_zero 				= 0;
	jaa_mcs_curr_command 	= ecs2mcs(DEFAULT_JAA_ECS_COMMAND);
	ita_curr_command		= DEFAULT_ITA_COMMAND;

	its_ma.begin();
	gfs_ma.begin();

	//Serial.println("Joint Constructor is called.");
	//Serial.print ("Joint :");		Serial.println(joint_id);
}

bool Joint::init_devices()
{
	Serial.println("OK");
	return true;
}

void Joint::loop()
{
	//convert joint torques to ITs
	joint_torques_2_internal_tensions_setpoints();
	//try to keep ITs
	it1_controller();
	it2_controller();
}

void Joint::set_joint_torque(float _joint_torque_setpoint)
{
	_joint_torque_setpoint = joint_torque_setpoint;
}

void Joint::update_sensor_data(float _its, float _gfs)
{
	its.raw = _its;
	gfs.raw = _gfs;

	its.ma = its_ma.reading(its.raw);
	gfs.ma = gfs_ma.reading(gfs.raw);

	its.converted = (its.ma - its.zero) * its_cal_factor * ITS_GRAM_TO_IT_N_FACTOR;
	gfs.converted = (gfs.ma - gfs.zero) * gfs_cal_factor * GFS_GRAM_TO_FTF_N_FACTOR * GFS_CORRECTION_FACTOR;
}

void Joint::calculate_it1_it2_estimateds()
{
	it2_estimated = (its.converted + (gfs.converted * GFS_TO_IT_CALCULATION_FACTOR) ) / -2;
	it1_estimated = (its.converted + it2_estimated) * -1;
}

uint8_t Joint::get_ita_id()
{
	return ita_id;
}

uint8_t Joint::get_jaa_id()
{
	return jaa_id;
}

float Joint::ecs2mcs(float angle_ecs)
{
	uint8_t sign = 0;
	if(is_ita_sign_positive)
	{
		sign = 1;
	}
	else
	{
		sign = -1;
	}
	return (sign * angle_ecs) + jaa_zero;
}

float Joint::mcs2ecs(float angle_mcs)
{
	uint8_t sign = 0;
	if(is_ita_sign_positive)
	{
		sign = 1;
	}
	else
	{
		sign = -1;
	}
	return (sign * angle_mcs) + jaa_zero;
}

void Joint::update_ranges(Range_t _ita, Range_t _jaa_ecs)
{
	ita.range.min 			= _ita.min;
	ita.range.center 		= _ita.center;
	ita.range.max 			= _ita.max;

	jaa_ecs.range.min 		= _jaa_ecs.min;
	jaa_ecs.range.center 	= _jaa_ecs.center;
	jaa_ecs.range.max 		= _jaa_ecs.max;

	jaa_mcs.range.min 		= ecs2mcs(jaa_ecs.range.min);
	jaa_mcs.range.center 	= ecs2mcs(jaa_ecs.range.center);
	jaa_mcs.range.max 		= ecs2mcs(jaa_ecs.range.max);
}

void Joint::joint_torques_2_internal_tensions_setpoints()
{
	it1_setpoint = 0;
	it2_setpoint = joint_torque_setpoint / JOINT_PULLEY_RADIUS_M;
}

void Joint::it1_controller_initialization()	//Controls ITA
{
	it1_pid_settings_t.Kp 			= 2;
	it1_pid_settings_t.Ki 			= 20;
	it1_pid_settings_t.Kd 			= 0;
	it1_pid_settings_t.Kp_base 		= 2;
	it1_pid_settings_t.Ki_base 		= 20;
	it1_pid_settings_t.Kd_base 		= 0;
	it1_pid_settings_t.pid_setpoint = 0;
	it1_pid_settings_t.output 		= ita.range.min;

	it1_pid.SetMode(AUTOMATIC);
	it1_pid.SetOutputLimits(IT1_DEFAULT_MIN_OUTPUT_LIMIT, 	IT1_DEFAULT_MAX_OUTPUT_LIMIT);
	it1_pid.SetSampleTime(IT1_DEFAULT_SAMPLE_TIME);
	it1_pid.SetAntiwindup(true,1);
	it1_pid.SetTunings(it1_pid_settings_t.Kp, it1_pid_settings_t.Ki, it1_pid_settings_t.Kd);
}

void Joint::it2_controller_initialization()	//Controls JAA
{
	it2_pid_settings_t.Kp 			= 0.04;
	it2_pid_settings_t.Ki 			= 0.4;
	it2_pid_settings_t.Kd 			= 0;
	it2_pid_settings_t.Kp_base 		= 0.12;
	it2_pid_settings_t.Ki_base 		= 3;
	it2_pid_settings_t.Kd_base 		= 0;
	it2_pid_settings_t.pid_setpoint = 0;
	it2_pid_settings_t.output 		= jaa_mcs.range.center;

	it2_pid.SetMode(AUTOMATIC);
	it2_pid.SetOutputLimits(IT2_DEFAULT_MIN_OUTPUT_LIMIT, 	IT2_DEFAULT_MAX_OUTPUT_LIMIT);
	it2_pid.SetSampleTime(IT2_DEFAULT_SAMPLE_TIME);
	it2_pid.SetAntiwindup(true,1);
	it2_pid.SetTunings(it2_pid_settings_t.Kp, it2_pid_settings_t.Ki, it2_pid_settings_t.Kd);
}

void Joint::it1_controller()
{
	//TODO - URGENT - Write the IT1 Controller
	it1_pid_settings_t.pid_setpoint = it1_setpoint;
	calculate_it1_it2_estimateds();

	it1_pid_settings_t.input = it1_estimated;
	//it2_settings_t.input = it2_estimated;

	if(it1_pid_settings_t.is_control_on)
	{
		it1_pid.Compute();
	}
	ita_calculate_output_safe();
	ita_curr_command		= ita.curr_command;
}

void Joint::it2_controller()
{
	//TODO - URGENT - Write the IT2 Controller
	it2_pid_settings_t.pid_setpoint = it2_setpoint;
	calculate_it1_it2_estimateds();

	it2_pid_settings_t.input = it2_estimated;

	if(it2_pid_settings_t.is_control_on)
	{
		it2_pid.Compute();
	}

	jaa_calculate_output_safe();
	jaa_mcs_curr_command 	= jaa_mcs.curr_command;
}


// CHECK RANGES/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float Joint::ita_check_range(float ita_set)
{
	if(ita.range.min > ita.range.max)
	{
		if(ita_set > ita.range.min)
		{
			return ita.range.min;
		}
		else if(ita_set < ita.range.max)
		{
			return ita.range.max;
		}
		else
		{
			return ita_set;
		}
	}
	else
	{
		if(ita_set < ita.range.min)
		{
			return ita.range.min;
		}
		else if(ita_set > ita.range.max)
		{
			return ita.range.max;
		}
		else
		{
			return ita_set;
		}
	}
}

float Joint::jaa_check_range_ecs(float angle_ecs)
{
	angle_ecs = wrap360(angle_ecs);

	if(angle_ecs > jaa_ecs.range.max && angle_ecs <= 147.5)
	{
		return jaa_ecs.range.max;
	}
	else if (angle_ecs > 147.5 && angle_ecs < jaa_ecs.range.min)
	{
		return jaa_ecs.range.min;
	}
	else
	{
		return angle_ecs;
	}
}

// SET ANGLES SAFE /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Joint::jaa_set_ecs_angle_command(float ecs)
{
	jaa_ecs.prev_command = jaa_ecs.curr_command;
	jaa_ecs.curr_command	= jaa_check_range_ecs(ecs);

	jaa_mcs.curr_command = ecs2mcs(jaa_ecs.curr_command);
	jaa_mcs.prev_command = ecs2mcs(jaa_ecs.prev_command);
}

void Joint::ita_set_pos_command(float pos)
{
	ita.prev_command = ita.curr_command;
	ita.curr_command = ita_check_range(pos);
}

// PID OUT TO ACTUATOR COMMAND WITH RANGE CHECK ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Joint::ita_calculate_output_safe()
{
	float value = it1_pid_settings_t.output;

	if(value > IT1_DEFAULT_MAX_OUTPUT_LIMIT)
	{
		value = IT1_DEFAULT_MAX_OUTPUT_LIMIT;
	}
	else if(value < IT1_DEFAULT_MIN_OUTPUT_LIMIT)
	{
		value = IT1_DEFAULT_MIN_OUTPUT_LIMIT;
	}

	float mapped_output = map(value, IT1_DEFAULT_MIN_OUTPUT_LIMIT, IT1_DEFAULT_MAX_OUTPUT_LIMIT, ita.range.min, ita.range.max);
	if(it1_pid_settings_t.is_control_on)
	{
		ita_set_pos_command(mapped_output);
	}
}

void Joint::jaa_calculate_output_safe()
{
	float value = it2_pid_settings_t.output;
	if(value > IT2_DEFAULT_MAX_OUTPUT_LIMIT)
	{
		value = IT2_DEFAULT_MAX_OUTPUT_LIMIT;
	}
	else if(value < IT2_DEFAULT_MIN_OUTPUT_LIMIT)
	{
		value = IT2_DEFAULT_MIN_OUTPUT_LIMIT;
	}

	float mapped_out = it2_pid_to_ecs(value);

	if(it2_pid_settings_t.is_control_on)
	{
		jaa_set_ecs_angle_command(mapped_out);
	}
}

float Joint::it2_pid_to_ecs(float pid_output)
{
	return (wrap360(pid_output + 325));
}

