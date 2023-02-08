/*
 * test_bench.cpp
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

Joint::Joint(uint8_t _finger_id, uint8_t _test_bench_id, uint8_t _ita_id, uint8_t _jaa_id)
{
	finger_id 		= _finger_id;
	test_bench_id	= _test_bench_id;
	ita_id 			= _ita_id;
	jaa_id 			= _jaa_id;

	//Initialization of members
	its 					= {};
	gfs 					= {};
	ita 					= {};
	jaa_ecs 				= {};
	jaa_mcs 				= {};
	joint_torque_setpoint 	= 0;
	it1 					= 0;
	it2 					= 0;
	it1_setpoint 			= 0;
	it2_setpoint 			= 0;

	is_ita_sign_positive 	= true;
	is_jaa_sign_positive 	= true;

	jaa_zero 				= 0;
	ita_zero 				= 0;
}


void Joint::set_joint_torque(float _joint_torque_setpoint)
{
	_joint_torque_setpoint = joint_torque_setpoint;
}

bool Joint::init_devices()
{
	return true;
}

void Joint::read_sensor_values(uint8_t _finger_id, uint8_t _test_bench_id)
{
	float  result[2];
	read_sensors(_finger_id, _test_bench_id);

	its.raw = result[0];
	gfs.raw = result[1];

	//FIXME - MA is missing
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

void Joint::load_default_ranges(Range_t _ita, Range_t _jaa_ecs)
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
