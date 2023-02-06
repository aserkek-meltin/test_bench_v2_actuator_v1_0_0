/*
 * test_bench.cpp
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#include "test_bench.h"
#include "../system/utilities/loadcell_reader.h"

Test_Bench::Test_Bench(uint8_t _finger_id, uint8_t _test_bench_id, uint8_t _ita_id, uint8_t _jaa_id)
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


void Test_Bench::set_joint_torque(float _joint_torque_setpoint)
{
	_joint_torque_setpoint = joint_torque_setpoint;
}

bool Test_Bench::init_devices()
{
	return true;
}

void Test_Bench::read_sensor_values(uint8_t _finger_id, uint8_t _test_bench_id)
{
	float  result[2];
	read_sensors(_finger_id, _test_bench_id);

	its.raw = result[0];
	gfs.raw = result[1];

	//FIXME - MA is missing
}

uint8_t Test_Bench::get_ita_id()
{
	return ita_id;
}

uint8_t Test_Bench::get_jaa_id()
{
	return jaa_id;
}

float Test_Bench::ecs2mcs(float angle_ecs)
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

float Test_Bench::mcs2ecs(float angle_mcs)
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

void Test_Bench::load_default_ranges(Range_t _ita, Range_t _jaa_ecs)
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
