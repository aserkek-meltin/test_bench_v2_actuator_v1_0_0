/*
 * test_bench.h
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_TEST_BENCH_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_TEST_BENCH_H_
#include "utilities.h"

class Test_Bench {

	//Each test bench controls a joint which is 2 wires.
	//There is;
	// 2 Loadcells 	named as 	ITS and GFS
	// 2 Dynamixels named as	ITA and JAA
public:
	Test_Bench(uint8_t _finger_id, uint8_t _test_bench_id, uint8_t _ita_id, uint8_t _jaa_id);

	////////////////////////
	// Parameters
	////////////////////////
	uint8_t	finger_id;						//Finger ID
	uint8_t	test_bench_id;					//Test Bench ID = Joint Angle ID

	////////////////////////
	// Functions
	////////////////////////
	bool 				init_devices();
	void				set_joint_torque(float _joint_torque_setpoint);
	uint8_t				get_ita_id();
	uint8_t				get_jaa_id();

	float				ecs2mcs(float ecs);
	float				mcs2ecs(float mcs);
	void 				load_default_ranges(Range_t ita, Range_t jaa_ecs);

private:
	uint8_t		ita_id;
	uint8_t		jaa_id;

	Actuator_t	ita;
	Actuator_t	jaa_ecs;
	Actuator_t	jaa_mcs;

	Data_t		its;
	Data_t		gfs;

	float 		joint_torque_setpoint;
	float		it1;
	float		it2;
	float		it1_setpoint;
	float		it2_setpoint;

	bool		is_ita_sign_positive;
	bool		is_jaa_sign_positive;
	float		jaa_zero;
	float		ita_zero;

	void 		read_sensor_values(uint8_t _finger_id, uint8_t _test_bench_id);
};

#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_TEST_BENCH_H_ */
