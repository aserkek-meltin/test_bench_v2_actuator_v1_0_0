/*
 * joint.h
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_TEST_BENCH_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_TEST_BENCH_H_
#include "utilities.h"
#include <PID_v1.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DEFINES----------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define GFS_CORRECTION_FACTOR			1/0.6892
#define GFS_TO_IT_CALCULATION_FACTOR	21.7643		//0.15/0.01*GFS_CORRECTION_FACTOR

#define	IT1_DEFAULT_SAMPLE_TIME			 1
#define	IT1_DEFAULT_MIN_OUTPUT_LIMIT	 -7500
#define	IT1_DEFAULT_MAX_OUTPUT_LIMIT	 7500

#define	IT2_DEFAULT_SAMPLE_TIME			 1
#define	IT2_DEFAULT_MIN_OUTPUT_LIMIT	 -55
#define	IT2_DEFAULT_MAX_OUTPUT_LIMIT	 55


typedef struct
{
	uint8_t 	joint_id;
	uint8_t 	ita_id;
	uint8_t 	jaa_id;
}Joint_Settings_t;

class Joint {

	//Each test bench controls a joint which is 2 wires.
	//There is;
	// 2 Loadcells 	named as 	ITS and GFS
	// 2 Dynamixels named as	ITA and JAA
public:
	Joint(Joint_Settings_t joint_settings_t);

	////////////////////////
	// Parameters
	////////////////////////
	uint8_t	joint_id;					//Test Bench ID = Joint Angle ID

	////////////////////////
	// Functions
	////////////////////////
	float				ita_curr_command;			//Includes final angle command for the related Dynamixel Actuator
	float				jaa_mcs_curr_command;		//Includes final angle command for the related Dynamixel Actuator

	bool 				init_devices();
	void				set_joint_torque(float _joint_torque_setpoint);
	uint8_t				get_ita_id();
	uint8_t				get_jaa_id();

	float				ecs2mcs(float ecs);
	float				mcs2ecs(float mcs);
	void 				update_ranges(Range_t ita, Range_t jaa_ecs);

private:
	uint8_t				ita_id;
	uint8_t				jaa_id;

	Actuator_t			ita;
	Actuator_t			jaa_ecs;
	Actuator_t			jaa_mcs;

	Data_t				its;
	Data_t				gfs;


	PID_Settings_t		it1_pid_settings_t;
	PID_Settings_t		it2_pid_settings_t;
	PID					it1_pid;
	PID					it2_pid;


	float 				joint_torque_setpoint;
	float				it1_estimated;
	float				it2_estimated;
	float				it1_setpoint;
	float				it2_setpoint;

	bool				is_ita_sign_positive;
	bool				is_jaa_sign_positive;
	float				jaa_zero;
	float				ita_zero;

	void 				read_sensor_values(uint8_t _finger_id, uint8_t _joint_id);
	void				calculate_it1_it2_estimateds();
	void				joint_torques_2_internal_tensions_setpoints();
	void				it1_controller_initialization();
	void				it2_controller_initialization();
	void				it1_controller();
	void				it2_controller();
	void				ita_set_pos_command(float pos);
	void				jaa_set_ecs_angle_command(float ecs);
	float				ita_check_range(float ita_set);
	float				jaa_check_range_ecs(float angle_ecs);
	void				ita_calculate_output_safe();
	void				jaa_calculate_output_safe();
	float				it2_pid_to_ecs(float pid_output);

	void				loop();
};

#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_TEST_BENCH_H_ */
