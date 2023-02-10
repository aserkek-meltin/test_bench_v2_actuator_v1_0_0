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
#include "movingAvg.h"
#include <array>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DEFINES----------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define GFS_CORRECTION_FACTOR			1/0.6892
#define GFS_TO_IT_CALCULATION_FACTOR	21.7643		//0.15/0.01*GFS_CORRECTION_FACTOR
#define ITS_GRAM_TO_IT_N_FACTOR			0.00981 //(g[9.81]/1000)
#define GFS_GRAM_TO_FTF_N_FACTOR 		0.000654

#define	IT1_DEFAULT_SAMPLE_TIME			 1
#define	IT1_DEFAULT_MIN_OUTPUT_LIMIT	 -7500
#define	IT1_DEFAULT_MAX_OUTPUT_LIMIT	 7500

#define	IT2_DEFAULT_SAMPLE_TIME			 1
#define	IT2_DEFAULT_MIN_OUTPUT_LIMIT	 -55
#define	IT2_DEFAULT_MAX_OUTPUT_LIMIT	 55

#define ITS_MA_FILTER_WINDOW 			10
#define GFS_MA_FILTER_WINDOW 			10


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//STRUCTS----------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
	uint8_t 	joint_id;
	uint8_t 	ita_id;
	uint8_t 	jaa_id;
}Joint_Settings_t;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CLASS------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Joint {

	//Each test bench controls a joint which is 2 wires.
	//There is;
	// 2 Loadcells 	named as 	ITS and GFS
	// 2 Dynamixels named as	ITA and JAA
public:
	Joint(Joint_Settings_t joint_settings_t);

	//PUBLIC PARAMETERS-------------------------------------------------------------------------------------------------
	uint8_t	joint_id;					//Test Bench ID = Joint Angle ID
	float				ita_curr_command;			//Includes final angle command for the related Dynamixel Actuator
	float				jaa_mcs_curr_command;		//Includes final angle command for the related Dynamixel Actuator

	//PUBLIC FUNCTIONS--------------------------------------------------------------------------------------------------
	bool 				init_devices();
	void				set_joint_torque(float _joint_torque_setpoint);
	void 				update_it1_pid_coefficients(float Kp, float Ki, float Kd, float Kf);
	void  				update_it2_pid_coefficients(float Kp, float Ki, float Kd, float Kf);
	uint8_t				get_ita_id();
	uint8_t				get_jaa_id();
	void				get_dynamixel_positions(std::array<float, 2> positions);
	void 				update_ranges(Range_t ita, Range_t jaa_ecs);
	void 				update_sensor_data(float _its, float _gfs);
	void 				jaa_set_mcs_angle_readings(float mcs);
	void 				ita_set_angle_readings(float pos);
	float 				get_jaa_mcs_angle();
	float 				get_jaa_ecs_angle();
	float				get_ja_estimation();
	float				get_it1();
	float				get_it2();
	float				get_joint_torque_setpoint();
	float				get_it1_setpoint();
	float				get_it2_setpoint();
	float				get_ita_pos_mm();
	float 				get_ita_angle();
	float				get_jaa_min();
	float				get_jaa_max();
	float				get_ita_min();
	float				get_ita_max();
	std::array<int,4> 	get_it1_pidf_coeff();
	std::array<int,4> 	get_it2_pidf_coeff();
	bool				get_is_ita_calibrated();
	bool				get_is_ita_sign_positive();
	float				ecs2mcs(float ecs);
	float				mcs2ecs(float mcs);
	bool				is_controller_on();
	void  				ita_calibrate();
	void				loop();


private:
	//PRIVATE PARAMETERS------------------------------------------------------------------------------------------------
	float				its_cal_factor;
	float				gfs_cal_factor;
	uint8_t				ita_id;
	uint8_t				jaa_id;

	Actuator_t			ita;
	Actuator_t			jaa_ecs;
	Actuator_t			jaa_mcs;

	Data_t				its;
	Data_t				gfs;
	Data_t				ja;


	PID_Settings_t		it1_pid_settings_t;
	PID_Settings_t		it2_pid_settings_t;
	PID					it1_pid;
	PID					it2_pid;

	movingAvg			its_ma;
	movingAvg			gfs_ma;


	float 				joint_torque_setpoint;
	float				it1_estimated;
	float				it2_estimated;
	float				it1_setpoint;
	float				it2_setpoint;

	bool				is_ita_sign_positive;
	bool				is_jaa_sign_positive;
	bool				is_ita_calibrated;
	float				jaa_zero;
	float				ita_zero;


	//PRIVATE FUNCTIONS-------------------------------------------------------------------------------------------------
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

};

#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_TEST_BENCH_H_ */
