/*
 * utilities.h
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_UTILITIES_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_UTILITIES_H_

#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

typedef struct{
	float raw;
	float ma;
	float converted;
	float lpf;
	float prev;
	float zero;
}Data_t;


typedef struct{
	float min;
	float center;
	float max;
} Range_t;

typedef struct{
	float Kp;
	float Ki;
	float Kd;
} PID_Coefficients_t;

typedef struct{
	double user_setpoint_SI;	//SI: International System of Units, meter, newtons, kg, sec etc
	double pid_setpoint;
	double input;
	double output;
	double Kp;
	double Ki;
	double Kd;
	double Kf;
	double Kp_base;
	double Ki_base;
	double Kd_base;
	bool   is_control_on;
} PID_Settings_t;

typedef struct{
	float 	current;
	float 	previous;
	float 	curr_command;
	float 	prev_command;
	float 	starting;
	Range_t	range;
}Actuator_t;


typedef struct{
	uint8_t	finger_id;
	uint8_t	test_bench_id;
	float	joint_torque_setpoint;
	float	it1;
	float	it2;
	float	it1_setpoint;
	float	it2_setpoint;
}Test_Bench_Info_t;

typedef struct{
	uint8_t	finger_id;
	uint8_t	test_bench_id;
	float	its;
	float	gfs;
}Test_Bench_Sensor_t;


#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_UTILITIES_H_ */
