/*
 * utilities.h
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_UTILITIES_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_UTILITIES_H_

#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include "../../../system/utilities/data_types.h"

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

typedef struct
{
	bool GFS_COULDNT_INITIALIZED;		//1
	bool ITS_COULDNT_INITIALIZED;		//2
	bool I2C_MUX_COULDNT_INITIALIZED;	//3
	bool ITA_COMMUNICATION_ERROR;		//4
	bool JAA_COMMUNICATION_ERROR;		//5
	bool SWB_COMMUNICATION_ERROR;		//6
	bool FM_COMMUNICATION_ERROR;		//7
	bool RESERVED1;						//8
}Error_t;

typedef struct
{
	bool COMMAND_RECIEVED;				//1
	bool ITA_CALIBRATION_COMPLETED;		//2
	bool JAA_CALIBRATION_COMPLETED;		//3
	bool IT_PID_ON;						//4
	bool JA_PID_ON;						//5
	bool BENDING_FRICTION_COMPENSATION_ON;//6
	bool ACTUATOR_TORQUE;				//7
	bool BENDING_DIRECTION_BENDING;		// true: Bending, false: Release
}Status_t;

typedef struct
{
	bool TEST_BIT_1;					// IT PID use adaptive PID Coeff
	bool TEST_BIT_2;					// IT1 PID is on
	bool TEST_BIT_3;					// IT2 PID is on
	bool TEST_BIT_4;					// Bending Angle Manual override is on
	bool TEST_BIT_5;					//5
	bool TEST_BIT_6;					//6
	bool TEST_BIT_7;					//7
bool TEST_BIT_8;						//8
}Test_Bit_t;


typedef union
{
	Error_t	error_t;
	UINT8	error_u;
}Error_u;

typedef union
{
	Status_t	status_t;
	UINT8		status_u;
}Status_u;

typedef union
{
	Test_Bit_t	test_bit_t;
	UINT8		test_bit_u;
}Test_Bit_u;

#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_TEST_BENCH_UTILITIES_H_ */
