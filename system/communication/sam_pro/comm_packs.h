/*
 * communication_packs.h
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#ifndef TEST_BENCH_ACTUATOR_COMMUNICATION_COMM_PACKS_H_
#define TEST_BENCH_ACTUATOR_COMMUNICATION_COMM_PACKS_H_


#include "comm_core/sam_pro.h"
#include "../../utilities/data_types.h"

typedef enum
{
    SAM_CMD_ACTUATORS_TORQUES_OFF = 0,
    SAM_CMD_ACTUATORS_TORQUES_ON,
    SAM_CMD_ACTUATORS_TO_CENTER,
    SAM_CMD_SEND_IT_PID_COEFF,
    SAM_CMD_SEND_JA_PID_COEFF,
    SAM_CMD_SEND_IT1_PID_COEFF,
    SAM_CMD_SEND_IT2_PID_COEFF,
    SAM_CMD_IT_PID_OFF,
    SAM_CMD_IT_PID_ON,
    SAM_CMD_JA_PID_OFF,
    SAM_CMD_JA_PID_ON,
    SAM_CMD_IT1_PID_OFF,
    SAM_CMD_IT1_PID_ON,
    SAM_CMD_IT2_PID_OFF,
    SAM_CMD_IT2_PID_ON,
    SAM_CMD_IT_ADAPTIVE_PID_COEFF_OFF,
    SAM_CMD_IT_ADAPTIVE_PID_COEFF_ON,
    SAM_CMD_ITA_CALIBRATE,
    SAM_CMD_BWBS_CALIBRATE,
    SAM_CMD_SENSORS_CALIBRATE,
    SAM_CMD_SEND_ITA_LIMITS,
    SAM_CMD_ITA_CALIBRATION_IS_TRUE,
    SAM_CMD_IT_PID_COFF_RESET,
    SAM_CMD_JA_PID_COFF_RESET,
    SAM_CMD_FTF_PID_COFF_RESET,
    SAM_CMD_SWB_TAKE_MEASUREMENT,
    SAM_CMD_BENDING_ANGLE_OVERRIDE_ON,
    SAM_CMD_BENDING_ANGLE_OVERRIDE_OFF,
    SAM_CMD_BENDING_ANGLE_OVERRIDE_BND,
    SAM_CMD_BENDING_ANGLE_OVERRIDE_RLS,
    SAM_CMD_BENDING_FRICTION_COMPENSATION_ON,
    SAM_CMD_BENDING_FRICTION_COMPENSATION_OFF,
	SAM_CMD_ACTUATORS_RESET,
	SAM_CMD_SEND_JA_PID_WFF_COEFF,
    SAM_CMD_JA_PID_WFF_ON,
    SAM_CMD_JA_PID_WFF_OFF,
}commands;

typedef enum
{
    SAM_STG_IT_SETPOINT = 0,
    SAM_STG_ITA_POS_UM,
    SAM_STG_JA_SETPOINT,
    SAM_STG_JAA_POS_UDEG,
    SAM_STG_IT_ZERO_OFFSET,
    SAM_STG_FTF_SETPOINT,
    SAM_STG_BENDING_ANGLE_MANUAL
}setting_ids;

typedef enum
{
	IT_PID,
	JA_PID,
	IT1_PID,
	IT2_PID,
	JA_PID_WFF
}pid_ids;

typedef enum
{
	COMMAND_PACK,
	SETTINGS_PACK,
	STATUS_PACK,
	PID_SETTINGS_PACK,
	PACK_GFS_LUT_SETTING,
	ITA_LIMITS_PACK,
	TEST_PACK
}pack_types;

typedef struct
{
	UINT8	command;					//Command IDA
}Sam_Command_Pack_t;

typedef struct
{
	UINT8	settings_id;				//Setting ID
	FLOAT32	data;						//Value for the setting
}Sam_Settings_Pack_t;


typedef struct
{
	UINT32	system_counter_s;			//Counter that increases every 1 sec

	UINT8	errors;						//Shows the errors in the system
	UINT8	status;						//Shows the status of the system components

	FLOAT32	ja;							//Joint Angle
	FLOAT32 ja_setpoint;				//Joint Angle PID Controller's setpoint

	FLOAT32 it;							//Measured raw data of the Internal Tension Loadcell
	FLOAT32 it_setpoint;				//Internal Tension PID Controller's setpoint (raw)

	FLOAT32 bwb;						//Bowden wire bending sensor's angle data
	FLOAT32 bending_estimation;			//Bending estimation

	FLOAT32 ftf;
	FLOAT32 ftf_setpoint;

	FLOAT32 test_value1_f32;
	FLOAT32 test_value2_f32;
	FLOAT32 test_value3_f32;
	FLOAT32 test_value4_f32;

	UINT8 test_value5_u8;
	UINT8 test_value6_u8;

}Sam_Status_Pack_t;

typedef struct
{
	UINT32	test_value1_u32;

	UINT8	test_bits1;
	UINT8	test_bits2;

	FLOAT32	test_value1_f32;
	FLOAT32 test_value2_f32;
	FLOAT32 test_value3_f32;
	FLOAT32 test_value4_f32;
	FLOAT32 test_value5_f32;
	FLOAT32 test_value6_f32;
	FLOAT32 test_value7_f32;
	FLOAT32 test_value8_f32;
	FLOAT32 test_value9_f32;
	FLOAT32 test_value10_f32;
	FLOAT32 test_value11_f32;
	FLOAT32 test_value12_f32;

	UINT8 test_value1_u8;
	UINT8 test_value2_u8;

}Sam_Test_Pack_t;

typedef struct
{
	UINT8	it_index;
	UINT8	ja_index;
	FLOAT32	new_value;
}Sam_LUT_Setting_Pack_t;

typedef struct
{
	UINT8    	pid_id;
	FLOAT32   	kp;
	FLOAT32   	ki;
	FLOAT32   	kd;
	FLOAT32   	kf;
}Sam_PID_Settings_Pack_t;

typedef struct
{
	FLOAT32   	ita_min;
	FLOAT32   	ita_max;
	FLOAT32		ita_current;
}Sam_ITA_Limits_Pack_t;

#endif /* TEST_BENCH_ACTUATOR_COMMUNICATION_COMM_PACKS_H_ */
